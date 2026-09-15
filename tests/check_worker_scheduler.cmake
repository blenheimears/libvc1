if(NOT DEFINED SOURCE)
  message(FATAL_ERROR "SOURCE not supplied")
endif()
file(READ "${SOURCE}" CORE)
foreach(NEEDED
    "std::map<uint64_t,EncodedGop> completed"
    "void harvest_workers()"
    "wait_for(std::chrono::seconds(0))"
    "completed.emplace(index,std::move(g))"
    "e->harvest_workers();"
    "finished future no longer occupies one of the worker slots"
    "outer GOP workers are the steady-state throughput path"
    "return static_cast<size_t>(std::max(1,param.i_threads));")
  string(FIND "${CORE}" "${NEEDED}" POS)
  if(POS EQUAL -1)
    message(FATAL_ERROR "missing worker-scheduler regression marker: ${NEEDED}")
  endif()
endforeach()

# 0.1.63: do not statically divide the outer GOP pool by the maximum nested
# helper count. Helpers are short-lived; reserving their slots for the entire
# GOP caused periodic 100%-CPU bursts followed by sustained underutilization.
string(FIND "${CORE}" "(param.i_threads+per_gop-1)/per_gop" STATIC_RESERVATION_POS)
if(NOT STATIC_RESERVATION_POS EQUAL -1)
  message(FATAL_ERROR "outer GOP pool again statically reserves intra-GOP helper slots")
endif()

# The old starvation pattern harvested only when the public output queue was
# empty.  It is okay to retain the blocking saturation check after the new
# unconditional harvest, but the old one-line form must not return.
string(FIND "${CORE}" "if (e->ready.empty() && e->pending.size()>=static_cast<size_t>(e->param.i_threads)) e->drain_one();" BLOCK_POS)
string(FIND "${CORE}" "e->harvest_workers();\n            if (e->ready.empty() && e->pending.size()>=static_cast<size_t>(e->param.i_threads)) e->drain_one();" HARVEST_POS)
if(NOT BLOCK_POS EQUAL -1 AND HARVEST_POS EQUAL -1)
  message(FATAL_ERROR "worker saturation check is not preceded by unconditional future harvesting")
endif()

# EOF/flush must harvest unconditionally too.  The 0.1.35 input-path fix did
# not cover this branch; once the reader hit EOF, buffered output could again
# prevent completed futures from being harvested and worker slots from being
# refilled.
string(FIND "${CORE}" "e->pump_workers();\n            // EOF does not mean the GOP workers are finished.  Keep harvesting\n            // completed futures even while older pictures are buffered in\n            // `ready`" EOF_COMMENT_POS)
string(FIND "${CORE}" "e->pump_workers();\n            // EOF does not mean the GOP workers are finished.  Keep harvesting\n            // completed futures even while older pictures are buffered in\n            // `ready`; otherwise the flush loop can spend dozens of calls\n            // emitting buffered AUs while completed futures occupy every worker\n            // slot and no queued GOP can start.  That reproduces the same\n            // one-core collapse fixed for the input path in 0.1.35.\n            e->harvest_workers();\n            while (e->ready.empty())" EOF_HARVEST_POS)
if(EOF_COMMENT_POS EQUAL -1 OR EOF_HARVEST_POS EQUAL -1)
  message(FATAL_ERROR "EOF/flush path does not unconditionally harvest completed GOP workers")
endif()

message(STATUS "worker scheduler harvesting/full-outer-pool checks ok")
