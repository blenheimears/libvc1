if(NOT DEFINED SOURCE)
  message(FATAL_ERROR "SOURCE not supplied")
endif()
file(READ "${SOURCE}" CORE)
foreach(NEEDED
    "struct PendingBoundedRetry"
    "retry_future=std::async(std::launch::async"
    "history_future=std::async(std::launch::async"
    "compute_bounded_retry"
    "compute_private_history_recovery"
    "try_private_history_candidate"
    "run_wave"
    "Phase::PrivateHistoryRecovery"
    "release_bounded_recovery_reservoir()"
    "if (bounded_retry_pending) return;"
    "kBoundedRecoveryHistoryGops=2"
    "kBoundedOutputReservoirGops=4"
    "kRecoveryBufferedInputGops=3")
  string(FIND "${CORE}" "${NEEDED}" POS)
  if(POS EQUAL -1)
    message(FATAL_ERROR "missing bounded private-recovery marker: ${NEEDED}")
  endif()
endforeach()

# Ordered commit itself may only enqueue correction; it must never synchronously
# perform a full-GOP encode or mutate previously exposed output.
string(FIND "${CORE}" "void process_completed_probe(EncodedGop probe)" BEGIN)
string(FIND "${CORE}" "void harvest_workers()" END)
if(BEGIN EQUAL -1 OR END EQUAL -1 OR END LESS BEGIN)
  message(FATAL_ERROR "cannot locate ordered predictor commit function")
endif()
math(EXPR LEN "${END}-${BEGIN}")
string(SUBSTRING "${CORE}" ${BEGIN} ${LEN} COMMIT)
string(FIND "${COMMIT}" "start_bounded_retry(std::move(probe))" START_POS)
string(FIND "${COMMIT}" "encode_gop_once(" DIRECT_POS)
if(START_POS EQUAL -1)
  message(FATAL_ERROR "ordered predictor commit does not enqueue asynchronous correction")
endif()
if(NOT DIRECT_POS EQUAL -1)
  message(FATAL_ERROR "ordered predictor commit still performs a synchronous full-GOP encode")
endif()

# 0.1.38 used a fixed post-GOP fullness guard. It could reject every candidate,
# including Q31, on a legal sustained-hard sequence. 0.1.39 guarantees recovery
# structurally by retaining only still-private predecessor GOPs instead.
string(FIND "${CORE}" "bounded_recovery_guard_bits" GUARD_POS)
if(NOT GUARD_POS EQUAL -1)
  message(FATAL_ERROR "obsolete fixed GOP-boundary recovery guard remains")
endif()

message(STATUS "bounded private-history recovery/output-reservoir checks ok")
