if(NOT DEFINED ENCODER)
  message(FATAL_ERROR "ENCODER is required")
endif()
foreach(opt IN ITEMS --old-rc --predictor-rc --i-share --p-share --b-share)
  execute_process(
    COMMAND "${ENCODER}" "${opt}"
    RESULT_VARIABLE rc
    OUTPUT_QUIET
    ERROR_VARIABLE err)
  if(rc EQUAL 0)
    message(FATAL_ERROR "removed rate-control option ${opt} was unexpectedly accepted")
  endif()
  string(FIND "${err}" "unknown option: ${opt}" pos)
  if(pos EQUAL -1)
    message(FATAL_ERROR "removed option ${opt} did not fail as an unknown option: ${err}")
  endif()
endforeach()
