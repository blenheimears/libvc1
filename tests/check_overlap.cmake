if(NOT DEFINED SRC)
  message(FATAL_ERROR "SRC is required")
endif()
file(READ "${SRC}/include/libvc1.h" H)
file(READ "${SRC}/src/encoder_internal.h" I)
file(READ "${SRC}/src/encoder_core.cpp" C)
file(READ "${SRC}/src/encoder_api_helpers.cpp" A)
file(READ "${SRC}/src/encoder_bitstream.cpp" B)
file(READ "${SRC}/src/encoder_filter.cpp" F)
file(READ "${SRC}/src/encoder_encode.cpp" E)
file(READ "${SRC}/src/frontend.cpp" CLI)
string(REGEX MATCH "#define[ \t]+LIBVC1_API_VERSION[ \t]+([0-9]+)" API_VERSION_MATCH "${H}")
if(NOT API_VERSION_MATCH)
  message(FATAL_ERROR "OVERLAP public API version marker missing")
endif()
if(CMAKE_MATCH_1 LESS 16)
  message(FATAL_ERROR "OVERLAP requires public API version >= 16; found ${CMAKE_MATCH_1}")
endif()
foreach(needle
    "int b_overlap;"
    "bool overlap = true"
    "p->b_overlap=1"
    "e->cfg.overlap=p.b_overlap!=0"
    "b.bit(c_.overlap)"
    "enum class OverlapMode"
    "OverlapMode::Select"
    "choose_i_overlap_plan"
    "apply_i_overlap"
    "apply_p_overlap"
    "overlap_vertical_edge"
    "overlap_horizontal_edge"
    "--overlap|--no-overlap")
  string(FIND "${H}\n${I}\n${C}\n${A}\n${B}\n${F}\n${E}\n${CLI}" "${needle}" at)
  if(at EQUAL -1)
    message(FATAL_ERROR "OVERLAP/CONDOVER implementation marker missing: ${needle}")
  endif()
endforeach()
foreach(needle
    "write_decode012(b,static_cast<int>(overlap_plan.mode))"
    "b.append(overlap_plan.bitplane.syntax)"
    "b.bit(overlap_plan.flags[mbpos]!=0)"
    "c_.overlap && c_.pqindex>=9"
    "c_.overlap && c_.pqindex<=8")
  string(FIND "${E}" "${needle}" at)
  if(at EQUAL -1)
    message(FATAL_ERROR "OVERLAP picture syntax/path marker missing: ${needle}")
  endif()
endforeach()
message(STATUS "OVERLAP/CONDOVER structural regression: PASS")
