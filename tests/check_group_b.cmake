file(READ "${HEADER}" H)
file(READ "${CORE}" C)
get_filename_component(_encdir "${CORE}" DIRECTORY)
file(GLOB _encparts "${_encdir}/encoder_*.cpp" "${_encdir}/encoder_internal.h")
foreach(_part IN LISTS _encparts)
  file(READ "${_part}" _part_text)
  string(APPEND C "\n${_part_text}")
endforeach()
file(READ "${FRONTEND}" F)
string(REGEX MATCH "#define[ \t]+LIBVC1_API_VERSION[ \t]+([0-9]+)" API_VERSION_MATCH "${H}")
if(NOT API_VERSION_MATCH)
  message(FATAL_ERROR "Group-B public API version marker missing")
endif()
if(CMAKE_MATCH_1 LESS 10)
  message(FATAL_ERROR "Group-B requires public API version >= 10; found ${CMAKE_MATCH_1}")
endif()
foreach(needle
    "i_debug_four_mv_macroblocks"
    "i_debug_intra_macroblocks"
    "i_four_mv_macroblocks"
    "i_intra_macroblocks")
  string(FIND "${H}" "${needle}" pos)
  if(pos EQUAL -1)
    message(FATAL_ERROR "Group-B public API marker missing: ${needle}")
  endif()
endforeach()
foreach(needle
    "refine_qpel_block("
    "predictor_info_mixed_1mv("
    "predictor_info_4mv("
    "future_anchor_4mv"
    "implicit_penalty"
    "P-intra decision"
    "c_.syntax==StreamSyntax::Advanced"
    "process_mb_v(mx,my);"
    "process_mb_h(mx-1,my);")
  string(FIND "${C}" "${needle}" pos)
  if(pos EQUAL -1)
    message(FATAL_ERROR "Group-B implementation marker missing: ${needle}")
  endif()
endforeach()
foreach(needle "p_four_mv_mb" "p_intra_mb" "P-4MV=" "P-intra=")
  string(FIND "${F}" "${needle}" pos)
  if(pos EQUAL -1)
    message(FATAL_ERROR "Group-B frontend/statistics marker missing: ${needle}")
  endif()
endforeach()
message(STATUS "Group-B qpel/Mixed-MV/P-intra/B-RDO markers passed")

# B-picture forward/backward analysis requests only 1-MV fields.  The 4-MV
# sub-block search must be gated so it is not computed and discarded for every B.
string(FIND "${C}" "if (allow_4mv) {" p_4mv_gate)
if(p_4mv_gate EQUAL -1)
  message(FATAL_ERROR "Group-B 4-MV search is not gated by allow_4mv")
endif()


# 0.1.64 performance regressions: the B-only 1-MV calls must remain ahead of
# a real allow_4mv gate, qpel refinement must accept a known seed SAD, and B
# mode RDO must keep the fused four-candidate sampling pass.
foreach(needle
    "analyze_p_picture_core(f,*past_prediction,mode,false,nullptr)"
    "analyze_p_picture_core(f,future,mode,false,nullptr)"
    "known_seed_sad"
    "six luma interpolation calls/pixel to four"
    "mv decomposition and interpolation mode are constant")
  string(FIND "${C}" "${needle}" p_perf)
  if(p_perf EQUAL -1)
    message(FATAL_ERROR "Group-B performance marker missing: ${needle}")
  endif()
endforeach()
