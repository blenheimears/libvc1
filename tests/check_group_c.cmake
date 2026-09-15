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
  message(FATAL_ERROR "Group-C public API version marker missing")
endif()
if(CMAKE_MATCH_1 LESS 11)
  message(FATAL_ERROR "Group-C requires public API version >= 11; found ${CMAKE_MATCH_1}")
endif()
foreach(needle
    "int b_dquant;"
    "i_debug_dquant_macroblocks"
    "i_debug_mquant_min"
    "i_debug_mquant_max"
    "f_debug_mquant_mean"
    "i_dquant_macroblocks")
  string(FIND "${H}" "${needle}" pos)
  if(pos EQUAL -1)
    message(FATAL_ERROR "Group-C public API marker missing: ${needle}")
  endif()
endforeach()
foreach(needle
    "b.bits(c_.dquant ? 1 : 0,2)"
    "enum class DQuantProfile"
    "FourEdges=0"
    "DoubleEdges=1"
    "SingleEdge=2"
    "AllMbs=3"
    "make_dquant_plan("
    "write_dquant_header("
    "write_dquant_mb("
    "dquant_mb_derived("
    "choose_dquant_transform_mb("
    "write_mqdiff("
    "record_mquant("
    "predict_dc_dquant("
    "scale_ac_predictor("
    "if (best.base && best.bits < esc3_bits)"
    "bool dquantfrm=false"
    "e->cfg.dquant=(p.b_dquant!=0)")
  string(FIND "${C}" "${needle}" pos)
  if(pos EQUAL -1)
    message(FATAL_ERROR "Group-C implementation marker missing: ${needle}")
  endif()
endforeach()
# The old implementation could only signal ALL_MBS/non-bilevel and hard-gated
# public DQUANT to Advanced Profile.  Those restrictions must not return.
foreach(stale
    "DQPROFILE=ALL_MBS"
    "DQBILEVEL=0"
    "e->cfg.dquant=(p.b_dquant!=0 && p.i_profile==VC1_PROFILE_ADVANCED)")
  string(FIND "${C}" "${stale}" pos)
  if(NOT pos EQUAL -1)
    message(FATAL_ERROR "stale partial-DQUANT restriction remains: ${stale}")
  endif()
endforeach()
string(FIND "${C}" "!(c_.syntax==StreamSyntax::Advanced && c_.dquant) && best.base" bad_escape_gate)
if(NOT bad_escape_gate EQUAL -1)
  message(FATAL_ERROR "Group-C must not globally disable VC-1 Escape Modes 1/2")
endif()
foreach(needle "--dquant" "--no-dquant" "dquant_mb" "mquant_min" "DQUANT-MB=")
  string(FIND "${F}" "${needle}" pos)
  if(pos EQUAL -1)
    message(FATAL_ERROR "Group-C frontend/statistics marker missing: ${needle}")
  endif()
endforeach()
message(STATUS "Group-C complete DQUANT API/syntax/frontend markers passed")
