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
  message(FATAL_ERROR "debug statistics public API version marker missing")
endif()
if(CMAKE_MATCH_1 LESS 30)
  message(FATAL_ERROR "macroblock statistics requires public API version >= 30; found ${CMAKE_MATCH_1}")
endif()
foreach(needle
    "int b_debug_stats;"
    "int b_debug_macroblock_stats;"
    "vc1_mb_debug_t"
    "p_debug_macroblocks"
    "i_debug_gop_index"
    "f_debug_qscale"
    "i_debug_acpred_macroblocks"
    "i_debug_four_mv_macroblocks"
    "i_debug_intra_macroblocks"
    "i_debug_dquant_macroblocks"
    "f_debug_mquant_mean"
    "i_debug_encode_trials")
  string(FIND "${H}" "${needle}" pos)
  if(pos EQUAL -1)
    message(FATAL_ERROR "debug-statistics public API marker missing: ${needle}")
  endif()
endforeach()
foreach(needle
    "e->base.debug_stats=p.b_debug_stats!=0"
    "e->base.debug_macroblock_stats=p.b_debug_macroblock_stats!=0"
    "e->base.keep_reconstruction=p.b_recon || p.b_transform_info || p.b_debug_stats || p.b_debug_macroblock_stats"
    "if (param.b_debug_stats)"
    "picture.debug_acpred_macroblocks"
    "picture.debug_encode_trials=static_cast<int>(cache.size())")
  string(FIND "${C}" "${needle}" pos)
  if(pos EQUAL -1)
    message(FATAL_ERROR "debug-statistics encoder marker missing: ${needle}")
  endif()
endforeach()
foreach(needle
    "--debug-stats"
    "--macroblock-stats"
    "previous_mse_y"
    "prediction_mse_y"
    "fwd_ref_display"
    "psnr_yuv_db"
    "snr_yuv_db"
    "debug statistics did not receive exactly one row per input frame")
  string(FIND "${F}" "${needle}" pos)
  if(pos EQUAL -1)
    message(FATAL_ERROR "debug-statistics frontend marker missing: ${needle}")
  endif()
endforeach()
message(STATUS "debug statistics API/frontend checks passed")

# API v13 picture-level transform diagnostics.
foreach(needle "int b_debug_ttmbf;" "int i_debug_ttfrm;" "int b_debug_ttfrm_exact_checked;")
  string(FIND "${H}" "${needle}" POS)
  if(POS EQUAL -1)
    message(FATAL_ERROR "missing transform debug field: ${needle}")
  endif()
endforeach()
