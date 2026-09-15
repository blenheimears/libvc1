if(NOT DEFINED CORE)
  message(FATAL_ERROR "CORE is required")
endif()
file(READ "${CORE}" S)
get_filename_component(_encdir "${CORE}" DIRECTORY)
file(GLOB _encparts "${_encdir}/encoder_*.cpp" "${_encdir}/encoder_internal.h")
foreach(_part IN LISTS _encparts)
  file(READ "${_part}" _part_text)
  string(APPEND S "\n${_part_text}")
endforeach()
set(required
  "enum class TransformSignalLevel"
  "struct TransformPicturePlan"
  "choose_p_transform_plan"
  "choose_b_transform_plan"
  "write_ttmb_macro"
  "write_ttmb_block"
  "write_ttblk"
  "refresh_transform_estimate"
  "TransformSignalLevel::Block"
  "TransformSignalLevel::Frame"
  "b.bit(transform_plan.frame_level)"
  "TTMBF"
  "TTFRM"
  "TTBLK"
)
foreach(marker IN LISTS required)
  string(FIND "${S}" "${marker}" pos)
  if(pos EQUAL -1)
    message(FATAL_ERROR "progressive-transform RDO marker missing: ${marker}")
  endif()
endforeach()
string(FIND "${S}" "if (c_.variable_transforms) b.bit(false);" stale)
if(NOT stale EQUAL -1)
  message(FATAL_ERROR "stale fixed TTMBF=0 picture-header path remains")
endif()
message(STATUS "progressive transform TTMBF/TTFRM/TTBLK structural checks ok")
