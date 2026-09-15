if(NOT DEFINED CORE)
  message(FATAL_ERROR "CORE not supplied")
endif()
file(READ "${CORE}" C)
get_filename_component(_encdir "${CORE}" DIRECTORY)
file(GLOB _encparts "${_encdir}/encoder_*.cpp" "${_encdir}/encoder_internal.h")
foreach(_part IN LISTS _encparts)
  file(READ "${_part}" _part_text)
  string(APPEND C "\n${_part_text}")
endforeach()
foreach(NEEDED
    "enum class ProgressiveMvMode"
    "OneMvQpel"
    "OneMvHpel"
    "OneMvHpelBilinear"
    "MixedMv"
    "sampled_p_mode_rd"
    "sampled_b_mode_rd"
    "const PAnalysis* shared_seed=nullptr"
    "mv_suffix_bits"
    "mv_mode_halfpel"
    "if (c_.syntax==StreamSyntax::Wmv9Main) return q")
  string(FIND "${C}" "${NEEDED}" POS)
  if(POS EQUAL -1)
    message(FATAL_ERROR "progressive motion-RDO marker missing: ${NEEDED}")
  endif()
endforeach()
# Half-pel MVDATA is not just qpel differential / 2. The largest ordinary
# category and long-vector escape have reduced suffix widths.
foreach(NEEDED
    "return sz[cat] - ((halfpel && cat==5)?1:0);"
    "const int r=motion_mvrange();"
    "return (mvrange_kx(r)-hp)+(mvrange_ky(r)-hp);"
    "const int xb=mvrange_kx(r)-hp, yb=mvrange_ky(r)-hp;")
  string(FIND "${C}" "${NEEDED}" POS)
  if(POS EQUAL -1)
    message(FATAL_ERROR "half-pel MVDATA width marker missing: ${NEEDED}")
  endif()
endforeach()
# P alternate 1-MV modes must share the qpel full-search result rather than
# launching a second complete integer/global search.
string(FIND "${C}" "analyze_p_picture_core(f,ref,modes[static_cast<size_t>(i)],false,&qseed)" POS)
if(POS EQUAL -1)
  message(FATAL_ERROR "P motion modes do not reuse the qpel search seed")
endif()
string(FIND "${C}" "PAnalysis best=analyze_p_picture_core(f,ref,best_mode,true,&best_seed);" FINAL_EDGE_POS)
if(FINAL_EDGE_POS EQUAL -1)
  message(FATAL_ERROR "final P motion analysis marker missing")
endif()
# 0.1.82 also shares that same expensive qpel seed across the provisional ABR
# rate-control pass and the final progressive mode-RDO pass.
foreach(NEEDED
    "const PAnalysis* qpel_seed=nullptr"
    "shared_p_qpel_seed.emplace"
    "shared_p_qpel_seed?&*shared_p_qpel_seed:nullptr")
  string(FIND "${C}" "${NEEDED}" POS)
  if(POS EQUAL -1)
    message(FATAL_ERROR "ABR qpel-search sharing marker missing: ${NEEDED}")
  endif()
endforeach()

# 0.1.72 removes the old HRD/ABR early exits: bitrate-controlled pictures
# must run the same progressive motion-mode candidate set as CQP.
foreach(FORBIDDEN
    "if (c_.hrd_enabled) return q;"
    "Preserve the proven\n        // 0.1.67 motion path whenever HRD/ABR is active")
  string(FIND "${C}" "${FORBIDDEN}" POS)
  if(NOT POS EQUAL -1)
    message(FATAL_ERROR "obsolete ABR motion-RDO guard remains: ${FORBIDDEN}")
  endif()
endforeach()
foreach(NEEDED
    "0.1.72: ABR/VBV uses the same progressive motion-mode RDO as CQP"
    "0.1.72: ABR/VBV and CQP share this qpel-vs-halfpel B-picture RDO")
  string(FIND "${C}" "${NEEDED}" POS)
  if(POS EQUAL -1)
    message(FATAL_ERROR "ABR motion-RDO parity marker missing: ${NEEDED}")
  endif()
endforeach()

message(STATUS "progressive motion-mode RDO structural checks ok")
