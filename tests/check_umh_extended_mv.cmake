if(NOT DEFINED CORE)
  message(FATAL_ERROR "CORE not supplied")
endif()
if(NOT DEFINED FRONTEND)
  message(FATAL_ERROR "FRONTEND not supplied")
endif()
file(READ "${CORE}" C)
get_filename_component(_encdir "${CORE}" DIRECTORY)
file(GLOB _encparts "${_encdir}/encoder_*.cpp" "${_encdir}/encoder_internal.h")
foreach(_part IN LISTS _encparts)
  file(READ "${_part}" _part_text)
  string(APPEND C "\n${_part_text}")
endforeach()
file(READ "${FRONTEND}" F)

# Long-range search policy: local UMH and a propagated distant candidate are
# compared by default; local-good may skip distant work after a strong local
# match; legacy preserves the old propagated-distant-first short-circuit. The
# expensive content-signature index remains lazy in every mode.
foreach(NEEDED
    "integer_motion_umh"
    "propagated_long_range_motion"
    "extended_content_motion"
    "motion_match_good"
    "VC1_LONG_RANGE_COMPARE"
    "VC1_LONG_RANGE_LOCAL_GOOD_SKIP"
    "VC1_LONG_RANGE_LEGACY_DISTANT_FIRST"
    "distant_match_decision(propagated.full_sad,local_sad,mx,my)"
    "distant_match_decision(pr.full_sad,im.full_sad,mx,my)"
    "c_.distant_match_max_mae"
    "c_.long_range_search_mode==VC1_LONG_RANGE_LOCAL_GOOD_SKIP"
    "c_.long_range_search_mode==VC1_LONG_RANGE_LEGACY_DISTANT_FIRST"
    "if (range>local_range && !motion_match_good(im.full_sad,mx,my))"
    "build_long_range_index"
    "long_range_signature_distance"
    "if (std::abs(dx)<=32 && std::abs(dy)<=32) continue"
    "const int local_range=std::min(configured_range,std::max(0,c_.motion_local_search_range))")
  string(FIND "${C}" "${NEEDED}" POS)
  if(POS EQUAL -1)
    message(FATAL_ERROR "UMH/long-range motion marker missing: ${NEEDED}")
  endif()
endforeach()

foreach(NEEDED
    "--long-range-search"
    "compare|local-good|legacy"
    "VC1_LONG_RANGE_COMPARE"
    "VC1_LONG_RANGE_LOCAL_GOOD_SKIP"
    "VC1_LONG_RANGE_LEGACY_DISTANT_FIRST"
    "--distant-match-max-error")
  string(FIND "${F}" "${NEEDED}" POS)
  if(POS EQUAL -1)
    message(FATAL_ERROR "long-range policy CLI marker missing: ${NEEDED}")
  endif()
endforeach()

# ST 421 MVRANGE=111 reaches 4096 qpel horizontally and 1024 qpel vertically,
# with k_x/k_y 13/11 for long-vector MVDATA. Advanced Profile keeps the public
# 1024-pixel request. WMV3/Main uses one symmetric radius and therefore caps
# that symmetric integer-pixel radius to 255 so the positive vertical side
# cannot reach the illegal +256-pixel endpoint.
foreach(NEEDED
    "motion search range must be 0..1024 pixels"
    "p.i_motion_search_range=std::min(p.i_motion_search_range,255);"
    "p.i_motion_local_search_range=std::min(p.i_motion_local_search_range,255);"
    "static constexpr int v[4]={256,512,2048,4096};"
    "static constexpr int v[4]={128,256,512,1024};"
    "static constexpr int v[4]={9,10,12,13};"
    "static constexpr int v[4]={8,9,10,11};"
    "if (r<=31) return 0;"
    "if (r<=63) return 1;"
    "if (r<=127) return 2;"
    "return 3;"
    "b.bit(extended_mv_enabled()); // EXTENDED_MV"
    "write_mvrange(b);"
    "const int xb=mvrange_kx(r)-hp, yb=mvrange_ky(r)-hp;")
  string(FIND "${C}" "${NEEDED}" POS)
  if(POS EQUAL -1)
    message(FATAL_ERROR "extended-MV marker missing: ${NEEDED}")
  endif()
endforeach()

string(FIND "${F}" "--search-range" POS)
if(POS EQUAL -1)
  message(FATAL_ERROR "frontend search-range option missing")
endif()
message(STATUS "UMH / maximum extended-MV structural checks ok")
