if(NOT DEFINED CORE OR NOT DEFINED HEADER OR NOT DEFINED FRONTEND)
  message(FATAL_ERROR "CORE/HEADER/FRONTEND not supplied")
endif()
file(READ "${CORE}" C)
get_filename_component(_encdir "${CORE}" DIRECTORY)
file(GLOB _encparts "${_encdir}/encoder_*.cpp" "${_encdir}/encoder_internal.h")
foreach(_part IN LISTS _encparts)
  file(READ "${_part}" _part_text)
  string(APPEND C "\n${_part_text}")
endforeach()
file(READ "${HEADER}" H)
file(READ "${FRONTEND}" F)
string(REGEX MATCH "#define[ \t]+LIBVC1_API_VERSION[ \t]+([0-9]+)" API_VERSION_MATCH "${H}")
if(NOT API_VERSION_MATCH)
  message(FATAL_ERROR "intra-GOP parallelism public API version marker missing")
endif()
if(CMAKE_MATCH_1 LESS 7)
  message(FATAL_ERROR "intra-GOP parallelism requires public API version >= 7; found ${CMAKE_MATCH_1}")
endif()
foreach(NEEDED
    "int b_intra_gop_parallelism;"
    "int b_fixed_gop_grid;")
  string(FIND "${H}" "${NEEDED}" P)
  if(P EQUAL -1)
    message(FATAL_ERROR "missing intra-GOP public API marker: ${NEEDED}")
  endif()
endforeach()
foreach(NEEDED
    "bool intra_gop_parallelism=false"
    "std::vector<std::future<libvc1::Vc1Encoder::BAnalysis>> b_analysis_jobs"
    "gop_worker_limit() const"
    "p->b_intra_gop_parallelism=0"
    "process_dynamic_boundaries(bool flush_all)"
    "detect_scene_flags(current_gop,scan_params)"
    "p->b_fixed_gop_grid=0")
  string(FIND "${C}" "${NEEDED}" P)
  if(P EQUAL -1)
    message(FATAL_ERROR "missing intra-GOP implementation marker: ${NEEDED}")
  endif()
endforeach()
string(FIND "${F}" "--no-intra-gop-parallelism" P)
if(P EQUAL -1)
  message(FATAL_ERROR "missing --no-intra-gop-parallelism CLI switch")
endif()
string(FIND "${F}" "--fixed-gop-grid" P)
if(P EQUAL -1)
  message(FATAL_ERROR "missing fixed-GOP-grid compatibility CLI switch")
endif()

# 0.1.51: the 0.1.50 frontend inserted gop-grid between keyint and bframes in
# the summary. Keep smoke assertions synchronized with that public diagnostic.
file(READ "${CMAKE_CURRENT_LIST_DIR}/smoke.sh" S)
foreach(NEEDED
    "keyint=24, gop-grid=scene-reset, bframes=0"
    "keyint=8, gop-grid=scene-reset, bframes=2"
    "keyint=24, gop-grid=scene-reset, bframes=2"
    "gop-grid=fixed, bframes=0")
  string(FIND "${S}" "${NEEDED}" P)
  if(P EQUAL -1)
    message(FATAL_ERROR "stale GOP-summary smoke assertion: ${NEEDED}")
  endif()
endforeach()

message(STATUS "intra-GOP API/CLI/implementation checks ok")
