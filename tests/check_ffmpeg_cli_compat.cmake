file(GLOB PY_TESTS "${SRC}/tests/*.py")
foreach(F IN LISTS PY_TESTS)
    file(READ "${F}" T)
    # FFmpeg 9 removed the legacy global sync option. Runtime decoder tests
    # must use the per-stream replacement so current FFmpeg remains usable.
    string(FIND "${T}" "\"-vsync\"" POS_DQ)
    string(FIND "${T}" "'-vsync'" POS_SQ)
    if(NOT POS_DQ EQUAL -1 OR NOT POS_SQ EQUAL -1)
        message(FATAL_ERROR "legacy FFmpeg sync option found in ${F}; use -fps_mode passthrough")
    endif()
endforeach()
message(STATUS "FFmpeg CLI compatibility check passed")
