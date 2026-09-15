cmake_minimum_required(VERSION 3.16)
file(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/VERSION" LIBVC1_SOURCE_VERSION LIMIT_COUNT 1)
if (NOT LIBVC1_SOURCE_VERSION MATCHES "^[0-9]+\\.[0-9]+\\.[0-9]+$")
    message(FATAL_ERROR "VERSION must contain a semantic version (major.minor.patch)")
endif()
project(libvc1 VERSION "${LIBVC1_SOURCE_VERSION}" LANGUAGES C CXX)

include(GNUInstallDirs)
include(CheckCXXCompilerFlag)
include(CMakePackageConfigHelpers)
find_package(Threads REQUIRED)

option(LIBVC1_BUILD_SHARED "Build shared libvc1" ON)
option(LIBVC1_BUILD_STATIC "Build static libvc1" ON)
option(LIBVC1_BUILD_FRONTEND "Build vc1enc command-line frontend" ON)
option(LIBVC1_FRONTEND_LINK_STATIC "Link vc1enc against the static libvc1 target" OFF)
option(LIBVC1_BUILD_TESTS "Build libvc1 API tests" ON)
option(LIBVC1_ENABLE_X86_SIMD "Build x86 SIMD dispatch targets when supported" ON)
option(LIBVC1_ENABLE_MATROSKA "Enable MKV output through official libmatroska/libebml" ON)
option(LIBVC1_ENABLE_VULKAN "Enable optional Vulkan compute acceleration" ON)
option(LIBVC1_ENABLE_SANITIZERS "Enable AddressSanitizer and UndefinedBehaviorSanitizer" OFF)

set(LIBVC1_SIMD_TEST_MODE "native" CACHE STRING
    "ISA-specific SIMD test execution: native, qemu-fallback, or qemu-force")
set_property(CACHE LIBVC1_SIMD_TEST_MODE PROPERTY STRINGS native qemu-fallback qemu-force)
set(LIBVC1_QEMU_X86_64 "" CACHE FILEPATH
    "qemu-x86_64 executable used by qemu-fallback/qemu-force SIMD tests")
if (NOT LIBVC1_SIMD_TEST_MODE MATCHES "^(native|qemu-fallback|qemu-force)$")
    message(FATAL_ERROR "LIBVC1_SIMD_TEST_MODE must be native, qemu-fallback, or qemu-force")
endif()

if (LIBVC1_ENABLE_SANITIZERS)
    if (MSVC)
        message(FATAL_ERROR "LIBVC1_ENABLE_SANITIZERS currently requires GCC or Clang")
    endif()
    add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
    add_link_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
endif()

set(LIBVC1_HAVE_VULKAN OFF)
if (LIBVC1_ENABLE_VULKAN)
    find_package(Vulkan QUIET)
    find_program(LIBVC1_GLSLANG_VALIDATOR NAMES glslangValidator glslang)
    find_package(Python3 QUIET COMPONENTS Interpreter)
    if (Vulkan_FOUND AND LIBVC1_GLSLANG_VALIDATOR AND Python3_Interpreter_FOUND)
        set(LIBVC1_HAVE_VULKAN ON)
        set(LIBVC1_VULKAN_GENERATED_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
        set(LIBVC1_VULKAN_SPV "${LIBVC1_VULKAN_GENERATED_DIR}/vulkan_motion_cost.spv")
        set(LIBVC1_VULKAN_SPV_HEADER "${LIBVC1_VULKAN_GENERATED_DIR}/vulkan_motion_cost_spv.h")
        add_custom_command(
            OUTPUT "${LIBVC1_VULKAN_SPV_HEADER}"
            COMMAND "${CMAKE_COMMAND}" -E make_directory "${LIBVC1_VULKAN_GENERATED_DIR}"
            COMMAND "${LIBVC1_GLSLANG_VALIDATOR}" -V --target-env vulkan1.1
                    -o "${LIBVC1_VULKAN_SPV}" "${CMAKE_CURRENT_SOURCE_DIR}/src/shaders/vulkan_motion_cost.comp"
            COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake/embed_spirv.py"
                    "${LIBVC1_VULKAN_SPV}" "${LIBVC1_VULKAN_SPV_HEADER}"
            DEPENDS
                "${CMAKE_CURRENT_SOURCE_DIR}/src/shaders/vulkan_motion_cost.comp"
                "${CMAKE_CURRENT_SOURCE_DIR}/cmake/embed_spirv.py"
            VERBATIM)
        add_custom_target(libvc1_vulkan_shader DEPENDS "${LIBVC1_VULKAN_SPV_HEADER}")
        message(STATUS "Vulkan compute: enabled (${Vulkan_LIBRARY}; shader compiler ${LIBVC1_GLSLANG_VALIDATOR})")
    else()
        message(STATUS "Vulkan compute: disabled (requires Vulkan headers/loader, glslangValidator, and Python 3)")
    endif()
endif()

set(LIBVC1_HAVE_MATROSKA OFF)
if (LIBVC1_BUILD_FRONTEND AND LIBVC1_ENABLE_MATROSKA)
    find_package(Matroska 1.7 QUIET CONFIG)
    if (TARGET Matroska::matroska)
        set(LIBVC1_HAVE_MATROSKA ON)
        message(STATUS "Matroska output: enabled via Matroska::matroska")
    else()
        message(STATUS "Matroska output: disabled (libmatroska/libebml development package not found)")
    endif()
endif()

set(LIBVC1_COMMON_SOURCES
    src/encoder_core.cpp
    src/encoder_api_helpers.cpp
    src/encoder_bitstream.cpp
    src/encoder_encode.cpp
    src/encoder_interlaced.cpp
    src/encoder_motion.cpp
    src/encoder_motion_compensation.cpp
    src/encoder_transform.cpp
    src/encoder_aq.cpp
    src/encoder_entropy.cpp
    src/encoder_filter.cpp
    src/encoder_compute.cpp
    src/encoder_simd.cpp
    src/encoder_rate_control.cpp
    src/vulkan_compute.cpp
    src/vulkan_compute.h
    src/vc1_entropy_tables.h
    src/vc1_groupa_tables.h
    src/simd.h
    src/simd_satd4x4.h
    src/simd_recon_kernels.h
    src/forward_transform_tables.h)
if (LIBVC1_HAVE_VULKAN)
    list(APPEND LIBVC1_COMMON_SOURCES "${LIBVC1_VULKAN_SPV_HEADER}")
endif()

set(LIBVC1_SIMD_OBJECTS "")
string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" LIBVC1_PROCESSOR_LOWER)

# Keep target creation data-driven.  Adding a tuned microarchitecture later is
# one helper call plus its kernel translation unit (e.g. prescott/core2/penryn/
# sandybridge), rather than another copy of the object-library boilerplate.
function(libvc1_add_x86_simd_target tag march source have_var)
    string(TOUPPER "${tag}" tag_upper)
    string(REPLACE "-" "_" tag_upper "${tag_upper}")
    string(JOIN " " target_flag_probe "-march=${march}" ${ARGN})
    check_cxx_compiler_flag("${target_flag_probe}" "LIBVC1_COMPILER_HAS_${tag_upper}_FLAGS")
    if ("${LIBVC1_COMPILER_HAS_${tag_upper}_FLAGS}")
        set(obj "libvc1_simd_${tag}")
        add_library(${obj} OBJECT ${source})
        set_target_properties(${obj} PROPERTIES POSITION_INDEPENDENT_CODE ON)
        target_compile_features(${obj} PRIVATE cxx_std_17)
        target_compile_options(${obj} PRIVATE -march=${march} ${ARGN} -ffp-contract=off -fvisibility=hidden -Wall -Wextra -Wpedantic)
        list(APPEND LIBVC1_SIMD_OBJECTS $<TARGET_OBJECTS:${obj}>)
        set(LIBVC1_SIMD_OBJECTS "${LIBVC1_SIMD_OBJECTS}" PARENT_SCOPE)
        set(${have_var} 1 PARENT_SCOPE)
    endif()
endfunction()

if (LIBVC1_ENABLE_X86_SIMD AND LIBVC1_PROCESSOR_LOWER MATCHES "^(x86_64|amd64)$" AND NOT MSVC)
    libvc1_add_x86_simd_target(x86_v1 x86-64 src/simd_x86_v1.cpp LIBVC1_HAVE_V1)
    libvc1_add_x86_simd_target(prescott x86-64 src/simd_prescott.cpp LIBVC1_HAVE_PRESCOTT -msse3 -mtune=nocona)
    libvc1_add_x86_simd_target(k10 x86-64 src/simd_k10.cpp LIBVC1_HAVE_K10
        -msse3 -msse4a -mpopcnt -mlzcnt -mtune=amdfam10)
    libvc1_add_x86_simd_target(conroe x86-64 src/simd_conroe.cpp LIBVC1_HAVE_CONROE -msse3 -mssse3 -mtune=core2)
    libvc1_add_x86_simd_target(penryn x86-64 src/simd_penryn.cpp LIBVC1_HAVE_PENRYN -msse3 -mssse3 -msse4.1 -mtune=core2)
    libvc1_add_x86_simd_target(x86_v2 x86-64-v2 src/simd_x86_v2.cpp LIBVC1_HAVE_V2)
    libvc1_add_x86_simd_target(sandybridge x86-64 src/simd_sandybridge.cpp LIBVC1_HAVE_SANDYBRIDGE
        -msse3 -mssse3 -msse4.1 -msse4.2 -mpopcnt -mavx -mtune=sandybridge)
    libvc1_add_x86_simd_target(bulldozer x86-64 src/simd_bulldozer.cpp LIBVC1_HAVE_BULLDOZER
        -msse3 -mssse3 -msse4.1 -msse4.2 -mpopcnt -mlzcnt -mavx -mxop -mfma4 -mtune=bdver1)
    libvc1_add_x86_simd_target(piledriver x86-64 src/simd_piledriver.cpp LIBVC1_HAVE_PILEDRIVER
        -msse3 -mssse3 -msse4.1 -msse4.2 -mpopcnt -mlzcnt -mavx -mxop -mfma4 -mfma -mf16c -mbmi -mtune=bdver2)
    libvc1_add_x86_simd_target(avx2_partial x86-64-v2 src/simd_avx2_partial.cpp LIBVC1_HAVE_AVX2_PARTIAL
        -mavx -mavx2 -mno-fma -mno-bmi -mno-bmi2 -mno-f16c)
    libvc1_add_x86_simd_target(x86_v3 x86-64-v3 src/simd_x86_v3.cpp LIBVC1_HAVE_V3)
    libvc1_add_x86_simd_target(x86_v4 x86-64-v4 src/simd_x86_v4.cpp LIBVC1_HAVE_V4)
endif()

function(libvc1_configure_target target)
    target_sources(${target} PRIVATE ${LIBVC1_COMMON_SOURCES} ${LIBVC1_SIMD_OBJECTS})
    target_compile_features(${target} PRIVATE cxx_std_17)
    target_include_directories(${target}
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
    target_link_libraries(${target} PRIVATE Threads::Threads)
    if (LIBVC1_HAVE_VULKAN)
        add_dependencies(${target} libvc1_vulkan_shader)
        target_compile_definitions(${target} PRIVATE LIBVC1_HAVE_VULKAN=1)
        target_include_directories(${target} PRIVATE "${LIBVC1_VULKAN_GENERATED_DIR}")
        target_link_libraries(${target} PRIVATE Vulkan::Vulkan)
    endif()
    if (LIBVC1_HAVE_V1)
        target_compile_definitions(${target} PRIVATE LIBVC1_HAVE_X86_64_V1=1)
    endif()
    if (LIBVC1_HAVE_PRESCOTT)
        target_compile_definitions(${target} PRIVATE LIBVC1_HAVE_PRESCOTT=1)
    endif()
    if (LIBVC1_HAVE_K10)
        target_compile_definitions(${target} PRIVATE LIBVC1_HAVE_K10=1)
    endif()
    if (LIBVC1_HAVE_CONROE)
        target_compile_definitions(${target} PRIVATE LIBVC1_HAVE_CONROE=1)
    endif()
    if (LIBVC1_HAVE_PENRYN)
        target_compile_definitions(${target} PRIVATE LIBVC1_HAVE_PENRYN=1)
    endif()
    if (LIBVC1_HAVE_V2)
        target_compile_definitions(${target} PRIVATE LIBVC1_HAVE_X86_64_V2=1)
    endif()
    if (LIBVC1_HAVE_SANDYBRIDGE)
        target_compile_definitions(${target} PRIVATE LIBVC1_HAVE_SANDYBRIDGE=1)
    endif()
    if (LIBVC1_HAVE_BULLDOZER)
        target_compile_definitions(${target} PRIVATE LIBVC1_HAVE_BULLDOZER=1)
    endif()
    if (LIBVC1_HAVE_PILEDRIVER)
        target_compile_definitions(${target} PRIVATE LIBVC1_HAVE_PILEDRIVER=1)
    endif()
    if (LIBVC1_HAVE_AVX2_PARTIAL)
        target_compile_definitions(${target} PRIVATE LIBVC1_HAVE_AVX2_PARTIAL=1)
    endif()
    if (LIBVC1_HAVE_V3)
        target_compile_definitions(${target} PRIVATE LIBVC1_HAVE_X86_64_V3=1)
    endif()
    if (LIBVC1_HAVE_V4)
        target_compile_definitions(${target} PRIVATE LIBVC1_HAVE_X86_64_V4=1)
    endif()
    if (MSVC)
        target_compile_options(${target} PRIVATE /W4 /permissive-)
    else()
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -ffp-contract=off)
    endif()
endfunction()

if (LIBVC1_BUILD_SHARED)
    add_library(vc1 SHARED)
    add_library(libvc1::vc1 ALIAS vc1)
    libvc1_configure_target(vc1)
    target_compile_definitions(vc1 PRIVATE LIBVC1_BUILDING_SHARED=1)
    set_target_properties(vc1 PROPERTIES
        OUTPUT_NAME vc1
        VERSION ${PROJECT_VERSION}
        SOVERSION 0
        CXX_VISIBILITY_PRESET hidden
        VISIBILITY_INLINES_HIDDEN YES)
    if (UNIX AND NOT APPLE AND NOT MSVC)
        target_link_options(vc1 PRIVATE "LINKER:--version-script=${CMAKE_CURRENT_SOURCE_DIR}/src/libvc1.map")
    endif()
endif()

if (LIBVC1_BUILD_STATIC)
    add_library(vc1_static STATIC)
    add_library(libvc1::vc1_static ALIAS vc1_static)
    libvc1_configure_target(vc1_static)
    target_compile_definitions(vc1_static PUBLIC LIBVC1_STATIC=1)
    # A C application may consume the C ABI while linking the C++ implementation statically.
    # Propagate the compiler runtime libraries so the consumer can still use a C linker driver.
    target_link_libraries(vc1_static INTERFACE ${CMAKE_CXX_IMPLICIT_LINK_LIBRARIES})
    if (WIN32)
        set_target_properties(vc1_static PROPERTIES OUTPUT_NAME vc1_static)
    else()
        set_target_properties(vc1_static PROPERTIES OUTPUT_NAME vc1)
    endif()
endif()

# Frontend and tests are defined below once their source files are present.
if (LIBVC1_BUILD_FRONTEND)
    add_executable(vc1enc src/frontend.cpp src/matroska_muxer.cpp src/matroska_muxer.h)
    target_compile_features(vc1enc PRIVATE cxx_std_17)
    target_include_directories(vc1enc PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/src)
    if (LIBVC1_FRONTEND_LINK_STATIC AND TARGET vc1_static)
        target_link_libraries(vc1enc PRIVATE vc1_static)
    elseif (TARGET vc1)
        target_link_libraries(vc1enc PRIVATE vc1)
    else()
        target_link_libraries(vc1enc PRIVATE vc1_static)
    endif()
    if (LIBVC1_HAVE_MATROSKA)
        target_link_libraries(vc1enc PRIVATE Matroska::matroska)
        target_compile_definitions(vc1enc PRIVATE LIBVC1_HAVE_MATROSKA=1)
    endif()
    if (MSVC)
        target_compile_options(vc1enc PRIVATE /W4 /permissive-)
    else()
        target_compile_options(vc1enc PRIVATE -Wall -Wextra -Wpedantic)
    endif()
    # Keep the installed frontend self-contained with the shared library.
    # This is especially important on NixOS, where there is intentionally no
    # global /usr/lib fallback.  A relative RUNPATH also keeps relocatable
    # tarball/local installs working.
    if (UNIX AND NOT APPLE AND TARGET vc1 AND NOT LIBVC1_FRONTEND_LINK_STATIC)
        set_target_properties(vc1enc PROPERTIES INSTALL_RPATH "$ORIGIN/../${CMAKE_INSTALL_LIBDIR}")
    elseif (APPLE AND TARGET vc1 AND NOT LIBVC1_FRONTEND_LINK_STATIC)
        set_target_properties(vc1enc PROPERTIES INSTALL_RPATH "@loader_path/../${CMAKE_INSTALL_LIBDIR}")
    endif()
endif()

if (LIBVC1_BUILD_TESTS)
    enable_testing()

    # Native is the historical/default behavior.  In qemu-fallback mode each
    # ISA-specific test executes directly first and only exit 77 (unsupported
    # host ISA) is retried under a target-constrained qemu-x86_64 CPU.
    # qemu-force always uses the constrained emulated CPU, which doubles as an
    # ISA-leakage test (for example, AVX2 accidentally entering the v2 path).
    if (NOT LIBVC1_SIMD_TEST_MODE STREQUAL "native")
        if (NOT UNIX OR NOT LIBVC1_PROCESSOR_LOWER MATCHES "^(x86_64|amd64)$")
            message(FATAL_ERROR "QEMU SIMD test modes currently require an x86-64 UNIX build")
        endif()
        find_package(Python3 COMPONENTS Interpreter REQUIRED)
        if (NOT LIBVC1_QEMU_X86_64)
            find_program(_LIBVC1_QEMU_X86_64 NAMES qemu-x86_64)
            if (_LIBVC1_QEMU_X86_64)
                set(LIBVC1_QEMU_X86_64 "${_LIBVC1_QEMU_X86_64}" CACHE FILEPATH
                    "qemu-x86_64 executable used by qemu-fallback/qemu-force SIMD tests" FORCE)
            endif()
        endif()
        if (NOT LIBVC1_QEMU_X86_64 OR NOT EXISTS "${LIBVC1_QEMU_X86_64}")
            message(FATAL_ERROR "${LIBVC1_SIMD_TEST_MODE} requires qemu-x86_64; install qemu-user or set LIBVC1_QEMU_X86_64")
        endif()

        add_executable(libvc1_simd_emulated_cpu_probe tests/simd_emulated_cpu_probe.cpp)
        target_compile_features(libvc1_simd_emulated_cpu_probe PRIVATE cxx_std_17)
        target_include_directories(libvc1_simd_emulated_cpu_probe PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include)
        if (TARGET vc1_static)
            target_link_libraries(libvc1_simd_emulated_cpu_probe PRIVATE vc1_static)
        else()
            target_link_libraries(libvc1_simd_emulated_cpu_probe PRIVATE vc1)
        endif()
    endif()

    function(libvc1_add_isa_test test_name executable target_tag)
        if (LIBVC1_SIMD_TEST_MODE STREQUAL "native")
            add_test(NAME ${test_name} COMMAND ${executable})
        else()
            if (LIBVC1_SIMD_TEST_MODE STREQUAL "qemu-force")
                set(_mode force)
            else()
                set(_mode fallback)
            endif()
            add_test(NAME ${test_name}
                COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/run_simd_isa_test.py
                    --mode ${_mode} --target ${target_tag}
                    --qemu ${LIBVC1_QEMU_X86_64}
                    --probe $<TARGET_FILE:libvc1_simd_emulated_cpu_probe>
                    $<TARGET_FILE:${executable}>)
        endif()
        set_tests_properties(${test_name} PROPERTIES SKIP_RETURN_CODE 77)
    endfunction()
    if (LIBVC1_HAVE_V1 AND UNIX)
        add_executable(libvc1_simd_chroma_edge_guard_v1
            tests/simd_chroma_edge_guard.cpp
            $<TARGET_OBJECTS:libvc1_simd_x86_v1>)
        target_compile_features(libvc1_simd_chroma_edge_guard_v1 PRIVATE cxx_std_17)
        target_include_directories(libvc1_simd_chroma_edge_guard_v1 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
        target_compile_definitions(libvc1_simd_chroma_edge_guard_v1 PRIVATE LIBVC1_TEST_SIMD_V1=1)
        libvc1_add_isa_test(libvc1-simd-chroma-edge-guard-v1 libvc1_simd_chroma_edge_guard_v1 v1)
        add_executable(libvc1_simd_forward_transform_v1
            tests/simd_forward_transform.cpp
            $<TARGET_OBJECTS:libvc1_simd_x86_v1>)
        target_compile_features(libvc1_simd_forward_transform_v1 PRIVATE cxx_std_17)
        target_include_directories(libvc1_simd_forward_transform_v1 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
        target_compile_definitions(libvc1_simd_forward_transform_v1 PRIVATE LIBVC1_TEST_SIMD_V1=1)
        libvc1_add_isa_test(libvc1-simd-forward-transform-v1 libvc1_simd_forward_transform_v1 v1)
        add_executable(libvc1_simd_frame_sad_v1
            tests/simd_frame_sad.cpp
            $<TARGET_OBJECTS:libvc1_simd_x86_v1>)
        target_compile_features(libvc1_simd_frame_sad_v1 PRIVATE cxx_std_17)
        target_include_directories(libvc1_simd_frame_sad_v1 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
        target_compile_definitions(libvc1_simd_frame_sad_v1 PRIVATE LIBVC1_TEST_SIMD_V1=1)
        libvc1_add_isa_test(libvc1-simd-frame-sad-v1 libvc1_simd_frame_sad_v1 v1)
    endif()
    if (LIBVC1_HAVE_PRESCOTT AND UNIX)
        add_executable(libvc1_simd_chroma_edge_guard_prescott
            tests/simd_chroma_edge_guard.cpp
            $<TARGET_OBJECTS:libvc1_simd_prescott>)
        target_compile_features(libvc1_simd_chroma_edge_guard_prescott PRIVATE cxx_std_17)
        target_include_directories(libvc1_simd_chroma_edge_guard_prescott PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
        target_compile_definitions(libvc1_simd_chroma_edge_guard_prescott PRIVATE LIBVC1_TEST_SIMD_PRESCOTT=1)
        libvc1_add_isa_test(libvc1-simd-chroma-edge-guard-prescott libvc1_simd_chroma_edge_guard_prescott prescott)
        add_executable(libvc1_simd_forward_transform_prescott
            tests/simd_forward_transform.cpp
            $<TARGET_OBJECTS:libvc1_simd_prescott>)
        target_compile_features(libvc1_simd_forward_transform_prescott PRIVATE cxx_std_17)
        target_include_directories(libvc1_simd_forward_transform_prescott PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
        target_compile_definitions(libvc1_simd_forward_transform_prescott PRIVATE LIBVC1_TEST_SIMD_PRESCOTT=1)
        libvc1_add_isa_test(libvc1-simd-forward-transform-prescott libvc1_simd_forward_transform_prescott prescott)
        add_executable(libvc1_simd_frame_sad_prescott
            tests/simd_frame_sad.cpp
            $<TARGET_OBJECTS:libvc1_simd_prescott>)
        target_compile_features(libvc1_simd_frame_sad_prescott PRIVATE cxx_std_17)
        target_include_directories(libvc1_simd_frame_sad_prescott PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
        target_compile_definitions(libvc1_simd_frame_sad_prescott PRIVATE LIBVC1_TEST_SIMD_PRESCOTT=1)
        libvc1_add_isa_test(libvc1-simd-frame-sad-prescott libvc1_simd_frame_sad_prescott prescott)
    endif()
    if (LIBVC1_HAVE_CONROE AND UNIX)
        add_executable(libvc1_simd_chroma_edge_guard_conroe
            tests/simd_chroma_edge_guard.cpp
            $<TARGET_OBJECTS:libvc1_simd_conroe>)
        target_compile_features(libvc1_simd_chroma_edge_guard_conroe PRIVATE cxx_std_17)
        target_include_directories(libvc1_simd_chroma_edge_guard_conroe PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
        target_compile_definitions(libvc1_simd_chroma_edge_guard_conroe PRIVATE LIBVC1_TEST_SIMD_CONROE=1)
        libvc1_add_isa_test(libvc1-simd-chroma-edge-guard-conroe libvc1_simd_chroma_edge_guard_conroe conroe)
        add_executable(libvc1_simd_forward_transform_conroe
            tests/simd_forward_transform.cpp
            $<TARGET_OBJECTS:libvc1_simd_conroe>)
        target_compile_features(libvc1_simd_forward_transform_conroe PRIVATE cxx_std_17)
        target_include_directories(libvc1_simd_forward_transform_conroe PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
        target_compile_definitions(libvc1_simd_forward_transform_conroe PRIVATE LIBVC1_TEST_SIMD_CONROE=1)
        libvc1_add_isa_test(libvc1-simd-forward-transform-conroe libvc1_simd_forward_transform_conroe conroe)
        add_executable(libvc1_simd_frame_sad_conroe
            tests/simd_frame_sad.cpp
            $<TARGET_OBJECTS:libvc1_simd_conroe>)
        target_compile_features(libvc1_simd_frame_sad_conroe PRIVATE cxx_std_17)
        target_include_directories(libvc1_simd_frame_sad_conroe PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
        target_compile_definitions(libvc1_simd_frame_sad_conroe PRIVATE LIBVC1_TEST_SIMD_CONROE=1)
        libvc1_add_isa_test(libvc1-simd-frame-sad-conroe libvc1_simd_frame_sad_conroe conroe)
    endif()
    if (LIBVC1_HAVE_PENRYN AND UNIX)
        add_executable(libvc1_simd_chroma_edge_guard_penryn
            tests/simd_chroma_edge_guard.cpp
            $<TARGET_OBJECTS:libvc1_simd_penryn>)
        target_compile_features(libvc1_simd_chroma_edge_guard_penryn PRIVATE cxx_std_17)
        target_include_directories(libvc1_simd_chroma_edge_guard_penryn PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
        target_compile_definitions(libvc1_simd_chroma_edge_guard_penryn PRIVATE LIBVC1_TEST_SIMD_PENRYN=1)
        libvc1_add_isa_test(libvc1-simd-chroma-edge-guard-penryn libvc1_simd_chroma_edge_guard_penryn penryn)
        add_executable(libvc1_simd_forward_transform_penryn
            tests/simd_forward_transform.cpp
            $<TARGET_OBJECTS:libvc1_simd_penryn>)
        target_compile_features(libvc1_simd_forward_transform_penryn PRIVATE cxx_std_17)
        target_include_directories(libvc1_simd_forward_transform_penryn PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
        target_compile_definitions(libvc1_simd_forward_transform_penryn PRIVATE LIBVC1_TEST_SIMD_PENRYN=1)
        libvc1_add_isa_test(libvc1-simd-forward-transform-penryn libvc1_simd_forward_transform_penryn penryn)
        add_executable(libvc1_simd_frame_sad_penryn
            tests/simd_frame_sad.cpp
            $<TARGET_OBJECTS:libvc1_simd_penryn>)
        target_compile_features(libvc1_simd_frame_sad_penryn PRIVATE cxx_std_17)
        target_include_directories(libvc1_simd_frame_sad_penryn PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
        target_compile_definitions(libvc1_simd_frame_sad_penryn PRIVATE LIBVC1_TEST_SIMD_PENRYN=1)
        libvc1_add_isa_test(libvc1-simd-frame-sad-penryn libvc1_simd_frame_sad_penryn penryn)
    endif()
    if (LIBVC1_HAVE_SANDYBRIDGE AND UNIX)
        add_executable(libvc1_simd_chroma_edge_guard_sandybridge
            tests/simd_chroma_edge_guard.cpp
            $<TARGET_OBJECTS:libvc1_simd_sandybridge>)
        target_compile_features(libvc1_simd_chroma_edge_guard_sandybridge PRIVATE cxx_std_17)
        target_include_directories(libvc1_simd_chroma_edge_guard_sandybridge PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
        target_compile_definitions(libvc1_simd_chroma_edge_guard_sandybridge PRIVATE LIBVC1_TEST_SIMD_SANDYBRIDGE=1)
        libvc1_add_isa_test(libvc1-simd-chroma-edge-guard-sandybridge libvc1_simd_chroma_edge_guard_sandybridge sandybridge)
        add_executable(libvc1_simd_forward_transform_sandybridge
            tests/simd_forward_transform.cpp
            $<TARGET_OBJECTS:libvc1_simd_sandybridge>)
        target_compile_features(libvc1_simd_forward_transform_sandybridge PRIVATE cxx_std_17)
        target_include_directories(libvc1_simd_forward_transform_sandybridge PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
        target_compile_definitions(libvc1_simd_forward_transform_sandybridge PRIVATE LIBVC1_TEST_SIMD_SANDYBRIDGE=1)
        libvc1_add_isa_test(libvc1-simd-forward-transform-sandybridge libvc1_simd_forward_transform_sandybridge sandybridge)
        add_executable(libvc1_simd_frame_sad_sandybridge
            tests/simd_frame_sad.cpp
            $<TARGET_OBJECTS:libvc1_simd_sandybridge>)
        target_compile_features(libvc1_simd_frame_sad_sandybridge PRIVATE cxx_std_17)
        target_include_directories(libvc1_simd_frame_sad_sandybridge PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
        target_compile_definitions(libvc1_simd_frame_sad_sandybridge PRIVATE LIBVC1_TEST_SIMD_SANDYBRIDGE=1)
        libvc1_add_isa_test(libvc1-simd-frame-sad-sandybridge libvc1_simd_frame_sad_sandybridge sandybridge)
    endif()
    if (LIBVC1_HAVE_V2 AND UNIX)
        add_executable(libvc1_simd_chroma_edge_guard_v2
            tests/simd_chroma_edge_guard.cpp
            $<TARGET_OBJECTS:libvc1_simd_x86_v2>)
        target_compile_features(libvc1_simd_chroma_edge_guard_v2 PRIVATE cxx_std_17)
        target_include_directories(libvc1_simd_chroma_edge_guard_v2 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
        target_compile_definitions(libvc1_simd_chroma_edge_guard_v2 PRIVATE LIBVC1_TEST_SIMD_V2=1)
        libvc1_add_isa_test(libvc1-simd-chroma-edge-guard-v2 libvc1_simd_chroma_edge_guard_v2 v2)
        add_executable(libvc1_simd_forward_transform_v2
            tests/simd_forward_transform.cpp
            $<TARGET_OBJECTS:libvc1_simd_x86_v2>)
        target_compile_features(libvc1_simd_forward_transform_v2 PRIVATE cxx_std_17)
        target_include_directories(libvc1_simd_forward_transform_v2 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
        target_compile_definitions(libvc1_simd_forward_transform_v2 PRIVATE LIBVC1_TEST_SIMD_V2=1)
        libvc1_add_isa_test(libvc1-simd-forward-transform-v2 libvc1_simd_forward_transform_v2 v2)
        add_executable(libvc1_simd_frame_sad_v2
            tests/simd_frame_sad.cpp
            $<TARGET_OBJECTS:libvc1_simd_x86_v2>)
        target_compile_features(libvc1_simd_frame_sad_v2 PRIVATE cxx_std_17)
        target_include_directories(libvc1_simd_frame_sad_v2 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
        target_compile_definitions(libvc1_simd_frame_sad_v2 PRIVATE LIBVC1_TEST_SIMD_V2=1)
        libvc1_add_isa_test(libvc1-simd-frame-sad-v2 libvc1_simd_frame_sad_v2 v2)
    endif()
    if (LIBVC1_HAVE_V3 AND UNIX)
        add_executable(libvc1_simd_chroma_edge_guard_v3
            tests/simd_chroma_edge_guard.cpp
            $<TARGET_OBJECTS:libvc1_simd_x86_v3>)
        target_compile_features(libvc1_simd_chroma_edge_guard_v3 PRIVATE cxx_std_17)
        target_include_directories(libvc1_simd_chroma_edge_guard_v3 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
        target_compile_definitions(libvc1_simd_chroma_edge_guard_v3 PRIVATE LIBVC1_TEST_SIMD_V3=1)
        libvc1_add_isa_test(libvc1-simd-chroma-edge-guard-v3 libvc1_simd_chroma_edge_guard_v3 v3)
        add_executable(libvc1_simd_forward_transform_v3
            tests/simd_forward_transform.cpp
            $<TARGET_OBJECTS:libvc1_simd_x86_v3>)
        target_compile_features(libvc1_simd_forward_transform_v3 PRIVATE cxx_std_17)
        target_include_directories(libvc1_simd_forward_transform_v3 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
        target_compile_definitions(libvc1_simd_forward_transform_v3 PRIVATE LIBVC1_TEST_SIMD_V3=1)
        libvc1_add_isa_test(libvc1-simd-forward-transform-v3 libvc1_simd_forward_transform_v3 v3)
        add_executable(libvc1_simd_frame_sad_v3
            tests/simd_frame_sad.cpp
            $<TARGET_OBJECTS:libvc1_simd_x86_v3>)
        target_compile_features(libvc1_simd_frame_sad_v3 PRIVATE cxx_std_17)
        target_include_directories(libvc1_simd_frame_sad_v3 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
        target_compile_definitions(libvc1_simd_frame_sad_v3 PRIVATE LIBVC1_TEST_SIMD_V3=1)
        libvc1_add_isa_test(libvc1-simd-frame-sad-v3 libvc1_simd_frame_sad_v3 v3)
    endif()
    if (LIBVC1_HAVE_V4 AND UNIX)
        add_executable(libvc1_simd_chroma_edge_guard_v4
            tests/simd_chroma_edge_guard.cpp
            $<TARGET_OBJECTS:libvc1_simd_x86_v4>)
        target_compile_features(libvc1_simd_chroma_edge_guard_v4 PRIVATE cxx_std_17)
        target_include_directories(libvc1_simd_chroma_edge_guard_v4 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
        target_compile_definitions(libvc1_simd_chroma_edge_guard_v4 PRIVATE LIBVC1_TEST_SIMD_V4=1)
        libvc1_add_isa_test(libvc1-simd-chroma-edge-guard-v4 libvc1_simd_chroma_edge_guard_v4 v4)
        add_executable(libvc1_simd_forward_transform_v4
            tests/simd_forward_transform.cpp
            $<TARGET_OBJECTS:libvc1_simd_x86_v4>)
        target_compile_features(libvc1_simd_forward_transform_v4 PRIVATE cxx_std_17)
        target_include_directories(libvc1_simd_forward_transform_v4 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
        target_compile_definitions(libvc1_simd_forward_transform_v4 PRIVATE LIBVC1_TEST_SIMD_V4=1)
        libvc1_add_isa_test(libvc1-simd-forward-transform-v4 libvc1_simd_forward_transform_v4 v4)
        add_executable(libvc1_simd_frame_sad_v4
            tests/simd_frame_sad.cpp
            $<TARGET_OBJECTS:libvc1_simd_x86_v4>)
        target_compile_features(libvc1_simd_frame_sad_v4 PRIVATE cxx_std_17)
        target_include_directories(libvc1_simd_frame_sad_v4 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
        target_compile_definitions(libvc1_simd_frame_sad_v4 PRIVATE LIBVC1_TEST_SIMD_V4=1)
        libvc1_add_isa_test(libvc1-simd-frame-sad-v4 libvc1_simd_frame_sad_v4 v4)
    endif()
    function(libvc1_add_feature_band_simd_tests tag object_target test_define)
        if (UNIX)
            add_executable(libvc1_simd_chroma_edge_guard_${tag}
                tests/simd_chroma_edge_guard.cpp $<TARGET_OBJECTS:${object_target}>)
            target_compile_features(libvc1_simd_chroma_edge_guard_${tag} PRIVATE cxx_std_17)
            target_include_directories(libvc1_simd_chroma_edge_guard_${tag} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
            target_compile_definitions(libvc1_simd_chroma_edge_guard_${tag} PRIVATE ${test_define}=1)
            libvc1_add_isa_test(libvc1-simd-chroma-edge-guard-${tag} libvc1_simd_chroma_edge_guard_${tag} ${tag})

            add_executable(libvc1_simd_forward_transform_${tag}
                tests/simd_forward_transform.cpp $<TARGET_OBJECTS:${object_target}>)
            target_compile_features(libvc1_simd_forward_transform_${tag} PRIVATE cxx_std_17)
            target_include_directories(libvc1_simd_forward_transform_${tag} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
            target_compile_definitions(libvc1_simd_forward_transform_${tag} PRIVATE ${test_define}=1)
            libvc1_add_isa_test(libvc1-simd-forward-transform-${tag} libvc1_simd_forward_transform_${tag} ${tag})

            add_executable(libvc1_simd_frame_sad_${tag}
                tests/simd_frame_sad.cpp $<TARGET_OBJECTS:${object_target}>)
            target_compile_features(libvc1_simd_frame_sad_${tag} PRIVATE cxx_std_17)
            target_include_directories(libvc1_simd_frame_sad_${tag} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
            target_compile_definitions(libvc1_simd_frame_sad_${tag} PRIVATE ${test_define}=1)
            libvc1_add_isa_test(libvc1-simd-frame-sad-${tag} libvc1_simd_frame_sad_${tag} ${tag})
        endif()
    endfunction()
    if (LIBVC1_HAVE_K10)
        libvc1_add_feature_band_simd_tests(k10 libvc1_simd_k10 LIBVC1_TEST_SIMD_K10)
    endif()
    if (LIBVC1_HAVE_BULLDOZER)
        libvc1_add_feature_band_simd_tests(bulldozer libvc1_simd_bulldozer LIBVC1_TEST_SIMD_BULLDOZER)
    endif()
    if (LIBVC1_HAVE_PILEDRIVER)
        libvc1_add_feature_band_simd_tests(piledriver libvc1_simd_piledriver LIBVC1_TEST_SIMD_PILEDRIVER)
    endif()
    if (LIBVC1_HAVE_AVX2_PARTIAL)
        libvc1_add_feature_band_simd_tests(avx2-partial libvc1_simd_avx2_partial LIBVC1_TEST_SIMD_AVX2_PARTIAL)
    endif()

    if (LIBVC1_HAVE_V1)
        add_executable(libvc1_simd_reconstruction tests/simd_reconstruction.cpp)
        target_compile_features(libvc1_simd_reconstruction PRIVATE cxx_std_17)
        target_include_directories(libvc1_simd_reconstruction PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include)
        if (TARGET vc1_static)
            target_link_libraries(libvc1_simd_reconstruction PRIVATE vc1_static)
        else()
            target_link_libraries(libvc1_simd_reconstruction PRIVATE vc1)
        endif()
        add_test(NAME libvc1-simd-reconstruction COMMAND libvc1_simd_reconstruction)
    endif()

    add_executable(libvc1_simd_dispatch_runtime tests/simd_dispatch_runtime.cpp)
    target_compile_features(libvc1_simd_dispatch_runtime PRIVATE cxx_std_17)
    target_include_directories(libvc1_simd_dispatch_runtime PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include)
    if (TARGET vc1_static)
        target_link_libraries(libvc1_simd_dispatch_runtime PRIVATE vc1_static)
    else()
        target_link_libraries(libvc1_simd_dispatch_runtime PRIVATE vc1)
    endif()
    add_test(NAME libvc1-simd-dispatch-runtime COMMAND libvc1_simd_dispatch_runtime)

    add_executable(libvc1_main_profile_loopfilter tests/main_profile_loopfilter.cpp)
    target_compile_features(libvc1_main_profile_loopfilter PRIVATE cxx_std_17)
    target_include_directories(libvc1_main_profile_loopfilter PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include)
    if (TARGET vc1_static)
        target_link_libraries(libvc1_main_profile_loopfilter PRIVATE vc1_static)
    else()
        target_link_libraries(libvc1_main_profile_loopfilter PRIVATE vc1)
    endif()
    add_test(NAME libvc1-main-profile-loopfilter COMMAND libvc1_main_profile_loopfilter)

    add_executable(libvc1_dimension_limits tests/dimension_limits.cpp)
    target_compile_features(libvc1_dimension_limits PRIVATE cxx_std_17)
    target_include_directories(libvc1_dimension_limits PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    if (TARGET vc1_static)
        target_link_libraries(libvc1_dimension_limits PRIVATE vc1_static)
    else()
        target_link_libraries(libvc1_dimension_limits PRIVATE vc1)
    endif()
    add_test(NAME libvc1-dimension-limits COMMAND libvc1_dimension_limits)

    add_executable(libvc1_bluray_compat tests/bluray_compat.cpp)
    target_compile_features(libvc1_bluray_compat PRIVATE cxx_std_17)
    target_include_directories(libvc1_bluray_compat PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    if (TARGET vc1_static)
        target_link_libraries(libvc1_bluray_compat PRIVATE vc1_static)
    else()
        target_link_libraries(libvc1_bluray_compat PRIVATE vc1)
    endif()
    add_test(NAME libvc1-bluray-compat COMMAND libvc1_bluray_compat)

    add_executable(libvc1_visible_edge_inter tests/visible_edge_inter.cpp)
    target_compile_features(libvc1_visible_edge_inter PRIVATE cxx_std_17)
    target_include_directories(libvc1_visible_edge_inter PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include)
    if (TARGET vc1_static)
        target_link_libraries(libvc1_visible_edge_inter PRIVATE vc1_static)
    else()
        target_link_libraries(libvc1_visible_edge_inter PRIVATE vc1)
    endif()
    add_test(NAME libvc1-visible-edge-inter COMMAND libvc1_visible_edge_inter)

    add_executable(libvc1_staged_motion_estimation tests/staged_motion_estimation.cpp)
    target_compile_features(libvc1_staged_motion_estimation PRIVATE cxx_std_17)
    target_include_directories(libvc1_staged_motion_estimation PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include)
    if (TARGET vc1_static)
        target_link_libraries(libvc1_staged_motion_estimation PRIVATE vc1_static)
    else()
        target_link_libraries(libvc1_staged_motion_estimation PRIVATE vc1)
    endif()
    add_test(NAME libvc1-staged-motion-estimation COMMAND libvc1_staged_motion_estimation)

    if (LIBVC1_HAVE_VULKAN)
        add_executable(libvc1_vulkan_compute_runtime tests/vulkan_compute_runtime.cpp)
        target_compile_features(libvc1_vulkan_compute_runtime PRIVATE cxx_std_17)
        target_include_directories(libvc1_vulkan_compute_runtime PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include)
        if (TARGET vc1_static)
            target_link_libraries(libvc1_vulkan_compute_runtime PRIVATE vc1_static)
        else()
            target_link_libraries(libvc1_vulkan_compute_runtime PRIVATE vc1)
        endif()
        add_test(NAME libvc1-vulkan-compute-runtime COMMAND libvc1_vulkan_compute_runtime)
        set_tests_properties(libvc1-vulkan-compute-runtime PROPERTIES SKIP_RETURN_CODE 77)
    endif()

    add_executable(libvc1_api_c tests/api_c.c)
    target_include_directories(libvc1_api_c PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    if (TARGET vc1_static)
        target_link_libraries(libvc1_api_c PRIVATE vc1_static)
    else()
        target_link_libraries(libvc1_api_c PRIVATE vc1)
    endif()
    set_target_properties(libvc1_api_c PROPERTIES LINKER_LANGUAGE CXX)
    add_test(NAME libvc1-api-c COMMAND libvc1_api_c)

    add_executable(libvc1_api_cpp tests/api_cpp.cpp)
    target_compile_features(libvc1_api_cpp PRIVATE cxx_std_17)
    target_include_directories(libvc1_api_cpp PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    if (TARGET vc1)
        target_link_libraries(libvc1_api_cpp PRIVATE vc1)
    else()
        target_link_libraries(libvc1_api_cpp PRIVATE vc1_static)
    endif()
    add_test(NAME libvc1-api-cpp COMMAND libvc1_api_cpp)

    add_test(NAME libvc1-matroska-reference-api
        COMMAND ${CMAKE_COMMAND}
            -DSOURCE=${CMAKE_CURRENT_SOURCE_DIR}/src/matroska_muxer.cpp
            -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/check_matroska_reference_api.cmake)

    add_test(NAME libvc1-worker-scheduler
        COMMAND ${CMAKE_COMMAND}
            -DSOURCE=${CMAKE_CURRENT_SOURCE_DIR}/src/encoder_core.cpp
            -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/check_worker_scheduler.cmake)

    add_test(NAME libvc1-source-layout
        COMMAND ${CMAKE_COMMAND}
            -DSRC=${CMAKE_CURRENT_SOURCE_DIR}
            -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/check_source_layout.cmake)

    add_test(NAME libvc1-intra-gop-parallelism
        COMMAND ${CMAKE_COMMAND}
            -DCORE=${CMAKE_CURRENT_SOURCE_DIR}/src/encoder_core.cpp
            -DHEADER=${CMAKE_CURRENT_SOURCE_DIR}/include/libvc1.h
            -DFRONTEND=${CMAKE_CURRENT_SOURCE_DIR}/src/frontend.cpp
            -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/check_intra_gop_parallelism.cmake)

    add_test(NAME libvc1-async-rate-control-retry
        COMMAND ${CMAKE_COMMAND}
            -DSOURCE=${CMAKE_CURRENT_SOURCE_DIR}/src/encoder_core.cpp
            -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/check_async_rate_control_retry.cmake)

    add_test(NAME libvc1-rate-control
        COMMAND ${CMAKE_COMMAND}
            -DSRC=${CMAKE_CURRENT_SOURCE_DIR}
            -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/check_rate_control.cmake)

    add_test(NAME libvc1-removed-rate-control-cli
        COMMAND ${CMAKE_COMMAND}
            -DENCODER=$<TARGET_FILE:vc1enc>
            -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/check_removed_rate_control_cli.cmake)

    add_test(NAME libvc1-group-a-entropy
        COMMAND ${CMAKE_COMMAND}
            -DENCODER_CORE=${CMAKE_CURRENT_SOURCE_DIR}/src/encoder_core.cpp
            -DGROUPA_TABLES=${CMAKE_CURRENT_SOURCE_DIR}/src/vc1_groupa_tables.h
            -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/check_group_a_entropy.cmake)

    add_test(NAME libvc1-group-b
        COMMAND ${CMAKE_COMMAND}
            -DHEADER=${CMAKE_CURRENT_SOURCE_DIR}/include/libvc1.h
            -DCORE=${CMAKE_CURRENT_SOURCE_DIR}/src/encoder_core.cpp
            -DFRONTEND=${CMAKE_CURRENT_SOURCE_DIR}/src/frontend.cpp
            -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/check_group_b.cmake)

    add_test(NAME libvc1-group-c
        COMMAND ${CMAKE_COMMAND}
            -DHEADER=${CMAKE_CURRENT_SOURCE_DIR}/include/libvc1.h
            -DCORE=${CMAKE_CURRENT_SOURCE_DIR}/src/encoder_core.cpp
            -DFRONTEND=${CMAKE_CURRENT_SOURCE_DIR}/src/frontend.cpp
            -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/check_group_c.cmake)

    add_executable(libvc1_group_c_dquant tests/group_c_dquant.cpp)
    target_compile_features(libvc1_group_c_dquant PRIVATE cxx_std_17)
    target_include_directories(libvc1_group_c_dquant PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    if (TARGET vc1_static)
        target_link_libraries(libvc1_group_c_dquant PRIVATE vc1_static)
    else()
        target_link_libraries(libvc1_group_c_dquant PRIVATE vc1)
    endif()
    add_test(NAME libvc1-group-c-dquant-runtime COMMAND libvc1_group_c_dquant)

    add_executable(libvc1_dquant_profiles tests/dquant_profiles.cpp)
    target_compile_features(libvc1_dquant_profiles PRIVATE cxx_std_17)
    target_include_directories(libvc1_dquant_profiles PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
    if (TARGET vc1_static)
        target_link_libraries(libvc1_dquant_profiles PRIVATE vc1_static)
    else()
        target_link_libraries(libvc1_dquant_profiles PRIVATE vc1)
    endif()
    add_test(NAME libvc1-dquant-profiles COMMAND libvc1_dquant_profiles)

    add_executable(libvc1_interlaced_field_refs tests/interlaced_field_refs.cpp)
    target_compile_features(libvc1_interlaced_field_refs PRIVATE cxx_std_17)
    target_include_directories(libvc1_interlaced_field_refs PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
    if (TARGET vc1_static)
        target_link_libraries(libvc1_interlaced_field_refs PRIVATE vc1_static)
    else()
        target_link_libraries(libvc1_interlaced_field_refs PRIVATE vc1)
    endif()
    add_test(NAME libvc1-interlaced-field-reference-runtime COMMAND libvc1_interlaced_field_refs)

    add_executable(libvc1_modern_lookahead tests/modern_lookahead.cpp)
    target_compile_features(libvc1_modern_lookahead PRIVATE cxx_std_17)
    target_include_directories(libvc1_modern_lookahead PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    if (TARGET vc1_static)
        target_link_libraries(libvc1_modern_lookahead PRIVATE vc1_static)
    else()
        target_link_libraries(libvc1_modern_lookahead PRIVATE vc1)
    endif()
    add_test(NAME libvc1-modern-lookahead COMMAND libvc1_modern_lookahead)

    add_executable(libvc1_intra_ac_prediction tests/intra_ac_prediction.cpp)
    target_compile_features(libvc1_intra_ac_prediction PRIVATE cxx_std_17)
    target_include_directories(libvc1_intra_ac_prediction PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    if (TARGET vc1_static)
        target_link_libraries(libvc1_intra_ac_prediction PRIVATE vc1_static)
    else()
        target_link_libraries(libvc1_intra_ac_prediction PRIVATE vc1)
    endif()
    add_test(NAME libvc1-intra-ac-prediction COMMAND libvc1_intra_ac_prediction)

    add_test(NAME libvc1-debug-statistics
        COMMAND ${CMAKE_COMMAND}
            -DHEADER=${CMAKE_CURRENT_SOURCE_DIR}/include/libvc1.h
            -DCORE=${CMAKE_CURRENT_SOURCE_DIR}/src/encoder_core.cpp
            -DFRONTEND=${CMAKE_CURRENT_SOURCE_DIR}/src/frontend.cpp
            -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/check_debug_statistics.cmake)

    find_package(Python3 COMPONENTS Interpreter QUIET)
    if (Python3_Interpreter_FOUND)
        add_test(NAME libvc1-simd-qemu-runner-selftest
            COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/simd_qemu_runner_selftest.py)
    endif()
    if (Python3_Interpreter_FOUND AND TARGET vc1enc)
        add_test(NAME libvc1-debug-statistics-cli
            COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/debug_stats_cli.py $<TARGET_FILE:vc1enc>)
        add_test(NAME libvc1-frame-weight-runtime
            COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/frame_weight_runtime.py $<TARGET_FILE:vc1enc>)
        add_test(NAME libvc1-residual-priority-runtime
            COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/residual_priority_runtime.py $<TARGET_FILE:vc1enc>)
        add_test(NAME libvc1-macroblock-statistics-cli
            COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/macroblock_stats_cli.py $<TARGET_FILE:vc1enc>)
        add_test(NAME libvc1-input-fps-override
            COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/input_fps_override.py $<TARGET_FILE:vc1enc>)
        add_test(NAME libvc1-perceptual-aq-priority
            COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/perceptual_aq_priority.py $<TARGET_FILE:vc1enc>)
        add_test(NAME libvc1-speed-profile-runtime
            COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/speed_profile_runtime.py $<TARGET_FILE:vc1enc>)
        add_test(NAME libvc1-scene-cut-interval-runtime
            COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/scene_cut_interval_runtime.py $<TARGET_FILE:vc1enc>)
        add_test(NAME libvc1-bluray-random-access-runtime
            COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/bluray_random_access_runtime.py $<TARGET_FILE:vc1enc>)
        add_test(NAME libvc1-simd-cli
            COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/simd_cli.py $<TARGET_FILE:vc1enc>)
    endif()

    find_program(LIBVC1_BASH_EXECUTABLE bash)
    if (LIBVC1_BASH_EXECUTABLE)
        add_test(NAME libvc1-smoke-harness
            COMMAND ${LIBVC1_BASH_EXECUTABLE}
                ${CMAKE_CURRENT_SOURCE_DIR}/tests/smoke.sh --self-test-harness)
    endif()

    add_test(NAME libvc1-ffmpeg-cli-compat
        COMMAND ${CMAKE_COMMAND}
            -DSRC=${CMAKE_CURRENT_SOURCE_DIR}
            -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/check_ffmpeg_cli_compat.cmake)

    add_test(NAME libvc1-release-version-consistency
        COMMAND ${CMAKE_COMMAND}
            -DSRC=${CMAKE_CURRENT_SOURCE_DIR}
            -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/check_release_version_consistency.cmake)

    add_test(NAME libvc1-smoke-periodic-i-policy
        COMMAND ${CMAKE_COMMAND}
            -DSRC=${CMAKE_CURRENT_SOURCE_DIR}
            -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/check_smoke_periodic_i.cmake)

    add_test(NAME libvc1-progressive-transform-rdo
        COMMAND ${CMAKE_COMMAND}
            -DCORE=${CMAKE_CURRENT_SOURCE_DIR}/src/encoder_core.cpp
            -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/check_progressive_transform_rdo.cmake)

    add_test(NAME libvc1-overlap-condover
        COMMAND ${CMAKE_COMMAND}
            -DSRC=${CMAKE_CURRENT_SOURCE_DIR}
            -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/check_overlap.cmake)
    find_program(LIBVC1_FFMPEG_EXECUTABLE ffmpeg)
    find_program(LIBVC1_FFPROBE_EXECUTABLE ffprobe)
    if (Python3_Interpreter_FOUND AND LIBVC1_FFMPEG_EXECUTABLE)
        add_test(NAME libvc1-interlaced-field-reference-decode
            COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/interlaced_field_decode_runtime.py
                $<TARGET_FILE:libvc1_interlaced_field_refs> ${LIBVC1_FFMPEG_EXECUTABLE})
    endif()
    if (Python3_Interpreter_FOUND AND LIBVC1_FFMPEG_EXECUTABLE AND TARGET vc1enc)
        add_test(NAME libvc1-raw-output-runtime
            COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/raw_output_runtime.py
                $<TARGET_FILE:vc1enc> ${LIBVC1_FFMPEG_EXECUTABLE})
        add_test(NAME libvc1-progressive-transform-runtime
            COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/progressive_transform_runtime.py
                $<TARGET_FILE:vc1enc> ${LIBVC1_FFMPEG_EXECUTABLE})
        add_test(NAME libvc1-visible-edge-runtime
            COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/visible_edge_runtime.py
                $<TARGET_FILE:vc1enc> ${LIBVC1_FFMPEG_EXECUTABLE})
        add_test(NAME libvc1-odd-dimension-runtime
            COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/odd_dimension_runtime.py
                $<TARGET_FILE:vc1enc> ${LIBVC1_FFMPEG_EXECUTABLE})
        add_test(NAME libvc1-high-dimension-runtime
            COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/high_dimension_runtime.py
                $<TARGET_FILE:vc1enc> ${LIBVC1_FFMPEG_EXECUTABLE})
        add_test(NAME libvc1-bluray-container-runtime
            COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/bluray_container_runtime.py
                $<TARGET_FILE:vc1enc> ${LIBVC1_FFMPEG_EXECUTABLE})
        add_test(NAME libvc1-abr-transform-aq-runtime
            COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/abr_transform_aq_runtime.py
                $<TARGET_FILE:vc1enc> ${LIBVC1_FFMPEG_EXECUTABLE})
        add_test(NAME libvc1-aq-dim-chroma-runtime
            COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/aq_dim_chroma_runtime.py
                $<TARGET_FILE:vc1enc>)
        add_test(NAME libvc1-abr-coding-law-runtime
            COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/abr_coding_law_runtime.py
                $<TARGET_FILE:vc1enc> ${LIBVC1_FFMPEG_EXECUTABLE})
        add_test(NAME libvc1-extended-mv-runtime
            COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/extended_mv_runtime.py
                $<TARGET_FILE:vc1enc> ${LIBVC1_FFMPEG_EXECUTABLE})
        add_test(NAME libvc1-long-range-policy-runtime
            COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/long_range_policy_runtime.py
                $<TARGET_FILE:vc1enc>)
        add_test(NAME libvc1-step2-b-intra-bi-runtime
            COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/step2_b_intra_bi_runtime.py
                $<TARGET_FILE:vc1enc> ${LIBVC1_FFMPEG_EXECUTABLE})
        add_test(NAME libvc1-debug-disable-p-intra-runtime
            COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/debug_disable_p_intra_runtime.py
                $<TARGET_FILE:vc1enc> ${LIBVC1_FFMPEG_EXECUTABLE})
        add_test(NAME libvc1-debug-disable-inter-intra-runtime
            COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/debug_disable_inter_intra_runtime.py
                $<TARGET_FILE:vc1enc> ${LIBVC1_FFMPEG_EXECUTABLE})
        add_test(NAME libvc1-main-inter-intra-dc-center-runtime
            COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/main_inter_intra_dc_center_runtime.py
                $<TARGET_FILE:vc1enc> ${LIBVC1_FFMPEG_EXECUTABLE})
        add_test(NAME libvc1-inter-intra-rate-complexity-runtime
            COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/inter_intra_rate_complexity_runtime.py
                $<TARGET_FILE:vc1enc> ${LIBVC1_FFMPEG_EXECUTABLE})
        add_test(NAME libvc1-step3-dquant-runtime
            COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/step3_dquant_runtime.py
                $<TARGET_FILE:vc1enc> ${LIBVC1_FFMPEG_EXECUTABLE})
        add_test(NAME libvc1-overlap-condover-runtime
            COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/overlap_runtime.py
                $<TARGET_FILE:vc1enc> ${LIBVC1_FFMPEG_EXECUTABLE})
        add_test(NAME libvc1-skipped-picture-runtime
            COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/skipped_picture_runtime.py
                $<TARGET_FILE:vc1enc> ${LIBVC1_FFMPEG_EXECUTABLE})
        if (LIBVC1_FFPROBE_EXECUTABLE)
            add_test(NAME libvc1-interlaced-runtime
                COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/interlaced_runtime.py
                    $<TARGET_FILE:vc1enc> ${LIBVC1_FFMPEG_EXECUTABLE} ${LIBVC1_FFPROBE_EXECUTABLE})
        endif()
    endif()
endif()

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
if (TARGET vc1)
    install(TARGETS vc1 EXPORT libvc1Targets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
endif()
if (TARGET vc1_static)
    install(TARGETS vc1_static EXPORT libvc1Targets ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
endif()
if (TARGET vc1enc)
    install(TARGETS vc1enc RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()

set(LIBVC1_PC_PRIVATE "")
foreach(_lib IN LISTS CMAKE_CXX_IMPLICIT_LINK_LIBRARIES)
    if (_lib MATCHES "^-" OR IS_ABSOLUTE "${_lib}")
        string(APPEND LIBVC1_PC_PRIVATE " ${_lib}")
    else()
        string(APPEND LIBVC1_PC_PRIVATE " -l${_lib}")
    endif()
endforeach()
if (CMAKE_THREAD_LIBS_INIT)
    string(APPEND LIBVC1_PC_PRIVATE " ${CMAKE_THREAD_LIBS_INIT}")
endif()
if (LIBVC1_HAVE_VULKAN)
    # Static consumers need the Vulkan loader in addition to libvc1 itself.
    # Preserve FindVulkan's resolved library path when available so pkg-config
    # works with non-system SDK layouts as well as the normal -lvulkan case.
    if (Vulkan_LIBRARY)
        string(APPEND LIBVC1_PC_PRIVATE " ${Vulkan_LIBRARY}")
    elseif (WIN32)
        string(APPEND LIBVC1_PC_PRIVATE " -lvulkan-1")
    else()
        string(APPEND LIBVC1_PC_PRIVATE " -lvulkan")
    endif()
endif()
if (WIN32)
    set(LIBVC1_STATIC_PC_BASENAME "vc1_static")
else()
    set(LIBVC1_STATIC_PC_BASENAME "vc1")
endif()
configure_file(cmake/libvc1.pc.in libvc1.pc @ONLY)
configure_file(cmake/libvc1-static.pc.in libvc1-static.pc @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/libvc1.pc ${CMAKE_CURRENT_BINARY_DIR}/libvc1-static.pc DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig)
if (LIBVC1_HAVE_VULKAN)
    set(LIBVC1_CONFIG_VULKAN_DEPENDENCY "find_dependency(Vulkan)")
else()
    set(LIBVC1_CONFIG_VULKAN_DEPENDENCY "")
endif()
configure_package_config_file(cmake/libvc1Config.cmake.in ${CMAKE_CURRENT_BINARY_DIR}/libvc1Config.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/libvc1)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/libvc1ConfigVersion.cmake
    VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion)
install(EXPORT libvc1Targets NAMESPACE libvc1:: DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/libvc1)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/libvc1Config.cmake ${CMAKE_CURRENT_BINARY_DIR}/libvc1ConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/libvc1)

add_test(NAME libvc1-quantizer-rc-parity
  COMMAND ${CMAKE_COMMAND} -DSRC=${CMAKE_CURRENT_SOURCE_DIR} -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/check_quantizer_rc_guard.cmake)

add_test(NAME libvc1-progressive-motion-rdo
  COMMAND ${CMAKE_COMMAND} -DCORE=${CMAKE_CURRENT_SOURCE_DIR}/src/encoder_core.cpp -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/check_progressive_motion_rdo.cmake)

add_test(NAME libvc1-umh-extended-mv
  COMMAND ${CMAKE_COMMAND} -DCORE=${CMAKE_CURRENT_SOURCE_DIR}/src/encoder_core.cpp -DFRONTEND=${CMAKE_CURRENT_SOURCE_DIR}/src/frontend.cpp -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/check_umh_extended_mv.cmake)
