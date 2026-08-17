
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Root
set(MOSS_ROOT ${REPO_ROOT}/src) # Switch Moss to src

set(MOSS_ROOT_INCLUDE ${REPO_ROOT}/include/Moss) # matches the repo's include/Moss layout

# The compiler's include *search path* needs to be the parent of include/Moss,
# since the codebase includes its own headers as "Moss/Foo.h" (e.g. Moss.h
# itself does #include "Moss/Moss_stdinc.h"). MOSS_ROOT_INCLUDE above stays
# pointed at include/Moss because that's where the actual files live (used
# for globbing and the precompiled header path).
set(MOSS_PUBLIC_INCLUDE_DIR ${REPO_ROOT}/include)

# -----------------------------------------------------------------------------
# File discovery
#
# Everything below uses file(GLOB_RECURSE ... CONFIGURE_DEPENDS ...) instead of
# a hand-maintained file list, so dropping a new .cpp/.h into one of these
# folders is picked up automatically - no editing this script.
#
# CONFIGURE_DEPENDS makes CMake re-check the glob on every build and
# re-configure if it changed. This is well supported by the Ninja and
# Makefiles generators; some IDE/VS generators only re-glob when you
# re-run CMake manually, so if a new file doesn't show up, re-configure once.
#
# Two things glob can't safely replace here:
#  1) Mutually-exclusive backends (Win32 vs X11 vs Wayland vs Cocoa, GL vs
#     Vulkan vs DX12 vs Metal) still need their own if()/elseif() - globbing
#     the whole tree would compile all of them at once and you'd get
#     duplicate-symbol link errors.
#  2) Files that exist on disk but are deliberately excluded from the build
#     (e.g. Renderer/GL/PipelineStateGL.*, which was commented out below) -
#     a glob would silently pull those back in, so they're filtered out
#     explicitly after the glob. If you want a file gone for good, it's
#     cleaner to delete it / rename it out of the tree than to rely on this
#     exclude list.
# -----------------------------------------------------------------------------

# Headers - not compiled (add_library below only takes MOSS_SRC_FILES), kept
# around so they show up as part of the target in IDEs / can be installed.
file(GLOB_RECURSE MOSS_HEADER_FILES CONFIGURE_DEPENDS
	"${MOSS_ROOT_INCLUDE}/*.h"
	"${MOSS_ROOT_INCLUDE}/*.inl"
)

# ---- Core sources: platform- and backend-independent ----
file(GLOB_RECURSE MOSS_SRC_FILES CONFIGURE_DEPENDS
	"${MOSS_ROOT}/Physics/*.cpp"
	"${MOSS_ROOT}/TriangleSplitter/*.cpp"
	"${MOSS_ROOT}/XR/*.cpp"
)

# enet networking core (address/host/peer/protocol/etc.) - third-party-ish,
# stable, and platform.c backends (win32.c/unix.c) live in the same folder,
# so this is kept as an explicit list rather than globbed to avoid pulling
# the wrong platform backend in.
list(APPEND MOSS_SRC_FILES
	${MOSS_ROOT}/Network/address.cpp
	${MOSS_ROOT}/Network/callbacks.cpp
	${MOSS_ROOT}/Network/host.cpp
	${MOSS_ROOT}/Network/list.cpp
	${MOSS_ROOT}/Network/packet.cpp
	${MOSS_ROOT}/Network/peer.cpp
	${MOSS_ROOT}/Network/protocol.cpp
)

# Generic audio backend (non-platform-specific). Non-recursive glob so it
# doesn't reach into Audio/xaudio, Audio/linux, Audio/macos.
file(GLOB MOSS_AUDIO_GENERIC_FILES CONFIGURE_DEPENDS
	"${MOSS_ROOT}/Audio/*.cpp"
)
list(APPEND MOSS_SRC_FILES ${MOSS_AUDIO_GENERIC_FILES})

# Files that exist on disk but should stay out of the build (mirrors what
# was previously commented out). Add to this list instead of re-commenting
# entries in a hand-written file list.
set(MOSS_EXCLUDED_SRC_FILES
	"${MOSS_ROOT}/Renderer/GL/PipelineStateGL.h"
	"${MOSS_ROOT}/Renderer/GL/PipelineStateGL.cpp"
)

# Platform-specific sources
if(WIN32)
	file(GLOB_RECURSE MOSS_PLATFORM_FILES CONFIGURE_DEPENDS
		"${MOSS_ROOT}/Platform/windows/*.h"
		"${MOSS_ROOT}/Platform/windows/*.cpp"
	)
	file(GLOB_RECURSE MOSS_AUDIO_PLATFORM_FILES CONFIGURE_DEPENDS
		"${MOSS_ROOT}/Audio/xaudio/*.h"
		"${MOSS_ROOT}/Audio/xaudio/*.cpp"
	)
	list(APPEND MOSS_SRC_FILES
		${MOSS_PLATFORM_FILES}
		${MOSS_AUDIO_PLATFORM_FILES}
		${MOSS_ROOT}/Network/win32.c
	)
elseif(UNIX AND NOT APPLE) # catches Linux/FreeBSD
	if(USE_WAYLAND)
		file(GLOB_RECURSE MOSS_PLATFORM_FILES CONFIGURE_DEPENDS
			"${MOSS_ROOT}/Platform/linux/wl_*.h"
			"${MOSS_ROOT}/Platform/linux/wl_*.c"
		)
	else()
		file(GLOB_RECURSE MOSS_PLATFORM_FILES CONFIGURE_DEPENDS
			"${MOSS_ROOT}/Platform/linux/x11_*.h"
			"${MOSS_ROOT}/Platform/linux/x11_*.c"
		)
	endif()

	file(GLOB_RECURSE MOSS_AUDIO_PLATFORM_FILES CONFIGURE_DEPENDS
		"${MOSS_ROOT}/Audio/linux/*.h"
		"${MOSS_ROOT}/Audio/linux/*.cpp"
	)

	list(APPEND MOSS_SRC_FILES
		${MOSS_PLATFORM_FILES}
		${MOSS_AUDIO_PLATFORM_FILES}
		${MOSS_ROOT}/Network/unix.c
	)
	# NOTE: previously Network/unix.c was only appended in the X11 branch,
	# so a Wayland build never got the enet unix backend. That looked like
	# an unintentional gap, so it's now appended for both - revert to the
	# old conditional if that exclusion was actually intentional.

elseif(APPLE)
	file(GLOB_RECURSE MOSS_PLATFORM_FILES CONFIGURE_DEPENDS
		"${MOSS_ROOT}/Platform/apple/mac/*.h"
		"${MOSS_ROOT}/Platform/apple/mac/*.mm"
	)
	file(GLOB_RECURSE MOSS_AUDIO_PLATFORM_FILES CONFIGURE_DEPENDS
		"${MOSS_ROOT}/audio/macos/*.h"
		"${MOSS_ROOT}/audio/macos/*.cpp"
	)
	list(APPEND MOSS_SRC_FILES
		${MOSS_PLATFORM_FILES}
		${MOSS_AUDIO_PLATFORM_FILES}
		${MOSS_ROOT}/Network/unix.c
	)
endif()

# Renderer backend sources (mutually exclusive - only one of these is built)
if(USE_OPENGL)
	file(GLOB_RECURSE MOSS_RENDERER_FILES CONFIGURE_DEPENDS
		"${MOSS_ROOT}/Renderer/GL/*.h"
		"${MOSS_ROOT}/Renderer/GL/*.cpp"
	)
	list(APPEND MOSS_SRC_FILES ${MOSS_RENDERER_FILES})
elseif(USE_VULKAN)
	file(GLOB_RECURSE MOSS_RENDERER_FILES CONFIGURE_DEPENDS
		"${MOSS_ROOT}/Renderer/VK/*.h"
		"${MOSS_ROOT}/Renderer/VK/*.cpp"
	)
	list(APPEND MOSS_SRC_FILES ${MOSS_RENDERER_FILES} ${MOSS_ROOT}/Moss_Platform.h)
elseif(USE_DIRECTX12)
	file(GLOB_RECURSE MOSS_RENDERER_FILES CONFIGURE_DEPENDS
		"${MOSS_ROOT}/Renderer/DX12/*.h"
		"${MOSS_ROOT}/Renderer/DX12/*.cpp"
	)
	list(APPEND MOSS_SRC_FILES ${MOSS_RENDERER_FILES})
elseif(USE_METAL)
	file(GLOB_RECURSE MOSS_RENDERER_FILES CONFIGURE_DEPENDS
		"${MOSS_ROOT}/Renderer/MTL/*.h"
		"${MOSS_ROOT}/Renderer/MTL/*.cpp"
	)
	list(APPEND MOSS_SRC_FILES ${MOSS_RENDERER_FILES})
endif()

# Drop anything that's explicitly excluded from the build (see
# MOSS_EXCLUDED_SRC_FILES above), and de-duplicate in case a glob pattern
# overlapped with the explicit Network/Audio lists.
list(REMOVE_ITEM MOSS_SRC_FILES ${MOSS_EXCLUDED_SRC_FILES})
list(REMOVE_DUPLICATES MOSS_SRC_FILES)

# add_library(Moss ${MOSS_HEADER_FILES} ${MOSS_SRC_FILES})
add_library(Moss ${MOSS_SRC_FILES})
add_library(Moss::Moss ALIAS Moss)

if (BUILD_SHARED_LIBS)
	# Set default visibility to hidden
	set(CMAKE_CXX_VISIBILITY_PRESET hidden)
	# set_target_properties(Moss PROPERTIESCXX_VISIBILITY_PRESET hidden VISIBILITY_INLINES_HIDDEN YES)

	if (GENERATE_DEBUG_SYMBOLS)
		if (MSVC)
			# MSVC specific option to enable PDB generation
			set(CMAKE_SHARED_LINKER_FLAGS_RELEASE "${CMAKE_SHARED_LINKER_FLAGS_RELEASE} /DEBUG:FASTLINK")
		else()
			# Clang/GCC option to enable debug symbol generation
			set(CMAKE_SHARED_LINKER_FLAGS_RELEASE "${CMAKE_SHARED_LINKER_FLAGS_RELEASE} -g")
		endif()
	endif()

	# Set linker flags for other build types to be the same as release
	set(CMAKE_SHARED_LINKER_FLAGS_RELEASEASAN "${CMAKE_SHARED_LINKER_FLAGS_RELEASE}")
	set(CMAKE_SHARED_LINKER_FLAGS_RELEASEUBSAN "${CMAKE_SHARED_LINKER_FLAGS_RELEASE}")
	set(CMAKE_SHARED_LINKER_FLAGS_RELEASETSAN "${CMAKE_SHARED_LINKER_FLAGS_RELEASE}")
	set(CMAKE_SHARED_LINKER_FLAGS_RELEASECOVERAGE "${CMAKE_SHARED_LINKER_FLAGS_RELEASE}")
	set(CMAKE_SHARED_LINKER_FLAGS_DISTRIBUTION "${CMAKE_SHARED_LINKER_FLAGS_RELEASE}")

	# Public define to instruct user code to import Jolt symbols (rather than use static linking)
	target_compile_definitions(Moss PUBLIC MOSS_SHARED_LIBRARY)

	# Private define to instruct the library to export symbols for shared linking
	target_compile_definitions(Moss PRIVATE MOSS_BUILD_SHARED_LIBRARY)
endif()


target_include_directories(Moss PUBLIC $<BUILD_INTERFACE:${MOSS_ROOT}> $<BUILD_INTERFACE:${MOSS_PUBLIC_INCLUDE_DIR}> $<INSTALL_INTERFACE:include>)

if(WIN32)
    target_link_libraries(Moss PRIVATE user32 gdi32)
elseif(APPLE)
    find_library(COCOA_LIBRARY Cocoa)
    target_link_libraries(Moss PRIVATE ${COCOA_LIBRARY})
elseif(UNIX)
    target_link_libraries(Moss PRIVATE X11 GL pthread)
endif()

if(USE_OPENGL OR USE_OPENGLES)
    if(USE_OPENGLES)
        target_compile_definitions(Moss PUBLIC MOSS_USE_OPENGLES)
    else()
        target_compile_definitions(Moss PUBLIC MOSS_USE_OPENGL)
    endif()
    if(WIN32)
        if(USE_OPENGLES)
            message(FATAL_ERROR "OpenGL ES is not supported natively on Windows without ANGLE or similar")
        else()
            target_link_libraries(Moss PRIVATE opengl32)
        endif()
    elseif(APPLE)
        if(USE_OPENGLES)
            message(FATAL_ERROR "OpenGL ES is not directly supported on macOS. Use Metal or ANGLE.")
        else()
            find_library(OpenGLFramework OpenGL)
            target_link_libraries(Moss PRIVATE ${OpenGLFramework})
        endif()
    elseif(UNIX)
        find_package(PkgConfig REQUIRED)
        if(USE_WAYLAND)
            pkg_check_modules(WAYLAND_CLIENT REQUIRED wayland-client)
            pkg_check_modules(WAYLAND_EGL REQUIRED wayland-egl)
            pkg_check_modules(EGL REQUIRED egl)
            target_include_directories(Moss PRIVATE
                ${WAYLAND_CLIENT_INCLUDE_DIRS}
                ${WAYLAND_EGL_INCLUDE_DIRS}
                ${EGL_INCLUDE_DIRS}
            )
            target_link_libraries(Moss PRIVATE
                ${WAYLAND_CLIENT_LIBRARIES}
                ${WAYLAND_EGL_LIBRARIES}
                ${EGL_LIBRARIES}
            )
        else()
            find_package(X11 REQUIRED)
            target_link_libraries(Moss PRIVATE ${X11_LIBRARIES})
        endif()
        if(USE_OPENGLES)
            # GLES2 or GLES3
            pkg_check_modules(GLES REQUIRED glesv2)
            target_include_directories(Moss PRIVATE ${GLES_INCLUDE_DIRS})
            target_link_libraries(Moss PRIVATE ${GLES_LIBRARIES})
        else()
            find_package(OpenGL REQUIRED)
            target_link_libraries(Moss PRIVATE OpenGL::GL)
        endif()
        target_link_libraries(Moss PRIVATE Threads::Threads)
    endif()
	if(ANDROID)
	    target_compile_definitions(Moss PUBLIC MOSS_USE_OPENGLES)
	    target_link_libraries(Moss PRIVATE GLESv3 EGL)
	endif()
endif()

if(USE_VULKAN)
	find_package(Vulkan REQUIRED)
	target_compile_definitions(Moss PUBLIC MOSS_USE_VULKAN)
	target_include_directories(Moss PUBLIC ${Vulkan_INCLUDE_DIRS})
	target_link_libraries(Moss PRIVATE ${Vulkan_LIBRARIES})

    if(UNIX)
        if(USE_WAYLAND)
            find_package(PkgConfig REQUIRED)
            pkg_check_modules(WAYLAND_CLIENT wayland-client)
            pkg_check_modules(WAYLAND_EGL wayland-egl)
            target_link_libraries(Moss PRIVATE
                ${WAYLAND_CLIENT_LIBRARIES}
                ${WAYLAND_EGL_LIBRARIES}
                pthread
            )
        else()
            find_package(X11 REQUIRED)
            target_link_libraries(Moss PRIVATE ${X11_LIBRARIES} pthread)
        endif()
    elseif(WIN32)
        # Vulkan loader on Windows is linked by Vulkan_LIBRARIES automatically
        target_link_libraries(Moss PRIVATE user32 gdi32)
    endif()
endif()

if(USE_DIRECTX12 AND WIN32)
    target_link_libraries(Moss PRIVATE user32 gdi32)
endif()

if(USE_METAL AND APPLE)
	enable_language(OBJC)
    enable_language(OBJCXX)
	
    target_compile_definitions(Moss PUBLIC MOSS_USE_METAL)
	
    find_library(METAL_FRAMEWORK Metal REQUIRED)
    find_library(METALKIT_FRAMEWORK MetalKit REQUIRED)
    find_library(QUARTZCORE_FRAMEWORK QuartzCore REQUIRED)
    find_library(FOUNDATION_FRAMEWORK Foundation REQUIRED)
    target_link_libraries(Moss PRIVATE
        ${METAL_FRAMEWORK}
        ${METALKIT_FRAMEWORK}
        ${QUARTZCORE_FRAMEWORK}
        ${FOUNDATION_FRAMEWORK}
    )

    if(CMAKE_OSX_SYSROOT MATCHES "macosx")
        find_library(APPKIT_FRAMEWORK AppKit REQUIRED)
        target_link_libraries(Moss PRIVATE ${APPKIT_FRAMEWORK})
    elseif(CMAKE_OSX_SYSROOT MATCHES "iphoneos" OR
           CMAKE_OSX_SYSROOT MATCHES "iphonesimulator" OR
           CMAKE_OSX_SYSROOT MATCHES "appletvos" OR
           CMAKE_OSX_SYSROOT MATCHES "appletvsimulator")

        find_library(UIKIT_FRAMEWORK UIKit REQUIRED)
        target_link_libraries(Moss PRIVATE ${UIKIT_FRAMEWORK})
    endif()

	target_compile_options(Moss PRIVATE "$<$<COMPILE_LANGUAGE:OBJCXX>:-fobjc-arc>")
endif()

if (CMAKE_GENERATOR STREQUAL "Ninja Multi-Config" AND MSVC)
	# The Ninja Multi-Config generator errors out when selectively disabling precompiled headers for certain configurations.
	# See: https://github.com/jrouwe/JoltPhysics/issues/1211
	target_precompile_headers(Moss PRIVATE "${MOSS_ROOT_INCLUDE}/Moss.h")
else()
	target_precompile_headers(Moss PRIVATE "$<$<NOT:$<CONFIG:ReleaseCoverage>>:${MOSS_ROOT_INCLUDE}/Moss.h>")
endif()

# Set the debug/non-debug build flags
target_compile_definitions(Moss PUBLIC "$<$<CONFIG:Debug>:_DEBUG>")
target_compile_definitions(Moss PUBLIC "$<$<CONFIG:Release,Distribution,ReleaseASAN,ReleaseUBSAN,ReleaseTSAN,ReleaseCoverage>:NDEBUG>")

# ASAN and TSAN should use the default allocators
target_compile_definitions(Moss PUBLIC "$<$<CONFIG:ReleaseASAN,ReleaseTSAN>:MOSS_DISABLE_TEMP_ALLOCATOR;MOSS_DISABLE_CUSTOM_ALLOCATOR>")

# Setting floating point exceptions
if (FLOATING_POINT_EXCEPTIONS_ENABLED AND "${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
	target_compile_definitions(Moss PUBLIC "$<$<CONFIG:Debug,Release>:MOSS_FLOATING_POINT_EXCEPTIONS_ENABLED>")
endif()


# Setting the disable custom allocator flag
if (DISABLE_CUSTOM_ALLOCATOR)
	target_compile_definitions(Moss PUBLIC MOSS_DISABLE_CUSTOM_ALLOCATOR)
endif()

# Setting enable asserts flag
if (ENABLE_DEBUG)
	target_compile_definitions(Moss PUBLIC MOSS_DEBUG)
endif()

# Setting double precision flag
if (DOUBLE_PRECISION)
	target_compile_definitions(Moss PUBLIC MOSS_DOUBLE_PRECISION)
endif()

# Setting to attempt cross platform determinism
if (CROSS_PLATFORM_DETERMINISTIC)
	target_compile_definitions(Moss PUBLIC MOSS_CROSS_PLATFORM_DETERMINISTIC)
endif()

# Setting to determine number of bits in ObjectLayer
if (OBJECT_LAYER_BITS)
	target_compile_definitions(Moss PUBLIC MOSS_OBJECT_LAYER_BITS=${OBJECT_LAYER_BITS})
endif()

if (USE_STD_VECTOR)
	target_compile_definitions(Moss PUBLIC MOSS_USE_STD_VECTOR)
endif()

# Setting to periodically trace broadphase stats to help determine if the broadphase layer configuration is optimal
if (TRACK_BROADPHASE_STATS)
	target_compile_definitions(Moss PUBLIC MOSS_TRACK_BROADPHASE_STATS)
endif()

# Setting to periodically trace narrowphase stats to help determine which collision queries could be optimized
if (TRACK_NARROWPHASE_STATS)
	target_compile_definitions(Moss PUBLIC MOSS_TRACK_NARROWPHASE_STATS)
endif()

# Enable the debug renderer
if (DEBUG_RENDERER_IN_DISTRIBUTION)
	target_compile_definitions(Moss PUBLIC "MOSS_DEBUG_RENDERER")
elseif (DEBUG_RENDERER_IN_DEBUG_AND_RELEASE)
	target_compile_definitions(Moss PUBLIC "$<$<CONFIG:Debug,Release,ReleaseASAN,ReleaseUBSAN,ReleaseTSAN>:MOSS_DEBUG_RENDERER>")
endif()

# Enable the profiler
if (MOSS_USE_EXTERNAL_PROFILE)
	set(MOSS_PROFILE_DEFINE MOSS_EXTERNAL_PROFILE)
else()
	set(MOSS_PROFILE_DEFINE MOSSH_PROFILE_ENABLED)
endif()
if (PROFILER_IN_DISTRIBUTION)
	target_compile_definitions(Moss PUBLIC "${MOSS_PROFILE_DEFINE}")
elseif (PROFILER_IN_DEBUG_AND_RELEASE)
	target_compile_definitions(Moss PUBLIC "$<$<CONFIG:Debug,Release,ReleaseASAN,ReleaseUBSAN,ReleaseTSAN>:${MOSS_PROFILE_DEFINE}>")
endif()

# Compile the ObjectStream class and RTTI attribute information
if (ENABLE_OBJECT_STREAM)
	target_compile_definitions(Moss PUBLIC MOSS_OBJECT_STREAM)
endif()

# Emit the instruction set definitions to ensure that child projects use the same settings even if they override the used instruction sets (a mismatch causes link errors)
function(EMIT_X86_INSTRUCTION_SET_DEFINITIONS)
	if (USE_AVX512)
		target_compile_definitions(Moss PUBLIC MOSS_SIMD_AVX512)
	endif()
	if (USE_AVX2)
		target_compile_definitions(Moss PUBLIC MOSS_SIMD_AVX2)
	endif()
	if (USE_AVX)
		target_compile_definitions(Moss PUBLIC MOSS_SIMD_AVX)
	endif()
	if (USE_SSE4_1)
		target_compile_definitions(Moss PUBLIC MOSS_SIMD_SSE4_1)
	endif()
	if (USE_SSE4_2)
		target_compile_definitions(Moss PUBLIC MOSS_SIMD_SSE4_2)
	endif()
	if (USE_LZCNT)
		target_compile_definitions(Moss PUBLIC MOSS_SIMD_LZCNT)
	endif()
	if (USE_TZCNT)
		target_compile_definitions(Moss PUBLIC MOSS_SIMD_TZCNT)
	endif()
	if (USE_F16C)
		target_compile_definitions(Moss PUBLIC MOSS_SIMD_F16C)
	endif()
	if (USE_FMADD AND NOT CROSS_PLATFORM_DETERMINISTIC)
		target_compile_definitions(Moss PUBLIC MOSS_USE_FMADD)
	endif()
endfunction()

# Add the compiler commandline flags to select the right instruction sets
if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
	if ("${CMAKE_VS_PLATFORM_NAME}" STREQUAL "x86" OR "${CMAKE_VS_PLATFORM_NAME}" STREQUAL "x64")
		if (USE_AVX512)
			target_compile_options(Moss PUBLIC /arch:AVX512)
		elseif (USE_AVX2)
			target_compile_options(Moss PUBLIC /arch:AVX2)
		elseif (USE_AVX)
			target_compile_options(Moss PUBLIC /arch:AVX)
		endif()
		EMIT_X86_INSTRUCTION_SET_DEFINITIONS()
	endif()
else()
	if (XCODE)
		# XCode builds for multiple architectures, we can't set global flags
	elseif (CROSS_COMPILE_ARM OR CMAKE_OSX_ARCHITECTURES MATCHES "arm64" OR "${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "aarch64")
		# ARM64 uses no special commandline flags
	elseif (EMSCRIPTEN)
		if (USE_WASM_SIMD)
			# Jolt currently doesn't implement the WASM specific SIMD intrinsics so uses the SSE 4.2 intrinsics
			# See: https://emscripten.org/docs/porting/simd.html#webassembly-simd-intrinsics
			# Note that this does not require the browser to actually support SSE 4.2 it merely means that it can translate those instructions to WASM SIMD instructions
			target_compile_options(Moss PUBLIC -msimd128 -msse4.2)
		endif()
		if (MOSS_USE_WASM64)
			target_compile_options(Moss PUBLIC -sMEMORY64)
			target_link_options(Moss PUBLIC -sMEMORY64)
		endif()
	elseif ("${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "x86_64" OR "${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "AMD64" OR "${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "x86" OR "${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "i386")
		# x86 and x86_64
		# On 32-bit builds we need to default to using SSE instructions, the x87 FPU instructions have higher intermediate precision
		# which will cause problems in the collision detection code (the effect is similar to leaving FMA on, search for
		# MOSS_PRECISE_MATH_ON for the locations where this is a problem).

		if (USE_AVX512)
			target_compile_options(Moss PUBLIC -mavx512f -mavx512vl -mavx512dq -mavx2 -mbmi -mpopcnt -mlzcnt -mf16c)
		elseif (USE_AVX2)
			target_compile_options(Moss PUBLIC -mavx2 -mbmi -mpopcnt -mlzcnt -mf16c)
		elseif (USE_AVX)
			target_compile_options(Moss PUBLIC -mavx -mpopcnt)
		elseif (USE_SSE4_2)
			target_compile_options(Moss PUBLIC -msse4.2 -mpopcnt)
		elseif (USE_SSE4_1)
			target_compile_options(Moss PUBLIC -msse4.1)
		else()
			target_compile_options(Moss PUBLIC -msse2)
		endif()
		if (USE_LZCNT)
			target_compile_options(Moss PUBLIC -mlzcnt)
		endif()
		if (USE_TZCNT)
			target_compile_options(Moss PUBLIC -mbmi)
		endif()
		if (USE_F16C)
			target_compile_options(Moss PUBLIC -mf16c)
		endif()
		if (USE_FMADD AND NOT CROSS_PLATFORM_DETERMINISTIC)
			target_compile_options(Moss PUBLIC -mfma)
		endif()

		if (NOT MSVC)
			target_compile_options(Moss PUBLIC -mfpmath=sse)
		endif()

		EMIT_X86_INSTRUCTION_SET_DEFINITIONS()
	endif()
endif()

# On Unix flavors we need the pthread library
if (NOT ("${CMAKE_SYSTEM_NAME}" STREQUAL "Windows") AND NOT EMSCRIPTEN)
	target_compile_options(Moss PUBLIC -pthread)
	target_link_options(Moss PUBLIC -pthread)
endif()

if (EMSCRIPTEN)
	# We need more than the default 64KB stack and 16MB memory
	# In your application, specify at least -sSTACK_SIZE=1048576 -sINITIAL_MEMORY=134217728
	# Also disable warning: running limited binaryen optimizations because DWARF info requested (or indirectly required)
	target_link_options(Moss PUBLIC -Wno-limited-postlink-optimizations)
endif()
