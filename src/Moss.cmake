
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Root
set(MOSS_ROOT ${REPO_ROOT}/src) # Switch Moss to src

set(MOSS_ROOT_INCLUDE ${REPO_ROOT}/Moss/include) # Switch Moss to src

set(MOSS_HEADER_FILES
	${MOSS_ROOT_INCLUDE}/*.h
    ${MOSS_ROOT_INCLUDE}/*.inl
)

set(MOSS_SRC_FILES
	${MOSS_ROOT_INCLUDE}/*.cpp
    ${MOSS_ROOT_INCLUDE}/*.c
)

# Platform-specific sources
if(WIN32)
    list(APPEND MOSS_SRC_FILES
        ${MOSS_ROOT}/Platform/windows/win32_platform.h
        ${MOSS_ROOT}/Platform/windows/win32_window.cpp
        ${MOSS_ROOT}/Platform/windows/win32_monitor.cpp
        ${MOSS_ROOT}/Platform/windows/win32_input.cpp
        ${MOSS_ROOT}/Network/win32.c

		${MOSS_ROOT}/Audio/xaudio/win32_audio.h
		${MOSS_ROOT}/Audio/xaudio/win32_audio.cpp
		${MOSS_ROOT}/Audio/xaudio/win32_microphone.cpp
		${MOSS_ROOT}/Audio/xaudio/win32_speaker.cpp
    )
elseif(UNIX AND NOT APPLE) # catches Linux/FreeBSD
	if(USE_WAYLAND)
		list(APPEND MOSS_SRC_FILES 
			${MOSS_ROOT}/Platform/linux/wl_platform.h
			${MOSS_ROOT}/Platform/linux/wl_window.c
			${MOSS_ROOT}/Platform/linux/wl_monitor.c
			${MOSS_ROOT}/Platform/linux/wl_input.c
			)
	else()
		list(APPEND MOSS_SRC_FILES 
			${MOSS_ROOT}/Platform/linux/x11_platform.h
			${MOSS_ROOT}/Platform/linux/x11_window.c
			${MOSS_ROOT}/Platform/linux/x11_monitor.c
			${MOSS_ROOT}/Platform/linux/x11_input.c
			)
    list(APPEND MOSS_SRC_FILES
        ${MOSS_ROOT}/Network/unix.c
    	)
	endif()
elseif(APPLE)
    list(APPEND MOSS_SRC_FILES
        ${MOSS_ROOT}/Platform/apple/mac/cocoa_platform.h
        ${MOSS_ROOT}/Platform/apple/mac/cocoa_monitor.mm
        ${MOSS_ROOT}/Platform/apple/mac/cocoa_window.mm
        ${MOSS_ROOT}/Platform/apple/mac/input.mm
        ${MOSS_ROOT}/Platform/apple/mac/joystick.mm
		${MOSS_ROOT}/Network/unix.c
    	)
endif()
if(USE_OPENGL)
	list(APPEND MOSS_SRC_FILES
	${MOSS_ROOT}/Renderer/GL/glad.h

	${MOSS_ROOT}/Renderer/GL/SurfaceGL.h
	${MOSS_ROOT}/Renderer/GL/SurfaceGL.cpp
	${MOSS_ROOT}/Renderer/GL/TextureGL.h
	${MOSS_ROOT}/Renderer/GL/TextureGL.cpp
	${MOSS_ROOT}/Renderer/GL/ShaderGL.h
	${MOSS_ROOT}/Renderer/GL/ShaderGL.cpp
	${MOSS_ROOT}/Renderer/GL/FontGL.h
	${MOSS_ROOT}/Renderer/GL/FontGL.cpp
	${MOSS_ROOT}/Renderer/GL/MeshGL.h
	#${MOSS_ROOT}/Renderer/GL/PipelineStateGL.h
	#${MOSS_ROOT}/Renderer/GL/PipelineStateGL.cpp
	${MOSS_ROOT}/Renderer/GL/PostProcess.h
	${MOSS_ROOT}/Renderer/GL/PostProcess.cpp

	${MOSS_ROOT}/Renderer/GL/Renderer_GL.h
	${MOSS_ROOT}/Renderer/GL/Renderer_GL.cpp

	#${MOSS_ROOT}/GUI/GL/Moss_gui_gl.cpp
	)
elseif(USE_VULKAN)
	list(APPEND MOSS_SRC_FILES
	#${MOSS_ROOT}/Renderer/VK/SurfaceVK.h
	#${MOSS_ROOT}/Renderer/VK/SurfaceVK.cpp
	#${MOSS_ROOT}/Renderer/VK/TextureVK.h
	#${MOSS_ROOT}/Renderer/VK/TextureVK.cpp
	${MOSS_ROOT}/Renderer/VK/ShaderVK.h
	${MOSS_ROOT}/Renderer/VK/ShaderVK.cpp
	${MOSS_ROOT}/Renderer/VK/PipelineStateVK.h
	${MOSS_ROOT}/Renderer/VK/PipelineStateVK.cpp
	#${MOSS_ROOT}/Renderer/VK/SubViewport.h
	#${MOSS_ROOT}/Renderer/VK/PipelineVK.h
	#${MOSS_ROOT}/Renderer/VK/MeshVK.h

	${MOSS_ROOT}/Renderer/VK/Renderer_VK.h
	${MOSS_ROOT}/Renderer/VK/Renderer_VK.cpp
	${MOSS_ROOT}/Renderer/VK/ConstantBufferVK.h
	${MOSS_ROOT}/Renderer/VK/ConstantBufferVK.cpp
	${MOSS_ROOT}/Moss_Platform.h
	#${MOSS_ROOT}/Renderer/VK/VKTEST.h
	#${MOSS_ROOT}/Renderer/VK/VKTEST.cpp

	${MOSS_ROOT}/Renderer/VK/Moss_Impli_Vulkan.h
	
	)
elseif(USE_DIRECTX12)
	list(APPEND MOSS_SRC_FILES
	${MOSS_ROOT}/Renderer/DX12/Surface.h
	${MOSS_ROOT}/Renderer/DX12/Surface.cpp
	${MOSS_ROOT}/Renderer/DX12/Texture.h
	${MOSS_ROOT}/Renderer/DX12/Texture.cpp
	${MOSS_ROOT}/Renderer/DX12/Shader.h
	${MOSS_ROOT}/Renderer/DX12/Shader.cpp
	${MOSS_ROOT}/Renderer/DX12/SubViewportDX12.h
	${MOSS_ROOT}/Renderer/DX12/PipelineDX12.h
	${MOSS_ROOT}/Renderer/DX12/MeshDX12.h

	${MOSS_ROOT}/Renderer/DX12/Renderer_DX12.h
	${MOSS_ROOT}/Renderer/DX12/Renderer_DX12.cpp
	)
elseif(USE_METAL)
	list(APPEND MOSS_SRC_FILES
	${MOSS_ROOT}/Renderer/MTL/Surface.h
	${MOSS_ROOT}/Renderer/MTL/Surface.cpp
	${MOSS_ROOT}/Renderer/MTL/Texture.h
	${MOSS_ROOT}/Renderer/MTL/Texture.cpp
	${MOSS_ROOT}/Renderer/MTL/Shader.h
	${MOSS_ROOT}/Renderer/MTL/Shader.cpp
	# Font			${MOSS_ROOT}/Renderer/Metal/FontMTL.h
	# Font			${MOSS_ROOT}/Renderer/Metal/FontMTL.cpp
	# SubViewport	${MOSS_ROOT}/Renderer/Metal/SubViewportMTL.h

	# Pipeline		${MOSS_ROOT}/Renderer/Metal/PipelineMTL.h
	#${MOSS_ROOT}/Renderer/Metal/MeshMTL.h

	${MOSS_ROOT}/Renderer/MTL/Renderer_MTL.h
	${MOSS_ROOT}/Renderer/MTL/Renderer_MTL.cpp
	)
endif()
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


target_include_directories(Moss PUBLIC $<BUILD_INTERFACE:${MOSS_ROOT}> $<BUILD_INTERFACE:${MOSS_ROOT_INCLUDE}> $<INSTALL_INTERFACE:include>)

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
	target_precompile_headers(Moss PRIVATE "${MOSS_ROOT}/Moss.h")
else()
	target_precompile_headers(Moss PRIVATE "$<$<NOT:$<CONFIG:ReleaseCoverage>>:${MOSS_ROOT}/Moss.h>")
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
