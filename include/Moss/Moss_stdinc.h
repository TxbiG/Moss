//                        MIT License
//
//                  Copyright (c) 2026 Toby
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

/*!
 * @file Moss_stdinc.h
 * @brief Standard include header for the Moss Framework.
 *
 * Provides:
 * - Platform and compiler abstraction.
 * - Standardized type definitions and compile-time configuration.
 * - Utility macros, assertions, and inline helpers.
 * - Integration foundation for all other Moss subsystems.
 *
 * ---
 *
 * ### Overview:
 * `Moss_stdinc.h` acts as the foundational include for the entire Moss Framework.
 * It ensures consistent definitions across compilers, platforms.
 *
 * ---
 *
 * ### Core Responsibilities:
 * - **Platform Detection**
 *   - Identifies OS and hardware targets.
 *   - Defines portable symbols for Windows, Linux, macOS, Android, iOS, and WebAssembly.
 *
 * - **Compiler Detection**
 *   - Supports MSVC, GCC, Clang, and AppleClang.
 *   - Enables warning suppression and compiler-specific optimization attributes.
 *
 * - **Type Normalization**
 *   - Defines fixed-size integer and float types (uint8, int16, float32, etc).
 *   - Establishes Moss-specific aliases for cross-language consistency.
 *
 * - **Macros & Attributes**
 *   - Inline, constexpr, restrict, likely/unlikely, alignment, and export/import macros.
 *   - Simplifies platform-specific linkage (DLLs, shared libraries).
 *
 * - **Error Handling & Debugging**
 *   - Unified `MOSS_ASSERT`, `MOSS_LOG`, and panic/debug hooks.
 *   - Optional integration with custom logging systems or crash reporters.
 *
 * - **Symbol Management**
 *   - Ensures consistent namespacing for C and C++.
 *   - Enables `MOSS_SUPRESS_WARNING_BEGIN` / `MOSS_SUPRESS_WARNING_END` macros.
 *
 * ---
 *
 * ### Platform Defines
 * ```
 * MOSS_PLATFORM_WINDOWS - Desktop
 * MOSS_PLATFORM_WINDOWS_UWP - Universal Windows Platform
 * MOSS_WINAPI_FAMILY_PHONE - Microsoft Phones
 * MOSS_PLATFORM_XBOXONE
 * MOSS_PLATFORM_XBOXSERIES
 * MOSS_PLATFORM_LINUX - X11, Wayland supported
 * MOSS_PLATFORM_MACOS - Metal and OpenGL supported
 * MOSS_PLATFORM_IOS - Metal-based rendering only
 * MOSS_PLATFORM_ANDROID - GLES / Vulkan support
 * MOSS_PLATFORM_3DS
 * MOSS_PLATFORM_AIX
 * MOSS_PLATFORM_BSDI
 * MOSS_PLATFORM_CYGWIN
 * MOSS_PLATFORM_EMSCRIPTEN - WebAssembly / WebGPU support
 * MOSS_PLATFORM_FREEBSD
 * MOSS_PLATFORM_GDK
 * MOSS_PLATFORM_HAIKU
 * MOSS_PLATFORM_HPUX
 * MOSS_PLATFORM_HURD
 * MOSS_PLATFORM_IRIX
 * MOSS_PLATFORM_NETBSD
 * MOSS_PLATFORM_NGAGE
 * MOSS_PLATFORM_OPENBSD
 * MOSS_PLATFORM_OS2
 * MOSS_PLATFORM_OSF
 * MOSS_PLATFORM_PS2
 * MOSS_PLATFORM_PSP
 * MOSS_PLATFORM_QNXNTO
 * MOSS_PLATFORM_RISCOS
 * MOSS_PLATFORM_SOLARIS
 * MOSS_PLATFORM_UNIX
 * MOSS_PLATFORM_VISIONOS
 * MOSS_PLATFORM_VITA
 * MOSS_PLATFORM_WINGDK
 * ```
 *
 * ---
 *
 * ### Compiler Defines:
 * | Compiler | Define | Notes |
 * |-----------|---------|-------|
 * | MSVC      | `MOSS_COMPILER_MSVC` | Supports MSVC-specific intrinsics |
 * | GCC       | `MOSS_COMPILER_GCC` | Enables GCC attributes |
 * | Clang     | `MOSS_COMPILER_CLANG` | Unified with AppleClang |
 *
 * ---
 *
 * ### Common Macros:
 * ```cpp
 * #define MOSS_INLINE       inline __attribute__((always_inline))
 * #define MOSS_NOINLINE     __attribute__((noinline))
 * #define MOSS_ALIGN(x)     __attribute__((aligned(x)))
 * #define MOSS_LIKELY(x)    __builtin_expect((x), 1)
 * #define MOSS_UNLIKELY(x)  __builtin_expect((x), 0)
 *
 * #define MOSS_EXPORT       extern "C" __declspec(dllexport)
 * #define MOSS_EXPORT       extern "C" __declspec(dllimport)
 * ```
 *
 * ---
 *
 * ### Utility Macros:
 * - `MOSS_ARRAY_SIZE(arr)` — Returns the number of elements in an array.
 * - `MOSS_UNUSED(x)` — Prevents unused variable warnings.
 * - `MOSS_BIT(x)` — Shifts bitmask `(1 << x)`.
 * - `MOSS_OFFSET_OF(type, member)` — Returns byte offset of struct member.
 *
 * ---
 *
 * Ensures version compatibility between Moss Framework components and bindings.
 *
 * ---
 *
 * This module acts as the os foundation upon which other systems
 * (Renderer, Audio, Platform, Physics, etc.) are initialized and operated.
 */

#ifndef MOSS_STDINC_H
#define MOSS_STDINC_H

// Determine platform
#if defined(MOSS_PLATFORM_BLUE)
	// Correct define already defined, this overrides everything else
#elif defined(__EMSCRIPTEN__)
	#define MOSS_PLATFORM_WASM
#elif defined(_WIN32) || defined(_WIN64)
	#include <winapifamily.h>

	// Windows
	#define MOSS_PLATFORM_WINDOWS

	#if (WINAPI_FAMILY == WINAPI_FAMILY_APP)
        #define MOSS_PLATFORM_WINDOWS_UWP
    #endif

	/* Xbox (GDK covers Xbox One + Series) */
    #if defined(_GAMING_XBOX)
        #define MOSS_PLATFORM_GDK
        #if defined(_GAMING_XBOX_X)
            #define MOSS_PLATFORM_XBOXSERIES
        #else
            #define MOSS_PLATFORM_XBOXONE
        #endif
    #endif
#elif defined(__APPLE__)
	#elif defined(__APPLE__)
    #include <TargetConditionals.h>
    #define MOSS_PLATFORM_APPLE

    #if TARGET_OS_OSX
        #define MOSS_PLATFORM_MACOS
    #elif TARGET_OS_IPHONE
        #define MOSS_PLATFORM_IOS
    #elif TARGET_OS_TV
        #define MOSS_PLATFORM_TVOS
    #elif TARGET_OS_VISION
        #define MOSS_PLATFORM_VISIONOS
    #endif
#elif defined(ANDROID) || defined(__ANDROID__) 	// Android is linux too, so that's why we check it first
	#define MOSS_PLATFORM_ANDROID
	// Add Android TV
#elif defined(linux) || defined(__linux) || defined(__linux__)
	// Linux
    #define MOSS_PLATFORM_LINUX
#elif defined(__FreeBSD__)
    #define MOSS_PLATFORM_FREEBSD
#elif defined(__OpenBSD__)
    #define MOSS_PLATFORM_OPENBSD
#elif defined(__NetBSD__)
    #define MOSS_PLATFORM_NETBSD
#elif defined(__bsdi__)
    #define MOSS_PLATFORM_BSDI
#elif defined(__sun)
    #define MOSS_PLATFORM_SOLARIS
#elif defined(_AIX)
    #define MOSS_PLATFORM_AIX
#elif defined(__hpux)
    #define MOSS_PLATFORM_HPUX
#elif defined(__HAIKU__)
    #define MOSS_PLATFORM_HAIKU
#elif defined(__GNU__)
    #define MOSS_PLATFORM_HURD
#elif defined(__sgi)
    #define MOSS_PLATFORM_IRIX
#elif defined(__QNX__)
    #define MOSS_PLATFORM_QNXNTO
#elif defined(__riscos__)
    #define MOSS_PLATFORM_RISCOS
#elif defined(__osf__)
    #define MOSS_PLATFORM_OSF
#elif defined(__unix__) || defined(__unix)
    #define MOSS_PLATFORM_UNIX
#elif defined(__3DS__)
    #define MOSS_PLATFORM_3DS
#elif defined(__NGAGE__)
    #define MOSS_PLATFORM_NGAGE
#elif defined(__psp__)
    #define MOSS_PLATFORM_PSP
#elif defined(__vita__)
    #define MOSS_PLATFORM_VITA
#elif defined(__ps2__)
    #define MOSS_PLATFORM_PS2
#elif defined(__CYGWIN__)
    #define MOSS_PLATFORM_CYGWIN
#elif defined(__OS2__)
    #define MOSS_PLATFORM_OS2
#elif defined(__EMSCRIPTEN__)
	#define MOSS_PLATFORM_WASM
#endif

// Platform helper macros
#ifdef MOSS_PLATFORM_ANDROID
	#define MOSS_IF_NOT_ANDROID(x)
#else
	#define MOSS_IF_NOT_ANDROID(x) x
#endif

// Determine compiler
#if defined(__clang__)
	#define MOSS_COMPILER_CLANG
#elif defined(__GNUC__)
	#define MOSS_COMPILER_GCC
#elif defined(_MSC_VER)
	#define MOSS_COMPILER_MSVC 1943
#endif

#if (defined(__MINGW64__) || defined(__MINGW32__))
	#define MOSS_COMPILER_MINGW
#endif


// Detect CPU architecture
#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
	// X86 CPU architecture
	#define MOSS_CPU_X86
	#if defined(__x86_64__) || defined(_M_X64)
		#define MOSS_CPU_ADDRESS_BITS 64
	#else
		#define MOSS_CPU_ADDRESS_BITS 32
	#endif
	#define MOSS_SIMD_SSE
	#define MOSS_VECTOR_ALIGNMENT 16
	#define MOSS_DVECTOR_ALIGNMENT 32

	#if defined(__AVX512F__) && defined(__AVX512VL__) && defined(__AVX512DQ__) && !defined(MOSS_SIMD_AVX512)
		#define MOSS_SIMD_AVX512
	#endif
	#if (defined(__AVX2__) || defined(MOSS_SIMD_AVX512)) && !defined(MOSS_SIMD_AVX2)
		#define MOSS_SIMD_AVX2
	#endif
	#if (defined(__AVX__) || defined(MOSS_SIMD_AVX2)) && !defined(MOSS_SIMD_AVX)
		#define MOSS_SIMD_AVX
	#endif
	#if (defined(__SSE4_2__) || defined(MOSS_SIMD_AVX)) && !defined(MOSS_SIMD_SSE4_2)
		#define MOSS_SIMD_SSE4_2
	#endif
	#if (defined(__SSE4_1__) || defined(MOSS_SIMD_SSE4_2)) && !defined(MOSS_SIMD_SSE4_1)
		#define MOSS_SIMD_SSE4_1
	#endif
	#if (defined(__F16C__) || defined(MOSS_SIMD_AVX2)) && !defined(MOSS_SIMD_F16C)
		#define MOSS_SIMD_F16C
	#endif
	#if (defined(__LZCNT__) || defined(MOSS_SIMD_AVX2)) && !defined(MOSS_SIMD_LZCNT)
		#define MOSS_SIMD_LZCNT
	#endif
	#if (defined(__BMI__) || defined(MOSS_SIMD_AVX2)) && !defined(MOSS_SIMD_TZCNT)
		#define MOSS_SIMD_TZCNT
	#endif
	#ifndef MOSS_CROSS_PLATFORM_DETERMINISTIC
		#if defined(MOSS_COMPILER_CLANG) || defined(MOSS_COMPILER_GCC)
			#if defined(__FMA__) && !defined(MOSS_USE_FMADD)
				#define MOSS_USE_FMADD
			#endif
		#elif defined(MOSS_COMPILER_MSVC)
			#if defined(__AVX2__) && !defined(MOSS_USE_FMADD) // AVX2 also enables fused multiply add
				#define MOSS_USE_FMADD
			#endif
		#else
			#error Undefined compiler
		#endif
	#endif // MOSS_CROSS_PLATFORM_DETERMINISTIC
#elif (defined(__aarch64__) || defined(_M_ARM64) || defined(__arm__) || defined(_M_ARM))
	// ARM CPU architecture
	#define MOSS_CPU_ARM
	#if defined(__aarch64__) || defined(_M_ARM64)
		#define MOSS_CPU_ADDRESS_BITS 64
		#define MOSS_SIMD_NEON
		#define MOSS_VECTOR_ALIGNMENT 16
		#define MOSS_DVECTOR_ALIGNMENT 32
	#else
		#define MOSS_CPU_ADDRESS_BITS 32
		#define MOSS_VECTOR_ALIGNMENT 8 // 32-bit ARM does not support aligning on the stack on 16 byte boundaries
		#define MOSS_DVECTOR_ALIGNMENT 8
	#endif
#elif defined(__riscv)
	// RISC-V CPU architecture
	#define MOSS_CPU_RISCV
	#if __riscv_xlen == 64
		#define MOSS_CPU_ADDRESS_BITS 64
		#define MOSS_VECTOR_ALIGNMENT 16
		#define MOSS_DVECTOR_ALIGNMENT 32
	#else
		#define MOSS_CPU_ADDRESS_BITS 32
		#define MOSS_VECTOR_ALIGNMENT 16
		#define MOSS_DVECTOR_ALIGNMENT 8
	#endif
#elif defined(__wasm_simd128__)
	#define MOSS_PLATFORM_WASM
	// WebAssembly CPU architecture
	#define MOSS_CPU_WASM
	#if defined(__wasm64__)
		#define MOSS_CPU_ADDRESS_BITS 64
	#else
		#define MOSS_CPU_ADDRESS_BITS 32
	#endif
	#define MOSS_VECTOR_ALIGNMENT 16
	#define MOSS_DVECTOR_ALIGNMENT 32
	#ifdef __wasm_simd128__
		#define MOSS_SIMD_SSE
		#define MOSS_SIMD_SSE4_1
		#define MOSS_SIMD_SSE4_2
	#endif
#elif (defined(__powerpc__) || defined(__powerpc64__))
	// PowerPC CPU architecture
	#define MOSS_CPU_PPC
	#if defined(__powerpc64__)
		#define MOSS_CPU_ADDRESS_BITS 64
	#else
		#define MOSS_CPU_ADDRESS_BITS 32
	#endif
	#ifdef _BIG_ENDIAN
		#define MOSS_CPU_BIG_ENDIAN
	#endif
	#define MOSS_VECTOR_ALIGNMENT 16
	#define MOSS_DVECTOR_ALIGNMENT 8
#elif defined(__loongarch__)
	// LoongArch CPU architecture
	#define MOSS_CPU_LOONGARCH
	#if defined(__loongarch64)
		#define MOSS_CPU_ADDRESS_BITS 64
	#else
		#define MOSS_CPU_ADDRESS_BITS 32
	#endif
	#define MOSS_VECTOR_ALIGNMENT 16
	#define MOSS_DVECTOR_ALIGNMENT 8
#elif defined(__e2k__)
	// E2K CPU architecture (MCST Elbrus 2000)
	#define MOSS_CPU_E2K
	#define MOSS_CPU_ADDRESS_BITS 64
	#define MOSS_VECTOR_ALIGNMENT 16
	#define MOSS_DVECTOR_ALIGNMENT 32

	// Compiler flags on e2k arch determine CPU features
	#if defined(__SSE__) && !defined(MOSS_SIMD_SSE)
		#define MOSS_SIMD_SSE
	#endif
#else
	#error Unsupported CPU architecture
#endif

// If this define is set, Moss is compiled as a shared library
#ifdef MOSS_SHARED_LIBRARY
	#ifdef MOSS_BUILD_SHARED_LIBRARY
		// While building the shared library, we must export these symbols
		#if defined(MOSS_PLATFORM_WINDOWS) && !defined(MOSS_COMPILER_MINGW)
			#define MOSS_EXPORT __declspec(dllexport)
		#else
			#define MOSS_EXPORT __attribute__ ((visibility ("default")))
			#if defined(MOSS_COMPILER_GCC)
				// Prevents an issue with GCC attribute parsing (see https://gcc.gnu.org/bugzilla/show_bug.cgi?id=69585)
				#define MOSS_EXPORT_GCC_BUG_WORKAROUND [[gnu::visibility("default")]]
			#endif
		#endif
	#else
		// When linking against Moss, we must import these symbols
		#if defined(MOSS_PLATFORM_WINDOWS) && !defined(MOSS_COMPILER_MINGW)
			#define MOSS_EXPORT __declspec(dllimport)
		#else
			#define MOSS_EXPORT __attribute__ ((visibility ("default")))
			#if defined(MOSS_COMPILER_GCC)
				// Prevents an issue with GCC attribute parsing (see https://gcc.gnu.org/bugzilla/show_bug.cgi?id=69585)
				#define MOSS_EXPORT_GCC_BUG_WORKAROUND [[gnu::visibility("default")]]
			#endif
		#endif
	#endif
#else
	// If the define is not set, we use static linking and symbols don't need to be imported or exported
	#define MOSS_EXPORT
#endif

#define MOSS_API MOSS_EXPORT

#ifndef MOSS_EXPORT_GCC_BUG_WORKAROUND
	#define MOSS_EXPORT_GCC_BUG_WORKAROUND MOSS_EXPORT
#endif

// Macro used by the RTTI macros to not export a function
#define MOSS_NO_EXPORT

/////////////////////////////////////////////////////////////////////////////////////////////////////////////


// Pragmas to store / restore the warning state and to disable individual warnings
#ifdef MOSS_COMPILER_CLANG
#define MOSS_PRAGMA(x)					_Pragma(#x)
#define MOSS_SUPPRESS_WARNING_PUSH		MOSS_PRAGMA(clang diagnostic push)
#define MOSS_SUPPRESS_WARNING_POP		MOSS_PRAGMA(clang diagnostic pop)
#define MOSS_CLANG_SUPPRESS_WARNING(w)	MOSS_PRAGMA(clang diagnostic ignored w)
#if __clang_major__ >= 13
	#define MOSS_CLANG_13_PLUS_SUPPRESS_WARNING(w) MOSS_CLANG_SUPPRESS_WARNING(w)
#else
	#define MOSS_CLANG_13_PLUS_SUPPRESS_WARNING(w)
#endif
#if __clang_major__ >= 16
	#define MOSS_CLANG_16_PLUS_SUPPRESS_WARNING(w) MOSS_CLANG_SUPPRESS_WARNING(w)
#else
	#define MOSS_CLANG_16_PLUS_SUPPRESS_WARNING(w)
#endif
#else
#define MOSS_CLANG_SUPPRESS_WARNING(w)
#define MOSS_CLANG_13_PLUS_SUPPRESS_WARNING(w)
#define MOSS_CLANG_16_PLUS_SUPPRESS_WARNING(w)
#endif
#ifdef MOSS_COMPILER_GCC
#define MOSS_PRAGMA(x)					_Pragma(#x)
#define MOSS_SUPPRESS_WARNING_PUSH		MOSS_PRAGMA(GCC diagnostic push)
#define MOSS_SUPPRESS_WARNING_POP		MOSS_PRAGMA(GCC diagnostic pop)
#define MOSS_GCC_SUPPRESS_WARNING(w)	MOSS_PRAGMA(GCC diagnostic ignored w)
#else
#define MOSS_GCC_SUPPRESS_WARNING(w)
#endif
#ifdef MOSS_COMPILER_MSVC
#define MOSS_PRAGMA(x)					__pragma(x)
#define MOSS_SUPPRESS_WARNING_PUSH		MOSS_PRAGMA(warning (push))
#define MOSS_SUPPRESS_WARNING_POP		MOSS_PRAGMA(warning (pop))
#define MOSS_MSVC_SUPPRESS_WARNING(w)	MOSS_PRAGMA(warning (disable : w))
#if _MSC_VER >= 1920 && _MSC_VER < 1930
	#define MOSS_MSVC2019_SUPPRESS_WARNING(w) MOSS_MSVC_SUPPRESS_WARNING(w)
#else
	#define MOSS_MSVC2019_SUPPRESS_WARNING(w)
#endif
#else
#define MOSS_MSVC_SUPPRESS_WARNING(w)
#define MOSS_MSVC2019_SUPPRESS_WARNING(w)
#endif

// Disable common warnings triggered by Moss when compiling with -Wall
#define MOSS_SUPPRESS_WARNINGS																	\
	MOSS_CLANG_SUPPRESS_WARNING("-Wc++98-compat")												\
	MOSS_CLANG_SUPPRESS_WARNING("-Wc++98-compat-pedantic")										\
	MOSS_CLANG_SUPPRESS_WARNING("-Wfloat-equal")												\
	MOSS_CLANG_SUPPRESS_WARNING("-Wsign-conversion")											\
	MOSS_CLANG_SUPPRESS_WARNING("-Wold-style-cast")												\
	MOSS_CLANG_SUPPRESS_WARNING("-Wgnu-anonymous-struct")										\
	MOSS_CLANG_SUPPRESS_WARNING("-Wnested-anon-types")											\
	MOSS_CLANG_SUPPRESS_WARNING("-Wglobal-constructors")										\
	MOSS_CLANG_SUPPRESS_WARNING("-Wexit-time-destructors")										\
	MOSS_CLANG_SUPPRESS_WARNING("-Wnonportable-system-include-path")							\
	MOSS_CLANG_SUPPRESS_WARNING("-Wlanguage-extension-token")									\
	MOSS_CLANG_SUPPRESS_WARNING("-Wunused-parameter")											\
	MOSS_CLANG_SUPPRESS_WARNING("-Wformat-nonliteral")											\
	MOSS_CLANG_SUPPRESS_WARNING("-Wcovered-switch-default")										\
	MOSS_CLANG_SUPPRESS_WARNING("-Wcast-align")													\
	MOSS_CLANG_SUPPRESS_WARNING("-Winvalid-offsetof")											\
	MOSS_CLANG_SUPPRESS_WARNING("-Wgnu-zero-variadic-macro-arguments")							\
	MOSS_CLANG_SUPPRESS_WARNING("-Wdocumentation-unknown-command")								\
	MOSS_CLANG_SUPPRESS_WARNING("-Wctad-maybe-unsupported")										\
	MOSS_CLANG_SUPPRESS_WARNING("-Wswitch-default")												\
	MOSS_CLANG_13_PLUS_SUPPRESS_WARNING("-Wdeprecated-copy")									\
	MOSS_CLANG_13_PLUS_SUPPRESS_WARNING("-Wdeprecated-copy-with-dtor")							\
	MOSS_CLANG_16_PLUS_SUPPRESS_WARNING("-Wunsafe-buffer-usage")								\
	MOSS_IF_NOT_ANDROID(MOSS_CLANG_SUPPRESS_WARNING("-Wimplicit-int-float-conversion"))			\
																								\
	MOSS_GCC_SUPPRESS_WARNING("-Wcomment")														\
	MOSS_GCC_SUPPRESS_WARNING("-Winvalid-offsetof")												\
	MOSS_GCC_SUPPRESS_WARNING("-Wclass-memaccess")												\
	MOSS_GCC_SUPPRESS_WARNING("-Wpedantic")														\
	MOSS_GCC_SUPPRESS_WARNING("-Wunused-parameter")												\
	MOSS_GCC_SUPPRESS_WARNING("-Wmaybe-uninitialized")											\
																								\
	MOSS_MSVC_SUPPRESS_WARNING(4619) /* #pragma warning: there is no warning number 'XXXX' */	\
	MOSS_MSVC_SUPPRESS_WARNING(4514) /* 'X' : unreferenced inline function has been removed */	\
	MOSS_MSVC_SUPPRESS_WARNING(4710) /* 'X' : function not inlined */							\
	MOSS_MSVC_SUPPRESS_WARNING(4711) /* function 'X' selected for automatic inline expansion */	\
	MOSS_MSVC_SUPPRESS_WARNING(4714) /* function 'X' marked as __forceinline not inlined */		\
	MOSS_MSVC_SUPPRESS_WARNING(4820) /* 'X': 'Y' bytes padding added after data member 'Z' */	\
	MOSS_MSVC_SUPPRESS_WARNING(4100) /* 'X' : unreferenced formal parameter */					\
	MOSS_MSVC_SUPPRESS_WARNING(4626) /* 'X' : assignment operator was implicitly defined as deleted because a base class assignment operator is inaccessible or deleted */ \
	MOSS_MSVC_SUPPRESS_WARNING(5027) /* 'X' : move assignment operator was implicitly defined as deleted because a base class move assignment operator is inaccessible or deleted */ \
	MOSS_MSVC_SUPPRESS_WARNING(4365) /* 'argument' : conversion from 'X' to 'Y', signed / unsigned mismatch */ \
	MOSS_MSVC_SUPPRESS_WARNING(4324) /* 'X' : structure was padded due to alignment specifier */ \
	MOSS_MSVC_SUPPRESS_WARNING(4625) /* 'X' : copy constructor was implicitly defined as deleted because a base class copy constructor is inaccessible or deleted */ \
	MOSS_MSVC_SUPPRESS_WARNING(5026) /* 'X': move constructor was implicitly defined as deleted because a base class move constructor is inaccessible or deleted */ \
	MOSS_MSVC_SUPPRESS_WARNING(4623) /* 'X' : default constructor was implicitly defined as deleted */ \
	MOSS_MSVC_SUPPRESS_WARNING(4201) /* nonstandard extension used: nameless struct/union */		\
	MOSS_MSVC_SUPPRESS_WARNING(4371) /* 'X': layout of class may have changed from a previous version of the compiler due to better packing of member 'Y' */ \
	MOSS_MSVC_SUPPRESS_WARNING(5045) /* Compiler will insert Spectre mitigation for memory load if /Qspectre switch specified */ \
	MOSS_MSVC_SUPPRESS_WARNING(4583) /* 'X': destructor is not implicitly called */				\
	MOSS_MSVC_SUPPRESS_WARNING(4582) /* 'X': constructor is not implicitly called */				\
	MOSS_MSVC_SUPPRESS_WARNING(5219) /* implicit conversion from 'X' to 'Y', possible loss of data  */ \
	MOSS_MSVC_SUPPRESS_WARNING(4826) /* Conversion from 'X *' to 'MOSS::uint64' is sign-extended. This may cause unexpected runtime behavior. (32-bit) */ \
	MOSS_MSVC_SUPPRESS_WARNING(5264) /* 'X': 'const' variable is not used */						\
	MOSS_MSVC_SUPPRESS_WARNING(4251) /* class 'X' needs to have DLL-interface to be used by clients of class 'Y' */ \
	MOSS_MSVC_SUPPRESS_WARNING(4738) /* storing 32-bit float result in memory, possible loss of performance */ \
	MOSS_MSVC2019_SUPPRESS_WARNING(5246) /* the initialization of a subobject should be wrapped in braces */

// OS-specific includes
#if defined(MOSS_PLATFORM_WINDOWS)
	#define MOSS_BREAKPOINT		__debugbreak()
#elif defined(MOSS_PLATFORM_BLUE)
	// Configuration for a popular game console.
	// This file is not distributed because it would violate an NDA.
	// Creating one should only be a couple of minutes of work if you have the documentation for the platform
	// (you only need to define MOSS_BREAKPOINT, MOSS_PLATFORM_BLUE_GET_TICKS, MOSS_PLATFORM_BLUE_MUTEX*, MOSS_PLATFORM_BLUE_RWLOCK*, MOSS_PLATFORM_BLUE_SEMAPHORE* and include the right header).
	//#include <Moss/Core/PlatformBlue.h>
#elif defined(MOSS_PLATFORM_LINUX) || defined(MOSS_PLATFORM_ANDROID) || defined(MOSS_PLATFORM_MACOS) || defined(MOSS_PLATFORM_IOS) || defined(MOSS_PLATFORM_BSD)
	#if defined(MOSS_CPU_X86)
		#define MOSS_BREAKPOINT	__asm volatile ("int $0x3")
	#elif defined(MOSS_CPU_ARM) || defined(MOSS_CPU_RISCV) || defined(MOSS_CPU_E2K) || defined(MOSS_CPU_PPC) || defined(MOSS_CPU_LOONGARCH)
		#define MOSS_BREAKPOINT	__builtin_trap()
	#else
		#error Unknown CPU architecture
	#endif
#elif defined(MOSS_PLATFORM_WASM)
	#define MOSS_BREAKPOINT		do { } while (false) // Not supported
#else
	#error Unknown platform
#endif


#define MOSS_SUPRESS_WARNINGS_BEGIN 					\
	MOSS_SUPPRESS_WARNING_PUSH							\
	MOSS_SUPPRESS_WARNINGS

#define MOSS_SUPRESS_WARNINGS_END 			MOSS_SUPPRESS_WARNING_POP


// Suppress warnings generated by the standard template library
#define MOSS_SUPPRESS_WARNINGS_STD_BEGIN															\
	MOSS_SUPPRESS_WARNING_PUSH																		\
	MOSS_MSVC_SUPPRESS_WARNING(4365)																\
	MOSS_MSVC_SUPPRESS_WARNING(4619)																\
	MOSS_MSVC_SUPPRESS_WARNING(4710)																\
	MOSS_MSVC_SUPPRESS_WARNING(4711)																\
	MOSS_MSVC_SUPPRESS_WARNING(4820)																\
	MOSS_MSVC_SUPPRESS_WARNING(4514)																\
	MOSS_MSVC_SUPPRESS_WARNING(5262)																\
	MOSS_MSVC_SUPPRESS_WARNING(5264)																\
	MOSS_MSVC_SUPPRESS_WARNING(4738)																\
	MOSS_MSVC_SUPPRESS_WARNING(5045)

#define MOSS_SUPPRESS_WARNINGS_STD_END MOSS_SUPPRESS_WARNING_POP


// Define inline macro
#if defined(MOSS_NO_FORCE_INLINE)
	#define MOSS_INLINE inline
#elif defined(MOSS_COMPILER_CLANG)
	#define MOSS_INLINE __inline__ __attribute__((always_inline))
#elif defined(MOSS_COMPILER_GCC)
	// On gcc 14 using always_inline in debug mode causes error: "inlining failed in call to 'always_inline' 'XXX': function not considered for inlining"
	#if __GNUC__ >= 14 && defined(MOSS_DEBUG)
		#define MOSS_INLINE inline
	#else
		#define MOSS_INLINE __inline__ __attribute__((always_inline))
	#endif
#elif defined(MOSS_COMPILER_MSVC)
	#define MOSS_INLINE __forceinline
#else
	#error Undefined
#endif

#define MOSS_NOINLINE     __attribute__((noinline))
#define MOSS_ALIGN(x)     __attribute__((aligned(x)))

#if defined(MOSS_COMPILER_GCC) || defined(MOSS_COMPILER_CLANG)
#define MOSS_LIKELY(x)    __builtin_expect((x), U1)
#define MOSS_UNLIKELY(x)  __builtin_expect((x), U0)
#else
#define MOSS_LIKELY(x) (x)
#define MOSS_UNLIKELY(x) (x)
#endif 

// Cache line size (used for aligning to cache line)
#ifndef MOSS_CACHE_LINE_SIZE
	#define MOSS_CACHE_LINE_SIZE 64
#endif

// Define macro to get current function name
#if defined(MOSS_COMPILER_CLANG) || defined(MOSS_COMPILER_GCC)
	#define MOSS_FUNCTION_NAME	__PRETTY_FUNCTION__
#elif defined(MOSS_COMPILER_MSVC)
	#define MOSS_FUNCTION_NAME	__FUNCTION__
#else
	#error Undefined
#endif
// Stack allocation
#define MOSS_STACK_ALLOC(n)		alloca(n)


//////////////////////////////////////////////////////////////////////////////////////////////////////////////


// Standard C++ includes
MOSS_SUPPRESS_WARNINGS_STD_BEGIN
#include <float.h>
#include <limits.h>
#include <string.h>
#include <utility>
#include <cmath>
#include <sstream>
#include <functional>
#include <algorithm>
#include <cstdint>
#include <atomic>
#include <mutex>
#include <shared_mutex>
#include <thread>
#include <cstdlib>  // for rand()
#include <ctime>    // for seeding

#ifdef MOSS_COMPILER_MSVC
	#include <malloc.h> // for alloca
#endif

// SIMD includes
#if defined(MOSS_SIMD_SSEEEE)
#include <immintrin.h>
#elif defined(MOSS_SIMD_NEON)
	#ifdef MOSS_COMPILER_MSVC
		#include <intrin.h>
		#include <arm64_neon.h>
	#else
		#include <arm_neon.h>
	#endif
#endif
MOSS_SUPPRESS_WARNINGS_STD_END


MOSS_SUPRESS_WARNINGS_BEGIN
// Commonly used STL types
using std::min;
using std::max;
using std::abs;
using std::acos;
using std::acosf;
using std::asinf;
using std::sqrt;
using std::ceil;
using std::floor;
using std::trunc;
using std::round;
using std::fmod;
using std::string_view;
using std::function;
using std::numeric_limits;
using std::isfinite;
using std::isnan;
using std::ostream;
using std::istream;
using std::asin;
using std::atan;
using std::atan2;
using std::exp;
using std::fabs;
using std::floor;
using std::fmod;
using std::pow;
using std::powf;
using std::sin;
using std::qsort;
using std::atof;
using std::atoi;
using std::ceil;
using std::cos;
using std::isnan;
using std::tan;
using std::log;
using std::log10;
using std::lround;
using std::sqrt;
using std::sqrtf;
using std::srand;

using std::asinhf;
using std::acoshf;
using std::asinhf;
using std::atanhf;
using std::asinf;
using std::fabsf;
using std::floorf;
using std::fmodf;
using std::isinf;
using std::modf;
using std::modff;
using std::sinf;
using std::atan2f;

using std::ceilf;
using std::cosf;
using std::nanf;
using std::tanf;
using std::atanf;
using std::log10f;
using std::logf;
using std::lroundf;
using std::sscanf;


using std::atomic;
using std::memory_order;
using std::memory_order_relaxed;
using std::memory_order_acquire;
using std::memory_order_release;
using std::memory_order_acq_rel;
using std::memory_order_seq_cst;


using std::mutex;
using std::shared_mutex;
using std::thread;
using std::lock_guard;
using std::shared_lock;
using std::unique_lock;

// Assert sizes of types
static_assert(sizeof(uint8) == 1, "Invalid size of uint8");
static_assert(sizeof(uint16) == 2, "Invalid size of uint16");
static_assert(sizeof(uint32) == 4, "Invalid size of uint32");
static_assert(sizeof(uint64) == 8, "Invalid size of uint64");
static_assert(sizeof(void *) == (MOSS_CPU_ADDRESS_BITS == 64? 8 : 4), "Invalid size of pointer" );


#ifndef MOSS_DISABLE_CUSTOM_ALLOCATOR

// Normal memory allocation, must be at least 8 byte aligned on 32 bit platform and 16 byte aligned on 64 bit platform
using AllocateFunction = void *(*)(size_t inSize);
using ReallocateFunction = void *(*)(void *inBlock, size_t inOldSize, size_t inNewSize);
using FreeFunction = void (*)(void *inBlock);

// Aligned memory allocation
using AlignedAllocateFunction = void *(*)(size_t inSize, size_t inAlignment);
using AlignedFreeFunction = void (*)(void *inBlock);

// User defined allocation / free functions
MOSS_EXPORT extern AllocateFunction Allocate;
MOSS_EXPORT extern ReallocateFunction Reallocate;
MOSS_EXPORT extern FreeFunction Free;
MOSS_EXPORT extern AlignedAllocateFunction AlignedAllocate;
MOSS_EXPORT extern AlignedFreeFunction AlignedFree;

/// Register platform default allocation / free functions
MOSS_EXPORT void RegisterDefaultAllocator();

/// Macro to override the new and delete functions
#define MOSS_OVERRIDE_NEW_DELETE 																																		\
	MOSS_INLINE void *operator new (size_t inCount)												{ return Allocate(inCount); } 											\
	MOSS_INLINE void operator delete (void *inPointer) noexcept									{ Free(inPointer); } 													\
	MOSS_INLINE void *operator new[] (size_t inCount)											{ return Allocate(inCount); } 											\
	MOSS_INLINE void operator delete[] (void *inPointer) noexcept								{ Free(inPointer); } 													\
	MOSS_INLINE void *operator new (size_t inCount, std::align_val_t inAlignment)				{ return AlignedAllocate(inCount, static_cast<size_t>(inAlignment)); } 	\
	MOSS_INLINE void operator delete (void *inPointer, [[maybe_unused]] std::align_val_t inAlignment) noexcept	{ AlignedFree(inPointer); } 							\
	MOSS_INLINE void *operator new[] (size_t inCount, std::align_val_t inAlignment)				{ return AlignedAllocate(inCount, static_cast<size_t>(inAlignment)); } 	\
	MOSS_INLINE void operator delete[] (void *inPointer, [[maybe_unused]] std::align_val_t inAlignment) noexcept	{ AlignedFree(inPointer); } 						\
	MOSS_INLINE void *operator new ([[maybe_unused]] size_t inCount, void *inPointer) noexcept	{ return inPointer; } 													\
	MOSS_INLINE void operator delete ([[maybe_unused]] void *inPointer, [[maybe_unused]] void *inPlace) noexcept { /* Do nothing */ } 									\
	MOSS_INLINE void *operator new[] ([[maybe_unused]] size_t inCount, void *inPointer) noexcept	{ return inPointer; } 												\
	MOSS_INLINE void operator delete[] ([[maybe_unused]] void *inPointer, [[maybe_unused]] void *inPlace) noexcept { /* Do nothing */ }

#else
// Directly define the allocation functions
MOSS_EXPORT void *Allocate(size_t inSize);
MOSS_EXPORT void *Reallocate(void *inBlock, size_t inOldSize, size_t inNewSize);
MOSS_EXPORT void Free(void *inBlock);
MOSS_EXPORT void *AlignedAllocate(size_t inSize, size_t inAlignment);
MOSS_EXPORT void AlignedFree(void *inBlock);

// Don't implement allocator registering
inline void RegisterDefaultAllocator() { }

// Don't override new/delete
#define MOSS_OVERRIDE_NEW_DELETE

#endif // !MOSS_DISABLE_CUSTOM_ALLOCATOR

// Shorthand for #ifdef MOSS_DEBUG / #endif
#ifdef MOSS_DEBUG
#include <stdio.h>
#include <stdarg.h>
#include <cstdint>

// Core logging function
static inline void Moss_Log(const char* prefix, const char* msg, const char* color, ...) {
    char formatBuffer[8192];
    snprintf(formatBuffer, sizeof(formatBuffer), "%s%s%s\x1b[0m\n", color, prefix, msg);

    va_list args;
    va_start(args, color);
    vprintf(formatBuffer, args);
    va_end(args);
}
#define MOSS_TRACE(msg, ...) Moss_Log("TRACE: ", msg, "\x1b[37m", ##__VA_ARGS__); // White
#define MOSS_INFO(msg, ...)  Moss_Log("INFO:  ", msg, "\x1b[32m", ##__VA_ARGS__); // Green
#define MOSS_WARN(msg, ...)  Moss_Log("WARN:  ", msg, "\x1b[33m", ##__VA_ARGS__); // Yellow
#define MOSS_ERROR(msg, ...) Moss_Log("ERROR: ", msg, "\x1b[31m", ##__VA_ARGS__); // Red
#define MOSS_FATAL(msg, ...) Moss_Log("FATAL: ", msg, "\x1b[35m", ##__VA_ARGS__); // Magenta

#define MOSS_CHECK(x, msg, ...)                                       \
    do {                                                              \
        if ((x)) {                                                    \
            MOSS_FATAL("Assertion Failed: " + msg, ##__VA_ARGS__)     \
            abort();                                             	  \
        }                                                             \
    } while (0)

// Assertion with logging
using MossAssertFailedFunc = bool (*)(const char* expression, const char* message, const char* file, uint32_t line);

// Inline function pointer (C++17+)
inline MossAssertFailedFunc MossAssertFailed = nullptr;

// Default implementation
inline bool MossAssertFailedImpl(const char* expression, const char* message, const char* file, uint32_t line)
{
    std::printf("MOSS ASSERT FAILED:\n  Expression: %s\n  File: %s:%u\n  Message: %s\n",
                expression, file, line, message ? message : "(none)");
    return true; // return true to trigger DEBUG_BREAK
}

// Assign default handler once
struct MossAssertInitializer
{
    MossAssertInitializer()
    {
        if (MossAssertFailed == nullptr)
            MossAssertFailed = MossAssertFailedImpl;
    }
};
inline MossAssertInitializer gMossAssertInitializer;

// Platform-specific debug break
#if defined(_WIN32)
    #define DEBUG_BREAK() __debugbreak()
#elif defined(__linux__) || defined(__APPLE__)
    #define DEBUG_BREAK() __builtin_trap()
#else
    #define DEBUG_BREAK() ((void)0)
#endif

// Helper to support optional message
struct MossAssertLastParam {};
inline bool MossAssertHelper(const char* expr, const char* file, uint32_t line, MossAssertLastParam)
{
    return MossAssertFailed(expr, nullptr, file, line);
}
inline bool MossAssertHelper(const char* expr, const char* file, uint32_t line, const char* msg, MossAssertLastParam)
{
    return MossAssertFailed(expr, msg, file, line);
}

// Final macro (optional message supported)
#define MOSS_ASSERT(expr, ...) \
    do { \
        if (!(expr)) { \
            if (MossAssertHelper(#expr, __FILE__, static_cast<uint32_t>(__LINE__), ##__VA_ARGS__, MossAssertLastParam())) \
                DEBUG_BREAK(); \
        } \
    } while (0)

#define MOSS_IF_ENABLE_ASSERTS(...)		__VA_ARGS__
#else
	#define MOSS_TRACE(msg, ...)
	#define MOSS_DEBUG(msg, ...)
	#define MOSS_INFO(msg, ...)
	#define MOSS_WARN(msg, ...)
	#define MOSS_ERROR(msg, ...)
	#define MOSS_FATAL(msg, ...)
	#define MOSS_CHECK(x, msg, ...)
	#define MOSS_ASSERT(...) __VA_ARGS__

	#define MOSS_IF_ENABLE_ASSERTS(...) __VA_ARGS__
#endif

// Shorthand for #ifdef MOSS_FLOATING_POINT_EXCEPTIONS_ENABLED / #endif
#ifdef MOSS_FLOATING_POINT_EXCEPTIONS_ENABLED
	#define MOSS_IF_FLOATING_POINT_EXCEPTIONS_ENABLED(...)	__VA_ARGS__
#else
	#define MOSS_IF_FLOATING_POINT_EXCEPTIONS_ENABLED(...)
#endif

// Helper macros to detect if we're running in single or double precision mode
#ifdef MOSS_DOUBLE_PRECISION
	#define MOSS_IF_SINGLE_PRECISION(...)
	#define MOSS_IF_SINGLE_PRECISION_ELSE(s, d) d
	#define MOSS_IF_DOUBLE_PRECISION(...) __VA_ARGS__
#else
	#define MOSS_IF_SINGLE_PRECISION(...) __VA_ARGS__
	#define MOSS_IF_SINGLE_PRECISION_ELSE(s, d) s
	#define MOSS_IF_DOUBLE_PRECISION(...)
#endif

// Helper macro to detect if the debug renderer is active
#ifndef MOSS_DEBUG_RENDERER
	#define MOSS_IF_DEBUG_RENDERER(...) __VA_ARGS__
	#define MOSS_IF_NOT_DEBUG_RENDERER(...)
#else
	#define MOSS_IF_DEBUG_RENDERER(...)
	#define MOSS_IF_NOT_DEBUG_RENDERER(...) __VA_ARGS__
#endif

// Macro to indicate that a parameter / variable is unused
#define MOSS_UNUSED(x)			(void)x

// Macro to enable floating point precise mode and to disable fused multiply add instructions
#if defined(MOSS_COMPILER_GCC) || defined(MOSS_CROSS_PLATFORM_DETERMINISTIC)
	// We compile without -ffast-math and -ffp-contract=fast, so we don't need to disable anything
	#define MOSS_PRECISE_MATH_ON
	#define MOSS_PRECISE_MATH_OFF
#elif defined(MOSS_COMPILER_CLANG)
	// We compile without -ffast-math because pragma float_control(precise, on) doesn't seem to actually negate all of the -ffast-math effects and causes the unit tests to fail (even if the pragma is added to all files)
	// On clang 14 and later we can turn off float contraction through a pragma (before it was buggy), so if FMA is on we can disable it through this macro
	#if (defined(MOSS_CPU_ARM) && !defined(MOSS_PLATFORM_ANDROID) && __clang_major__ >= 16) || (defined(MOSS_CPU_X86) && __clang_major__ >= 14)
		#define MOSS_PRECISE_MATH_ON						\
			_Pragma("float_control(precise, on, push)")	\
			_Pragma("clang fp contract(off)")
		#define MOSS_PRECISE_MATH_OFF					\
			_Pragma("float_control(pop)")
	#elif __clang_major__ >= 14 && (defined(MOSS_USE_FMADD) || defined(FP_FAST_FMA))
		#define MOSS_PRECISE_MATH_ON						\
			_Pragma("clang fp contract(off)")
		#define MOSS_PRECISE_MATH_OFF					\
			_Pragma("clang fp contract(on)")
	#else
		#define MOSS_PRECISE_MATH_ON
		#define MOSS_PRECISE_MATH_OFF
	#endif
#elif defined(MOSS_COMPILER_MSVC)
	// Unfortunately there is no way to push the state of fp_contract, so we have to assume it was turned on before MOSS_PRECISE_MATH_ON
	#define MOSS_PRECISE_MATH_ON							\
		__pragma(float_control(precise, on, push))		\
		__pragma(fp_contract(off))
	#define MOSS_PRECISE_MATH_OFF						\
		__pragma(fp_contract(on))						\
		__pragma(float_control(pop))
#else
	#error Undefined
#endif

// Check if Thread Sanitizer is enabled
#ifdef __has_feature
	#if __has_feature(thread_sanitizer)
		#define MOSS_TSAN_ENABLED
	#endif
#else
	#ifdef __SANITIZE_THREAD__
		#define MOSS_TSAN_ENABLED
	#endif
#endif


#define MOSS_CALL __cdecl

// Attribute to disable Thread Sanitizer for a particular function
#ifdef MOSS_TSAN_ENABLED
	#define MOSS_TSAN_NO_SANITIZE __attribute__((no_sanitize("thread")))
#else
	#define MOSS_TSAN_NO_SANITIZE
#endif

#define MOSS_FALSE 	0U
#define MOSS_TRUE 	1U

#define MOSS_PI          	3.14159265358979323846F
#define MOSS_HALF_PI        1.5707963267948966F
#define MOSS_QUARTER_PI     0.7853981633974483F
#define MOSS_E        		2.71828182845904523536F   // e
#define M_LOG2E    			1.44269504088896340736F   // log2(e)
#define M_LOG10E   			0.434294481903251827651F  // log10(e)
#define M_LN2      			0.693147180559945309417F  // ln(2)
#define M_LN10     			2.30258509299404568402F   // ln(10)
#define M_2_SQRTPI 			1.12837916709551257390F   // 2/sqrt(pi)
#define M_SQRT2    			1.41421356237309504880F   // sqrt(2)
#define M_SQRT1_2  			0.707106781186547524401F  // 1/sqrt(2)
#define ATAN_POLY(x)  ((x) * (0.999866 + 0.333331 * (x) * (x)))
#define MOSS_FLT_EPSILON 	1.1920928955078125e-07F

#define RAD(x)        ((x) * (PI / 180.0))
#define MOSS_DIFFERENCE(x, y) ((x) < (y) ? (y) - (x) : (x) - (y))

#define ArraySize(x) (sizeof(x)) / (sizeof((x)[0]))

typedef signed char         int8;
typedef signed short        int16;
typedef signed int          int32;
typedef signed long long    int64;

typedef unsigned char       uint8;
typedef unsigned short      uint16;
typedef unsigned int        uint32;
typedef unsigned long long  uint64;

// Signed

#define MAX_INT8    ((int8)(0x7F))
#define MAX_INT16   ((int16)(0x7FFF))
#define MAX_INT32   ((int32)(0x7FFFFFFF))
#define MAX_INT64   ((int64)(0x7FFFFFFFFFFFFFFF))
#define MIN_INT8    ((int8)(~0x7F))
#define MIN_INT16   ((int16)~0x7FFF)
#define MIN_INT32   ((int32)(~0x7FFFFFFF))
#define MIN_INT64   ((int64)(~0x7FFFFFFFFFFFFFFF))

// Unsigned

#define MAX_UINT8   ((uint8)(0xFF))
#define MAX_UINT16  ((uint16)(0xFFFF))
#define MAX_UINT32  ((uint32)(0xFFFFFFFFu))
#define MAX_UINT64  ((uint64)(0xFFFFFFFFFFFFFFFF))
#define MIN_UINT8   ((uint8)0x00)
#define MIN_UINT16  ((uint16)0x0000)
#define MIN_UINT32  ((uint32)0x00000000)
#define MIN_UINT64  ((uint64)(0x0000000000000000))

static constexpr float cLargeFloat = 1.0e15f;
#define MOSS_LARGE_FLOAT cLargeFloat


static void* Moss_Malloc(size_t size) { return malloc(size); }
static void Moss_Free(void* ptr) { if (ptr) free(ptr); }

#ifndef MOSS_MALLOC
    #define MOSS_MALLOC(size)       Moss_Malloc(size)
#endif
#ifndef MOSS_CALLOC
    #define MOSS_CALLOC(nmemb, size)     calloc(n,sz)
#endif
#ifndef MOSS_REALLOC
    #define MOSS_REALLOC(ptr, size)  realloc(ptr,sz)
#endif
#ifndef MOSS_FREE
    #define MOSS_FREE(ptr)       Moss_Free(ptr)
#endif
#ifndef MOSS_ALIGNED_ALLOC
    #define MOSS_ALIGNED_ALLOC(ptr, alignment, size) aligned_alloc(alignment, alignment*sizeof *ptr);
#endif
#ifndef MOSS_ALIGNED_FREE
    #define MOSS_ALIGNED_FREE(mem) aligned_free(void *mem)
#endif

typedef enum Colorspace {
    COLORSPACE_UNKNOWN = 0,
    COLORSPACE_SRGB = 0x120005a0u,
    COLORSPACE_SRGB_LINEAR = 0x12000500u,
    COLORSPACE_HDR10 = 0x12002600u, 
    COLORSPACE_JPEG = 0x220004c6u,
    COLORSPACE_BT601_LIMITED = 0x211018c6u,
    COLORSPACE_BT601_FULL = 0x221018c6u,
    COLORSPACE_BT709_LIMITED = 0x21100421u,
    COLORSPACE_BT709_FULL = 0x22100421u, 
    COLORSPACE_BT2020_LIMITED = 0x21102609u,
    COLORSPACE_BT2020_FULL = 0x22102609u,

    COLORSPACE_RGB_DEFAULT = COLORSPACE_SRGB,
    COLORSPACE_YUV_DEFAULT = COLORSPACE_BT601_LIMITED
};

enum class PixelFormat {
    PIXELFORMAT_UNKNOWN = 0,
    PIXELFORMAT_INDEX1LSB = 0x11100100u,
    PIXELFORMAT_INDEX1MSB = 0x11200100u,
    PIXELFORMAT_INDEX2LSB = 0x1c100200u,
    PIXELFORMAT_INDEX2MSB = 0x1c200200u,
    PIXELFORMAT_INDEX4LSB = 0x12100400u,
    PIXELFORMAT_INDEX4MSB = 0x12200400u,
    PIXELFORMAT_INDEX8 = 0x13000801u,
    PIXELFORMAT_RGB332 = 0x14110801u,
    PIXELFORMAT_XRGB4444 = 0x15120c02u,
    PIXELFORMAT_XBGR4444 = 0x15520c02u,
    PIXELFORMAT_XRGB1555 = 0x15130f02u,
    PIXELFORMAT_XBGR1555 = 0x15530f02u,
    PIXELFORMAT_ARGB4444 = 0x15321002u,
    PIXELFORMAT_RGBA4444 = 0x15421002u,
    PIXELFORMAT_ABGR4444 = 0x15721002u,
    PIXELFORMAT_BGRA4444 = 0x15821002u,
    PIXELFORMAT_ARGB1555 = 0x15331002u,
    PIXELFORMAT_RGBA5551 = 0x15441002u,
    PIXELFORMAT_ABGR1555 = 0x15731002u,
    PIXELFORMAT_BGRA5551 = 0x15841002u,
    PIXELFORMAT_RGB565 = 0x15151002u,
    PIXELFORMAT_BGR565 = 0x15551002u,
    PIXELFORMAT_RGB24 = 0x17101803u,
    PIXELFORMAT_BGR24 = 0x17401803u,
    PIXELFORMAT_XRGB8888 = 0x16161804u,
    PIXELFORMAT_RGBX8888 = 0x16261804u,
    PIXELFORMAT_XBGR8888 = 0x16561804u,
    PIXELFORMAT_BGRX8888 = 0x16661804u,
    PIXELFORMAT_ARGB8888 = 0x16362004u,
    PIXELFORMAT_RGBA8888 = 0x16462004u,
    PIXELFORMAT_ABGR8888 = 0x16762004u,
    PIXELFORMAT_BGRA8888 = 0x16862004u,
    PIXELFORMAT_XRGB2101010 = 0x16172004u,
    PIXELFORMAT_XBGR2101010 = 0x16572004u,
    PIXELFORMAT_ARGB2101010 = 0x16372004u,
    PIXELFORMAT_ABGR2101010 = 0x16772004u,
    PIXELFORMAT_RGB48 = 0x18103006u,
    PIXELFORMAT_BGR48 = 0x18403006u,
    PIXELFORMAT_RGBA64 = 0x18204008u,
    PIXELFORMAT_ARGB64 = 0x18304008u,
    PIXELFORMAT_BGRA64 = 0x18504008u,
    PIXELFORMAT_ABGR64 = 0x18604008u,
    PIXELFORMAT_RGB48_FLOAT = 0x1a103006u,
    PIXELFORMAT_BGR48_FLOAT = 0x1a403006u,
    PIXELFORMAT_RGBA64_FLOAT = 0x1a204008u,
    PIXELFORMAT_ARGB64_FLOAT = 0x1a304008u,
    PIXELFORMAT_BGRA64_FLOAT = 0x1a504008u,
    PIXELFORMAT_ABGR64_FLOAT = 0x1a604008u,
    PIXELFORMAT_RGB96_FLOAT = 0x1b10600cu,
    PIXELFORMAT_BGR96_FLOAT = 0x1b40600cu,
    PIXELFORMAT_RGBA128_FLOAT = 0x1b208010u,
    PIXELFORMAT_ARGB128_FLOAT = 0x1b308010u,
    PIXELFORMAT_BGRA128_FLOAT = 0x1b508010u,
    PIXELFORMAT_ABGR128_FLOAT = 0x1b608010u,
    PIXELFORMAT_YV12 = 0x32315659u,
    PIXELFORMAT_IYUV = 0x56555949u,
    PIXELFORMAT_YUY2 = 0x32595559u,
    PIXELFORMAT_UYVY = 0x59565955u,
    PIXELFORMAT_YVYU = 0x55595659u,
    PIXELFORMAT_NV12 = 0x3231564eu,
    PIXELFORMAT_NV21 = 0x3132564eu,
    PIXELFORMAT_P010 = 0x30313050u,
    PIXELFORMAT_EXTERNAL_OES = 0x2053454fu,
    PIXELFORMAT_MJPG = 0x47504a4du,
};

enum {
	SWIZZLE_X = 0,			///< Use the X component
	SWIZZLE_Y = 1,			///< Use the Y component
	SWIZZLE_Z = 2,			///< Use the Z component
	SWIZZLE_W = 3,			///< Use the W component
	SWIZZLE_UNUSED = 2,		///< We always use the Z component when we don't specifically want to initialize a value, this is consistent with what is done in Vec3(x, y, z), Vec3(Float3 &) and Vec3::sLoadFloat3Unsafe
};

struct BumpAlloc {
    size_t capacity;
    size_t used;
    char* memory;
};

/// Convert a value from degrees to radians
MOSS_INLINE constexpr float DegreesToRadians(float value) { return value * (MOSS_PI / 180.0f); }
/// Convert a value from radians to degrees
MOSS_INLINE constexpr float RadiansToDegrees(float value) { return value * (180.0f / MOSS_PI); }
//
MOSS_INLINE constexpr void seed_random() { std::srand(static_cast<unsigned int>(std::time(nullptr))); }
// Returns a float between min and max
MOSS_INLINE constexpr float randf_range(float min, float max) { return min + static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX) * (max - min); }
//
MOSS_INLINE constexpr int randi_range(int min, int max) { return min + std::rand() % (max - min + 1); }

/*
// Memory management
// Random functions
MOSS_API int32  Moss_rand(int32 n);
MOSS_API float  Moss_randf(void);
MOSS_API int    Moss_randi(void);
MOSS_API void   Moss_srand(int64 seed);
MOSS_API int32  Moss_randi_range(int32 from, int32 to);
MOSS_API float  Moss_randf_range(float from, float to);
MOSS_API float  Moss_rand_gaussian(float mean, float stddev);
MOSS_API float  Moss_rand_unit();
*/



/*            Timer          */

/*! @brief Get every Tick. @ingroup Time.
MOSS_API Moss_Time Moss_GetTicks();

/*! @brief Get Seconds. @ingroup Time.
MOSS_API double Moss_GetSeconds(Moss_Time time);

/// Get the milliseconds passed from an initial tick value.
MOSS_API float Moss_GetMilliseconds(Moss_Time ticks);

/// Get the milliseconds passed from an initial tick value. Resets the passed in
/// value to the current tick value.
MOSS_API float Moss_GetMillisecondsAndReset(Moss_Time* ticks);

/*! @brief Get Nanoseconds. @ingroup Time.
void Moss_GetNanoseconds(uint64 nanoseconds);

/// Yield to be used in a busy loop.
MOSS_API void Moss_Yield(void);

/*! @brief Delay Makes a time delay for the next below to run. @param delay milliseconds. @ingroup Time.
MOSS_API double Moss_Delay(double delay);


Moss_MS_PER_SECOND
Moss_MS_TO_NS
Moss_NS_PER_MS
Moss_NS_PER_SECOND
Moss_NS_PER_US
Moss_NS_TO_MS
Moss_NS_TO_SECONDS
Moss_NS_TO_US
Moss_SECONDS_TO_NS
Moss_US_PER_SECOND
Moss_US_TO_NS

Moss_AddTimer
Moss_AddTimerNS
Moss_Delay
Moss_DelayNS
Moss_DelayPrecise
Moss_GetPerformanceCounter
Moss_GetPerformanceFrequency
Moss_GetTicks
Moss_GetTicksNS
Moss_RemoveTimer

/*            Time         */

/*! @brief Returns "HH:MM:SS"
MOSS_API const char* Moss_LocalTime(void);         

/*! @brief Returns "YYYY-MM-DD_HH-MM-SS" (useful for filenames)
MOSS_API const char* Moss_TimeStamp(void);         

/*! @brief Returns Unix timestamp as string (seconds since epoch)
MOSS_API const char* Moss_TimeNow(void);           

/*! @brief Returns full ctime string: "Sun May 18 12:34:56 2025"
MOSS_API const char* Moss_CTimeNow(void);          

/*! @brief Returns "YYYY-MM-DD_HH-MM-SS" in GMT*/
//MOSS_API const char* Moss_TimeStampGMT(void);

/*! @brief Returns "YYYY-MM-DD_HH-MM-SS" in Eastern Time.
MOSS_API const char* Moss_TimeStampET(void);

/*! @brief Returns "YYYY-MM-DD_HH-MM-SS" in Central Time.
MOSS_API const char* Moss_TimeStampCT(void);

/*! @brief Returns "YYYY-MM-DD_HH-MM-SS" in Mountain Time.
MOSS_API const char* Moss_TimeStampMT(void);

/*! @brief Returns "YYYY-MM-DD_HH-MM-SS" in Pacific Time.
MOSS_API const char* Moss_TimeStampPT(void);

/*!
// Date & Time Components.
@param %A Full weekday name, e.g., "Sunday".
@param %a Abbreviated weekday name, e.g., "Sun".
@param %B Full month name, e.g., "January".
@param %b Abbreviated month name, e.g., "Jan".
@param %C Century number (year / 100), e.g., "20" for 2025.
@param %d Day of the month [01,31].
@param %e Day of the month [1,31], single digits not zero-padded (non-standard on Windows).
@param %H Hour in 24-hour format [00,23].
@param %I Hour in 12-hour format [01,12].
@param %j Day of the year [001,366].
@param %m Month as number [01,12].
@param %M Minutes [00,59].
@param %S Seconds [00,60] (can be 60 for leap seconds).
@param %U Week number of the year (Sunday as first day of week) [00,53].
@param %w Weekday as number [0,6] (0 = Sunday).
@param %W Week number (Monday as first day of week) [00,53].
@param %y Year as last two digits [00,99].
@param %Y Year as four digits, e.g., "2025".
@param %z Numeric UTC offset, e.g., "-0500".
@param %Z Time zone abbreviation, e.g., "EST", "UTC".
@param %p AM or PM.

// Combined Date/Time Formats.

@param %c Locale's date and time representation, e.g., "Sun May 18 14:30:00 2025".
@param %D Equivalent to "%m/%d/%y", e.g., "05/18/25".
@param %F Equivalent to "%Y-%m-%d" (ISO 8601), e.g., "2025-05-18".
@param %r Time in 12-hour format with AM/PM, e.g., "02:30:00 PM".
@param %R Time in 24-hour format without seconds, e.g., "14:30".
@param %TTime in 24-hour format with seconds, e.g., "14:30:00".
@param %x Locale's date format only, e.g., "05/18/25".
@param %X Locale's time format only, e.g., "14:30:00".

// Literal Characters
@param %% A literal '%' character

MOSS_API const char* Moss_FormatTime(const char* fmt);


Moss_DateTimeToTime
Moss_GetCurrentTime
Moss_GetDateTimeLocalePreferences
Moss_GetDayOfWeek
Moss_GetDayOfYear
Moss_GetDaysInMonth
Moss_TimeFromWindows
Moss_TimeToDateTime
Moss_TimeToWindows




AddTimer
AddTimerNS
Delay
DelayNS
DelayPrecise
GetPerformanceCounter
GetPerformanceFrequency
GetTicks
GetTicksNS
RemoveTimer

#define US_PER_SECOND   1000000
#define MS_PER_SECOND   1000
#define NS_PER_MS       1000000
#define NS_PER_SECOND   1000000000LL
#define NS_PER_US       1000
#define NS_TO_MS(NS)        ((NS) / NS_PER_MS)
#define MS_TO_NS(MS)        (((Uint64)(MS)) * NS_PER_MS)
#define NS_TO_SECONDS(NS)   ((NS) / NS_PER_SECOND)
#define NS_TO_US(NS)        ((NS) / NS_PER_US)
#define SECONDS_TO_NS(S)    (((Uint64)(S)) * NS_PER_SECOND)
#define US_TO_NS(US)        (((Uint64)(US)) * NS_PER_US)


typedef Uint64 (MOSS_CALL *NSTimerCallback)(void *userdata, TimerID timerID, Uint64 interval);
typedef Uint32 (MOSS_CALL *TimerCallback)(void *userdata, TimerID timerID, Uint32 interval);
*/

// Convert angle in radians to the range \f$[-\pi, \pi]\f$
inline float CenterAngleAroundZero(float inV) { 
	if (inV < -MOSS_PI)
	{
		do
			inV += 2.0f * MOSS_PI;
		while (inV < -MOSS_PI);
	}
	else if (inV > MOSS_PI)
	{
		do
			inV -= 2.0f * MOSS_PI;
		while (inV > MOSS_PI);
	}
	MOSS_ASSERT(inV >= -MOSS_PI && inV <= MOSS_PI);
	return inV;
}

// Clamp a value between two values
template <typename T>
MOSS_INLINE constexpr T Clamp(T inV, T inMin, T inMax) { return min(max(inV, inMin), inMax); }

template <typename T>
MOSS_INLINE constexpr T Lerp(T inV, T inMin, T inMax) { return ((a) + (t) * ((b) - (a))) }

template<class T> inline void Swap(T& a, T& b) { T t = a; a = b; b = t; }

template<class T> inline T Sqr(T a) { return a * a; }

// Square a value
template <typename T>
MOSS_INLINE constexpr T Square(T inV) { return inV * inV; }

/// Returns \f$inV^3\f$.
template <typename T>
MOSS_INLINE constexpr T Cubed(T inV) { return inV * inV * inV; }

// Get the sign of a value
template <typename T>
MOSS_INLINE constexpr T Sign(T inV) { return inV < 0? T(-1) : T(1); }

// Check if inV is a power of 2
template <typename T>
constexpr bool IsPowerOf2(T inV) { return (inV & (inV - 1)) == 0; }

// Align inV up to the next inAlignment bytes
template <typename T>
inline T AlignUp(T inV, uint64 inAlignment) {
	MOSS_ASSERT(IsPowerOf2(inAlignment));
	return T((uint64(inV) + inAlignment - 1) & ~(inAlignment - 1));
}

// Check if inV is inAlignment aligned
template <typename T>
inline bool IsAligned(T inV, uint64 inAlignment) {
	MOSS_ASSERT(IsPowerOf2(inAlignment));
	return (uint64(inV) & (inAlignment - 1)) == 0;
}

// Compute number of trailing zero bits (how many low bits are zero)
inline uint32 CountTrailingZeros(uint32 inValue) {
#if defined(MOSS_CPU_X86) || defined(MOSS_CPU_WASM)
	#if defined(MOSS_SIMD_TZCNT)
		return _tzcnt_u32(inValue);
	#elif defined(MOSS_COMPILER_MSVC)
		if (inValue == 0)
			return 32;
		unsigned long result;
		_BitScanForward(&result, inValue);
		return result;
	#else
		if (inValue == 0)
			return 32;
		return __builtin_ctz(inValue);
	#endif
#elif defined(MOSS_CPU_ARM)
	#if defined(MOSS_COMPILER_MSVC)
		if (inValue == 0)
			return 32;
		unsigned long result;
		_BitScanForward(&result, inValue);
		return result;
	#else
		if (inValue == 0)
			return 32;
		return __builtin_ctz(inValue);
	#endif
#elif defined(MOSS_CPU_E2K)
		return inValue ? __builtin_ctz(inValue) : 32;
#else
	#error Undefined
#endif

}

// Compute the number of leading zero bits (how many high bits are zero)
inline uint32 CountLeadingZeros(uint32 inValue) {

#if defined(MOSS_CPU_X86) || defined(MOSS_CPU_WASM)
	#if defined(MOSS_SIMD_LZCNT)
		return _lzcnt_u32(inValue);
	#elif defined(MOSS_COMPILER_MSVC)
		if (inValue == 0)
			return 32;
		unsigned long result;
		_BitScanReverse(&result, inValue);
		return 31 - result;
	#else
		if (inValue == 0)
			return 32;
		return __builtin_clz(inValue);
	#endif
#elif defined(MOSS_CPU_ARM)
	#if defined(MOSS_COMPILER_MSVC)
		return _CountLeadingZeros(inValue);
	#else
		return __builtin_clz(inValue);
	#endif
#elif defined(MOSS_CPU_E2K)
		return inValue ? __builtin_clz(inValue) : 32;
#else
	#error Undefined
#endif

}

// Count the number of 1 bits in a value
inline uint32 CountBits(uint32 inValue) {

#if defined(MOSS_COMPILER_CLANG) || defined(MOSS_COMPILER_GCC)
	return __builtin_popcount(inValue);
#elif defined(MOSS_COMPILER_MSVC)
	#if defined(MOSS_SIMD_SSE4_2)
		return _mm_popcnt_u32(inValue);
	#elif defined(MOSS_SIMD_NEON) && (_MSC_VER >= 1930) // _CountOneBits not available on MSVC2019
		return _CountOneBits(inValue);
	#else
		inValue = inValue - ((inValue >> 1) & 0x55555555);
		inValue = (inValue & 0x33333333) + ((inValue >> 2) & 0x33333333);
		inValue = (inValue + (inValue >> 4)) & 0x0F0F0F0F;
		return (inValue * 0x01010101) >> 24;
	#endif
#else
	return 0;
#endif
}

// Get the next higher power of 2 of a value, or the value itself if the value is already a power of 2
inline uint32 GetNextPowerOf2(uint32 inValue) {
	return inValue <= 1? uint32(1) : uint32(1) << (32 - CountLeadingZeros(inValue - 1));
}

// Simple implementation of C++20 std::bit_cast (unfortunately not constexpr)
template <class To, class From>
MOSS_INLINE To BitCast(const From &inValue) {
	static_assert(std::is_trivially_constructible_v<To>);
	static_assert(sizeof(From) == sizeof(To));

	union FromTo
	{
		To			mTo;
		From		mFrom;
	};

	FromTo convert;
	convert.mFrom = inValue;
	return convert.mTo;
}


// Atomically compute the min(ioAtomic, inValue) and store it in ioAtomic, returns true if value was updated
template <class T>
bool AtomicMin(atomic<T> &ioAtomic, const T inValue, const memory_order inMemoryOrder = memory_order_seq_cst) {
	T cur_value = ioAtomic.load(memory_order_relaxed);
	while (cur_value > inValue)
		if (ioAtomic.compare_exchange_weak(cur_value, inValue, inMemoryOrder))
			return true;
	return false;
}

// Atomically compute the max(ioAtomic, inValue) and store it in ioAtomic, returns true if value was updated
template <class T>
bool AtomicMax(atomic<T> &ioAtomic, const T inValue, const memory_order inMemoryOrder = memory_order_seq_cst) {
	T cur_value = ioAtomic.load(memory_order_relaxed);
	while (cur_value < inValue)
		if (ioAtomic.compare_exchange_weak(cur_value, inValue, inMemoryOrder))
			return true;
	return false;
}

#ifdef MOSS_SIMD_NEON
// Constructing NEON values
#ifdef MOSS_COMPILER_MSVC
	#define MOSS_NEON_INT32x4(v1, v2, v3, v4) { int64_t(v1) + (int64_t(v2) << 32), int64_t(v3) + (int64_t(v4) << 32) }
	#define MOSS_NEON_UINT32x4(v1, v2, v3, v4) { uint64_t(v1) + (uint64_t(v2) << 32), uint64_t(v3) + (uint64_t(v4) << 32) }
	#define MOSS_NEON_INT8x16(v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16) { int64_t(v1) + (int64_t(v2) << 8) + (int64_t(v3) << 16) + (int64_t(v4) << 24) + (int64_t(v5) << 32) + (int64_t(v6) << 40) + (int64_t(v7) << 48) + (int64_t(v8) << 56), int64_t(v9) + (int64_t(v10) << 8) + (int64_t(v11) << 16) + (int64_t(v12) << 24) + (int64_t(v13) << 32) + (int64_t(v14) << 40) + (int64_t(v15) << 48) + (int64_t(v16) << 56) }
	#define MOSS_NEON_UINT8x16(v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16) { uint64_t(v1) + (uint64_t(v2) << 8) + (uint64_t(v3) << 16) + (uint64_t(v4) << 24) + (uint64_t(v5) << 32) + (uint64_t(v6) << 40) + (uint64_t(v7) << 48) + (uint64_t(v8) << 56), uint64_t(v9) + (uint64_t(v10) << 8) + (uint64_t(v11) << 16) + (uint64_t(v12) << 24) + (uint64_t(v13) << 32) + (uint64_t(v14) << 40) + (uint64_t(v15) << 48) + (uint64_t(v16) << 56) }
#else
	#define MOSS_NEON_INT32x4(v1, v2, v3, v4) { v1, v2, v3, v4 }
	#define MOSS_NEON_UINT32x4(v1, v2, v3, v4) { v1, v2, v3, v4 }
	#define MOSS_NEON_INT8x16(v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16) { v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16 }
	#define MOSS_NEON_UINT8x16(v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16) { v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16 }
#endif

// MSVC and GCC prior to version 12 don't define __builtin_shufflevector
#if defined(MOSS_COMPILER_MSVC) || (defined(MOSS_COMPILER_GCC) && __GNUC__ < 12)
#else
	// Shuffle a vector
	#define MOSS_NEON_SHUFFLE_F32x4(vec1, vec2, index1, index2, index3, index4) __builtin_shufflevector(vec1, vec2, index1, index2, index3, index4)
	#define MOSS_NEON_SHUFFLE_U32x4(vec1, vec2, index1, index2, index3, index4) __builtin_shufflevector(vec1, vec2, index1, index2, index3, index4)
#endif

#if defined(MOSS_COMPILER_MSVC) || (defined(MOSS_COMPILER_GCC) && __GNUC__ < 12)
// Generic shuffle vector template
template <unsigned I1, unsigned I2, unsigned I3, unsigned I4>
MOSS_INLINE float32x4_t NeonShuffleFloat32x4(float32x4_t inV1, float32x4_t inV2) {
	float32x4_t ret;
	ret = vmovq_n_f32(vgetq_lane_f32(I1 >= 4? inV2 : inV1, I1 & 0b11));
	ret = vsetq_lane_f32(vgetq_lane_f32(I2 >= 4? inV2 : inV1, I2 & 0b11), ret, 1);
	ret = vsetq_lane_f32(vgetq_lane_f32(I3 >= 4? inV2 : inV1, I3 & 0b11), ret, 2);
	ret = vsetq_lane_f32(vgetq_lane_f32(I4 >= 4? inV2 : inV1, I4 & 0b11), ret, 3);
	return ret;
}

	// Specializations
template <>
MOSS_INLINE float32x4_t NeonShuffleFloat32x4<0, 1, 2, 2>(float32x4_t inV1, float32x4_t inV2) { return vcombine_f32(vget_low_f32(inV1), vdup_lane_f32(vget_high_f32(inV1), 0)); }

template <>
MOSS_INLINE float32x4_t NeonShuffleFloat32x4<0, 1, 3, 3>(float32x4_t inV1, float32x4_t inV2) { return vcombine_f32(vget_low_f32(inV1), vdup_lane_f32(vget_high_f32(inV1), 1)); }

template <>
MOSS_INLINE float32x4_t NeonShuffleFloat32x4<0, 1, 2, 3>(float32x4_t inV1, float32x4_t inV2) { return inV1; }

template <>
MOSS_INLINE float32x4_t NeonShuffleFloat32x4<1, 0, 3, 2>(float32x4_t inV1, float32x4_t inV2) { return vcombine_f32(vrev64_f32(vget_low_f32(inV1)), vrev64_f32(vget_high_f32(inV1))); }

template <>
MOSS_INLINE float32x4_t NeonShuffleFloat32x4<2, 2, 1, 0>(float32x4_t inV1, float32x4_t inV2) { return vcombine_f32(vdup_lane_f32(vget_high_f32(inV1), 0), vrev64_f32(vget_low_f32(inV1))); }

template <>
MOSS_INLINE float32x4_t NeonShuffleFloat32x4<2, 3, 0, 1>(float32x4_t inV1, float32x4_t inV2) { return vcombine_f32(vget_high_f32(inV1), vget_low_f32(inV1)); }

// Used extensively by cross product
template <>
MOSS_INLINE float32x4_t NeonShuffleFloat32x4<1, 2, 0, 0>(float32x4_t inV1, float32x4_t inV2) {
	static uint8x16_t table = MOSS_NEON_UINT8x16(0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x00, 0x01, 0x02, 0x03, 0x00, 0x01, 0x02, 0x03);
	return vreinterpretq_f32_u8(vqtbl1q_u8(vreinterpretq_u8_f32(inV1), table));
}

// Shuffle a vector
#define MOSS_NEON_SHUFFLE_F32x4(vec1, vec2, index1, index2, index3, index4) NeonShuffleFloat32x4<index1, index2, index3, index4>(vec1, vec2)
#define MOSS_NEON_SHUFFLE_U32x4(vec1, vec2, index1, index2, index3, index4) vreinterpretq_u32_f32((NeonShuffleFloat32x4<index1, index2, index3, index4>(vreinterpretq_f32_u32(vec1), vreinterpretq_f32_u32(vec2))))
#endif
#endif // MOSS_SIMD_NEON



void Moss_Create_BumpAlloc(BumpAlloc* alloc, size_t size)
{
    alloc->buffer = (char*)malloc(size);
    alloc->offset = 0;
    alloc->capacity = size;
}

void* Moss_BumpAlloc(BumpAlloc* ba, size_t size)
{
    if (alloc->offset + size > alloc->capacity) return NULL;
    void* ptr = alloc->buffer + alloc->offset;
    alloc->offset += size;
    return ptr
}

void reset_BumpAlloc(BumpAlloc* alloc) { alloc->offset = 0; }
void free_BumpAlloc(BumpAlloc* alloc) { free(alloc->buffer); }



long long Moss_get_FileTimeStamp(char* file)
{
    struct stat file_stat = {};
    stat(file, &file_stat);
    return file_stat.st_mtime;
}

bool Moss_FileExists(char* filepath)
{
    MOSS_ERROR(filepath, "No filepath supplied.");

    int file = fopen(filepath, "rb");
    if (!file) { return false; }
    
    fclose(file);

    return true;
}

long Moss_get_FileSize(char* filepath)
{
    MOSS_ERROR(filepath, "No filepath supplied.");

    long fileSize = 0;
    int file = fopen(filepath, "rb");
    if (!file) { MOSS_ERROR(filepath, "Failed opening file: %s", filepath); return 0; }

    fseek(file, 0, SEEK_END);
    fileSize = ftell(file);

    fseek(file, 0, SEEK_SET);
    fclose(file);

    return fileSize;
}

/*
* Reads a file into a supplied buffer. We manage our own
* memory and therefore want more control over where it 
* is allocated
*/
char* Moss_FileRead(char* filepath, int fileSize, char* buffer)
{
    MOSS_ERROR(filePath, "No filePath supplied!");
    MOSS_ERROR(fileSize, "No fileSize supplied!");
    MOSS_ERROR(buffer, "No buffer supplied!");

    *fileSize = 0;
    auto file = fopen(filePath, "rb");
    if(!file)
    {
        MOSS_ERROR("Failed opening File: %s", filePath);
        return nullptr;
    }

    fseek(file, 0, SEEK_END);
    *fileSize = ftell(file);
    fseek(file, 0, SEEK_SET);

    memset(buffer, 0, *fileSize + 1);
    fread(buffer, sizeof(char), *fileSize, file);

    fclose(file);

    return buffer;
}


char* read_file(const char* filePath, int* fileSize, BumpAlloc* BumpAlloc)
{
  char* file = nullptr;
  long fileSize2 = get_file_size(filePath);

  if(fileSize2)
  {
    char* buffer = bump_alloc(BumpAlloc, fileSize2 + 1);

    file = read_file(filePath, fileSize, buffer);
  }

  return file; 
}

void Moss_FileWrite(char* filepath, char* buffer, int size)
{
    MOSS_ERROR(filePath, "No filePath supplied!");
    MOSS_ERROR(buffer, "No buffer supplied!");
    auto file = fopen(filePath, "wb");
    if(!file)
    {
        MOSS_ERROR("Failed opening File: %s", filePath);
        return;
    }

    fwrite(buffer, sizeof(char), size, file);
    fclose(file);
}



bool copy_file(const char* fileName, const char* outputName, char* buffer)
{
  int fileSize = 0;
  char* data = read_file(fileName, &fileSize, buffer);

  auto outputFile = fopen(outputName, "wb");
  if(!outputFile)
  {
    MOSS_ERROR("Failed opening File: %s", outputName);
    return false;
  }

  int result = fwrite(data, sizeof(char), fileSize, outputFile);
  if(!result)
  {
    MOSS_ERROR("Failed opening File: %s", outputName);
    return false;
  }
  
  fclose(outputFile);

  return true;
}

bool copy_file(const char* fileName, const char* outputName, BumpAlloc* BumpAlloc)
{
  char* file = 0;
  long fileSize2 = get_file_size(fileName);

  if(fileSize2)
  {
    char* buffer = bump_alloc(BumpAlloc, fileSize2 + 1);

    return copy_file(fileName, outputName, buffer);
  }

  return false;
}

MOSS_SUPRESS_WARNINGS_END

#endif // MOSS_STDINC_H


/// Find the roots of \f$inA \: x^2 + inB \: x + inC = 0\f$.
/// @return The number of roots, actual roots in outX1 and outX2.
/// If number of roots returned is 1 then outX1 == outX2.
template <typename T>
inline int FindRoot(const T inA, const T inB, const T inC, T &outX1, T &outX2) {
	// Check if this is a linear equation
	if (inA == T(0))
	{
		// Check if this is a constant equation
		if (inB == T(0))
			return 0;

		// Linear equation with 1 solution
		outX1 = outX2 = -inC / inB;
		return 1;
	}

	// See Numerical Recipes in C, Chapter 5.6 Quadratic and Cubic Equations
	T det = Square(inB) - T(4) * inA * inC;
	if (det < T(0))
		return 0;
	T q = (inB + Sign(inB) * sqrt(det)) / T(-2);
	outX1 = q / inA;
	if (q == T(0))
	{
		outX2 = outX1;
		return 1;
	}
	outX2 = inC / q;
	return 2;
}

/*
MOSS_SUPPRESS_WARNINGS_STD_BEGIN
#include <cstdlib>
MOSS_SUPPRESS_WARNINGS_STD_END
#include <stdlib.h>

MOSS_SUPRESS_WARNINGS_BEGIN

#ifdef MOSS_DISABLE_CUSTOM_ALLOCATOR
	#define MOSS_ALLOC_FN(x)	x
	#define MOSS_ALLOC_SCOPE
#else
	#define MOSS_ALLOC_FN(x)	x##Impl
	#define MOSS_ALLOC_SCOPE static
#endif

MOSS_ALLOC_SCOPE void *MOSS_ALLOC_FN(Allocate)(size_t inSize)
{
	MOSS_ASSERT(inSize > 0);
	return malloc(inSize);
}

MOSS_ALLOC_SCOPE void *MOSS_ALLOC_FN(Reallocate)(void *inBlock, [[maybe_unused]] size_t inOldSize, size_t inNewSize)
{
	MOSS_ASSERT(inNewSize > 0);
	return realloc(inBlock, inNewSize);
}

MOSS_ALLOC_SCOPE void MOSS_ALLOC_FN(Free)(void *inBlock)
{
	free(inBlock);
}

MOSS_ALLOC_SCOPE void *MOSS_ALLOC_FN(AlignedAllocate)(size_t inSize, size_t inAlignment)
{
	MOSS_ASSERT(inSize > 0 && inAlignment > 0);

#if defined(MOSS_PLATFORM_WINDOWS)
	// Microsoft doesn't implement posix_memalign
	return _aligned_malloc(inSize, inAlignment);
#else
	void *block = nullptr;
	MOSS_SUPPRESS_WARNING_PUSH
	MOSS_GCC_SUPPRESS_WARNING("-Wunused-result")
	MOSS_CLANG_SUPPRESS_WARNING("-Wunused-result")
	posix_memalign(&block, inAlignment, inSize);
	MOSS_SUPPRESS_WARNING_POP
	return block;
#endif
}

MOSS_ALLOC_SCOPE void MOSS_ALLOC_FN(AlignedFree)(void *inBlock)
{
#if defined(MOSS_PLATFORM_WINDOWS)
	_aligned_free(inBlock);
#else
	free(inBlock);
#endif
}

#ifndef MOSS_DISABLE_CUSTOM_ALLOCATOR

AllocateFunction Allocate = nullptr;
ReallocateFunction Reallocate = nullptr;
FreeFunction Free = nullptr;
AlignedAllocateFunction AlignedAllocate = nullptr;
AlignedFreeFunction AlignedFree = nullptr;

void RegisterDefaultAllocator()
{
	Allocate = AllocateImpl;
	Reallocate = ReallocateImpl;
	Free = FreeImpl;
	AlignedAllocate = AlignedAllocateImpl;
	AlignedFree = AlignedFreeImpl;
}
#endif // MOSS_DISABLE_CUSTOM_ALLOCATOR




using std::expf;
using std::itoa;
using std::lltoa;
using std::asprintf;
using std::inff;
using std::ltoa;
bsearch
bsearch_r
copysign
copysignf
crc16
crc32
getenv
getenv_unsafe
GetMemoryFunctions
GetNumAllocations
GetOriginalMemoryFunctions
iconv
iconv_close
iconv_open
iconv_string
isalnum
isalpha
isblank
iscntrl
isdigit
isgraph
islower
isprint
ispunct
isspace
isupper
isxdigit
murmur3_32
qsort_r
rand
rand_bits
rand_bits_r
rand_r
randf
randf_r
realloc
round
roundf
scalbn
scalbnf
setenv_unsafe
SetMemoryFunctions
size_add_check_overflow
size_mul_check_overflow
snprintf
StepBackUTF8
StepUTF8
strcasecmp
strcasestr
strchr
strcmp
strdup
strlcat
strlcpy
strlen
strlwr
strncasecmp
strncmp
strndup
strnlen
strnstr
strpbrk
strrchr
strrev
strstr
strtod
strtok_r
strtol
strtoll
strtoul
strtoull
strupr
swprintf
tolower
toupper
trunc
truncf
UCS4ToUTF8
uitoa
ulltoa
ultoa
unsetenv_unsafe
utf8strlcpy
utf8strlen
utf8strnlen
vasprintf
vsnprintf
vsscanf
vswprintf
wcscasecmp
wcscmp
wcsdup
wcslcat
wcslcpy
wcslen
wcsncasecmp
wcsncmp
wcsnlen
wcsnstr
wcsstr
wcstol


typedef int64 Moss_Time;
#define MOSS_MAX_TIME MOSS_MAX_SINT64
#define MOSS_MIN_TIME MOSS_MIN_SINT64

typedef void *(MOSSCALL *MOSS_realloc_func)(void *mem, size_t size);
typedef void *(MOSSCALL *MOSS_malloc_func)(size_t size);
typedef void (MOSSCALL *MOSS_free_func)(void *mem);
typedef void (*MOSS_FunctionPointer)(void);
typedef int (MOSSCALL *MOSS_CompareCallback_r)(void *userdata, const void *a, const void *b);
typedef int (MOSSCALL *MOSS_CompareCallback)(const void *a, const void *b);
typedef void *(MOSSCALL *MOSS_calloc_func)(size_t nmemb, size_t size);



#define MOSS_const_cast(type, expression) const_cast<type>(expression)
#define MOSS_iconv_utf8_locale(S)    MOSS_iconv_string("", "UTF-8", S, MOSS_strlen(S)+1)
#define MOSS_iconv_utf8_ucs2(S)      MOSS_reinterpret_cast(Uint16 *, MOSS_iconv_string("UCS-2", "UTF-8", S, MOSS_strlen(S)+1))
#define MOSS_iconv_utf8_ucs4(S)      MOSS_reinterpret_cast(Uint32 *, MOSS_iconv_string("UCS-4", "UTF-8", S, MOSS_strlen(S)+1))
#define MOSS_iconv_wchar_utf8(S)     MOSS_iconv_string("UTF-8", "WCHAR_T", MOSS_reinterpret_cast(const char *, S), (MOSS_wcslen(S)+1)*sizeof(wchar_t))
#define MOSS_PRINTF_FORMAT_STRING _Printf_format_string_
#define MOSS_PRINTF_VARARG_FUNC( fmtargnumber ) __attribute__ (( format( __printf__, fmtargnumber, fmtargnumber+1 )))
#define MOSS_PRINTF_VARARG_FUNCV( fmtargnumber ) __attribute__(( format( __printf__, fmtargnumber, 0 )))
#define MOSS_reinterpret_cast(type, expression) reinterpret_cast<type>(expression)
#define MOSS_SCANF_FORMAT_STRING _Scanf_format_string_impl_
#define MOSS_SCANF_VARARG_FUNC( fmtargnumber ) __attribute__ (( format( __scanf__, fmtargnumber, fmtargnumber+1 )))
#define MOSS_SCANF_VARARG_FUNCV( fmtargnumber ) __attribute__(( format( __scanf__, fmtargnumber, 0 )))
#define MOSS_stack_alloc(type, count)    (type*)alloca(sizeof(type)*(count))
#define MOSS_stack_free(data)
#define MOSS_static_cast(type, expression) static_cast<type>(expression)
#define MOSS_STRINGIFY_ARG(arg)  #arg
#define MOSS_zero(x) MOSS_memset(&(x), 0, sizeof((x)))
#define MOSS_zeroa(x) MOSS_memset((x), 0, sizeof((x)))
#define MOSS_zerop(x) MOSS_memset((x), 0, sizeof(*(x)))




typedef int Moss_SpinLock;

#define Moss_AtomicDecRef(a)    (Moss_AddAtomicInt(a, -1) == 1)
#define Moss_AtomicIncRef(a)    Moss_AddAtomicInt(a, 1)

int Moss_AddAtomicInt(Moss_AtomicInt *a, int v);
Uint32 Moss_AddAtomicU32(Moss_AtomicU32 *a, int v);
bool Moss_CompareAndSwapAtomicInt(Moss_AtomicInt *a, int oldval, int newval);
bool Moss_CompareAndSwapAtomicPointer(void **a, void *oldval, void *newval);
bool Moss_CompareAndSwapAtomicU32(Moss_AtomicU32 *a, Uint32 oldval, Uint32 newval);
int Moss_GetAtomicInt(Moss_AtomicInt *a);
void * Moss_GetAtomicPointer(void **a);
Uint32 Moss_GetAtomicU32(Moss_AtomicU32 *a);
void Moss_LockSpinlock(Moss_SpinLock *lock);
void Moss_MemoryBarrierAcquireFunction(void);
void Moss_MemoryBarrierReleaseFunction(void);
int Moss_SetAtomicInt(Moss_AtomicInt *a, int v);
void * Moss_SetAtomicPointer(void **a, void *v);
void Moss_MemoryBarrierReleaseFunction(void);
bool Moss_TryLockSpinlock(Moss_SpinLock *lock);
Uint32 Moss_SetAtomicU32(Moss_AtomicU32 *a, Uint32 v);
void Moss_UnlockSpinlock(Moss_SpinLock *lock);
*/