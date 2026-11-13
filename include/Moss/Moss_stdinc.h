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
 * MOSS_PLATFORM_EMBEDDED
 * MOSS_PLATFORM_TERRARIUM
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
 * #define MOSS_API       extern "C" __declspec(dllexport)
 * #define MOSS_API       extern "C" __declspec(dllimport)
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

// Add SIMD, Multithreading

#ifndef MOSS_STDINC_H
#define MOSS_STDINC_H

// Determine platform
#if defined(MOSS_PLATFORM_BLUE)
	// Correct define already defined, this overrides everything else
	#define MOSS_PLATFORM_EMBEDDED
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
			#define MOSS_PLATFORM_EMBEDDED
        #else
            #define MOSS_PLATFORM_XBOXONE
			#define MOSS_PLATFORM_EMBEDDED
        #endif
    #endif
#elif defined(__APPLE__)
    #include <TargetConditionals.h>
    #define MOSS_PLATFORM_APPLE

    #if TARGET_OS_OSX
        #define MOSS_PLATFORM_MACOS
    #elif TARGET_OS_IPHONE
        #define MOSS_PLATFORM_IOS
		#define MOSS_PLATFORM_EMBEDDED
    #elif TARGET_OS_TV
        #define MOSS_PLATFORM_TVOS
		#define MOSS_PLATFORM_EMBEDDED
    #elif TARGET_OS_VISION
        #define MOSS_PLATFORM_VISIONOS
    #endif
#elif defined(ANDROID) || defined(__ANDROID__) 	// Android is linux too, so that's why we check it first
	#define MOSS_PLATFORM_ANDROID
	#define MOSS_PLATFORM_EMBEDDED
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
#elif defined(__terrarium__)
	#define MOSS_PLATFORM_TERRARIUM

	#if defined(TERRARIUM_DEVICE_RPI)
		#define MOSS_PLATFORM_EMBEDDED
		#define MOSS_LOW_MEMORY
	#elif defined(TERRARIUM_DEVICE_CONSOLE)
		#define MOSS_PLATFORM_TERRARIUM_CONSOL
	#elif defined(TERRARIUM_DEVICE_TABLET)
		#define MOSS_PLATFORM_TERRARIUM_TABLET
	#elif defined(TERRARIUM_DEVICE_TV)
		#define MOSS_PLATFORM_TERRARIUM_TV
	#elif defined(TERRARIUM_DEVICE_XR)
		#define MOSS_PLATFORM_TERRARIUM_XR
	#else
		#error "Unknown TerrariumOS build macro"
	#endif
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
		#undef MOSS_DVECTOR_ALIGNMENT
		#define MOSS_DVECTOR_ALIGNMENT 64 // Optimize double-precision vectors for AVX-512 ZMM registers
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
		#if defined(__ARM_FEATURE_FMA) && !defined(MOSS_CROSS_PLATFORM_DETERMINISTIC)
			#define MOSS_USE_FMADD
		#endif
		#if defined(__ARM_FEATURE_SVE)
			#define MOSS_SIMD_SVE
		#endif
	#else
		#define MOSS_CPU_ADDRESS_BITS 32
		#define MOSS_VECTOR_ALIGNMENT 8 // 32-bit ARM does not support aligning on the stack on 16 byte boundaries
		#define MOSS_DVECTOR_ALIGNMENT 8
		#if defined(__ARM_NEON)
			#include <arm_neon.h>
			#define MOSS_SIMD_NEON
		#else
			#include <immintrin.h>
		#endif
	#endif // __aarch64__ || _M_ARM64
#elif defined(__riscv)
	// RISC-V CPU architecture
	#define MOSS_CPU_RISCV
	#if __riscv_xlen == 64
		#define MOSS_CPU_ADDRESS_BITS 64
	#else
		#define MOSS_CPU_ADDRESS_BITS 32
	#endif

	// RISC-V Vector (RVV) Standard Detection
	#if defined(__riscv_vector) || defined(__riscv_v_intrinsic)
		#define MOSS_SIMD_RISCV_VECTOR
		#define MOSS_VECTOR_ALIGNMENT 32   // Align high for VLA structures
		#define MOSS_DVECTOR_ALIGNMENT 64
	#else
		#define MOSS_VECTOR_ALIGNMENT 16
		#define MOSS_DVECTOR_ALIGNMENT 32
	#endif

	// Fused Multiply-Add is built inherently into standard single/double float extensions (F/D)
	#if (defined(__riscv_flen) && __riscv_flen >= 32) && !defined(MOSS_CROSS_PLATFORM_DETERMINISTIC)
		#define MOSS_USE_FMADD
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
		#define MOSS_SIMD_WASM128
	#endif
#elif (defined(__powerpc__) || defined(__powerpc64__) || defined(__ppc64__))
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

	// Map PowerPC Vector Engine tiers
	#if defined(__VSX__)
		#define MOSS_SIMD_VSX
		#define MOSS_SIMD_ALTIVEC
		#define MOSS_VECTOR_ALIGNMENT 16
		#define MOSS_DVECTOR_ALIGNMENT 32
		#ifndef MOSS_CROSS_PLATFORM_DETERMINISTIC
			#define MOSS_USE_FMADD // VSX introduces hardware float FMA
		#endif
	#elif defined(__ALTIVEC__)
		#define MOSS_SIMD_ALTIVEC
		#define MOSS_VECTOR_ALIGNMENT 16
		#define MOSS_DVECTOR_ALIGNMENT 16
	#endif
#elif defined(__loongarch__)
	// LoongArch CPU architecture
	#define MOSS_CPU_LOONGARCH
	#if defined(__loongarch64)
		#define MOSS_CPU_ADDRESS_BITS 64
	#else
		#define MOSS_CPU_ADDRESS_BITS 32
	#endif

	// Map Loongson Vector Extensions
	#if defined(__loongarch_asx)
		#define MOSS_SIMD_LASX
		#define MOSS_SIMD_LSX
		#define MOSS_VECTOR_ALIGNMENT 32 // 256-bit alignment for LASX
		#define MOSS_DVECTOR_ALIGNMENT 32
	#elif defined(__loongarch_sx)
		#define MOSS_SIMD_LSX
		#define MOSS_VECTOR_ALIGNMENT 16 // 128-bit alignment for LSX
		#define MOSS_DVECTOR_ALIGNMENT 16
	#endif
	#if (defined(__loongarch_sx) || defined(__loongarch_asx)) && !defined(MOSS_CROSS_PLATFORM_DETERMINISTIC)
		#define MOSS_USE_FMADD
	#endif

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

#if defined(MOSS_SIMD_AVX) || defined(MOSS_SIMD_AVX2)
    #include <immintrin.h> // Pulls in SSE, AVX, AVX2, TZCNT, LZCNT
#elif defined(__SSE__) || defined(_M_AMD64) || defined(_M_X64)
    #include <xmmintrin.h>
#endif

// If this define is set, Moss is compiled as a shared library
#ifdef MOSS_SHARED_LIBRARY
	#ifdef MOSS_BUILD_SHARED_LIBRARY
		// While building the shared library, we must export these symbols
		#if defined(MOSS_PLATFORM_WINDOWS) && !defined(MOSS_COMPILER_MINGW)
			#define MOSS_API __declspec(dllexport)
		#else
			#define MOSS_API __attribute__ ((visibility ("default")))
			#if defined(MOSS_COMPILER_GCC)
				// Prevents an issue with GCC attribute parsing (see https://gcc.gnu.org/bugzilla/show_bug.cgi?id=69585)
				#define MOSS_API_GCC_BUG_WORKAROUND [[gnu::visibility("default")]]
			#endif
		#endif
	#else
		// When linking against Moss, we must import these symbols
		#if defined(MOSS_PLATFORM_WINDOWS) && !defined(MOSS_COMPILER_MINGW)
			#define MOSS_API __declspec(dllimport)
		#else
			#define MOSS_API __attribute__ ((visibility ("default")))
			#if defined(MOSS_COMPILER_GCC)
				// Prevents an issue with GCC attribute parsing (see https://gcc.gnu.org/bugzilla/show_bug.cgi?id=69585)
				#define MOSS_API_GCC_BUG_WORKAROUND [[gnu::visibility("default")]]
			#endif
		#endif
	#endif
#else
	// If the define is not set, we use static linking and symbols don't need to be imported or exported
	#define MOSS_API
#endif

#ifndef MOSS_API_GCC_BUG_WORKAROUND
	#define MOSS_API_GCC_BUG_WORKAROUND MOSS_API
#endif

// Macro used by the RTTI macros to not export a function
#define MOSS_NO_EXPORT

/////////////////////////////////////////////////////////////////////////

// Unified C++17 alignment decorators
#if defined(__cplusplus) && __cplusplus >= 201703L
    #define MOSS_ALIGNAS(x) alignas(x)
#elif defined(MOSS_COMPILER_MSVC)
    #define MOSS_ALIGNAS(x) __declspec(align(x))
#elif defined(MOSS_COMPILER_GCC) || defined(MOSS_COMPILER_CLANG)
    #define MOSS_ALIGNAS(x) __attribute__((aligned(x)))
#else
    #define MOSS_ALIGNAS(x)
#endif

// Visual hooks for structural declarations
#define MOSS_ALIGN_VECTOR  MOSS_ALIGNAS(MOSS_VECTOR_ALIGNMENT)
#define MOSS_ALIGN_DVECTOR MOSS_ALIGNAS(MOSS_DVECTOR_ALIGNMENT)

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


#define MOSS_SUPPRESS_WARNINGS_BEGIN 					\
	MOSS_SUPPRESS_WARNING_PUSH							\
	MOSS_SUPPRESS_WARNINGS

#define MOSS_SUPPRESS_WARNINGS_END 			MOSS_SUPPRESS_WARNING_POP


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

#ifndef MOSS_CALL
#if defined(MOSS_PLATFORM_WINDOWS) && !defined(__GNUC__)
#define MOSS_CALL __cdecl
#elif defined(MOSS_PLATFORM_WINDOWS) && defined(__GNUC__)
#define MOSS_CALL __attribute__((cdecl))
#else
#define MOSS_CALL
#endif
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

#if defined(NAN)
    #define MOSS_NAN NAN
#elif defined(INFINITY)
    /* Fallback: infinity divided by infinity yields NaN */
    #define MOSS_NAN (0.0 / 0.0)
#else
    /* Ultimate fallback bitwise/constant approach if needed */
    #define MOSS_NAN (0.0 / 0.0)
#endif

//////////////////////////////////////////////////////////////////////////


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
#if defined(MOSS_SIMD_NEON)
	#ifdef MOSS_COMPILER_MSVC
		#include <intrin.h>
		#include <arm64_neon.h>
	#else
		#include <arm_neon.h>
	#endif
#endif
MOSS_SUPPRESS_WARNINGS_STD_END


MOSS_SUPPRESS_WARNINGS_BEGIN
// Commonly used STL types
using std::min;
using std::max;
using std::abs;
using std::acos;
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
using std::srand;

using std::asinhf;
using std::acoshf;
using std::asinhf;
using std::atanhf;
using std::isinf;
using std::modf;

using std::nanf;
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


#ifndef MOSS_DISABLE_CUSTOM_ALLOCATOR

// Normal memory allocation, must be at least 8 byte aligned on 32 bit platform and 16 byte aligned on 64 bit platform
using AllocateFunction = void *(*)(size_t inSize);
using ReallocateFunction = void *(*)(void *inBlock, size_t inOldSize, size_t inNewSize);
using FreeFunction = void (*)(void *inBlock);

// Aligned memory allocation
using AlignedAllocateFunction = void *(*)(size_t inSize, size_t inAlignment);
using AlignedFreeFunction = void (*)(void *inBlock);

// User defined allocation / free functions
MOSS_API extern AllocateFunction Allocate;
MOSS_API extern ReallocateFunction Reallocate;
MOSS_API extern FreeFunction Free;
MOSS_API extern AlignedAllocateFunction AlignedAllocate;
MOSS_API extern AlignedFreeFunction AlignedFree;

// Register platform default allocation / free functions
MOSS_API void RegisterDefaultAllocator();

// Macro to override the new and delete functions
#define MOSS_OVERRIDE_NEW_DELETE \
	MOSS_INLINE void *operator new (size_t inCount)                         { return ::Allocate(inCount); } \
	MOSS_INLINE void operator delete (void *inPointer) noexcept             { ::Free(inPointer); } \
	MOSS_INLINE void *operator new[] (size_t inCount)                       { return ::Allocate(inCount); } \
	MOSS_INLINE void operator delete[] (void *inPointer) noexcept           { ::Free(inPointer); } \
	MOSS_INLINE void *operator new (size_t inCount, std::align_val_t inAlignment) { return ::AlignedAllocate(inCount, static_cast<size_t>(inAlignment)); } \
	MOSS_INLINE void operator delete (void *inPointer, [[maybe_unused]] std::align_val_t inAlignment) noexcept { ::AlignedFree(inPointer); } \
	MOSS_INLINE void *operator new[] (size_t inCount, std::align_val_t inAlignment) { return ::AlignedAllocate(inCount, static_cast<size_t>(inAlignment)); } \
	MOSS_INLINE void operator delete[] (void *inPointer, [[maybe_unused]] std::align_val_t inAlignment) noexcept { ::AlignedFree(inPointer); } \
	MOSS_INLINE void *operator new ([[maybe_unused]] size_t inCount, void *inPointer) noexcept { return inPointer; } \
	MOSS_INLINE void operator delete ([[maybe_unused]] void *inPointer, [[maybe_unused]] void *inPlace) noexcept { } \
	MOSS_INLINE void *operator new[] ([[maybe_unused]] size_t inCount, void *inPointer) noexcept { return inPointer; } \
	MOSS_INLINE void operator delete[] ([[maybe_unused]] void *inPointer, [[maybe_unused]] void *inPlace) noexcept { }

#else
// Directly define the allocation functions
MOSS_API void *Allocate(size_t inSize);
MOSS_API void *Reallocate(void *inBlock, size_t inOldSize, size_t inNewSize);
MOSS_API void Free(void *inBlock);
MOSS_API void *AlignedAllocate(size_t inSize, size_t inAlignment);
MOSS_API void AlignedFree(void *inBlock);

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
#define MOSS_LOG(msg, ...) 	 Moss_Log("LOG:   ", msg, "\x1b[37m", ##__VA_ARGS__); // White
#define MOSS_TRACE(msg, ...) Moss_Log("TRACE: ", msg, "\x1b[37m", ##__VA_ARGS__); // White
#define MOSS_INFO(msg, ...)  Moss_Log("INFO:  ", msg, "\x1b[32m", ##__VA_ARGS__); // Green
#define MOSS_WARN(msg, ...)  Moss_Log("WARN:  ", msg, "\x1b[33m", ##__VA_ARGS__); // Yellow
#define MOSS_ERROR(msg, ...) Moss_Log("ERROR: ", msg, "\x1b[31m", ##__VA_ARGS__); // Red
#define MOSS_FATAL(msg, ...) Moss_Log("FATAL: ", msg, "\x1b[35m", ##__VA_ARGS__); // Magenta

#define MOSS_CHECK(x, msg, ...)                                       \
    do {                                                              \
        if (!(x)) {                                                    \
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
    #error Unsupported OS
#endif

// Helper to support optional message
struct MossAssertLastParam {};
inline bool MossAssertHelper(const char* expr, const char* file, uint32_t line, MossAssertLastParam) {
    return MossAssertFailed(expr, nullptr, file, line);
}
inline bool MossAssertHelper(const char* expr, const char* file, uint32_t line, const char* msg, MossAssertLastParam) {
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

#define MOSS_IF_ENABLE_ASSERTS(...)  __VA_ARGS_
#else
	#define MOSS_LOG(msg, ...)  		((void)0)
	#define MOSS_TRACE(msg, ...)  		((void)0)
	#define MOSS_DEBUG(msg, ...)  		((void)0)
	#define MOSS_INFO(msg, ...)  		((void)0)
	#define MOSS_WARN(msg, ...)  		((void)0)
	#define MOSS_ERROR(msg, ...)  		((void)0)
	#define MOSS_FATAL(msg, ...)  		((void)0)
	#define MOSS_CHECK(x, msg, ...)  	((void)0)
	#define MOSS_ASSERT(...)  			((void)0)

	#define MOSS_IF_ENABLE_ASSERTS(...)
	#define DEBUG_BREAK() ((void)0)
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

#if defined(__GNUC__) || defined(__clang__)
    // rw: 0 = read, 1 = write
    // locality: 0 = no temporal locality (NTA), 1 = low, 2 = moderate, 3 = high (T0)
    #define MOSS_PREFETCH(addr, rw, locality) __builtin_prefetch((addr), (rw), (locality))
#elif defined(_MSC_VER) && (defined(_M_IX86) || defined(_M_X64))
    #include <xmmintrin.h>
    #define MOSS_PREFETCH(addr, rw, locality) _mm_prefetch((const char*)(addr), (locality))
#else
    // Fallback no-op for unsupported compilers/architectures
    #define MOSS_PREFETCH(addr, rw, locality) ((void)0)
#endif

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
#define MOSS_LOG2E    			1.44269504088896340736F   // log2(e)
#define MOSS_LOG10E   			0.434294481903251827651F  // log10(e)
#define MOSS_LN2      			0.693147180559945309417F  // ln(2)
#define MOSS_LN10     			2.30258509299404568402F   // ln(10)
#define MOSS_2_SQRTPI 			1.12837916709551257390F   // 2/sqrt(pi)
#define MOSS_SQRT2    			1.41421356237309504880F   // sqrt(2)
#define MOSS_SQRT1_2  			0.707106781186547524401F  // 1/sqrt(2)
#define ATAN_POLY(x)  ((x) * (0.999866 + 0.333331 * (x) * (x)))
#define MOSS_FLT_EPSILON 	1.1920928955078125e-07F

#define RAD(x)        ((x) * (PI / 180.0))
#define MOSS_DIFFERENCE(x, y) ((x) < (y) ? (y) - (x) : (x) - (y))

#define ArraySize(x) (sizeof(x)) / (sizeof((x)[0]))

using int8 = signed char;
using int16 = signed short;
using int32 = signed int;
using int64 = signed long long;

using uint8 = unsigned char;
using uint16 = unsigned short;
using uint32_t = unsigned int;
using uint64 = unsigned long long;

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
#define MAX_UINT32  ((uint32_t)(0xFFFFFFFFu))
#define MAX_UINT64  ((uint64)(0xFFFFFFFFFFFFFFFF))
#define MIN_UINT8   ((uint8)0x00)
#define MIN_UINT16  ((uint16)0x0000)
#define MIN_UINT32  ((uint32_t)0x00000000)
#define MIN_UINT64  ((uint64)(0x0000000000000000))

// Unit conversion constants & macros

#define MOSS_US_PER_SECOND   1000000ULL
#define MOSS_MS_PER_SECOND   1000ULL
#define MOSS_NS_PER_US       1000ULL
#define MOSS_NS_PER_MS       1000000ULL
#define MOSS_NS_PER_SECOND   1000000000ULL

#define MOSS_NS_TO_MS(ns)        ((ns) / MOSS_NS_PER_MS)
#define MOSS_MS_TO_NS(ms)        ((uint64_t)(ms) * MOSS_NS_PER_MS)
#define MOSS_NS_TO_SECONDS(ns)   ((double)(ns) / (double)MOSS_NS_PER_SECOND)
#define MOSS_SECONDS_TO_NS(s)    ((uint64_t)((s) * (double)MOSS_NS_PER_SECOND))
#define MOSS_NS_TO_US(ns)        ((ns) / MOSS_NS_PER_US)
#define MOSS_US_TO_NS(us)        ((uint64_t)(us) * MOSS_NS_PER_US)

// Assert sizes of types
static_assert(sizeof(uint8) == 1, "Invalid size of uint8");
static_assert(sizeof(uint16) == 2, "Invalid size of uint16");
static_assert(sizeof(uint32_t) == 4, "Invalid size of uint32_t");
static_assert(sizeof(uint64) == 8, "Invalid size of uint64");
static_assert(sizeof(void *) == (MOSS_CPU_ADDRESS_BITS == 64? 8 : 4), "Invalid size of pointer" );

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

enum class Colorspace {
    UNKNOWN = 0,
    SRGB = 0x120005a0u,
    SRGB_LINEAR = 0x12000500u,
    HDR10 = 0x12002600u, 
    JPEG = 0x220004c6u,
    BT601_LIMITED = 0x211018c6u,
    BT601_FULL = 0x221018c6u,
    BT709_LIMITED = 0x21100421u,
    BT709_FULL = 0x22100421u, 
    BT2020_LIMITED = 0x21102609u,
    BT2020_FULL = 0x22102609u,

    RGB_DEFAULT = SRGB,
    YUV_DEFAULT = BT601_LIMITED
};

enum class PixelFormat {
    UNKNOWN = 0,
    INDEX1LSB = 0x11100100u,
    INDEX1MSB = 0x11200100u,
    INDEX2LSB = 0x1c100200u,
    INDEX2MSB = 0x1c200200u,
    INDEX4LSB = 0x12100400u,
    INDEX4MSB = 0x12200400u,
    INDEX8 = 0x13000801u,
    RGB332 = 0x14110801u,
    XRGB4444 = 0x15120c02u,
    XBGR4444 = 0x15520c02u,
    XRGB1555 = 0x15130f02u,
    XBGR1555 = 0x15530f02u,
    ARGB4444 = 0x15321002u,
    RGBA4444 = 0x15421002u,
    ABGR4444 = 0x15721002u,
    BGRA4444 = 0x15821002u,
    ARGB1555 = 0x15331002u,
    RGBA5551 = 0x15441002u,
    ABGR1555 = 0x15731002u,
    BGRA5551 = 0x15841002u,
    RGB565 = 0x15151002u,
    BGR565 = 0x15551002u,
    RGB24 = 0x17101803u,
    BGR24 = 0x17401803u,
    XRGB8888 = 0x16161804u,
    RGBX8888 = 0x16261804u,
    XBGR8888 = 0x16561804u,
    BGRX8888 = 0x16661804u,
    ARGB8888 = 0x16362004u,
    RGBA8888 = 0x16462004u,
    ABGR8888 = 0x16762004u,
    BGRA8888 = 0x16862004u,
    XRGB2101010 = 0x16172004u,
    XBGR2101010 = 0x16572004u,
    ARGB2101010 = 0x16372004u,
    ABGR2101010 = 0x16772004u,
    RGB48 = 0x18103006u,
    BGR48 = 0x18403006u,
    RGBA64 = 0x18204008u,
    ARGB64 = 0x18304008u,
    BGRA64 = 0x18504008u,
    ABGR64 = 0x18604008u,
    RGB48_FLOAT = 0x1a103006u,
    BGR48_FLOAT = 0x1a403006u,
    RGBA64_FLOAT = 0x1a204008u,
    ARGB64_FLOAT = 0x1a304008u,
    BGRA64_FLOAT = 0x1a504008u,
    ABGR64_FLOAT = 0x1a604008u,
    RGB96_FLOAT = 0x1b10600cu,
    BGR96_FLOAT = 0x1b40600cu,
    RGBA128_FLOAT = 0x1b208010u,
    ARGB128_FLOAT = 0x1b308010u,
    BGRA128_FLOAT = 0x1b508010u,
    ABGR128_FLOAT = 0x1b608010u,
    YV12 = 0x32315659u,
    IYUV = 0x56555949u,
    YUY2 = 0x32595559u,
    UYVY = 0x59565955u,
    YVYU = 0x55595659u,
    NV12 = 0x3231564eu,
    NV21 = 0x3132564eu,
    P010 = 0x30313050u,
    EXTERNAL_OES = 0x2053454fu,
    MJPG = 0x47504a4du,
};

enum {
	SWIZZLE_X = 0,			// Use the X component
	SWIZZLE_Y = 1,			// Use the Y component
	SWIZZLE_Z = 2,			// Use the Z component
	SWIZZLE_W = 3,			// Use the W component
	SWIZZLE_UNUSED = 2,		// We always use the Z component when we don't specifically want to initialize a value, this is consistent with what is done in Vec3(x, y, z), Vec3(Float3 &) and Vec3::LoadFloat3Unsafe
};



using Moss_TimerID = uint32_t;
using Moss_SpinLock = int;

#define MOSS_MAX_TIME MAX_INT64
#define MOSS_MIN_TIME MIN_INT64

using Moss_Time = int64_t;	// Nanoseconds since Unix epoch (Jan 1, 1970 UTC)

struct Moss_DateTime {
    int year;        // e.g., 2026
    int month;       // [1..12]
    int day;         // [1..31]
    int hour;        // [0..23]
    int minute;      // [0..59]
    int second;      // [0..60]
    int nanosecond;  // [0..999999999]
    int day_of_week; // [0..6] (0 = Sunday)
    int day_of_year; // [0..365]
    int utc_offset;  // Seconds east of UTC
};

struct Moss_RNG {
    uint64_t state;
    uint64_t inc;
};

// Wrap atomic types in structs so callers cannot accidentally perform
// non-atomic operations (like a->value++) without compiler errors.
struct Moss_AtomicInt { volatile int value; };
struct Moss_AtomicU32 { volatile uint32_t value; };
struct Moss_AtomicU64 { volatile uint64_t value; };

typedef uint64_t (MOSS_CALL *Moss_NSTimerCallback)(void* userdata, Moss_TimerID timerID, uint64_t intervalNS);
typedef uint32_t (MOSS_CALL *Moss_TimerCallback)(void* userdata, Moss_TimerID timerID, uint32_t intervalMS);

// Convert a value from degrees to radians
MOSS_INLINE constexpr float DegreesToRadians(float value) { return value * (MOSS_PI / 180.0f); }
// Convert a value from radians to degrees
MOSS_INLINE constexpr float RadiansToDegrees(float value) { return value * (180.0f / MOSS_PI); }

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
MOSS_INLINE constexpr T Lerp(T value, T inMin, T inMax) {  return ((inMin) + (value) * ((inMax) - (inMin))); }

template<class T> inline void Swap(T& a, T& b) { T t = a; a = b; b = t; }

template<class T> inline T Sqr(T a) { return a * a; }

// Square a value
template <typename T>
MOSS_INLINE constexpr T Square(T inV) { return inV * inV; }

// Returns \f$inV^3\f$.
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
inline uint32_t CountTrailingZeros(uint32_t inValue) {
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
inline uint32_t CountLeadingZeros(uint32_t inValue) {

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
inline uint32_t CountBits(uint32_t inValue) {

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
inline uint32_t GetNextPowerOf2(uint32_t inValue) {
	return inValue <= 1? uint32_t(1) : uint32_t(1) << (32 - CountLeadingZeros(inValue - 1));
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

// Reference counting helpers
#define Moss_AtomicIncRef(a)    Moss_AddAtomicInt(a, 1)
#define Moss_AtomicDecRef(a)    (Moss_AddAtomicInt(a, -1) == 1)

// --- Barriers & CPU Hints ---
void Moss_CompilerBarrier(void);
void Moss_MemoryBarrierAcquireFunction(void);
void Moss_MemoryBarrierReleaseFunction(void);
void Moss_MemoryBarrierFullFunction(void);
void Moss_CPUPause(void); // _mm_pause() on x86, yield on ARM

// --- Spinlocks ---
bool Moss_TryLockSpinlock(Moss_SpinLock *lock);
void Moss_LockSpinlock(Moss_SpinLock *lock);
void Moss_UnlockSpinlock(Moss_SpinLock *lock);

// --- 32-bit Signed Int ---
int  Moss_GetAtomicInt(Moss_AtomicInt *a);
int  Moss_SetAtomicInt(Moss_AtomicInt *a, int v);
int  Moss_AddAtomicInt(Moss_AtomicInt *a, int v);
bool Moss_CompareAndSwapAtomicInt(Moss_AtomicInt *a, int oldval, int newval);

// --- 32-bit Unsigned Int (Flags & Counters) ---
uint32_t Moss_GetAtomicU32(Moss_AtomicU32 *a);
uint32_t Moss_SetAtomicU32(Moss_AtomicU32 *a, uint32_t v);
uint32_t Moss_AddAtomicU32(Moss_AtomicU32 *a, int32_t v);
uint32_t Moss_AndAtomicU32(Moss_AtomicU32 *a, uint32_t v);
uint32_t Moss_OrAtomicU32(Moss_AtomicU32 *a, uint32_t v);
uint32_t Moss_XorAtomicU32(Moss_AtomicU32 *a, uint32_t v);
bool     Moss_CompareAndSwapAtomicU32(Moss_AtomicU32 *a, uint32_t oldval, uint32_t newval);

// --- 64-bit Unsigned Int (Timers, Ring Buffers, ABA Tags) ---
uint64_t Moss_GetAtomicU64(Moss_AtomicU64 *a);
uint64_t Moss_SetAtomicU64(Moss_AtomicU64 *a, uint64_t v);
uint64_t Moss_AddAtomicU64(Moss_AtomicU64 *a, int64_t v);
uint64_t Moss_AndAtomicU64(Moss_AtomicU64 *a, uint64_t v);
uint64_t Moss_OrAtomicU64(Moss_AtomicU64 *a, uint64_t v);
uint64_t Moss_XorAtomicU64(Moss_AtomicU64 *a, uint64_t v);
bool     Moss_CompareAndSwapAtomicU64(Moss_AtomicU64 *a, uint64_t oldval, uint64_t newval);

// --- Pointers ---
void *Moss_GetAtomicPointer(void * volatile *a);
void *Moss_SetAtomicPointer(void * volatile *a, void *v);
bool  Moss_CompareAndSwapAtomicPointer(void * volatile *a, void *oldval, void *newval);

// High-resolution monotonic clock
MOSS_API uint64_t Moss_GetTicksMS(void);
MOSS_API uint64_t Moss_GetTicksNS(void);
MOSS_API uint64_t Moss_GetPerformanceCounter(void);
MOSS_API uint64_t Moss_GetPerformanceFrequency(void);

// Delta-time helpers
MOSS_API float    Moss_GetElapsedSeconds(uint64_t startCounter);
MOSS_API float    Moss_GetElapsedMilliseconds(uint64_t startCounter);
MOSS_API float    Moss_GetMillisecondsAndReset(uint64_t* counter);

// Sleeping & Yielding
MOSS_API void     Moss_Yield(void);
MOSS_API void     Moss_DelayMS(uint32_t ms);
MOSS_API void     Moss_DelayNS(uint64_t ns);
MOSS_API void     Moss_DelayPreciseNS(uint64_t ns); // Sleep + spin-wait tail for frame pacing

// Callback Timers
MOSS_API Moss_TimerID Moss_AddTimerMS(uint32_t intervalMS, Moss_TimerCallback callback, void* userdata);
MOSS_API Moss_TimerID Moss_AddTimerNS(uint64_t intervalNS, Moss_NSTimerCallback callback, void* userdata);
MOSS_API bool         Moss_RemoveTimer(Moss_TimerID id);


// Wall-clock queries & conversions
MOSS_API bool Moss_GetCurrentTime(Moss_Time* ticks);
MOSS_API bool Moss_TimeToDateTime(Moss_Time ticks, Moss_DateTime* dt, bool localTime);
MOSS_API bool Moss_DateTimeToTime(const Moss_DateTime* dt, Moss_Time* ticks);

// Calendar helpers
MOSS_API int  Moss_GetDaysInMonth(int year, int month);
MOSS_API int  Moss_GetDayOfYear(int year, int month, int day);
MOSS_API int  Moss_GetDayOfWeek(int year, int month, int day);

// Thread-safe formatting (writes into caller's buffer)
MOSS_API bool Moss_FormatTime(char* outBuf, size_t bufSize, const char* fmt, const Moss_DateTime* dt);
MOSS_API bool Moss_FormatTimeNow(char* outBuf, size_t bufSize, const char* fmt, bool localTime);

// Explicit state API (Thread-safe & deterministic)
MOSS_API void     Moss_RngSeed(Moss_RNG* rng, uint64_t seed, uint64_t sequence);
MOSS_API uint32_t Moss_RngNextU32(Moss_RNG* rng);
MOSS_API uint32_t Moss_RngBoundU32(Moss_RNG* rng, uint32_t bound); // Unbiased [0, bound)
MOSS_API int32_t  Moss_RngRangeI32(Moss_RNG* rng, int32_t min, int32_t max); // Inclusive [min, max]
MOSS_API float    Moss_RngNextFloat(Moss_RNG* rng); // [0.0f, 1.0f)
MOSS_API float    Moss_RngRangeFloat(Moss_RNG* rng, float min, float max);
MOSS_API float    Moss_RngGaussian(Moss_RNG* rng, float mean, float stddev);

// Convenience global API (wraps a default global Moss_RNG)
MOSS_API void     Moss_srand(uint64_t seed);
MOSS_API int32_t  Moss_rand(int32_t n);
MOSS_API int32_t  Moss_randi(void);
MOSS_API float    Moss_randf(void);
MOSS_API int32_t  Moss_randi_range(int32_t from, int32_t to);
MOSS_API float    Moss_randf_range(float from, float to);
MOSS_API float    Moss_rand_gaussian(float mean, float stddev);

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
#endif // MOSS_COMPILER_MSVC || MOSS_COMPILER_GCC
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
#endif // MOSS_SIMD_NEON


#ifdef MOSS_USE_RVV

template <unsigned IndexX, unsigned IndexY, unsigned IndexZ, unsigned IndexW>
MOSS_INLINE vfloat32m1_t RVVShuffleFloat32x4(vfloat32m1_t inV0, vfloat32m1_t inV1)
{
	vfloat32m2_t combined = __riscv_vlmul_ext_v_f32m1_f32m2(inV0);
	combined = __riscv_vslideup_vx_f32m2(combined, __riscv_vlmul_ext_v_f32m1_f32m2(inV1), 4, 8);

	const uint32 indices_raw[4] = { IndexX, IndexY, IndexZ, IndexW };
	const vuint32m1_t v_indices_m1 = __riscv_vle32_v_u32m1(indices_raw, 4);
	const vuint32m2_t v_indices_m2 = __riscv_vlmul_ext_v_u32m1_u32m2(v_indices_m1);

	const vfloat32m2_t gathered_m2 = __riscv_vrgather_vv_f32m2(combined, v_indices_m2, 4);
	return __riscv_vlmul_trunc_v_f32m2_f32m1(gathered_m2);
}

template <>
MOSS_INLINE vfloat32m1_t RVVShuffleFloat32x4<0, 1, 2, 3>(vfloat32m1_t inV0, vfloat32m1_t inV1)
{
	return inV0;
}

template <>
MOSS_INLINE vfloat32m1_t RVVShuffleFloat32x4<0, 1, 4, 5>(vfloat32m1_t inV0, vfloat32m1_t inV1)
{
	vfloat32m1_t result = inV0;
	return __riscv_vslideup_vx_f32m1(result, inV1, 2, 4);
}

template <>
MOSS_INLINE vfloat32m1_t RVVShuffleFloat32x4<1, 0, 3, 2>(vfloat32m1_t inV0, vfloat32m1_t inV1)
{
	// Avoids m2 extension overhead that the default implementation of RVVShuffleFloat32x4 has
	const uint32 indices_raw[4] = { 1, 0, 3, 2 };
	const vuint32m1_t indices = __riscv_vle32_v_u32m1(indices_raw, 4);
	return __riscv_vrgather_vv_f32m1(inV0, indices, 4);
}

template <>
MOSS_INLINE vfloat32m1_t RVVShuffleFloat32x4<2, 3, 0, 1>(vfloat32m1_t inV0, vfloat32m1_t inV1)
{
	vfloat32m1_t upper = __riscv_vslidedown_vx_f32m1(inV0, 2, 4);
	return __riscv_vslideup_vx_f32m1(upper, inV0, 2, 4);
}

template <>
MOSS_INLINE vfloat32m1_t RVVShuffleFloat32x4<2, 3, 6, 7>(vfloat32m1_t inV0, vfloat32m1_t inV1)
{
	return __riscv_vslidedown_vx_f32m1_tu(inV1, inV0, 2, 2);
}

template <>
MOSS_INLINE vfloat32m1_t RVVShuffleFloat32x4<4, 5, 6, 7>(vfloat32m1_t inV0, vfloat32m1_t inV1)
{
	return inV1;
}

template <>
MOSS_INLINE vfloat32m1_t RVVShuffleFloat32x4<0, 2, 4, 6>(vfloat32m1_t inV0, vfloat32m1_t inV1)
{
	vfloat32m2_t combined = __riscv_vlmul_ext_v_f32m1_f32m2(inV0);
	combined = __riscv_vslideup_vx_f32m2(combined, __riscv_vlmul_ext_v_f32m1_f32m2(inV1), 4, 8);

	vuint64m2_t combined_u64 = __riscv_vreinterpret_v_u32m2_u64m2(__riscv_vreinterpret_v_f32m2_u32m2(combined));

	// vnsrl extracts lower 32 bits from all 4 u64 elements -> [0, 2, 4, 6]
	vuint32m1_t result = __riscv_vnsrl_wx_u32m1(combined_u64, 0, 4);

	return __riscv_vreinterpret_v_u32m1_f32m1(result);
}

template <>
MOSS_INLINE vfloat32m1_t RVVShuffleFloat32x4<1, 3, 5, 7>(vfloat32m1_t inV0, vfloat32m1_t inV1)
{
	vfloat32m2_t combined = __riscv_vlmul_ext_v_f32m1_f32m2(inV0);
	combined = __riscv_vslideup_vx_f32m2(combined, __riscv_vlmul_ext_v_f32m1_f32m2(inV1), 4, 8);

	vuint64m2_t combined_u64 = __riscv_vreinterpret_v_u32m2_u64m2(__riscv_vreinterpret_v_f32m2_u32m2(combined));

	// vnsrl with shift=32 extracts upper 32 bits from all 4 u64 elements -> [1, 3, 5, 7]
	vuint32m1_t result = __riscv_vnsrl_wx_u32m1(combined_u64, 32, 4);

	return __riscv_vreinterpret_v_u32m1_f32m1(result);
}

/// Given inV = (a, b, c, d), calculates (a + b) + (c + d) when cross platform determinism is on, otherwise calculates a + b + c + d (order undefined)
MOSS_INLINE vfloat32m1_t RVVSumElementsFloat32x4(vfloat32m1_t inV)
{
#ifdef MOSS_CROSS_PLATFORM_DETERMINISTIC
	const vfloat32m1_t shift1 = __riscv_vslidedown_vx_f32m1(inV, 1, 4);
	const vfloat32m1_t sum_pairs = __riscv_vfadd_vv_f32m1(inV, shift1, 4);
	const vfloat32m1_t shift2 = __riscv_vslidedown_vx_f32m1(sum_pairs, 2, 4);
	const vfloat32m1_t sum = __riscv_vfadd_vv_f32m1(sum_pairs, shift2, 4);
#else
	const vfloat32m1_t zeros = __riscv_vfmv_v_f_f32m1(0.0f, 4);
	const vfloat32m1_t sum = __riscv_vfredusum_vs_f32m1_f32m1(inV, zeros, 4);
#endif
	return sum;
}

#endif // MOSS_USE_RVV

struct Moss_BumpAlloc {
    uint8_t* buffer = nullptr;
    size_t offset = 0;
    size_t capacity = 0;
};

MOSS_API inline bool Moss_CreateBumpAlloc(Moss_BumpAlloc* alloc, size_t capacity) {
    if (!alloc || capacity == 0)
        return false;
	
	// Prevent accidental replacement of an existing allocation.
    if (alloc->buffer)
        return false;

    alloc->buffer = static_cast<uint8_t*>(Allocate(capacity));

    if (!alloc->buffer) {
        alloc->offset = 0;
        alloc->capacity = 0;
        return false;
    }

    alloc->offset = 0;
    alloc->capacity = capacity;

    return true;
}

MOSS_API inline char* Moss_AllocateBumpAlloc(Moss_BumpAlloc* alloc, size_t size, size_t alignment = 16) {
    if (!alloc || !alloc->buffer || size == 0)
        return nullptr;

    if (alignment == 0 || (alignment & (alignment - 1)) != 0)
        return nullptr;

    // Prevent overflow during alignment.
    if (alloc->offset > SIZE_MAX - (alignment - 1))
        return nullptr;

	const uintptr_t base = reinterpret_cast<uintptr_t>(alloc->buffer);
    if (alloc->offset > alloc->capacity)
        return nullptr;

    if (base > UINTPTR_MAX - alloc->offset)
        return nullptr;

    const uintptr_t current = base + alloc->offset;
    const uintptr_t mask = alignment - 1;

    if (current > UINTPTR_MAX - mask)
        return nullptr;

    const uintptr_t aligned = (current + mask) & ~mask;

    const size_t alignedOffset = static_cast<size_t>(aligned - base);

    if (alignedOffset > alloc->capacity || size > alloc->capacity - alignedOffset)
        return nullptr;

    char* result = reinterpret_cast<char*>(aligned);
    alloc->offset = alignedOffset + size;

	return result;
}

MOSS_INLINE void Moss_ResetBumpAlloc(Moss_BumpAlloc* alloc) { if (alloc) alloc->offset = 0; }

MOSS_INLINE void Moss_FreeBumpAlloc(Moss_BumpAlloc* alloc) {
    if (alloc && alloc->buffer) {
        Free(alloc->buffer);
        alloc->buffer   = nullptr;
        alloc->offset   = 0;
        alloc->capacity = 0;
    }
}



MOSS_INLINE int64_t Moss_GetFileTimeStamp(const char* filepath) {
    if (!filepath) return -1;
    struct stat fileStat = {};
    if (stat(filepath, &fileStat) != 0) { return -1; }
    return static_cast<int64_t>(fileStat.st_mtime);
}

MOSS_INLINE bool Moss_FileExists(const char* filepath) {
    if (!filepath) return false;
    FILE* file = fopen(filepath, "rb");
    if (!file) return false;
    fclose(file);
    return true;
}

MOSS_INLINE int64_t Moss_GetFileSize(const char* filepath) {
    if (!filepath) return -1;
    FILE* file = fopen(filepath, "rb");
    if (!file) return -1;

    if (fseek(file, 0, SEEK_END) != 0) { fclose(file); return -1; }

    long size = ftell(file);
    fclose(file);
    return (size < 0) ? -1 : static_cast<int64_t>(size);
}

// Reads file into a caller-supplied buffer of size >= fileSize + 1
MOSS_INLINE char* Moss_FileReadBinary(const char* filepath, size_t* outFileSize, char* buffer, size_t capacity) {
    if (outFileSize)
        *outFileSize = 0;

    if (!filepath || !buffer)
        return nullptr;

    FILE* file = fopen(filepath, "rb");

    if (!file)
        return nullptr;

    if (fseek(file, 0, SEEK_END) != 0) { fclose(file); return nullptr; }

    long rawSize = ftell(file);

    if (rawSize < 0) { fclose(file); return nullptr; }

    if (fseek(file, 0, SEEK_SET) != 0) { fclose(file); return nullptr; }

    const size_t size = static_cast<size_t>(rawSize);

    if (size > capacity) {
        fclose(file);
        return nullptr;
    }

    const size_t bytesRead = fread(buffer, 1, size, file);

    const bool readSucceeded = bytesRead == size;
    const bool closeSucceeded = fclose(file) == 0;

    if (!readSucceeded || !closeSucceeded)
        return nullptr;

    if (outFileSize)
        *outFileSize = bytesRead;

    return buffer;
}
MOSS_INLINE char* Moss_FileReadBinary(const char* filepath, size_t* outFileSize, Moss_BumpAlloc* alloc) {
    if (outFileSize)
        *outFileSize = 0;

    if (!filepath || !alloc || !alloc->buffer)
        return nullptr;

    const int64_t fileSize = Moss_GetFileSize(filepath);

    if (fileSize < 0 || static_cast<uint64_t>(fileSize) > SIZE_MAX)
        return nullptr;

    const size_t allocationSize = fileSize > 0 ? static_cast<size_t>(fileSize) : 1;

    const size_t previousOffset = alloc->offset;

    char* buffer = Moss_AllocateBumpAlloc(alloc, allocationSize, alignof(char));

    if (!buffer)
        return nullptr;

    char* result = Moss_FileReadBinary(filepath, outFileSize, buffer, allocationSize);

    if (!result)
        alloc->offset = previousOffset;

    return result;
}


MOSS_INLINE char* Moss_FileReadText(const char* filepath, size_t* outFileSize, char* buffer, size_t capacity) {
    if (outFileSize)
        *outFileSize = 0;

    if (!filepath || !buffer || capacity == 0)
        return nullptr;

    size_t bytesRead = 0;

    if (!Moss_FileReadBinary(filepath, &bytesRead, buffer, capacity - 1)) { return nullptr; }

    buffer[bytesRead] = '\0';

    if (outFileSize)
        *outFileSize = bytesRead;

    return buffer;
}

// Overload using Moss_BumpAlloc
MOSS_INLINE char* Moss_FileRead(const char* filepath, size_t* outFileSize, Moss_BumpAlloc* alloc) {
    if (outFileSize)
        *outFileSize = 0;

    if (!filepath || !alloc || !alloc->buffer)
        return nullptr;

	const int64_t fileSize = Moss_GetFileSize(filepath);

	if (fileSize < 0 || static_cast<uint64_t>(fileSize) >= SIZE_MAX)
        return nullptr;

    const size_t capacity = static_cast<size_t>(fileSize) + 1;
    const size_t previousOffset = alloc->offset;

    char* buffer = Moss_AllocateBumpAlloc(alloc, capacity, alignof(char));

    if (!buffer)
        return nullptr;

    char* result = Moss_FileReadText(filepath, outFileSize, buffer, capacity);

    if (!result)
        alloc->offset = previousOffset;

    return result;
}

MOSS_INLINE char* Moss_FileReadText(const char* filepath, size_t* outFileSize, Moss_BumpAlloc* alloc) {
    if (outFileSize) { *outFileSize = 0; }

    if (!filepath || !alloc) { return nullptr; }

    const int64_t fileSize = Moss_GetFileSize(filepath);

    if (fileSize < 0 || static_cast<uint64_t>(fileSize) >= SIZE_MAX) { return nullptr; }

    const size_t capacity = static_cast<size_t>(fileSize) + 1;
    const size_t previousOffset = alloc->offset;

    char* buffer = static_cast<char*>(Moss_AllocateBumpAlloc(alloc, capacity, 16));

    if (!buffer)
        return nullptr;

    char* result = Moss_FileReadText(filepath, outFileSize, buffer, capacity);

    if (!result) { alloc->offset = previousOffset; }

    return result;
}

MOSS_INLINE bool Moss_FileWrite(const char* filepath, const void* buffer, size_t size) {
    if (!filepath || (!buffer && size > 0)) return false;

    FILE* file = fopen(filepath, "wb");

    if (!file)
        return false;

    const size_t written = fwrite(buffer, 1, size, file);

    const bool writeSucceeded = written == size;
    const bool closeSucceeded = fclose(file) == 0;

    return writeSucceeded && closeSucceeded;
}

MOSS_INLINE bool Moss_CopyFile(const char* srcPath, const char* dstPath, Moss_BumpAlloc* alloc) {
	if (!srcPath || !dstPath || !alloc || !alloc->buffer)
        return false;

    const size_t previousOffset = alloc->offset;
    size_t fileSize = 0;
    char* data = Moss_FileReadBinary(srcPath, &fileSize, alloc);

    if (!data)
        return false;

    const bool success = Moss_FileWrite(dstPath, data, fileSize);

    // Release temporary allocation.
    alloc->offset = previousOffset;

    return success;
}


// Find the roots of \f$inA \: x^2 + inB \: x + inC = 0\f$.
// @return The number of roots, actual roots in outX1 and outX2.
// If number of roots returned is 1 then outX1 == outX2.
template <typename T>
inline int FindRoot(const T inA, const T inB, const T inC, T &outX1, T &outX2) {
	// Check if this is a linear equation
	if (inA == T(0)) {
		// Check if this is a constant equation
		if (inB == T(0)) { return 0; }
		// Linear equation with 1 solution
		outX1 = outX2 = -inC / inB;
		return 1;
	}

	// See Numerical Recipes in C, Chapter 5.6 Quadratic and Cubic Equations
	T det = Square(inB) - T(4) * inA * inC;
	if (det < T(0)) { return 0; }
	T q = (inB + Sign(inB) * sqrt(det)) / T(-2);
	outX1 = q / inA;
	if (q == T(0)) {
		outX2 = outX1;
		return 1;
	}
	outX2 = inC / q;
	return 2;
}

#ifdef MOSS_DISABLE_CUSTOM_ALLOCATOR
	#define MOSS_ALLOC_FN(x)	x
	#define MOSS_ALLOC_SCOPE
#else
	#define MOSS_ALLOC_FN(x)	x##Impl
	#define MOSS_ALLOC_SCOPE static
#endif

MOSS_ALLOC_SCOPE void *MOSS_ALLOC_FN(Allocate)(size_t inSize) {
	MOSS_ASSERT(inSize > 0);
	return malloc(inSize);
}

MOSS_ALLOC_SCOPE void *MOSS_ALLOC_FN(Reallocate)(void *inBlock, [[maybe_unused]] size_t inOldSize, size_t inNewSize) {
	MOSS_ASSERT(inNewSize > 0);
	return realloc(inBlock, inNewSize);
}

MOSS_ALLOC_SCOPE void MOSS_ALLOC_FN(Free)(void *inBlock) { free(inBlock); }

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

void RegisterDefaultAllocator() {
	Allocate = AllocateImpl;
	Reallocate = ReallocateImpl;
	Free = FreeImpl;
	AlignedAllocate = AlignedAllocateImpl;
	AlignedFree = AlignedFreeImpl;
}
#endif // MOSS_DISABLE_CUSTOM_ALLOCATOR


// ============================================================================
// Compiler Format Checking Annotations (MSVC SAL vs GCC/Clang Attributes)
// ============================================================================
#if defined(_MSC_VER)
    #define MOSS_PRINTF_FORMAT_STRING _Printf_format_string_
    #define MOSS_SCANF_FORMAT_STRING  _Scanf_format_string_impl_
    #define MOSS_PRINTF_VARARG_FUNC(fmtarg)
    #define MOSS_PRINTF_VARARG_FUNCV(fmtarg)
    #define MOSS_SCANF_VARARG_FUNC(fmtarg)
    #define MOSS_SCANF_VARARG_FUNCV(fmtarg)
#elif defined(__GNUC__) || defined(__clang__)
    #define MOSS_PRINTF_FORMAT_STRING
    #define MOSS_SCANF_FORMAT_STRING
    #define MOSS_PRINTF_VARARG_FUNC(fmtarg)  __attribute__((format(__printf__, fmtarg, fmtarg + 1)))
    #define MOSS_PRINTF_VARARG_FUNCV(fmtarg) __attribute__((format(__printf__, fmtarg, 0)))
    #define MOSS_SCANF_VARARG_FUNC(fmtarg)   __attribute__((format(__scanf__, fmtarg, fmtarg + 1)))
    #define MOSS_SCANF_VARARG_FUNCV(fmtarg)  __attribute__((format(__scanf__, fmtarg, 0)))
#else
    #define MOSS_PRINTF_FORMAT_STRING
    #define MOSS_SCANF_FORMAT_STRING
    #define MOSS_PRINTF_VARARG_FUNC(fmtarg)
    #define MOSS_PRINTF_VARARG_FUNCV(fmtarg)
    #define MOSS_SCANF_VARARG_FUNC(fmtarg)
    #define MOSS_SCANF_VARARG_FUNCV(fmtarg)
#endif

// ============================================================================
// C / C++ Safe Cast Macros
// ============================================================================
#ifdef __cplusplus
    #define MOSS_static_cast(type, expression)      static_cast<type>(expression)
    #define MOSS_reinterpret_cast(type, expression) reinterpret_cast<type>(expression)
    #define MOSS_const_cast(type, expression)       const_cast<type>(expression)
#else
    #define MOSS_static_cast(type, expression)      ((type)(expression))
    #define MOSS_reinterpret_cast(type, expression) ((type)(expression))
    #define MOSS_const_cast(type, expression)       ((type)(expression))
#endif

// ============================================================================
// Memory, Stack & String Helpers
// ============================================================================
#define MOSS_STRINGIFY_ARG(arg) #arg

#if defined(_MSC_VER)
    #define MOSS_stack_alloc(type, count) MOSS_static_cast(type*, _alloca(sizeof(type) * (count)))
#else
    #define MOSS_stack_alloc(type, count) MOSS_static_cast(type*, alloca(sizeof(type) * (count)))
#endif
#define MOSS_stack_free(data) ((void)(data))

#define MOSS_zero(x)  MOSS_memset(&(x), 0, sizeof((x)))
#define MOSS_zeroa(x) MOSS_memset((x), 0, sizeof((x)))
#define MOSS_zerop(x) MOSS_memset((x), 0, sizeof(*(x)))

// ============================================================================
// Core Callback Typedefs
// ============================================================================
typedef void* (MOSS_CALL *MOSS_malloc_func)(size_t size);
typedef void* (MOSS_CALL *MOSS_calloc_func)(size_t nmemb, size_t size);
typedef void* (MOSS_CALL *MOSS_realloc_func)(void* mem, size_t size);
typedef void  (MOSS_CALL *MOSS_free_func)(void* mem);

typedef void (*MOSS_FunctionPointer)(void);
typedef int  (MOSS_CALL *MOSS_CompareCallback)(const void* a, const void* b);
typedef int  (MOSS_CALL *MOSS_CompareCallback_r)(void* userdata, const void* a, const void* b);


#if defined(__AVX2__)
    #include <immintrin.h>
    typedef __m128  Moss_Simd128f;
    typedef __m128d Moss_Simd128d;
    typedef __m128i Moss_Simd128i;
    typedef __m256  Moss_Simd256f;
    typedef __m256d Moss_Simd256d;
    typedef __m256i Moss_Simd256i;
#elif defined(__SSE2__) || defined(_M_X64) || (defined(_M_IX86_FP) && _M_IX86_FP >= 2)
    #include <emmintrin.h>
    typedef __m128  Moss_Simd128f;
    typedef __m128d Moss_Simd128d;
    typedef __m128i Moss_Simd128i;
    typedef struct { __m128 lo, hi; }  Moss_Simd256f;
    typedef struct { __m128d lo, hi; } Moss_Simd256d;
    typedef struct { __m128i lo, hi; } Moss_Simd256i;
#elif defined(__ARM_NEON)
    #include <arm_neon.h>
    typedef float32x4_t Moss_Simd128f;
    typedef float64x2_t Moss_Simd128d;
    typedef int32x4_t   Moss_Simd128i;
    typedef struct { float32x4_t lo, hi; } Moss_Simd256f;
    typedef struct { float64x2_t lo, hi; } Moss_Simd256d;
    typedef struct { int32x4_t lo, hi; }   Moss_Simd256i;
#else
    typedef struct { float v[4]; }    Moss_Simd128f;
    typedef struct { double v[2]; }   Moss_Simd128d;
    typedef struct { int32_t v[4]; }  Moss_Simd128i;
    typedef struct { float v[8]; }    Moss_Simd256f;
    typedef struct { double v[4]; }   Moss_Simd256d;
    typedef struct { int32_t v[8]; }  Moss_Simd256i;
#endif

// ============================================================================
// Memory Load & Store Operations (Explicit Memory Boundaries)
// ============================================================================
MOSS_INLINE Moss_Simd128i Moss_Simd_i_Load(const void* ptr);          // 16-byte aligned
MOSS_INLINE Moss_Simd128i Moss_Simd_i_LoadU(const void* ptr);         // Unaligned
MOSS_INLINE void          Moss_Simd_i_Store(void* ptr, Moss_Simd128i v);
MOSS_INLINE void          Moss_Simd_i_StoreU(void* ptr, Moss_Simd128i v);

MOSS_INLINE Moss_Simd128f Moss_Simd_f_Load(const float* ptr);         // 16-byte aligned
MOSS_INLINE Moss_Simd128f Moss_Simd_f_LoadU(const float* ptr);        // Unaligned
MOSS_INLINE void          Moss_Simd_f_Store(float* ptr, Moss_Simd128f v);
MOSS_INLINE void          Moss_Simd_f_StoreU(float* ptr, Moss_Simd128f v);

MOSS_INLINE Moss_Simd128d Moss_Simd_d_Load(const double* ptr);        // 16-byte aligned
MOSS_INLINE Moss_Simd128d Moss_Simd_d_LoadU(const double* ptr);       // Unaligned
MOSS_INLINE void          Moss_Simd_d_Store(double* ptr, Moss_Simd128d v);
MOSS_INLINE void          Moss_Simd_d_StoreU(double* ptr, Moss_Simd128d v);


// ============================================================================
// Boolean & Bitwise Masks (_b / 128-bit raw register bits)
// ============================================================================
MOSS_INLINE Moss_Simd128i Moss_Simd_b_And(Moss_Simd128i a, Moss_Simd128i b);
MOSS_INLINE Moss_Simd128i Moss_Simd_b_Or(Moss_Simd128i a, Moss_Simd128i b);
MOSS_INLINE Moss_Simd128i Moss_Simd_b_Xor(Moss_Simd128i a, Moss_Simd128i b);
MOSS_INLINE Moss_Simd128i Moss_Simd_b_Not(Moss_Simd128i src);
MOSS_INLINE Moss_Simd128i Moss_Simd_b_AndNot(Moss_Simd128i a, Moss_Simd128i b); // (~a) & b


// ============================================================================
// 8-Bit Integers / Characters (_c / 16 elements per 128-bit register)
// ============================================================================
MOSS_INLINE Moss_Simd128i Moss_Simd_c_Add(Moss_Simd128i a, Moss_Simd128i b);     // Wrapping signed/unsigned
MOSS_INLINE Moss_Simd128i Moss_Simd_c_Sub(Moss_Simd128i a, Moss_Simd128i b);     // Wrapping signed/unsigned
MOSS_INLINE Moss_Simd128i Moss_Simd_c_AddSatU(Moss_Simd128i a, Moss_Simd128i b); // Unsigned saturating [0..255]
MOSS_INLINE Moss_Simd128i Moss_Simd_c_SubSatU(Moss_Simd128i a, Moss_Simd128i b); // Unsigned saturating [0..255]
MOSS_INLINE Moss_Simd128i Moss_Simd_c_MinU(Moss_Simd128i a, Moss_Simd128i b);    // Unsigned min
MOSS_INLINE Moss_Simd128i Moss_Simd_c_MaxU(Moss_Simd128i a, Moss_Simd128i b);    // Unsigned max


// ============================================================================
// 16-Bit Integers / Shorts (_s / 8 elements per 128-bit register)
// ============================================================================
MOSS_INLINE Moss_Simd128i Moss_Simd_s_Add(Moss_Simd128i a, Moss_Simd128i b);
MOSS_INLINE Moss_Simd128i Moss_Simd_s_Sub(Moss_Simd128i a, Moss_Simd128i b);
MOSS_INLINE Moss_Simd128i Moss_Simd_s_MulLow(Moss_Simd128i a, Moss_Simd128i b);  // Low 16 bits of product
MOSS_INLINE Moss_Simd128i Moss_Simd_s_ShiftLeft(Moss_Simd128i src, int count);
MOSS_INLINE Moss_Simd128i Moss_Simd_s_ShiftRightA(Moss_Simd128i src, int count); // Arithmetic (sign-extending)
MOSS_INLINE Moss_Simd128i Moss_Simd_s_ShiftRightL(Moss_Simd128i src, int count); // Logical (zero-filling)


// ============================================================================
// 32-Bit Integers (_i / 4 elements per 128-bit register)
// ============================================================================
MOSS_INLINE Moss_Simd128i Moss_Simd_i_Set1(int32_t v);
MOSS_INLINE Moss_Simd128i Moss_Simd_i_Add(Moss_Simd128i a, Moss_Simd128i b);
MOSS_INLINE Moss_Simd128i Moss_Simd_i_Sub(Moss_Simd128i a, Moss_Simd128i b);
MOSS_INLINE Moss_Simd128i Moss_Simd_i_Mul(Moss_Simd128i a, Moss_Simd128i b);
MOSS_INLINE Moss_Simd128i Moss_Simd_i_Cmpeq(Moss_Simd128i a, Moss_Simd128i b);   // 0xFFFFFFFF or 0x00000000 per lane


// ============================================================================
// 32-Bit Floating Point (_f / 4 elements per 128-bit register)
// ============================================================================
MOSS_INLINE Moss_Simd128f Moss_Simd_f_Set1(float v);
MOSS_INLINE Moss_Simd128f Moss_Simd_f_Add(Moss_Simd128f a, Moss_Simd128f b);
MOSS_INLINE Moss_Simd128f Moss_Simd_f_Sub(Moss_Simd128f a, Moss_Simd128f b);
MOSS_INLINE Moss_Simd128f Moss_Simd_f_Mul(Moss_Simd128f a, Moss_Simd128f b);
MOSS_INLINE Moss_Simd128f Moss_Simd_f_Div(Moss_Simd128f a, Moss_Simd128f b);
MOSS_INLINE Moss_Simd128f Moss_Simd_f_Sqrt(Moss_Simd128f src);
MOSS_INLINE Moss_Simd128f Moss_Simd_f_FMA(Moss_Simd128f a, Moss_Simd128f b, Moss_Simd128f c); // (a * b) + c
MOSS_INLINE float         Moss_Simd_f_Dot3(Moss_Simd128f a, Moss_Simd128f b);
MOSS_INLINE Moss_Simd128f Moss_Simd_f_Cmplt(Moss_Simd128f a, Moss_Simd128f b);   // Returns lane mask
MOSS_INLINE int           Moss_Simd_f_Movemask(Moss_Simd128f mask);              // Extracts 4-bit scalar mask
MOSS_INLINE Moss_Simd128f Moss_Simd_f_Blend(Moss_Simd128f a, Moss_Simd128f b, Moss_Simd128f mask);


// ============================================================================
// 64-Bit Floating Point / Doubles (_d / 2 elements per 128-bit register)
// ============================================================================
MOSS_INLINE Moss_Simd128d Moss_Simd_d_Set1(double v);
MOSS_INLINE Moss_Simd128d Moss_Simd_d_Add(Moss_Simd128d a, Moss_Simd128d b);
MOSS_INLINE Moss_Simd128d Moss_Simd_d_Sub(Moss_Simd128d a, Moss_Simd128d b);
MOSS_INLINE Moss_Simd128d Moss_Simd_d_Mul(Moss_Simd128d a, Moss_Simd128d b);
MOSS_INLINE Moss_Simd128d Moss_Simd_d_Div(Moss_Simd128d a, Moss_Simd128d b);


// ============================================================================
// High-End Register Scales (256-Bit / AVX2 Variations)
// ============================================================================
MOSS_INLINE Moss_Simd256i Moss_Simd256_i_Add(Moss_Simd256i a, Moss_Simd256i b);
MOSS_INLINE Moss_Simd256f Moss_Simd256_f_Add(Moss_Simd256f a, Moss_Simd256f b);
MOSS_INLINE Moss_Simd256d Moss_Simd256_d_Add(Moss_Simd256d a, Moss_Simd256d b);

MOSS_SUPPRESS_WARNINGS_END

#endif // MOSS_STDINC_H