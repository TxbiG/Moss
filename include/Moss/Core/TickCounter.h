// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

// Include for __rdtsc
#if defined(MOSS_PLATFORM_WINDOWS)
	#include <intrin.h>
#elif defined(MOSS_CPU_X86) && defined(MOSS_COMPILER_GCC)
	#include <x86intrin.h>
#elif defined(MOSS_CPU_E2K)
	#include <x86intrin.h>
#endif

MOSS_SUPRESS_WARNINGS_BEGIN

#if defined(MOSS_PLATFORM_WINDOWS_UWP) || (defined(MOSS_PLATFORM_WINDOWS) && defined(MOSS_CPU_ARM))

/// Functionality to get the processors cycle counter
uint64 GetProcessorTickCount(); // Not inline to avoid having to include Windows.h

#else

/// Functionality to get the processors cycle counter
MOSS_INLINE uint64 GetProcessorTickCount()
{
#if defined(MOSS_PLATFORM_BLUE)
	return MOSS_PLATFORM_BLUE_GET_TICKS();
#elif defined(MOSS_CPU_X86)
	return __rdtsc();
#elif defined(MOSS_CPU_E2K)
	return __rdtsc();
#elif defined(MOSS_CPU_ARM) && defined(MOSS_SIMD_NEON)
	uint64 val;
	asm volatile("mrs %0, cntvct_el0" : "=r" (val));
	return val;
#elif defined(MOSS_CPU_ARM) || defined(MOSS_CPU_RISCV) || defined(MOSS_CPU_WASM) || defined(MOSS_CPU_PPC) || defined(MOSS_CPU_LOONGARCH)
	return 0; // Not supported
#else
	#error Undefined
#endif
}

#endif // MOSS_PLATFORM_WINDOWS_UWP || (MOSS_PLATFORM_WINDOWS && MOSS_CPU_ARM)

MOSS_SUPRESS_WARNINGS_END
