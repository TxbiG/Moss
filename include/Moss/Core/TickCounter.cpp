// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <Moss/Core/TickCounter.h>

#if defined(MOSS_PLATFORM_WINDOWS)
	MOSS_SUPPRESS_WARNING_PUSH
	MOSS_MSVC_SUPPRESS_WARNING(5039) // winbase.h(13179): warning C5039: 'TpSetCallbackCleanupGroup': pointer or reference to potentially throwing function passed to 'extern "C"' function under -EHc. Undefined behavior may occur if this function throws an exception.
	#ifndef WIN32_LEAN_AND_MEAN
		#define WIN32_LEAN_AND_MEAN
	#endif
#ifndef MOSS_COMPILER_MINGW
	#include <Windows.h>
#else
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
	#include <Windows.h>
#endif
	MOSS_SUPPRESS_WARNING_POP
#endif

MOSS_SUPRESS_WARNINGS_BEGIN

#if defined(MOSS_PLATFORM_WINDOWS_UWP) || (defined(MOSS_PLATFORM_WINDOWS) && defined(MOSS_CPU_ARM))

uint64 GetProcessorTickCount()
{
	LARGE_INTEGER count;
	QueryPerformanceCounter(&count);
	return uint64(count.QuadPart);
}

#endif // MOSS_PLATFORM_WINDOWS_UWP || (MOSS_PLATFORM_WINDOWS && MOSS_CPU_ARM)

MOSS_SUPRESS_WARNINGS_END
