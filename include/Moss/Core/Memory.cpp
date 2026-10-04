// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <Moss/Moss_stdinc.h>

MOSS_SUPPRESS_WARNINGS_BEGIN
#include <cstdlib>
MOSS_SUPPRESS_WARNINGS_END
#include <stdlib.h>

MOSS_NAMESPACE_BEGIN

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

MOSS_NAMESPACE_END
