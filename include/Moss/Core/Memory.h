// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

MOSS_NAMESPACE_BEGIN

#ifndef MOSS_DISABLE_CUSTOM_ALLOCATOR

/// Normal memory allocation, must be at least 8 byte aligned on 32 bit platform and 16 byte aligned on 64 bit platform.
/// Note that you can override MOSS_DEFAULT_ALLOCATE_ALIGNMENT if your allocator's alignment is different from the alignment as defined by `__STDCPP_DEFAULT_NEW_ALIGNMENT__`.
using AllocateFunction = void *(*)(size_t inSize);

/// Reallocate memory. inBlock can be nullptr in which case it must behave as a memory allocation.
using ReallocateFunction = void *(*)(void *inBlock, size_t inOldSize, size_t inNewSize);

/// Free memory. inBlock can be nullptr in which case it must do nothing.
using FreeFunction = void (*)(void *inBlock);

/// Aligned memory allocation.
using AlignedAllocateFunction = void *(*)(size_t inSize, size_t inAlignment);

/// Free aligned memory. inBlock can be nullptr in which case it must do nothing.
using AlignedFreeFunction = void (*)(void *inBlock);

// User defined allocation / free functions
MOSS_API extern AllocateFunction Allocate;
MOSS_API extern ReallocateFunction Reallocate;
MOSS_API extern FreeFunction Free;
MOSS_API extern AlignedAllocateFunction AlignedAllocate;
MOSS_API extern AlignedFreeFunction AlignedFree;

/// Register platform default allocation / free functions
MOSS_API void RegisterDefaultAllocator();

// 32-bit MinGW g++ doesn't call the correct overload for the new operator when a type is 16 bytes aligned.
// It uses the non-aligned version, which on 32 bit platforms usually returns an 8 byte aligned block.
// We therefore default to 16 byte aligned allocations when the regular new operator is used.
// See: https://github.com/godotengine/godot/issues/105455#issuecomment-2824311547
// Note: See similar fix for the definition of MOSS_DEFAULT_ALLOCATE_ALIGNMENT
#if defined(MOSS_COMPILER_MINGW) && MOSS_CPU_ARCH_BITS == 32
	#define MOSS_INTERNAL_DEFAULT_ALLOCATE(size) AlignedAllocate(size, 16)
	#define MOSS_INTERNAL_DEFAULT_FREE(pointer) AlignedFree(pointer)
#else
	#define MOSS_INTERNAL_DEFAULT_ALLOCATE(size) Allocate(size)
	#define MOSS_INTERNAL_DEFAULT_FREE(pointer) Free(pointer)
#endif

/// Macro to override the new and delete functions
#define MOSS_OVERRIDE_NEW_DELETE \
	MOSS_INLINE void *operator new (size_t inCount)												{ return MOSS_INTERNAL_DEFAULT_ALLOCATE(inCount); } \
	MOSS_INLINE void operator delete (void *inPointer) noexcept									{ MOSS_INTERNAL_DEFAULT_FREE(inPointer); } \
	MOSS_INLINE void operator delete (void *inPointer, [[maybe_unused]] size_t inSize) noexcept	{ MOSS_INTERNAL_DEFAULT_FREE(inPointer); } \
	MOSS_INLINE void *operator new[] (size_t inCount)											{ return MOSS_INTERNAL_DEFAULT_ALLOCATE(inCount); } \
	MOSS_INLINE void operator delete[] (void *inPointer) noexcept								{ MOSS_INTERNAL_DEFAULT_FREE(inPointer); } \
	MOSS_INLINE void operator delete[] (void *inPointer, [[maybe_unused]] size_t inSize) noexcept{ MOSS_INTERNAL_DEFAULT_FREE(inPointer); } \
	MOSS_INLINE void *operator new (size_t inCount, std::align_val_t inAlignment)				{ return AlignedAllocate(inCount, static_cast<size_t>(inAlignment)); } \
	MOSS_INLINE void operator delete (void *inPointer, [[maybe_unused]] std::align_val_t inAlignment) noexcept { AlignedFree(inPointer); } \
	MOSS_INLINE void operator delete (void *inPointer, [[maybe_unused]] size_t inSize, [[maybe_unused]] std::align_val_t inAlignment) noexcept { AlignedFree(inPointer); } \
	MOSS_INLINE void *operator new[] (size_t inCount, std::align_val_t inAlignment)				{ return AlignedAllocate(inCount, static_cast<size_t>(inAlignment)); } \
	MOSS_INLINE void operator delete[] (void *inPointer, [[maybe_unused]] std::align_val_t inAlignment) noexcept	{ AlignedFree(inPointer); } \
	MOSS_INLINE void operator delete[] (void *inPointer, [[maybe_unused]] size_t inSize, [[maybe_unused]] std::align_val_t inAlignment) noexcept { AlignedFree(inPointer); } \
	MOSS_INLINE void *operator new ([[maybe_unused]] size_t inCount, void *inPointer) noexcept	{ return inPointer; } \
	MOSS_INLINE void operator delete ([[maybe_unused]] void *inPointer, [[maybe_unused]] void *inPlace) noexcept { /* Do nothing */ } \
	MOSS_INLINE void *operator new[] ([[maybe_unused]] size_t inCount, void *inPointer) noexcept	{ return inPointer; } \
	MOSS_INLINE void operator delete[] ([[maybe_unused]] void *inPointer, [[maybe_unused]] void *inPlace) noexcept { /* Do nothing */ }

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

MOSS_NAMESPACE_END
