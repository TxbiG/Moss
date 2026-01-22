// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Moss/Core/FPControlWord.h>

MOSS_SUPRESS_WARNINGS_BEGIN

#if defined(MOSS_CPU_WASM) || defined(MOSS_CPU_RISCV) || defined(MOSS_CPU_PPC) || defined(MOSS_CPU_LOONGARCH)

// Not supported
class FPFlushDenormals { };

#elif defined(MOSS_SIMD_SSE)

/// Helper class that needs to be put on the stack to enable flushing denormals to zero
/// This can make floating point operations much faster when working with very small numbers
class FPFlushDenormals : public FPControlWord<_MM_FLUSH_ZERO_ON, _MM_FLUSH_ZERO_MASK> { };

#elif defined(MOSS_CPU_ARM) && defined(MOSS_COMPILER_MSVC)

/// Helper class that needs to be put on the stack to enable flushing denormals to zero
/// This can make floating point operations much faster when working with very small numbers
class FPFlushDenormals : public FPControlWord<_DN_FLUSH, _MCW_DN> { };

#elif defined(MOSS_CPU_ARM)

/// Flush denormals to zero bit
static constexpr uint64 FP_FZ = 1 << 24;

/// Helper class that needs to be put on the stack to enable flushing denormals to zero
/// This can make floating point operations much faster when working with very small numbers
class FPFlushDenormals : public FPControlWord<FP_FZ, FP_FZ> { };

#else

#error Unsupported CPU architecture

#endif

MOSS_SUPRESS_WARNINGS_END
