// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2022 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Moss/Core/Variants/Vector/DVec3.h>
#include <Moss/Core/Variants/Matrix/DMat44.h>

MOSS_SUPRESS_WARNINGS_BEGIN

#ifdef MOSS_DOUBLE_PRECISION

// Define real to double
using Real = double;
using Real3 = Double3;
using RVec3 = DVec3;
using RVec3Arg = DVec3Arg;
using RMat44 = DMat44;
using RMat44Arg = DMat44Arg;

#define MOSS_RVECTOR_ALIGNMENT MOSS_DVECTOR_ALIGNMENT

#else

// Define real to float
using Real = float;
using Real3 = Float3;
using RVec3 = Vec3;
using RVec3Arg = Vec3Arg;
using RMat44 = Mat44;
using RMat44Arg = Mat44Arg;

#define MOSS_RVECTOR_ALIGNMENT MOSS_VECTOR_ALIGNMENT

#endif // MOSS_DOUBLE_PRECISION

// Put the 'real' operator in a namespace so that users can opt in to use it:
// using namespace MOSS::literals;
namespace literals {
	constexpr Real operator ""_r (long double inValue) { return Real(inValue); }
};

MOSS_SUPRESS_WARNINGS_END
