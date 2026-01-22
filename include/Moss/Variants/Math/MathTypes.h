// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

MOSS_SUPRESS_WARNINGS_BEGIN

class Vec2;
class Vec3;
class Vec4;
class BVec16;


class Mat3;
class Mat44;
class Mat4x4;


class DVec3;
class UVec4;

class DMat3;
class DMat44;

class Quat;
/*
class ivec2;
class ivec3;
class ivec4;
class uvec2;
class uvec3;
class uvec4;

class Mat4x3
class Mat4x2
class Mat3x2
class Mat3x4
class Mat2
class Mat2x3
class Mat2x4
class Mat2x4

class DMat4x3
class DMat4x2
class DMat3x2
class DMat3x4
class DMat2
class DMat2x3
class DMat2x4
*/


// Types to use for passing arguments to functions
using Vec3Arg = const Vec3;
#ifdef MOSS_SIMD_AVX
	using DVec3Arg = const DVec3;
#else
	using DVec3Arg = const DVec3&;
#endif
using Vec4Arg = const Vec4;
using UVec4Arg = const UVec4;
using QuatArg = const Quat;
using Mat44Arg = const Mat44&;
using DMat44Arg = const DMat44&;

MOSS_SUPRESS_WARNINGS_END
