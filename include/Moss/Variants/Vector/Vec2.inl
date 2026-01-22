// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <Moss/Core/Variants/Vector/Vec4.h>
#include <Moss/Core/Variants/Vector/UVec4.h>
#include <Moss/Core/HashCombine.h>

MOSS_SUPPRESS_WARNINGS_STD_BEGIN
#include <random>
MOSS_SUPPRESS_WARNINGS_STD_END

// Create a std::hash for Vec2
MOSS_MAKE_HASHABLE(Vec2, t.GetX(), t.GetY())

MOSS_SUPRESS_WARNINGS_BEGIN

void Vec2::CheckW() const
{
#ifdef MOSS_FLOATING_POINT_EXCEPTIONS_ENABLED
	// Avoid asserts when both components are NaN
	MOSS_ASSERT(reinterpret_cast<const uint32 *>(mF32)[2] == reinterpret_cast<const uint32 *>(mF32)[3]);
#endif // MOSS_FLOATING_POINT_EXCEPTIONS_ENABLED
}

MOSS_INLINE Vec2::Type Vec2::sFixW(Type inValue)
{
#ifdef MOSS_FLOATING_POINT_EXCEPTIONS_ENABLED
    #if defined(MOSS_SIMD_SSE)
        // Keep x, y, z; set w = z (which is zero here)
        return _mm_shuffle_ps(inValue, inValue, _MM_SHUFFLE(2, 2, 1, 0));
    #elif defined(MOSS_SIMD_NEON)
        return MOSS_NEON_SHUFFLE_F32x4(inValue, inValue, 0, 1, 2, 2);
    #else
        Type value;
        value.mData[0] = inValue.mData[0];
        value.mData[1] = inValue.mData[1];
        value.mData[2] = inValue.mData[2];
        value.mData[3] = inValue.mData[2];
        return value;
    #endif
#else
    return inValue;
#endif // MOSS_FLOATING_POINT_EXCEPTIONS_ENABLED
}

//Vec2::Vec2(const Vec2& inRHS) : mValue(sFixW(inRHS.mValue)) {}

Vec2::Vec2(const Float2 &inV)
{
#if defined(MOSS_SIMD_SSE)
    // Load x and y components only, set z and w to zero or some default
    Type x = _mm_load_ss(&inV.x);
    Type y = _mm_load_ss(&inV.y);
    // Unpack low floats of x and y into a single __m128
    Type xy = _mm_unpacklo_ps(x, y);
    // Set z and w to zero
    mValue = _mm_move_ss(_mm_setzero_ps(), xy); // put xy in lower part, zeros in upper
#elif defined(MOSS_SIMD_NEON)
    float32x2_t xy = vld1_f32(&inV.x);
    float32x2_t zeros = vdup_n_f32(0.0f); // zero for z,w
    mValue = vcombine_f32(xy, zeros);
#else
    mF32[0] = inV.x;
    mF32[1] = inV.y;
    mF32[2] = 0.0f;
    #ifdef MOSS_FLOATING_POINT_EXCEPTIONS_ENABLED
        mF32[3] = 0.0f;
    #endif
#endif
}

Vec2::Vec2(float inX, float inY) {
#if defined(MOSS_SIMD_SSE)
	// Set Y and X, leave upper values as 0.0f or unused
	mValue = _mm_set_ps(0.0f, 0.0f, inY, inX);
#elif defined(MOSS_SIMD_NEON)
	// Store only X and Y, upper lanes filled with 0.0f
	uint32x2_t xy = vcreate_u32(static_cast<uint64>(BitCast<uint32>(inX)) | (static_cast<uint64>(BitCast<uint32>(inY)) << 32));
	uint32x2_t zero = vdup_n_u32(0);
	mValue = vreinterpretq_f32_u32(vcombine_u32(xy, zero));
#else
	mF32[0] = inX;
	mF32[1] = inY;
	mF32[2] = 0.0f;
	#ifdef MOSS_FLOATING_POINT_EXCEPTIONS_ENABLED
		mF32[3] = 0.0f;
	#endif
#endif
}

template<uint32 SwizzleX, uint32 SwizzleY>
Vec2 Vec2::Swizzle() const
{
    static_assert(SwizzleX <= 3, "SwizzleX template parameter out of range");
    static_assert(SwizzleY <= 3, "SwizzleY template parameter out of range");

#if defined(MOSS_SIMD_SSE)
    constexpr int shuffleMask = _MM_SHUFFLE(2, 2, SwizzleY, SwizzleX);
    return Vec2(_mm_shuffle_ps(mValue, mValue, shuffleMask));
#elif defined(MOSS_SIMD_NEON)
    return Vec2(MOSS_NEON_SHUFFLE_F32x4(mValue, mValue, SwizzleX, SwizzleY, 2, 2));
#else
    return Vec2(mF32[SwizzleX], mF32[SwizzleY]);
#endif
}

Vec2 Vec2::sZero()
{
#if defined(MOSS_SIMD_SSE)
	return _mm_setzero_ps();
#elif defined(MOSS_SIMD_NEON)
	return vdupq_n_f32(0);
#else
	return Vec2(0, 0);
#endif
}

Vec2 Vec2::sReplicate(float inV)
{
#if defined(MOSS_SIMD_SSE)
	return _mm_set1_ps(inV);
#elif defined(MOSS_SIMD_NEON)
	return vdupq_n_f32(inV);
#else
	return Vec2(inV, inV);
#endif
}

Vec2 Vec2::sNaN()
{
	return sReplicate(numeric_limits<float>::quiet_NaN());
}

Vec2 Vec2::sLoadFloat2Unsafe(const Float2 &inV)
{
#if defined(MOSS_SIMD_SSE)
	Type v = _mm_loadu_ps(&inV.x);
#elif defined(MOSS_SIMD_NEON)
	Type v = vld1q_f32(&inV.x);
#else
	Type v = { inV.x, inV.y };
#endif
	return sFixW(v);
}

Vec2 Vec2::sMin(Vec2 inV1, Vec2 inV2)
{
#if defined(MOSS_SIMD_SSE)
	return _mm_min_ps(inV1.mValue, inV2.mValue);
#elif defined(MOSS_SIMD_NEON)
	return vminq_f32(inV1.mValue, inV2.mValue);
#else
	return Vec2(min(inV1.mF32[0], inV2.mF32[0]),
				min(inV1.mF32[1], inV2.mF32[1]));
#endif
}

Vec2 Vec2::sMax(const Vec2 inV1, const Vec2 inV2)
{
#if defined(MOSS_SIMD_SSE)
	return _mm_max_ps(inV1.mValue, inV2.mValue);
#elif defined(MOSS_SIMD_NEON)
	return vmaxq_f32(inV1.mValue, inV2.mValue);
#else
	return Vec2(max(inV1.mF32[0], inV2.mF32[0]),
				max(inV1.mF32[1], inV2.mF32[1]));
#endif
}

Vec2 Vec2::sClamp(const Vec2 inV, const Vec2 inMin, const Vec2 inMax)
{
	return sMax(sMin(inV, inMax), inMin);
}

UVec4 Vec2::sEquals(const Vec2 inV1, const Vec2 inV2)
{
#if defined(MOSS_SIMD_SSE)
	return _mm_castps_si128(_mm_cmpeq_ps(inV1.mValue, inV2.mValue));
#elif defined(MOSS_SIMD_NEON)
	return vceqq_f32(inV1.mValue, inV2.mValue);
#else
	uint32 z = inV1.mF32[2] == inV2.mF32[2]? 0xffffffffu : 0;
	return UVec4(inV1.mF32[0] == inV2.mF32[0]? 0xffffffffu : 0,
				 inV1.mF32[1] == inV2.mF32[1]? 0xffffffffu : 0,
				 z,
				 z);
#endif
}

UVec4 Vec2::sLess(const Vec2 inV1, const Vec2 inV2)
{
#if defined(MOSS_SIMD_SSE)
	return _mm_castps_si128(_mm_cmplt_ps(inV1.mValue, inV2.mValue));
#elif defined(MOSS_SIMD_NEON)
	return vcltq_f32(inV1.mValue, inV2.mValue);
#else
	uint32 z = inV1.mF32[2] < inV2.mF32[2]? 0xffffffffu : 0;
	return UVec4(inV1.mF32[0] < inV2.mF32[0]? 0xffffffffu : 0,
				 inV1.mF32[1] < inV2.mF32[1]? 0xffffffffu : 0,
				 z,
				 z);
#endif
}

UVec4 Vec2::sLessOrEqual(const Vec2 inV1, const Vec2 inV2)
{
#if defined(MOSS_SIMD_SSE)
	return _mm_castps_si128(_mm_cmple_ps(inV1.mValue, inV2.mValue));
#elif defined(MOSS_SIMD_NEON)
	return vcleq_f32(inV1.mValue, inV2.mValue);
#else
	uint32 z = inV1.mF32[2] <= inV2.mF32[2]? 0xffffffffu : 0;
	return UVec4(inV1.mF32[0] <= inV2.mF32[0]? 0xffffffffu : 0,
				 inV1.mF32[1] <= inV2.mF32[1]? 0xffffffffu : 0,
				 z,
				 z);
#endif
}

UVec4 Vec2::sGreater(const Vec2 inV1, const Vec2 inV2)
{
#if defined(MOSS_SIMD_SSE)
	return _mm_castps_si128(_mm_cmpgt_ps(inV1.mValue, inV2.mValue));
#elif defined(MOSS_SIMD_NEON)
	return vcgtq_f32(inV1.mValue, inV2.mValue);
#else
	uint32 z = inV1.mF32[2] > inV2.mF32[2]? 0xffffffffu : 0;
	return UVec4(inV1.mF32[0] > inV2.mF32[0]? 0xffffffffu : 0,
				 inV1.mF32[1] > inV2.mF32[1]? 0xffffffffu : 0,
				 z,
				 z);
#endif
}

UVec4 Vec2::sGreaterOrEqual(const Vec2 inV1, const Vec2 inV2)
{
#if defined(MOSS_SIMD_SSE)
	return _mm_castps_si128(_mm_cmpge_ps(inV1.mValue, inV2.mValue));
#elif defined(MOSS_SIMD_NEON)
	return vcgeq_f32(inV1.mValue, inV2.mValue);
#else
	uint32 z = inV1.mF32[2] >= inV2.mF32[2]? 0xffffffffu : 0;
	return UVec4(inV1.mF32[0] >= inV2.mF32[0]? 0xffffffffu : 0,
				 inV1.mF32[1] >= inV2.mF32[1]? 0xffffffffu : 0,
				 z,
				 z);
#endif
}

Vec2 Vec2::sFusedMultiplyAdd(const Vec2 inMul1, const Vec2 inMul2, const Vec2 inAdd)
{
#if defined(MOSS_SIMD_SSE)
	#ifdef MOSS_USE_FMADD
		return _mm_fmadd_ps(inMul1.mValue, inMul2.mValue, inAdd.mValue);
	#else
		return _mm_add_ps(_mm_mul_ps(inMul1.mValue, inMul2.mValue), inAdd.mValue);
	#endif
#elif defined(MOSS_SIMD_NEON)
	return vmlaq_f32(inAdd.mValue, inMul1.mValue, inMul2.mValue);
#else
	return Vec2(inMul1.mF32[0] * inMul2.mF32[0] + inAdd.mF32[0],
				inMul1.mF32[1] * inMul2.mF32[1] + inAdd.mF32[1],
				inMul1.mF32[2] * inMul2.mF32[2] + inAdd.mF32[2]);
#endif
}

Vec2 Vec2::sSelect(const Vec2 inNotSet, const Vec2 inSet, const UVec4 inControl)
{
#if defined(MOSS_SIMD_SSE4_11) && !defined(MOSS_PLATFORM_WASM) // _mm_blendv_ps has problems on FireFox
	Type v = _mm_blendv_ps(inNotSet.mValue, inSet.mValue, _mm_castsi128_ps(inControl.mValue));
	return sFixW(v);
#elif defined(MOSS_SIMD_SSE)
	__m128 is_set = _mm_castsi128_ps(_mm_srai_epi32(inControl.mValue, 31));
	Type v = _mm_or_ps(_mm_and_ps(is_set, inSet.mValue), _mm_andnot_ps(is_set, inNotSet.mValue));
	return sFixW(v);
#elif defined(MOSS_SIMD_NEON)
	Type v = vbslq_f32(vreinterpretq_u32_s32(vshrq_n_s32(vreinterpretq_s32_u32(inControl.mValue), 31)), inSet.mValue, inNotSet.mValue);
	return sFixW(v);
#else
	Vec2 result;
	for (int i = 0; i < 3; i++)
		result.mF32[i] = (inControl.mU32[i] & 0x80000000u) ? inSet.mF32[i] : inNotSet.mF32[i];
#ifdef MOSS_FLOATING_POINT_EXCEPTIONS_ENABLED
	result.mF32[3] = result.mF32[2];
#endif // MOSS_FLOATING_POINT_EXCEPTIONS_ENABLED
	return result;
#endif
}

Vec2 Vec2::sOr(const Vec2 inV1, const Vec2 inV2)
{
#if defined(MOSS_SIMD_SSE)
	return _mm_or_ps(inV1.mValue, inV2.mValue);
#elif defined(MOSS_SIMD_NEON)
	return vreinterpretq_f32_u32(vorrq_u32(vreinterpretq_u32_f32(inV1.mValue), vreinterpretq_u32_f32(inV2.mValue)));
#else
	return Vec2(UVec4::sOr(inV1.ReinterpretAsInt(), inV2.ReinterpretAsInt()).ReinterpretAsFloat());
#endif
}

Vec2 Vec2::sXor(const Vec2 inV1, const Vec2 inV2)
{
#if defined(MOSS_SIMD_SSE)
	return _mm_xor_ps(inV1.mValue, inV2.mValue);
#elif defined(MOSS_SIMD_NEON)
	return vreinterpretq_f32_u32(veorq_u32(vreinterpretq_u32_f32(inV1.mValue), vreinterpretq_u32_f32(inV2.mValue)));
#else
	return Vec2(UVec4::sXor(inV1.ReinterpretAsInt(), inV2.ReinterpretAsInt()).ReinterpretAsFloat());
#endif
}

Vec2 Vec2::sAnd(const Vec2 inV1, const Vec2 inV2)
{
#if defined(MOSS_SIMD_SSE)
	return _mm_and_ps(inV1.mValue, inV2.mValue);
#elif defined(MOSS_SIMD_NEON)
	return vreinterpretq_f32_u32(vandq_u32(vreinterpretq_u32_f32(inV1.mValue), vreinterpretq_u32_f32(inV2.mValue)));
#else
	return Vec2(UVec4::sAnd(inV1.ReinterpretAsInt(), inV2.ReinterpretAsInt()).ReinterpretAsFloat());
#endif
}

Vec2 Vec2::sUnitSpherical(float inTheta, float inPhi)
{
    Vec4 s, c;
    Vec4(inTheta, inPhi, 0, 0).SinCos(s, c);
    // Return just the X and Y components, ignore Z
    return Vec2(s.GetX() * c.GetY(), s.GetX() * s.GetY());
}

template <class Random>
Vec2 Vec2::sRandom(Random &inRandom)
{
	std::uniform_real_distribution<float> zero_to_one(0.0f, 1.0f);
	float theta = MOSS_PI * zero_to_one(inRandom);
	float phi = 2.0f * MOSS_PI * zero_to_one(inRandom);
	return sUnitSpherical(theta, phi);
}

bool Vec2::operator == (const Vec2 inV2) const
{
	return sEquals(*this, inV2).TestAllXYZTrue();
}

bool Vec2::IsClose(const Vec2 inV2, float inMaxDistSq) const
{
	return (inV2 - *this).LengthSq() <= inMaxDistSq;
}

bool Vec2::IsNearZero(float inMaxDistSq) const
{
	return LengthSq() <= inMaxDistSq;
}

Vec2 Vec2::operator * (const Vec2 inV2) const
{
#if defined(MOSS_SIMD_SSE)
	return _mm_mul_ps(mValue, inV2.mValue);
#elif defined(MOSS_SIMD_NEON)
	return vmulq_f32(mValue, inV2.mValue);
#else
	return Vec2(mF32[0] * inV2.mF32[0], mF32[1] * inV2.mF32[1], mF32[2] * inV2.mF32[2]);
#endif
}

Vec2 Vec2::operator * (float inV2) const
{
#if defined(MOSS_SIMD_SSE)
	return _mm_mul_ps(mValue, _mm_set1_ps(inV2));
#elif defined(MOSS_SIMD_NEON)
	return vmulq_n_f32(mValue, inV2);
#else
	return Vec2(mF32[0] * inV2, mF32[1] * inV2, mF32[2] * inV2);
#endif
}

Vec2 operator * (float inV1, const Vec2 inV2)
{
#if defined(MOSS_SIMD_SSE)
	return _mm_mul_ps(_mm_set1_ps(inV1), inV2.mValue);
#elif defined(MOSS_SIMD_NEON)
	return vmulq_n_f32(inV2.mValue, inV1);
#else
	return Vec2(inV1 * inV2.mF32[0], inV1 * inV2.mF32[1], inV1 * inV2.mF32[2]);
#endif
}

Vec2 Vec2::operator / (float inV2) const
{
#if defined(MOSS_SIMD_SSE)
	return _mm_div_ps(mValue, _mm_set1_ps(inV2));
#elif defined(MOSS_SIMD_NEON)
	return vdivq_f32(mValue, vdupq_n_f32(inV2));
#else
	return Vec2(mF32[0] / inV2, mF32[1] / inV2, mF32[2] / inV2);
#endif
}

Vec2 &Vec2::operator *= (float inV2)
{
#if defined(MOSS_SIMD_SSE)
	mValue = _mm_mul_ps(mValue, _mm_set1_ps(inV2));
#elif defined(MOSS_SIMD_NEON)
	mValue = vmulq_n_f32(mValue, inV2);
#else
	for (int i = 0; i < 3; ++i)
		mF32[i] *= inV2;
	#ifdef MOSS_FLOATING_POINT_EXCEPTIONS_ENABLED
		mF32[3] = mF32[2];
	#endif
#endif
	return *this;
}

Vec2 &Vec2::operator *= (const Vec2 inV2)
{
#if defined(MOSS_SIMD_SSE)
	mValue = _mm_mul_ps(mValue, inV2.mValue);
#elif defined(MOSS_SIMD_NEON)
	mValue = vmulq_f32(mValue, inV2.mValue);
#else
	for (int i = 0; i < 3; ++i)
		mF32[i] *= inV2.mF32[i];
	#ifdef MOSS_FLOATING_POINT_EXCEPTIONS_ENABLED
		mF32[3] = mF32[2];
	#endif
#endif
	return *this;
}

Vec2 &Vec2::operator /= (float inV2)
{
#if defined(MOSS_SIMD_SSE)
	mValue = _mm_div_ps(mValue, _mm_set1_ps(inV2));
#elif defined(MOSS_SIMD_NEON)
	mValue = vdivq_f32(mValue, vdupq_n_f32(inV2));
#else
	for (int i = 0; i < 3; ++i)
		mF32[i] /= inV2;
	#ifdef MOSS_FLOATING_POINT_EXCEPTIONS_ENABLED
		mF32[3] = mF32[2];
	#endif
#endif
	return *this;
}

Vec2 Vec2::operator + (const Vec2 inV2) const
{
#if defined(MOSS_SIMD_SSE)
	return _mm_add_ps(mValue, inV2.mValue);
#elif defined(MOSS_SIMD_NEON)
	return vaddq_f32(mValue, inV2.mValue);
#else
	return Vec2(mF32[0] + inV2.mF32[0], mF32[1] + inV2.mF32[1], mF32[2] + inV2.mF32[2]);
#endif
}

Vec2 &Vec2::operator += (const Vec2 inV2)
{
#if defined(MOSS_SIMD_SSE)
	mValue = _mm_add_ps(mValue, inV2.mValue);
#elif defined(MOSS_SIMD_NEON)
	mValue = vaddq_f32(mValue, inV2.mValue);
#else
	for (int i = 0; i < 3; ++i)
		mF32[i] += inV2.mF32[i];
	#ifdef MOSS_FLOATING_POINT_EXCEPTIONS_ENABLED
		mF32[3] = mF32[2];
	#endif
#endif
	return *this;
}

Vec2 Vec2::operator - () const
{
#if defined(MOSS_SIMD_SSE)
	return _mm_sub_ps(_mm_setzero_ps(), mValue);
#elif defined(MOSS_SIMD_NEON)
	#ifdef MOSS_CROSS_PLATFORM_DETERMINISTIC
		return vsubq_f32(vdupq_n_f32(0), mValue);
	#else
		return vnegq_f32(mValue);
	#endif
#else
	#ifdef MOSS_CROSS_PLATFORM_DETERMINISTIC
		return Vec2(0.0f - mF32[0], 0.0f - mF32[1], 0.0f - mF32[2]);
	#else
		return Vec2(-mF32[0], -mF32[1], -mF32[2]);
	#endif
#endif
}

Vec2 Vec2::operator - (const Vec2 inV2) const
{
#if defined(MOSS_SIMD_SSE)
	return _mm_sub_ps(mValue, inV2.mValue);
#elif defined(MOSS_SIMD_NEON)
	return vsubq_f32(mValue, inV2.mValue);
#else
	return Vec2(mF32[0] - inV2.mF32[0], mF32[1] - inV2.mF32[1], mF32[2] - inV2.mF32[2]);
#endif
}

Vec2 &Vec2::operator -= (const Vec2 inV2)
{
#if defined(MOSS_SIMD_SSE)
	mValue = _mm_sub_ps(mValue, inV2.mValue);
#elif defined(MOSS_SIMD_NEON)
	mValue = vsubq_f32(mValue, inV2.mValue);
#else
	for (int i = 0; i < 3; ++i)
		mF32[i] -= inV2.mF32[i];
	#ifdef MOSS_FLOATING_POINT_EXCEPTIONS_ENABLED
		mF32[3] = mF32[2];
	#endif
#endif
	return *this;
}

Vec2 Vec2::operator / (const Vec2 inV2) const
{
	inV2.CheckW(); // Check W equals Z to avoid div by zero
#if defined(MOSS_SIMD_SSE)
	return _mm_div_ps(mValue, inV2.mValue);
#elif defined(MOSS_SIMD_NEON)
	return vdivq_f32(mValue, inV2.mValue);
#else
	return Vec2(mF32[0] / inV2.mF32[0], mF32[1] / inV2.mF32[1], mF32[2] / inV2.mF32[2]);
#endif
}

Vec4 Vec2::SplatX() const
{
#if defined(MOSS_SIMD_SSE)
	return _mm_shuffle_ps(mValue, mValue, _MM_SHUFFLE(0, 0, 0, 0));
#elif defined(MOSS_SIMD_NEON)
	return vdupq_laneq_f32(mValue, 0);
#else
	return Vec4(mF32[0], mF32[0], mF32[0], mF32[0]);
#endif
}

Vec4 Vec2::SplatY() const
{
#if defined(MOSS_SIMD_SSE)
	return _mm_shuffle_ps(mValue, mValue, _MM_SHUFFLE(1, 1, 1, 1));
#elif defined(MOSS_SIMD_NEON)
	return vdupq_laneq_f32(mValue, 1);
#else
	return Vec4(mF32[1], mF32[1], mF32[1], mF32[1]);
#endif
}

Vec4 Vec2::SplatZ() const
{
#if defined(MOSS_SIMD_SSE)
	return _mm_shuffle_ps(mValue, mValue, _MM_SHUFFLE(2, 2, 2, 2));
#elif defined(MOSS_SIMD_NEON)
	return vdupq_laneq_f32(mValue, 2);
#else
	return Vec4(mF32[2], mF32[2], mF32[2], mF32[2]);
#endif
}

int Vec2::GetLowestComponentIndex() const { return GetX() < GetY() ? 0 : 1; }
int Vec2::GetHighestComponentIndex() const { return GetX() > GetY() ? 0 : 1; }

Vec2 Vec2::Abs() const
{
#if defined(MOSS_SIMD_AVX512)
	return _mm_range_ps(mValue, mValue, 0b1000);
#elif defined(MOSS_SIMD_SSE)
	return _mm_max_ps(_mm_sub_ps(_mm_setzero_ps(), mValue), mValue);
#elif defined(MOSS_SIMD_NEON)
	return vabsq_f32(mValue);
#else
	return Vec2(abs(mF32[0]), abs(mF32[1]), abs(mF32[2]));
#endif
}

Vec2 Vec2::Reciprocal() const
{
	return sReplicate(1.0f) / mValue;
}

Vec2 Vec2::Cross(const Vec2 inV2) const
{
#if defined(MOSS_SIMD_SSE)
	Type t1 = _mm_shuffle_ps(inV2.mValue, inV2.mValue, _MM_SHUFFLE(0, 0, 2, 1)); // Assure Z and W are the same
	t1 = _mm_mul_ps(t1, mValue);
	Type t2 = _mm_shuffle_ps(mValue, mValue, _MM_SHUFFLE(0, 0, 2, 1)); // Assure Z and W are the same
	t2 = _mm_mul_ps(t2, inV2.mValue);
	Type t3 = _mm_sub_ps(t1, t2);
	return _mm_shuffle_ps(t3, t3, _MM_SHUFFLE(0, 0, 2, 1)); // Assure Z and W are the same
#elif defined(MOSS_SIMD_NEON)
	Type t1 = MOSS_NEON_SHUFFLE_F32x4(inV2.mValue, inV2.mValue, 1, 2, 0, 0); // Assure Z and W are the same
	t1 = vmulq_f32(t1, mValue);
	Type t2 = MOSS_NEON_SHUFFLE_F32x4(mValue, mValue, 1, 2, 0, 0); // Assure Z and W are the same
	t2 = vmulq_f32(t2, inV2.mValue);
	Type t3 = vsubq_f32(t1, t2);
	return MOSS_NEON_SHUFFLE_F32x4(t3, t3, 1, 2, 0, 0); // Assure Z and W are the same
#else
	return Vec2(mF32[1] * inV2.mF32[2] - mF32[2] * inV2.mF32[1],
				mF32[2] * inV2.mF32[0] - mF32[0] * inV2.mF32[2],
				mF32[0] * inV2.mF32[1] - mF32[1] * inV2.mF32[0]);
#endif
}

Vec2 Vec2::DotV(const Vec2 inV2) const
{
#if defined(MOSS_SIMD_SSE4_1)
	return _mm_dp_ps(mValue, inV2.mValue, 0x7f);
#elif defined(MOSS_SIMD_NEON)
	float32x4_t mul = vmulq_f32(mValue, inV2.mValue);
	mul = vsetq_lane_f32(0, mul, 3);
	return vdupq_n_f32(vaddvq_f32(mul));
#else
	float dot = 0.0f;
	for (int i = 0; i < 3; i++)
		dot += mF32[i] * inV2.mF32[i];
	return Vec2::sReplicate(dot);
#endif
}

Vec4 Vec2::DotV4(const Vec2 inV2) const
{
#if defined(MOSS_SIMD_SSE4_1)
	return _mm_dp_ps(mValue, inV2.mValue, 0x7f);
#elif defined(MOSS_SIMD_NEON)
	float32x4_t mul = vmulq_f32(mValue, inV2.mValue);
	mul = vsetq_lane_f32(0, mul, 3);
	return vdupq_n_f32(vaddvq_f32(mul));
#else
	float dot = 0.0f;
	for (int i = 0; i < 3; i++)
		dot += mF32[i] * inV2.mF32[i];
	return Vec4::sReplicate(dot);
#endif
}

float Vec2::Dot(const Vec2 inV2) const
{
#if defined(MOSS_SIMD_SSE4_1)
	return _mm_cvtss_f32(_mm_dp_ps(mValue, inV2.mValue, 0x7f));
#elif defined(MOSS_SIMD_NEON)
	float32x4_t mul = vmulq_f32(mValue, inV2.mValue);
	mul = vsetq_lane_f32(0, mul, 3);
	return vaddvq_f32(mul);
#else
	float dot = 0.0f;
	for (int i = 0; i < 3; i++)
		dot += mF32[i] * inV2.mF32[i];
	return dot;
#endif
}

float Vec2::LengthSq() const
{
#if defined(MOSS_SIMD_SSE4_1)
	return _mm_cvtss_f32(_mm_dp_ps(mValue, mValue, 0x7f));
#elif defined(MOSS_SIMD_NEON)
	float32x4_t mul = vmulq_f32(mValue, mValue);
	mul = vsetq_lane_f32(0, mul, 3);
	return vaddvq_f32(mul);
#else
	float len_sq = 0.0f;
	for (int i = 0; i < 3; i++)
		len_sq += mF32[i] * mF32[i];
	return len_sq;
#endif
}

float Vec2::Length() const
{
#if defined(MOSS_SIMD_SSE4_1)
	return _mm_cvtss_f32(_mm_sqrt_ss(_mm_dp_ps(mValue, mValue, 0x7f)));
#elif defined(MOSS_SIMD_NEON)
	float32x4_t mul = vmulq_f32(mValue, mValue);
	mul = vsetq_lane_f32(0, mul, 3);
	float32x2_t sum = vdup_n_f32(vaddvq_f32(mul));
	return vget_lane_f32(vsqrt_f32(sum), 0);
#else
	return sqrt(LengthSq());
#endif
}

Vec2 Vec2::Sqrt() const
{
#if defined(MOSS_SIMD_SSE)
	return _mm_sqrt_ps(mValue);
#elif defined(MOSS_SIMD_NEON)
	return vsqrtq_f32(mValue);
#else
	return Vec2(sqrt(mF32[0]), sqrt(mF32[1]), sqrt(mF32[2]));
#endif
}

Vec2 Vec2::Normalized() const
{
#if defined(MOSS_SIMD_SSE4_1)
	return _mm_div_ps(mValue, _mm_sqrt_ps(_mm_dp_ps(mValue, mValue, 0x7f)));
#elif defined(MOSS_SIMD_NEON)
	float32x4_t mul = vmulq_f32(mValue, mValue);
	mul = vsetq_lane_f32(0, mul, 3);
	float32x4_t sum = vdupq_n_f32(vaddvq_f32(mul));
	return vdivq_f32(mValue, vsqrtq_f32(sum));
#else
	return *this / Length();
#endif
}

Vec2 Vec2::NormalizedOr(const Vec2 inZeroValue) const
{
#if defined(MOSS_SIMD_SSE4_1) && !defined(MOSS_PLATFORM_WASM) // _mm_blendv_ps has problems on FireFox
	Type len_sq = _mm_dp_ps(mValue, mValue, 0x7f);
	Type is_zero = _mm_cmpeq_ps(len_sq, _mm_setzero_ps());
#ifdef MOSS_FLOATING_POINT_EXCEPTIONS_ENABLED
	if (_mm_movemask_ps(is_zero) == 0xf)
		return inZeroValue;
	else
		return _mm_div_ps(mValue, _mm_sqrt_ps(len_sq));
#else
	return _mm_blendv_ps(_mm_div_ps(mValue, _mm_sqrt_ps(len_sq)), inZeroValue.mValue, is_zero);
#endif // MOSS_FLOATING_POINT_EXCEPTIONS_ENABLED
#elif defined(MOSS_SIMD_NEON)
	float32x4_t mul = vmulq_f32(mValue, mValue);
	mul = vsetq_lane_f32(0, mul, 3);
	float32x4_t sum = vdupq_n_f32(vaddvq_f32(mul));
	float32x4_t len = vsqrtq_f32(sum);
	uint32x4_t is_zero = vceqq_f32(len, vdupq_n_f32(0));
	return vbslq_f32(is_zero, inZeroValue.mValue, vdivq_f32(mValue, len));
#else
	float len_sq = LengthSq();
	if (len_sq == 0.0f)
		return inZeroValue;
	else
		return *this / sqrt(len_sq);
#endif
}

bool Vec2::IsNormalized(float inTolerance) const
{
	return abs(LengthSq() - 1.0f) <= inTolerance;
}

bool Vec2::IsNaN() const
{
#if defined(MOSS_SIMD_AVX512)
	return (_mm_fpclass_ps_mask(mValue, 0b10000001) & 0x7) != 0;
#elif defined(MOSS_SIMD_SSE)
	return (_mm_movemask_ps(_mm_cmpunord_ps(mValue, mValue)) & 0x7) != 0;
#elif defined(MOSS_SIMD_NEON)
	uint32x4_t mask = MOSS_NEON_UINT32x4(1, 1, 1, 0);
	uint32x4_t is_equal = vceqq_f32(mValue, mValue); // If a number is not equal to itself it's a NaN
	return vaddvq_u32(vandq_u32(is_equal, mask)) != 3;
#else
	return isnan(mF32[0]) || isnan(mF32[1]) || isnan(mF32[2]);
#endif
}

void Vec2::StoreFloat2(Float2 *outV) const
{
#if defined(MOSS_SIMD_SSE)
    _mm_store_ss(&outV->x, mValue);
    __m128 t = _mm_shuffle_ps(mValue, mValue, _MM_SHUFFLE(1,1,1,1)); // get y component
    _mm_store_ss(&outV->y, t);
#elif defined(MOSS_SIMD_NEON)
    float32x2_t xy = vget_low_f32(mValue);
    vst1_f32(&outV->x, xy);
#else
    outV->x = mF32[0];
    outV->y = mF32[1];
#endif
}

UVec4 Vec2::ToInt() const
{
#if defined(MOSS_SIMD_SSE)
	return _mm_cvttps_epi32(mValue);
#elif defined(MOSS_SIMD_NEON)
	return vcvtq_u32_f32(mValue);
#else
	return UVec4(uint32(mF32[0]), uint32(mF32[1]), uint32(mF32[2]), uint32(mF32[3]));
#endif
}

UVec4 Vec2::ReinterpretAsInt() const
{
#if defined(MOSS_SIMD_SSE)
	return UVec4(_mm_castps_si128(mValue));
#elif defined(MOSS_SIMD_NEON)
	return vreinterpretq_u32_f32(mValue);
#else
	return *reinterpret_cast<const UVec4 *>(this);
#endif
}

float Vec2::ReduceMin() const
{
#if defined(MOSS_SIMD_SSE)
	__m128 temp = _mm_min_ps(mValue, _mm_shuffle_ps(mValue, mValue, _MM_SHUFFLE(0, 0, 0, 1)));
    return _mm_cvtss_f32(temp);
#elif defined(MOSS_SIMD_NEON)
	float32x2_t xy = vget_low_f32(mValue);
    float32x2_t minVal = vpmin_f32(xy, xy); // pairwise min
    return vget_lane_f32(minVal, 0);
#else
	return std::min(mF32[0], mF32[1]);
#endif 
}

float Vec2::ReduceMax() const
{
#if defined(MOSS_SIMD_SSE)
	__m128 temp = _mm_max_ps(mValue, _mm_shuffle_ps(mValue, mValue, _MM_SHUFFLE(0, 0, 0, 1)));
    return _mm_cvtss_f32(temp);
#elif defined(MOSS_SIMD_NEON)
	float32x2_t xy = vget_low_f32(mValue);
    float32x2_t maxVal = vpmax_f32(xy, xy); // pairwise max
    return vget_lane_f32(maxVal, 0);
#else
	return std::max(mF32[0], mF32[1]);
#endif
}

Vec2 Vec2::GetNormalizedPerpendicular() const
{
    // Perpendicular vector: (y, -x)
    Vec2 perp(mF32[1], -mF32[0]);

    float len = sqrt(perp.mF32[0] * perp.mF32[0] + perp.mF32[1] * perp.mF32[1]);
    if (len == 0.0f) return Vec2(0.0f, 0.0f);  // avoid division by zero

    return perp / len;
}


Vec2 Vec2::GetSign() const
{
#if defined(MOSS_SIMD_AVX512)
	return _mm_fixupimm_ps(mValue, mValue, _mm_set1_epi32(0xA9A90A00), 0);
#elif defined(MOSS_SIMD_SSE)
	Type minus_one = _mm_set1_ps(-1.0f);
	Type one = _mm_set1_ps(1.0f);
	return _mm_or_ps(_mm_and_ps(mValue, minus_one), one);
#elif defined(MOSS_SIMD_NEON)
	Type minus_one = vdupq_n_f32(-1.0f);
	Type one = vdupq_n_f32(1.0f);
	return vreinterpretq_f32_u32(vorrq_u32(vandq_u32(vreinterpretq_u32_f32(mValue), vreinterpretq_u32_f32(minus_one)), vreinterpretq_u32_f32(one)));
#else
	return Vec2(std::signbit(mF32[0])? -1.0f : 1.0f, std::signbit(mF32[1])? -1.0f : 1.0f);
#endif
}

MOSS_SUPRESS_WARNINGS_END
