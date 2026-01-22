// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT


#include <Moss/Physics/physics_intern.h>
#ifndef MOSS_DEBUG_RENDERER
	#include <Moss/Moss_Renderer.h>
#endif // MOSS_DEBUG_RENDERER

#include <Moss/Core/StreamIn.h>
#include <Moss/Core/StreamOut.h>

MOSS_SUPRESS_WARNINGS_BEGIN


// Approximation of a face of the tapered cylinder
static const Vec3 cTaperedCylinderFace[] =
{
	Vec3(0.0f,			0.0f,	1.0f),
	Vec3(0.707106769f,	0.0f,	0.707106769f),
	Vec3(1.0f,			0.0f,	0.0f),
	Vec3(0.707106769f,	0.0f,	-0.707106769f),
	Vec3(-0.0f,			0.0f,	-1.0f),
	Vec3(-0.707106769f,	0.0f,	-0.707106769f),
	Vec3(-1.0f,			0.0f,	0.0f),
	Vec3(-0.707106769f,	0.0f,	0.707106769f)
};

static const int cCapsuleDetailLevel = 2;

// Codecs this mesh shape is using
using TriangleCodec = TriangleCodecIndexed8BitPackSOA4Flags;
using NodeCodec = NodeCodecQuadTreeHalfFloat;


static const TStaticArray<Vec3, 192> sCapsuleTopTriangles = []() {
	TStaticArray<Vec3, 192> verts;
	GetTrianglesContextVertexList::sCreateHalfUnitSphereTop(verts, cCapsuleDetailLevel);
	return verts;
}();

static const TStaticArray<Vec3, 96> sCapsuleMiddleTriangles = []() {
	TStaticArray<Vec3, 96> verts;
	GetTrianglesContextVertexList::sCreateUnitOpenCylinder(verts, cCapsuleDetailLevel);
	return verts;
}();

static const TStaticArray<Vec3, 192> sCapsuleBottomTriangles = []() {
	TStaticArray<Vec3, 192> verts;
	GetTrianglesContextVertexList::sCreateHalfUnitSphereBottom(verts, cCapsuleDetailLevel);
	return verts;
}();

#ifndef MOSS_DEBUG_RENDERER
bool Shape::sDrawSubmergedVolumes = false;
#endif // MOSS_DEBUG_RENDERER

ShapeFunctions ShapeFunctions::sRegistry[NumSubShapeTypes];

static const Vec3 sUnitBoxTriangles[] = {
	Vec3(-1, 1, -1),	Vec3(-1, 1, 1),		Vec3(1, 1, 1),
	Vec3(-1, 1, -1),	Vec3(1, 1, 1),		Vec3(1, 1, -1),
	Vec3(-1, -1, -1),	Vec3(1, -1, -1),	Vec3(1, -1, 1),
	Vec3(-1, -1, -1),	Vec3(1, -1, 1),		Vec3(-1, -1, 1),
	Vec3(-1, 1, -1),	Vec3(-1, -1, -1),	Vec3(-1, -1, 1),
	Vec3(-1, 1, -1),	Vec3(-1, -1, 1),	Vec3(-1, 1, 1),
	Vec3(1, 1, 1),		Vec3(1, -1, 1),		Vec3(1, -1, -1),
	Vec3(1, 1, 1),		Vec3(1, -1, -1),	Vec3(1, 1, -1),
	Vec3(-1, 1, 1),		Vec3(-1, -1, 1),	Vec3(1, -1, 1),
	Vec3(-1, 1, 1),		Vec3(1, -1, 1),		Vec3(1, 1, 1),
	Vec3(-1, 1, -1),	Vec3(1, 1, -1),		Vec3(1, -1, -1),
	Vec3(-1, 1, -1),	Vec3(1, -1, -1),	Vec3(-1, -1, -1)
};
// Approximation of top face with 8 vertices
static const Vec3 cCylinderTopFace[] =
{
	Vec3(0.0f,			1.0f,	1.0f),
	Vec3(0.707106769f,	1.0f,	0.707106769f),
	Vec3(1.0f,			1.0f,	0.0f),
	Vec3(0.707106769f,	1.0f,	-0.707106769f),
	Vec3(-0.0f,			1.0f,	-1.0f),
	Vec3(-0.707106769f,	1.0f,	-0.707106769f),
	Vec3(-1.0f,			1.0f,	0.0f),
	Vec3(-0.707106769f,	1.0f,	0.707106769f)
};

static const TStaticArray<Vec3, 96> sUnitCylinderTriangles = []() {
	TStaticArray<Vec3, 96> verts;

	const Vec3 bottom_offset(0.0f, -2.0f, 0.0f);

	int num_verts = sizeof(cCylinderTopFace) / sizeof(Vec3);
	for (int i = 0; i < num_verts; ++i)
	{
		Vec3 t1 = cCylinderTopFace[i];
		Vec3 t2 = cCylinderTopFace[(i + 1) % num_verts];
		Vec3 b1 = cCylinderTopFace[i] + bottom_offset;
		Vec3 b2 = cCylinderTopFace[(i + 1) % num_verts] + bottom_offset;

		// Top
		verts.emplace_back(0.0f, 1.0f, 0.0f);
		verts.push_back(t1);
		verts.push_back(t2);

		// Bottom
		verts.emplace_back(0.0f, -1.0f, 0.0f);
		verts.push_back(b2);
		verts.push_back(b1);

		// Side
		verts.push_back(t1);
		verts.push_back(b1);
		verts.push_back(t2);

		verts.push_back(t2);
		verts.push_back(b1);
		verts.push_back(b2);
	}

	return verts;
}();


const TStaticArray<Vec3, 384> ConvexShape::sUnitSphereTriangles = []() {
	const int level = 2;

	TStaticArray<Vec3, 384> verts;
	GetTrianglesContextVertexList::sCreateHalfUnitSphereTop(verts, level);
	GetTrianglesContextVertexList::sCreateHalfUnitSphereBottom(verts, level);
	return verts;
}();

const uint HeightFieldShape::sGridOffsets[] =
{
	0,			// level:  0, max x/y:     0, offset: 0
	1,			// level:  1, max x/y:     1, offset: 1
	5,			// level:  2, max x/y:     3, offset: 1 + 4
	21,			// level:  3, max x/y:     7, offset: 1 + 4 + 16
	85,			// level:  4, max x/y:    15, offset: 1 + 4 + 16 + 64
	341,		// level:  5, max x/y:    31, offset: 1 + 4 + 16 + 64 + 256
	1365,		// level:  6, max x/y:    63, offset: 1 + 4 + 16 + 64 + 256 + 1024
	5461,		// level:  7, max x/y:   127, offset: 1 + 4 + 16 + 64 + 256 + 1024 + 4096
	21845,		// level:  8, max x/y:   255, offset: 1 + 4 + 16 + 64 + 256 + 1024 + 4096 + ...
	87381,		// level:  9, max x/y:   511, offset: 1 + 4 + 16 + 64 + 256 + 1024 + 4096 + ...
	349525,		// level: 10, max x/y:  1023, offset: 1 + 4 + 16 + 64 + 256 + 1024 + 4096 + ...
	1398101,	// level: 11, max x/y:  2047, offset: 1 + 4 + 16 + 64 + 256 + 1024 + 4096 + ...
	5592405,	// level: 12, max x/y:  4095, offset: 1 + 4 + 16 + 64 + 256 + 1024 + 4096 + ...
	22369621,	// level: 13, max x/y:  8191, offset: 1 + 4 + 16 + 64 + 256 + 1024 + 4096 + ...
	89478485,	// level: 14, max x/y: 16383, offset: 1 + 4 + 16 + 64 + 256 + 1024 + 4096 + ...
};


MOSS_IMPLEMENT_SERIALIZABLE_VIRTUAL(SphereShapeSettings)
{
	MOSS_ADD_BASE_CLASS(SphereShapeSettings, ConvexShapeSettings)

	MOSS_ADD_ATTRIBUTE(SphereShapeSettings, mRadius)
}
MOSS_IMPLEMENT_SERIALIZABLE_VIRTUAL(TaperedCapsuleShapeSettings)
{
	MOSS_ADD_BASE_CLASS(TaperedCapsuleShapeSettings, ConvexShapeSettings)

	MOSS_ADD_ATTRIBUTE(TaperedCapsuleShapeSettings, mHalfHeightOfTaperedCylinder)
	MOSS_ADD_ATTRIBUTE(TaperedCapsuleShapeSettings, mTopRadius)
	MOSS_ADD_ATTRIBUTE(TaperedCapsuleShapeSettings, mBottomRadius)
}
MOSS_IMPLEMENT_SERIALIZABLE_VIRTUAL(TaperedCylinderShapeSettings)
{
	MOSS_ADD_BASE_CLASS(TaperedCylinderShapeSettings, ConvexShapeSettings)

	MOSS_ADD_ATTRIBUTE(TaperedCylinderShapeSettings, mHalfHeight)
	MOSS_ADD_ATTRIBUTE(TaperedCylinderShapeSettings, mTopRadius)
	MOSS_ADD_ATTRIBUTE(TaperedCylinderShapeSettings, mBottomRadius)
	MOSS_ADD_ATTRIBUTE(TaperedCylinderShapeSettings, mConvexRadius)
}
MOSS_IMPLEMENT_SERIALIZABLE_VIRTUAL(StaticCompoundShapeSettings)
{
	MOSS_ADD_BASE_CLASS(StaticCompoundShapeSettings, CompoundShapeSettings)
}
MOSS_IMPLEMENT_SERIALIZABLE_VIRTUAL(TriangleShapeSettings)
{
	MOSS_ADD_BASE_CLASS(TriangleShapeSettings, ConvexShapeSettings)

	MOSS_ADD_ATTRIBUTE(TriangleShapeSettings, mV1)
	MOSS_ADD_ATTRIBUTE(TriangleShapeSettings, mV2)
	MOSS_ADD_ATTRIBUTE(TriangleShapeSettings, mV3)
	MOSS_ADD_ATTRIBUTE(TriangleShapeSettings, mConvexRadius)
}

MOSS_IMPLEMENT_SERIALIZABLE_ABSTRACT_BASE(ShapeSettings)
{
	MOSS_ADD_BASE_CLASS(ShapeSettings, SerializableObject)

	MOSS_ADD_ATTRIBUTE(ShapeSettings, mUserData)
}
MOSS_IMPLEMENT_SERIALIZABLE_VIRTUAL(CapsuleShapeSettings)
{
	MOSS_ADD_BASE_CLASS(CapsuleShapeSettings, ConvexShapeSettings)

	MOSS_ADD_ATTRIBUTE(CapsuleShapeSettings, mRadius)
	MOSS_ADD_ATTRIBUTE(CapsuleShapeSettings, mHalfHeightOfCylinder)
}
MOSS_IMPLEMENT_SERIALIZABLE_VIRTUAL(BoxShapeSettings)
{
	MOSS_ADD_BASE_CLASS(BoxShapeSettings, ConvexShapeSettings)

	MOSS_ADD_ATTRIBUTE(BoxShapeSettings, mHalfExtent)
	MOSS_ADD_ATTRIBUTE(BoxShapeSettings, mConvexRadius)
}
MOSS_IMPLEMENT_SERIALIZABLE_ABSTRACT(CompoundShapeSettings)
{
	MOSS_ADD_BASE_CLASS(CompoundShapeSettings, ShapeSettings)

	MOSS_ADD_ATTRIBUTE(CompoundShapeSettings, mSubShapes)
}

MOSS_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(CompoundShapeSettings::SubShapeSettings)
{
	MOSS_ADD_ATTRIBUTE(CompoundShapeSettings::SubShapeSettings, mShape)
	MOSS_ADD_ATTRIBUTE(CompoundShapeSettings::SubShapeSettings, mPosition)
	MOSS_ADD_ATTRIBUTE(CompoundShapeSettings::SubShapeSettings, mRotation)
	MOSS_ADD_ATTRIBUTE(CompoundShapeSettings::SubShapeSettings, mUserData)
}

MOSS_IMPLEMENT_SERIALIZABLE_ABSTRACT(ConvexShapeSettings)
{
	MOSS_ADD_BASE_CLASS(ConvexShapeSettings, ShapeSettings)

	MOSS_ADD_ATTRIBUTE(ConvexShapeSettings, mDensity)
	MOSS_ADD_ATTRIBUTE(ConvexShapeSettings, mMaterial)
}
MOSS_IMPLEMENT_SERIALIZABLE_VIRTUAL(ConvexHullShapeSettings)
{
	MOSS_ADD_BASE_CLASS(ConvexHullShapeSettings, ConvexShapeSettings)

	MOSS_ADD_ATTRIBUTE(ConvexHullShapeSettings, mPoints)
	MOSS_ADD_ATTRIBUTE(ConvexHullShapeSettings, mMaxConvexRadius)
	MOSS_ADD_ATTRIBUTE(ConvexHullShapeSettings, mMaxErrorConvexRadius)
	MOSS_ADD_ATTRIBUTE(ConvexHullShapeSettings, mHullTolerance)
}
MOSS_IMPLEMENT_SERIALIZABLE_VIRTUAL(CylinderShapeSettings)
{
	MOSS_ADD_BASE_CLASS(CylinderShapeSettings, ConvexShapeSettings)

	MOSS_ADD_ATTRIBUTE(CylinderShapeSettings, mHalfHeight)
	MOSS_ADD_ATTRIBUTE(CylinderShapeSettings, mRadius)
	MOSS_ADD_ATTRIBUTE(CylinderShapeSettings, mConvexRadius)
}

MOSS_IMPLEMENT_SERIALIZABLE_ABSTRACT(DecoratedShapeSettings)
{
	MOSS_ADD_BASE_CLASS(DecoratedShapeSettings, ShapeSettings)

	MOSS_ADD_ATTRIBUTE(DecoratedShapeSettings, mInnerShape)
}
MOSS_IMPLEMENT_SERIALIZABLE_VIRTUAL(EmptyShapeSettings)
{
	MOSS_ADD_BASE_CLASS(EmptyShapeSettings, ShapeSettings)

	MOSS_ADD_ATTRIBUTE(EmptyShapeSettings, mCenterOfMass)
}
MOSS_IMPLEMENT_SERIALIZABLE_VIRTUAL(HeightFieldShapeSettings)
{
	MOSS_ADD_BASE_CLASS(HeightFieldShapeSettings, ShapeSettings)

	MOSS_ADD_ATTRIBUTE(HeightFieldShapeSettings, mHeightSamples)
	MOSS_ADD_ATTRIBUTE(HeightFieldShapeSettings, mOffset)
	MOSS_ADD_ATTRIBUTE(HeightFieldShapeSettings, mScale)
	MOSS_ADD_ATTRIBUTE(HeightFieldShapeSettings, mMinHeightValue)
	MOSS_ADD_ATTRIBUTE(HeightFieldShapeSettings, mMaxHeightValue)
	MOSS_ADD_ATTRIBUTE(HeightFieldShapeSettings, mMaterialsCapacity)
	MOSS_ADD_ATTRIBUTE(HeightFieldShapeSettings, mSampleCount)
	MOSS_ADD_ATTRIBUTE(HeightFieldShapeSettings, mBlockSize)
	MOSS_ADD_ATTRIBUTE(HeightFieldShapeSettings, mBitsPerSample)
	MOSS_ADD_ATTRIBUTE(HeightFieldShapeSettings, mMaterialIndices)
	MOSS_ADD_ATTRIBUTE(HeightFieldShapeSettings, mMaterials)
	MOSS_ADD_ATTRIBUTE(HeightFieldShapeSettings, mActiveEdgeCosThresholdAngle)
}
MOSS_IMPLEMENT_SERIALIZABLE_VIRTUAL(MeshShapeSettings)
{
	MOSS_ADD_BASE_CLASS(MeshShapeSettings, ShapeSettings)

	MOSS_ADD_ATTRIBUTE(MeshShapeSettings, mTriangleVertices)
	MOSS_ADD_ATTRIBUTE(MeshShapeSettings, mIndexedTriangles)
	MOSS_ADD_ATTRIBUTE(MeshShapeSettings, mMaterials)
	MOSS_ADD_ATTRIBUTE(MeshShapeSettings, mMaxTrianglesPerLeaf)
	MOSS_ADD_ATTRIBUTE(MeshShapeSettings, mActiveEdgeCosThresholdAngle)
	MOSS_ADD_ATTRIBUTE(MeshShapeSettings, mPerTriangleUserData)
	MOSS_ADD_ENUM_ATTRIBUTE(MeshShapeSettings, mBuildQuality)
}
MOSS_IMPLEMENT_SERIALIZABLE_VIRTUAL(MutableCompoundShapeSettings)
{
	MOSS_ADD_BASE_CLASS(MutableCompoundShapeSettings, CompoundShapeSettings)
}
MOSS_IMPLEMENT_SERIALIZABLE_VIRTUAL(RotatedTranslatedShapeSettings)
{
	MOSS_ADD_BASE_CLASS(RotatedTranslatedShapeSettings, DecoratedShapeSettings)

	MOSS_ADD_ATTRIBUTE(RotatedTranslatedShapeSettings, mPosition)
	MOSS_ADD_ATTRIBUTE(RotatedTranslatedShapeSettings, mRotation)
}
MOSS_IMPLEMENT_SERIALIZABLE_VIRTUAL(PlaneShapeSettings)
{
	MOSS_ADD_BASE_CLASS(PlaneShapeSettings, ShapeSettings)

	MOSS_ADD_ATTRIBUTE(PlaneShapeSettings, mPlane)
	MOSS_ADD_ATTRIBUTE(PlaneShapeSettings, mMaterial)
	MOSS_ADD_ATTRIBUTE(PlaneShapeSettings, mHalfExtent)
}
MOSS_IMPLEMENT_SERIALIZABLE_VIRTUAL(ScaledShapeSettings)
{
	MOSS_ADD_BASE_CLASS(ScaledShapeSettings, DecoratedShapeSettings)

	MOSS_ADD_ATTRIBUTE(ScaledShapeSettings, mScale)
}



/*													*/

ShapeSettings::ShapeResult SphereShapeSettings::Create() const
{
	if (mCachedResult.IsEmpty())
		Ref<Shape> shape = new SphereShape(*this, mCachedResult);
	return mCachedResult;
}

SphereShape::SphereShape(const SphereShapeSettings &inSettings, ShapeResult &outResult) :
	ConvexShape(EShapeSubType::Sphere, inSettings, outResult),
	mRadius(inSettings.mRadius)
{
	if (inSettings.mRadius <= 0.0f)
	{
		outResult.SetError("Invalid radius");
		return;
	}

	outResult.Set(this);
}

float SphereShape::GetScaledRadius(Vec3Arg inScale) const
{
	MOSS_ASSERT(IsValidScale(inScale));

	Vec3 abs_scale = inScale.Abs();
	return abs_scale.GetX() * mRadius;
}

AABox SphereShape::GetLocalBounds() const
{
	Vec3 half_extent = Vec3::sReplicate(mRadius);
	return AABox(-half_extent, half_extent);
}

AABox SphereShape::GetWorldSpaceBounds(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale) const
{
	float scaled_radius = GetScaledRadius(inScale);
	Vec3 half_extent = Vec3::sReplicate(scaled_radius);
	AABox bounds(-half_extent, half_extent);
	bounds.Translate(inCenterOfMassTransform.GetTranslation());
	return bounds;
}

class SphereShape::SphereNoConvex final : public Support
{
public:
	explicit		SphereNoConvex(float inRadius) :
		mRadius(inRadius)
	{
		static_assert(sizeof(SphereNoConvex) <= sizeof(SupportBuffer), "Buffer size too small");
		MOSS_ASSERT(IsAligned(this, alignof(SphereNoConvex)));
	}

	virtual Vec3	GetSupport(Vec3Arg inDirection) const override
	{
		return Vec3::sZero();
	}

	virtual float	GetConvexRadius() const override
	{
		return mRadius;
	}

private:
	float			mRadius;
};

class SphereShape::SphereWithConvex final : public Support
{
public:
	explicit		SphereWithConvex(float inRadius) :
		mRadius(inRadius)
	{
		static_assert(sizeof(SphereWithConvex) <= sizeof(SupportBuffer), "Buffer size too small");
		MOSS_ASSERT(IsAligned(this, alignof(SphereWithConvex)));
	}

	virtual Vec3	GetSupport(Vec3Arg inDirection) const override
	{
		float len = inDirection.Length();
		return len > 0.0f? (mRadius / len) * inDirection : Vec3::sZero();
	}

	virtual float	GetConvexRadius() const override
	{
		return 0.0f;
	}

private:
	float			mRadius;
};

const ConvexShape::Support *SphereShape::GetSupportFunction(ESupportMode inMode, SupportBuffer &inBuffer, Vec3Arg inScale) const
{
	float scaled_radius = GetScaledRadius(inScale);

	switch (inMode)
	{
	case ESupportMode::IncludeConvexRadius:
		return new (&inBuffer) SphereWithConvex(scaled_radius);

	case ESupportMode::ExcludeConvexRadius:
	case ESupportMode::Default:
		return new (&inBuffer) SphereNoConvex(scaled_radius);
	}

	MOSS_ASSERT(false);
	return nullptr;
}

MassProperties SphereShape::GetMassProperties() const
{
	MassProperties p;

	// Calculate mass
	float r2 = mRadius * mRadius;
	p.mMass = (4.0f / 3.0f * MOSS_PI) * mRadius * r2 * GetDensity();

	// Calculate inertia
	float inertia = (2.0f / 5.0f) * p.mMass * r2;
	p.mInertia = Mat44::sScale(inertia);

	return p;
}

Vec3 SphereShape::GetSurfaceNormal(const SubShapeID &inSubShapeID, Vec3Arg inLocalSurfacePosition) const
{
	MOSS_ASSERT(inSubShapeID.IsEmpty(), "Invalid subshape ID");

	float len = inLocalSurfacePosition.Length();
	return len != 0.0f? inLocalSurfacePosition / len : Vec3::sAxisY();
}

void SphereShape::GetSubmergedVolume(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const Plane &inSurface, float &outTotalVolume, float &outSubmergedVolume, Vec3 &outCenterOfBuoyancy MOSS_IF_DEBUG_RENDERER(, RVec3Arg inBaseOffset)) const
{
	float scaled_radius = GetScaledRadius(inScale);
	outTotalVolume = (4.0f / 3.0f * MOSS_PI) * Cubed(scaled_radius);

	float distance_to_surface = inSurface.SignedDistance(inCenterOfMassTransform.GetTranslation());
	if (distance_to_surface >= scaled_radius)
	{
		// Above surface
		outSubmergedVolume = 0.0f;
		outCenterOfBuoyancy = Vec3::sZero();
	}
	else if (distance_to_surface <= -scaled_radius)
	{
		// Under surface
		outSubmergedVolume = outTotalVolume;
		outCenterOfBuoyancy = inCenterOfMassTransform.GetTranslation();
	}
	else
	{
		// Intersecting surface

		// Calculate submerged volume, see: https://en.wikipedia.org/wiki/Spherical_cap
		float h = scaled_radius - distance_to_surface;
		outSubmergedVolume = (MOSS_PI / 3.0f) * Square(h) * (3.0f * scaled_radius - h);

		// Calculate center of buoyancy, see: http://mathworld.wolfram.com/SphericalCap.html (eq 10)
		float z = (3.0f / 4.0f) * Square(2.0f * scaled_radius - h) / (3.0f * scaled_radius - h);
		outCenterOfBuoyancy = inCenterOfMassTransform.GetTranslation() - z * inSurface.GetNormal(); // Negative normal since we want the portion under the water

	#ifndef MOSS_DEBUG_RENDERER
		// Draw intersection between sphere and water plane
		if (sDrawSubmergedVolumes)
		{
			Vec3 circle_center = inCenterOfMassTransform.GetTranslation() - distance_to_surface * inSurface.GetNormal();
			float circle_radius = sqrt(Square(scaled_radius) - Square(distance_to_surface));
			DebugRenderer::sInstance->DrawPie(inBaseOffset + circle_center, circle_radius, inSurface.GetNormal(), inSurface.GetNormal().GetNormalizedPerpendicular(), -MOSS_PI, MOSS_PI, Color::sGreen, DebugRenderer::ECastShadow::Off);
		}
	#endif // MOSS_DEBUG_RENDERER
	}

#ifndef MOSS_DEBUG_RENDERER
	// Draw center of buoyancy
	if (sDrawSubmergedVolumes)
		DebugRenderer::sInstance->DrawWireSphere(inBaseOffset + outCenterOfBuoyancy, 0.05f, Color::sRed, 1);
#endif // MOSS_DEBUG_RENDERER
}

#ifndef MOSS_DEBUG_RENDERER
void SphereShape::Draw(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, Vec3Arg inScale, ColorArg inColor, bool inUseMaterialColors, bool inDrawWireframe) const
{
	DebugRenderer::EDrawMode draw_mode = inDrawWireframe? DebugRenderer::EDrawMode::Wireframe : DebugRenderer::EDrawMode::Solid;
	inRenderer->DrawUnitSphere(inCenterOfMassTransform * Mat44::sScale(mRadius * inScale.Abs().GetX()), inUseMaterialColors? GetMaterial()->GetDebugColor() : inColor, DebugRenderer::ECastShadow::On, draw_mode);
}
#endif // MOSS_DEBUG_RENDERER

bool SphereShape::CastRay(const RayCast &inRay, const SubShapeIDCreator &inSubShapeIDCreator, RayCastResult &ioHit) const
{
	float fraction = RaySphere(inRay.mOrigin, inRay.mDirection, Vec3::sZero(), mRadius);
	if (fraction < ioHit.mFraction)
	{
		ioHit.mFraction = fraction;
		ioHit.mSubShapeID2 = inSubShapeIDCreator.GetID();
		return true;
	}
	return false;
}

void SphereShape::CastRay(const RayCast &inRay, const RayCastSettings &inRayCastSettings, const SubShapeIDCreator &inSubShapeIDCreator, CastRayCollector &ioCollector, const ShapeFilter &inShapeFilter) const
{
	// Test shape filter
	if (!inShapeFilter.ShouldCollide(this, inSubShapeIDCreator.GetID()))
		return;

	float min_fraction, max_fraction;
	int num_results = RaySphere(inRay.mOrigin, inRay.mDirection, Vec3::sZero(), mRadius, min_fraction, max_fraction);
	if (num_results > 0 // Ray should intersect
		&& max_fraction >= 0.0f // End of ray should be inside sphere
		&& min_fraction < ioCollector.GetEarlyOutFraction()) // Start of ray should be before early out fraction
	{
		// Better hit than the current hit
		RayCastResult hit;
		hit.mBodyID = TransformedShape::sGetBodyID(ioCollector.GetContext());
		hit.mSubShapeID2 = inSubShapeIDCreator.GetID();

		// Check front side hit
		if (inRayCastSettings.mTreatConvexAsSolid || min_fraction > 0.0f)
		{
			hit.mFraction = max(0.0f, min_fraction);
			ioCollector.AddHit(hit);
		}

		// Check back side hit
		if (inRayCastSettings.mBackFaceModeConvex == EBackFaceMode::CollideWithBackFaces
			&& num_results > 1 // Ray should have 2 intersections
			&& max_fraction < ioCollector.GetEarlyOutFraction()) // End of ray should be before early out fraction
		{
			hit.mFraction = max_fraction;
			ioCollector.AddHit(hit);
		}
	}
}

void SphereShape::CollidePoint(Vec3Arg inPoint, const SubShapeIDCreator &inSubShapeIDCreator, CollidePointCollector &ioCollector, const ShapeFilter &inShapeFilter) const
{
	// Test shape filter
	if (!inShapeFilter.ShouldCollide(this, inSubShapeIDCreator.GetID()))
		return;

	if (inPoint.LengthSq() <= Square(mRadius))
		ioCollector.AddHit({ TransformedShape::sGetBodyID(ioCollector.GetContext()), inSubShapeIDCreator.GetID() });
}

void SphereShape::CollideSoftBodyVertices(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const CollideSoftBodyVertexIterator &inVertices, uint inNumVertices, int inCollidingShapeIndex) const
{
	Vec3 center = inCenterOfMassTransform.GetTranslation();
	float radius = GetScaledRadius(inScale);

	for (CollideSoftBodyVertexIterator v = inVertices, sbv_end = inVertices + inNumVertices; v != sbv_end; ++v)
		if (v.GetInvMass() > 0.0f)
		{
			// Calculate penetration
			Vec3 delta = v.GetPosition() - center;
			float distance = delta.Length();
			float penetration = radius - distance;
			if (v.UpdatePenetration(penetration))
			{
				// Calculate contact point and normal
				Vec3 normal = distance > 0.0f? delta / distance : Vec3::sAxisY();
				Vec3 point = center + radius * normal;

				// Store collision
				v.SetCollision(Plane::sFromPointAndNormal(point, normal), inCollidingShapeIndex);
			}
		}
}

void SphereShape::GetTrianglesStart(GetTrianglesContext &ioContext, const AABox &inBox, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale) const
{
	float scaled_radius = GetScaledRadius(inScale);
	new (&ioContext) GetTrianglesContextVertexList(inPositionCOM, inRotation, Vec3::sOne(), Mat44::sScale(scaled_radius), sUnitSphereTriangles.data(), sUnitSphereTriangles.size(), GetMaterial());
}

int SphereShape::GetTrianglesNext(GetTrianglesContext &ioContext, int inMaxTrianglesRequested, Float3 *outTriangleVertices, const PhysicsMaterial **outMaterials) const
{
	return ((GetTrianglesContextVertexList &)ioContext).GetTrianglesNext(inMaxTrianglesRequested, outTriangleVertices, outMaterials);
}

void SphereShape::SaveBinaryState(StreamOut &inStream) const
{
	ConvexShape::SaveBinaryState(inStream);

	inStream.Write(mRadius);
}

void SphereShape::RestoreBinaryState(StreamIn &inStream)
{
	ConvexShape::RestoreBinaryState(inStream);

	inStream.Read(mRadius);
}

bool SphereShape::IsValidScale(Vec3Arg inScale) const
{
	return ConvexShape::IsValidScale(inScale) && ScaleHelpers::IsUniformScale(inScale.Abs());
}

Vec3 SphereShape::MakeScaleValid(Vec3Arg inScale) const
{
	Vec3 scale = ScaleHelpers::MakeNonZeroScale(inScale);

	return scale.GetSign() * ScaleHelpers::MakeUniformScale(scale.Abs());
}

void SphereShape::sRegister()
{
	ShapeFunctions &f = ShapeFunctions::sGet(EShapeSubType::Sphere);
	f.mConstruct = []() -> Shape * { return new SphereShape; };
	f.mColor = Color::sGreen;
}

/*													*/

bool TaperedCapsuleShapeSettings::IsSphere() const
{
	return max(mTopRadius, mBottomRadius) >= 2.0f * mHalfHeightOfTaperedCylinder + min(mTopRadius, mBottomRadius);
}

ShapeSettings::ShapeResult TaperedCapsuleShapeSettings::Create() const
{
	if (mCachedResult.IsEmpty())
	{
		Ref<Shape> shape;
		if (IsValid() && IsSphere())
		{
			// Determine sphere center and radius
			float radius, center;
			if (mTopRadius > mBottomRadius)
			{
				radius = mTopRadius;
				center = mHalfHeightOfTaperedCylinder;
			}
			else
			{
				radius = mBottomRadius;
				center = -mHalfHeightOfTaperedCylinder;
			}

			// Create sphere
			shape = new SphereShape(radius, mMaterial);

			// Offset sphere if needed
			if (abs(center) > 1.0e-6f)
			{
				RotatedTranslatedShapeSettings rot_trans(Vec3(0, center, 0), Quat::sIdentity(), shape);
				mCachedResult = rot_trans.Create();
			}
			else
				mCachedResult.Set(shape);
		}
		else
		{
			// Normal tapered capsule shape
			shape = new TaperedCapsuleShape(*this, mCachedResult);
		}
	}
	return mCachedResult;
}

TaperedCapsuleShapeSettings::TaperedCapsuleShapeSettings(float inHalfHeightOfTaperedCylinder, float inTopRadius, float inBottomRadius, const PhysicsMaterial *inMaterial) :
	ConvexShapeSettings(inMaterial),
	mHalfHeightOfTaperedCylinder(inHalfHeightOfTaperedCylinder),
	mTopRadius(inTopRadius),
	mBottomRadius(inBottomRadius)
{
}

TaperedCapsuleShape::TaperedCapsuleShape(const TaperedCapsuleShapeSettings &inSettings, ShapeResult &outResult) :
	ConvexShape(EShapeSubType::TaperedCapsule, inSettings, outResult),
	mTopRadius(inSettings.mTopRadius),
	mBottomRadius(inSettings.mBottomRadius)
{
	if (mTopRadius <= 0.0f)
	{
		outResult.SetError("Invalid top radius");
		return;
	}

	if (mBottomRadius <= 0.0f)
	{
		outResult.SetError("Invalid bottom radius");
		return;
	}

	if (inSettings.mHalfHeightOfTaperedCylinder <= 0.0f)
	{
		outResult.SetError("Invalid height");
		return;
	}

	// If this goes off one of the sphere ends falls totally inside the other and you should use a sphere instead
	if (inSettings.IsSphere())
	{
		outResult.SetError("One sphere embedded in other sphere, please use sphere shape instead");
		return;
	}

	// Approximation: The center of mass is exactly half way between the top and bottom cap of the tapered capsule
	mTopCenter = inSettings.mHalfHeightOfTaperedCylinder + 0.5f * (mBottomRadius - mTopRadius);
	mBottomCenter = -inSettings.mHalfHeightOfTaperedCylinder + 0.5f * (mBottomRadius - mTopRadius);

	// Calculate center of mass
	mCenterOfMass = Vec3(0, inSettings.mHalfHeightOfTaperedCylinder - mTopCenter, 0);

	// Calculate convex radius
	mConvexRadius = min(mTopRadius, mBottomRadius);
	MOSS_ASSERT(mConvexRadius > 0.0f);

	// Calculate the sin and tan of the angle that the cone surface makes with the Y axis
	// See: TaperedCapsuleShape.gliffy
	mSinAlpha = (mBottomRadius - mTopRadius) / (mTopCenter - mBottomCenter);
	MOSS_ASSERT(mSinAlpha >= -1.0f && mSinAlpha <= 1.0f);
	mTanAlpha = Tan(ASin(mSinAlpha));

	outResult.Set(this);
}

class TaperedCapsuleShape::TaperedCapsule final : public Support
{
public:
					TaperedCapsule(Vec3Arg inTopCenter, Vec3Arg inBottomCenter, float inTopRadius, float inBottomRadius, float inConvexRadius) :
		mTopCenter(inTopCenter),
		mBottomCenter(inBottomCenter),
		mTopRadius(inTopRadius),
		mBottomRadius(inBottomRadius),
		mConvexRadius(inConvexRadius)
	{
		static_assert(sizeof(TaperedCapsule) <= sizeof(SupportBuffer), "Buffer size too small");
		MOSS_ASSERT(IsAligned(this, alignof(TaperedCapsule)));
	}

	virtual Vec3	GetSupport(Vec3Arg inDirection) const override
	{
		// Check zero vector
		float len = inDirection.Length();
		if (len == 0.0f)
			return mTopCenter + Vec3(0, mTopRadius, 0); // Return top

		// Check if the support of the top sphere or bottom sphere is bigger
		Vec3 support_top = mTopCenter + (mTopRadius / len) * inDirection;
		Vec3 support_bottom = mBottomCenter + (mBottomRadius / len) * inDirection;
		if (support_top.Dot(inDirection) > support_bottom.Dot(inDirection))
			return support_top;
		else
			return support_bottom;
	}

	virtual float	GetConvexRadius() const override
	{
		return mConvexRadius;
	}

private:
	Vec3			mTopCenter;
	Vec3			mBottomCenter;
	float			mTopRadius;
	float			mBottomRadius;
	float			mConvexRadius;
};

const ConvexShape::Support *TaperedCapsuleShape::GetSupportFunction(ESupportMode inMode, SupportBuffer &inBuffer, Vec3Arg inScale) const
{
	MOSS_ASSERT(IsValidScale(inScale));

	// Get scaled tapered capsule
	Vec3 abs_scale = inScale.Abs();
	float scale_xz = abs_scale.GetX();
	float scale_y = inScale.GetY(); // The sign of y is important as it flips the tapered capsule
	Vec3 scaled_top_center = Vec3(0, scale_y * mTopCenter, 0);
	Vec3 scaled_bottom_center = Vec3(0, scale_y * mBottomCenter, 0);
	float scaled_top_radius = scale_xz * mTopRadius;
	float scaled_bottom_radius = scale_xz * mBottomRadius;
	float scaled_convex_radius = scale_xz * mConvexRadius;

	switch (inMode)
	{
	case ESupportMode::IncludeConvexRadius:
		return new (&inBuffer) TaperedCapsule(scaled_top_center, scaled_bottom_center, scaled_top_radius, scaled_bottom_radius, 0.0f);

	case ESupportMode::ExcludeConvexRadius:
	case ESupportMode::Default:
		{
			// Get radii reduced by convex radius
			float tr = scaled_top_radius - scaled_convex_radius;
			float br = scaled_bottom_radius - scaled_convex_radius;
			MOSS_ASSERT(tr >= 0.0f && br >= 0.0f);
			MOSS_ASSERT(tr == 0.0f || br == 0.0f, "Convex radius should be that of the smallest sphere");
			return new (&inBuffer) TaperedCapsule(scaled_top_center, scaled_bottom_center, tr, br, scaled_convex_radius);
		}
	}

	MOSS_ASSERT(false);
	return nullptr;
}

void TaperedCapsuleShape::GetSupportingFace(const SubShapeID &inSubShapeID, Vec3Arg inDirection, Vec3Arg inScale, Mat44Arg inCenterOfMassTransform, SupportingFace &outVertices) const
{
	MOSS_ASSERT(inSubShapeID.IsEmpty(), "Invalid subshape ID");
	MOSS_ASSERT(IsValidScale(inScale));

	// Check zero vector
	float len = inDirection.Length();
	if (len == 0.0f)
		return;

	// Get scaled tapered capsule
	Vec3 abs_scale = inScale.Abs();
	float scale_xz = abs_scale.GetX();
	float scale_y = inScale.GetY(); // The sign of y is important as it flips the tapered capsule
	Vec3 scaled_top_center = Vec3(0, scale_y * mTopCenter, 0);
	Vec3 scaled_bottom_center = Vec3(0, scale_y * mBottomCenter, 0);
	float scaled_top_radius = scale_xz * mTopRadius;
	float scaled_bottom_radius = scale_xz * mBottomRadius;

	// Get support point for top and bottom sphere in the opposite of inDirection (including convex radius)
	Vec3 support_top = scaled_top_center - (scaled_top_radius / len) * inDirection;
	Vec3 support_bottom = scaled_bottom_center - (scaled_bottom_radius / len) * inDirection;

	// Get projection on inDirection
	float proj_top = support_top.Dot(inDirection);
	float proj_bottom = support_bottom.Dot(inDirection);

	// If projection is roughly equal then return line, otherwise we return nothing as there's only 1 point
	if (abs(proj_top - proj_bottom) < cCapsuleProjectionSlop * len)
	{
		outVertices.push_back(inCenterOfMassTransform * support_top);
		outVertices.push_back(inCenterOfMassTransform * support_bottom);
	}
}

MassProperties TaperedCapsuleShape::GetMassProperties() const
{
	AABox box = GetInertiaApproximation();

	MassProperties p;
	p.SetMassAndInertiaOfSolidBox(box.GetSize(), GetDensity());
	return p;
}

Vec3 TaperedCapsuleShape::GetSurfaceNormal(const SubShapeID &inSubShapeID, Vec3Arg inLocalSurfacePosition) const
{
	MOSS_ASSERT(inSubShapeID.IsEmpty(), "Invalid subshape ID");

	// See: TaperedCapsuleShape.gliffy
	// We need to calculate ty and by in order to see if the position is on the top or bottom sphere
	// sin(alpha) = by / br = ty / tr
	// => by = sin(alpha) * br, ty = sin(alpha) * tr

	if (inLocalSurfacePosition.GetY() > mTopCenter + mSinAlpha * mTopRadius)
		return (inLocalSurfacePosition - Vec3(0, mTopCenter, 0)).Normalized();
	else if (inLocalSurfacePosition.GetY() < mBottomCenter + mSinAlpha * mBottomRadius)
		return (inLocalSurfacePosition - Vec3(0, mBottomCenter, 0)).Normalized();
	else
	{
		// Get perpendicular vector to the surface in the xz plane
		Vec3 perpendicular = Vec3(inLocalSurfacePosition.GetX(), 0, inLocalSurfacePosition.GetZ()).NormalizedOr(Vec3::sAxisX());

		// We know that the perpendicular has length 1 and that it needs a y component where tan(alpha) = y / 1 in order to align it to the surface
		perpendicular.SetY(mTanAlpha);
		return perpendicular.Normalized();
	}
}

AABox TaperedCapsuleShape::GetLocalBounds() const
{
	float max_radius = max(mTopRadius, mBottomRadius);
	return AABox(Vec3(-max_radius, mBottomCenter - mBottomRadius, -max_radius), Vec3(max_radius, mTopCenter + mTopRadius, max_radius));
}

AABox TaperedCapsuleShape::GetWorldSpaceBounds(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale) const
{
	MOSS_ASSERT(IsValidScale(inScale));

	Vec3 abs_scale = inScale.Abs();
	float scale_xz = abs_scale.GetX();
	float scale_y = inScale.GetY(); // The sign of y is important as it flips the tapered capsule
	Vec3 bottom_extent = Vec3::sReplicate(scale_xz * mBottomRadius);
	Vec3 bottom_center = inCenterOfMassTransform * Vec3(0, scale_y * mBottomCenter, 0);
	Vec3 top_extent = Vec3::sReplicate(scale_xz * mTopRadius);
	Vec3 top_center = inCenterOfMassTransform * Vec3(0, scale_y * mTopCenter, 0);
	Vec3 p1 = Vec3::sMin(top_center - top_extent, bottom_center - bottom_extent);
	Vec3 p2 = Vec3::sMax(top_center + top_extent, bottom_center + bottom_extent);
	return AABox(p1, p2);
}

void TaperedCapsuleShape::CollideSoftBodyVertices(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const CollideSoftBodyVertexIterator &inVertices, uint inNumVertices, int inCollidingShapeIndex) const
{
	MOSS_ASSERT(IsValidScale(inScale));

	Mat44 inverse_transform = inCenterOfMassTransform.InversedRotationTranslation();

	// Get scaled tapered capsule
	Vec3 abs_scale = inScale.Abs();
	float scale_y = abs_scale.GetY();
	float scale_xz = abs_scale.GetX();
	Vec3 scale_y_flip(1, Sign(inScale.GetY()), 1);
	Vec3 scaled_top_center(0, scale_y * mTopCenter, 0);
	Vec3 scaled_bottom_center(0, scale_y * mBottomCenter, 0);
	float scaled_top_radius = scale_xz * mTopRadius;
	float scaled_bottom_radius = scale_xz * mBottomRadius;

	for (CollideSoftBodyVertexIterator v = inVertices, sbv_end = inVertices + inNumVertices; v != sbv_end; ++v)
		if (v.GetInvMass() > 0.0f)
		{
			Vec3 local_pos = scale_y_flip * (inverse_transform * v.GetPosition());

			Vec3 position, normal;

			// If the vertex is inside the cone starting at the top center pointing along the y-axis with angle PI/2 - alpha then the closest point is on the top sphere
			// This corresponds to: Dot(y-axis, (local_pos - top_center) / |local_pos - top_center|) >= cos(PI/2 - alpha)
			// <=> (local_pos - top_center).y >= sin(alpha) * |local_pos - top_center|
			Vec3 top_center_to_local_pos = local_pos - scaled_top_center;
			float top_center_to_local_pos_len = top_center_to_local_pos.Length();
			if (top_center_to_local_pos.GetY() >= mSinAlpha * top_center_to_local_pos_len)
			{
				// Top sphere
				normal = top_center_to_local_pos_len != 0.0f? top_center_to_local_pos / top_center_to_local_pos_len : Vec3::sAxisY();
				position = scaled_top_center + scaled_top_radius * normal;
			}
			else
			{
				// If the vertex is outside the cone starting at the bottom center pointing along the y-axis with angle PI/2 - alpha then the closest point is on the bottom sphere
				// This corresponds to: Dot(y-axis, (local_pos - bottom_center) / |local_pos - bottom_center|) <= cos(PI/2 - alpha)
				// <=> (local_pos - bottom_center).y <= sin(alpha) * |local_pos - bottom_center|
				Vec3 bottom_center_to_local_pos = local_pos - scaled_bottom_center;
				float bottom_center_to_local_pos_len = bottom_center_to_local_pos.Length();
				if (bottom_center_to_local_pos.GetY() <= mSinAlpha * bottom_center_to_local_pos_len)
				{
					// Bottom sphere
					normal = bottom_center_to_local_pos_len != 0.0f? bottom_center_to_local_pos / bottom_center_to_local_pos_len : -Vec3::sAxisY();
				}
				else
				{
					// Tapered cylinder
					normal = Vec3(local_pos.GetX(), 0, local_pos.GetZ()).NormalizedOr(Vec3::sAxisX());
					normal.SetY(mTanAlpha);
					normal = normal.NormalizedOr(Vec3::sAxisX());
				}
				position = scaled_bottom_center + scaled_bottom_radius * normal;
			}

			Plane plane = Plane::sFromPointAndNormal(position, normal);
			float penetration = -plane.SignedDistance(local_pos);
			if (v.UpdatePenetration(penetration))
			{
				// Need to flip the normal's y if capsule is flipped (this corresponds to flipping both the point and the normal around y)
				plane.SetNormal(scale_y_flip * plane.GetNormal());

				// Store collision
				v.SetCollision(plane.GetTransformed(inCenterOfMassTransform), inCollidingShapeIndex);
			}
		}
}

#ifndef MOSS_DEBUG_RENDERER
void TaperedCapsuleShape::Draw(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, Vec3Arg inScale, ColorArg inColor, bool inUseMaterialColors, bool inDrawWireframe) const
{
	if (mGeometry == nullptr)
	{
		SupportBuffer buffer;
		const Support *support = GetSupportFunction(ESupportMode::IncludeConvexRadius, buffer, Vec3::sOne());
		mGeometry = inRenderer->CreateTriangleGeometryForConvex([support](Vec3Arg inDirection) { return support->GetSupport(inDirection); });
	}

	// Preserve flip along y axis but make sure we're not inside out
	Vec3 scale = ScaleHelpers::IsInsideOut(inScale)? Vec3(-1, 1, 1) * inScale : inScale;
	RMat44 world_transform = inCenterOfMassTransform * Mat44::sScale(scale);

	AABox bounds = Shape::GetWorldSpaceBounds(inCenterOfMassTransform, inScale);

	float lod_scale_sq = Square(max(mTopRadius, mBottomRadius));

	Color color = inUseMaterialColors? GetMaterial()->GetDebugColor() : inColor;

	DebugRenderer::EDrawMode draw_mode = inDrawWireframe? DebugRenderer::EDrawMode::Wireframe : DebugRenderer::EDrawMode::Solid;

	inRenderer->DrawGeometry(world_transform, bounds, lod_scale_sq, color, mGeometry, DebugRenderer::ECullMode::CullBackFace, DebugRenderer::ECastShadow::On, draw_mode);
}
#endif // MOSS_DEBUG_RENDERER

AABox TaperedCapsuleShape::GetInertiaApproximation() const
{
	// TODO: For now the mass and inertia is that of a box
	float avg_radius = 0.5f * (mTopRadius + mBottomRadius);
	return AABox(Vec3(-avg_radius, mBottomCenter - mBottomRadius, -avg_radius), Vec3(avg_radius, mTopCenter + mTopRadius, avg_radius));
}

void TaperedCapsuleShape::SaveBinaryState(StreamOut &inStream) const
{
	ConvexShape::SaveBinaryState(inStream);

	inStream.Write(mCenterOfMass);
	inStream.Write(mTopRadius);
	inStream.Write(mBottomRadius);
	inStream.Write(mTopCenter);
	inStream.Write(mBottomCenter);
	inStream.Write(mConvexRadius);
	inStream.Write(mSinAlpha);
	inStream.Write(mTanAlpha);
}

void TaperedCapsuleShape::RestoreBinaryState(StreamIn &inStream)
{
	ConvexShape::RestoreBinaryState(inStream);

	inStream.Read(mCenterOfMass);
	inStream.Read(mTopRadius);
	inStream.Read(mBottomRadius);
	inStream.Read(mTopCenter);
	inStream.Read(mBottomCenter);
	inStream.Read(mConvexRadius);
	inStream.Read(mSinAlpha);
	inStream.Read(mTanAlpha);
}

bool TaperedCapsuleShape::IsValidScale(Vec3Arg inScale) const
{
	return ConvexShape::IsValidScale(inScale) && ScaleHelpers::IsUniformScale(inScale.Abs());
}

Vec3 TaperedCapsuleShape::MakeScaleValid(Vec3Arg inScale) const
{
	Vec3 scale = ScaleHelpers::MakeNonZeroScale(inScale);

	return scale.GetSign() * ScaleHelpers::MakeUniformScale(scale.Abs());
}

void TaperedCapsuleShape::sRegister()
{
	ShapeFunctions &f = ShapeFunctions::sGet(EShapeSubType::TaperedCapsule);
	f.mConstruct = []() -> Shape * { return new TaperedCapsuleShape; };
	f.mColor = Color::sGreen;
}

/*													*/

ShapeSettings::ShapeResult TaperedCylinderShapeSettings::Create() const
{
	if (mCachedResult.IsEmpty())
	{
		Ref<Shape> shape;
		if (mTopRadius == mBottomRadius)
		{
			// Convert to regular cylinder
			CylinderShapeSettings settings;
			settings.mHalfHeight = mHalfHeight;
			settings.mRadius = mTopRadius;
			settings.mMaterial = mMaterial;
			settings.mConvexRadius = mConvexRadius;
			new CylinderShape(settings, mCachedResult);
		}
		else
		{
			// Normal tapered cylinder shape
			new TaperedCylinderShape(*this, mCachedResult);
		}
	}
	return mCachedResult;
}

TaperedCylinderShapeSettings::TaperedCylinderShapeSettings(float inHalfHeightOfTaperedCylinder, float inTopRadius, float inBottomRadius, float inConvexRadius, const PhysicsMaterial *inMaterial) :
	ConvexShapeSettings(inMaterial),
	mHalfHeight(inHalfHeightOfTaperedCylinder),
	mTopRadius(inTopRadius),
	mBottomRadius(inBottomRadius),
	mConvexRadius(inConvexRadius)
{
}

TaperedCylinderShape::TaperedCylinderShape(const TaperedCylinderShapeSettings &inSettings, ShapeResult &outResult) :
	ConvexShape(EShapeSubType::TaperedCylinder, inSettings, outResult),
	mTopRadius(inSettings.mTopRadius),
	mBottomRadius(inSettings.mBottomRadius),
	mConvexRadius(inSettings.mConvexRadius)
{
	if (mTopRadius < 0.0f)
	{
		outResult.SetError("Invalid top radius");
		return;
	}

	if (mBottomRadius < 0.0f)
	{
		outResult.SetError("Invalid bottom radius");
		return;
	}

	if (inSettings.mHalfHeight <= 0.0f)
	{
		outResult.SetError("Invalid height");
		return;
	}

	if (inSettings.mConvexRadius < 0.0f)
	{
		outResult.SetError("Invalid convex radius");
		return;
	}

	if (inSettings.mTopRadius < inSettings.mConvexRadius)
	{
		outResult.SetError("Convex radius must be smaller than convex radius");
		return;
	}

	if (inSettings.mBottomRadius < inSettings.mConvexRadius)
	{
		outResult.SetError("Convex radius must be smaller than bottom radius");
		return;
	}

	// Calculate the center of mass (using wxMaxima).
	// Radius of cross section for tapered cylinder from 0 to h:
	// r(x):=br+x*(tr-br)/h;
	// Area:
	// area(x):=%pi*r(x)^2;
	// Total volume of cylinder:
	// volume(h):=integrate(area(x),x,0,h);
	// Center of mass:
	// com(br,tr,h):=integrate(x*area(x),x,0,h)/volume(h);
	// Results:
	// ratsimp(com(br,tr,h),br,bt);
	// Non-tapered cylinder should have com = 0.5:
	// ratsimp(com(r,r,h));
	// Cone with tip at origin and height h should have com = 3/4 h
	// ratsimp(com(0,r,h));
	float h = 2.0f * inSettings.mHalfHeight;
	float tr = mTopRadius;
	float tr2 = Square(tr);
	float br = mBottomRadius;
	float br2 = Square(br);
	float com = h * (3 * tr2 + 2 * br * tr + br2) / (4.0f * (tr2 + br * tr + br2));
	mTop = h - com;
	mBottom = -com;

	outResult.Set(this);
}

class TaperedCylinderShape::TaperedCylinder final : public Support
{
public:
					TaperedCylinder(float inTop, float inBottom, float inTopRadius, float inBottomRadius, float inConvexRadius) :
		mTop(inTop),
		mBottom(inBottom),
		mTopRadius(inTopRadius),
		mBottomRadius(inBottomRadius),
		mConvexRadius(inConvexRadius)
	{
		static_assert(sizeof(TaperedCylinder) <= sizeof(SupportBuffer), "Buffer size too small");
		MOSS_ASSERT(IsAligned(this, alignof(TaperedCylinder)));
	}

	virtual Vec3	GetSupport(Vec3Arg inDirection) const override
	{
		float x = inDirection.GetX(), y = inDirection.GetY(), z = inDirection.GetZ();
		float o = sqrt(Square(x) + Square(z));
		if (o > 0.0f)
		{
			Vec3 top_support((mTopRadius * x) / o, mTop, (mTopRadius * z) / o);
			Vec3 bottom_support((mBottomRadius * x) / o, mBottom, (mBottomRadius * z) / o);
			return inDirection.Dot(top_support) > inDirection.Dot(bottom_support)? top_support : bottom_support;
		}
		else
		{
			if (y > 0.0f)
				return Vec3(0, mTop, 0);
			else
				return Vec3(0, mBottom, 0);
		}
	}

	virtual float	GetConvexRadius() const override
	{
		return mConvexRadius;
	}

private:
	float			mTop;
	float			mBottom;
	float			mTopRadius;
	float			mBottomRadius;
	float			mConvexRadius;
};

MOSS_INLINE void TaperedCylinderShape::GetScaled(Vec3Arg inScale, float &outTop, float &outBottom, float &outTopRadius, float &outBottomRadius, float &outConvexRadius) const
{
	Vec3 abs_scale = inScale.Abs();
	float scale_xz = abs_scale.GetX();
	float scale_y = inScale.GetY();

	outTop = scale_y * mTop;
	outBottom = scale_y * mBottom;
	outTopRadius = scale_xz * mTopRadius;
	outBottomRadius = scale_xz * mBottomRadius;
	outConvexRadius = min(abs_scale.GetY(), scale_xz) * mConvexRadius;

	// Negative Y-scale flips the top and bottom
	if (outBottom > outTop)
	{
		std::swap(outTop, outBottom);
		std::swap(outTopRadius, outBottomRadius);
	}
}

const ConvexShape::Support *TaperedCylinderShape::GetSupportFunction(ESupportMode inMode, SupportBuffer &inBuffer, Vec3Arg inScale) const
{
	MOSS_ASSERT(IsValidScale(inScale));

	// Get scaled tapered cylinder
	float top, bottom, top_radius, bottom_radius, convex_radius;
	GetScaled(inScale, top, bottom, top_radius, bottom_radius, convex_radius);

	switch (inMode)
	{
	case ESupportMode::IncludeConvexRadius:
	case ESupportMode::Default:
		return new (&inBuffer) TaperedCylinder(top, bottom, top_radius, bottom_radius, 0.0f);

	case ESupportMode::ExcludeConvexRadius:
		return new (&inBuffer) TaperedCylinder(top - convex_radius, bottom + convex_radius, top_radius - convex_radius, bottom_radius - convex_radius, convex_radius);
	}

	MOSS_ASSERT(false);
	return nullptr;
}

MOSS_INLINE static Vec3 sCalculateSideNormalXZ(Vec3Arg inSurfacePosition)
{
	return (Vec3(1, 0, 1) * inSurfacePosition).NormalizedOr(Vec3::sAxisX());
}

MOSS_INLINE static Vec3 sCalculateSideNormal(Vec3Arg inNormalXZ, float inTop, float inBottom, float inTopRadius, float inBottomRadius)
{
	float tan_alpha = (inBottomRadius - inTopRadius) / (inTop - inBottom);
	return Vec3(inNormalXZ.GetX(), tan_alpha, inNormalXZ.GetZ()).Normalized();
}

void TaperedCylinderShape::GetSupportingFace(const SubShapeID &inSubShapeID, Vec3Arg inDirection, Vec3Arg inScale, Mat44Arg inCenterOfMassTransform, SupportingFace &outVertices) const
{
	MOSS_ASSERT(inSubShapeID.IsEmpty(), "Invalid subshape ID");
	MOSS_ASSERT(IsValidScale(inScale));

	// Get scaled tapered cylinder
	float top, bottom, top_radius, bottom_radius, convex_radius;
	GetScaled(inScale, top, bottom, top_radius, bottom_radius, convex_radius);

	// Get the normal of the side of the cylinder
	Vec3 normal_xz = sCalculateSideNormalXZ(-inDirection);
	Vec3 normal = sCalculateSideNormal(normal_xz, top, bottom, top_radius, bottom_radius);

	constexpr float cMinRadius = 1.0e-3f;

	// Check if the normal is closer to the side than to the top or bottom
	if (abs(normal.Dot(inDirection)) > abs(inDirection.GetY()))
	{
		// Return the side of the cylinder
		outVertices.push_back(inCenterOfMassTransform * (normal_xz * top_radius + Vec3(0, top, 0)));
		outVertices.push_back(inCenterOfMassTransform * (normal_xz * bottom_radius + Vec3(0, bottom, 0)));
	}
	else
	{
		// When the inDirection is more than 5 degrees from vertical, align the vertices so that 1 of the vertices
		// points towards inDirection in the XZ plane. This ensures that we always have a vertex towards max penetration depth.
		Mat44 transform = inCenterOfMassTransform;
		Vec4 base_x = Vec4(inDirection.GetX(), 0, inDirection.GetZ(), 0);
		float xz_sq = base_x.LengthSq();
		float y_sq = Square(inDirection.GetY());
		if (xz_sq > 0.00765427f * y_sq)
		{
			base_x /= sqrt(xz_sq);
			Vec4 base_z = base_x.Swizzle<SWIZZLE_Z, SWIZZLE_Y, SWIZZLE_X, SWIZZLE_W>() * Vec4(-1, 0, 1, 0);
			transform = transform * Mat44(base_x, Vec4(0, 1, 0, 0), base_z, Vec4(0, 0, 0, 1));
		}

		if (inDirection.GetY() < 0.0f)
		{
			// Top of the cylinder
			if (top_radius > cMinRadius)
			{
				Vec3 top_3d(0, top, 0);
				for (Vec3 v : cTaperedCylinderFace)
					outVertices.push_back(transform * (top_radius * v + top_3d));
			}
		}
		else
		{
			// Bottom of the cylinder
			if (bottom_radius > cMinRadius)
			{
				Vec3 bottom_3d(0, bottom, 0);
				for (const Vec3 *v = cTaperedCylinderFace + std::size(cTaperedCylinderFace) - 1; v >= cTaperedCylinderFace; --v)
					outVertices.push_back(transform * (bottom_radius * *v + bottom_3d));
			}
		}
	}
}

MassProperties TaperedCylinderShape::GetMassProperties() const
{
	MassProperties p;

	// Calculate mass
	float density = GetDensity();
	p.mMass = GetVolume() * density;

	// Calculate inertia of a tapered cylinder (using wxMaxima)
	// Radius:
	// r(x):=br+(x-b)*(tr-br)/(t-b);
	// Where t=top, b=bottom, tr=top radius, br=bottom radius
	// Area of the cross section of the cylinder at x:
	// area(x):=%pi*r(x)^2;
	// Inertia x slice at x (using inertia of a solid disc, see https://en.wikipedia.org/wiki/List_of_moments_of_inertia, note needs to be multiplied by density):
	// dix(x):=area(x)*r(x)^2/4;
	// Inertia y slice at y (note needs to be multiplied by density)
	// diy(x):=area(x)*r(x)^2/2;
	// Volume:
	// volume(b,t):=integrate(area(x),x,b,t);
	// The constant density (note that we have this through GetDensity() so we'll use that instead):
	// density(b,t):=m/volume(b,t);
	// Inertia tensor element xx, note that we use the parallel axis theorem to move the inertia: Ixx' = Ixx + m translation^2, also note we multiply by density here:
	// Ixx(br,tr,b,t):=integrate(dix(x)+area(x)*x^2,x,b,t)*density(b,t);
	// Inertia tensor element yy:
	// Iyy(br,tr,b,t):=integrate(diy(x),x,b,t)*density(b,t);
	// Note that we can simplify Ixx by using:
	// Ixx_delta(br,tr,b,t):=Ixx(br,tr,b,t)-Iyy(br,tr,b,t)/2;
	// For a cylinder this formula matches what is listed on the wiki:
	// factor(Ixx(r,r,-h/2,h/2));
	// factor(Iyy(r,r,-h/2,h/2));
	// For a cone with tip at origin too:
	// factor(Ixx(0,r,0,h));
	// factor(Iyy(0,r,0,h));
	// Now for the tapered cylinder:
	// rat(Ixx(br,tr,b,t),br,bt);
	// rat(Iyy(br,tr,b,t),br,bt);
	// rat(Ixx_delta(br,tr,b,t),br,bt);
	float t = mTop;
	float t2 = Square(t);
	float t3 = t * t2;

	float b = mBottom;
	float b2 = Square(b);
	float b3 = b * b2;

	float br = mBottomRadius;
	float br2 = Square(br);
	float br3 = br * br2;
	float br4 = Square(br2);

	float tr = mTopRadius;
	float tr2 = Square(tr);
	float tr3 = tr * tr2;
	float tr4 = Square(tr2);

	float inertia_y = (MOSS_PI / 10.0f) * density * (t - b) * (br4 + tr * br3 + tr2 * br2 + tr3 * br + tr4);
	float inertia_x_delta = (MOSS_PI / 30.0f) * density * ((t3 + 2 * b * t2 + 3 * b2 * t - 6 * b3) * br2 + (3 * t3 + b * t2 - b2 * t - 3 * b3) * tr * br + (6 * t3 - 3 * b * t2 - 2 * b2 * t - b3) * tr2);
	float inertia_x = inertia_x_delta + inertia_y / 2;
	float inertia_z = inertia_x;
	p.mInertia = Mat44::sScale(Vec3(inertia_x, inertia_y, inertia_z));
	return p;
}

Vec3 TaperedCylinderShape::GetSurfaceNormal(const SubShapeID &inSubShapeID, Vec3Arg inLocalSurfacePosition) const
{
	MOSS_ASSERT(inSubShapeID.IsEmpty(), "Invalid subshape ID");

	constexpr float cEpsilon = 1.0e-5f;

	if (inLocalSurfacePosition.GetY() > mTop - cEpsilon)
		return Vec3(0, 1, 0);
	else if (inLocalSurfacePosition.GetY() < mBottom + cEpsilon)
		return Vec3(0, -1, 0);
	else
		return sCalculateSideNormal(sCalculateSideNormalXZ(inLocalSurfacePosition), mTop, mBottom, mTopRadius, mBottomRadius);
}

AABox TaperedCylinderShape::GetLocalBounds() const
{
	float max_radius = max(mTopRadius, mBottomRadius);
	return AABox(Vec3(-max_radius, mBottom, -max_radius), Vec3(max_radius, mTop, max_radius));
}

void TaperedCylinderShape::CollidePoint(Vec3Arg inPoint, const SubShapeIDCreator &inSubShapeIDCreator, CollidePointCollector &ioCollector, const ShapeFilter &inShapeFilter) const
{
	// Test shape filter
	if (!inShapeFilter.ShouldCollide(this, inSubShapeIDCreator.GetID()))
		return;

	// Check if the point is in the tapered cylinder
	if (inPoint.GetY() >= mBottom && inPoint.GetY() <= mTop // Within height
		&& Square(inPoint.GetX()) + Square(inPoint.GetZ()) <= Square(mBottomRadius + (inPoint.GetY() - mBottom) * (mTopRadius - mBottomRadius) / (mTop - mBottom))) // Within the radius
		ioCollector.AddHit({ TransformedShape::sGetBodyID(ioCollector.GetContext()), inSubShapeIDCreator.GetID() });
}

void TaperedCylinderShape::CollideSoftBodyVertices(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const CollideSoftBodyVertexIterator &inVertices, uint inNumVertices, int inCollidingShapeIndex) const
{
	MOSS_ASSERT(IsValidScale(inScale));

	Mat44 inverse_transform = inCenterOfMassTransform.InversedRotationTranslation();

	// Get scaled tapered cylinder
	float top, bottom, top_radius, bottom_radius, convex_radius;
	GetScaled(inScale, top, bottom, top_radius, bottom_radius, convex_radius);
	Vec3 top_3d(0, top, 0);
	Vec3 bottom_3d(0, bottom, 0);

	for (CollideSoftBodyVertexIterator v = inVertices, sbv_end = inVertices + inNumVertices; v != sbv_end; ++v)
		if (v.GetInvMass() > 0.0f)
		{
			Vec3 local_pos = inverse_transform * v.GetPosition();

			// Calculate penetration into side surface
			Vec3 normal_xz = sCalculateSideNormalXZ(local_pos);
			Vec3 side_normal = sCalculateSideNormal(normal_xz, top, bottom, top_radius, bottom_radius);
			Vec3 side_support_top = normal_xz * top_radius + top_3d;
			float side_penetration = (side_support_top - local_pos).Dot(side_normal);

			// Calculate penetration into top and bottom plane
			float top_penetration = top - local_pos.GetY();
			float bottom_penetration = local_pos.GetY() - bottom;
			float min_top_bottom_penetration = min(top_penetration, bottom_penetration);

			Vec3 point, normal;
			if (side_penetration < 0.0f || min_top_bottom_penetration < 0.0f)
			{
				// We're outside the cylinder
				// Calculate the closest point on the line segment from bottom to top support point:
				// closest_point = bottom + fraction * (top - bottom) / |top - bottom|^2
				Vec3 side_support_bottom = normal_xz * bottom_radius + bottom_3d;
				Vec3 bottom_to_top = side_support_top - side_support_bottom;
				float fraction = (local_pos - side_support_bottom).Dot(bottom_to_top);

				// Calculate the distance to the axis of the cylinder
				float distance_to_axis = normal_xz.Dot(local_pos);
				bool inside_top_radius = distance_to_axis <= top_radius;
				bool inside_bottom_radius = distance_to_axis <= bottom_radius;

				/*
					Regions of tapered cylinder (side view):

						_  B |       |
						 --_ |   A   |
							 t-------+
					   C    /         \
						   /  tapered  \
					_     /  cylinder   \
					 --_ /               \
						b-----------------+
					 D  |        E        |
						|                 |

					t = side_support_top, b = side_support_bottom
					Lines between B and C and C and D are at a 90 degree angle to the line between t and b
				*/
				if (fraction >= bottom_to_top.LengthSq() // Region B: Above the line segment
					&& !inside_top_radius) // Outside the top radius
				{
					// Top support point is closest
					point = side_support_top;
					normal = (local_pos - point).NormalizedOr(Vec3::sAxisY());
				}
				else if (fraction < 0.0f // Region D: Below the line segment
					&& !inside_bottom_radius) // Outside the bottom radius
				{
					// Bottom support point is closest
					point = side_support_bottom;
					normal = (local_pos - point).NormalizedOr(Vec3::sAxisY());
				}
				else if (top_penetration < 0.0f // Region A: Above the top plane
					&& inside_top_radius) // Inside the top radius
				{
					// Top plane is closest
					point = top_3d;
					normal = Vec3(0, 1, 0);
				}
				else if (bottom_penetration < 0.0f // Region E: Below the bottom plane
					&& inside_bottom_radius) // Inside the bottom radius
				{
					// Bottom plane is closest
					point = bottom_3d;
					normal = Vec3(0, -1, 0);
				}
				else // Region C
				{
					// Side surface is closest
					point = side_support_top;
					normal = side_normal;
				}
			}
			else if (side_penetration < min_top_bottom_penetration)
			{
				// Side surface is closest
				point = side_support_top;
				normal = side_normal;
			}
			else if (top_penetration < bottom_penetration)
			{
				// Top plane is closest
				point = top_3d;
				normal = Vec3(0, 1, 0);
			}
			else
			{
				// Bottom plane is closest
				point = bottom_3d;
				normal = Vec3(0, -1, 0);
			}

			// Calculate penetration
			Plane plane = Plane::sFromPointAndNormal(point, normal);
			float penetration = -plane.SignedDistance(local_pos);
			if (v.UpdatePenetration(penetration))
				v.SetCollision(plane.GetTransformed(inCenterOfMassTransform), inCollidingShapeIndex);
		}
}

class TaperedCylinderShape::TCSGetTrianglesContext
{
public:
	explicit	TCSGetTrianglesContext(Mat44Arg inTransform) : mTransform(inTransform) { }

	Mat44		mTransform;
	uint		mProcessed = 0; // Which elements we processed, bit 0 = top, bit 1 = bottom, bit 2 = side
};

void TaperedCylinderShape::GetTrianglesStart(GetTrianglesContext &ioContext, const AABox &inBox, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale) const
{
	static_assert(sizeof(TCSGetTrianglesContext) <= sizeof(GetTrianglesContext), "GetTrianglesContext too small");
	MOSS_ASSERT(IsAligned(&ioContext, alignof(TCSGetTrianglesContext)));

	// Make sure the scale is not inside out
	Vec3 scale = ScaleHelpers::IsInsideOut(inScale)? Vec3(-1, 1, 1) * inScale : inScale;

	// Mark top and bottom processed if their radius is too small
	TCSGetTrianglesContext *context = new (&ioContext) TCSGetTrianglesContext(Mat44::sRotationTranslation(inRotation, inPositionCOM) * Mat44::sScale(scale));
	constexpr float cMinRadius = 1.0e-3f;
	if (mTopRadius < cMinRadius)
		context->mProcessed |= 0b001;
	if (mBottomRadius < cMinRadius)
		context->mProcessed |= 0b010;
}

int TaperedCylinderShape::GetTrianglesNext(GetTrianglesContext &ioContext, int inMaxTrianglesRequested, Float3 *outTriangleVertices, const PhysicsMaterial **outMaterials) const
{
	constexpr int cNumVertices = int(std::size(cTaperedCylinderFace));

	static_assert(cGetTrianglesMinTrianglesRequested >= 2 * cNumVertices);
	MOSS_ASSERT(inMaxTrianglesRequested >= cGetTrianglesMinTrianglesRequested);

	TCSGetTrianglesContext &context = (TCSGetTrianglesContext &)ioContext;

	int total_num_triangles = 0;

	// Top cap
	Vec3 top_3d(0, mTop, 0);
	if ((context.mProcessed & 0b001) == 0)
	{
		Vec3 v0 = context.mTransform * (top_3d + mTopRadius * cTaperedCylinderFace[0]);
		Vec3 v1 = context.mTransform * (top_3d + mTopRadius * cTaperedCylinderFace[1]);

		for (const Vec3 *v = cTaperedCylinderFace + 2, *v_end = cTaperedCylinderFace + cNumVertices; v < v_end; ++v)
		{
			Vec3 v2 = context.mTransform * (top_3d + mTopRadius * *v);

			v0.StoreFloat3(outTriangleVertices++);
			v1.StoreFloat3(outTriangleVertices++);
			v2.StoreFloat3(outTriangleVertices++);

			v1 = v2;
		}

		total_num_triangles = cNumVertices - 2;
		context.mProcessed |= 0b001;
	}

	// Bottom cap
	Vec3 bottom_3d(0, mBottom, 0);
	if ((context.mProcessed & 0b010) == 0
		&& total_num_triangles + cNumVertices - 2 < inMaxTrianglesRequested)
	{
		Vec3 v0 = context.mTransform * (bottom_3d + mBottomRadius * cTaperedCylinderFace[0]);
		Vec3 v1 = context.mTransform * (bottom_3d + mBottomRadius * cTaperedCylinderFace[1]);

		for (const Vec3 *v = cTaperedCylinderFace + 2, *v_end = cTaperedCylinderFace + cNumVertices; v < v_end; ++v)
		{
			Vec3 v2 = context.mTransform * (bottom_3d + mBottomRadius * *v);

			v0.StoreFloat3(outTriangleVertices++);
			v2.StoreFloat3(outTriangleVertices++);
			v1.StoreFloat3(outTriangleVertices++);

			v1 = v2;
		}

		total_num_triangles += cNumVertices - 2;
		context.mProcessed |= 0b010;
	}

	// Side
	if ((context.mProcessed & 0b100) == 0
		&& total_num_triangles + 2 * cNumVertices < inMaxTrianglesRequested)
	{
		Vec3 v0t = context.mTransform * (top_3d + mTopRadius * cTaperedCylinderFace[cNumVertices - 1]);
		Vec3 v0b = context.mTransform * (bottom_3d + mBottomRadius * cTaperedCylinderFace[cNumVertices - 1]);

		for (const Vec3 *v = cTaperedCylinderFace, *v_end = cTaperedCylinderFace + cNumVertices; v < v_end; ++v)
		{
			Vec3 v1t = context.mTransform * (top_3d + mTopRadius * *v);
			v0t.StoreFloat3(outTriangleVertices++);
			v0b.StoreFloat3(outTriangleVertices++);
			v1t.StoreFloat3(outTriangleVertices++);

			Vec3 v1b = context.mTransform * (bottom_3d + mBottomRadius * *v);
			v1t.StoreFloat3(outTriangleVertices++);
			v0b.StoreFloat3(outTriangleVertices++);
			v1b.StoreFloat3(outTriangleVertices++);

			v0t = v1t;
			v0b = v1b;
		}

		total_num_triangles += 2 * cNumVertices;
		context.mProcessed |= 0b100;
	}

	// Store materials
	if (outMaterials != nullptr)
	{
		const PhysicsMaterial *material = GetMaterial();
		for (const PhysicsMaterial **m = outMaterials, **m_end = outMaterials + total_num_triangles; m < m_end; ++m)
			*m = material;
	}

	return total_num_triangles;
}

#ifndef MOSS_DEBUG_RENDERER
void TaperedCylinderShape::Draw(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, Vec3Arg inScale, ColorArg inColor, bool inUseMaterialColors, bool inDrawWireframe) const
{
	// Preserve flip along y axis but make sure we're not inside out
	Vec3 scale = ScaleHelpers::IsInsideOut(inScale)? Vec3(-1, 1, 1) * inScale : inScale;
	RMat44 world_transform = inCenterOfMassTransform * Mat44::sScale(scale);

	DebugRenderer::EDrawMode draw_mode = inDrawWireframe? DebugRenderer::EDrawMode::Wireframe : DebugRenderer::EDrawMode::Solid;
	inRenderer->DrawTaperedCylinder(world_transform, mTop, mBottom, mTopRadius, mBottomRadius, inUseMaterialColors? GetMaterial()->GetDebugColor() : inColor, DebugRenderer::ECastShadow::On, draw_mode);
}
#endif // MOSS_DEBUG_RENDERER

void TaperedCylinderShape::SaveBinaryState(StreamOut &inStream) const
{
	ConvexShape::SaveBinaryState(inStream);

	inStream.Write(mTop);
	inStream.Write(mBottom);
	inStream.Write(mTopRadius);
	inStream.Write(mBottomRadius);
	inStream.Write(mConvexRadius);
}

void TaperedCylinderShape::RestoreBinaryState(StreamIn &inStream)
{
	ConvexShape::RestoreBinaryState(inStream);

	inStream.Read(mTop);
	inStream.Read(mBottom);
	inStream.Read(mTopRadius);
	inStream.Read(mBottomRadius);
	inStream.Read(mConvexRadius);
}

float TaperedCylinderShape::GetVolume() const
{
	// Volume of a tapered cylinder is: integrate(%pi*(b+x*(t-b)/h)^2,x,0,h) where t is the top radius, b is the bottom radius and h is the height
	return (MOSS_PI / 3.0f) * (mTop - mBottom) * (Square(mTopRadius) + mTopRadius * mBottomRadius + Square(mBottomRadius));
}

bool TaperedCylinderShape::IsValidScale(Vec3Arg inScale) const
{
	return ConvexShape::IsValidScale(inScale) && ScaleHelpers::IsUniformScaleXZ(inScale.Abs());
}

Vec3 TaperedCylinderShape::MakeScaleValid(Vec3Arg inScale) const
{
	Vec3 scale = ScaleHelpers::MakeNonZeroScale(inScale);

	return scale.GetSign() * ScaleHelpers::MakeUniformScaleXZ(scale.Abs());
}

void TaperedCylinderShape::sRegister()
{
	ShapeFunctions &f = ShapeFunctions::sGet(EShapeSubType::TaperedCylinder);
	f.mConstruct = []() -> Shape * { return new TaperedCylinderShape; };
	f.mColor = Color::sGreen;
}


/*													*/

ShapeSettings::ShapeResult StaticCompoundShapeSettings::Create(TempAllocator &inTempAllocator) const
{
	if (mCachedResult.IsEmpty())
	{
		if (mSubShapes.size() == 0)
		{
			// It's an error to create a compound with no subshapes (the compound cannot encode this)
			mCachedResult.SetError("Compound needs a sub shape!");
		}
		else if (mSubShapes.size() == 1)
		{
			// If there's only 1 part we don't need a StaticCompoundShape
			const SubShapeSettings &s = mSubShapes[0];
			if (s.mPosition == Vec3::sZero()
				&& s.mRotation == Quat::sIdentity())
			{
				// No rotation or translation, we can use the shape directly
				if (s.mShapePtr != nullptr)
					mCachedResult.Set(const_cast<Shape *>(s.mShapePtr.GetPtr()));
				else if (s.mShape != nullptr)
					mCachedResult = s.mShape->Create();
				else
					mCachedResult.SetError("Sub shape is null!");
			}
			else
			{
				// We can use a RotatedTranslatedShape instead
				RotatedTranslatedShapeSettings settings;
				settings.mPosition = s.mPosition;
				settings.mRotation = s.mRotation;
				settings.mInnerShape = s.mShape;
				settings.mInnerShapePtr = s.mShapePtr;
				Ref<Shape> shape = new RotatedTranslatedShape(settings, mCachedResult);
			}
		}
		else
		{
			// Build a regular compound shape
			Ref<Shape> shape = new StaticCompoundShape(*this, inTempAllocator, mCachedResult);
		}
	}
	return mCachedResult;
}

ShapeSettings::ShapeResult StaticCompoundShapeSettings::Create() const
{
	TempAllocatorMalloc allocator;
	return Create(allocator);
}

void StaticCompoundShape::Node::SetChildInvalid(uint inIndex)
{
	// Make this an invalid node
	mNodeProperties[inIndex] = INVALID_NODE;

	// Make bounding box invalid
	mBoundsMinX[inIndex] = HALF_FLT_MAX;
	mBoundsMinY[inIndex] = HALF_FLT_MAX;
	mBoundsMinZ[inIndex] = HALF_FLT_MAX;
	mBoundsMaxX[inIndex] = HALF_FLT_MAX;
	mBoundsMaxY[inIndex] = HALF_FLT_MAX;
	mBoundsMaxZ[inIndex] = HALF_FLT_MAX;
}

void StaticCompoundShape::Node::SetChildBounds(uint inIndex, const AABox &inBounds)
{
	mBoundsMinX[inIndex] = HalfFloatConversion::FromFloat<HalfFloatConversion::ROUND_TO_NEG_INF>(inBounds.mMin.GetX());
	mBoundsMinY[inIndex] = HalfFloatConversion::FromFloat<HalfFloatConversion::ROUND_TO_NEG_INF>(inBounds.mMin.GetY());
	mBoundsMinZ[inIndex] = HalfFloatConversion::FromFloat<HalfFloatConversion::ROUND_TO_NEG_INF>(inBounds.mMin.GetZ());
	mBoundsMaxX[inIndex] = HalfFloatConversion::FromFloat<HalfFloatConversion::ROUND_TO_POS_INF>(inBounds.mMax.GetX());
	mBoundsMaxY[inIndex] = HalfFloatConversion::FromFloat<HalfFloatConversion::ROUND_TO_POS_INF>(inBounds.mMax.GetY());
	mBoundsMaxZ[inIndex] = HalfFloatConversion::FromFloat<HalfFloatConversion::ROUND_TO_POS_INF>(inBounds.mMax.GetZ());
}

void StaticCompoundShape::sPartition(uint *ioBodyIdx, AABox *ioBounds, int inNumber, int &outMidPoint)
{
	// Handle trivial case
	if (inNumber <= 4)
	{
		outMidPoint = inNumber / 2;
		return;
	}

	// Calculate bounding box of box centers
	Vec3 center_min = Vec3::sReplicate(FLT_MAX);
	Vec3 center_max = Vec3::sReplicate(-FLT_MAX);
	for (const AABox *b = ioBounds, *b_end = ioBounds + inNumber; b < b_end; ++b)
	{
		Vec3 center = b->GetCenter();
		center_min = Vec3::sMin(center_min, center);
		center_max = Vec3::sMax(center_max, center);
	}

	// Calculate split plane
	int dimension = (center_max - center_min).GetHighestComponentIndex();
	float split = 0.5f * (center_min + center_max)[dimension];

	// Divide bodies
	int start = 0, end = inNumber;
	while (start < end)
	{
		// Search for first element that is on the right hand side of the split plane
		while (start < end && ioBounds[start].GetCenter()[dimension] < split)
			++start;

		// Search for the first element that is on the left hand side of the split plane
		while (start < end && ioBounds[end - 1].GetCenter()[dimension] >= split)
			--end;

		if (start < end)
		{
			// Swap the two elements
			std::swap(ioBodyIdx[start], ioBodyIdx[end - 1]);
			std::swap(ioBounds[start], ioBounds[end - 1]);
			++start;
			--end;
		}
	}
	MOSS_ASSERT(start == end);

	if (start > 0 && start < inNumber)
	{
		// Success!
		outMidPoint = start;
	}
	else
	{
		// Failed to divide bodies
		outMidPoint = inNumber / 2;
	}
}

void StaticCompoundShape::sPartition4(uint *ioBodyIdx, AABox *ioBounds, int inBegin, int inEnd, int *outSplit)
{
	uint *body_idx = ioBodyIdx + inBegin;
	AABox *node_bounds = ioBounds + inBegin;
	int number = inEnd - inBegin;

	// Partition entire range
	sPartition(body_idx, node_bounds, number, outSplit[2]);

	// Partition lower half
	sPartition(body_idx, node_bounds, outSplit[2], outSplit[1]);

	// Partition upper half
	sPartition(body_idx + outSplit[2], node_bounds + outSplit[2], number - outSplit[2], outSplit[3]);

	// Convert to proper range
	outSplit[0] = inBegin;
	outSplit[1] += inBegin;
	outSplit[2] += inBegin;
	outSplit[3] += outSplit[2];
	outSplit[4] = inEnd;
}

StaticCompoundShape::StaticCompoundShape(const StaticCompoundShapeSettings &inSettings, TempAllocator &inTempAllocator, ShapeResult &outResult) :
	CompoundShape(EShapeSubType::StaticCompound, inSettings, outResult)
{
	// Check that there's at least 1 shape
	uint num_subshapes = (uint)inSettings.mSubShapes.size();
	if (num_subshapes < 2)
	{
		outResult.SetError("Compound needs at least 2 sub shapes, otherwise you should use a RotatedTranslatedShape!");
		return;
	}

	// Keep track of total mass to calculate center of mass
	float mass = 0.0f;

	mSubShapes.resize(num_subshapes);
	for (uint i = 0; i < num_subshapes; ++i)
	{
		const CompoundShapeSettings::SubShapeSettings &shape = inSettings.mSubShapes[i];

		// Start constructing the runtime sub shape
		SubShape &out_shape = mSubShapes[i];
		if (!out_shape.FromSettings(shape, outResult))
			return;

		// Calculate mass properties of child
		MassProperties child = out_shape.mShape->GetMassProperties();

		// Accumulate center of mass
		mass += child.mMass;
		mCenterOfMass += out_shape.GetPositionCOM() * child.mMass;
	}

	if (mass > 0.0f)
		mCenterOfMass /= mass;

	// Cache the inner radius as it can take a while to recursively iterate over all sub shapes
	CalculateInnerRadius();

	// Temporary storage for the bounding boxes of all shapes
	uint bounds_size = num_subshapes * sizeof(AABox);
	AABox *bounds = (AABox *)inTempAllocator.Allocate(bounds_size);
	MOSS_SCOPE_EXIT([&inTempAllocator, bounds, bounds_size]{ inTempAllocator.Free(bounds, bounds_size); });

	// Temporary storage for body indexes (we're shuffling them)
	uint body_idx_size = num_subshapes * sizeof(uint);
	uint *body_idx = (uint *)inTempAllocator.Allocate(body_idx_size);
	MOSS_SCOPE_EXIT([&inTempAllocator, body_idx, body_idx_size]{ inTempAllocator.Free(body_idx, body_idx_size); });

	// Shift all shapes so that the center of mass is now at the origin and calculate bounds
	for (uint i = 0; i < num_subshapes; ++i)
	{
		SubShape &shape = mSubShapes[i];

		// Shift the shape so it's centered around our center of mass
		shape.SetPositionCOM(shape.GetPositionCOM() - mCenterOfMass);

		// Transform the shape's bounds into our local space
		Mat44 transform = Mat44::sRotationTranslation(shape.GetRotation(), shape.GetPositionCOM());
		AABox shape_bounds = shape.mShape->GetWorldSpaceBounds(transform, Vec3::sOne());

		// Store bounds and body index for tree construction
		bounds[i] = shape_bounds;
		body_idx[i] = i;

		// Update our local bounds
		mLocalBounds.Encapsulate(shape_bounds);
	}

	// The algorithm is a recursive tree build, but to avoid the call overhead we keep track of a stack here
	struct StackEntry
	{
		uint32			mNodeIdx;					// Node index of node that is generated
		int				mChildIdx;					// Index of child that we're currently processing
		int				mSplit[5];					// Indices where the node ID's have been split to form 4 partitions
		AABox			mBounds;					// Bounding box of this node
	};
	uint stack_size = num_subshapes * sizeof(StackEntry);
	StackEntry *stack = (StackEntry *)inTempAllocator.Allocate(stack_size);
	MOSS_SCOPE_EXIT([&inTempAllocator, stack, stack_size]{ inTempAllocator.Free(stack, stack_size); });
	int top = 0;

	// Reserve enough space so that every sub shape gets its own leaf node
	uint next_node_idx = 0;
	mNodes.resize(num_subshapes + (num_subshapes + 2) / 3); // = Sum(num_subshapes * 4^-i) with i = [0, Inf].

	// Create root node
	stack[0].mNodeIdx = next_node_idx++;
	stack[0].mChildIdx = -1;
	stack[0].mBounds = AABox();
	sPartition4(body_idx, bounds, 0, num_subshapes, stack[0].mSplit);

	for (;;)
	{
		StackEntry &cur_stack = stack[top];

		// Next child
		cur_stack.mChildIdx++;

		// Check if all children processed
		if (cur_stack.mChildIdx >= 4)
		{
			// Terminate if there's nothing left to pop
			if (top <= 0)
				break;

			// Add our bounds to our parents bounds
			StackEntry &prev_stack = stack[top - 1];
			prev_stack.mBounds.Encapsulate(cur_stack.mBounds);

			// Store this node's properties in the parent node
			Node &parent_node = mNodes[prev_stack.mNodeIdx];
			parent_node.mNodeProperties[prev_stack.mChildIdx] = cur_stack.mNodeIdx;
			parent_node.SetChildBounds(prev_stack.mChildIdx, cur_stack.mBounds);

			// Pop entry from stack
			--top;
		}
		else
		{
			// Get low and high index to bodies to process
			int low = cur_stack.mSplit[cur_stack.mChildIdx];
			int high = cur_stack.mSplit[cur_stack.mChildIdx + 1];
			int num_bodies = high - low;

			if (num_bodies == 0)
			{
				// Mark invalid
				Node &node = mNodes[cur_stack.mNodeIdx];
				node.SetChildInvalid(cur_stack.mChildIdx);
			}
			else if (num_bodies == 1)
			{
				// Get body info
				uint child_node_idx = body_idx[low];
				const AABox &child_bounds = bounds[low];

				// Update node
				Node &node = mNodes[cur_stack.mNodeIdx];
				node.mNodeProperties[cur_stack.mChildIdx] = child_node_idx | IS_SUBSHAPE;
				node.SetChildBounds(cur_stack.mChildIdx, child_bounds);

				// Encapsulate bounding box in parent
				cur_stack.mBounds.Encapsulate(child_bounds);
			}
			else
			{
				// Allocate new node
				StackEntry &new_stack = stack[++top];
				MOSS_ASSERT(top < (int)num_subshapes);
				new_stack.mNodeIdx = next_node_idx++;
				new_stack.mChildIdx = -1;
				new_stack.mBounds = AABox();
				sPartition4(body_idx, bounds, low, high, new_stack.mSplit);
			}
		}
	}

	// Resize nodes to actual size
	MOSS_ASSERT(next_node_idx <= mNodes.size());
	mNodes.resize(next_node_idx);
	mNodes.shrink_to_fit();

	// Check if we ran out of bits for addressing a node
	if (next_node_idx > IS_SUBSHAPE)
	{
		outResult.SetError("Compound hierarchy has too many nodes");
		return;
	}

	// Check if we're not exceeding the amount of sub shape id bits
	if (GetSubShapeIDBitsRecursive() > SubShapeID::MaxBits)
	{
		outResult.SetError("Compound hierarchy is too deep and exceeds the amount of available sub shape ID bits");
		return;
	}

	outResult.Set(this);
}

template <class Visitor>
inline void StaticCompoundShape::WalkTree(Visitor &ioVisitor) const
{
	uint32 node_stack[cStackSize];
	node_stack[0] = 0;
	int top = 0;
	do
	{
		// Test if the node is valid, the node should rarely be invalid but it is possible when testing
		// a really large box against the tree that the invalid nodes will intersect with the box
		uint32 node_properties = node_stack[top];
		if (node_properties != INVALID_NODE)
		{
			// Test if node contains triangles
			bool is_node = (node_properties & IS_SUBSHAPE) == 0;
			if (is_node)
			{
				const Node &node = mNodes[node_properties];

				// Unpack bounds
				UVec4 bounds_minxy = UVec4::sLoadInt4(reinterpret_cast<const uint32 *>(&node.mBoundsMinX[0]));
				Vec4 bounds_minx = HalfFloatConversion::ToFloat(bounds_minxy);
				Vec4 bounds_miny = HalfFloatConversion::ToFloat(bounds_minxy.Swizzle<SWIZZLE_Z, SWIZZLE_W, SWIZZLE_UNUSED, SWIZZLE_UNUSED>());

				UVec4 bounds_minzmaxx = UVec4::sLoadInt4(reinterpret_cast<const uint32 *>(&node.mBoundsMinZ[0]));
				Vec4 bounds_minz = HalfFloatConversion::ToFloat(bounds_minzmaxx);
				Vec4 bounds_maxx = HalfFloatConversion::ToFloat(bounds_minzmaxx.Swizzle<SWIZZLE_Z, SWIZZLE_W, SWIZZLE_UNUSED, SWIZZLE_UNUSED>());

				UVec4 bounds_maxyz = UVec4::sLoadInt4(reinterpret_cast<const uint32 *>(&node.mBoundsMaxY[0]));
				Vec4 bounds_maxy = HalfFloatConversion::ToFloat(bounds_maxyz);
				Vec4 bounds_maxz = HalfFloatConversion::ToFloat(bounds_maxyz.Swizzle<SWIZZLE_Z, SWIZZLE_W, SWIZZLE_UNUSED, SWIZZLE_UNUSED>());

				// Load properties for 4 children
				UVec4 properties = UVec4::sLoadInt4(&node.mNodeProperties[0]);

				// Check which sub nodes to visit
				int num_results = ioVisitor.VisitNodes(bounds_minx, bounds_miny, bounds_minz, bounds_maxx, bounds_maxy, bounds_maxz, properties, top);

				// Push them onto the stack
				MOSS_ASSERT(top + 4 < cStackSize);
				properties.StoreInt4(&node_stack[top]);
				top += num_results;
			}
			else
			{
				// Points to a sub shape
				uint32 sub_shape_idx = node_properties ^ IS_SUBSHAPE;
				const SubShape &sub_shape = mSubShapes[sub_shape_idx];

				ioVisitor.VisitShape(sub_shape, sub_shape_idx);
			}

			// Check if we're done
			if (ioVisitor.ShouldAbort())
				break;
		}

		// Fetch next node until we find one that the visitor wants to see
		do
			--top;
		while (top >= 0 && !ioVisitor.ShouldVisitNode(top));
	}
	while (top >= 0);
}

bool StaticCompoundShape::CastRay(const RayCast &inRay, const SubShapeIDCreator &inSubShapeIDCreator, RayCastResult &ioHit) const
{
	MOSS_PROFILE_FUNCTION();

	struct Visitor : public CastRayVisitor
	{
		using CastRayVisitor::CastRayVisitor;

		MOSS_INLINE bool		ShouldVisitNode(int inStackTop) const
		{
			return mDistanceStack[inStackTop] < mHit.mFraction;
		}

		MOSS_INLINE int		VisitNodes(Vec4Arg inBoundsMinX, Vec4Arg inBoundsMinY, Vec4Arg inBoundsMinZ, Vec4Arg inBoundsMaxX, Vec4Arg inBoundsMaxY, Vec4Arg inBoundsMaxZ, UVec4 &ioProperties, int inStackTop)
		{
			// Test bounds of 4 children
			Vec4 distance = TestBounds(inBoundsMinX, inBoundsMinY, inBoundsMinZ, inBoundsMaxX, inBoundsMaxY, inBoundsMaxZ);

			// Sort so that highest values are first (we want to first process closer hits and we process stack top to bottom)
			return SortReverseAndStore(distance, mHit.mFraction, ioProperties, &mDistanceStack[inStackTop]);
		}

		float				mDistanceStack[cStackSize];
	};

	Visitor visitor(inRay, this, inSubShapeIDCreator, ioHit);
	WalkTree(visitor);
	return visitor.mReturnValue;
}

void StaticCompoundShape::CastRay(const RayCast &inRay, const RayCastSettings &inRayCastSettings, const SubShapeIDCreator &inSubShapeIDCreator, CastRayCollector &ioCollector, const ShapeFilter &inShapeFilter) const
{
	// Test shape filter
	if (!inShapeFilter.ShouldCollide(this, inSubShapeIDCreator.GetID()))
		return;

	MOSS_PROFILE_FUNCTION();

	struct Visitor : public CastRayVisitorCollector
	{
		using CastRayVisitorCollector::CastRayVisitorCollector;

		MOSS_INLINE bool		ShouldVisitNode(int inStackTop) const
		{
			return mDistanceStack[inStackTop] < mCollector.GetEarlyOutFraction();
		}

		MOSS_INLINE int		VisitNodes(Vec4Arg inBoundsMinX, Vec4Arg inBoundsMinY, Vec4Arg inBoundsMinZ, Vec4Arg inBoundsMaxX, Vec4Arg inBoundsMaxY, Vec4Arg inBoundsMaxZ, UVec4 &ioProperties, int inStackTop)
		{
			// Test bounds of 4 children
			Vec4 distance = TestBounds(inBoundsMinX, inBoundsMinY, inBoundsMinZ, inBoundsMaxX, inBoundsMaxY, inBoundsMaxZ);

			// Sort so that highest values are first (we want to first process closer hits and we process stack top to bottom)
			return SortReverseAndStore(distance, mCollector.GetEarlyOutFraction(), ioProperties, &mDistanceStack[inStackTop]);
		}

		float				mDistanceStack[cStackSize];
	};

	Visitor visitor(inRay, inRayCastSettings, this, inSubShapeIDCreator, ioCollector, inShapeFilter);
	WalkTree(visitor);
}

void StaticCompoundShape::CollidePoint(Vec3Arg inPoint, const SubShapeIDCreator &inSubShapeIDCreator, CollidePointCollector &ioCollector, const ShapeFilter &inShapeFilter) const
{
	MOSS_PROFILE_FUNCTION();

	struct Visitor : public CollidePointVisitor
	{
		using CollidePointVisitor::CollidePointVisitor;

		MOSS_INLINE bool		ShouldVisitNode([[maybe_unused]] int inStackTop) const
		{
			return true;
		}

		MOSS_INLINE int		VisitNodes(Vec4Arg inBoundsMinX, Vec4Arg inBoundsMinY, Vec4Arg inBoundsMinZ, Vec4Arg inBoundsMaxX, Vec4Arg inBoundsMaxY, Vec4Arg inBoundsMaxZ, UVec4 &ioProperties, [[maybe_unused]] int inStackTop) const
		{
			// Test if point overlaps with box
			UVec4 collides = TestBounds(inBoundsMinX, inBoundsMinY, inBoundsMinZ, inBoundsMaxX, inBoundsMaxY, inBoundsMaxZ);
			return CountAndSortTrues(collides, ioProperties);
		}
	};

	Visitor visitor(inPoint, this, inSubShapeIDCreator, ioCollector, inShapeFilter);
	WalkTree(visitor);
}

void StaticCompoundShape::sCastShapeVsCompound(const ShapeCast &inShapeCast, const ShapeCastSettings &inShapeCastSettings, const Shape *inShape, Vec3Arg inScale, const ShapeFilter &inShapeFilter, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, CastShapeCollector &ioCollector)
{
	MOSS_PROFILE_FUNCTION();

	struct Visitor : public CastShapeVisitor
	{
		using CastShapeVisitor::CastShapeVisitor;

		MOSS_INLINE bool		ShouldVisitNode(int inStackTop) const
		{
			return mDistanceStack[inStackTop] < mCollector.GetPositiveEarlyOutFraction();
		}

		MOSS_INLINE int		VisitNodes(Vec4Arg inBoundsMinX, Vec4Arg inBoundsMinY, Vec4Arg inBoundsMinZ, Vec4Arg inBoundsMaxX, Vec4Arg inBoundsMaxY, Vec4Arg inBoundsMaxZ, UVec4 &ioProperties, int inStackTop)
		{
			// Test bounds of 4 children
			Vec4 distance = TestBounds(inBoundsMinX, inBoundsMinY, inBoundsMinZ, inBoundsMaxX, inBoundsMaxY, inBoundsMaxZ);

			// Sort so that highest values are first (we want to first process closer hits and we process stack top to bottom)
			return SortReverseAndStore(distance, mCollector.GetPositiveEarlyOutFraction(), ioProperties, &mDistanceStack[inStackTop]);
		}

		float				mDistanceStack[cStackSize];
	};

	MOSS_ASSERT(inShape->GetSubType() == EShapeSubType::StaticCompound);
	const StaticCompoundShape *shape = static_cast<const StaticCompoundShape *>(inShape);

	Visitor visitor(inShapeCast, inShapeCastSettings, shape, inScale, inShapeFilter, inCenterOfMassTransform2, inSubShapeIDCreator1, inSubShapeIDCreator2, ioCollector);
	shape->WalkTree(visitor);
}

void StaticCompoundShape::CollectTransformedShapes(const AABox &inBox, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale, const SubShapeIDCreator &inSubShapeIDCreator, TransformedShapeCollector &ioCollector, const ShapeFilter &inShapeFilter) const
{
	MOSS_PROFILE_FUNCTION();

	// Test shape filter
	if (!inShapeFilter.ShouldCollide(this, inSubShapeIDCreator.GetID()))
		return;

	struct Visitor : public CollectTransformedShapesVisitor
	{
		using CollectTransformedShapesVisitor::CollectTransformedShapesVisitor;

		MOSS_INLINE bool		ShouldVisitNode([[maybe_unused]] int inStackTop) const
		{
			return true;
		}

		MOSS_INLINE int		VisitNodes(Vec4Arg inBoundsMinX, Vec4Arg inBoundsMinY, Vec4Arg inBoundsMinZ, Vec4Arg inBoundsMaxX, Vec4Arg inBoundsMaxY, Vec4Arg inBoundsMaxZ, UVec4 &ioProperties, [[maybe_unused]] int inStackTop) const
		{
			// Test which nodes collide
			UVec4 collides = TestBounds(inBoundsMinX, inBoundsMinY, inBoundsMinZ, inBoundsMaxX, inBoundsMaxY, inBoundsMaxZ);
			return CountAndSortTrues(collides, ioProperties);
		}
	};

	Visitor visitor(inBox, this, inPositionCOM, inRotation, inScale, inSubShapeIDCreator, ioCollector, inShapeFilter);
	WalkTree(visitor);
}

int StaticCompoundShape::GetIntersectingSubShapes(const AABox &inBox, uint *outSubShapeIndices, int inMaxSubShapeIndices) const
{
	MOSS_PROFILE_FUNCTION();

	GetIntersectingSubShapesVisitorSC<AABox> visitor(inBox, outSubShapeIndices, inMaxSubShapeIndices);
	WalkTree(visitor);
	return visitor.GetNumResults();
}

int StaticCompoundShape::GetIntersectingSubShapes(const OrientedBox &inBox, uint *outSubShapeIndices, int inMaxSubShapeIndices) const
{
	MOSS_PROFILE_FUNCTION();

	GetIntersectingSubShapesVisitorSC<OrientedBox> visitor(inBox, outSubShapeIndices, inMaxSubShapeIndices);
	WalkTree(visitor);
	return visitor.GetNumResults();
}

void StaticCompoundShape::sCollideCompoundVsShape(const Shape *inShape1, const Shape *inShape2, Vec3Arg inScale1, Vec3Arg inScale2, Mat44Arg inCenterOfMassTransform1, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, const CollideShapeSettings &inCollideShapeSettings, CollideShapeCollector &ioCollector, const ShapeFilter &inShapeFilter)
{
	MOSS_PROFILE_FUNCTION();

	MOSS_ASSERT(inShape1->GetSubType() == EShapeSubType::StaticCompound);
	const StaticCompoundShape *shape1 = static_cast<const StaticCompoundShape *>(inShape1);

	struct Visitor : public CollideCompoundVsShapeVisitor
	{
		using CollideCompoundVsShapeVisitor::CollideCompoundVsShapeVisitor;

		MOSS_INLINE bool		ShouldVisitNode([[maybe_unused]] int inStackTop) const
		{
			return true;
		}

		MOSS_INLINE int		VisitNodes(Vec4Arg inBoundsMinX, Vec4Arg inBoundsMinY, Vec4Arg inBoundsMinZ, Vec4Arg inBoundsMaxX, Vec4Arg inBoundsMaxY, Vec4Arg inBoundsMaxZ, UVec4 &ioProperties, [[maybe_unused]] int inStackTop) const
		{
			// Test which nodes collide
			UVec4 collides = TestBounds(inBoundsMinX, inBoundsMinY, inBoundsMinZ, inBoundsMaxX, inBoundsMaxY, inBoundsMaxZ);
			return CountAndSortTrues(collides, ioProperties);
		}
	};

	Visitor visitor(shape1, inShape2, inScale1, inScale2, inCenterOfMassTransform1, inCenterOfMassTransform2, inSubShapeIDCreator1, inSubShapeIDCreator2, inCollideShapeSettings, ioCollector, inShapeFilter);
	shape1->WalkTree(visitor);
}

void StaticCompoundShape::sCollideShapeVsCompound(const Shape *inShape1, const Shape *inShape2, Vec3Arg inScale1, Vec3Arg inScale2, Mat44Arg inCenterOfMassTransform1, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, const CollideShapeSettings &inCollideShapeSettings, CollideShapeCollector &ioCollector, const ShapeFilter &inShapeFilter)
{
	MOSS_PROFILE_FUNCTION();

	struct Visitor : public CollideShapeVsCompoundVisitor
	{
		using CollideShapeVsCompoundVisitor::CollideShapeVsCompoundVisitor;

		MOSS_INLINE bool		ShouldVisitNode([[maybe_unused]] int inStackTop) const
		{
			return true;
		}

		MOSS_INLINE int		VisitNodes(Vec4Arg inBoundsMinX, Vec4Arg inBoundsMinY, Vec4Arg inBoundsMinZ, Vec4Arg inBoundsMaxX, Vec4Arg inBoundsMaxY, Vec4Arg inBoundsMaxZ, UVec4 &ioProperties, [[maybe_unused]] int inStackTop) const
		{
			// Test which nodes collide
			UVec4 collides = TestBounds(inBoundsMinX, inBoundsMinY, inBoundsMinZ, inBoundsMaxX, inBoundsMaxY, inBoundsMaxZ);
			return CountAndSortTrues(collides, ioProperties);
		}
	};

	MOSS_ASSERT(inShape2->GetSubType() == EShapeSubType::StaticCompound);
	const StaticCompoundShape *shape2 = static_cast<const StaticCompoundShape *>(inShape2);

	Visitor visitor(inShape1, shape2, inScale1, inScale2, inCenterOfMassTransform1, inCenterOfMassTransform2, inSubShapeIDCreator1, inSubShapeIDCreator2, inCollideShapeSettings, ioCollector, inShapeFilter);
	shape2->WalkTree(visitor);
}

void StaticCompoundShape::SaveBinaryState(StreamOut &inStream) const
{
	CompoundShape::SaveBinaryState(inStream);

	inStream.Write(mNodes);
}

void StaticCompoundShape::RestoreBinaryState(StreamIn &inStream)
{
	CompoundShape::RestoreBinaryState(inStream);

	inStream.Read(mNodes);
}

void StaticCompoundShape::sRegister()
{
	ShapeFunctions &f = ShapeFunctions::sGet(EShapeSubType::StaticCompound);
	f.mConstruct = []() -> Shape * { return new StaticCompoundShape; };
	f.mColor = Color::sOrange;

	for (EShapeSubType s : sAllSubShapeTypes)
	{
		CollisionDispatch::sRegisterCollideShape(EShapeSubType::StaticCompound, s, sCollideCompoundVsShape);
		CollisionDispatch::sRegisterCollideShape(s, EShapeSubType::StaticCompound, sCollideShapeVsCompound);
		CollisionDispatch::sRegisterCastShape(s, EShapeSubType::StaticCompound, sCastShapeVsCompound);
	}
}


/*													*/

const Shape *Shape::GetLeafShape([[maybe_unused]] const SubShapeID &inSubShapeID, SubShapeID &outRemainder) const
{
	outRemainder = inSubShapeID;
	return this;
}

TransformedShape Shape::GetSubShapeTransformedShape(const SubShapeID &inSubShapeID, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale, SubShapeID &outRemainder) const
{
	// We have reached the leaf shape so there is no remainder
	outRemainder = SubShapeID();

	// Just return the transformed shape for this shape
	TransformedShape ts(RVec3(inPositionCOM), inRotation, this, BodyID());
	ts.SetShapeScale(inScale);
	return ts;
}

void Shape::CollectTransformedShapes(const AABox &inBox, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale, const SubShapeIDCreator &inSubShapeIDCreator, TransformedShapeCollector &ioCollector, const ShapeFilter &inShapeFilter) const
{
	// Test shape filter
	if (!inShapeFilter.ShouldCollide(this, inSubShapeIDCreator.GetID()))
		return;

	TransformedShape ts(RVec3(inPositionCOM), inRotation, this, TransformedShape::sGetBodyID(ioCollector.GetContext()), inSubShapeIDCreator);
	ts.SetShapeScale(inScale);
	ioCollector.AddHit(ts);
}

void Shape::TransformShape(Mat44Arg inCenterOfMassTransform, TransformedShapeCollector &ioCollector) const
{
	Vec3 scale;
	Mat44 transform = inCenterOfMassTransform.Decompose(scale);
	TransformedShape ts(RVec3(transform.GetTranslation()), transform.GetQuaternion(), this, BodyID(), SubShapeIDCreator());
	ts.SetShapeScale(MakeScaleValid(scale));
	ioCollector.AddHit(ts);
}

void Shape::SaveBinaryState(StreamOut &inStream) const
{
	inStream.Write(mShapeSubType);
	inStream.Write(mUserData);
}

void Shape::RestoreBinaryState(StreamIn &inStream)
{
	// Type hash read by sRestoreFromBinaryState
	inStream.Read(mUserData);
}

Shape::ShapeResult Shape::sRestoreFromBinaryState(StreamIn &inStream)
{
	ShapeResult result;

	// Read the type of the shape
	EShapeSubType shape_sub_type;
	inStream.Read(shape_sub_type);
	if (inStream.IsEOF() || inStream.IsFailed())
	{
		result.SetError("Failed to read type id");
		return result;
	}

	// Construct and read the data of the shape
	Ref<Shape> shape = ShapeFunctions::sGet(shape_sub_type).mConstruct();
	shape->RestoreBinaryState(inStream);
	if (inStream.IsEOF() || inStream.IsFailed())
	{
		result.SetError("Failed to restore shape");
		return result;
	}

	result.Set(shape);
	return result;
}

void Shape::SaveWithChildren(StreamOut &inStream, ShapeToIDMap &ioShapeMap, MaterialToIDMap &ioMaterialMap) const
{
	ShapeToIDMap::const_iterator shape_id_iter = ioShapeMap.find(this);
	if (shape_id_iter == ioShapeMap.end())
	{
		// Write shape ID of this shape
		uint32 shape_id = ioShapeMap.size();
		ioShapeMap[this] = shape_id;
		inStream.Write(shape_id);

		// Write the shape itself
		SaveBinaryState(inStream);

		// Write the ID's of all sub shapes
		ShapeList sub_shapes;
		SaveSubShapeState(sub_shapes);
		inStream.Write(uint32(sub_shapes.size()));
		for (const Shape *shape : sub_shapes)
		{
			if (shape == nullptr)
				inStream.Write(~uint32(0));
			else
				shape->SaveWithChildren(inStream, ioShapeMap, ioMaterialMap);
		}

		// Write the materials
		PhysicsMaterialList materials;
		SaveMaterialState(materials);
		StreamUtils::SaveObjectArray(inStream, materials, &ioMaterialMap);
	}
	else
	{
		// Known shape, just write the ID
		inStream.Write(shape_id_iter->second);
	}
}

Shape::ShapeResult Shape::sRestoreWithChildren(StreamIn &inStream, IDToShapeMap &ioShapeMap, IDToMaterialMap &ioMaterialMap)
{
	ShapeResult result;

	// Read ID of this shape
	uint32 shape_id;
	inStream.Read(shape_id);
	if (inStream.IsEOF() || inStream.IsFailed())
	{
		result.SetError("Failed to read shape id");
		return result;
	}

	// Check nullptr shape
	if (shape_id == ~uint32(0))
	{
		result.Set(nullptr);
		return result;
	}

	// Check if we already read this shape
	if (shape_id < ioShapeMap.size())
	{
		result.Set(ioShapeMap[shape_id]);
		return result;
	}

	// Read the shape
	result = sRestoreFromBinaryState(inStream);
	if (result.HasError())
		return result;
	MOSS_ASSERT(ioShapeMap.size() == shape_id); // Assert that this is the next ID in the map
	ioShapeMap.push_back(result.Get());

	// Read the sub shapes
	uint32 len;
	inStream.Read(len);
	if (inStream.IsEOF() || inStream.IsFailed())
	{
		result.SetError("Failed to read stream");
		return result;
	}
	ShapeList sub_shapes;
	sub_shapes.reserve(len);
	for (size_t i = 0; i < len; ++i)
	{
		ShapeResult sub_shape_result = sRestoreWithChildren(inStream, ioShapeMap, ioMaterialMap);
		if (sub_shape_result.HasError())
			return sub_shape_result;
		sub_shapes.push_back(sub_shape_result.Get());
	}
	result.Get()->RestoreSubShapeState(sub_shapes.data(), (uint)sub_shapes.size());

	// Read the materials
	Result mlresult = StreamUtils::RestoreObjectArray<PhysicsMaterialList>(inStream, ioMaterialMap);
	if (mlresult.HasError())
	{
		result.SetError(mlresult.GetError());
		return result;
	}
	const PhysicsMaterialList &materials = mlresult.Get();
	result.Get()->RestoreMaterialState(materials.data(), (uint)materials.size());

	return result;
}

Shape::Stats Shape::GetStatsRecursive(VisitedShapes &ioVisitedShapes) const
{
	Stats stats = GetStats();

	// If shape is already visited, don't count its size again
	if (!ioVisitedShapes.insert(this).second)
		stats.mSizeBytes = 0;

	return stats;
}

bool Shape::IsValidScale(Vec3Arg inScale) const
{
	return !ScaleHelpers::IsZeroScale(inScale);
}

Vec3 Shape::MakeScaleValid(Vec3Arg inScale) const
{
	return ScaleHelpers::MakeNonZeroScale(inScale);
}

Shape::ShapeResult Shape::ScaleShape(Vec3Arg inScale) const
{
	const Vec3 unit_scale = Vec3::sOne();

	if (inScale.IsNearZero())
	{
		ShapeResult result;
		result.SetError("Can't use zero scale!");
		return result;
	}

	// First test if we can just wrap this shape in a scaled shape
	if (IsValidScale(inScale))
	{
		// Test if the scale is near unit
		ShapeResult result;
		if (inScale.IsClose(unit_scale))
			result.Set(const_cast<Shape *>(this));
		else
			result.Set(new ScaledShape(this, inScale));
		return result;
	}

	// Collect the leaf shapes and their transforms
	struct Collector : TransformedShapeCollector
	{
		virtual void				AddHit(const ResultType &inResult) override
		{
			mShapes.push_back(inResult);
		}

		TArray<TransformedShape>		mShapes;
	};
	Collector collector;
	TransformShape(Mat44::sScale(inScale) * Mat44::sTranslation(GetCenterOfMass()), collector);

	// Construct a compound shape
	StaticCompoundShapeSettings compound;
	compound.mSubShapes.reserve(collector.mShapes.size());
	for (const TransformedShape &ts : collector.mShapes)
	{
		const Shape *shape = ts.mShape;

		// Construct a scaled shape if scale is not unit
		Vec3 scale = ts.GetShapeScale();
		if (!scale.IsClose(unit_scale))
			shape = new ScaledShape(shape, scale);

		// Add the shape
		compound.AddShape(Vec3(ts.mShapePositionCOM) - ts.mShapeRotation * shape->GetCenterOfMass(), ts.mShapeRotation, shape);
	}

	return compound.Create();
}

void Shape::sCollidePointUsingRayCast(const Shape &inShape, Vec3Arg inPoint, const SubShapeIDCreator &inSubShapeIDCreator, CollidePointCollector &ioCollector, const ShapeFilter &inShapeFilter)
{
	// First test if we're inside our bounding box
	AABox bounds = inShape.GetLocalBounds();
	if (bounds.Contains(inPoint))
	{
		// A collector that just counts the number of hits
		class HitCountCollector : public CastRayCollector
		{
		public:
			virtual void	AddHit(const RayCastResult &inResult) override
			{
				// Store the last sub shape ID so that we can provide something to our outer hit collector
				mSubShapeID = inResult.mSubShapeID2;

				++mHitCount;
			}

			int				mHitCount = 0;
			SubShapeID		mSubShapeID;
		};
		HitCountCollector collector;

		// Configure the raycast
		RayCastSettings settings;
		settings.SetBackFaceMode(EBackFaceMode::CollideWithBackFaces);

		// Cast a ray that's 10% longer than the height of our bounding box
		inShape.CastRay(RayCast { inPoint, 1.1f * bounds.GetSize().GetY() * Vec3::sAxisY() }, settings, inSubShapeIDCreator, collector, inShapeFilter);

		// Odd amount of hits means inside
		if ((collector.mHitCount & 1) == 1)
			ioCollector.AddHit({ TransformedShape::sGetBodyID(ioCollector.GetContext()), collector.mSubShapeID });
	}
}

/*													*/

ShapeSettings::ShapeResult CapsuleShapeSettings::Create() const
{
	if (mCachedResult.IsEmpty())
	{
		Ref<Shape> shape;
		if (IsValid() && IsSphere())
		{
			// If the capsule has no height, use a sphere instead
			shape = new SphereShape(mRadius, mMaterial);
			mCachedResult.Set(shape);
		}
		else
			shape = new CapsuleShape(*this, mCachedResult);
	}
	return mCachedResult;
}

CapsuleShape::CapsuleShape(const CapsuleShapeSettings &inSettings, ShapeResult &outResult) :
	ConvexShape(EShapeSubType::Capsule, inSettings, outResult),
	mRadius(inSettings.mRadius),
	mHalfHeightOfCylinder(inSettings.mHalfHeightOfCylinder)
{
	if (inSettings.mHalfHeightOfCylinder <= 0.0f)
	{
		outResult.SetError("Invalid height");
		return;
	}

	if (inSettings.mRadius <= 0.0f)
	{
		outResult.SetError("Invalid radius");
		return;
	}

	outResult.Set(this);
}

class CapsuleShape::CapsuleNoConvex final : public Support
{
public:
					CapsuleNoConvex(Vec3Arg inHalfHeightOfCylinder, float inConvexRadius) :
		mHalfHeightOfCylinder(inHalfHeightOfCylinder),
		mConvexRadius(inConvexRadius)
	{
		static_assert(sizeof(CapsuleNoConvex) <= sizeof(SupportBuffer), "Buffer size too small");
		MOSS_ASSERT(IsAligned(this, alignof(CapsuleNoConvex)));
	}

	virtual Vec3	GetSupport(Vec3Arg inDirection) const override
	{
		if (inDirection.GetY() > 0)
			return mHalfHeightOfCylinder;
		else
			return -mHalfHeightOfCylinder;
	}

	virtual float	GetConvexRadius() const override
	{
		return mConvexRadius;
	}

private:
	Vec3			mHalfHeightOfCylinder;
	float			mConvexRadius;
};

class CapsuleShape::CapsuleWithConvex final : public Support
{
public:
					CapsuleWithConvex(Vec3Arg inHalfHeightOfCylinder, float inRadius) :
		mHalfHeightOfCylinder(inHalfHeightOfCylinder),
		mRadius(inRadius)
	{
		static_assert(sizeof(CapsuleWithConvex) <= sizeof(SupportBuffer), "Buffer size too small");
		MOSS_ASSERT(IsAligned(this, alignof(CapsuleWithConvex)));
	}

	virtual Vec3	GetSupport(Vec3Arg inDirection) const override
	{
		float len = inDirection.Length();
		Vec3 radius = len > 0.0f? inDirection * (mRadius / len) : Vec3::sZero();

		if (inDirection.GetY() > 0)
			return radius + mHalfHeightOfCylinder;
		else
			return radius - mHalfHeightOfCylinder;
	}

	virtual float	GetConvexRadius() const override
	{
		return 0.0f;
	}

private:
	Vec3			mHalfHeightOfCylinder;
	float			mRadius;
};

const ConvexShape::Support *CapsuleShape::GetSupportFunction(ESupportMode inMode, SupportBuffer &inBuffer, Vec3Arg inScale) const
{
	MOSS_ASSERT(IsValidScale(inScale));

	// Get scaled capsule
	Vec3 abs_scale = inScale.Abs();
	float scale = abs_scale.GetX();
	Vec3 scaled_half_height_of_cylinder = Vec3(0, scale * mHalfHeightOfCylinder, 0);
	float scaled_radius = scale * mRadius;

	switch (inMode)
	{
	case ESupportMode::IncludeConvexRadius:
		return new (&inBuffer) CapsuleWithConvex(scaled_half_height_of_cylinder, scaled_radius);

	case ESupportMode::ExcludeConvexRadius:
	case ESupportMode::Default:
		return new (&inBuffer) CapsuleNoConvex(scaled_half_height_of_cylinder, scaled_radius);
	}

	MOSS_ASSERT(false);
	return nullptr;
}

void CapsuleShape::GetSupportingFace(const SubShapeID &inSubShapeID, Vec3Arg inDirection, Vec3Arg inScale, Mat44Arg inCenterOfMassTransform, SupportingFace &outVertices) const
{
	MOSS_ASSERT(inSubShapeID.IsEmpty(), "Invalid subshape ID");
	MOSS_ASSERT(IsValidScale(inScale));

	// Get direction in horizontal plane
	Vec3 direction = inDirection;
	direction.SetComponent(1, 0.0f);

	// Check zero vector, in this case we're hitting from top/bottom so there's no supporting face
	float len = direction.Length();
	if (len == 0.0f)
		return;

	// Get scaled capsule
	Vec3 abs_scale = inScale.Abs();
	float scale = abs_scale.GetX();
	Vec3 scaled_half_height_of_cylinder = Vec3(0, scale * mHalfHeightOfCylinder, 0);
	float scaled_radius = scale * mRadius;

	// Get support point for top and bottom sphere in the opposite of 'direction' (including convex radius)
	Vec3 support = (scaled_radius / len) * direction;
	Vec3 support_top = scaled_half_height_of_cylinder - support;
	Vec3 support_bottom = -scaled_half_height_of_cylinder - support;

	// Get projection on inDirection
	// Note that inDirection is not normalized, so we need to divide by inDirection.Length() to get the actual projection
	// We've multiplied both sides of the if below with inDirection.Length()
	float proj_top = support_top.Dot(inDirection);
	float proj_bottom = support_bottom.Dot(inDirection);

	// If projection is roughly equal then return line, otherwise we return nothing as there's only 1 point
	if (abs(proj_top - proj_bottom) < cCapsuleProjectionSlop * inDirection.Length())
	{
		outVertices.push_back(inCenterOfMassTransform * support_top);
		outVertices.push_back(inCenterOfMassTransform * support_bottom);
	}
}

MassProperties CapsuleShape::GetMassProperties() const
{
	MassProperties p;

	float density = GetDensity();

	// Calculate inertia and mass according to:
	// https://www.gamedev.net/resources/_/technical/math-and-physics/capsule-inertia-tensor-r3856
	// Note that there is an error in eq 14, H^2/2 should be H^2/4 in Ixx and Izz, eq 12 does contain the correct value
	float radius_sq = Square(mRadius);
	float height = 2.0f * mHalfHeightOfCylinder;
	float cylinder_mass = MOSS_PI * height * radius_sq * density;
	float hemisphere_mass = (2.0f * MOSS_PI / 3.0f) * radius_sq * mRadius * density;

	// From cylinder
	float height_sq = Square(height);
	float inertia_y = radius_sq * cylinder_mass * 0.5f;
	float inertia_xz = inertia_y * 0.5f + cylinder_mass * height_sq / 12.0f;

	// From hemispheres
	float temp = hemisphere_mass * 4.0f * radius_sq / 5.0f;
	inertia_y += temp;
	inertia_xz += temp + hemisphere_mass * (0.5f * height_sq + (3.0f / 4.0f) * height * mRadius);

	// Mass is cylinder + hemispheres
	p.mMass = cylinder_mass + hemisphere_mass * 2.0f;

	// Set inertia
	p.mInertia = Mat44::sScale(Vec3(inertia_xz, inertia_y, inertia_xz));

	return p;
}

Vec3 CapsuleShape::GetSurfaceNormal(const SubShapeID &inSubShapeID, Vec3Arg inLocalSurfacePosition) const
{
	MOSS_ASSERT(inSubShapeID.IsEmpty(), "Invalid subshape ID");

	if (inLocalSurfacePosition.GetY() > mHalfHeightOfCylinder)
		return (inLocalSurfacePosition - Vec3(0, mHalfHeightOfCylinder, 0)).Normalized();
	else if (inLocalSurfacePosition.GetY() < -mHalfHeightOfCylinder)
		return (inLocalSurfacePosition - Vec3(0, -mHalfHeightOfCylinder, 0)).Normalized();
	else
		return Vec3(inLocalSurfacePosition.GetX(), 0, inLocalSurfacePosition.GetZ()).NormalizedOr(Vec3::sAxisX());
}

AABox CapsuleShape::GetLocalBounds() const
{
	Vec3 extent = Vec3::sReplicate(mRadius) + Vec3(0, mHalfHeightOfCylinder, 0);
	return AABox(-extent, extent);
}

AABox CapsuleShape::GetWorldSpaceBounds(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale) const
{
	MOSS_ASSERT(IsValidScale(inScale));

	Vec3 abs_scale = inScale.Abs();
	float scale = abs_scale.GetX();
	Vec3 extent = Vec3::sReplicate(scale * mRadius);
	Vec3 height = Vec3(0, scale * mHalfHeightOfCylinder, 0);
	Vec3 p1 = inCenterOfMassTransform * -height;
	Vec3 p2 = inCenterOfMassTransform * height;
	return AABox(Vec3::sMin(p1, p2) - extent, Vec3::sMax(p1, p2) + extent);
}

#ifndef MOSS_DEBUG_RENDERER
void CapsuleShape::Draw(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, Vec3Arg inScale, ColorArg inColor, bool inUseMaterialColors, bool inDrawWireframe) const
{
	DebugRenderer::EDrawMode draw_mode = inDrawWireframe? DebugRenderer::EDrawMode::Wireframe : DebugRenderer::EDrawMode::Solid;
	inRenderer->DrawCapsule(inCenterOfMassTransform * Mat44::sScale(inScale.Abs().GetX()), mHalfHeightOfCylinder, mRadius, inUseMaterialColors? GetMaterial()->GetDebugColor() : inColor, DebugRenderer::ECastShadow::On, draw_mode);
}
#endif // MOSS_DEBUG_RENDERER

bool CapsuleShape::CastRay(const RayCast &inRay, const SubShapeIDCreator &inSubShapeIDCreator, RayCastResult &ioHit) const
{
	// Test ray against capsule
	float fraction = RayCapsule(inRay.mOrigin, inRay.mDirection, mHalfHeightOfCylinder, mRadius);
	if (fraction < ioHit.mFraction)
	{
		ioHit.mFraction = fraction;
		ioHit.mSubShapeID2 = inSubShapeIDCreator.GetID();
		return true;
	}
	return false;
}

void CapsuleShape::CollidePoint(Vec3Arg inPoint, const SubShapeIDCreator &inSubShapeIDCreator, CollidePointCollector &ioCollector, const ShapeFilter &inShapeFilter) const
{
	// Test shape filter
	if (!inShapeFilter.ShouldCollide(this, inSubShapeIDCreator.GetID()))
		return;

	float radius_sq = Square(mRadius);

	// Get vertical distance to the top/bottom sphere centers
	float delta_y = abs(inPoint.GetY()) - mHalfHeightOfCylinder;

	// Get distance in horizontal plane
	float xz_sq = Square(inPoint.GetX()) + Square(inPoint.GetZ());

	// Check if the point is in one of the two spheres
	bool in_sphere = xz_sq + Square(delta_y) <= radius_sq;

	// Check if the point is in the cylinder in the middle
	bool in_cylinder = delta_y <= 0.0f && xz_sq <= radius_sq;

	if (in_sphere || in_cylinder)
		ioCollector.AddHit({ TransformedShape::sGetBodyID(ioCollector.GetContext()), inSubShapeIDCreator.GetID() });
}

void CapsuleShape::CollideSoftBodyVertices(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const CollideSoftBodyVertexIterator &inVertices, uint inNumVertices, int inCollidingShapeIndex) const
{
	MOSS_ASSERT(IsValidScale(inScale));

	Mat44 inverse_transform = inCenterOfMassTransform.InversedRotationTranslation();

	// Get scaled capsule
	float scale = abs(inScale.GetX());
	float half_height_of_cylinder = scale * mHalfHeightOfCylinder;
	float radius = scale * mRadius;

	for (CollideSoftBodyVertexIterator v = inVertices, sbv_end = inVertices + inNumVertices; v != sbv_end; ++v)
		if (v.GetInvMass() > 0.0f)
		{
			// Calculate penetration
			Vec3 local_pos = inverse_transform * v.GetPosition();
			if (abs(local_pos.GetY()) <= half_height_of_cylinder)
			{
				// Near cylinder
				Vec3 normal = local_pos;
				normal.SetY(0.0f);
				float normal_length = normal.Length();
				float penetration = radius - normal_length;
				if (v.UpdatePenetration(penetration))
				{
					// Calculate contact point and normal
					normal = normal_length > 0.0f? normal / normal_length : Vec3::sAxisX();
					Vec3 point = radius * normal;

					// Store collision
					v.SetCollision(Plane::sFromPointAndNormal(point, normal).GetTransformed(inCenterOfMassTransform), inCollidingShapeIndex);
				}
			}
			else
			{
				// Near cap
				Vec3 center = Vec3(0, Sign(local_pos.GetY()) * half_height_of_cylinder, 0);
				Vec3 delta = local_pos - center;
				float distance = delta.Length();
				float penetration = radius - distance;
				if (v.UpdatePenetration(penetration))
				{
					// Calculate contact point and normal
					Vec3 normal = delta / distance;
					Vec3 point = center + radius * normal;

					// Store collision
					v.SetCollision(Plane::sFromPointAndNormal(point, normal).GetTransformed(inCenterOfMassTransform), inCollidingShapeIndex);
				}
			}
		}
}

void CapsuleShape::GetTrianglesStart(GetTrianglesContext &ioContext, const AABox &inBox, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale) const
{
	MOSS_ASSERT(IsValidScale(inScale));

	Vec3 abs_scale = inScale.Abs();
	float scale = abs_scale.GetX();

	GetTrianglesContextMultiVertexList *context = new (&ioContext) GetTrianglesContextMultiVertexList(false, GetMaterial());

	Mat44 world_matrix = Mat44::sRotationTranslation(inRotation, inPositionCOM) * Mat44::sScale(scale);

	Mat44 top_matrix = world_matrix * Mat44(Vec4(mRadius, 0, 0, 0), Vec4(0, mRadius, 0, 0), Vec4(0, 0, mRadius, 0), Vec4(0, mHalfHeightOfCylinder, 0, 1));
	context->AddPart(top_matrix, sCapsuleTopTriangles.data(), sCapsuleTopTriangles.size());

	Mat44 middle_matrix = world_matrix * Mat44::sScale(Vec3(mRadius, mHalfHeightOfCylinder, mRadius));
	context->AddPart(middle_matrix, sCapsuleMiddleTriangles.data(), sCapsuleMiddleTriangles.size());

	Mat44 bottom_matrix = world_matrix * Mat44(Vec4(mRadius, 0, 0, 0), Vec4(0, mRadius, 0, 0), Vec4(0, 0, mRadius, 0), Vec4(0, -mHalfHeightOfCylinder, 0, 1));
	context->AddPart(bottom_matrix, sCapsuleBottomTriangles.data(), sCapsuleBottomTriangles.size());
}

int CapsuleShape::GetTrianglesNext(GetTrianglesContext &ioContext, int inMaxTrianglesRequested, Float3 *outTriangleVertices, const PhysicsMaterial **outMaterials) const
{
	return ((GetTrianglesContextMultiVertexList &)ioContext).GetTrianglesNext(inMaxTrianglesRequested, outTriangleVertices, outMaterials);
}

void CapsuleShape::SaveBinaryState(StreamOut &inStream) const
{
	ConvexShape::SaveBinaryState(inStream);

	inStream.Write(mRadius);
	inStream.Write(mHalfHeightOfCylinder);
}

void CapsuleShape::RestoreBinaryState(StreamIn &inStream)
{
	ConvexShape::RestoreBinaryState(inStream);

	inStream.Read(mRadius);
	inStream.Read(mHalfHeightOfCylinder);
}

bool CapsuleShape::IsValidScale(Vec3Arg inScale) const
{
	return ConvexShape::IsValidScale(inScale) && ScaleHelpers::IsUniformScale(inScale.Abs());
}

Vec3 CapsuleShape::MakeScaleValid(Vec3Arg inScale) const
{
	Vec3 scale = ScaleHelpers::MakeNonZeroScale(inScale);

	return scale.GetSign() * ScaleHelpers::MakeUniformScale(scale.Abs());
}

void CapsuleShape::sRegister()
{
	ShapeFunctions &f = ShapeFunctions::sGet(EShapeSubType::Capsule);
	f.mConstruct = []() -> Shape * { return new CapsuleShape; };
	f.mColor = Color::sGreen;
}

/*													*/

ShapeSettings::ShapeResult BoxShapeSettings::Create() const
{
	if (mCachedResult.IsEmpty())
		Ref<Shape> shape = new BoxShape(*this, mCachedResult);
	return mCachedResult;
}

BoxShape::BoxShape(const BoxShapeSettings &inSettings, ShapeResult &outResult) :
	ConvexShape(EShapeSubType::Box, inSettings, outResult),
	mHalfExtent(inSettings.mHalfExtent),
	mConvexRadius(inSettings.mConvexRadius)
{
	// Check convex radius
	if (inSettings.mConvexRadius < 0.0f
		|| inSettings.mHalfExtent.ReduceMin() < inSettings.mConvexRadius)
	{
		outResult.SetError("Invalid convex radius");
		return;
	}

	// Result is valid
	outResult.Set(this);
}

class BoxShape::Box final : public Support
{
public:
					Box(const AABox &inBox, float inConvexRadius) :
		mBox(inBox),
		mConvexRadius(inConvexRadius)
	{
		static_assert(sizeof(Box) <= sizeof(SupportBuffer), "Buffer size too small");
		MOSS_ASSERT(IsAligned(this, alignof(Box)));
	}

	virtual Vec3	GetSupport(Vec3Arg inDirection) const override
	{
		return mBox.GetSupport(inDirection);
	}

	virtual float	GetConvexRadius() const override
	{
		return mConvexRadius;
	}

private:
	AABox			mBox;
	float			mConvexRadius;
};

const ConvexShape::Support *BoxShape::GetSupportFunction(ESupportMode inMode, SupportBuffer &inBuffer, Vec3Arg inScale) const
{
	// Scale our half extents
	Vec3 scaled_half_extent = inScale.Abs() * mHalfExtent;

	switch (inMode)
	{
	case ESupportMode::IncludeConvexRadius:
	case ESupportMode::Default:
		{
			// Make box out of our half extents
			AABox box = AABox(-scaled_half_extent, scaled_half_extent);
			MOSS_ASSERT(box.IsValid());
			return new (&inBuffer) Box(box, 0.0f);
		}

	case ESupportMode::ExcludeConvexRadius:
		{
			// Reduce the box by our convex radius
			float convex_radius = ScaleHelpers::ScaleConvexRadius(mConvexRadius, inScale);
			Vec3 convex_radius3 = Vec3::sReplicate(convex_radius);
			Vec3 reduced_half_extent = scaled_half_extent - convex_radius3;
			AABox box = AABox(-reduced_half_extent, reduced_half_extent);
			MOSS_ASSERT(box.IsValid());
			return new (&inBuffer) Box(box, convex_radius);
		}
	}

	MOSS_ASSERT(false);
	return nullptr;
}

void BoxShape::GetSupportingFace(const SubShapeID &inSubShapeID, Vec3Arg inDirection, Vec3Arg inScale, Mat44Arg inCenterOfMassTransform, SupportingFace &outVertices) const
{
	MOSS_ASSERT(inSubShapeID.IsEmpty(), "Invalid subshape ID");

	Vec3 scaled_half_extent = inScale.Abs() * mHalfExtent;
	AABox box(-scaled_half_extent, scaled_half_extent);
	box.GetSupportingFace(inDirection, outVertices);

	// Transform to world space
	for (Vec3 &v : outVertices)
		v = inCenterOfMassTransform * v;
}

MassProperties BoxShape::GetMassProperties() const
{
	MassProperties p;
	p.SetMassAndInertiaOfSolidBox(2.0f * mHalfExtent, GetDensity());
	return p;
}

Vec3 BoxShape::GetSurfaceNormal(const SubShapeID &inSubShapeID, Vec3Arg inLocalSurfacePosition) const
{
	MOSS_ASSERT(inSubShapeID.IsEmpty(), "Invalid subshape ID");

	// Get component that is closest to the surface of the box
	int index = (inLocalSurfacePosition.Abs() - mHalfExtent).Abs().GetLowestComponentIndex();

	// Calculate normal
	Vec3 normal = Vec3::sZero();
	normal.SetComponent(index, inLocalSurfacePosition[index] > 0.0f? 1.0f : -1.0f);
	return normal;
}

#ifndef MOSS_DEBUG_RENDERER
void BoxShape::Draw(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, Vec3Arg inScale, ColorArg inColor, bool inUseMaterialColors, bool inDrawWireframe) const
{
	DebugRenderer::EDrawMode draw_mode = inDrawWireframe? DebugRenderer::EDrawMode::Wireframe : DebugRenderer::EDrawMode::Solid;
	inRenderer->DrawBox(inCenterOfMassTransform * Mat44::sScale(inScale.Abs()), GetLocalBounds(), inUseMaterialColors? GetMaterial()->GetDebugColor() : inColor, DebugRenderer::ECastShadow::On, draw_mode);
}
#endif // MOSS_DEBUG_RENDERER

bool BoxShape::CastRay(const RayCast &inRay, const SubShapeIDCreator &inSubShapeIDCreator, RayCastResult &ioHit) const
{
	// Test hit against box
	float fraction = max(RayAABox(inRay.mOrigin, RayInvDirection(inRay.mDirection), -mHalfExtent, mHalfExtent), 0.0f);
	if (fraction < ioHit.mFraction)
	{
		ioHit.mFraction = fraction;
		ioHit.mSubShapeID2 = inSubShapeIDCreator.GetID();
		return true;
	}
	return false;
}

void BoxShape::CastRay(const RayCast &inRay, const RayCastSettings &inRayCastSettings, const SubShapeIDCreator &inSubShapeIDCreator, CastRayCollector &ioCollector, const ShapeFilter &inShapeFilter) const
{
	// Test shape filter
	if (!inShapeFilter.ShouldCollide(this, inSubShapeIDCreator.GetID()))
		return;

	float min_fraction, max_fraction;
	RayAABox(inRay.mOrigin, RayInvDirection(inRay.mDirection), -mHalfExtent, mHalfExtent, min_fraction, max_fraction);
	if (min_fraction <= max_fraction // Ray should intersect
		&& max_fraction >= 0.0f // End of ray should be inside box
		&& min_fraction < ioCollector.GetEarlyOutFraction()) // Start of ray should be before early out fraction
	{
		// Better hit than the current hit
		RayCastResult hit;
		hit.mBodyID = TransformedShape::sGetBodyID(ioCollector.GetContext());
		hit.mSubShapeID2 = inSubShapeIDCreator.GetID();

		// Check front side
		if (inRayCastSettings.mTreatConvexAsSolid || min_fraction > 0.0f)
		{
			hit.mFraction = max(0.0f, min_fraction);
			ioCollector.AddHit(hit);
		}

		// Check back side hit
		if (inRayCastSettings.mBackFaceModeConvex == EBackFaceMode::CollideWithBackFaces
			&& max_fraction < ioCollector.GetEarlyOutFraction())
		{
			hit.mFraction = max_fraction;
			ioCollector.AddHit(hit);
		}
	}
}

void BoxShape::CollidePoint(Vec3Arg inPoint, const SubShapeIDCreator &inSubShapeIDCreator, CollidePointCollector &ioCollector, const ShapeFilter &inShapeFilter) const
{
	// Test shape filter
	if (!inShapeFilter.ShouldCollide(this, inSubShapeIDCreator.GetID()))
		return;

	if (Vec3::sLessOrEqual(inPoint.Abs(), mHalfExtent).TestAllXYZTrue())
		ioCollector.AddHit({ TransformedShape::sGetBodyID(ioCollector.GetContext()), inSubShapeIDCreator.GetID() });
}

void BoxShape::CollideSoftBodyVertices(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const CollideSoftBodyVertexIterator &inVertices, uint inNumVertices, int inCollidingShapeIndex) const
{
	Mat44 inverse_transform = inCenterOfMassTransform.InversedRotationTranslation();
	Vec3 half_extent = inScale.Abs() * mHalfExtent;

	for (CollideSoftBodyVertexIterator v = inVertices, sbv_end = inVertices + inNumVertices; v != sbv_end; ++v)
		if (v.GetInvMass() > 0.0f)
		{
			// Convert to local space
			Vec3 local_pos = inverse_transform * v.GetPosition();

			// Clamp point to inside box
			Vec3 clamped_point = Vec3::sMax(Vec3::sMin(local_pos, half_extent), -half_extent);

			// Test if point was inside
			if (clamped_point == local_pos)
			{
				// Calculate closest distance to surface
				Vec3 delta = half_extent - local_pos.Abs();
				int index = delta.GetLowestComponentIndex();
				float penetration = delta[index];
				if (v.UpdatePenetration(penetration))
				{
					// Calculate contact point and normal
					Vec3 possible_normals[] = { Vec3::sAxisX(), Vec3::sAxisY(), Vec3::sAxisZ() };
					Vec3 normal = local_pos.GetSign() * possible_normals[index];
					Vec3 point = normal * half_extent;

					// Store collision
					v.SetCollision(Plane::sFromPointAndNormal(point, normal).GetTransformed(inCenterOfMassTransform), inCollidingShapeIndex);
				}
			}
			else
			{
				// Calculate normal
				Vec3 normal = local_pos - clamped_point;
				float normal_length = normal.Length();

				// Penetration will be negative since we're not penetrating
				float penetration = -normal_length;
				if (v.UpdatePenetration(penetration))
				{
					normal /= normal_length;

					// Store collision
					v.SetCollision(Plane::sFromPointAndNormal(clamped_point, normal).GetTransformed(inCenterOfMassTransform), inCollidingShapeIndex);
				}
			}
		}
}

void BoxShape::GetTrianglesStart(GetTrianglesContext &ioContext, const AABox &inBox, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale) const
{
	new (&ioContext) GetTrianglesContextVertexList(inPositionCOM, inRotation, inScale, Mat44::sScale(mHalfExtent), sUnitBoxTriangles, std::size(sUnitBoxTriangles), GetMaterial());
}

int BoxShape::GetTrianglesNext(GetTrianglesContext &ioContext, int inMaxTrianglesRequested, Float3 *outTriangleVertices, const PhysicsMaterial **outMaterials) const
{
	return ((GetTrianglesContextVertexList &)ioContext).GetTrianglesNext(inMaxTrianglesRequested, outTriangleVertices, outMaterials);
}

void BoxShape::SaveBinaryState(StreamOut &inStream) const
{
	ConvexShape::SaveBinaryState(inStream);

	inStream.Write(mHalfExtent);
	inStream.Write(mConvexRadius);
}

void BoxShape::RestoreBinaryState(StreamIn &inStream)
{
	ConvexShape::RestoreBinaryState(inStream);

	inStream.Read(mHalfExtent);
	inStream.Read(mConvexRadius);
}

void BoxShape::sRegister()
{
	ShapeFunctions &f = ShapeFunctions::sGet(EShapeSubType::Box);
	f.mConstruct = []() -> Shape * { return new BoxShape; };
	f.mColor = Color::sGreen;
}

/*													*/

void CompoundShapeSettings::AddShape(Vec3Arg inPosition, QuatArg inRotation, const ShapeSettings *inShape, uint32 inUserData)
{
	// Add shape
	SubShapeSettings shape;
	shape.mPosition = inPosition;
	shape.mRotation = inRotation;
	shape.mShape = inShape;
	shape.mUserData = inUserData;
	mSubShapes.push_back(shape);
}

void CompoundShapeSettings::AddShape(Vec3Arg inPosition, QuatArg inRotation, const Shape *inShape, uint32 inUserData)
{
	// Add shape
	SubShapeSettings shape;
	shape.mPosition = inPosition;
	shape.mRotation = inRotation;
	shape.mShapePtr = inShape;
	shape.mUserData = inUserData;
	mSubShapes.push_back(shape);
}

bool CompoundShape::MustBeStatic() const
{
	for (const SubShape &shape : mSubShapes)
		if (shape.mShape->MustBeStatic())
			return true;

	return false;
}

MassProperties CompoundShape::GetMassProperties() const
{
	MassProperties p;

	// Calculate mass and inertia
	p.mMass = 0.0f;
	p.mInertia = Mat44::sZero();
	for (const SubShape &shape : mSubShapes)
	{
		// Rotate and translate inertia of child into place
		MassProperties child = shape.mShape->GetMassProperties();
		child.Rotate(Mat44::sRotation(shape.GetRotation()));
		child.Translate(shape.GetPositionCOM());

		// Accumulate mass and inertia
		p.mMass += child.mMass;
		p.mInertia += child.mInertia;
	}

	// Ensure that inertia is a 3x3 matrix, adding inertias causes the bottom right element to change
	p.mInertia.SetColumn4(3, Vec4(0, 0, 0, 1));

	return p;
}

AABox CompoundShape::GetWorldSpaceBounds(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale) const
{
	if (mSubShapes.empty())
	{
		// If there are no sub-shapes, we must return an empty box to avoid overflows in the broadphase
		return AABox(inCenterOfMassTransform.GetTranslation(), inCenterOfMassTransform.GetTranslation());
	}
	else if (mSubShapes.size() <= 10)
	{
		AABox bounds;
		for (const SubShape &shape : mSubShapes)
		{
			Mat44 transform = inCenterOfMassTransform * shape.GetLocalTransformNoScale(inScale);
			bounds.Encapsulate(shape.mShape->GetWorldSpaceBounds(transform, shape.TransformScale(inScale)));
		}
		return bounds;
	}
	else
	{
		// If there are too many shapes, use the base class function (this will result in a slightly wider bounding box)
		return Shape::GetWorldSpaceBounds(inCenterOfMassTransform, inScale);
	}
}

uint CompoundShape::GetSubShapeIDBitsRecursive() const
{
	// Add max of child bits to our bits
	uint child_bits = 0;
	for (const SubShape &shape : mSubShapes)
		child_bits = max(child_bits, shape.mShape->GetSubShapeIDBitsRecursive());
	return child_bits + GetSubShapeIDBits();
}

const PhysicsMaterial *CompoundShape::GetMaterial(const SubShapeID &inSubShapeID) const
{
	// Decode sub shape index
	SubShapeID remainder;
	uint32 index = GetSubShapeIndexFromID(inSubShapeID, remainder);

	// Pass call on
	return mSubShapes[index].mShape->GetMaterial(remainder);
}

const Shape *CompoundShape::GetLeafShape(const SubShapeID &inSubShapeID, SubShapeID &outRemainder) const
{
	// Decode sub shape index
	SubShapeID remainder;
	uint32 index = GetSubShapeIndexFromID(inSubShapeID, remainder);
	if (index >= mSubShapes.size())
	{
		// No longer valid index
		outRemainder = SubShapeID();
		return nullptr;
	}

	// Pass call on
	return mSubShapes[index].mShape->GetLeafShape(remainder, outRemainder);
}

uint64 CompoundShape::GetSubShapeUserData(const SubShapeID &inSubShapeID) const
{
	// Decode sub shape index
	SubShapeID remainder;
	uint32 index = GetSubShapeIndexFromID(inSubShapeID, remainder);
	if (index >= mSubShapes.size())
		return 0; // No longer valid index

	// Pass call on
	return mSubShapes[index].mShape->GetSubShapeUserData(remainder);
}

TransformedShape CompoundShape::GetSubShapeTransformedShape(const SubShapeID &inSubShapeID, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale, SubShapeID &outRemainder) const
{
	// Get the sub shape
	const SubShape &sub_shape = mSubShapes[GetSubShapeIndexFromID(inSubShapeID, outRemainder)];

	// Calculate transform for sub shape
	Vec3 position = inPositionCOM + inRotation * (inScale * sub_shape.GetPositionCOM());
	Quat rotation = inRotation * sub_shape.GetRotation();
	Vec3 scale = sub_shape.TransformScale(inScale);

	// Return transformed shape
	TransformedShape ts(RVec3(position), rotation, sub_shape.mShape, BodyID());
	ts.SetShapeScale(scale);
	return ts;
}

Vec3 CompoundShape::GetSurfaceNormal(const SubShapeID &inSubShapeID, Vec3Arg inLocalSurfacePosition) const
{
	// Decode sub shape index
	SubShapeID remainder;
	uint32 index = GetSubShapeIndexFromID(inSubShapeID, remainder);

	// Transform surface position to local space and pass call on
	const SubShape &shape = mSubShapes[index];
	Mat44 transform = Mat44::sInverseRotationTranslation(shape.GetRotation(), shape.GetPositionCOM());
	Vec3 normal = shape.mShape->GetSurfaceNormal(remainder, transform * inLocalSurfacePosition);

	// Transform normal to this shape's space
	return transform.Multiply3x3Transposed(normal);
}

void CompoundShape::GetSupportingFace(const SubShapeID &inSubShapeID, Vec3Arg inDirection, Vec3Arg inScale, Mat44Arg inCenterOfMassTransform, SupportingFace &outVertices) const
{
	// Decode sub shape index
	SubShapeID remainder;
	uint32 index = GetSubShapeIndexFromID(inSubShapeID, remainder);

	// Apply transform and pass on to sub shape
	const SubShape &shape = mSubShapes[index];
	Mat44 transform = shape.GetLocalTransformNoScale(inScale);
	shape.mShape->GetSupportingFace(remainder, transform.Multiply3x3Transposed(inDirection), shape.TransformScale(inScale), inCenterOfMassTransform * transform, outVertices);
}

void CompoundShape::GetSubmergedVolume(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const Plane &inSurface, float &outTotalVolume, float &outSubmergedVolume, Vec3 &outCenterOfBuoyancy MOSS_IF_DEBUG_RENDERER(, RVec3Arg inBaseOffset)) const
{
	outTotalVolume = 0.0f;
	outSubmergedVolume = 0.0f;
	outCenterOfBuoyancy = Vec3::sZero();

	for (const SubShape &shape : mSubShapes)
	{
		// Get center of mass transform of child
		Mat44 transform = inCenterOfMassTransform * shape.GetLocalTransformNoScale(inScale);

		// Recurse to child
		float total_volume, submerged_volume;
		Vec3 center_of_buoyancy;
		shape.mShape->GetSubmergedVolume(transform, shape.TransformScale(inScale), inSurface, total_volume, submerged_volume, center_of_buoyancy MOSS_IF_DEBUG_RENDERER(, inBaseOffset));

		// Accumulate volumes
		outTotalVolume += total_volume;
		outSubmergedVolume += submerged_volume;

		// The center of buoyancy is the weighted average of the center of buoyancy of our child shapes
		outCenterOfBuoyancy += submerged_volume * center_of_buoyancy;
	}

	if (outSubmergedVolume > 0.0f)
		outCenterOfBuoyancy /= outSubmergedVolume;

#ifndef MOSS_DEBUG_RENDERER
	// Draw center of buoyancy
	if (sDrawSubmergedVolumes)
		DebugRenderer::sInstance->DrawWireSphere(inBaseOffset + outCenterOfBuoyancy, 0.05f, Color::sRed, 1);
#endif // MOSS_DEBUG_RENDERER
}

#ifndef MOSS_DEBUG_RENDERER
void CompoundShape::Draw(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, Vec3Arg inScale, ColorArg inColor, bool inUseMaterialColors, bool inDrawWireframe) const
{
	for (const SubShape &shape : mSubShapes)
	{
		Mat44 transform = shape.GetLocalTransformNoScale(inScale);
		shape.mShape->Draw(inRenderer, inCenterOfMassTransform * transform, shape.TransformScale(inScale), inColor, inUseMaterialColors, inDrawWireframe);
	}
}

void CompoundShape::DrawGetSupportFunction(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, Vec3Arg inScale, ColorArg inColor, bool inDrawSupportDirection) const
{
	for (const SubShape &shape : mSubShapes)
	{
		Mat44 transform = shape.GetLocalTransformNoScale(inScale);
		shape.mShape->DrawGetSupportFunction(inRenderer, inCenterOfMassTransform * transform, shape.TransformScale(inScale), inColor, inDrawSupportDirection);
	}
}

void CompoundShape::DrawGetSupportingFace(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, Vec3Arg inScale) const
{
	for (const SubShape &shape : mSubShapes)
	{
		Mat44 transform = shape.GetLocalTransformNoScale(inScale);
		shape.mShape->DrawGetSupportingFace(inRenderer, inCenterOfMassTransform * transform, shape.TransformScale(inScale));
	}
}
#endif // MOSS_DEBUG_RENDERER

void CompoundShape::CollideSoftBodyVertices(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const CollideSoftBodyVertexIterator &inVertices, uint inNumVertices, int inCollidingShapeIndex) const
{
	for (const SubShape &shape : mSubShapes)
	{
		Mat44 transform = shape.GetLocalTransformNoScale(inScale);
		shape.mShape->CollideSoftBodyVertices(inCenterOfMassTransform * transform, shape.TransformScale(inScale), inVertices, inNumVertices, inCollidingShapeIndex);
	}
}

void CompoundShape::TransformShape(Mat44Arg inCenterOfMassTransform, TransformedShapeCollector &ioCollector) const
{
	for (const SubShape &shape : mSubShapes)
		shape.mShape->TransformShape(inCenterOfMassTransform * Mat44::sRotationTranslation(shape.GetRotation(), shape.GetPositionCOM()), ioCollector);
}

void CompoundShape::sCastCompoundVsShape(const ShapeCast &inShapeCast, const ShapeCastSettings &inShapeCastSettings, const Shape *inShape, Vec3Arg inScale, const ShapeFilter &inShapeFilter, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, CastShapeCollector &ioCollector)
{
	MOSS_PROFILE_FUNCTION();

	// Fetch compound shape from cast shape
	MOSS_ASSERT(inShapeCast.mShape->GetType() == EShapeType::Compound);
	const CompoundShape *compound = static_cast<const CompoundShape *>(inShapeCast.mShape);

	// Number of sub shapes
	int n = (int)compound->mSubShapes.size();

	// Determine amount of bits for sub shape
	uint sub_shape_bits = compound->GetSubShapeIDBits();

	// Recurse to sub shapes
	for (int i = 0; i < n; ++i)
	{
		const SubShape &shape = compound->mSubShapes[i];

		// Create ID for sub shape
		SubShapeIDCreator shape1_sub_shape_id = inSubShapeIDCreator1.PushID(i, sub_shape_bits);

		// Transform the shape cast and update the shape
		Mat44 transform = inShapeCast.mCenterOfMassStart * shape.GetLocalTransformNoScale(inShapeCast.mScale);
		Vec3 scale = shape.TransformScale(inShapeCast.mScale);
		ShapeCast shape_cast(shape.mShape, scale, transform, inShapeCast.mDirection);

		CollisionDispatch::sCastShapeVsShapeLocalSpace(shape_cast, inShapeCastSettings, inShape, inScale, inShapeFilter, inCenterOfMassTransform2, shape1_sub_shape_id, inSubShapeIDCreator2, ioCollector);

		if (ioCollector.ShouldEarlyOut())
			break;
	}
}

void CompoundShape::SaveBinaryState(StreamOut &inStream) const
{
	Shape::SaveBinaryState(inStream);

	inStream.Write(mCenterOfMass);
	inStream.Write(mLocalBounds.mMin);
	inStream.Write(mLocalBounds.mMax);
	inStream.Write(mInnerRadius);

	// Write sub shapes
	inStream.Write(mSubShapes, [](const SubShape &inElement, StreamOut &inS) {
		inS.Write(inElement.mUserData);
		inS.Write(inElement.mPositionCOM);
		inS.Write(inElement.mRotation);
	});
}

void CompoundShape::RestoreBinaryState(StreamIn &inStream)
{
	Shape::RestoreBinaryState(inStream);

	inStream.Read(mCenterOfMass);
	inStream.Read(mLocalBounds.mMin);
	inStream.Read(mLocalBounds.mMax);
	inStream.Read(mInnerRadius);

	// Read sub shapes
	inStream.Read(mSubShapes, [](StreamIn &inS, SubShape &outElement) {
		inS.Read(outElement.mUserData);
		inS.Read(outElement.mPositionCOM);
		inS.Read(outElement.mRotation);
		outElement.mIsRotationIdentity = outElement.mRotation == Float3(0, 0, 0);
	});
}

void CompoundShape::SaveSubShapeState(ShapeList &outSubShapes) const
{
	outSubShapes.clear();
	outSubShapes.reserve(mSubShapes.size());
	for (const SubShape &shape : mSubShapes)
		outSubShapes.push_back(shape.mShape);
}

void CompoundShape::RestoreSubShapeState(const ShapeRefC *inSubShapes, uint inNumShapes)
{
	MOSS_ASSERT(mSubShapes.size() == inNumShapes);
	for (uint i = 0; i < inNumShapes; ++i)
		mSubShapes[i].mShape = inSubShapes[i];
}

Shape::Stats CompoundShape::GetStatsRecursive(VisitedShapes &ioVisitedShapes) const
{
	// Get own stats
	Stats stats = Shape::GetStatsRecursive(ioVisitedShapes);

	// Add child stats
	for (const SubShape &shape : mSubShapes)
	{
		Stats child_stats = shape.mShape->GetStatsRecursive(ioVisitedShapes);
		stats.mSizeBytes += child_stats.mSizeBytes;
		stats.mNumTriangles += child_stats.mNumTriangles;
	}

	return stats;
}

float CompoundShape::GetVolume() const
{
	float volume = 0.0f;
	for (const SubShape &shape : mSubShapes)
		volume += shape.mShape->GetVolume();
	return volume;
}

bool CompoundShape::IsValidScale(Vec3Arg inScale) const
{
	if (!Shape::IsValidScale(inScale))
		return false;

	for (const SubShape &shape : mSubShapes)
	{
		// Test if the scale is non-uniform and the shape is rotated
		if (!shape.IsValidScale(inScale))
			return false;

		// Test the child shape
		if (!shape.mShape->IsValidScale(shape.TransformScale(inScale)))
			return false;
	}

	return true;
}

Vec3 CompoundShape::MakeScaleValid(Vec3Arg inScale) const
{
	Vec3 scale = ScaleHelpers::MakeNonZeroScale(inScale);
	if (CompoundShape::IsValidScale(scale))
		return scale;

	Vec3 abs_uniform_scale = ScaleHelpers::MakeUniformScale(scale.Abs());
	Vec3 uniform_scale = scale.GetSign() * abs_uniform_scale;
	if (CompoundShape::IsValidScale(uniform_scale))
		return uniform_scale;

	return Sign(scale.GetX()) * abs_uniform_scale;
}

void CompoundShape::sRegister()
{
	for (EShapeSubType s1 : sCompoundSubShapeTypes)
		for (EShapeSubType s2 : sAllSubShapeTypes)
			CollisionDispatch::sRegisterCastShape(s1, s2, sCastCompoundVsShape);
}

/*													*/

void ConvexShape::sCollideConvexVsConvex(const Shape *inShape1, const Shape *inShape2, Vec3Arg inScale1, Vec3Arg inScale2, Mat44Arg inCenterOfMassTransform1, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, const CollideShapeSettings &inCollideShapeSettings, CollideShapeCollector &ioCollector, [[maybe_unused]] const ShapeFilter &inShapeFilter)
{
	MOSS_PROFILE_FUNCTION();

	// Get the shapes
	MOSS_ASSERT(inShape1->GetType() == EShapeType::Convex);
	MOSS_ASSERT(inShape2->GetType() == EShapeType::Convex);
	const ConvexShape *shape1 = static_cast<const ConvexShape *>(inShape1);
	const ConvexShape *shape2 = static_cast<const ConvexShape *>(inShape2);

	// Get transforms
	Mat44 inverse_transform1 = inCenterOfMassTransform1.InversedRotationTranslation();
	Mat44 transform_2_to_1 = inverse_transform1 * inCenterOfMassTransform2;

	// Get bounding boxes
	float max_separation_distance = inCollideShapeSettings.mMaxSeparationDistance;
	AABox shape1_bbox = shape1->GetLocalBounds().Scaled(inScale1);
	shape1_bbox.ExpandBy(Vec3::sReplicate(max_separation_distance));
	AABox shape2_bbox = shape2->GetLocalBounds().Scaled(inScale2);

	// Check if they overlap
	if (!OrientedBox(transform_2_to_1, shape2_bbox).Overlaps(shape1_bbox))
		return;

	// Note: As we don't remember the penetration axis from the last iteration, and it is likely that shape2 is pushed out of
	// collision relative to shape1 by comparing their COM's, we use that as an initial penetration axis: shape2.com - shape1.com
	// This has been seen to improve performance by approx. 1% over using a fixed axis like (1, 0, 0).
	Vec3 penetration_axis = transform_2_to_1.GetTranslation();

	// Ensure that we do not pass in a near zero penetration axis
	if (penetration_axis.IsNearZero())
		penetration_axis = Vec3::sAxisX();

	Vec3 point1, point2;
	EPAPenetrationDepth pen_depth;
	EPAPenetrationDepth::EStatus status;

	// Scope to limit lifetime of SupportBuffer
	{
		// Create support function
		SupportBuffer buffer1_excl_cvx_radius, buffer2_excl_cvx_radius;
		const Support *shape1_excl_cvx_radius = shape1->GetSupportFunction(ConvexShape::ESupportMode::ExcludeConvexRadius, buffer1_excl_cvx_radius, inScale1);
		const Support *shape2_excl_cvx_radius = shape2->GetSupportFunction(ConvexShape::ESupportMode::ExcludeConvexRadius, buffer2_excl_cvx_radius, inScale2);

		// Transform shape 2 in the space of shape 1
		TransformedConvexObject transformed2_excl_cvx_radius(transform_2_to_1, *shape2_excl_cvx_radius);

		// Perform GJK step
		status = pen_depth.GetPenetrationDepthStepGJK(*shape1_excl_cvx_radius, shape1_excl_cvx_radius->GetConvexRadius() + max_separation_distance, transformed2_excl_cvx_radius, shape2_excl_cvx_radius->GetConvexRadius(), inCollideShapeSettings.mCollisionTolerance, penetration_axis, point1, point2);
	}

	// Check result of collision detection
	switch (status)
	{
	case EPAPenetrationDepth::EStatus::Colliding:
		break;

	case EPAPenetrationDepth::EStatus::NotColliding:
		return;

	case EPAPenetrationDepth::EStatus::Indeterminate:
		{
			// Need to run expensive EPA algorithm

			// We know we're overlapping at this point, so we can set the max separation distance to 0.
			// Numerically it is possible that GJK finds that the shapes are overlapping but EPA finds that they're separated.
			// In order to avoid this, we clamp the max separation distance to 1 so that we don't excessively inflate the shape,
			// but we still inflate it enough to avoid the case where EPA misses the collision.
			max_separation_distance = min(max_separation_distance, 1.0f);

			// Create support function
			SupportBuffer buffer1_incl_cvx_radius, buffer2_incl_cvx_radius;
			const Support *shape1_incl_cvx_radius = shape1->GetSupportFunction(ConvexShape::ESupportMode::IncludeConvexRadius, buffer1_incl_cvx_radius, inScale1);
			const Support *shape2_incl_cvx_radius = shape2->GetSupportFunction(ConvexShape::ESupportMode::IncludeConvexRadius, buffer2_incl_cvx_radius, inScale2);

			// Add separation distance
			AddConvexRadius shape1_add_max_separation_distance(*shape1_incl_cvx_radius, max_separation_distance);

			// Transform shape 2 in the space of shape 1
			TransformedConvexObject transformed2_incl_cvx_radius(transform_2_to_1, *shape2_incl_cvx_radius);

			// Perform EPA step
			if (!pen_depth.GetPenetrationDepthStepEPA(shape1_add_max_separation_distance, transformed2_incl_cvx_radius, inCollideShapeSettings.mPenetrationTolerance, penetration_axis, point1, point2))
				return;
			break;
		}
	}

	// Check if the penetration is bigger than the early out fraction
	float penetration_depth = (point2 - point1).Length() - max_separation_distance;
	if (-penetration_depth >= ioCollector.GetEarlyOutFraction())
		return;

	// Correct point1 for the added separation distance
	float penetration_axis_len = penetration_axis.Length();
	if (penetration_axis_len > 0.0f)
		point1 -= penetration_axis * (max_separation_distance / penetration_axis_len);

	// Convert to world space
	point1 = inCenterOfMassTransform1 * point1;
	point2 = inCenterOfMassTransform1 * point2;
	Vec3 penetration_axis_world = inCenterOfMassTransform1.Multiply3x3(penetration_axis);

	// Create collision result
	CollideShapeResult result(point1, point2, penetration_axis_world, penetration_depth, inSubShapeIDCreator1.GetID(), inSubShapeIDCreator2.GetID(), TransformedShape::sGetBodyID(ioCollector.GetContext()));

	// Gather faces
	if (inCollideShapeSettings.mCollectFacesMode == ECollectFacesMode::CollectFaces)
	{
		// Get supporting face of shape 1
		shape1->GetSupportingFace(SubShapeID(), -penetration_axis, inScale1, inCenterOfMassTransform1, result.mShape1Face);

		// Get supporting face of shape 2
		shape2->GetSupportingFace(SubShapeID(), transform_2_to_1.Multiply3x3Transposed(penetration_axis), inScale2, inCenterOfMassTransform2, result.mShape2Face);
	}

	// Notify the collector
	MOSS_IF_TRACK_NARROWPHASE_STATS(TrackNarrowPhaseCollector track;)
	ioCollector.AddHit(result);
}

bool ConvexShape::CastRay(const RayCast &inRay, const SubShapeIDCreator &inSubShapeIDCreator, RayCastResult &ioHit) const
{
	// Note: This is a fallback routine, most convex shapes should implement a more performant version!

	MOSS_PROFILE_FUNCTION();

	// Create support function
	SupportBuffer buffer;
	const Support *support = GetSupportFunction(ConvexShape::ESupportMode::IncludeConvexRadius, buffer, Vec3::sOne());

	// Cast ray
	GJKClosestPoint gjk;
	if (gjk.CastRay(inRay.mOrigin, inRay.mDirection, cDefaultCollisionTolerance, *support, ioHit.mFraction))
	{
		ioHit.mSubShapeID2 = inSubShapeIDCreator.GetID();
		return true;
	}

	return false;
}

void ConvexShape::CastRay(const RayCast &inRay, const RayCastSettings &inRayCastSettings, const SubShapeIDCreator &inSubShapeIDCreator, CastRayCollector &ioCollector, const ShapeFilter &inShapeFilter) const
{
	// Note: This is a fallback routine, most convex shapes should implement a more performant version!

	// Test shape filter
	if (!inShapeFilter.ShouldCollide(this, inSubShapeIDCreator.GetID()))
		return;

	// First do a normal raycast, limited to the early out fraction
	RayCastResult hit;
	hit.mFraction = ioCollector.GetEarlyOutFraction();
	if (CastRay(inRay, inSubShapeIDCreator, hit))
	{
		// Check front side
		if (inRayCastSettings.mTreatConvexAsSolid || hit.mFraction > 0.0f)
		{
			hit.mBodyID = TransformedShape::sGetBodyID(ioCollector.GetContext());
			ioCollector.AddHit(hit);
		}

		// Check if we want back facing hits and the collector still accepts additional hits
		if (inRayCastSettings.mBackFaceModeConvex == EBackFaceMode::CollideWithBackFaces && !ioCollector.ShouldEarlyOut())
		{
			// Invert the ray, going from the early out fraction back to the fraction where we found our forward hit
			float start_fraction = min(1.0f, ioCollector.GetEarlyOutFraction());
			float delta_fraction = hit.mFraction - start_fraction;
			if (delta_fraction < 0.0f)
			{
				RayCast inverted_ray { inRay.mOrigin + start_fraction * inRay.mDirection, delta_fraction * inRay.mDirection };

				// Cast another ray
				RayCastResult inverted_hit;
				inverted_hit.mFraction = 1.0f;
				if (CastRay(inverted_ray, inSubShapeIDCreator, inverted_hit)
					&& inverted_hit.mFraction > 0.0f) // Ignore hits with fraction 0, this means the ray ends inside the object and we don't want to report it as a back facing hit
				{
					// Invert fraction and rescale it to the fraction of the original ray
					inverted_hit.mFraction = hit.mFraction + (inverted_hit.mFraction - 1.0f) * delta_fraction;
					inverted_hit.mBodyID = TransformedShape::sGetBodyID(ioCollector.GetContext());
					ioCollector.AddHit(inverted_hit);
				}
			}
		}
	}
}

void ConvexShape::CollidePoint(Vec3Arg inPoint, const SubShapeIDCreator &inSubShapeIDCreator, CollidePointCollector &ioCollector, const ShapeFilter &inShapeFilter) const
{
	// Test shape filter
	if (!inShapeFilter.ShouldCollide(this, inSubShapeIDCreator.GetID()))
		return;

	// First test bounding box
	if (GetLocalBounds().Contains(inPoint))
	{
		// Create support function
		SupportBuffer buffer;
		const Support *support = GetSupportFunction(ConvexShape::ESupportMode::IncludeConvexRadius, buffer, Vec3::sOne());

		// Create support function for point
		PointConvexSupport point { inPoint };

		// Test intersection
		GJKClosestPoint gjk;
		Vec3 v = inPoint;
		if (gjk.Intersects(*support, point, cDefaultCollisionTolerance, v))
			ioCollector.AddHit({ TransformedShape::sGetBodyID(ioCollector.GetContext()), inSubShapeIDCreator.GetID() });
	}
}

void ConvexShape::sCastConvexVsConvex(const ShapeCast &inShapeCast, const ShapeCastSettings &inShapeCastSettings, const Shape *inShape, Vec3Arg inScale, [[maybe_unused]] const ShapeFilter &inShapeFilter, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, CastShapeCollector &ioCollector)
{
	MOSS_PROFILE_FUNCTION();

	// Only supported for convex shapes
	MOSS_ASSERT(inShapeCast.mShape->GetType() == EShapeType::Convex);
	const ConvexShape *cast_shape = static_cast<const ConvexShape *>(inShapeCast.mShape);

	MOSS_ASSERT(inShape->GetType() == EShapeType::Convex);
	const ConvexShape *shape = static_cast<const ConvexShape *>(inShape);

	// Determine if we want to use the actual shape or a shrunken shape with convex radius
	ConvexShape::ESupportMode support_mode = inShapeCastSettings.mUseShrunkenShapeAndConvexRadius? ConvexShape::ESupportMode::ExcludeConvexRadius : ConvexShape::ESupportMode::Default;

	// Create support function for shape to cast
	SupportBuffer cast_buffer;
	const Support *cast_support = cast_shape->GetSupportFunction(support_mode, cast_buffer, inShapeCast.mScale);

	// Create support function for target shape
	SupportBuffer target_buffer;
	const Support *target_support = shape->GetSupportFunction(support_mode, target_buffer, inScale);

	// Do a raycast against the result
	EPAPenetrationDepth epa;
	float fraction = ioCollector.GetEarlyOutFraction();
	Vec3 contact_point_a, contact_point_b, contact_normal;
	if (epa.CastShape(inShapeCast.mCenterOfMassStart, inShapeCast.mDirection, inShapeCastSettings.mCollisionTolerance, inShapeCastSettings.mPenetrationTolerance, *cast_support, *target_support, cast_support->GetConvexRadius(), target_support->GetConvexRadius(), inShapeCastSettings.mReturnDeepestPoint, fraction, contact_point_a, contact_point_b, contact_normal)
		&& (inShapeCastSettings.mBackFaceModeConvex == EBackFaceMode::CollideWithBackFaces
			|| contact_normal.Dot(inShapeCast.mDirection) > 0.0f)) // Test if backfacing
	{
		// Convert to world space
		contact_point_a = inCenterOfMassTransform2 * contact_point_a;
		contact_point_b = inCenterOfMassTransform2 * contact_point_b;
		Vec3 contact_normal_world = inCenterOfMassTransform2.Multiply3x3(contact_normal);

		ShapeCastResult result(fraction, contact_point_a, contact_point_b, contact_normal_world, false, inSubShapeIDCreator1.GetID(), inSubShapeIDCreator2.GetID(), TransformedShape::sGetBodyID(ioCollector.GetContext()));

		// Early out if this hit is deeper than the collector's early out value
		if (fraction == 0.0f && -result.mPenetrationDepth >= ioCollector.GetEarlyOutFraction())
			return;

		// Gather faces
		if (inShapeCastSettings.mCollectFacesMode == ECollectFacesMode::CollectFaces)
		{
			// Get supporting face of shape 1
			Mat44 transform_1_to_2 = inShapeCast.mCenterOfMassStart;
			transform_1_to_2.SetTranslation(transform_1_to_2.GetTranslation() + fraction * inShapeCast.mDirection);
			cast_shape->GetSupportingFace(SubShapeID(), transform_1_to_2.Multiply3x3Transposed(-contact_normal), inShapeCast.mScale, inCenterOfMassTransform2 * transform_1_to_2, result.mShape1Face);

			// Get supporting face of shape 2
			shape->GetSupportingFace(SubShapeID(), contact_normal, inScale, inCenterOfMassTransform2, result.mShape2Face);
		}

		MOSS_IF_TRACK_NARROWPHASE_STATS(TrackNarrowPhaseCollector track;)
		ioCollector.AddHit(result);
	}
}

class ConvexShape::CSGetTrianglesContext
{
public:
				CSGetTrianglesContext(const ConvexShape *inShape, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale) :
		mLocalToWorld(Mat44::sRotationTranslation(inRotation, inPositionCOM) * Mat44::sScale(inScale)),
		mIsInsideOut(ScaleHelpers::IsInsideOut(inScale))
	{
		mSupport = inShape->GetSupportFunction(ESupportMode::IncludeConvexRadius, mSupportBuffer, Vec3::sOne());
	}

	SupportBuffer		mSupportBuffer;
	const Support *		mSupport;
	Mat44				mLocalToWorld;
	bool				mIsInsideOut;
	size_t				mCurrentVertex = 0;
};

void ConvexShape::GetTrianglesStart(GetTrianglesContext &ioContext, const AABox &inBox, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale) const
{
	static_assert(sizeof(CSGetTrianglesContext) <= sizeof(GetTrianglesContext), "GetTrianglesContext too small");
	MOSS_ASSERT(IsAligned(&ioContext, alignof(CSGetTrianglesContext)));

	new (&ioContext) CSGetTrianglesContext(this, inPositionCOM, inRotation, inScale);
}

int ConvexShape::GetTrianglesNext(GetTrianglesContext &ioContext, int inMaxTrianglesRequested, Float3 *outTriangleVertices, const PhysicsMaterial **outMaterials) const
{
	MOSS_ASSERT(inMaxTrianglesRequested >= cGetTrianglesMinTrianglesRequested);

	CSGetTrianglesContext &context = (CSGetTrianglesContext &)ioContext;

	int total_num_vertices = min(inMaxTrianglesRequested * 3, int(sUnitSphereTriangles.size() - context.mCurrentVertex));

	if (context.mIsInsideOut)
	{
		// Store triangles flipped
		for (const Vec3 *v = sUnitSphereTriangles.data() + context.mCurrentVertex, *v_end = v + total_num_vertices; v < v_end; v += 3)
		{
			(context.mLocalToWorld * context.mSupport->GetSupport(v[0])).StoreFloat3(outTriangleVertices++);
			(context.mLocalToWorld * context.mSupport->GetSupport(v[2])).StoreFloat3(outTriangleVertices++);
			(context.mLocalToWorld * context.mSupport->GetSupport(v[1])).StoreFloat3(outTriangleVertices++);
		}
	}
	else
	{
		// Store triangles
		for (const Vec3 *v = sUnitSphereTriangles.data() + context.mCurrentVertex, *v_end = v + total_num_vertices; v < v_end; v += 3)
		{
			(context.mLocalToWorld * context.mSupport->GetSupport(v[0])).StoreFloat3(outTriangleVertices++);
			(context.mLocalToWorld * context.mSupport->GetSupport(v[1])).StoreFloat3(outTriangleVertices++);
			(context.mLocalToWorld * context.mSupport->GetSupport(v[2])).StoreFloat3(outTriangleVertices++);
		}
	}

	context.mCurrentVertex += total_num_vertices;
	int total_num_triangles = total_num_vertices / 3;

	// Store materials
	if (outMaterials != nullptr)
	{
		const PhysicsMaterial *material = GetMaterial();
		for (const PhysicsMaterial **m = outMaterials, **m_end = outMaterials + total_num_triangles; m < m_end; ++m)
			*m = material;
	}

	return total_num_triangles;
}

void ConvexShape::GetSubmergedVolume(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const Plane &inSurface, float &outTotalVolume, float &outSubmergedVolume, Vec3 &outCenterOfBuoyancy MOSS_IF_DEBUG_RENDERER(, RVec3Arg inBaseOffset)) const
{
	// Calculate total volume
	Vec3 abs_scale = inScale.Abs();
	Vec3 extent = GetLocalBounds().GetExtent() * abs_scale;
	outTotalVolume = 8.0f * extent.GetX() * extent.GetY() * extent.GetZ();

	// Points of the bounding box
	Vec3 points[] =
	{
		Vec3(-1, -1, -1),
		Vec3( 1, -1, -1),
		Vec3(-1,  1, -1),
		Vec3( 1,  1, -1),
		Vec3(-1, -1,  1),
		Vec3( 1, -1,  1),
		Vec3(-1,  1,  1),
		Vec3( 1,  1,  1),
	};

	// Faces of the bounding box
	using Face = int[5];
	#define MAKE_FACE(a, b, c, d) { a, b, c, d, ((1 << a) | (1 << b) | (1 << c) | (1 << d)) } // Last int is a bit mask that indicates which indices are used
	Face faces[] =
	{
		MAKE_FACE(0, 2, 3, 1),
		MAKE_FACE(4, 6, 2, 0),
		MAKE_FACE(4, 5, 7, 6),
		MAKE_FACE(1, 3, 7, 5),
		MAKE_FACE(2, 6, 7, 3),
		MAKE_FACE(0, 1, 5, 4),
	};

	PolyhedronSubmergedVolumeCalculator::Point *buffer = (PolyhedronSubmergedVolumeCalculator::Point *)MOSS_STACK_ALLOC(8 * sizeof(PolyhedronSubmergedVolumeCalculator::Point));
	PolyhedronSubmergedVolumeCalculator submerged_vol_calc(inCenterOfMassTransform * Mat44::sScale(extent), points, sizeof(Vec3), 8, inSurface, buffer MOSS_IF_DEBUG_RENDERER(, inBaseOffset));

	if (submerged_vol_calc.AreAllAbove())
	{
		// We're above the water
		outSubmergedVolume = 0.0f;
		outCenterOfBuoyancy = Vec3::sZero();
	}
	else if (submerged_vol_calc.AreAllBelow())
	{
		// We're fully submerged
		outSubmergedVolume = outTotalVolume;
		outCenterOfBuoyancy = inCenterOfMassTransform.GetTranslation();
	}
	else
	{
		// Calculate submerged volume
		int reference_point_bit = 1 << submerged_vol_calc.GetReferencePointIdx();
		for (const Face &f : faces)
		{
			// Test if this face includes the reference point
			if ((f[4] & reference_point_bit) == 0)
			{
				// Triangulate the face (a quad)
				submerged_vol_calc.AddFace(f[0], f[1], f[2]);
				submerged_vol_calc.AddFace(f[0], f[2], f[3]);
			}
		}

		submerged_vol_calc.GetResult(outSubmergedVolume, outCenterOfBuoyancy);
	}
}

#ifndef MOSS_DEBUG_RENDERER
void ConvexShape::DrawGetSupportFunction(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, Vec3Arg inScale, ColorArg inColor, bool inDrawSupportDirection) const
{
	// Get the support function with convex radius
	SupportBuffer buffer;
	const Support *support = GetSupportFunction(ESupportMode::ExcludeConvexRadius, buffer, inScale);
	AddConvexRadius add_convex(*support, support->GetConvexRadius());

	// Draw the shape
	DebugRenderer::GeometryRef geometry = inRenderer->CreateTriangleGeometryForConvex([&add_convex](Vec3Arg inDirection) { return add_convex.GetSupport(inDirection); });
	AABox bounds = geometry->mBounds.Transformed(inCenterOfMassTransform);
	float lod_scale_sq = geometry->mBounds.GetExtent().LengthSq();
	inRenderer->DrawGeometry(inCenterOfMassTransform, bounds, lod_scale_sq, inColor, geometry);

	if (inDrawSupportDirection)
	{
		// Iterate on all directions and draw the support point and an arrow in the direction that was sampled to test if the support points make sense
		for (Vec3 v : Vec3::sUnitSphere)
		{
			Vec3 direction = 0.05f * v;
			Vec3 pos = add_convex.GetSupport(direction);
			RVec3 from = inCenterOfMassTransform * pos;
			RVec3 to = inCenterOfMassTransform * (pos + direction);
			inRenderer->DrawMarker(from, Color::sWhite, 0.001f);
			inRenderer->DrawArrow(from, to, Color::sWhite, 0.001f);
		}
	}
}

void ConvexShape::DrawGetSupportingFace(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, Vec3Arg inScale) const
{
	// Sample directions and map which faces belong to which directions
	using FaceToDirection = TMap<SupportingFace, TArray<Vec3>>;
	FaceToDirection faces;
	for (Vec3 v : Vec3::sUnitSphere)
	{
		Vec3 direction = 0.05f * v;

		SupportingFace face;
		GetSupportingFace(SubShapeID(), direction, inScale, Mat44::sIdentity(), face);

		if (!face.empty())
		{
			MOSS_ASSERT(face.size() >= 2, "The GetSupportingFace function should either return nothing or at least an edge");
			faces[face].push_back(direction);
		}
	}

	// Draw each face in a unique color and draw corresponding directions
	int color_it = 0;
	for (FaceToDirection::value_type &ftd : faces)
	{
		Color color = Color::sGetDistinctColor(color_it++);

		// Create copy of face (key in map is read only)
		SupportingFace face = ftd.first;

		// Displace the face a little bit forward so it is easier to see
		Vec3 normal = face.size() >= 3? (face[2] - face[1]).Cross(face[0] - face[1]).NormalizedOr(Vec3::sZero()) : Vec3::sZero();
		Vec3 displacement = 0.001f * normal;

		// Transform face to world space and calculate center of mass
		Vec3 com_ls = Vec3::sZero();
		for (Vec3 &v : face)
		{
			v = inCenterOfMassTransform.Multiply3x3(v + displacement);
			com_ls += v;
		}
		RVec3 com = inCenterOfMassTransform.GetTranslation() + com_ls / (float)face.size();

		// Draw the polygon and directions
		inRenderer->DrawWirePolygon(RMat44::sTranslation(inCenterOfMassTransform.GetTranslation()), face, color, face.size() >= 3? 0.001f : 0.0f);
		if (face.size() >= 3)
			inRenderer->DrawArrow(com, com + inCenterOfMassTransform.Multiply3x3(normal), color, 0.01f);
		for (Vec3 &v : ftd.second)
			inRenderer->DrawArrow(com, com + inCenterOfMassTransform.Multiply3x3(-v), color, 0.001f);
	}
}
#endif // MOSS_DEBUG_RENDERER

void ConvexShape::SaveBinaryState(StreamOut &inStream) const
{
	Shape::SaveBinaryState(inStream);

	inStream.Write(mDensity);
}

void ConvexShape::RestoreBinaryState(StreamIn &inStream)
{
	Shape::RestoreBinaryState(inStream);

	inStream.Read(mDensity);
}

void ConvexShape::SaveMaterialState(PhysicsMaterialList &outMaterials) const
{
	outMaterials.clear();
	outMaterials.push_back(mMaterial);
}

void ConvexShape::RestoreMaterialState(const PhysicsMaterialRefC *inMaterials, uint inNumMaterials)
{
	MOSS_ASSERT(inNumMaterials == 1);
	mMaterial = inMaterials[0];
}

void ConvexShape::sRegister()
{
	for (EShapeSubType s1 : sConvexSubShapeTypes)
		for (EShapeSubType s2 : sConvexSubShapeTypes)
		{
			CollisionDispatch::sRegisterCollideShape(s1, s2, sCollideConvexVsConvex);
			CollisionDispatch::sRegisterCastShape(s1, s2, sCastConvexVsConvex);
		}
}

/*													*/

ShapeSettings::ShapeResult ConvexHullShapeSettings::Create() const
{
	if (mCachedResult.IsEmpty())
		Ref<Shape> shape = new ConvexHullShape(*this, mCachedResult);
	return mCachedResult;
}

ConvexHullShape::ConvexHullShape(const ConvexHullShapeSettings &inSettings, ShapeResult &outResult) :
	ConvexShape(EShapeSubType::ConvexHull, inSettings, outResult),
	mConvexRadius(inSettings.mMaxConvexRadius)
{
	using BuilderFace = ConvexHullBuilder::Face;
	using Edge = ConvexHullBuilder::Edge;
	using Faces = TArray<BuilderFace *>;

	// Check convex radius
	if (mConvexRadius < 0.0f)
	{
		outResult.SetError("Invalid convex radius");
		return;
	}

	// Build convex hull
	const char *error = nullptr;
	ConvexHullBuilder builder(inSettings.mPoints);
	ConvexHullBuilder::EResult result = builder.Initialize(cMaxPointsInHull, inSettings.mHullTolerance, error);
	if (result != ConvexHullBuilder::EResult::Success && result != ConvexHullBuilder::EResult::MaxVerticesReached)
	{
		outResult.SetError(error);
		return;
	}
	const Faces &builder_faces = builder.GetFaces();

	// Check the consistency of the resulting hull if we fully built it
	if (result == ConvexHullBuilder::EResult::Success)
	{
		ConvexHullBuilder::Face *max_error_face;
		float max_error_distance, coplanar_distance;
		int max_error_idx;
		builder.DetermineMaxError(max_error_face, max_error_distance, max_error_idx, coplanar_distance);
		if (max_error_distance > 4.0f * max(coplanar_distance, inSettings.mHullTolerance)) // Coplanar distance could be bigger than the allowed tolerance if the points are far apart
		{
			outResult.SetError(StringFormat("Hull building failed, point %d had an error of %g (relative to tolerance: %g)", max_error_idx, (double)max_error_distance, double(max_error_distance / inSettings.mHullTolerance)));
			return;
		}
	}

	// Calculate center of mass and volume
	builder.GetCenterOfMassAndVolume(mCenterOfMass, mVolume);

	// Calculate covariance matrix
	// See:
	// - Why the inertia tensor is the inertia tensor - Jonathan Blow (http://number-none.com/blow/inertia/deriving_i.html)
	// - How to find the inertia tensor (or other mass properties) of a 3D solid body represented by a triangle mesh (Draft) - Jonathan Blow, Atman J Binstock (http://number-none.com/blow/inertia/bb_inertia.doc)
	Mat44 covariance_canonical(Vec4(1.0f / 60.0f, 1.0f / 120.0f, 1.0f / 120.0f, 0), Vec4(1.0f / 120.0f, 1.0f / 60.0f, 1.0f / 120.0f, 0), Vec4(1.0f / 120.0f, 1.0f / 120.0f, 1.0f / 60.0f, 0), Vec4(0, 0, 0, 1));
	Mat44 covariance_matrix = Mat44::sZero();
	for (BuilderFace *f : builder_faces)
	{
		// Fourth point of the tetrahedron is at the center of mass, we subtract it from the other points so we get a tetrahedron with one vertex at zero
		// The first point on the face will be used to form a triangle fan
		Edge *e = f->mFirstEdge;
		Vec3 v1 = inSettings.mPoints[e->mStartIdx] - mCenterOfMass;

		// Get the 2nd point
		e = e->mNextEdge;
		Vec3 v2 = inSettings.mPoints[e->mStartIdx] - mCenterOfMass;

		// Loop over the triangle fan
		for (e = e->mNextEdge; e != f->mFirstEdge; e = e->mNextEdge)
		{
			Vec3 v3 = inSettings.mPoints[e->mStartIdx] - mCenterOfMass;

			// Affine transform that transforms a unit tetrahedron (with vertices (0, 0, 0), (1, 0, 0), (0, 1, 0) and (0, 0, 1) to this tetrahedron
			Mat44 a(Vec4(v1, 0), Vec4(v2, 0), Vec4(v3, 0), Vec4(0, 0, 0, 1));

			// Calculate covariance matrix for this tetrahedron
			float det_a = a.GetDeterminant3x3();
			Mat44 c = det_a * (a * covariance_canonical * a.Transposed());

			// Add it
			covariance_matrix += c;

			// Prepare for next triangle
			v2 = v3;
		}
	}

	// Calculate inertia matrix assuming density is 1, note that element (3, 3) is garbage
	mInertia = Mat44::sIdentity() * (covariance_matrix(0, 0) + covariance_matrix(1, 1) + covariance_matrix(2, 2)) - covariance_matrix;

	// Convert polygons from the builder to our internal representation
	using VtxMap = TMap<int, uint8>;
	VtxMap vertex_map;
	vertex_map.reserve(VtxMap::size_type(inSettings.mPoints.size()));
	for (BuilderFace *builder_face : builder_faces)
	{
		// Determine where the vertices go
		MOSS_ASSERT(mVertexIdx.size() <= 0xFFFF);
		uint16 first_vertex = (uint16)mVertexIdx.size();
		uint16 num_vertices = 0;

		// Loop over vertices in face
		Edge *edge = builder_face->mFirstEdge;
		do
		{
			// Remap to new index, not all points in the original input set are required to form the hull
			uint8 new_idx;
			int original_idx = edge->mStartIdx;
			VtxMap::iterator m = vertex_map.find(original_idx);
			if (m != vertex_map.end())
			{
				// Found, reuse
				new_idx = m->second;
			}
			else
			{
				// This is a new point
				// Make relative to center of mass
				Vec3 p = inSettings.mPoints[original_idx] - mCenterOfMass;

				// Update local bounds
				mLocalBounds.Encapsulate(p);

				// Add to point list
				MOSS_ASSERT(mPoints.size() <= 0xff);
				new_idx = (uint8)mPoints.size();
				mPoints.push_back({ p });
				vertex_map[original_idx] = new_idx;
			}

			// Append to vertex list
			MOSS_ASSERT(mVertexIdx.size() < 0xffff);
			mVertexIdx.push_back(new_idx);
			num_vertices++;

			edge = edge->mNextEdge;
		} while (edge != builder_face->mFirstEdge);

		// Add face
		mFaces.push_back({ first_vertex, num_vertices });

		// Add plane
		Plane plane = Plane::sFromPointAndNormal(builder_face->mCentroid - mCenterOfMass, builder_face->mNormal.Normalized());
		mPlanes.push_back(plane);
	}

	// Test if GetSupportFunction can support this many points
	if (mPoints.size() > cMaxPointsInHull)
	{
		outResult.SetError(StringFormat("Internal error: Too many points in hull (%u), max allowed %d", (uint)mPoints.size(), cMaxPointsInHull));
		return;
	}

	for (int p = 0; p < (int)mPoints.size(); ++p)
	{
		// For each point, find faces that use the point
		TArray<int> faces;
		for (int f = 0; f < (int)mFaces.size(); ++f)
		{
			const Face &face = mFaces[f];
			for (int v = 0; v < face.mNumVertices; ++v)
				if (mVertexIdx[face.mFirstVertex + v] == p)
				{
					faces.push_back(f);
					break;
				}
		}

		if (faces.size() < 2)
		{
			outResult.SetError("A point must be connected to 2 or more faces!");
			return;
		}

		// Find the 3 normals that form the largest tetrahedron
		// The largest tetrahedron we can get is ((1, 0, 0) x (0, 1, 0)) . (0, 0, 1) = 1, if the volume is only 5% of that,
		// the three vectors are too coplanar and we fall back to using only 2 plane normals
		float biggest_volume = 0.05f;
		int best3[3] = { -1, -1, -1 };

		// When using 2 normals, we get the two with the biggest angle between them with a minimal difference of 1 degree
		// otherwise we fall back to just using 1 plane normal
		float smallest_dot = Cos(DegreesToRadians(1.0f));
		int best2[2] = { -1, -1 };

		for (int face1 = 0; face1 < (int)faces.size(); ++face1)
		{
			Vec3 normal1 = mPlanes[faces[face1]].GetNormal();
			for (int face2 = face1 + 1; face2 < (int)faces.size(); ++face2)
			{
				Vec3 normal2 = mPlanes[faces[face2]].GetNormal();
				Vec3 cross = normal1.Cross(normal2);

				// Determine the 2 face normals that are most apart
				float dot = normal1.Dot(normal2);
				if (dot < smallest_dot)
				{
					smallest_dot = dot;
					best2[0] = faces[face1];
					best2[1] = faces[face2];
				}

				// Determine the 3 face normals that form the largest tetrahedron
				for (int face3 = face2 + 1; face3 < (int)faces.size(); ++face3)
				{
					Vec3 normal3 = mPlanes[faces[face3]].GetNormal();
					float volume = abs(cross.Dot(normal3));
					if (volume > biggest_volume)
					{
						biggest_volume = volume;
						best3[0] = faces[face1];
						best3[1] = faces[face2];
						best3[2] = faces[face3];
					}
				}
			}
		}

		// If we didn't find 3 planes, use 2, if we didn't find 2 use 1
		if (best3[0] != -1)
			faces = { best3[0], best3[1], best3[2] };
		else if (best2[0] != -1)
			faces = { best2[0], best2[1] };
		else
			faces = { faces[0] };

		// Copy the faces to the points buffer
		Point &point = mPoints[p];
		point.mNumFaces = (int)faces.size();
		for (int i = 0; i < (int)faces.size(); ++i)
			point.mFaces[i] = faces[i];
	}

	// If the convex radius is already zero, there's no point in further reducing it
	if (mConvexRadius > 0.0f)
	{
		// Find out how thin the hull is by walking over all planes and checking the thickness of the hull in that direction
		float min_size = FLT_MAX;
		for (const Plane &plane : mPlanes)
		{
			// Take the point that is furthest away from the plane as thickness of this hull
			float max_dist = 0.0f;
			for (const Point &point : mPoints)
			{
				float dist = -plane.SignedDistance(point.mPosition); // Point is always behind plane, so we need to negate
				if (dist > max_dist)
					max_dist = dist;
			}
			min_size = min(min_size, max_dist);
		}

		// We need to fit in 2x the convex radius in min_size, so reduce the convex radius if it's bigger than that
		mConvexRadius = min(mConvexRadius, 0.5f * min_size);
	}

	// Now walk over all points and see if we have to further reduce the convex radius because of sharp edges
	if (mConvexRadius > 0.0f)
	{
		for (const Point &point : mPoints)
			if (point.mNumFaces != 1) // If we have a single face, shifting back is easy and we don't need to reduce the convex radius
			{
				// Get first two planes
				Plane p1 = mPlanes[point.mFaces[0]];
				Plane p2 = mPlanes[point.mFaces[1]];
				Plane p3;
				Vec3 offset_mask;

				if (point.mNumFaces == 3)
				{
					// Get third plane
					p3 = mPlanes[point.mFaces[2]];

					// All 3 planes will be offset by the convex radius
					offset_mask = Vec3::sReplicate(1);
				}
				else
				{
					// Third plane has normal perpendicular to the other two planes and goes through the vertex position
					MOSS_ASSERT(point.mNumFaces == 2);
					p3 = Plane::sFromPointAndNormal(point.mPosition, p1.GetNormal().Cross(p2.GetNormal()));

					// Only the first and 2nd plane will be offset, the 3rd plane is only there to guide the intersection point
					offset_mask = Vec3(1, 1, 0);
				}

				// Plane equation: point . normal + constant = 0
				// Offsetting the plane backwards with convex radius r: point . normal + constant + r = 0
				// To find the intersection 'point' of 3 planes we solve:
				// |n1x n1y n1z| |x|     | r + c1 |
				// |n2x n2y n2z| |y| = - | r + c2 | <=> n point = -r (1, 1, 1) - (c1, c2, c3)
				// |n3x n3y n3z| |z|     | r + c3 |
				// Where point = (x, y, z), n1x is the x component of the first plane, c1 = plane constant of plane 1, etc.
				// The relation between how much the intersection point shifts as a function of r is: -r * n^-1 (1, 1, 1) = r * offset
				// Where offset = -n^-1 (1, 1, 1) or -n^-1 (1, 1, 0) in case only the first 2 planes are offset
				// The error that is introduced by a convex radius r is: error = r * |offset| - r
				// So the max convex radius given error is: r = error / (|offset| - 1)
				Mat44 n = Mat44(Vec4(p1.GetNormal(), 0), Vec4(p2.GetNormal(), 0), Vec4(p3.GetNormal(), 0), Vec4(0, 0, 0, 1)).Transposed();
				float det_n = n.GetDeterminant3x3();
				if (det_n == 0.0f)
				{
					// If the determinant is zero, the matrix is not invertible so no solution exists to move the point backwards and we have to choose a convex radius of zero
					mConvexRadius = 0.0f;
					break;
				}
				Mat44 adj_n = n.Adjointed3x3();
				float offset = ((adj_n * offset_mask) / det_n).Length();
				MOSS_ASSERT(offset > 1.0f);
				float max_convex_radius = inSettings.mMaxErrorConvexRadius / (offset - 1.0f);
				mConvexRadius = min(mConvexRadius, max_convex_radius);
			}
		}

	// Calculate the inner radius by getting the minimum distance from the origin to the planes of the hull
	mInnerRadius = FLT_MAX;
	for (const Plane &p : mPlanes)
		mInnerRadius = min(mInnerRadius, -p.GetConstant());
	mInnerRadius = max(0.0f, mInnerRadius); // Clamp against zero, this should do nothing as the shape is centered around the center of mass but for flat convex hulls there may be numerical round off issues

	outResult.Set(this);
}

MassProperties ConvexHullShape::GetMassProperties() const
{
	MassProperties p;

	float density = GetDensity();

	// Calculate mass
	p.mMass = density * mVolume;

	// Calculate inertia matrix
	p.mInertia = density * mInertia;
	p.mInertia(3, 3) = 1.0f;

	return p;
}

Vec3 ConvexHullShape::GetSurfaceNormal(const SubShapeID &inSubShapeID, Vec3Arg inLocalSurfacePosition) const
{
	MOSS_ASSERT(inSubShapeID.IsEmpty(), "Invalid subshape ID");

	const Plane &first_plane = mPlanes[0];
	Vec3 best_normal = first_plane.GetNormal();
	float best_dist = abs(first_plane.SignedDistance(inLocalSurfacePosition));

	// Find the face that has the shortest distance to the surface point
	for (TArray<Face>::size_type i = 1; i < mFaces.size(); ++i)
	{
		const Plane &plane = mPlanes[i];
		Vec3 plane_normal = plane.GetNormal();
		float dist = abs(plane.SignedDistance(inLocalSurfacePosition));
		if (dist < best_dist)
		{
			best_dist = dist;
			best_normal = plane_normal;
		}
	}

	return best_normal;
}

class ConvexHullShape::HullNoConvex final : public Support
{
public:
	explicit				HullNoConvex(float inConvexRadius) :
		mConvexRadius(inConvexRadius)
	{
		static_assert(sizeof(HullNoConvex) <= sizeof(SupportBuffer), "Buffer size too small");
		MOSS_ASSERT(IsAligned(this, alignof(HullNoConvex)));
	}

	virtual Vec3			GetSupport(Vec3Arg inDirection) const override
	{
		// Find the point with the highest projection on inDirection
		float best_dot = -FLT_MAX;
		Vec3 best_point = Vec3::sZero();

		for (Vec3 point : mPoints)
		{
			// Check if its support is bigger than the current max
			float dot = point.Dot(inDirection);
			if (dot > best_dot)
			{
				best_dot = dot;
				best_point = point;
			}
		}

		return best_point;
	}

	virtual float			GetConvexRadius() const override
	{
		return mConvexRadius;
	}

	using PointsArray = TStaticArray<Vec3, cMaxPointsInHull>;

	inline PointsArray &	GetPoints()
	{
		return mPoints;
	}

	const PointsArray &		GetPoints() const
	{
		return mPoints;
	}

private:
	float					mConvexRadius;
	PointsArray				mPoints;
};

class ConvexHullShape::HullWithConvex final : public Support
{
public:
	explicit				HullWithConvex(const ConvexHullShape *inShape) :
		mShape(inShape)
	{
		static_assert(sizeof(HullWithConvex) <= sizeof(SupportBuffer), "Buffer size too small");
		MOSS_ASSERT(IsAligned(this, alignof(HullWithConvex)));
	}

	virtual Vec3			GetSupport(Vec3Arg inDirection) const override
	{
		// Find the point with the highest projection on inDirection
		float best_dot = -FLT_MAX;
		Vec3 best_point = Vec3::sZero();

		for (const Point &point : mShape->mPoints)
		{
			// Check if its support is bigger than the current max
			float dot = point.mPosition.Dot(inDirection);
			if (dot > best_dot)
			{
				best_dot = dot;
				best_point = point.mPosition;
			}
		}

		return best_point;
	}

	virtual float			GetConvexRadius() const override
	{
		return 0.0f;
	}

private:
	const ConvexHullShape *	mShape;
};

class ConvexHullShape::HullWithConvexScaled final : public Support
{
public:
							HullWithConvexScaled(const ConvexHullShape *inShape, Vec3Arg inScale) :
		mShape(inShape),
		mScale(inScale)
	{
		static_assert(sizeof(HullWithConvexScaled) <= sizeof(SupportBuffer), "Buffer size too small");
		MOSS_ASSERT(IsAligned(this, alignof(HullWithConvexScaled)));
	}

	virtual Vec3			GetSupport(Vec3Arg inDirection) const override
	{
		// Find the point with the highest projection on inDirection
		float best_dot = -FLT_MAX;
		Vec3 best_point = Vec3::sZero();

		for (const Point &point : mShape->mPoints)
		{
			// Calculate scaled position
			Vec3 pos = mScale * point.mPosition;

			// Check if its support is bigger than the current max
			float dot = pos.Dot(inDirection);
			if (dot > best_dot)
			{
				best_dot = dot;
				best_point = pos;
			}
		}

		return best_point;
	}

	virtual float			GetConvexRadius() const override
	{
		return 0.0f;
	}

private:
	const ConvexHullShape *	mShape;
	Vec3					mScale;
};

const ConvexShape::Support *ConvexHullShape::GetSupportFunction(ESupportMode inMode, SupportBuffer &inBuffer, Vec3Arg inScale) const
{
	// If there's no convex radius, we don't need to shrink the hull
	if (mConvexRadius == 0.0f)
	{
		if (ScaleHelpers::IsNotScaled(inScale))
			return new (&inBuffer) HullWithConvex(this);
		else
			return new (&inBuffer) HullWithConvexScaled(this, inScale);
	}

	switch (inMode)
	{
	case ESupportMode::IncludeConvexRadius:
	case ESupportMode::Default:
		if (ScaleHelpers::IsNotScaled(inScale))
			return new (&inBuffer) HullWithConvex(this);
		else
			return new (&inBuffer) HullWithConvexScaled(this, inScale);

	case ESupportMode::ExcludeConvexRadius:
		if (ScaleHelpers::IsNotScaled(inScale))
		{
			// Create support function
			HullNoConvex *hull = new (&inBuffer) HullNoConvex(mConvexRadius);
			HullNoConvex::PointsArray &transformed_points = hull->GetPoints();
			MOSS_ASSERT(mPoints.size() <= cMaxPointsInHull, "Not enough space, this should have been caught during shape creation!");

			for (const Point &point : mPoints)
			{
				Vec3 new_point;

				if (point.mNumFaces == 1)
				{
					// Simply shift back by the convex radius using our 1 plane
					new_point = point.mPosition - mPlanes[point.mFaces[0]].GetNormal() * mConvexRadius;
				}
				else
				{
					// Get first two planes and offset inwards by convex radius
					Plane p1 = mPlanes[point.mFaces[0]].Offset(-mConvexRadius);
					Plane p2 = mPlanes[point.mFaces[1]].Offset(-mConvexRadius);
					Plane p3;

					if (point.mNumFaces == 3)
					{
						// Get third plane and offset inwards by convex radius
						p3 = mPlanes[point.mFaces[2]].Offset(-mConvexRadius);
					}
					else
					{
						// Third plane has normal perpendicular to the other two planes and goes through the vertex position
						MOSS_ASSERT(point.mNumFaces == 2);
						p3 = Plane::sFromPointAndNormal(point.mPosition, p1.GetNormal().Cross(p2.GetNormal()));
					}

					// Find intersection point between the three planes
					if (!Plane::sIntersectPlanes(p1, p2, p3, new_point))
					{
						// Fallback: Just push point back using the first plane
						new_point = point.mPosition - p1.GetNormal() * mConvexRadius;
					}
				}

				// Add point
				transformed_points.push_back(new_point);
			}

			return hull;
		}
		else
		{
			// Calculate scaled convex radius
			float convex_radius = ScaleHelpers::ScaleConvexRadius(mConvexRadius, inScale);

			// Create new support function
			HullNoConvex *hull = new (&inBuffer) HullNoConvex(convex_radius);
			HullNoConvex::PointsArray &transformed_points = hull->GetPoints();
			MOSS_ASSERT(mPoints.size() <= cMaxPointsInHull, "Not enough space, this should have been caught during shape creation!");

			// Precalculate inverse scale
			Vec3 inv_scale = inScale.Reciprocal();

			for (const Point &point : mPoints)
			{
				// Calculate scaled position
				Vec3 pos = inScale * point.mPosition;

				// Transform normals for plane 1 with scale
				Vec3 n1 = (inv_scale * mPlanes[point.mFaces[0]].GetNormal()).Normalized();

				Vec3 new_point;

				if (point.mNumFaces == 1)
				{
					// Simply shift back by the convex radius using our 1 plane
					new_point = pos - n1 * convex_radius;
				}
				else
				{
					// Transform normals for plane 2 with scale
					Vec3 n2 = (inv_scale * mPlanes[point.mFaces[1]].GetNormal()).Normalized();

					// Get first two planes and offset inwards by convex radius
					Plane p1 = Plane::sFromPointAndNormal(pos, n1).Offset(-convex_radius);
					Plane p2 = Plane::sFromPointAndNormal(pos, n2).Offset(-convex_radius);
					Plane p3;

					if (point.mNumFaces == 3)
					{
						// Transform last normal with scale
						Vec3 n3 = (inv_scale * mPlanes[point.mFaces[2]].GetNormal()).Normalized();

						// Get third plane and offset inwards by convex radius
						p3 = Plane::sFromPointAndNormal(pos, n3).Offset(-convex_radius);
					}
					else
					{
						// Third plane has normal perpendicular to the other two planes and goes through the vertex position
						MOSS_ASSERT(point.mNumFaces == 2);
						p3 = Plane::sFromPointAndNormal(pos, n1.Cross(n2));
					}

					// Find intersection point between the three planes
					if (!Plane::sIntersectPlanes(p1, p2, p3, new_point))
					{
						// Fallback: Just push point back using the first plane
						new_point = pos - n1 * convex_radius;
					}
				}

				// Add point
				transformed_points.push_back(new_point);
			}

			return hull;
		}
	}

	MOSS_ASSERT(false);
	return nullptr;
}

void ConvexHullShape::GetSupportingFace(const SubShapeID &inSubShapeID, Vec3Arg inDirection, Vec3Arg inScale, Mat44Arg inCenterOfMassTransform, SupportingFace &outVertices) const
{
	MOSS_ASSERT(inSubShapeID.IsEmpty(), "Invalid subshape ID");

	Vec3 inv_scale = inScale.Reciprocal();

	// Need to transform the plane normals using inScale
	// Transforming a direction with matrix M is done through multiplying by (M^-1)^T
	// In this case M is a diagonal matrix with the scale vector, so we need to multiply our normal by 1 / scale and renormalize afterwards
	Vec3 plane0_normal = inv_scale * mPlanes[0].GetNormal();
	float best_dot = plane0_normal.Dot(inDirection) / plane0_normal.Length();
	int best_face_idx = 0;

	for (TArray<Plane>::size_type i = 1; i < mPlanes.size(); ++i)
	{
		Vec3 plane_normal = inv_scale * mPlanes[i].GetNormal();
		float dot = plane_normal.Dot(inDirection) / plane_normal.Length();
		if (dot < best_dot)
		{
			best_dot = dot;
			best_face_idx = (int)i;
		}
	}

	// Get vertices
	const Face &best_face = mFaces[best_face_idx];
	const uint8 *first_vtx = mVertexIdx.data() + best_face.mFirstVertex;
	const uint8 *end_vtx = first_vtx + best_face.mNumVertices;

	// If we have more than 1/2 the capacity of outVertices worth of vertices, we start skipping vertices (note we can't fill the buffer completely since extra edges will be generated by clipping).
	// TODO: This really needs a better algorithm to determine which vertices are important!
	int max_vertices_to_return = outVertices.capacity() / 2;
	int delta_vtx = (int(best_face.mNumVertices) + max_vertices_to_return) / max_vertices_to_return;

	// Calculate transform with scale
	Mat44 transform = inCenterOfMassTransform.PreScaled(inScale);

	if (ScaleHelpers::IsInsideOut(inScale))
	{
		// Flip winding of supporting face
		for (const uint8 *v = end_vtx - 1; v >= first_vtx; v -= delta_vtx)
			outVertices.push_back(transform * mPoints[*v].mPosition);
	}
	else
	{
		// Normal winding of supporting face
		for (const uint8 *v = first_vtx; v < end_vtx; v += delta_vtx)
			outVertices.push_back(transform * mPoints[*v].mPosition);
	}
}

void ConvexHullShape::GetSubmergedVolume(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const Plane &inSurface, float &outTotalVolume, float &outSubmergedVolume, Vec3 &outCenterOfBuoyancy MOSS_IF_DEBUG_RENDERER(, RVec3Arg inBaseOffset)) const
{
	// Trivially calculate total volume
	Vec3 abs_scale = inScale.Abs();
	outTotalVolume = mVolume * abs_scale.GetX() * abs_scale.GetY() * abs_scale.GetZ();

	// Check if shape has been scaled inside out
	bool is_inside_out = ScaleHelpers::IsInsideOut(inScale);

	// Convert the points to world space and determine the distance to the surface
	int num_points = int(mPoints.size());
	PolyhedronSubmergedVolumeCalculator::Point *buffer = (PolyhedronSubmergedVolumeCalculator::Point *)MOSS_STACK_ALLOC(num_points * sizeof(PolyhedronSubmergedVolumeCalculator::Point));
	PolyhedronSubmergedVolumeCalculator submerged_vol_calc(inCenterOfMassTransform * Mat44::sScale(inScale), &mPoints[0].mPosition, sizeof(Point), num_points, inSurface, buffer MOSS_IF_DEBUG_RENDERER(, inBaseOffset));

	if (submerged_vol_calc.AreAllAbove())
	{
		// We're above the water
		outSubmergedVolume = 0.0f;
		outCenterOfBuoyancy = Vec3::sZero();
	}
	else if (submerged_vol_calc.AreAllBelow())
	{
		// We're fully submerged
		outSubmergedVolume = outTotalVolume;
		outCenterOfBuoyancy = inCenterOfMassTransform.GetTranslation();
	}
	else
	{
		// Calculate submerged volume
		int reference_point_idx = submerged_vol_calc.GetReferencePointIdx();
		for (const Face &f : mFaces)
		{
			const uint8 *first_vtx = mVertexIdx.data() + f.mFirstVertex;
			const uint8 *end_vtx = first_vtx + f.mNumVertices;

			// If any of the vertices of this face are the reference point, the volume will be zero so we can skip this face
			bool degenerate = false;
			for (const uint8 *v = first_vtx; v < end_vtx; ++v)
				if (*v == reference_point_idx)
				{
					degenerate = true;
					break;
				}
			if (degenerate)
				continue;

			// Triangulate the face
			int i1 = *first_vtx;
			if (is_inside_out)
			{
				// Reverse winding
				for (const uint8 *v = first_vtx + 2; v < end_vtx; ++v)
				{
					int i2 = *(v - 1);
					int i3 = *v;
					submerged_vol_calc.AddFace(i1, i3, i2);
				}
			}
			else
			{
				// Normal winding
				for (const uint8 *v = first_vtx + 2; v < end_vtx; ++v)
				{
					int i2 = *(v - 1);
					int i3 = *v;
					submerged_vol_calc.AddFace(i1, i2, i3);
				}
			}
		}

		// Get the results
		submerged_vol_calc.GetResult(outSubmergedVolume, outCenterOfBuoyancy);
	}

#ifndef MOSS_DEBUG_RENDERER
	// Draw center of buoyancy
	if (sDrawSubmergedVolumes)
		DebugRenderer::sInstance->DrawWireSphere(inBaseOffset + outCenterOfBuoyancy, 0.05f, Color::sRed, 1);
#endif // MOSS_DEBUG_RENDERER
}

#ifndef MOSS_DEBUG_RENDERER
void ConvexHullShape::Draw(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, Vec3Arg inScale, ColorArg inColor, bool inUseMaterialColors, bool inDrawWireframe) const
{
	if (mGeometry == nullptr)
	{
		TArray<DebugRenderer::Triangle> triangles;
		for (const Face &f : mFaces)
		{
			const uint8 *first_vtx = mVertexIdx.data() + f.mFirstVertex;
			const uint8 *end_vtx = first_vtx + f.mNumVertices;

			// Draw first triangle of polygon
			Vec3 v0 = mPoints[first_vtx[0]].mPosition;
			Vec3 v1 = mPoints[first_vtx[1]].mPosition;
			Vec3 v2 = mPoints[first_vtx[2]].mPosition;
			Vec3 uv_direction = (v1 - v0).Normalized();
			triangles.push_back({ v0, v1, v2, Color::sWhite, v0, uv_direction });

			// Draw any other triangles in this polygon
			for (const uint8 *v = first_vtx + 3; v < end_vtx; ++v)
				triangles.push_back({ v0, mPoints[*(v - 1)].mPosition, mPoints[*v].mPosition, Color::sWhite, v0, uv_direction });
		}
		mGeometry = new DebugRenderer::Geometry(inRenderer->CreateTriangleBatch(triangles), GetLocalBounds());
	}

	// Test if the shape is scaled inside out
	DebugRenderer::ECullMode cull_mode = ScaleHelpers::IsInsideOut(inScale)? DebugRenderer::ECullMode::CullFrontFace : DebugRenderer::ECullMode::CullBackFace;

	// Determine the draw mode
	DebugRenderer::EDrawMode draw_mode = inDrawWireframe? DebugRenderer::EDrawMode::Wireframe : DebugRenderer::EDrawMode::Solid;

	// Draw the geometry
	Color color = inUseMaterialColors? GetMaterial()->GetDebugColor() : inColor;
	RMat44 transform = inCenterOfMassTransform.PreScaled(inScale);
	inRenderer->DrawGeometry(transform, color, mGeometry, cull_mode, DebugRenderer::ECastShadow::On, draw_mode);

	// Draw the outline if requested
	if (sDrawFaceOutlines)
		for (const Face &f : mFaces)
		{
			const uint8 *first_vtx = mVertexIdx.data() + f.mFirstVertex;
			const uint8 *end_vtx = first_vtx + f.mNumVertices;

			// Draw edges of face
			inRenderer->DrawLine(transform * mPoints[*(end_vtx - 1)].mPosition, transform * mPoints[*first_vtx].mPosition, Color::sGrey);
			for (const uint8 *v = first_vtx + 1; v < end_vtx; ++v)
				inRenderer->DrawLine(transform * mPoints[*(v - 1)].mPosition, transform * mPoints[*v].mPosition, Color::sGrey);
		}
}

void ConvexHullShape::DrawShrunkShape(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, Vec3Arg inScale) const
{
	// Get the shrunk points
	SupportBuffer buffer;
	const HullNoConvex *support = mConvexRadius > 0.0f? static_cast<const HullNoConvex *>(GetSupportFunction(ESupportMode::ExcludeConvexRadius, buffer, inScale)) : nullptr;

	RMat44 transform = inCenterOfMassTransform * Mat44::sScale(inScale);

	for (int p = 0; p < (int)mPoints.size(); ++p)
	{
		const Point &point = mPoints[p];
		RVec3 position = transform * point.mPosition;
		RVec3 shrunk_point = support != nullptr? transform * support->GetPoints()[p] : position;

		// Draw difference between shrunk position and position
		inRenderer->DrawLine(position, shrunk_point, Color::sGreen);

		// Draw face normals that are contributing
		for (int i = 0; i < point.mNumFaces; ++i)
			inRenderer->DrawLine(position, position + 0.1f * mPlanes[point.mFaces[i]].GetNormal(), Color::sYellow);

		// Draw point index
		inRenderer->DrawText3D(position, ConvertToString(p), Color::sWhite, 0.1f);
	}
}
#endif // MOSS_DEBUG_RENDERER

bool ConvexHullShape::CastRayHelper(const RayCast &inRay, float &outMinFraction, float &outMaxFraction) const
{
	if (mFaces.size() == 2)
	{
		// If we have only 2 faces, we're a flat convex hull and we need to test edges instead of planes

		// Check if plane is parallel to ray
		const Plane &p = mPlanes.front();
		Vec3 plane_normal = p.GetNormal();
		float direction_projection = inRay.mDirection.Dot(plane_normal);
		if (abs(direction_projection) >= 1.0e-12f)
		{
			// Calculate intersection point
			float distance_to_plane = inRay.mOrigin.Dot(plane_normal) + p.GetConstant();
			float fraction = -distance_to_plane / direction_projection;
			if (fraction < 0.0f || fraction > 1.0f)
			{
				// Does not hit plane, no hit
				outMinFraction = 0.0f;
				outMaxFraction = 1.0f + FLT_EPSILON;
				return false;
			}
			Vec3 intersection_point = inRay.mOrigin + fraction * inRay.mDirection;

			// Test all edges to see if point is inside polygon
			const Face &f = mFaces.front();
			const uint8 *first_vtx = mVertexIdx.data() + f.mFirstVertex;
			const uint8 *end_vtx = first_vtx + f.mNumVertices;
			Vec3 p1 = mPoints[*end_vtx].mPosition;
			for (const uint8 *v = first_vtx; v < end_vtx; ++v)
			{
				Vec3 p2 = mPoints[*v].mPosition;
				if ((p2 - p1).Cross(intersection_point - p1).Dot(plane_normal) < 0.0f)
				{
					// Outside polygon, no hit
					outMinFraction = 0.0f;
					outMaxFraction = 1.0f + FLT_EPSILON;
					return false;
				}
				p1 = p2;
			}

			// Inside polygon, a hit
			outMinFraction = fraction;
			outMaxFraction = fraction;
			return true;
		}
		else
		{
			// Parallel ray doesn't hit
			outMinFraction = 0.0f;
			outMaxFraction = 1.0f + FLT_EPSILON;
			return false;
		}
	}
	else
	{
		// Clip ray against all planes
		int fractions_set = 0;
		bool all_inside = true;
		float min_fraction = 0.0f, max_fraction = 1.0f + FLT_EPSILON;
		for (const Plane &p : mPlanes)
		{
			// Check if the ray origin is behind this plane
			Vec3 plane_normal = p.GetNormal();
			float distance_to_plane = inRay.mOrigin.Dot(plane_normal) + p.GetConstant();
			bool is_outside = distance_to_plane > 0.0f;
			all_inside &= !is_outside;

			// Check if plane is parallel to ray
			float direction_projection = inRay.mDirection.Dot(plane_normal);
			if (abs(direction_projection) >= 1.0e-12f)
			{
				// Get intersection fraction between ray and plane
				float fraction = -distance_to_plane / direction_projection;

				// Update interval of ray that is inside the hull
				if (direction_projection < 0.0f)
				{
					min_fraction = max(fraction, min_fraction);
					fractions_set |= 1;
				}
				else
				{
					max_fraction = min(fraction, max_fraction);
					fractions_set |= 2;
				}
			}
			else if (is_outside)
				return false; // Outside the plane and parallel, no hit!
		}

		// Test if both min and max have been set
		if (fractions_set == 3)
		{
			// Output fractions
			outMinFraction = min_fraction;
			outMaxFraction = max_fraction;

			// Test if the infinite ray intersects with the hull (the length will be checked later)
			return min_fraction <= max_fraction && max_fraction >= 0.0f;
		}
		else
		{
			// Degenerate case, either the ray is parallel to all planes or the ray has zero length
			outMinFraction = 0.0f;
			outMaxFraction = 1.0f + FLT_EPSILON;

			// Return if the origin is inside the hull
			return all_inside;
		}
	}
}

bool ConvexHullShape::CastRay(const RayCast &inRay, const SubShapeIDCreator &inSubShapeIDCreator, RayCastResult &ioHit) const
{
	// Determine if ray hits the shape
	float min_fraction, max_fraction;
	if (CastRayHelper(inRay, min_fraction, max_fraction)
		&& min_fraction < ioHit.mFraction) // Check if this is a closer hit
	{
		// Better hit than the current hit
		ioHit.mFraction = min_fraction;
		ioHit.mSubShapeID2 = inSubShapeIDCreator.GetID();
		return true;
	}
	return false;
}

void ConvexHullShape::CastRay(const RayCast &inRay, const RayCastSettings &inRayCastSettings, const SubShapeIDCreator &inSubShapeIDCreator, CastRayCollector &ioCollector, const ShapeFilter &inShapeFilter) const
{
	// Test shape filter
	if (!inShapeFilter.ShouldCollide(this, inSubShapeIDCreator.GetID()))
		return;

	// Determine if ray hits the shape
	float min_fraction, max_fraction;
	if (CastRayHelper(inRay, min_fraction, max_fraction)
		&& min_fraction < ioCollector.GetEarlyOutFraction()) // Check if this is closer than the early out fraction
	{
		// Better hit than the current hit
		RayCastResult hit;
		hit.mBodyID = TransformedShape::sGetBodyID(ioCollector.GetContext());
		hit.mSubShapeID2 = inSubShapeIDCreator.GetID();

		// Check front side hit
		if (inRayCastSettings.mTreatConvexAsSolid || min_fraction > 0.0f)
		{
			hit.mFraction = min_fraction;
			ioCollector.AddHit(hit);
		}

		// Check back side hit
		if (inRayCastSettings.mBackFaceModeConvex == EBackFaceMode::CollideWithBackFaces
			&& max_fraction < ioCollector.GetEarlyOutFraction())
		{
			hit.mFraction = max_fraction;
			ioCollector.AddHit(hit);
		}
	}
}

void ConvexHullShape::CollidePoint(Vec3Arg inPoint, const SubShapeIDCreator &inSubShapeIDCreator, CollidePointCollector &ioCollector, const ShapeFilter &inShapeFilter) const
{
	// Test shape filter
	if (!inShapeFilter.ShouldCollide(this, inSubShapeIDCreator.GetID()))
		return;

	// Check if point is behind all planes
	for (const Plane &p : mPlanes)
		if (p.SignedDistance(inPoint) > 0.0f)
			return;

	// Point is inside
	ioCollector.AddHit({ TransformedShape::sGetBodyID(ioCollector.GetContext()), inSubShapeIDCreator.GetID() });
}

void ConvexHullShape::CollideSoftBodyVertices(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const CollideSoftBodyVertexIterator &inVertices, uint inNumVertices, int inCollidingShapeIndex) const
{
	Mat44 inverse_transform = inCenterOfMassTransform.InversedRotationTranslation();

	Vec3 inv_scale = inScale.Reciprocal();
	bool is_not_scaled = ScaleHelpers::IsNotScaled(inScale);
	float scale_flip = ScaleHelpers::IsInsideOut(inScale)? -1.0f : 1.0f;

	for (CollideSoftBodyVertexIterator v = inVertices, sbv_end = inVertices + inNumVertices; v != sbv_end; ++v)
		if (v.GetInvMass() > 0.0f)
		{
			Vec3 local_pos = inverse_transform * v.GetPosition();

			// Find most facing plane
			float max_distance = -FLT_MAX;
			Vec3 max_plane_normal = Vec3::sZero();
			uint max_plane_idx = 0;
			if (is_not_scaled)
			{
				// Without scale, it is trivial to calculate the distance to the hull
				for (const Plane &p : mPlanes)
				{
					float distance = p.SignedDistance(local_pos);
					if (distance > max_distance)
					{
						max_distance = distance;
						max_plane_normal = p.GetNormal();
						max_plane_idx = uint(&p - mPlanes.data());
					}
				}
			}
			else
			{
				// When there's scale we need to calculate the planes first
				for (uint i = 0; i < (uint)mPlanes.size(); ++i)
				{
					// Calculate plane normal and point by scaling the original plane
					Vec3 plane_normal = (inv_scale * mPlanes[i].GetNormal()).Normalized();
					Vec3 plane_point = inScale * mPoints[mVertexIdx[mFaces[i].mFirstVertex]].mPosition;

					float distance = plane_normal.Dot(local_pos - plane_point);
					if (distance > max_distance)
					{
						max_distance = distance;
						max_plane_normal = plane_normal;
						max_plane_idx = i;
					}
				}
			}
			bool is_outside = max_distance > 0.0f;

			// Project point onto that plane
			Vec3 closest_point = local_pos - max_distance * max_plane_normal;

			// Check edges if we're outside the hull (when inside we know the closest face is also the closest point to the surface)
			if (is_outside)
			{
				// Loop over edges
				float closest_point_dist_sq = FLT_MAX;
				const Face &face = mFaces[max_plane_idx];
				for (const uint8 *v_start = &mVertexIdx[face.mFirstVertex], *v1 = v_start, *v_end = v_start + face.mNumVertices; v1 < v_end; ++v1)
				{
					// Find second point
					const uint8 *v2 = v1 + 1;
					if (v2 == v_end)
						v2 = v_start;

					// Get edge points
					Vec3 p1 = inScale * mPoints[*v1].mPosition;
					Vec3 p2 = inScale * mPoints[*v2].mPosition;

					// Check if the position is outside the edge (if not, the face will be closer)
					Vec3 edge_normal = (p2 - p1).Cross(max_plane_normal);
					if (scale_flip * edge_normal.Dot(local_pos - p1) > 0.0f)
					{
						// Get closest point on edge
						uint32 set;
						Vec3 closest = ClosestPoint::GetClosestPointOnLine(p1 - local_pos, p2 - local_pos, set);
						float distance_sq = closest.LengthSq();
						if (distance_sq < closest_point_dist_sq)
							closest_point = local_pos + closest;
					}
				}
			}

			// Check if this is the largest penetration
			Vec3 normal = local_pos - closest_point;
			float normal_length = normal.Length();
			float penetration = normal_length;
			if (is_outside)
				penetration = -penetration;
			else
				normal = -normal;
			if (v.UpdatePenetration(penetration))
			{
				// Calculate contact plane
				normal = normal_length > 0.0f? normal / normal_length : max_plane_normal;
				Plane plane = Plane::sFromPointAndNormal(closest_point, normal);

				// Store collision
				v.SetCollision(plane.GetTransformed(inCenterOfMassTransform), inCollidingShapeIndex);
			}
		}
}

class ConvexHullShape::CHSGetTrianglesContext
{
public:
				CHSGetTrianglesContext(Mat44Arg inTransform, bool inIsInsideOut) : mTransform(inTransform), mIsInsideOut(inIsInsideOut) { }

	Mat44		mTransform;
	bool		mIsInsideOut;
	size_t		mCurrentFace = 0;
};

void ConvexHullShape::GetTrianglesStart(GetTrianglesContext &ioContext, const AABox &inBox, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale) const
{
	static_assert(sizeof(CHSGetTrianglesContext) <= sizeof(GetTrianglesContext), "GetTrianglesContext too small");
	MOSS_ASSERT(IsAligned(&ioContext, alignof(CHSGetTrianglesContext)));

	new (&ioContext) CHSGetTrianglesContext(Mat44::sRotationTranslation(inRotation, inPositionCOM) * Mat44::sScale(inScale), ScaleHelpers::IsInsideOut(inScale));
}

int ConvexHullShape::GetTrianglesNext(GetTrianglesContext &ioContext, int inMaxTrianglesRequested, Float3 *outTriangleVertices, const PhysicsMaterial **outMaterials) const
{
	static_assert(cGetTrianglesMinTrianglesRequested >= 12, "cGetTrianglesMinTrianglesRequested is too small");
	MOSS_ASSERT(inMaxTrianglesRequested >= cGetTrianglesMinTrianglesRequested);

	CHSGetTrianglesContext &context = (CHSGetTrianglesContext &)ioContext;

	int total_num_triangles = 0;
	for (; context.mCurrentFace < mFaces.size(); ++context.mCurrentFace)
	{
		const Face &f = mFaces[context.mCurrentFace];

		const uint8 *first_vtx = mVertexIdx.data() + f.mFirstVertex;
		const uint8 *end_vtx = first_vtx + f.mNumVertices;

		// Check if there is still room in the output buffer for this face
		int num_triangles = f.mNumVertices - 2;
		inMaxTrianglesRequested -= num_triangles;
		if (inMaxTrianglesRequested < 0)
			break;
		total_num_triangles += num_triangles;

		// Get first triangle of polygon
		Vec3 v0 = context.mTransform * mPoints[first_vtx[0]].mPosition;
		Vec3 v1 = context.mTransform * mPoints[first_vtx[1]].mPosition;
		Vec3 v2 = context.mTransform * mPoints[first_vtx[2]].mPosition;
		v0.StoreFloat3(outTriangleVertices++);
		if (context.mIsInsideOut)
		{
			// Store first triangle in this polygon flipped
			v2.StoreFloat3(outTriangleVertices++);
			v1.StoreFloat3(outTriangleVertices++);

			// Store other triangles in this polygon flipped
			for (const uint8 *v = first_vtx + 3; v < end_vtx; ++v)
			{
				v0.StoreFloat3(outTriangleVertices++);
				(context.mTransform * mPoints[*v].mPosition).StoreFloat3(outTriangleVertices++);
				(context.mTransform * mPoints[*(v - 1)].mPosition).StoreFloat3(outTriangleVertices++);
			}
		}
		else
		{
			// Store first triangle in this polygon
			v1.StoreFloat3(outTriangleVertices++);
			v2.StoreFloat3(outTriangleVertices++);

			// Store other triangles in this polygon
			for (const uint8 *v = first_vtx + 3; v < end_vtx; ++v)
			{
				v0.StoreFloat3(outTriangleVertices++);
				(context.mTransform * mPoints[*(v - 1)].mPosition).StoreFloat3(outTriangleVertices++);
				(context.mTransform * mPoints[*v].mPosition).StoreFloat3(outTriangleVertices++);
			}
		}
	}

	// Store materials
	if (outMaterials != nullptr)
	{
		const PhysicsMaterial *material = GetMaterial();
		for (const PhysicsMaterial **m = outMaterials, **m_end = outMaterials + total_num_triangles; m < m_end; ++m)
			*m = material;
	}

	return total_num_triangles;
}

void ConvexHullShape::SaveBinaryState(StreamOut &inStream) const
{
	ConvexShape::SaveBinaryState(inStream);

	inStream.Write(mCenterOfMass);
	inStream.Write(mInertia);
	inStream.Write(mLocalBounds.mMin);
	inStream.Write(mLocalBounds.mMax);
	inStream.Write(mPoints);
	inStream.Write(mFaces);
	inStream.Write(mPlanes);
	inStream.Write(mVertexIdx);
	inStream.Write(mConvexRadius);
	inStream.Write(mVolume);
	inStream.Write(mInnerRadius);
}

void ConvexHullShape::RestoreBinaryState(StreamIn &inStream)
{
	ConvexShape::RestoreBinaryState(inStream);

	inStream.Read(mCenterOfMass);
	inStream.Read(mInertia);
	inStream.Read(mLocalBounds.mMin);
	inStream.Read(mLocalBounds.mMax);
	inStream.Read(mPoints);
	inStream.Read(mFaces);
	inStream.Read(mPlanes);
	inStream.Read(mVertexIdx);
	inStream.Read(mConvexRadius);
	inStream.Read(mVolume);
	inStream.Read(mInnerRadius);
}

Shape::Stats ConvexHullShape::GetStats() const
{
	// Count number of triangles
	uint triangle_count = 0;
	for (const Face &f : mFaces)
		triangle_count += f.mNumVertices - 2;

	return Stats(
		sizeof(*this)
			+ mPoints.size() * sizeof(Point)
			+ mFaces.size() * sizeof(Face)
			+ mPlanes.size() * sizeof(Plane)
			+ mVertexIdx.size() * sizeof(uint8),
		triangle_count);
}

void ConvexHullShape::sRegister()
{
	ShapeFunctions &f = ShapeFunctions::sGet(EShapeSubType::ConvexHull);
	f.mConstruct = []() -> Shape * { return new ConvexHullShape; };
	f.mColor = Color::sGreen;
}

/*													*/

ShapeSettings::ShapeResult CylinderShapeSettings::Create() const
{
	if (mCachedResult.IsEmpty())
		Ref<Shape> shape = new CylinderShape(*this, mCachedResult);
	return mCachedResult;
}

CylinderShape::CylinderShape(const CylinderShapeSettings &inSettings, ShapeResult &outResult) :
	ConvexShape(EShapeSubType::Cylinder, inSettings, outResult),
	mHalfHeight(inSettings.mHalfHeight),
	mRadius(inSettings.mRadius),
	mConvexRadius(inSettings.mConvexRadius)
{
	if (inSettings.mHalfHeight < inSettings.mConvexRadius)
	{
		outResult.SetError("Invalid height");
		return;
	}

	if (inSettings.mRadius < inSettings.mConvexRadius)
	{
		outResult.SetError("Invalid radius");
		return;
	}

	if (inSettings.mConvexRadius < 0.0f)
	{
		outResult.SetError("Invalid convex radius");
		return;
	}

	outResult.Set(this);
}

CylinderShape::CylinderShape(float inHalfHeight, float inRadius, float inConvexRadius, const PhysicsMaterial *inMaterial) :
	ConvexShape(EShapeSubType::Cylinder, inMaterial),
	mHalfHeight(inHalfHeight),
	mRadius(inRadius),
	mConvexRadius(inConvexRadius)
{
	MOSS_ASSERT(inHalfHeight >= inConvexRadius);
	MOSS_ASSERT(inRadius >= inConvexRadius);
	MOSS_ASSERT(inConvexRadius >= 0.0f);
}

class CylinderShape::Cylinder final : public Support
{
public:
					Cylinder(float inHalfHeight, float inRadius, float inConvexRadius) :
		mHalfHeight(inHalfHeight),
		mRadius(inRadius),
		mConvexRadius(inConvexRadius)
	{
		static_assert(sizeof(Cylinder) <= sizeof(SupportBuffer), "Buffer size too small");
		MOSS_ASSERT(IsAligned(this, alignof(Cylinder)));
	}

	virtual Vec3	GetSupport(Vec3Arg inDirection) const override
	{
		// Support mapping, taken from:
		// A Fast and Robust GJK Implementation for Collision Detection of Convex Objects - Gino van den Bergen
		// page 8
		float x = inDirection.GetX(), y = inDirection.GetY(), z = inDirection.GetZ();
		float o = sqrt(Square(x) + Square(z));
		if (o > 0.0f)
			return Vec3((mRadius * x) / o, Sign(y) * mHalfHeight, (mRadius * z) / o);
		else
			return Vec3(0, Sign(y) * mHalfHeight, 0);
	}

	virtual float	GetConvexRadius() const override
	{
		return mConvexRadius;
	}

private:
	float			mHalfHeight;
	float			mRadius;
	float			mConvexRadius;
};

const ConvexShape::Support *CylinderShape::GetSupportFunction(ESupportMode inMode, SupportBuffer &inBuffer, Vec3Arg inScale) const
{
	MOSS_ASSERT(IsValidScale(inScale));

	// Get scaled cylinder
	Vec3 abs_scale = inScale.Abs();
	float scale_xz = abs_scale.GetX();
	float scale_y = abs_scale.GetY();
	float scaled_half_height = scale_y * mHalfHeight;
	float scaled_radius = scale_xz * mRadius;
	float scaled_convex_radius = ScaleHelpers::ScaleConvexRadius(mConvexRadius, inScale);

	switch (inMode)
	{
	case ESupportMode::IncludeConvexRadius:
	case ESupportMode::Default:
		return new (&inBuffer) Cylinder(scaled_half_height, scaled_radius, 0.0f);

	case ESupportMode::ExcludeConvexRadius:
		return new (&inBuffer) Cylinder(scaled_half_height - scaled_convex_radius, scaled_radius - scaled_convex_radius, scaled_convex_radius);
	}

	MOSS_ASSERT(false);
	return nullptr;
}

void CylinderShape::GetSupportingFace(const SubShapeID &inSubShapeID, Vec3Arg inDirection, Vec3Arg inScale, Mat44Arg inCenterOfMassTransform, SupportingFace &outVertices) const
{
	MOSS_ASSERT(inSubShapeID.IsEmpty(), "Invalid subshape ID");
	MOSS_ASSERT(IsValidScale(inScale));

	// Get scaled cylinder
	Vec3 abs_scale = inScale.Abs();
	float scale_xz = abs_scale.GetX();
	float scale_y = abs_scale.GetY();
	float scaled_half_height = scale_y * mHalfHeight;
	float scaled_radius = scale_xz * mRadius;

	float x = inDirection.GetX(), y = inDirection.GetY(), z = inDirection.GetZ();
	float xz_sq = Square(x) + Square(z);
	float y_sq = Square(y);

	// Check which component is bigger
	if (xz_sq > y_sq)
	{
		// Hitting side
		float f = -scaled_radius / sqrt(xz_sq);
		float vx = x * f;
		float vz = z * f;
		outVertices.push_back(inCenterOfMassTransform * Vec3(vx, scaled_half_height, vz));
		outVertices.push_back(inCenterOfMassTransform * Vec3(vx, -scaled_half_height, vz));
	}
	else
	{
		// Hitting top or bottom

		// When the inDirection is more than 5 degrees from vertical, align the vertices so that 1 of the vertices
		// points towards inDirection in the XZ plane. This ensures that we always have a vertex towards max penetration depth.
		Mat44 transform = inCenterOfMassTransform;
		if (xz_sq > 0.00765427f * y_sq)
		{
			Vec4 base_x = Vec4(x, 0, z, 0) / sqrt(xz_sq);
			Vec4 base_z = base_x.Swizzle<SWIZZLE_Z, SWIZZLE_Y, SWIZZLE_X, SWIZZLE_W>() * Vec4(-1, 0, 1, 0);
			transform = transform * Mat44(base_x, Vec4(0, 1, 0, 0), base_z, Vec4(0, 0, 0, 1));
		}

		// Adjust for scale and height
		Vec3 multiplier = y < 0.0f? Vec3(scaled_radius, scaled_half_height, scaled_radius) : Vec3(-scaled_radius, -scaled_half_height, scaled_radius);
		transform = transform.PreScaled(multiplier);

		for (const Vec3 &v : cCylinderTopFace)
			outVertices.push_back(transform * v);
	}
}

MassProperties CylinderShape::GetMassProperties() const
{
	MassProperties p;

	// Mass is surface of circle * height
	float radius_sq = Square(mRadius);
	float height = 2.0f * mHalfHeight;
	p.mMass = MOSS_PI * radius_sq * height * GetDensity();

	// Inertia according to https://en.wikipedia.org/wiki/List_of_moments_of_inertia:
	float inertia_y = radius_sq * p.mMass * 0.5f;
	float inertia_x = inertia_y * 0.5f + p.mMass * height * height / 12.0f;
	float inertia_z = inertia_x;

	// Set inertia
	p.mInertia = Mat44::sScale(Vec3(inertia_x, inertia_y, inertia_z));

	return p;
}

Vec3 CylinderShape::GetSurfaceNormal(const SubShapeID &inSubShapeID, Vec3Arg inLocalSurfacePosition) const
{
	MOSS_ASSERT(inSubShapeID.IsEmpty(), "Invalid subshape ID");

	// Calculate distance to infinite cylinder surface
	Vec3 local_surface_position_xz(inLocalSurfacePosition.GetX(), 0, inLocalSurfacePosition.GetZ());
	float local_surface_position_xz_len = local_surface_position_xz.Length();
	float distance_to_curved_surface = abs(local_surface_position_xz_len - mRadius);

	// Calculate distance to top or bottom plane
	float distance_to_top_or_bottom = abs(abs(inLocalSurfacePosition.GetY()) - mHalfHeight);

	// Return normal according to closest surface
	if (distance_to_curved_surface < distance_to_top_or_bottom)
		return local_surface_position_xz / local_surface_position_xz_len;
	else
		return inLocalSurfacePosition.GetY() > 0.0f? Vec3::sAxisY() : -Vec3::sAxisY();
}

AABox CylinderShape::GetLocalBounds() const
{
	Vec3 extent = Vec3(mRadius, mHalfHeight, mRadius);
	return AABox(-extent, extent);
}

#ifndef MOSS_DEBUG_RENDERER
void CylinderShape::Draw(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, Vec3Arg inScale, ColorArg inColor, bool inUseMaterialColors, bool inDrawWireframe) const
{
	DebugRenderer::EDrawMode draw_mode = inDrawWireframe? DebugRenderer::EDrawMode::Wireframe : DebugRenderer::EDrawMode::Solid;
	inRenderer->DrawCylinder(inCenterOfMassTransform * Mat44::sScale(inScale.Abs()), mHalfHeight, mRadius, inUseMaterialColors? GetMaterial()->GetDebugColor() : inColor, DebugRenderer::ECastShadow::On, draw_mode);
}
#endif // MOSS_DEBUG_RENDERER

bool CylinderShape::CastRay(const RayCast &inRay, const SubShapeIDCreator &inSubShapeIDCreator, RayCastResult &ioHit) const
{
	// Test ray against capsule
	float fraction = RayCylinder(inRay.mOrigin, inRay.mDirection, mHalfHeight, mRadius);
	if (fraction < ioHit.mFraction)
	{
		ioHit.mFraction = fraction;
		ioHit.mSubShapeID2 = inSubShapeIDCreator.GetID();
		return true;
	}
	return false;
}

void CylinderShape::CollidePoint(Vec3Arg inPoint, const SubShapeIDCreator &inSubShapeIDCreator, CollidePointCollector &ioCollector, const ShapeFilter &inShapeFilter) const
{
	// Test shape filter
	if (!inShapeFilter.ShouldCollide(this, inSubShapeIDCreator.GetID()))
		return;

	// Check if the point is in the cylinder
	if (abs(inPoint.GetY()) <= mHalfHeight											// Within the height
		&& Square(inPoint.GetX()) + Square(inPoint.GetZ()) <= Square(mRadius))		// Within the radius
		ioCollector.AddHit({ TransformedShape::sGetBodyID(ioCollector.GetContext()), inSubShapeIDCreator.GetID() });
}

void CylinderShape::CollideSoftBodyVertices(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const CollideSoftBodyVertexIterator &inVertices, uint inNumVertices, int inCollidingShapeIndex) const
{
	MOSS_ASSERT(IsValidScale(inScale));

	Mat44 inverse_transform = inCenterOfMassTransform.InversedRotationTranslation();

	// Get scaled cylinder
	Vec3 abs_scale = inScale.Abs();
	float half_height = abs_scale.GetY() * mHalfHeight;
	float radius = abs_scale.GetX() * mRadius;

	for (CollideSoftBodyVertexIterator v = inVertices, sbv_end = inVertices + inNumVertices; v != sbv_end; ++v)
		if (v.GetInvMass() > 0.0f)
		{
			Vec3 local_pos = inverse_transform * v.GetPosition();

			// Calculate penetration into side surface
			Vec3 side_normal = local_pos;
			side_normal.SetY(0.0f);
			float side_normal_length = side_normal.Length();
			float side_penetration = radius - side_normal_length;

			// Calculate penetration into top or bottom plane
			float top_penetration = half_height - abs(local_pos.GetY());

			Vec3 point, normal;
			if (side_penetration < 0.0f && top_penetration < 0.0f)
			{
				// We're outside the cylinder height and radius
				point = side_normal * (radius / side_normal_length) + Vec3(0, half_height * Sign(local_pos.GetY()), 0);
				normal = (local_pos - point).NormalizedOr(Vec3::sAxisY());
			}
			else if (side_penetration < top_penetration)
			{
				// Side surface is closest
				normal = side_normal_length > 0.0f? side_normal / side_normal_length : Vec3::sAxisX();
				point = radius * normal;
			}
			else
			{
				// Top or bottom plane is closest
				normal = Vec3(0, Sign(local_pos.GetY()), 0);
				point = half_height * normal;
			}

			// Calculate penetration
			Plane plane = Plane::sFromPointAndNormal(point, normal);
			float penetration = -plane.SignedDistance(local_pos);
			if (v.UpdatePenetration(penetration))
				v.SetCollision(plane.GetTransformed(inCenterOfMassTransform), inCollidingShapeIndex);
		}
}

void CylinderShape::GetTrianglesStart(GetTrianglesContext &ioContext, const AABox &inBox, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale) const
{
	Mat44 unit_cylinder_transform(Vec4(mRadius, 0, 0, 0), Vec4(0, mHalfHeight, 0, 0), Vec4(0, 0, mRadius, 0), Vec4(0, 0, 0, 1));
	new (&ioContext) GetTrianglesContextVertexList(inPositionCOM, inRotation, inScale, unit_cylinder_transform, sUnitCylinderTriangles.data(), sUnitCylinderTriangles.size(), GetMaterial());
}

int CylinderShape::GetTrianglesNext(GetTrianglesContext &ioContext, int inMaxTrianglesRequested, Float3 *outTriangleVertices, const PhysicsMaterial **outMaterials) const
{
	return ((GetTrianglesContextVertexList &)ioContext).GetTrianglesNext(inMaxTrianglesRequested, outTriangleVertices, outMaterials);
}

void CylinderShape::SaveBinaryState(StreamOut &inStream) const
{
	ConvexShape::SaveBinaryState(inStream);

	inStream.Write(mHalfHeight);
	inStream.Write(mRadius);
	inStream.Write(mConvexRadius);
}

void CylinderShape::RestoreBinaryState(StreamIn &inStream)
{
	ConvexShape::RestoreBinaryState(inStream);

	inStream.Read(mHalfHeight);
	inStream.Read(mRadius);
	inStream.Read(mConvexRadius);
}

bool CylinderShape::IsValidScale(Vec3Arg inScale) const
{
	return ConvexShape::IsValidScale(inScale) && ScaleHelpers::IsUniformScaleXZ(inScale.Abs());
}

Vec3 CylinderShape::MakeScaleValid(Vec3Arg inScale) const
{
	Vec3 scale = ScaleHelpers::MakeNonZeroScale(inScale);

	return scale.GetSign() * ScaleHelpers::MakeUniformScaleXZ(scale.Abs());
}

void CylinderShape::sRegister()
{
	ShapeFunctions &f = ShapeFunctions::sGet(EShapeSubType::Cylinder);
	f.mConstruct = []() -> Shape * { return new CylinderShape; };
	f.mColor = Color::sGreen;
}

/*													*/

DecoratedShape::DecoratedShape(EShapeSubType inSubType, const DecoratedShapeSettings &inSettings, ShapeResult &outResult) :
	Shape(EShapeType::Decorated, inSubType, inSettings, outResult)
{
	// Check that there's a shape
	if (inSettings.mInnerShape == nullptr && inSettings.mInnerShapePtr == nullptr)
	{
		outResult.SetError("Inner shape is null!");
		return;
	}

	if (inSettings.mInnerShapePtr != nullptr)
	{
		// Use provided shape
		mInnerShape = inSettings.mInnerShapePtr;
	}
	else
	{
		// Create child shape
		ShapeResult child_result = inSettings.mInnerShape->Create();
		if (!child_result.IsValid())
		{
			outResult = child_result;
			return;
		}
		mInnerShape = child_result.Get();
	}
}

const PhysicsMaterial *DecoratedShape::GetMaterial(const SubShapeID &inSubShapeID) const
{
	return mInnerShape->GetMaterial(inSubShapeID);
}

void DecoratedShape::GetSupportingFace(const SubShapeID &inSubShapeID, Vec3Arg inDirection, Vec3Arg inScale, Mat44Arg inCenterOfMassTransform, SupportingFace &outVertices) const
{
	mInnerShape->GetSupportingFace(inSubShapeID, inDirection, inScale, inCenterOfMassTransform, outVertices);
}

uint64 DecoratedShape::GetSubShapeUserData(const SubShapeID &inSubShapeID) const
{
	return mInnerShape->GetSubShapeUserData(inSubShapeID);
}

void DecoratedShape::SaveSubShapeState(ShapeList &outSubShapes) const
{
	outSubShapes.clear();
	outSubShapes.push_back(mInnerShape);
}

void DecoratedShape::RestoreSubShapeState(const ShapeRefC *inSubShapes, uint inNumShapes)
{
	MOSS_ASSERT(inNumShapes == 1);
	mInnerShape = inSubShapes[0];
}

Shape::Stats DecoratedShape::GetStatsRecursive(VisitedShapes &ioVisitedShapes) const
{
	// Get own stats
	Stats stats = Shape::GetStatsRecursive(ioVisitedShapes);

	// Add child stats
	Stats child_stats = mInnerShape->GetStatsRecursive(ioVisitedShapes);
	stats.mSizeBytes += child_stats.mSizeBytes;
	stats.mNumTriangles += child_stats.mNumTriangles;

	return stats;
}

/*													*/

ShapeSettings::ShapeResult EmptyShapeSettings::Create() const
{
	if (mCachedResult.IsEmpty())
		new EmptyShape(*this, mCachedResult);

	return mCachedResult;
}

MassProperties EmptyShape::GetMassProperties() const
{
	MassProperties mass_properties;
	mass_properties.mMass = 1.0f;
	mass_properties.mInertia = Mat44::sIdentity();
	return mass_properties;
}

#ifndef MOSS_DEBUG_RENDERER
void EmptyShape::Draw(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, Vec3Arg inScale, ColorArg inColor, [[maybe_unused]] bool inUseMaterialColors, [[maybe_unused]] bool inDrawWireframe) const
{
	inRenderer->DrawMarker(inCenterOfMassTransform.GetTranslation(), inColor, abs(inScale.GetX()) * 0.1f);
}
#endif // MOSS_DEBUG_RENDERER

void EmptyShape::sRegister()
{
	ShapeFunctions &f = ShapeFunctions::sGet(EShapeSubType::Empty);
	f.mConstruct = []() -> Shape * { return new EmptyShape; };
	f.mColor = Color::sBlack;

	auto collide_empty = []([[maybe_unused]] const Shape *inShape1, [[maybe_unused]] const Shape *inShape2, [[maybe_unused]] Vec3Arg inScale1, [[maybe_unused]] Vec3Arg inScale2, [[maybe_unused]] Mat44Arg inCenterOfMassTransform1, [[maybe_unused]] Mat44Arg inCenterOfMassTransform2, [[maybe_unused]] const SubShapeIDCreator &inSubShapeIDCreator1, [[maybe_unused]] const SubShapeIDCreator &inSubShapeIDCreator2, [[maybe_unused]] const CollideShapeSettings &inCollideShapeSettings, [[maybe_unused]] CollideShapeCollector &ioCollector, [[maybe_unused]] const ShapeFilter &inShapeFilter) { /* Do Nothing */ };
	auto cast_empty = []([[maybe_unused]] const ShapeCast &inShapeCast, [[maybe_unused]] const ShapeCastSettings &inShapeCastSettings, [[maybe_unused]] const Shape *inShape, [[maybe_unused]] Vec3Arg inScale, [[maybe_unused]] const ShapeFilter &inShapeFilter, [[maybe_unused]] Mat44Arg inCenterOfMassTransform2, [[maybe_unused]] const SubShapeIDCreator &inSubShapeIDCreator1, [[maybe_unused]] const SubShapeIDCreator &inSubShapeIDCreator2, [[maybe_unused]] CastShapeCollector &ioCollector) { /* Do nothing */ };

	for (const EShapeSubType s : sAllSubShapeTypes)
	{
		CollisionDispatch::sRegisterCollideShape(EShapeSubType::Empty, s, collide_empty);
		CollisionDispatch::sRegisterCollideShape(s, EShapeSubType::Empty, collide_empty);

		CollisionDispatch::sRegisterCastShape(EShapeSubType::Empty, s, cast_empty);
		CollisionDispatch::sRegisterCastShape(s, EShapeSubType::Empty, cast_empty);
	}
}

/*													*/

HeightFieldShapeSettings::HeightFieldShapeSettings(const float *inSamples, Vec3Arg inOffset, Vec3Arg inScale, uint32 inSampleCount, const uint8 *inMaterialIndices, const PhysicsMaterialList &inMaterialList) :
	mOffset(inOffset),
	mScale(inScale),
	mSampleCount(inSampleCount)
{
	mHeightSamples.assign(inSamples, inSamples + Square(inSampleCount));

	if (!inMaterialList.empty() && inMaterialIndices != nullptr)
	{
		mMaterialIndices.assign(inMaterialIndices, inMaterialIndices + Square(inSampleCount - 1));
		mMaterials = inMaterialList;
	}
	else
	{
		MOSS_ASSERT(inMaterialList.empty());
		MOSS_ASSERT(inMaterialIndices == nullptr);
	}
}

ShapeSettings::ShapeResult HeightFieldShapeSettings::Create() const
{
	if (mCachedResult.IsEmpty())
		Ref<Shape> shape = new HeightFieldShape(*this, mCachedResult);
	return mCachedResult;
}

void HeightFieldShapeSettings::DetermineMinAndMaxSample(float &outMinValue, float &outMaxValue, float &outQuantizationScale) const
{
	// Determine min and max value
	outMinValue = mMinHeightValue;
	outMaxValue = mMaxHeightValue;
	for (float h : mHeightSamples)
		if (h != cNoCollisionValue)
		{
			outMinValue = min(outMinValue, h);
			outMaxValue = max(outMaxValue, h);
		}

	// Prevent dividing by zero by setting a minimal height difference
	float height_diff = max(outMaxValue - outMinValue, 1.0e-6f);

	// Calculate the scale factor to quantize to 16 bits
	outQuantizationScale = float(cMaxHeightValue16) / height_diff;
}

uint32 HeightFieldShapeSettings::CalculateBitsPerSampleForError(float inMaxError) const
{
	// Start with 1 bit per sample
	uint32 bits_per_sample = 1;

	// Determine total range
	float min_value, max_value, scale;
	DetermineMinAndMaxSample(min_value, max_value, scale);
	if (min_value < max_value)
	{
		// Loop over all blocks
		for (uint y = 0; y < mSampleCount; y += mBlockSize)
			for (uint x = 0; x < mSampleCount; x += mBlockSize)
			{
				// Determine min and max block value + take 1 sample border just like we do while building the hierarchical grids
				float block_min_value = FLT_MAX, block_max_value = -FLT_MAX;
				for (uint bx = x; bx < min(x + mBlockSize + 1, mSampleCount); ++bx)
					for (uint by = y; by < min(y + mBlockSize + 1, mSampleCount); ++by)
					{
						float h = mHeightSamples[by * mSampleCount + bx];
						if (h != cNoCollisionValue)
						{
							block_min_value = min(block_min_value, h);
							block_max_value = max(block_max_value, h);
						}
					}

				if (block_min_value < block_max_value)
				{
					// Quantize then dequantize block min/max value
					block_min_value = min_value + floor((block_min_value - min_value) * scale) / scale;
					block_max_value = min_value + ceil((block_max_value - min_value) * scale) / scale;
					float block_height = block_max_value - block_min_value;

					// Loop over the block again
					for (uint bx = x; bx < x + mBlockSize; ++bx)
						for (uint by = y; by < y + mBlockSize; ++by)
						{
							// Get the height
							float height = mHeightSamples[by * mSampleCount + bx];
							if (height != cNoCollisionValue)
							{
								for (;;)
								{
									// Determine bitmask for sample
									uint32 sample_mask = (1 << bits_per_sample) - 1;

									// Quantize
									float quantized_height = floor((height - block_min_value) * float(sample_mask) / block_height);
									quantized_height = Clamp(quantized_height, 0.0f, float(sample_mask - 1));

									// Dequantize and check error
									float dequantized_height = block_min_value + (quantized_height + 0.5f) * block_height / float(sample_mask);
									if (abs(dequantized_height - height) <= inMaxError)
										break;

									// Not accurate enough, increase bits per sample
									bits_per_sample++;

									// Don't go above 8 bits per sample
									if (bits_per_sample == 8)
										return bits_per_sample;
								}
							}
						}
				}
			}

	}

	return bits_per_sample;
}

void HeightFieldShape::CalculateActiveEdges(uint inX, uint inY, uint inSizeX, uint inSizeY, const float *inHeights, uint inHeightsStartX, uint inHeightsStartY, intptr_t inHeightsStride, float inHeightsScale, float inActiveEdgeCosThresholdAngle, TempAllocator &inAllocator)
{
	// Limit the block size so we don't allocate more than 64K memory from the temp allocator
	uint block_size_x = min(inSizeX, 44u);
	uint block_size_y = min(inSizeY, 44u);

	// Allocate temporary buffer for normals
	uint normals_size = 2 * (block_size_x + 1) * (block_size_y + 1) * sizeof(Vec3);
	Vec3 *normals = (Vec3 *)inAllocator.Allocate(normals_size);
	MOSS_SCOPE_EXIT([&inAllocator, normals, normals_size]{ inAllocator.Free(normals, normals_size); });

	// Update the edges in blocks
	for (uint block_y = 0; block_y < inSizeY; block_y += block_size_y)
		for (uint block_x = 0; block_x < inSizeX; block_x += block_size_x)
		{
			// Calculate the bottom right corner of the block
			uint block_x_end = min(block_x + block_size_x, inSizeX);
			uint block_y_end = min(block_y + block_size_y, inSizeY);

			// If we're not at the first block in x, we need one extra column of normals to the left
			uint normals_x_start, normals_x_skip;
			if (block_x > 0)
			{
				normals_x_start = block_x - 1;
				normals_x_skip = 2; // We need to skip over that extra column
			}
			else
			{
				normals_x_start = 0;
				normals_x_skip = 0;
			}

			// If we're not at the last block in y, we need one extra row of normals at the bottom
			uint normals_y_end = block_y_end < inSizeY? block_y_end + 1 : inSizeY;

			// Calculate triangle normals and make normals zero for triangles that are missing
			Vec3 *out_normal = normals;
			for (uint y = block_y; y < normals_y_end; ++y)
			{
				for (uint x = normals_x_start; x < block_x_end; ++x)
				{
					// Get height on diagonal
					const float *height_samples = inHeights + (inY - inHeightsStartY + y) * inHeightsStride + (inX - inHeightsStartX + x);
					float x1y1_h = height_samples[0];
					float x2y2_h = height_samples[inHeightsStride + 1];
					if (x1y1_h != cNoCollisionValue && x2y2_h != cNoCollisionValue)
					{
						// Calculate normal for lower left triangle (e.g. T1A)
						float x1y2_h = height_samples[inHeightsStride];
						if (x1y2_h != cNoCollisionValue)
						{
							Vec3 x2y2_minus_x1y2(mScale.GetX(), inHeightsScale * (x2y2_h - x1y2_h), 0);
							Vec3 x1y1_minus_x1y2(0, inHeightsScale * (x1y1_h - x1y2_h), -mScale.GetZ());
							out_normal[0] = x2y2_minus_x1y2.Cross(x1y1_minus_x1y2).Normalized();
						}
						else
							out_normal[0] = Vec3::sZero();

						// Calculate normal for upper right triangle (e.g. T1B)
						float x2y1_h = height_samples[1];
						if (x2y1_h != cNoCollisionValue)
						{
							Vec3 x1y1_minus_x2y1(-mScale.GetX(), inHeightsScale * (x1y1_h - x2y1_h), 0);
							Vec3 x2y2_minus_x2y1(0, inHeightsScale * (x2y2_h - x2y1_h), mScale.GetZ());
							out_normal[1] = x1y1_minus_x2y1.Cross(x2y2_minus_x2y1).Normalized();
						}
						else
							out_normal[1] = Vec3::sZero();
					}
					else
					{
						out_normal[0] = Vec3::sZero();
						out_normal[1] = Vec3::sZero();
					}

					out_normal += 2;
				}
			}

			// Number of vectors to skip to get to the next row of normals
			uint normals_pitch = 2 * (block_x_end - normals_x_start);

			// Calculate active edges
			const Vec3 *in_normal = normals;
			uint global_bit_pos = 3 * ((inY + block_y) * (mSampleCount - 1) + (inX + block_x));
			for (uint y = block_y; y < block_y_end; ++y)
			{
				in_normal += normals_x_skip; // If we have an extra column to the left, skip it here, we'll read it with in_normal[-1] below

				for (uint x = block_x; x < block_x_end; ++x)
				{
					// Get vertex heights
					const float *height_samples = inHeights + (inY - inHeightsStartY + y) * inHeightsStride + (inX - inHeightsStartX + x);
					float x1y1_h = height_samples[0];
					float x1y2_h = height_samples[inHeightsStride];
					float x2y2_h = height_samples[inHeightsStride + 1];
					bool x1y1_valid = x1y1_h != cNoCollisionValue;
					bool x1y2_valid = x1y2_h != cNoCollisionValue;
					bool x2y2_valid = x2y2_h != cNoCollisionValue;

					// Calculate the edge flags (3 bits)
					// See diagram in the next function for the edge numbering
					uint16 edge_mask = 0b111;
					uint16 edge_flags = 0;

					// Edge 0
					if (x == 0)
						edge_mask &= 0b110; // We need normal x - 1 which we didn't calculate, don't update this edge
					else if (x1y1_valid && x1y2_valid)
					{
						Vec3 edge0_direction(0, inHeightsScale * (x1y2_h - x1y1_h), mScale.GetZ());
						if (ActiveEdges::IsEdgeActive(in_normal[0], in_normal[-1], edge0_direction, inActiveEdgeCosThresholdAngle))
							edge_flags |= 0b001;
					}

					// Edge 1
					if (y == inSizeY - 1)
						edge_mask &= 0b101; // We need normal y + 1 which we didn't calculate, don't update this edge
					else if (x1y2_valid && x2y2_valid)
					{
						Vec3 edge1_direction(mScale.GetX(), inHeightsScale * (x2y2_h - x1y2_h), 0);
						if (ActiveEdges::IsEdgeActive(in_normal[0], in_normal[normals_pitch + 1], edge1_direction, inActiveEdgeCosThresholdAngle))
							edge_flags |= 0b010;
					}

					// Edge 2
					if (x1y1_valid && x2y2_valid)
					{
						Vec3 edge2_direction(-mScale.GetX(), inHeightsScale * (x1y1_h - x2y2_h), -mScale.GetZ());
						if (ActiveEdges::IsEdgeActive(in_normal[0], in_normal[1], edge2_direction, inActiveEdgeCosThresholdAngle))
							edge_flags |= 0b100;
					}

					// Store the edge flags in the array
					uint byte_pos = global_bit_pos >> 3;
					uint bit_pos = global_bit_pos & 0b111;
					MOSS_ASSERT(byte_pos < mActiveEdgesSize);
					uint8 *edge_flags_ptr = &mActiveEdges[byte_pos];
					uint16 combined_edge_flags = uint16(edge_flags_ptr[0]) | uint16(uint16(edge_flags_ptr[1]) << 8);
					combined_edge_flags &= ~(edge_mask << bit_pos);
					combined_edge_flags |= edge_flags << bit_pos;
					edge_flags_ptr[0] = uint8(combined_edge_flags);
					edge_flags_ptr[1] = uint8(combined_edge_flags >> 8);

					in_normal += 2;
					global_bit_pos += 3;
				}

				global_bit_pos += 3 * (mSampleCount - 1 - (block_x_end - block_x));
			}
		}
}

void HeightFieldShape::CalculateActiveEdges(const HeightFieldShapeSettings &inSettings)
{
	/*
		Store active edges. The triangles are organized like this:
			x --->

		y   +       +
			| \ T1B | \ T2B
		|  e0   e2  |   \
		|   | T1A \ | T2A \
		V   +--e1---+-------+
			| \ T3B | \ T4B
			|   \   |   \
			| T3A \ | T4A \
			+-------+-------+
		We store active edges e0 .. e2 as bits 0 .. 2.
		We store triangles horizontally then vertically (order T1A, T2A, T3A and T4A).
		The top edge and right edge of the heightfield are always active so we do not need to store them,
		therefore we only need to store (mSampleCount - 1)^2 * 3-bit
		The triangles T1B, T2B, T3B and T4B do not need to be stored, their active edges can be constructed from adjacent triangles.
		Add 1 byte padding so we can always read 1 uint16 to get the bits that cross an 8 bit boundary
	*/

	// Make all edges active (if mSampleCount is bigger than inSettings.mSampleCount we need to fill up the padding,
	// also edges at x = 0 and y = inSettings.mSampleCount - 1 are not updated)
	memset(mActiveEdges, 0xff, mActiveEdgesSize);

	// Now clear the edges that are not active
	TempAllocatorMalloc allocator;
	CalculateActiveEdges(0, 0, inSettings.mSampleCount - 1, inSettings.mSampleCount - 1, inSettings.mHeightSamples.data(), 0, 0, inSettings.mSampleCount, inSettings.mScale.GetY(), inSettings.mActiveEdgeCosThresholdAngle, allocator);
}

void HeightFieldShape::StoreMaterialIndices(const HeightFieldShapeSettings &inSettings)
{
	// We need to account for any rounding of the sample count to the nearest block size
	uint in_count_min_1 = inSettings.mSampleCount - 1;
	uint out_count_min_1 = mSampleCount - 1;

	mNumBitsPerMaterialIndex = 32 - CountLeadingZeros(max((uint32)mMaterials.size(), inSettings.mMaterialsCapacity) - 1);
	mMaterialIndices.resize(((Square(out_count_min_1) * mNumBitsPerMaterialIndex + 7) >> 3) + 1, 0); // Add 1 byte so we don't read out of bounds when reading an uint16

	if (mMaterials.size() > 1)
		for (uint y = 0; y < out_count_min_1; ++y)
			for (uint x = 0; x < out_count_min_1; ++x)
			{
				// Read material
				uint16 material_index = x < in_count_min_1 && y < in_count_min_1? uint16(inSettings.mMaterialIndices[x + y * in_count_min_1]) : 0;

				// Calculate byte and bit position where the material index needs to go
				uint sample_pos = x + y * out_count_min_1;
				uint bit_pos = sample_pos * mNumBitsPerMaterialIndex;
				uint byte_pos = bit_pos >> 3;
				bit_pos &= 0b111;

				// Write the material index
				material_index <<= bit_pos;
				MOSS_ASSERT(byte_pos + 1 < mMaterialIndices.size());
				mMaterialIndices[byte_pos] |= uint8(material_index);
				mMaterialIndices[byte_pos + 1] |= uint8(material_index >> 8);
			}
}

void HeightFieldShape::CacheValues()
{
	mSampleMask = uint8((uint32(1) << mBitsPerSample) - 1);
}

void HeightFieldShape::AllocateBuffers()
{
	uint num_blocks = GetNumBlocks();
	uint max_stride = (num_blocks + 1) >> 1;
	mRangeBlocksSize = sGridOffsets[sGetMaxLevel(num_blocks) - 1] + Square(max_stride);
	mHeightSamplesSize = (mSampleCount * mSampleCount * mBitsPerSample + 7) / 8 + 1;
	mActiveEdgesSize = (Square(mSampleCount - 1) * 3 + 7) / 8 + 1; // See explanation at HeightFieldShape::CalculateActiveEdges

	MOSS_ASSERT(mRangeBlocks == nullptr && mHeightSamples == nullptr && mActiveEdges == nullptr);
	void *data = AlignedAllocate(mRangeBlocksSize * sizeof(RangeBlock) + mHeightSamplesSize + mActiveEdgesSize, alignof(RangeBlock));
	mRangeBlocks = reinterpret_cast<RangeBlock *>(data);
	mHeightSamples = reinterpret_cast<uint8 *>(mRangeBlocks + mRangeBlocksSize);
	mActiveEdges = mHeightSamples + mHeightSamplesSize;
}

HeightFieldShape::HeightFieldShape(const HeightFieldShapeSettings &inSettings, ShapeResult &outResult) :
	Shape(EShapeType::HeightField, EShapeSubType::HeightField, inSettings, outResult),
	mOffset(inSettings.mOffset),
	mScale(inSettings.mScale),
	mSampleCount(((inSettings.mSampleCount + inSettings.mBlockSize - 1) / inSettings.mBlockSize) * inSettings.mBlockSize), // Round sample count to nearest block size
	mBlockSize(inSettings.mBlockSize),
	mBitsPerSample(uint8(inSettings.mBitsPerSample))
{
	CacheValues();

	// Reserve a bigger materials list if requested
	if (inSettings.mMaterialsCapacity > 0)
		mMaterials.reserve(inSettings.mMaterialsCapacity);
	mMaterials = inSettings.mMaterials;

	// Check block size
	if (mBlockSize < 2 || mBlockSize > 8)
	{
		outResult.SetError("HeightFieldShape: Block size must be in the range [2, 8]!");
		return;
	}

	// Check bits per sample
	if (inSettings.mBitsPerSample < 1 || inSettings.mBitsPerSample > 8)
	{
		outResult.SetError("HeightFieldShape: Bits per sample must be in the range [1, 8]!");
		return;
	}

	// We stop at mBlockSize x mBlockSize height sample blocks
	uint num_blocks = GetNumBlocks();

	// We want at least 1 grid layer
	if (num_blocks < 2)
	{
		outResult.SetError("HeightFieldShape: Sample count too low!");
		return;
	}

	// Check that we don't overflow our 32 bit 'properties'
	if (num_blocks > (1 << cNumBitsXY))
	{
		outResult.SetError("HeightFieldShape: Sample count too high!");
		return;
	}

	// Check if we're not exceeding the amount of sub shape id bits
	if (GetSubShapeIDBitsRecursive() > SubShapeID::MaxBits)
	{
		outResult.SetError("HeightFieldShape: Size exceeds the amount of available sub shape ID bits!");
		return;
	}

	if (!mMaterials.empty())
	{
		// Validate materials
		if (mMaterials.size() > 256)
		{
			outResult.SetError("Supporting max 256 materials per height field");
			return;
		}
		for (uint8 s : inSettings.mMaterialIndices)
			if (s >= mMaterials.size())
			{
				outResult.SetError(StringFormat("Material %u is beyond material list (size: %u)", s, (uint)mMaterials.size()));
				return;
			}
	}
	else
	{
		// No materials assigned, validate that no materials have been specified
		if (!inSettings.mMaterialIndices.empty())
		{
			outResult.SetError("No materials present, mMaterialIndices should be empty");
			return;
		}
	}

	// Determine range
	float min_value, max_value, scale;
	inSettings.DetermineMinAndMaxSample(min_value, max_value, scale);
	if (min_value > max_value)
	{
		// If there is no collision with this heightmap, leave everything empty
		mMaterials.clear();
		outResult.Set(this);
		return;
	}

	// Allocate space for this shape
	AllocateBuffers();

	// Quantize to uint16
	TArray<uint16> quantized_samples;
	quantized_samples.reserve(mSampleCount * mSampleCount);
	for (uint y = 0; y < inSettings.mSampleCount; ++y)
	{
		for (uint x = 0; x < inSettings.mSampleCount; ++x)
		{
			float h = inSettings.mHeightSamples[x + y * inSettings.mSampleCount];
			if (h == cNoCollisionValue)
			{
				quantized_samples.push_back(cNoCollisionValue16);
			}
			else
			{
				// Floor the quantized height to get a lower bound for the quantized value
				int quantized_height = (int)floor(scale * (h - min_value));

				// Ensure that the height says below the max height value so we can safely add 1 to get the upper bound for the quantized value
				quantized_height = Clamp(quantized_height, 0, int(cMaxHeightValue16 - 1));

				quantized_samples.push_back(uint16(quantized_height));
			}
		}
		// Pad remaining columns with no collision
		for (uint x = inSettings.mSampleCount; x < mSampleCount; ++x)
			quantized_samples.push_back(cNoCollisionValue16);
	}
	// Pad remaining rows with no collision
	for (uint y = inSettings.mSampleCount; y < mSampleCount; ++y)
		for (uint x = 0; x < mSampleCount; ++x)
			quantized_samples.push_back(cNoCollisionValue16);

	// Update offset and scale to account for the compression to uint16
	if (min_value <= max_value) // Only when there was collision
	{
		// In GetPosition we always add 0.5 to the quantized sample in order to reduce the average error.
		// We want to be able to exactly quantize min_value (this is important in case the heightfield is entirely flat) so we subtract that value from min_value.
		min_value -= 0.5f / (scale * mSampleMask);

		mOffset.SetY(mOffset.GetY() + mScale.GetY() * min_value);
	}
	mScale.SetY(mScale.GetY() / scale);

	// Calculate amount of grids
	uint max_level = sGetMaxLevel(num_blocks);

	// Temporary data structure used during creating of a hierarchy of grids
	struct Range
	{
		uint16	mMin;
		uint16	mMax;
	};

	// Reserve size for temporary range data + reserve 1 extra for a 1x1 grid that we won't store but use for calculating the bounding box
	TArray<TArray<Range>> ranges;
	ranges.resize(max_level + 1);

	// Calculate highest detail grid by combining mBlockSize x mBlockSize height samples
	TArray<Range> *cur_range_vector = &ranges.back();
	uint num_blocks_pow2 = GetNextPowerOf2(num_blocks); // We calculate the range blocks as if the heightfield was a power of 2, when we save the range blocks we'll ignore the extra samples (this makes downsampling easier)
	cur_range_vector->resize(num_blocks_pow2 * num_blocks_pow2);
	Range *range_dst = &cur_range_vector->front();
	for (uint y = 0; y < num_blocks_pow2; ++y)
		for (uint x = 0; x < num_blocks_pow2; ++x)
		{
			range_dst->mMin = 0xffff;
			range_dst->mMax = 0;
			uint max_bx = x == num_blocks_pow2 - 1? mBlockSize : mBlockSize + 1; // for interior blocks take 1 more because the triangles connect to the next block so we must include their height too
			uint max_by = y == num_blocks_pow2 - 1? mBlockSize : mBlockSize + 1;
			for (uint by = 0; by < max_by; ++by)
				for (uint bx = 0; bx < max_bx; ++bx)
				{
					uint sx = x * mBlockSize + bx;
					uint sy = y * mBlockSize + by;
					if (sx < mSampleCount && sy < mSampleCount)
					{
						uint16 h = quantized_samples[sy * mSampleCount + sx];
						if (h != cNoCollisionValue16)
						{
							range_dst->mMin = min(range_dst->mMin, h);
							range_dst->mMax = max(range_dst->mMax, uint16(h + 1)); // Add 1 to the max so we know the real value is between mMin and mMax
						}
					}
				}
			++range_dst;
		}

	// Calculate remaining grids
	for (uint n = num_blocks_pow2 >> 1; n >= 1; n >>= 1)
	{
		// Get source buffer
		const Range *range_src = &cur_range_vector->front();

		// Previous array element
		--cur_range_vector;

		// Make space for this grid
		cur_range_vector->resize(n * n);

		// Get target buffer
		range_dst = &cur_range_vector->front();

		// Combine the results of 2x2 ranges
		for (uint y = 0; y < n; ++y)
			for (uint x = 0; x < n; ++x)
			{
				range_dst->mMin = 0xffff;
				range_dst->mMax = 0;
				for (uint by = 0; by < 2; ++by)
					for (uint bx = 0; bx < 2; ++bx)
					{
						const Range &r = range_src[(y * 2 + by) * n * 2 + x * 2 + bx];
						range_dst->mMin = min(range_dst->mMin, r.mMin);
						range_dst->mMax = max(range_dst->mMax, r.mMax);
					}
				++range_dst;
			}
	}
	MOSS_ASSERT(cur_range_vector == &ranges.front());

	// Store global range for bounding box calculation
	mMinSample = ranges[0][0].mMin;
	mMaxSample = ranges[0][0].mMax;

#ifdef MOSS_DEBUG
	// Validate that we did not lose range along the way
	uint16 minv = 0xffff, maxv = 0;
	for (uint16 v : quantized_samples)
		if (v != cNoCollisionValue16)
		{
			minv = min(minv, v);
			maxv = max(maxv, uint16(v + 1));
		}
	MOSS_ASSERT(mMinSample == minv && mMaxSample == maxv);
#endif

	// Now erase the first element, we need a 2x2 grid to start with
	ranges.erase(ranges.begin());

	// Create blocks
	uint max_stride = (num_blocks + 1) >> 1;
	RangeBlock *current_block = mRangeBlocks;
	for (uint level = 0; level < ranges.size(); ++level)
	{
		MOSS_ASSERT(uint(current_block - mRangeBlocks) == sGridOffsets[level]);

		uint in_n = 1 << level;
		uint out_n = min(in_n, max_stride); // At the most detailed level we store a non-power of 2 number of blocks

		for (uint y = 0; y < out_n; ++y)
			for (uint x = 0; x < out_n; ++x)
			{
				// Convert from 2x2 Range structure to 1 RangeBlock structure
				RangeBlock &rb = *current_block++;
				for (uint by = 0; by < 2; ++by)
					for (uint bx = 0; bx < 2; ++bx)
					{
						uint src_pos = (y * 2 + by) * 2 * in_n + (x * 2 + bx);
						uint dst_pos = by * 2 + bx;
						rb.mMin[dst_pos] = ranges[level][src_pos].mMin;
						rb.mMax[dst_pos] = ranges[level][src_pos].mMax;
					}
			}
	}
	MOSS_ASSERT(uint32(current_block - mRangeBlocks) == mRangeBlocksSize);

	// Quantize height samples
	memset(mHeightSamples, 0, mHeightSamplesSize);
	int sample = 0;
	for (uint y = 0; y < mSampleCount; ++y)
		for (uint x = 0; x < mSampleCount; ++x)
		{
			uint32 output_value;

			float h = x < inSettings.mSampleCount && y < inSettings.mSampleCount? inSettings.mHeightSamples[x + y * inSettings.mSampleCount] : cNoCollisionValue;
			if (h == cNoCollisionValue)
			{
				// No collision
				output_value = mSampleMask;
			}
			else
			{
				// Get range of block so we know what range to compress to
				uint bx = x / mBlockSize;
				uint by = y / mBlockSize;
				const Range &range = ranges.back()[by * num_blocks_pow2 + bx];
				MOSS_ASSERT(range.mMin < range.mMax);

				// Quantize to mBitsPerSample bits, note that mSampleMask is reserved for indicating that there's no collision.
				// We divide the range into mSampleMask segments and use the mid points of these segments as the quantized values.
				// This results in a lower error than if we had quantized our data using the lowest point of all these segments.
				float h_min = min_value + range.mMin / scale;
				float h_delta = float(range.mMax - range.mMin) / scale;
				float quantized_height = floor((h - h_min) * float(mSampleMask) / h_delta);
				output_value = uint32(Clamp((int)quantized_height, 0, int(mSampleMask) - 1)); // mSampleMask is reserved as 'no collision value'
			}

			// Store the sample
			uint byte_pos = sample >> 3;
			uint bit_pos = sample & 0b111;
			output_value <<= bit_pos;
			MOSS_ASSERT(byte_pos + 1 < mHeightSamplesSize);
			mHeightSamples[byte_pos] |= uint8(output_value);
			mHeightSamples[byte_pos + 1] |= uint8(output_value >> 8);
			sample += inSettings.mBitsPerSample;
		}

	// Calculate the active edges
	CalculateActiveEdges(inSettings);

	// Compress material indices
	if (mMaterials.size() > 1 || inSettings.mMaterialsCapacity > 1)
		StoreMaterialIndices(inSettings);

	outResult.Set(this);
}

HeightFieldShape::~HeightFieldShape()
{
	if (mRangeBlocks != nullptr)
		AlignedFree(mRangeBlocks);
}

Ref<HeightFieldShape> HeightFieldShape::Clone() const
{
	Ref<HeightFieldShape> clone = new HeightFieldShape;
	clone->SetUserData(GetUserData());

	clone->mOffset = mOffset;
	clone->mScale = mScale;
	clone->mSampleCount = mSampleCount;
	clone->mBlockSize = mBlockSize;
	clone->mBitsPerSample = mBitsPerSample;
	clone->mSampleMask = mSampleMask;
	clone->mMinSample = mMinSample;
	clone->mMaxSample = mMaxSample;

	clone->AllocateBuffers();
	memcpy(clone->mRangeBlocks, mRangeBlocks, mRangeBlocksSize * sizeof(RangeBlock) + mHeightSamplesSize + mActiveEdgesSize); // Copy the entire buffer in 1 go

	clone->mMaterials.reserve(mMaterials.capacity()); // Ensure we keep the capacity of the original
	clone->mMaterials = mMaterials;
	clone->mMaterialIndices = mMaterialIndices;
	clone->mNumBitsPerMaterialIndex = mNumBitsPerMaterialIndex;

#ifndef MOSS_DEBUG_RENDERER
	clone->mGeometry = mGeometry;
	clone->mCachedUseMaterialColors = mCachedUseMaterialColors;
#endif // MOSS_DEBUG_RENDERER

	return clone;
}

inline void HeightFieldShape::sGetRangeBlockOffsetAndStride(uint inNumBlocks, uint inMaxLevel, uint &outRangeBlockOffset, uint &outRangeBlockStride)
{
	outRangeBlockOffset = sGridOffsets[inMaxLevel - 1];
	outRangeBlockStride = (inNumBlocks + 1) >> 1;
}

inline void HeightFieldShape::GetRangeBlock(uint inBlockX, uint inBlockY, uint inRangeBlockOffset, uint inRangeBlockStride, RangeBlock *&outBlock, uint &outIndexInBlock)
{
	MOSS_ASSERT(inBlockX < GetNumBlocks() && inBlockY < GetNumBlocks());

	// Convert to location of range block
	uint rbx = inBlockX >> 1;
	uint rby = inBlockY >> 1;
	outIndexInBlock = ((inBlockY & 1) << 1) + (inBlockX & 1);

	uint offset = inRangeBlockOffset + rby * inRangeBlockStride + rbx;
	MOSS_ASSERT(offset < mRangeBlocksSize);
	outBlock = mRangeBlocks + offset;
}

inline void HeightFieldShape::GetBlockOffsetAndScale(uint inBlockX, uint inBlockY, uint inRangeBlockOffset, uint inRangeBlockStride, float &outBlockOffset, float &outBlockScale) const
{
	MOSS_ASSERT(inBlockX < GetNumBlocks() && inBlockY < GetNumBlocks());

	// Convert to location of range block
	uint rbx = inBlockX >> 1;
	uint rby = inBlockY >> 1;
	uint n = ((inBlockY & 1) << 1) + (inBlockX & 1);

	// Calculate offset and scale
	uint offset = inRangeBlockOffset + rby * inRangeBlockStride + rbx;
	MOSS_ASSERT(offset < mRangeBlocksSize);
	const RangeBlock &block = mRangeBlocks[offset];
	outBlockOffset = float(block.mMin[n]);
	outBlockScale = float(block.mMax[n] - block.mMin[n]) / float(mSampleMask);
}

inline uint8 HeightFieldShape::GetHeightSample(uint inX, uint inY) const
{
	MOSS_ASSERT(inX < mSampleCount);
	MOSS_ASSERT(inY < mSampleCount);

	// Determine bit position of sample
	uint sample = (inY * mSampleCount + inX) * uint(mBitsPerSample);
	uint byte_pos = sample >> 3;
	uint bit_pos = sample & 0b111;

	// Fetch the height sample value
	MOSS_ASSERT(byte_pos + 1 < mHeightSamplesSize);
	const uint8 *height_samples = mHeightSamples + byte_pos;
	uint16 height_sample = uint16(height_samples[0]) | uint16(uint16(height_samples[1]) << 8);
	return uint8(height_sample >> bit_pos) & mSampleMask;
}

inline Vec3 HeightFieldShape::GetPosition(uint inX, uint inY, float inBlockOffset, float inBlockScale, bool &outNoCollision) const
{
	// Get quantized value
	uint8 height_sample = GetHeightSample(inX, inY);
	outNoCollision = height_sample == mSampleMask;

	// Add 0.5 to the quantized value to minimize the error (see constructor)
	return mOffset + mScale * Vec3(float(inX), inBlockOffset + (0.5f + height_sample) * inBlockScale, float(inY));
}

Vec3 HeightFieldShape::GetPosition(uint inX, uint inY) const
{
	// Test if there are any samples
	if (mHeightSamplesSize == 0)
		return mOffset + mScale * Vec3(float(inX), 0.0f, float(inY));

	// Get block location
	uint bx = inX / mBlockSize;
	uint by = inY / mBlockSize;

	// Calculate offset and stride
	uint num_blocks = GetNumBlocks();
	uint range_block_offset, range_block_stride;
	sGetRangeBlockOffsetAndStride(num_blocks, sGetMaxLevel(num_blocks), range_block_offset, range_block_stride);

	float offset, scale;
	GetBlockOffsetAndScale(bx, by, range_block_offset, range_block_stride, offset, scale);

	bool no_collision;
	return GetPosition(inX, inY, offset, scale, no_collision);
}

bool HeightFieldShape::IsNoCollision(uint inX, uint inY) const
{
	return mHeightSamplesSize == 0 || GetHeightSample(inX, inY) == mSampleMask;
}

bool HeightFieldShape::ProjectOntoSurface(Vec3Arg inLocalPosition, Vec3 &outSurfacePosition, SubShapeID &outSubShapeID) const
{
	// Check if we have collision
	if (mHeightSamplesSize == 0)
		return false;

	// Convert coordinate to integer space
	Vec3 integer_space = (inLocalPosition - mOffset) / mScale;

	// Get x coordinate and fraction
	float x_frac = integer_space.GetX();
	if (x_frac < 0.0f || x_frac >= mSampleCount - 1)
		return false;
	uint x = (uint)floor(x_frac);
	x_frac -= x;

	// Get y coordinate and fraction
	float y_frac = integer_space.GetZ();
	if (y_frac < 0.0f || y_frac >= mSampleCount - 1)
		return false;
	uint y = (uint)floor(y_frac);
	y_frac -= y;

	// If one of the diagonal points doesn't have collision, we don't have a height at this location
	if (IsNoCollision(x, y) || IsNoCollision(x + 1, y + 1))
		return false;

	if (y_frac >= x_frac)
	{
		// Left bottom triangle, test the 3rd point
		if (IsNoCollision(x, y + 1))
			return false;

		// Interpolate height value
		Vec3 v1 = GetPosition(x, y);
		Vec3 v2 = GetPosition(x, y + 1);
		Vec3 v3 = GetPosition(x + 1, y + 1);
		outSurfacePosition = v1 + y_frac * (v2 - v1) + x_frac * (v3 - v2);
		SubShapeIDCreator creator;
		outSubShapeID = EncodeSubShapeID(creator, x, y, 0);
		return true;
	}
	else
	{
		// Right top triangle, test the third point
		if (IsNoCollision(x + 1, y))
			return false;

		// Interpolate height value
		Vec3 v1 = GetPosition(x, y);
		Vec3 v2 = GetPosition(x + 1, y + 1);
		Vec3 v3 = GetPosition(x + 1, y);
		outSurfacePosition = v1 + y_frac * (v2 - v3) + x_frac * (v3 - v1);
		SubShapeIDCreator creator;
		outSubShapeID = EncodeSubShapeID(creator, x, y, 1);
		return true;
	}
}

void HeightFieldShape::GetHeights(uint inX, uint inY, uint inSizeX, uint inSizeY, float *outHeights, intptr_t inHeightsStride) const
{
	if (inSizeX == 0 || inSizeY == 0)
		return;

	MOSS_ASSERT(inX % mBlockSize == 0 && inY % mBlockSize == 0);
	MOSS_ASSERT(inX < mSampleCount && inY < mSampleCount);
	MOSS_ASSERT(inX + inSizeX <= mSampleCount && inY + inSizeY <= mSampleCount);

	// Test if there are any samples
	if (mHeightSamplesSize == 0)
	{
		// No samples, return the offset
		float offset = mOffset.GetY();
		for (uint y = 0; y < inSizeY; ++y, outHeights += inHeightsStride)
			for (uint x = 0; x < inSizeX; ++x)
				outHeights[x] = offset;
	}
	else
	{
		// Calculate offset and stride
		uint num_blocks = GetNumBlocks();
		uint range_block_offset, range_block_stride;
		sGetRangeBlockOffsetAndStride(num_blocks, sGetMaxLevel(num_blocks), range_block_offset, range_block_stride);

		// Loop over blocks
		uint block_start_x = inX / mBlockSize;
		uint block_start_y = inY / mBlockSize;
		uint num_blocks_x = inSizeX / mBlockSize;
		uint num_blocks_y = inSizeY / mBlockSize;
		for (uint block_y = 0; block_y < num_blocks_y; ++block_y)
			for (uint block_x = 0; block_x < num_blocks_x; ++block_x)
			{
				// Get offset and scale for block
				float offset, scale;
				GetBlockOffsetAndScale(block_start_x + block_x, block_start_y + block_y, range_block_offset, range_block_stride, offset, scale);

				// Adjust by global offset and scale
				// Note: This is the math applied in GetPosition() written out to reduce calculations in the inner loop
				scale *= mScale.GetY();
				offset = mOffset.GetY() + mScale.GetY() * offset + 0.5f * scale;

				// Loop over samples in block
				for (uint sample_y = 0; sample_y < mBlockSize; ++sample_y)
					for (uint sample_x = 0; sample_x < mBlockSize; ++sample_x)
					{
						// Calculate output coordinate
						uint output_x = block_x * mBlockSize + sample_x;
						uint output_y = block_y * mBlockSize + sample_y;

						// Get quantized value
						uint8 height_sample = GetHeightSample(inX + output_x, inY + output_y);

						// Dequantize
						float h = height_sample != mSampleMask? offset + height_sample * scale : cNoCollisionValue;
						outHeights[output_y * inHeightsStride + output_x] = h;
					}
			}
	}
}

void HeightFieldShape::SetHeights(uint inX, uint inY, uint inSizeX, uint inSizeY, const float *inHeights, intptr_t inHeightsStride, TempAllocator &inAllocator, float inActiveEdgeCosThresholdAngle)
{
	if (inSizeX == 0 || inSizeY == 0)
		return;

	MOSS_ASSERT(mHeightSamplesSize > 0);
	MOSS_ASSERT(inX % mBlockSize == 0 && inY % mBlockSize == 0);
	MOSS_ASSERT(inX < mSampleCount && inY < mSampleCount);
	MOSS_ASSERT(inX + inSizeX <= mSampleCount && inY + inSizeY <= mSampleCount);

	// If we have a block in negative x/y direction, we will affect its range so we need to take it into account
	bool need_temp_heights = false;
	uint affected_x = inX;
	uint affected_y = inY;
	uint affected_size_x = inSizeX;
	uint affected_size_y = inSizeY;
	if (inX > 0) { affected_x -= mBlockSize; affected_size_x += mBlockSize; need_temp_heights = true; }
	if (inY > 0) { affected_y -= mBlockSize; affected_size_y += mBlockSize; need_temp_heights = true; }

	// If we have a block in positive x/y direction, our ranges are affected by it so we need to take it into account
	uint heights_size_x = affected_size_x;
	uint heights_size_y = affected_size_y;
	if (inX + inSizeX < mSampleCount) { heights_size_x += mBlockSize; need_temp_heights = true; }
	if (inY + inSizeY < mSampleCount) { heights_size_y += mBlockSize; need_temp_heights = true; }

	// Get heights for affected area
	const float *heights;
	intptr_t heights_stride;
	float *temp_heights;
	if (need_temp_heights)
	{
		// Fetch the surrounding height data (note we're forced to recompress this data with a potentially different range so there will be some precision loss here)
		temp_heights = (float *)inAllocator.Allocate(heights_size_x * heights_size_y * sizeof(float));
		heights = temp_heights;
		heights_stride = heights_size_x;

		// We need to fill in the following areas:
		//
		// +-----------------+
		// |        2        |
		// |---+---------+---|
		// |   |         |   |
		// | 3 |    1    | 4 |
		// |   |         |   |
		// |---+---------+---|
		// |        5        |
		// +-----------------+
		//
		// 1. The area that is affected by the new heights (we just copy these)
		// 2-5. These areas are either needed to calculate the range of the affected blocks or they need to be recompressed with a different range
		uint offset_x = inX - affected_x;
		uint offset_y = inY - affected_y;

		// Area 2
		GetHeights(affected_x, affected_y, heights_size_x, offset_y, temp_heights, heights_size_x);
		float *area3_start = temp_heights + offset_y * heights_size_x;

		// Area 3
		GetHeights(affected_x, inY, offset_x, inSizeY, area3_start, heights_size_x);

		// Area 1
		float *area1_start = area3_start + offset_x;
		for (uint y = 0; y < inSizeY; ++y, area1_start += heights_size_x, inHeights += inHeightsStride)
			memcpy(area1_start, inHeights, inSizeX * sizeof(float));

		// Area 4
		uint area4_x = inX + inSizeX;
		GetHeights(area4_x, inY, affected_x + heights_size_x - area4_x, inSizeY, area3_start + area4_x - affected_x, heights_size_x);

		// Area 5
		uint area5_y = inY + inSizeY;
		float *area5_start = temp_heights + (area5_y - affected_y) * heights_size_x;
		GetHeights(affected_x, area5_y, heights_size_x, affected_y + heights_size_y - area5_y, area5_start, heights_size_x);
	}
	else
	{
		// We can directly use the input buffer because there are no extra edges to take into account
		heights = inHeights;
		heights_stride = inHeightsStride;
		temp_heights = nullptr;
	}

	// Calculate offset and stride
	uint num_blocks = GetNumBlocks();
	uint range_block_offset, range_block_stride;
	uint max_level = sGetMaxLevel(num_blocks);
	sGetRangeBlockOffsetAndStride(num_blocks, max_level, range_block_offset, range_block_stride);

	// Loop over blocks
	uint block_start_x = affected_x / mBlockSize;
	uint block_start_y = affected_y / mBlockSize;
	uint num_blocks_x = affected_size_x / mBlockSize;
	uint num_blocks_y = affected_size_y / mBlockSize;
	for (uint block_y = 0, sample_start_y = 0; block_y < num_blocks_y; ++block_y, sample_start_y += mBlockSize)
		for (uint block_x = 0, sample_start_x = 0; block_x < num_blocks_x; ++block_x, sample_start_x += mBlockSize)
		{
			// Determine quantized min and max value for block
			// Note that we need to include 1 extra row in the positive x/y direction to account for connecting triangles
			int min_value = 0xffff;
			int max_value = 0;
			uint sample_x_end = min(sample_start_x + mBlockSize + 1, mSampleCount - affected_x);
			uint sample_y_end = min(sample_start_y + mBlockSize + 1, mSampleCount - affected_y);
			for (uint sample_y = sample_start_y; sample_y < sample_y_end; ++sample_y)
				for (uint sample_x = sample_start_x; sample_x < sample_x_end; ++sample_x)
				{
					float h = heights[sample_y * heights_stride + sample_x];
					if (h != cNoCollisionValue)
					{
						int quantized_height = Clamp((int)floor((h - mOffset.GetY()) / mScale.GetY()), 0, int(cMaxHeightValue16 - 1));
						min_value = min(min_value, quantized_height);
						max_value = max(max_value, quantized_height + 1);
					}
				}
			if (min_value > max_value)
				min_value = max_value = cNoCollisionValue16;

			// Update range for block
			RangeBlock *range_block;
			uint index_in_block;
			GetRangeBlock(block_start_x + block_x, block_start_y + block_y, range_block_offset, range_block_stride, range_block, index_in_block);
			range_block->mMin[index_in_block] = uint16(min_value);
			range_block->mMax[index_in_block] = uint16(max_value);

			// Get offset and scale for block
			float offset_block = float(min_value);
			float scale_block = float(max_value - min_value) / float(mSampleMask);

			// Calculate scale and offset using the formula used in GetPosition() solved for the quantized height (excluding 0.5 because we round down while quantizing)
			float scale = scale_block * mScale.GetY();
			float offset = mOffset.GetY() + offset_block * mScale.GetY();

			// Loop over samples in block
			sample_x_end = sample_start_x + mBlockSize;
			sample_y_end = sample_start_y + mBlockSize;
			for (uint sample_y = sample_start_y; sample_y < sample_y_end; ++sample_y)
				for (uint sample_x = sample_start_x; sample_x < sample_x_end; ++sample_x)
				{
					// Quantize height
					float h = heights[sample_y * heights_stride + sample_x];
					uint8 quantized_height = h != cNoCollisionValue? uint8(Clamp((int)floor((h - offset) / scale), 0, int(mSampleMask) - 1)) : mSampleMask;

					// Determine bit position of sample
					uint sample = ((affected_y + sample_y) * mSampleCount + affected_x + sample_x) * uint(mBitsPerSample);
					uint byte_pos = sample >> 3;
					uint bit_pos = sample & 0b111;

					// Update the height value sample
					MOSS_ASSERT(byte_pos + 1 < mHeightSamplesSize);
					uint8 *height_samples = mHeightSamples + byte_pos;
					uint16 height_sample = uint16(height_samples[0]) | uint16(uint16(height_samples[1]) << 8);
					height_sample &= ~(uint16(mSampleMask) << bit_pos);
					height_sample |= uint16(quantized_height) << bit_pos;
					height_samples[0] = uint8(height_sample);
					height_samples[1] = uint8(height_sample >> 8);
				}
		}

	// Update active edges
	// Note that we must take an extra row on all sides to account for connecting triangles
	uint ae_x = inX > 1? inX - 2 : 0;
	uint ae_y = inY > 1? inY - 2 : 0;
	uint ae_sx = min(inX + inSizeX + 1, mSampleCount - 1) - ae_x;
	uint ae_sy = min(inY + inSizeY + 1, mSampleCount - 1) - ae_y;
	CalculateActiveEdges(ae_x, ae_y, ae_sx, ae_sy, heights, affected_x, affected_y, heights_stride, 1.0f, inActiveEdgeCosThresholdAngle, inAllocator);

	// Free temporary buffer
	if (temp_heights != nullptr)
		inAllocator.Free(temp_heights, heights_size_x * heights_size_y * sizeof(float));

	// Update hierarchy of range blocks
	while (max_level > 1)
	{
		// Get offset and stride for destination blocks
		uint dst_range_block_offset, dst_range_block_stride;
		sGetRangeBlockOffsetAndStride(num_blocks >> 1, max_level - 1, dst_range_block_offset, dst_range_block_stride);

		// We'll be processing 2x2 blocks below so we need the start coordinates to be even and we extend the number of blocks to correct for that
		if (block_start_x & 1) { --block_start_x; ++num_blocks_x; }
		if (block_start_y & 1) { --block_start_y; ++num_blocks_y; }

		// Loop over all affected blocks
		uint block_end_x = block_start_x + num_blocks_x;
		uint block_end_y = block_start_y + num_blocks_y;
		for (uint block_y = block_start_y; block_y < block_end_y; block_y += 2)
			for (uint block_x = block_start_x; block_x < block_end_x; block_x += 2)
			{
				// Get source range block
				RangeBlock *src_range_block;
				uint index_in_src_block;
				GetRangeBlock(block_x, block_y, range_block_offset, range_block_stride, src_range_block, index_in_src_block);

				// Determine quantized min and max value for the entire 2x2 block
				uint16 min_value = 0xffff;
				uint16 max_value = 0;
				for (uint i = 0; i < 4; ++i)
					if (src_range_block->mMin[i] != cNoCollisionValue16)
					{
						min_value = min(min_value, src_range_block->mMin[i]);
						max_value = max(max_value, src_range_block->mMax[i]);
					}

				// Write to destination block
				RangeBlock *dst_range_block;
				uint index_in_dst_block;
				GetRangeBlock(block_x >> 1, block_y >> 1, dst_range_block_offset, dst_range_block_stride, dst_range_block, index_in_dst_block);
				dst_range_block->mMin[index_in_dst_block] = uint16(min_value);
				dst_range_block->mMax[index_in_dst_block] = uint16(max_value);
			}

		// Go up one level
		--max_level;
		num_blocks >>= 1;
		block_start_x >>= 1;
		block_start_y >>= 1;
		num_blocks_x = min((num_blocks_x + 1) >> 1, num_blocks);
		num_blocks_y = min((num_blocks_y + 1) >> 1, num_blocks);

		// Update stride and offset for source to old destination
		range_block_offset = dst_range_block_offset;
		range_block_stride = dst_range_block_stride;
	}

	// Calculate new min and max sample for the entire height field
	mMinSample = 0xffff;
	mMaxSample = 0;
	for (uint i = 0; i < 4; ++i)
		if (mRangeBlocks[0].mMin[i] != cNoCollisionValue16)
		{
			mMinSample = min(mMinSample, mRangeBlocks[0].mMin[i]);
			mMaxSample = max(mMaxSample, mRangeBlocks[0].mMax[i]);
		}

#ifndef MOSS_DEBUG_RENDERER
	// Invalidate temporary rendering data
	mGeometry.clear();
#endif
}

void HeightFieldShape::GetMaterials(uint inX, uint inY, uint inSizeX, uint inSizeY, uint8 *outMaterials, intptr_t inMaterialsStride) const
{
	if (inSizeX == 0 || inSizeY == 0)
		return;

	if (mMaterialIndices.empty())
	{
		// Return all 0's
		for (uint y = 0; y < inSizeY; ++y)
		{
			uint8 *out_indices = outMaterials + y * inMaterialsStride;
			for (uint x = 0; x < inSizeX; ++x)
				*out_indices++ = 0;
		}
		return;
	}

	MOSS_ASSERT(inX < mSampleCount && inY < mSampleCount);
	MOSS_ASSERT(inX + inSizeX < mSampleCount && inY + inSizeY < mSampleCount);

	uint count_min_1 = mSampleCount - 1;
	uint16 material_index_mask = uint16((1 << mNumBitsPerMaterialIndex) - 1);

	for (uint y = 0; y < inSizeY; ++y)
	{
		// Calculate input position
		uint bit_pos = (inX + (inY + y) * count_min_1) * mNumBitsPerMaterialIndex;
		const uint8 *in_indices = mMaterialIndices.data() + (bit_pos >> 3);
		bit_pos &= 0b111;

		// Calculate output position
		uint8 *out_indices = outMaterials + y * inMaterialsStride;

		for (uint x = 0; x < inSizeX; ++x)
		{
			// Get material index
			uint16 material_index = uint16(in_indices[0]) + uint16(uint16(in_indices[1]) << 8);
			material_index >>= bit_pos;
			material_index &= material_index_mask;
			*out_indices = uint8(material_index);

			// Go to the next index
			bit_pos += mNumBitsPerMaterialIndex;
			in_indices += bit_pos >> 3;
			bit_pos &= 0b111;
			++out_indices;
		}
	}
}

bool HeightFieldShape::SetMaterials(uint inX, uint inY, uint inSizeX, uint inSizeY, const uint8 *inMaterials, intptr_t inMaterialsStride, const PhysicsMaterialList *inMaterialList, TempAllocator &inAllocator)
{
	if (inSizeX == 0 || inSizeY == 0)
		return true;

	MOSS_ASSERT(inX < mSampleCount && inY < mSampleCount);
	MOSS_ASSERT(inX + inSizeX < mSampleCount && inY + inSizeY < mSampleCount);

	// Remap materials
	uint material_remap_table_size = uint(inMaterialList != nullptr? inMaterialList->size() : mMaterials.size());
	uint8 *material_remap_table = (uint8 *)inAllocator.Allocate(material_remap_table_size);
	MOSS_SCOPE_EXIT([&inAllocator, material_remap_table, material_remap_table_size]{ inAllocator.Free(material_remap_table, material_remap_table_size); });
	if (inMaterialList != nullptr)
	{
		// Conservatively reserve more space if the incoming material list is bigger
		if (inMaterialList->size() > mMaterials.size())
			mMaterials.reserve(inMaterialList->size());

		// Create a remap table
		uint8 *remap_entry = material_remap_table;
		for (const PhysicsMaterial *material : *inMaterialList)
		{
			// Try to find it in the existing list
			PhysicsMaterialList::const_iterator it = std::find(mMaterials.begin(), mMaterials.end(), material);
			if (it != mMaterials.end())
			{
				// Found it, calculate index
				*remap_entry = uint8(it - mMaterials.begin());
			}
			else
			{
				// Not found, add it
				if (mMaterials.size() >= 256)
				{
					// We can't have more than 256 materials since we use uint8 as indices
					return false;
				}
				*remap_entry = uint8(mMaterials.size());
				mMaterials.push_back(material);
			}
			++remap_entry;
		}
	}
	else
	{
		// No remapping
		for (uint i = 0; i < material_remap_table_size; ++i)
			material_remap_table[i] = uint8(i);
	}

	if (mMaterials.size() == 1)
	{
		// Only 1 material, we don't need to store the material indices
		return true;
	}

	// Check if we need to resize the material indices array
	uint count_min_1 = mSampleCount - 1;
	uint32 new_bits_per_material_index = 32 - CountLeadingZeros((uint32)mMaterials.size() - 1);
	MOSS_ASSERT(mNumBitsPerMaterialIndex <= 8 && new_bits_per_material_index <= 8);
	if (new_bits_per_material_index > mNumBitsPerMaterialIndex)
	{
		// Resize the material indices array
		mMaterialIndices.resize(((Square(count_min_1) * new_bits_per_material_index + 7) >> 3) + 1, 0); // Add 1 byte so we don't read out of bounds when reading an uint16

		// Calculate old and new mask
		uint16 old_material_index_mask = uint16((1 << mNumBitsPerMaterialIndex) - 1);
		uint16 new_material_index_mask = uint16((1 << new_bits_per_material_index) - 1);

		// Loop through the array backwards to avoid overwriting data
		int in_bit_pos = (count_min_1 * count_min_1 - 1) * mNumBitsPerMaterialIndex;
		const uint8 *in_indices = mMaterialIndices.data() + (in_bit_pos >> 3);
		in_bit_pos &= 0b111;
		int out_bit_pos = (count_min_1 * count_min_1 - 1) * new_bits_per_material_index;
		uint8 *out_indices = mMaterialIndices.data() + (out_bit_pos >> 3);
		out_bit_pos &= 0b111;

		while (out_indices >= mMaterialIndices.data())
		{
			// Read the material index
			uint16 material_index = uint16(in_indices[0]) + uint16(uint16(in_indices[1]) << 8);
			material_index >>= in_bit_pos;
			material_index &= old_material_index_mask;

			// Write the material index
			uint16 output_data = uint16(out_indices[0]) + uint16(uint16(out_indices[1]) << 8);
			output_data &= ~(new_material_index_mask << out_bit_pos);
			output_data |= material_index << out_bit_pos;
			out_indices[0] = uint8(output_data);
			out_indices[1] = uint8(output_data >> 8);

			// Go to the previous index
			in_bit_pos -= int(mNumBitsPerMaterialIndex);
			in_indices += in_bit_pos >> 3;
			in_bit_pos &= 0b111;
			out_bit_pos -= int(new_bits_per_material_index);
			out_indices += out_bit_pos >> 3;
			out_bit_pos &= 0b111;
		}

		// Accept the new bits per material index
		mNumBitsPerMaterialIndex = new_bits_per_material_index;
	}

	uint16 material_index_mask = uint16((1 << mNumBitsPerMaterialIndex) - 1);
	for (uint y = 0; y < inSizeY; ++y)
	{
		// Calculate input position
		const uint8 *in_indices = inMaterials + y * inMaterialsStride;

		// Calculate output position
		uint bit_pos = (inX + (inY + y) * count_min_1) * mNumBitsPerMaterialIndex;
		uint8 *out_indices = mMaterialIndices.data() + (bit_pos >> 3);
		bit_pos &= 0b111;

		for (uint x = 0; x < inSizeX; ++x)
		{
			// Update material
			uint16 output_data = uint16(out_indices[0]) + uint16(uint16(out_indices[1]) << 8);
			output_data &= ~(material_index_mask << bit_pos);
			output_data |= material_remap_table[*in_indices] << bit_pos;
			out_indices[0] = uint8(output_data);
			out_indices[1] = uint8(output_data >> 8);

			// Go to the next index
			in_indices++;
			bit_pos += mNumBitsPerMaterialIndex;
			out_indices += bit_pos >> 3;
			bit_pos &= 0b111;
		}
	}

	return true;
}

MassProperties HeightFieldShape::GetMassProperties() const
{
	// Object should always be static, return default mass properties
	return MassProperties();
}

const PhysicsMaterial *HeightFieldShape::GetMaterial(uint inX, uint inY) const
{
	if (mMaterials.empty())
		return PhysicsMaterial::sDefault;
	if (mMaterials.size() == 1)
		return mMaterials[0];

	uint count_min_1 = mSampleCount - 1;
	MOSS_ASSERT(inX < count_min_1);
	MOSS_ASSERT(inY < count_min_1);

	// Calculate at which bit the material index starts
	uint bit_pos = (inX + inY * count_min_1) * mNumBitsPerMaterialIndex;
	uint byte_pos = bit_pos >> 3;
	bit_pos &= 0b111;

	// Read the material index
	MOSS_ASSERT(byte_pos + 1 < mMaterialIndices.size());
	const uint8 *material_indices = mMaterialIndices.data() + byte_pos;
	uint16 material_index = uint16(material_indices[0]) + uint16(uint16(material_indices[1]) << 8);
	material_index >>= bit_pos;
	material_index &= (1 << mNumBitsPerMaterialIndex) - 1;

	// Return the material
	return mMaterials[material_index];
}

uint HeightFieldShape::GetSubShapeIDBits() const
{
	// Need to store X, Y and 1 extra bit to specify the triangle number in the quad
	return 2 * (32 - CountLeadingZeros(mSampleCount - 1)) + 1;
}

SubShapeID HeightFieldShape::EncodeSubShapeID(const SubShapeIDCreator &inCreator, uint inX, uint inY, uint inTriangle) const
{
	return inCreator.PushID((inX + inY * mSampleCount) * 2 + inTriangle, GetSubShapeIDBits()).GetID();
}

void HeightFieldShape::DecodeSubShapeID(const SubShapeID &inSubShapeID, uint &outX, uint &outY, uint &outTriangle) const
{
	// Decode sub shape id
	SubShapeID remainder;
	uint32 id = inSubShapeID.PopID(GetSubShapeIDBits(), remainder);
	MOSS_ASSERT(remainder.IsEmpty(), "Invalid subshape ID");

	// Get triangle index
	outTriangle = id & 1;
	id >>= 1;

	// Fetch the x and y coordinate
	outX = id % mSampleCount;
	outY = id / mSampleCount;
}

void HeightFieldShape::GetSubShapeCoordinates(const SubShapeID &inSubShapeID, uint &outX, uint &outY, uint &outTriangleIndex) const
{
	DecodeSubShapeID(inSubShapeID, outX, outY, outTriangleIndex);
}

const PhysicsMaterial *HeightFieldShape::GetMaterial(const SubShapeID &inSubShapeID) const
{
	// Decode ID
	uint x, y, triangle;
	DecodeSubShapeID(inSubShapeID, x, y, triangle);

	// Fetch the material
	return GetMaterial(x, y);
}

Vec3 HeightFieldShape::GetSurfaceNormal(const SubShapeID &inSubShapeID, Vec3Arg inLocalSurfacePosition) const
{
	// Decode ID
	uint x, y, triangle;
	DecodeSubShapeID(inSubShapeID, x, y, triangle);

	// Fetch vertices that both triangles share
	Vec3 x1y1 = GetPosition(x, y);
	Vec3 x2y2 = GetPosition(x + 1, y + 1);

	// Get normal depending on which triangle was selected
	Vec3 normal;
	if (triangle == 0)
	{
		Vec3 x1y2 = GetPosition(x, y + 1);
		normal = (x2y2 - x1y2).Cross(x1y1 - x1y2);
	}
	else
	{
		Vec3 x2y1 = GetPosition(x + 1, y);
		normal = (x1y1 - x2y1).Cross(x2y2 - x2y1);
	}

	return normal.Normalized();
}

void HeightFieldShape::GetSupportingFace(const SubShapeID &inSubShapeID, Vec3Arg inDirection, Vec3Arg inScale, Mat44Arg inCenterOfMassTransform, SupportingFace &outVertices) const
{
	// Decode ID
	uint x, y, triangle;
	DecodeSubShapeID(inSubShapeID, x, y, triangle);

	// Fetch the triangle
	outVertices.resize(3);
	outVertices[0] = GetPosition(x, y);
	Vec3 v2 = GetPosition(x + 1, y + 1);
	if (triangle == 0)
	{
		outVertices[1] = GetPosition(x, y + 1);
		outVertices[2] = v2;
	}
	else
	{
		outVertices[1] = v2;
		outVertices[2] = GetPosition(x + 1, y);
	}

	// Flip triangle if scaled inside out
	if (ScaleHelpers::IsInsideOut(inScale))
		std::swap(outVertices[1], outVertices[2]);

	// Transform to world space
	Mat44 transform = inCenterOfMassTransform.PreScaled(inScale);
	for (Vec3 &v : outVertices)
		v = transform * v;
}

inline uint8 HeightFieldShape::GetEdgeFlags(uint inX, uint inY, uint inTriangle) const
{
	MOSS_ASSERT(inX < mSampleCount - 1 && inY < mSampleCount - 1);

	if (inTriangle == 0)
	{
		// The edge flags for this triangle are directly stored, find the right 3 bits
		uint bit_pos = 3 * (inX + inY * (mSampleCount - 1));
		uint byte_pos = bit_pos >> 3;
		bit_pos &= 0b111;
		MOSS_ASSERT(byte_pos + 1 < mActiveEdgesSize);
		const uint8 *active_edges = mActiveEdges + byte_pos;
		uint16 edge_flags = uint16(active_edges[0]) + uint16(uint16(active_edges[1]) << 8);
		return uint8(edge_flags >> bit_pos) & 0b111;
	}
	else
	{
		// We don't store this triangle directly, we need to look at our three neighbours to construct the edge flags
		uint8 edge0 = (GetEdgeFlags(inX, inY, 0) & 0b100) != 0? 0b001 : 0; // Diagonal edge
		uint8 edge1 = inX == mSampleCount - 2 || (GetEdgeFlags(inX + 1, inY, 0) & 0b001) != 0? 0b010 : 0; // Vertical edge
		uint8 edge2 = inY == 0 || (GetEdgeFlags(inX, inY - 1, 0) & 0b010) != 0? 0b100 : 0; // Horizontal edge
		return edge0 | edge1 | edge2;
	}
}

AABox HeightFieldShape::GetLocalBounds() const
{
	if (mMinSample == cNoCollisionValue16)
	{
		// This whole height field shape doesn't have any collision, return the center point
		Vec3 center = mOffset + 0.5f * mScale * Vec3(float(mSampleCount - 1), 0.0f, float(mSampleCount - 1));
		return AABox(center, center);
	}
	else
	{
		// Bounding box based on min and max sample height
		Vec3 bmin = mOffset + mScale * Vec3(0.0f, float(mMinSample), 0.0f);
		Vec3 bmax = mOffset + mScale * Vec3(float(mSampleCount - 1), float(mMaxSample), float(mSampleCount - 1));
		return AABox(bmin, bmax);
	}
}

#ifndef MOSS_DEBUG_RENDERER
void HeightFieldShape::Draw(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, Vec3Arg inScale, ColorArg inColor, bool inUseMaterialColors, bool inDrawWireframe) const
{
	// Don't draw anything if we don't have any collision
	if (mHeightSamplesSize == 0)
		return;

	// Reset the batch if we switch coloring mode
	if (mCachedUseMaterialColors != inUseMaterialColors)
	{
		mGeometry.clear();
		mCachedUseMaterialColors = inUseMaterialColors;
	}

	if (mGeometry.empty())
	{
		// Divide terrain in triangle batches of max 64x64x2 triangles to allow better culling of the terrain
		uint32 block_size = min<uint32>(mSampleCount, 64);
		for (uint32 by = 0; by < mSampleCount; by += block_size)
			for (uint32 bx = 0; bx < mSampleCount; bx += block_size)
			{
				// Create vertices for a block
				TArray<DebugRenderer::Triangle> triangles;
				triangles.resize(block_size * block_size * 2);
				DebugRenderer::Triangle *out_tri = &triangles[0];
				for (uint32 y = by, max_y = min(by + block_size, mSampleCount - 1); y < max_y; ++y)
					for (uint32 x = bx, max_x = min(bx + block_size, mSampleCount - 1); x < max_x; ++x)
						if (!IsNoCollision(x, y) && !IsNoCollision(x + 1, y + 1))
						{
							Vec3 x1y1 = GetPosition(x, y);
							Vec3 x2y2 = GetPosition(x + 1, y + 1);
							Color color = inUseMaterialColors? GetMaterial(x, y)->GetDebugColor() : Color::sWhite;

							if (!IsNoCollision(x, y + 1))
							{
								Vec3 x1y2 = GetPosition(x, y + 1);

								x1y1.StoreFloat3(&out_tri->mV[0].mPosition);
								x1y2.StoreFloat3(&out_tri->mV[1].mPosition);
								x2y2.StoreFloat3(&out_tri->mV[2].mPosition);

								Vec3 normal = (x2y2 - x1y2).Cross(x1y1 - x1y2).Normalized();
								for (DebugRenderer::Vertex &v : out_tri->mV)
								{
									v.mColor = color;
									v.mUV = Float2(0, 0);
									normal.StoreFloat3(&v.mNormal);
								}

								++out_tri;
							}

							if (!IsNoCollision(x + 1, y))
							{
								Vec3 x2y1 = GetPosition(x + 1, y);

								x1y1.StoreFloat3(&out_tri->mV[0].mPosition);
								x2y2.StoreFloat3(&out_tri->mV[1].mPosition);
								x2y1.StoreFloat3(&out_tri->mV[2].mPosition);

								Vec3 normal = (x1y1 - x2y1).Cross(x2y2 - x2y1).Normalized();
								for (DebugRenderer::Vertex &v : out_tri->mV)
								{
									v.mColor = color;
									v.mUV = Float2(0, 0);
									normal.StoreFloat3(&v.mNormal);
								}

								++out_tri;
							}
						}

				// Resize triangles array to actual amount of triangles written
				size_t num_triangles = out_tri - &triangles[0];
				triangles.resize(num_triangles);

				// Create batch
				if (num_triangles > 0)
					mGeometry.push_back(new DebugRenderer::Geometry(inRenderer->CreateTriangleBatch(triangles), DebugRenderer::sCalculateBounds(&triangles[0].mV[0], int(3 * num_triangles))));
			}
	}

	// Get transform including scale
	RMat44 transform = inCenterOfMassTransform.PreScaled(inScale);

	// Test if the shape is scaled inside out
	DebugRenderer::ECullMode cull_mode = ScaleHelpers::IsInsideOut(inScale)? DebugRenderer::ECullMode::CullFrontFace : DebugRenderer::ECullMode::CullBackFace;

	// Determine the draw mode
	DebugRenderer::EDrawMode draw_mode = inDrawWireframe? DebugRenderer::EDrawMode::Wireframe : DebugRenderer::EDrawMode::Solid;

	// Draw the geometry
	for (const DebugRenderer::GeometryRef &b : mGeometry)
		inRenderer->DrawGeometry(transform, inColor, b, cull_mode, DebugRenderer::ECastShadow::On, draw_mode);

	if (sDrawTriangleOutlines)
	{
		struct Visitor
		{
			MOSS_INLINE explicit		Visitor(const HeightFieldShape *inShape, DebugRenderer *inRenderer, RMat44Arg inTransform) :
				mShape(inShape),
				mRenderer(inRenderer),
				mTransform(inTransform)
			{
			}

			MOSS_INLINE bool			ShouldAbort() const
			{
				return false;
			}

			MOSS_INLINE bool			ShouldVisitRangeBlock([[maybe_unused]] int inStackTop) const
			{
				return true;
			}

			MOSS_INLINE int			VisitRangeBlock(Vec4Arg inBoundsMinX, Vec4Arg inBoundsMinY, Vec4Arg inBoundsMinZ, Vec4Arg inBoundsMaxX, Vec4Arg inBoundsMaxY, Vec4Arg inBoundsMaxZ, UVec4 &ioProperties, [[maybe_unused]] int inStackTop) const
			{
				UVec4 valid = Vec4::sLessOrEqual(inBoundsMinY, inBoundsMaxY);
				return CountAndSortTrues(valid, ioProperties);
			}

			MOSS_INLINE void			VisitTriangle(uint inX, uint inY, uint inTriangle, Vec3Arg inV0, Vec3Arg inV1, Vec3Arg inV2) const
			{
				// Determine active edges
				uint8 active_edges = mShape->GetEdgeFlags(inX, inY, inTriangle);

				// Loop through edges
				Vec3 v[] = { inV0, inV1, inV2 };
				for (uint edge_idx = 0; edge_idx < 3; ++edge_idx)
				{
					RVec3 v1 = mTransform * v[edge_idx];
					RVec3 v2 = mTransform * v[(edge_idx + 1) % 3];

					// Draw active edge as a green arrow, other edges as grey
					if (active_edges & (1 << edge_idx))
						mRenderer->DrawArrow(v1, v2, Color::sGreen, 0.01f);
					else
						mRenderer->DrawLine(v1, v2, Color::sGrey);
				}
			}

			const HeightFieldShape *mShape;
			DebugRenderer *			mRenderer;
			RMat44					mTransform;
		};

		Visitor visitor(this, inRenderer, inCenterOfMassTransform.PreScaled(inScale));
		WalkHeightField(visitor);
	}
}
#endif // MOSS_DEBUG_RENDERER

class HeightFieldShape::DecodingContext
{
public:
	MOSS_INLINE explicit			DecodingContext(const HeightFieldShape *inShape) :
		mShape(inShape)
	{
		static_assert(std::size(sGridOffsets) == cNumBitsXY + 1, "Offsets array is not long enough");

		// Construct root stack entry
		mPropertiesStack[0] = 0; // level: 0, x: 0, y: 0
	}

	template <class Visitor>
	MOSS_INLINE void				WalkHeightField(Visitor &ioVisitor)
	{
		// Early out if there's no collision
		if (mShape->mHeightSamplesSize == 0)
			return;

		// Assert that an inside-out bounding box does not collide
		MOSS_IF_ENABLE_ASSERTS(UVec4 dummy = UVec4::sReplicate(0);)
		MOSS_ASSERT(ioVisitor.VisitRangeBlock(Vec4::sReplicate(-1.0e6f), Vec4::sReplicate(1.0e6f), Vec4::sReplicate(-1.0e6f), Vec4::sReplicate(1.0e6f), Vec4::sReplicate(-1.0e6f), Vec4::sReplicate(1.0e6f), dummy, 0) == 0);

		// Precalculate values relating to sample count
		uint32 sample_count = mShape->mSampleCount;
		UVec4 sample_count_min_1 = UVec4::sReplicate(sample_count - 1);

		// Precalculate values relating to block size
		uint32 block_size = mShape->mBlockSize;
		uint32 block_size_plus_1 = block_size + 1;
		uint num_blocks = mShape->GetNumBlocks();
		uint num_blocks_min_1 = num_blocks - 1;
		uint max_level = HeightFieldShape::sGetMaxLevel(num_blocks);
		uint32 max_stride = (num_blocks + 1) >> 1;

		// Precalculate range block offset and stride for GetBlockOffsetAndScale
		uint range_block_offset, range_block_stride;
		sGetRangeBlockOffsetAndStride(num_blocks, max_level, range_block_offset, range_block_stride);

		// Allocate space for vertices and 'no collision' flags
		int array_size = Square(block_size_plus_1);
		Vec3 *vertices = reinterpret_cast<Vec3 *>(MOSS_STACK_ALLOC(array_size * sizeof(Vec3)));
		bool *no_collision = reinterpret_cast<bool *>(MOSS_STACK_ALLOC(array_size * sizeof(bool)));

		// Splat offsets
		Vec4 ox = mShape->mOffset.SplatX();
		Vec4 oy = mShape->mOffset.SplatY();
		Vec4 oz = mShape->mOffset.SplatZ();

		// Splat scales
		Vec4 sx = mShape->mScale.SplatX();
		Vec4 sy = mShape->mScale.SplatY();
		Vec4 sz = mShape->mScale.SplatZ();

		do
		{
			// Decode properties
			uint32 properties_top = mPropertiesStack[mTop];
			uint32 x = properties_top & cMaskBitsXY;
			uint32 y = (properties_top >> cNumBitsXY) & cMaskBitsXY;
			uint32 level = properties_top >> cLevelShift;

			if (level >= max_level)
			{
				// Determine actual range of samples (minus one because we eventually want to iterate over the triangles, not the samples)
				uint32 min_x = x * block_size;
				uint32 max_x = min_x + block_size;
				uint32 min_y = y * block_size;
				uint32 max_y = min_y + block_size;

				// Decompress vertices of block at (x, y)
				Vec3 *dst_vertex = vertices;
				bool *dst_no_collision = no_collision;
				float block_offset, block_scale;
				mShape->GetBlockOffsetAndScale(x, y, range_block_offset, range_block_stride, block_offset, block_scale);
				for (uint32 v_y = min_y; v_y < max_y; ++v_y)
				{
					for (uint32 v_x = min_x; v_x < max_x; ++v_x)
					{
						*dst_vertex = mShape->GetPosition(v_x, v_y, block_offset, block_scale, *dst_no_collision);
						++dst_vertex;
						++dst_no_collision;
					}

					// Skip last column, these values come from a different block
					++dst_vertex;
					++dst_no_collision;
				}

				// Decompress block (x + 1, y)
				uint32 max_x_decrement = 0;
				if (x < num_blocks_min_1)
				{
					dst_vertex = vertices + block_size;
					dst_no_collision = no_collision + block_size;
					mShape->GetBlockOffsetAndScale(x + 1, y, range_block_offset, range_block_stride, block_offset, block_scale);
					for (uint32 v_y = min_y; v_y < max_y; ++v_y)
					{
						*dst_vertex = mShape->GetPosition(max_x, v_y, block_offset, block_scale, *dst_no_collision);
						dst_vertex += block_size_plus_1;
						dst_no_collision += block_size_plus_1;
					}
				}
				else
					max_x_decrement = 1; // We don't have a next block, one less triangle to test

				// Decompress block (x, y + 1)
				if (y < num_blocks_min_1)
				{
					uint start = block_size * block_size_plus_1;
					dst_vertex = vertices + start;
					dst_no_collision = no_collision + start;
					mShape->GetBlockOffsetAndScale(x, y + 1, range_block_offset, range_block_stride, block_offset, block_scale);
					for (uint32 v_x = min_x; v_x < max_x; ++v_x)
					{
						*dst_vertex = mShape->GetPosition(v_x, max_y, block_offset, block_scale, *dst_no_collision);
						++dst_vertex;
						++dst_no_collision;
					}

					// Decompress single sample of block at (x + 1, y + 1)
					if (x < num_blocks_min_1)
					{
						mShape->GetBlockOffsetAndScale(x + 1, y + 1, range_block_offset, range_block_stride, block_offset, block_scale);
						*dst_vertex = mShape->GetPosition(max_x, max_y, block_offset, block_scale, *dst_no_collision);
					}
				}
				else
					--max_y; // We don't have a next block, one less triangle to test

				// Update max_x (we've been using it so we couldn't update it earlier)
				max_x -= max_x_decrement;

				// We're going to divide the vertices in 4 blocks to do one more runtime sub-division, calculate the ranges of those blocks
				struct Range
				{
					uint32 mMinX, mMinY, mNumTrianglesX, mNumTrianglesY;
				};
				uint32 half_block_size = block_size >> 1;
				uint32 block_size_x = max_x - min_x - half_block_size;
				uint32 block_size_y = max_y - min_y - half_block_size;
				Range ranges[] =
				{
					{ 0, 0,									half_block_size, half_block_size },
					{ half_block_size, 0,					block_size_x, half_block_size },
					{ 0, half_block_size,					half_block_size, block_size_y },
					{ half_block_size, half_block_size,		block_size_x, block_size_y },
				};

				// Calculate the min and max of each of the blocks
				Mat44 block_min, block_max;
				for (int block = 0; block < 4; ++block)
				{
					// Get the range for this block
					const Range &range = ranges[block];
					uint32 start = range.mMinX + range.mMinY * block_size_plus_1;
					uint32 size_x_plus_1 = range.mNumTrianglesX + 1;
					uint32 size_y_plus_1 = range.mNumTrianglesY + 1;

					// Calculate where to start reading
					const Vec3 *src_vertex = vertices + start;
					const bool *src_no_collision = no_collision + start;
					uint32 stride = block_size_plus_1 - size_x_plus_1;

					// Start range with a very large inside-out box
					Vec3 value_min = Vec3::sReplicate(cLargeFloat);
					Vec3 value_max = Vec3::sReplicate(-cLargeFloat);

					// Loop over the samples to determine the min and max of this block
					for (uint32 block_y = 0; block_y < size_y_plus_1; ++block_y)
					{
						for (uint32 block_x = 0; block_x < size_x_plus_1; ++block_x)
						{
							if (!*src_no_collision)
							{
								value_min = Vec3::sMin(value_min, *src_vertex);
								value_max = Vec3::sMax(value_max, *src_vertex);
							}
							++src_vertex;
							++src_no_collision;
						}
						src_vertex += stride;
						src_no_collision += stride;
					}
					block_min.SetColumn4(block, Vec4(value_min));
					block_max.SetColumn4(block, Vec4(value_max));
				}

			#ifdef MOSS_DEBUG_HEIGHT_FIELD
				// Draw the bounding boxes of the sub-nodes
				for (int block = 0; block < 4; ++block)
				{
					AABox bounds(block_min.GetColumn3(block), block_max.GetColumn3(block));
					if (bounds.IsValid())
						DebugRenderer::sInstance->DrawWireBox(bounds, Color::sYellow);
				}
			#endif // MOSS_DEBUG_HEIGHT_FIELD

				// Transpose so we have the mins and maxes of each of the blocks in rows instead of columns
				Mat44 transposed_min = block_min.Transposed();
				Mat44 transposed_max = block_max.Transposed();

				// Check which blocks collide
				// Note: At this point we don't use our own stack but we do allow the visitor to use its own stack
				// to store collision distances so that we can still early out when no closer hits have been found.
				UVec4 colliding_blocks(0, 1, 2, 3);
				int num_results = ioVisitor.VisitRangeBlock(transposed_min.GetColumn4(0), transposed_min.GetColumn4(1), transposed_min.GetColumn4(2), transposed_max.GetColumn4(0), transposed_max.GetColumn4(1), transposed_max.GetColumn4(2), colliding_blocks, mTop);

				// Loop through the results backwards (closest first)
				int result = num_results - 1;
				while (result >= 0)
				{
					// Calculate the min and max of this block
					uint32 block = colliding_blocks[result];
					const Range &range = ranges[block];
					uint32 block_min_x = min_x + range.mMinX;
					uint32 block_max_x = block_min_x + range.mNumTrianglesX;
					uint32 block_min_y = min_y + range.mMinY;
					uint32 block_max_y = block_min_y + range.mNumTrianglesY;

					// Loop triangles
					for (uint32 v_y = block_min_y; v_y < block_max_y; ++v_y)
						for (uint32 v_x = block_min_x; v_x < block_max_x; ++v_x)
						{
							// Get first vertex
							const int offset = (v_y - min_y) * block_size_plus_1 + (v_x - min_x);
							const Vec3 *start_vertex = vertices + offset;
							const bool *start_no_collision = no_collision + offset;

							// Check if vertices shared by both triangles have collision
							if (!start_no_collision[0] && !start_no_collision[block_size_plus_1 + 1])
							{
								// Loop 2 triangles
								for (uint t = 0; t < 2; ++t)
								{
									// Determine triangle vertices
									Vec3 v0, v1, v2;
									if (t == 0)
									{
										// Check third vertex
										if (start_no_collision[block_size_plus_1])
											continue;

										// Get vertices for triangle
										v0 = start_vertex[0];
										v1 = start_vertex[block_size_plus_1];
										v2 = start_vertex[block_size_plus_1 + 1];
									}
									else
									{
										// Check third vertex
										if (start_no_collision[1])
											continue;

										// Get vertices for triangle
										v0 = start_vertex[0];
										v1 = start_vertex[block_size_plus_1 + 1];
										v2 = start_vertex[1];
									}

								#ifdef MOSS_DEBUG_HEIGHT_FIELD
									DebugRenderer::sInstance->DrawWireTriangle(RVec3(v0), RVec3(v1), RVec3(v2), Color::sWhite);
								#endif

									// Call visitor
									ioVisitor.VisitTriangle(v_x, v_y, t, v0, v1, v2);

									// Check if we're done
									if (ioVisitor.ShouldAbort())
										return;
								}
							}
						}

					// Fetch next block until we find one that the visitor wants to see
					do
						--result;
					while (result >= 0 && !ioVisitor.ShouldVisitRangeBlock(mTop + result));
				}
			}
			else
			{
				// Visit child grid
				uint32 stride = min(1U << level, max_stride); // At the most detailed level we store a non-power of 2 number of blocks
				uint32 offset = sGridOffsets[level] + stride * y + x;

				// Decode min/max height
				MOSS_ASSERT(offset < mShape->mRangeBlocksSize);
				UVec4 block = UVec4::sLoadInt4Aligned(reinterpret_cast<const uint32 *>(&mShape->mRangeBlocks[offset]));
				Vec4 bounds_miny = oy + sy * block.Expand4Uint16Lo().ToFloat();
				Vec4 bounds_maxy = oy + sy * block.Expand4Uint16Hi().ToFloat();

				// Calculate size of one cell at this grid level
				UVec4 internal_cell_size = UVec4::sReplicate(block_size << (max_level - level - 1)); // subtract 1 from level because we have an internal grid of 2x2

				// Calculate min/max x and z
				UVec4 two_x = UVec4::sReplicate(2 * x); // multiply by two because we have an internal grid of 2x2
				Vec4 bounds_minx = ox + sx * (internal_cell_size * (two_x + UVec4(0, 1, 0, 1))).ToFloat();
				Vec4 bounds_maxx = ox + sx * UVec4::sMin(internal_cell_size * (two_x + UVec4(1, 2, 1, 2)), sample_count_min_1).ToFloat();

				UVec4 two_y = UVec4::sReplicate(2 * y);
				Vec4 bounds_minz = oz + sz * (internal_cell_size * (two_y + UVec4(0, 0, 1, 1))).ToFloat();
				Vec4 bounds_maxz = oz + sz * UVec4::sMin(internal_cell_size * (two_y + UVec4(1, 1, 2, 2)), sample_count_min_1).ToFloat();

				// Calculate properties of child blocks
				UVec4 properties = UVec4::sReplicate(((level + 1) << cLevelShift) + (y << (cNumBitsXY + 1)) + (x << 1)) + UVec4(0, 1, 1 << cNumBitsXY, (1 << cNumBitsXY) + 1);

			#ifdef MOSS_DEBUG_HEIGHT_FIELD
				// Draw boxes
				for (int i = 0; i < 4; ++i)
				{
					AABox b(Vec3(bounds_minx[i], bounds_miny[i], bounds_minz[i]), Vec3(bounds_maxx[i], bounds_maxy[i], bounds_maxz[i]));
					if (b.IsValid())
						DebugRenderer::sInstance->DrawWireBox(b, Color::sGreen);
				}
			#endif

				// Check which sub nodes to visit
				int num_results = ioVisitor.VisitRangeBlock(bounds_minx, bounds_miny, bounds_minz, bounds_maxx, bounds_maxy, bounds_maxz, properties, mTop);

				// Push them onto the stack
				MOSS_ASSERT(mTop + 4 < cStackSize);
				properties.StoreInt4(&mPropertiesStack[mTop]);
				mTop += num_results;
			}

			// Check if we're done
			if (ioVisitor.ShouldAbort())
				return;

			// Fetch next node until we find one that the visitor wants to see
			do
				--mTop;
			while (mTop >= 0 && !ioVisitor.ShouldVisitRangeBlock(mTop));
		}
		while (mTop >= 0);
	}

	// This can be used to have the visitor early out (ioVisitor.ShouldAbort() returns true) and later continue again (call WalkHeightField() again)
	MOSS_INLINE bool				IsDoneWalking() const
	{
		return mTop < 0;
	}

private:
	const HeightFieldShape *	mShape;
	int							mTop = 0;
	uint32						mPropertiesStack[cStackSize];
};

template <class Visitor>
void HeightFieldShape::WalkHeightField(Visitor &ioVisitor) const
{
	DecodingContext ctx(this);
	ctx.WalkHeightField(ioVisitor);
}

bool HeightFieldShape::CastRay(const RayCast &inRay, const SubShapeIDCreator &inSubShapeIDCreator, RayCastResult &ioHit) const
{
	MOSS_PROFILE_FUNCTION();

	struct Visitor
	{
		MOSS_INLINE explicit		Visitor(const HeightFieldShape *inShape, const RayCast &inRay, const SubShapeIDCreator &inSubShapeIDCreator, RayCastResult &ioHit) :
			mHit(ioHit),
			mRayOrigin(inRay.mOrigin),
			mRayDirection(inRay.mDirection),
			mRayInvDirection(inRay.mDirection),
			mShape(inShape),
			mSubShapeIDCreator(inSubShapeIDCreator)
		{
		}

		MOSS_INLINE bool			ShouldAbort() const
		{
			return mHit.mFraction <= 0.0f;
		}

		MOSS_INLINE bool			ShouldVisitRangeBlock(int inStackTop) const
		{
			return mDistanceStack[inStackTop] < mHit.mFraction;
		}

		MOSS_INLINE int			VisitRangeBlock(Vec4Arg inBoundsMinX, Vec4Arg inBoundsMinY, Vec4Arg inBoundsMinZ, Vec4Arg inBoundsMaxX, Vec4Arg inBoundsMaxY, Vec4Arg inBoundsMaxZ, UVec4 &ioProperties, int inStackTop)
		{
			// Test bounds of 4 children
			Vec4 distance = RayAABox4(mRayOrigin, mRayInvDirection, inBoundsMinX, inBoundsMinY, inBoundsMinZ, inBoundsMaxX, inBoundsMaxY, inBoundsMaxZ);

			// Sort so that highest values are first (we want to first process closer hits and we process stack top to bottom)
			return SortReverseAndStore(distance, mHit.mFraction, ioProperties, &mDistanceStack[inStackTop]);
		}

		MOSS_INLINE void			VisitTriangle(uint inX, uint inY, uint inTriangle, Vec3Arg inV0, Vec3Arg inV1, Vec3Arg inV2)
		{
			float fraction = RayTriangle(mRayOrigin, mRayDirection, inV0, inV1, inV2);
			if (fraction < mHit.mFraction)
			{
				// It's a closer hit
				mHit.mFraction = fraction;
				mHit.mSubShapeID2 = mShape->EncodeSubShapeID(mSubShapeIDCreator, inX, inY, inTriangle);
				mReturnValue = true;
			}
		}

		RayCastResult &			mHit;
		Vec3					mRayOrigin;
		Vec3					mRayDirection;
		RayInvDirection			mRayInvDirection;
		const HeightFieldShape *mShape;
		SubShapeIDCreator		mSubShapeIDCreator;
		bool					mReturnValue = false;
		float					mDistanceStack[cStackSize];
	};

	Visitor visitor(this, inRay, inSubShapeIDCreator, ioHit);
	WalkHeightField(visitor);

	return visitor.mReturnValue;
}

void HeightFieldShape::CastRay(const RayCast &inRay, const RayCastSettings &inRayCastSettings, const SubShapeIDCreator &inSubShapeIDCreator, CastRayCollector &ioCollector, const ShapeFilter &inShapeFilter) const
{
	MOSS_PROFILE_FUNCTION();

	// Test shape filter
	if (!inShapeFilter.ShouldCollide(this, inSubShapeIDCreator.GetID()))
		return;

	struct Visitor
	{
		MOSS_INLINE explicit		Visitor(const HeightFieldShape *inShape, const RayCast &inRay, const RayCastSettings &inRayCastSettings, const SubShapeIDCreator &inSubShapeIDCreator, CastRayCollector &ioCollector) :
			mCollector(ioCollector),
			mRayOrigin(inRay.mOrigin),
			mRayDirection(inRay.mDirection),
			mRayInvDirection(inRay.mDirection),
			mBackFaceMode(inRayCastSettings.mBackFaceModeTriangles),
			mShape(inShape),
			mSubShapeIDCreator(inSubShapeIDCreator)
		{
		}

		MOSS_INLINE bool			ShouldAbort() const
		{
			return mCollector.ShouldEarlyOut();
		}

		MOSS_INLINE bool			ShouldVisitRangeBlock(int inStackTop) const
		{
			return mDistanceStack[inStackTop] < mCollector.GetEarlyOutFraction();
		}

		MOSS_INLINE int			VisitRangeBlock(Vec4Arg inBoundsMinX, Vec4Arg inBoundsMinY, Vec4Arg inBoundsMinZ, Vec4Arg inBoundsMaxX, Vec4Arg inBoundsMaxY, Vec4Arg inBoundsMaxZ, UVec4 &ioProperties, int inStackTop)
		{
			// Test bounds of 4 children
			Vec4 distance = RayAABox4(mRayOrigin, mRayInvDirection, inBoundsMinX, inBoundsMinY, inBoundsMinZ, inBoundsMaxX, inBoundsMaxY, inBoundsMaxZ);

			// Sort so that highest values are first (we want to first process closer hits and we process stack top to bottom)
			return SortReverseAndStore(distance, mCollector.GetEarlyOutFraction(), ioProperties, &mDistanceStack[inStackTop]);
		}

		MOSS_INLINE void			VisitTriangle(uint inX, uint inY, uint inTriangle, Vec3Arg inV0, Vec3Arg inV1, Vec3Arg inV2) const
		{
			// Back facing check
			if (mBackFaceMode == EBackFaceMode::IgnoreBackFaces && (inV2 - inV0).Cross(inV1 - inV0).Dot(mRayDirection) < 0)
				return;

			// Check the triangle
			float fraction = RayTriangle(mRayOrigin, mRayDirection, inV0, inV1, inV2);
			if (fraction < mCollector.GetEarlyOutFraction())
			{
				RayCastResult hit;
				hit.mBodyID = TransformedShape::sGetBodyID(mCollector.GetContext());
				hit.mFraction = fraction;
				hit.mSubShapeID2 = mShape->EncodeSubShapeID(mSubShapeIDCreator, inX, inY, inTriangle);
				mCollector.AddHit(hit);
			}
		}

		CastRayCollector &		mCollector;
		Vec3					mRayOrigin;
		Vec3					mRayDirection;
		RayInvDirection			mRayInvDirection;
		EBackFaceMode			mBackFaceMode;
		const HeightFieldShape *mShape;
		SubShapeIDCreator		mSubShapeIDCreator;
		float					mDistanceStack[cStackSize];
	};

	Visitor visitor(this, inRay, inRayCastSettings, inSubShapeIDCreator, ioCollector);
	WalkHeightField(visitor);
}

void HeightFieldShape::CollidePoint(Vec3Arg inPoint, const SubShapeIDCreator &inSubShapeIDCreator, CollidePointCollector &ioCollector, const ShapeFilter &inShapeFilter) const
{
	// A height field doesn't have volume, so we can't test insideness
}

void HeightFieldShape::CollideSoftBodyVertices(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const CollideSoftBodyVertexIterator &inVertices, uint inNumVertices, int inCollidingShapeIndex) const
{
	MOSS_PROFILE_FUNCTION();

	struct Visitor : public CollideSoftBodyVerticesVsTriangles
	{
		using CollideSoftBodyVerticesVsTriangles::CollideSoftBodyVerticesVsTriangles;

		MOSS_INLINE bool	ShouldAbort() const
		{
			return false;
		}

		MOSS_INLINE bool	ShouldVisitRangeBlock([[maybe_unused]] int inStackTop) const
		{
			return mDistanceStack[inStackTop] < mClosestDistanceSq;
		}

		MOSS_INLINE int	VisitRangeBlock(Vec4Arg inBoundsMinX, Vec4Arg inBoundsMinY, Vec4Arg inBoundsMinZ, Vec4Arg inBoundsMaxX, Vec4Arg inBoundsMaxY, Vec4Arg inBoundsMaxZ, UVec4 &ioProperties, int inStackTop)
		{
			// Get distance to vertex
			Vec4 dist_sq = AABox4DistanceSqToPoint(mLocalPosition, inBoundsMinX, inBoundsMinY, inBoundsMinZ, inBoundsMaxX, inBoundsMaxY, inBoundsMaxZ);

			// Clear distance for invalid bounds
			dist_sq = Vec4::sSelect(Vec4::sReplicate(FLT_MAX), dist_sq, Vec4::sLessOrEqual(inBoundsMinY, inBoundsMaxY));

			// Sort so that highest values are first (we want to first process closer hits and we process stack top to bottom)
			return SortReverseAndStore(dist_sq, mClosestDistanceSq, ioProperties, &mDistanceStack[inStackTop]);
		}

		MOSS_INLINE void	VisitTriangle([[maybe_unused]] uint inX, [[maybe_unused]] uint inY, [[maybe_unused]] uint inTriangle, Vec3Arg inV0, Vec3Arg inV1, Vec3Arg inV2)
		{
			ProcessTriangle(inV0, inV1, inV2);
		}

		float			mDistanceStack[cStackSize];
	};

	Visitor visitor(inCenterOfMassTransform, inScale);

	for (CollideSoftBodyVertexIterator v = inVertices, sbv_end = inVertices + inNumVertices; v != sbv_end; ++v)
		if (v.GetInvMass() > 0.0f)
		{
			visitor.StartVertex(v);
			WalkHeightField(visitor);
			visitor.FinishVertex(v, inCollidingShapeIndex);
		}
}

void HeightFieldShape::sCastConvexVsHeightField(const ShapeCast &inShapeCast, const ShapeCastSettings &inShapeCastSettings, const Shape *inShape, Vec3Arg inScale, [[maybe_unused]] const ShapeFilter &inShapeFilter, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, CastShapeCollector &ioCollector)
{
	MOSS_PROFILE_FUNCTION();

	struct Visitor : public CastConvexVsTriangles
	{
		using CastConvexVsTriangles::CastConvexVsTriangles;

		MOSS_INLINE bool				ShouldAbort() const
		{
			return mCollector.ShouldEarlyOut();
		}

		MOSS_INLINE bool				ShouldVisitRangeBlock(int inStackTop) const
		{
			return mDistanceStack[inStackTop] < mCollector.GetPositiveEarlyOutFraction();
		}

		MOSS_INLINE int				VisitRangeBlock(Vec4Arg inBoundsMinX, Vec4Arg inBoundsMinY, Vec4Arg inBoundsMinZ, Vec4Arg inBoundsMaxX, Vec4Arg inBoundsMaxY, Vec4Arg inBoundsMaxZ, UVec4 &ioProperties, int inStackTop)
		{
			// Scale the bounding boxes of this node
			Vec4 bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z;
			AABox4Scale(mScale, inBoundsMinX, inBoundsMinY, inBoundsMinZ, inBoundsMaxX, inBoundsMaxY, inBoundsMaxZ, bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);

			// Enlarge them by the casted shape's box extents
			AABox4EnlargeWithExtent(mBoxExtent, bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);

			// Test bounds of 4 children
			Vec4 distance = RayAABox4(mBoxCenter, mInvDirection, bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);

			// Clear distance for invalid bounds
			distance = Vec4::sSelect(Vec4::sReplicate(FLT_MAX), distance, Vec4::sLessOrEqual(inBoundsMinY, inBoundsMaxY));

			// Sort so that highest values are first (we want to first process closer hits and we process stack top to bottom)
			return SortReverseAndStore(distance, mCollector.GetPositiveEarlyOutFraction(), ioProperties, &mDistanceStack[inStackTop]);
		}

		MOSS_INLINE void				VisitTriangle(uint inX, uint inY, uint inTriangle, Vec3Arg inV0, Vec3Arg inV1, Vec3Arg inV2)
		{
			// Create sub shape id for this part
			SubShapeID triangle_sub_shape_id = mShape2->EncodeSubShapeID(mSubShapeIDCreator2, inX, inY, inTriangle);

			// Determine active edges
			uint8 active_edges = mShape2->GetEdgeFlags(inX, inY, inTriangle);

			Cast(inV0, inV1, inV2, active_edges, triangle_sub_shape_id);
		}

		const HeightFieldShape *	mShape2;
		RayInvDirection				mInvDirection;
		Vec3						mBoxCenter;
		Vec3						mBoxExtent;
		SubShapeIDCreator			mSubShapeIDCreator2;
		float						mDistanceStack[cStackSize];
	};

	MOSS_ASSERT(inShape->GetSubType() == EShapeSubType::HeightField);
	const HeightFieldShape *shape = static_cast<const HeightFieldShape *>(inShape);

	Visitor visitor(inShapeCast, inShapeCastSettings, inScale, inCenterOfMassTransform2, inSubShapeIDCreator1, ioCollector);
	visitor.mShape2 = shape;
	visitor.mInvDirection.Set(inShapeCast.mDirection);
	visitor.mBoxCenter = inShapeCast.mShapeWorldBounds.GetCenter();
	visitor.mBoxExtent = inShapeCast.mShapeWorldBounds.GetExtent();
	visitor.mSubShapeIDCreator2 = inSubShapeIDCreator2;
	shape->WalkHeightField(visitor);
}

void HeightFieldShape::sCastSphereVsHeightField(const ShapeCast &inShapeCast, const ShapeCastSettings &inShapeCastSettings, const Shape *inShape, Vec3Arg inScale, [[maybe_unused]] const ShapeFilter &inShapeFilter, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, CastShapeCollector &ioCollector)
{
	MOSS_PROFILE_FUNCTION();

	struct Visitor : public CastSphereVsTriangles
	{
		using CastSphereVsTriangles::CastSphereVsTriangles;

		MOSS_INLINE bool				ShouldAbort() const
		{
			return mCollector.ShouldEarlyOut();
		}

		MOSS_INLINE bool				ShouldVisitRangeBlock(int inStackTop) const
		{
			return mDistanceStack[inStackTop] < mCollector.GetPositiveEarlyOutFraction();
		}

		MOSS_INLINE int				VisitRangeBlock(Vec4Arg inBoundsMinX, Vec4Arg inBoundsMinY, Vec4Arg inBoundsMinZ, Vec4Arg inBoundsMaxX, Vec4Arg inBoundsMaxY, Vec4Arg inBoundsMaxZ, UVec4 &ioProperties, int inStackTop)
		{
			// Scale the bounding boxes of this node
			Vec4 bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z;
			AABox4Scale(mScale, inBoundsMinX, inBoundsMinY, inBoundsMinZ, inBoundsMaxX, inBoundsMaxY, inBoundsMaxZ, bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);

			// Enlarge them by the radius of the sphere
			AABox4EnlargeWithExtent(Vec3::sReplicate(mRadius), bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);

			// Test bounds of 4 children
			Vec4 distance = RayAABox4(mStart, mInvDirection, bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);

			// Clear distance for invalid bounds
			distance = Vec4::sSelect(Vec4::sReplicate(FLT_MAX), distance, Vec4::sLessOrEqual(inBoundsMinY, inBoundsMaxY));

			// Sort so that highest values are first (we want to first process closer hits and we process stack top to bottom)
			return SortReverseAndStore(distance, mCollector.GetPositiveEarlyOutFraction(), ioProperties, &mDistanceStack[inStackTop]);
		}

		MOSS_INLINE void				VisitTriangle(uint inX, uint inY, uint inTriangle, Vec3Arg inV0, Vec3Arg inV1, Vec3Arg inV2)
		{
			// Create sub shape id for this part
			SubShapeID triangle_sub_shape_id = mShape2->EncodeSubShapeID(mSubShapeIDCreator2, inX, inY, inTriangle);

			// Determine active edges
			uint8 active_edges = mShape2->GetEdgeFlags(inX, inY, inTriangle);

			Cast(inV0, inV1, inV2, active_edges, triangle_sub_shape_id);
		}

		const HeightFieldShape *	mShape2;
		RayInvDirection				mInvDirection;
		SubShapeIDCreator			mSubShapeIDCreator2;
		float						mDistanceStack[cStackSize];
	};

	MOSS_ASSERT(inShape->GetSubType() == EShapeSubType::HeightField);
	const HeightFieldShape *shape = static_cast<const HeightFieldShape *>(inShape);

	Visitor visitor(inShapeCast, inShapeCastSettings, inScale, inCenterOfMassTransform2, inSubShapeIDCreator1, ioCollector);
	visitor.mShape2 = shape;
	visitor.mInvDirection.Set(inShapeCast.mDirection);
	visitor.mSubShapeIDCreator2 = inSubShapeIDCreator2;
	shape->WalkHeightField(visitor);
}

struct HeightFieldShape::HSGetTrianglesContext
{
			HSGetTrianglesContext(const HeightFieldShape *inShape, const AABox &inBox, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale) :
		mDecodeCtx(inShape),
		mShape(inShape),
		mLocalBox(Mat44::sInverseRotationTranslation(inRotation, inPositionCOM), inBox),
		mHeightFieldScale(inScale),
		mLocalToWorld(Mat44::sRotationTranslation(inRotation, inPositionCOM) * Mat44::sScale(inScale)),
		mIsInsideOut(ScaleHelpers::IsInsideOut(inScale))
	{
	}

	bool	ShouldAbort() const
	{
		return mShouldAbort;
	}

	bool	ShouldVisitRangeBlock([[maybe_unused]] int inStackTop) const
	{
		return true;
	}

	int		VisitRangeBlock(Vec4Arg inBoundsMinX, Vec4Arg inBoundsMinY, Vec4Arg inBoundsMinZ, Vec4Arg inBoundsMaxX, Vec4Arg inBoundsMaxY, Vec4Arg inBoundsMaxZ, UVec4 &ioProperties, [[maybe_unused]] int inStackTop) const
	{
		// Scale the bounding boxes of this node
		Vec4 bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z;
		AABox4Scale(mHeightFieldScale, inBoundsMinX, inBoundsMinY, inBoundsMinZ, inBoundsMaxX, inBoundsMaxY, inBoundsMaxZ, bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);

		// Test which nodes collide
		UVec4 collides = AABox4VsBox(mLocalBox, bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);

		// Filter out invalid bounding boxes
		collides = UVec4::sAnd(collides, Vec4::sLessOrEqual(inBoundsMinY, inBoundsMaxY));

		return CountAndSortTrues(collides, ioProperties);
	}

	void	VisitTriangle(uint inX, uint inY, [[maybe_unused]] uint inTriangle, Vec3Arg inV0, Vec3Arg inV1, Vec3Arg inV2)
	{
		// When the buffer is full and we cannot process the triangles, abort the height field walk. The next time GetTrianglesNext is called we will continue here.
		if (mNumTrianglesFound + 1 > mMaxTrianglesRequested)
		{
			mShouldAbort = true;
			return;
		}

		// Store vertices as Float3
		if (mIsInsideOut)
		{
			// Reverse vertices
			(mLocalToWorld * inV0).StoreFloat3(mTriangleVertices++);
			(mLocalToWorld * inV2).StoreFloat3(mTriangleVertices++);
			(mLocalToWorld * inV1).StoreFloat3(mTriangleVertices++);
		}
		else
		{
			// Normal scale
			(mLocalToWorld * inV0).StoreFloat3(mTriangleVertices++);
			(mLocalToWorld * inV1).StoreFloat3(mTriangleVertices++);
			(mLocalToWorld * inV2).StoreFloat3(mTriangleVertices++);
		}

		// Decode material
		if (mMaterials != nullptr)
			*mMaterials++ = mShape->GetMaterial(inX, inY);

		// Accumulate triangles found
		mNumTrianglesFound++;
	}

	DecodingContext				mDecodeCtx;
	const HeightFieldShape *	mShape;
	OrientedBox					mLocalBox;
	Vec3						mHeightFieldScale;
	Mat44						mLocalToWorld;
	int							mMaxTrianglesRequested;
	Float3 *					mTriangleVertices;
	int							mNumTrianglesFound;
	const PhysicsMaterial **	mMaterials;
	bool						mShouldAbort;
	bool						mIsInsideOut;
};

void HeightFieldShape::GetTrianglesStart(GetTrianglesContext &ioContext, const AABox &inBox, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale) const
{
	static_assert(sizeof(HSGetTrianglesContext) <= sizeof(GetTrianglesContext), "GetTrianglesContext too small");
	MOSS_ASSERT(IsAligned(&ioContext, alignof(HSGetTrianglesContext)));

	new (&ioContext) HSGetTrianglesContext(this, inBox, inPositionCOM, inRotation, inScale);
}

int HeightFieldShape::GetTrianglesNext(GetTrianglesContext &ioContext, int inMaxTrianglesRequested, Float3 *outTriangleVertices, const PhysicsMaterial **outMaterials) const
{
	static_assert(cGetTrianglesMinTrianglesRequested >= 1, "cGetTrianglesMinTrianglesRequested is too small");
	MOSS_ASSERT(inMaxTrianglesRequested >= cGetTrianglesMinTrianglesRequested);

	// Check if we're done
	HSGetTrianglesContext &context = (HSGetTrianglesContext &)ioContext;
	if (context.mDecodeCtx.IsDoneWalking())
		return 0;

	// Store parameters on context
	context.mMaxTrianglesRequested = inMaxTrianglesRequested;
	context.mTriangleVertices = outTriangleVertices;
	context.mMaterials = outMaterials;
	context.mShouldAbort = false; // Reset the abort flag
	context.mNumTrianglesFound = 0;

	// Continue (or start) walking the height field
	context.mDecodeCtx.WalkHeightField(context);
	return context.mNumTrianglesFound;
}

void HeightFieldShape::sCollideConvexVsHeightField(const Shape *inShape1, const Shape *inShape2, Vec3Arg inScale1, Vec3Arg inScale2, Mat44Arg inCenterOfMassTransform1, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, const CollideShapeSettings &inCollideShapeSettings, CollideShapeCollector &ioCollector, [[maybe_unused]] const ShapeFilter &inShapeFilter)
{
	MOSS_PROFILE_FUNCTION();

	// Get the shapes
	MOSS_ASSERT(inShape1->GetType() == EShapeType::Convex);
	MOSS_ASSERT(inShape2->GetType() == EShapeType::HeightField);
	const ConvexShape *shape1 = static_cast<const ConvexShape *>(inShape1);
	const HeightFieldShape *shape2 = static_cast<const HeightFieldShape *>(inShape2);

	struct Visitor : public CollideConvexVsTriangles
	{
		using CollideConvexVsTriangles::CollideConvexVsTriangles;

		MOSS_INLINE bool				ShouldAbort() const
		{
			return mCollector.ShouldEarlyOut();
		}

		MOSS_INLINE bool				ShouldVisitRangeBlock([[maybe_unused]] int inStackTop) const
		{
			return true;
		}

		MOSS_INLINE int				VisitRangeBlock(Vec4Arg inBoundsMinX, Vec4Arg inBoundsMinY, Vec4Arg inBoundsMinZ, Vec4Arg inBoundsMaxX, Vec4Arg inBoundsMaxY, Vec4Arg inBoundsMaxZ, UVec4 &ioProperties, [[maybe_unused]] int inStackTop) const
		{
			// Scale the bounding boxes of this node
			Vec4 bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z;
			AABox4Scale(mScale2, inBoundsMinX, inBoundsMinY, inBoundsMinZ, inBoundsMaxX, inBoundsMaxY, inBoundsMaxZ, bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);

			// Test which nodes collide
			UVec4 collides = AABox4VsBox(mBoundsOf1InSpaceOf2, bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);

			// Filter out invalid bounding boxes
			collides = UVec4::sAnd(collides, Vec4::sLessOrEqual(inBoundsMinY, inBoundsMaxY));

			return CountAndSortTrues(collides, ioProperties);
		}

		MOSS_INLINE void				VisitTriangle(uint inX, uint inY, uint inTriangle, Vec3Arg inV0, Vec3Arg inV1, Vec3Arg inV2)
		{
			// Create ID for triangle
			SubShapeID triangle_sub_shape_id = mShape2->EncodeSubShapeID(mSubShapeIDCreator2, inX, inY, inTriangle);

			// Determine active edges
			uint8 active_edges = mShape2->GetEdgeFlags(inX, inY, inTriangle);

			Collide(inV0, inV1, inV2, active_edges, triangle_sub_shape_id);
		}

		const HeightFieldShape *	mShape2;
		SubShapeIDCreator			mSubShapeIDCreator2;
	};

	Visitor visitor(shape1, inScale1, inScale2, inCenterOfMassTransform1, inCenterOfMassTransform2, inSubShapeIDCreator1.GetID(), inCollideShapeSettings, ioCollector);
	visitor.mShape2 = shape2;
	visitor.mSubShapeIDCreator2 = inSubShapeIDCreator2;
	shape2->WalkHeightField(visitor);
}

void HeightFieldShape::sCollideSphereVsHeightField(const Shape *inShape1, const Shape *inShape2, Vec3Arg inScale1, Vec3Arg inScale2, Mat44Arg inCenterOfMassTransform1, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, const CollideShapeSettings &inCollideShapeSettings, CollideShapeCollector &ioCollector, [[maybe_unused]] const ShapeFilter &inShapeFilter)
{
	MOSS_PROFILE_FUNCTION();

	// Get the shapes
	MOSS_ASSERT(inShape1->GetSubType() == EShapeSubType::Sphere);
	MOSS_ASSERT(inShape2->GetType() == EShapeType::HeightField);
	const SphereShape *shape1 = static_cast<const SphereShape *>(inShape1);
	const HeightFieldShape *shape2 = static_cast<const HeightFieldShape *>(inShape2);

	struct Visitor : public CollideSphereVsTriangles
	{
		using CollideSphereVsTriangles::CollideSphereVsTriangles;

		MOSS_INLINE bool				ShouldAbort() const
		{
			return mCollector.ShouldEarlyOut();
		}

		MOSS_INLINE bool				ShouldVisitRangeBlock([[maybe_unused]] int inStackTop) const
		{
			return true;
		}

		MOSS_INLINE int				VisitRangeBlock(Vec4Arg inBoundsMinX, Vec4Arg inBoundsMinY, Vec4Arg inBoundsMinZ, Vec4Arg inBoundsMaxX, Vec4Arg inBoundsMaxY, Vec4Arg inBoundsMaxZ, UVec4 &ioProperties, [[maybe_unused]] int inStackTop) const
		{
			// Scale the bounding boxes of this node
			Vec4 bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z;
			AABox4Scale(mScale2, inBoundsMinX, inBoundsMinY, inBoundsMinZ, inBoundsMaxX, inBoundsMaxY, inBoundsMaxZ, bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);

			// Test which nodes collide
			UVec4 collides = AABox4VsSphere(mSphereCenterIn2, mRadiusPlusMaxSeparationSq, bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);

			// Filter out invalid bounding boxes
			collides = UVec4::sAnd(collides, Vec4::sLessOrEqual(inBoundsMinY, inBoundsMaxY));

			return CountAndSortTrues(collides, ioProperties);
		}

		MOSS_INLINE void				VisitTriangle(uint inX, uint inY, uint inTriangle, Vec3Arg inV0, Vec3Arg inV1, Vec3Arg inV2)
		{
			// Create ID for triangle
			SubShapeID triangle_sub_shape_id = mShape2->EncodeSubShapeID(mSubShapeIDCreator2, inX, inY, inTriangle);

			// Determine active edges
			uint8 active_edges = mShape2->GetEdgeFlags(inX, inY, inTriangle);

			Collide(inV0, inV1, inV2, active_edges, triangle_sub_shape_id);
		}

		const HeightFieldShape *	mShape2;
		SubShapeIDCreator			mSubShapeIDCreator2;
	};

	Visitor visitor(shape1, inScale1, inScale2, inCenterOfMassTransform1, inCenterOfMassTransform2, inSubShapeIDCreator1.GetID(), inCollideShapeSettings, ioCollector);
	visitor.mShape2 = shape2;
	visitor.mSubShapeIDCreator2 = inSubShapeIDCreator2;
	shape2->WalkHeightField(visitor);
}

void HeightFieldShape::SaveBinaryState(StreamOut &inStream) const
{
	Shape::SaveBinaryState(inStream);

	inStream.Write(mOffset);
	inStream.Write(mScale);
	inStream.Write(mSampleCount);
	inStream.Write(mBlockSize);
	inStream.Write(mBitsPerSample);
	inStream.Write(mMinSample);
	inStream.Write(mMaxSample);
	inStream.Write(mMaterialIndices);
	inStream.Write(mNumBitsPerMaterialIndex);

	if (mRangeBlocks != nullptr)
	{
		inStream.Write(true);
		inStream.WriteBytes(mRangeBlocks, mRangeBlocksSize * sizeof(RangeBlock) + mHeightSamplesSize + mActiveEdgesSize);
	}
	else
	{
		inStream.Write(false);
	}
}

void HeightFieldShape::RestoreBinaryState(StreamIn &inStream)
{
	Shape::RestoreBinaryState(inStream);

	inStream.Read(mOffset);
	inStream.Read(mScale);
	inStream.Read(mSampleCount);
	inStream.Read(mBlockSize);
	inStream.Read(mBitsPerSample);
	inStream.Read(mMinSample);
	inStream.Read(mMaxSample);
	inStream.Read(mMaterialIndices);
	inStream.Read(mNumBitsPerMaterialIndex);

	// We don't have the exact number of reserved materials anymore, but ensure that our array is big enough
	// TODO: Next time when we bump the binary serialization format of this class we should store the capacity and allocate the right amount, for now we accept a little bit of waste
	mMaterials.reserve(PhysicsMaterialList::size_type(1) << mNumBitsPerMaterialIndex);

	CacheValues();

	bool has_heights = false;
	inStream.Read(has_heights);
	if (has_heights)
	{
		AllocateBuffers();
		inStream.ReadBytes(mRangeBlocks, mRangeBlocksSize * sizeof(RangeBlock) + mHeightSamplesSize + mActiveEdgesSize);
	}
}

void HeightFieldShape::SaveMaterialState(PhysicsMaterialList &outMaterials) const
{
	outMaterials = mMaterials;
}

void HeightFieldShape::RestoreMaterialState(const PhysicsMaterialRefC *inMaterials, uint inNumMaterials)
{
	mMaterials.assign(inMaterials, inMaterials + inNumMaterials);
}

Shape::Stats HeightFieldShape::GetStats() const
{
	return Stats(
		sizeof(*this)
			+ mMaterials.size() * sizeof(Ref<PhysicsMaterial>)
			+ mRangeBlocksSize * sizeof(RangeBlock)
			+ mHeightSamplesSize * sizeof(uint8)
			+ mActiveEdgesSize * sizeof(uint8)
			+ mMaterialIndices.size() * sizeof(uint8),
		mHeightSamplesSize == 0? 0 : Square(mSampleCount - 1) * 2);
}

void HeightFieldShape::sRegister()
{
	ShapeFunctions &f = ShapeFunctions::sGet(EShapeSubType::HeightField);
	f.mConstruct = []() -> Shape * { return new HeightFieldShape; };
	f.mColor = Color::sPurple;

	for (EShapeSubType s : sConvexSubShapeTypes)
	{
		CollisionDispatch::sRegisterCollideShape(s, EShapeSubType::HeightField, sCollideConvexVsHeightField);
		CollisionDispatch::sRegisterCastShape(s, EShapeSubType::HeightField, sCastConvexVsHeightField);

		CollisionDispatch::sRegisterCastShape(EShapeSubType::HeightField, s, CollisionDispatch::sReversedCastShape);
		CollisionDispatch::sRegisterCollideShape(EShapeSubType::HeightField, s, CollisionDispatch::sReversedCollideShape);
	}

	// Specialized collision functions
	CollisionDispatch::sRegisterCollideShape(EShapeSubType::Sphere, EShapeSubType::HeightField, sCollideSphereVsHeightField);
	CollisionDispatch::sRegisterCastShape(EShapeSubType::Sphere, EShapeSubType::HeightField, sCastSphereVsHeightField);
}

/*													*/

// Get header for tree
static MOSS_INLINE const NodeCodec::Header *sGetNodeHeader(const ByteBuffer &inTree)
{
	return inTree.Get<NodeCodec::Header>(0);
}

// Get header for triangles
static MOSS_INLINE const TriangleCodec::TriangleHeader *sGetTriangleHeader(const ByteBuffer &inTree)
{
	return inTree.Get<TriangleCodec::TriangleHeader>(NodeCodec::HeaderSize);
}

MeshShapeSettings::MeshShapeSettings(const TriangleList &inTriangles, PhysicsMaterialList inMaterials) :
	mMaterials(std::move(inMaterials))
{
	Indexify(inTriangles, mTriangleVertices, mIndexedTriangles);

	Sanitize();
}

MeshShapeSettings::MeshShapeSettings(VertexList inVertices, IndexedTriangleList inTriangles, PhysicsMaterialList inMaterials) :
	mTriangleVertices(std::move(inVertices)),
	mIndexedTriangles(std::move(inTriangles)),
	mMaterials(std::move(inMaterials))
{
	Sanitize();
}

void MeshShapeSettings::Sanitize()
{
	// Remove degenerate and duplicate triangles
	TSet<IndexedTriangle> triangles;
	triangles.reserve(TSet<IndexedTriangle>::size_type(mIndexedTriangles.size()));
	TriangleCodec::ValidationContext validation_ctx(mIndexedTriangles, mTriangleVertices);
	for (int t = (int)mIndexedTriangles.size() - 1; t >= 0; --t)
	{
		const IndexedTriangle &tri = mIndexedTriangles[t];

		if (tri.IsDegenerate(mTriangleVertices)						// Degenerate triangle
			|| validation_ctx.IsDegenerate(tri)						// Triangle is degenerate in the quantized space
			|| !triangles.insert(tri.GetLowestIndexFirst()).second) // Duplicate triangle
		{
			// The order of triangles doesn't matter (gets reordered while building the tree), so we can just swap the last triangle into this slot
			mIndexedTriangles[t] = mIndexedTriangles.back();
			mIndexedTriangles.pop_back();
		}
	}
}

ShapeSettings::ShapeResult MeshShapeSettings::Create() const
{
	if (mCachedResult.IsEmpty())
		Ref<Shape> shape = new MeshShape(*this, mCachedResult);
	return mCachedResult;
}

MeshShape::MeshShape(const MeshShapeSettings &inSettings, ShapeResult &outResult) :
	Shape(EShapeType::Mesh, EShapeSubType::Mesh, inSettings, outResult)
{
	// Check if there are any triangles
	if (inSettings.mIndexedTriangles.empty())
	{
		outResult.SetError("Need triangles to create a mesh shape!");
		return;
	}

	// Check triangles
	TriangleCodec::ValidationContext validation_ctx(inSettings.mIndexedTriangles, inSettings.mTriangleVertices);
	for (int t = (int)inSettings.mIndexedTriangles.size() - 1; t >= 0; --t)
	{
		const IndexedTriangle &triangle = inSettings.mIndexedTriangles[t];
		if (triangle.IsDegenerate(inSettings.mTriangleVertices)
			|| validation_ctx.IsDegenerate(triangle))
		{
			outResult.SetError(StringFormat("Triangle %d is degenerate!", t));
			return;
		}
		else
		{
			// Check vertex indices
			for (uint32 idx : triangle.mIdx)
				if (idx >= inSettings.mTriangleVertices.size())
				{
					outResult.SetError(StringFormat("Vertex index %u is beyond vertex list (size: %u)", idx, (uint)inSettings.mTriangleVertices.size()));
					return;
				}
		}
	}

	// Copy materials
	mMaterials = inSettings.mMaterials;
	if (!mMaterials.empty())
	{
		// Validate materials
		if (mMaterials.size() > (1 << FLAGS_MATERIAL_BITS))
		{
			outResult.SetError(StringFormat("Supporting max %d materials per mesh", 1 << FLAGS_MATERIAL_BITS));
			return;
		}
		for (const IndexedTriangle &t : inSettings.mIndexedTriangles)
			if (t.mMaterialIndex >= mMaterials.size())
			{
				outResult.SetError(StringFormat("Triangle material %u is beyond material list (size: %u)", t.mMaterialIndex, (uint)mMaterials.size()));
				return;
			}
	}
	else
	{
		// No materials assigned, validate that all triangles use material index 0
		for (const IndexedTriangle &t : inSettings.mIndexedTriangles)
			if (t.mMaterialIndex != 0)
			{
				outResult.SetError("No materials present, all triangles should have material index 0");
				return;
			}
	}

	// Check max triangles
	if (inSettings.mMaxTrianglesPerLeaf < 1 || inSettings.mMaxTrianglesPerLeaf > MaxTrianglesPerLeaf)
	{
		outResult.SetError("Invalid max triangles per leaf");
		return;
	}

	// Fill in active edge bits
	IndexedTriangleList indexed_triangles = inSettings.mIndexedTriangles; // Copy indices since we're adding the 'active edge' flag
	sFindActiveEdges(inSettings, indexed_triangles);

	// Create triangle splitter
	union Storage
	{
									Storage() { }
									~Storage() { }

		TriangleSplitterBinning		mBinning;
		TriangleSplitterMean		mMean;
	};
	Storage storage;
	TriangleSplitter *splitter = nullptr;
	switch (inSettings.mBuildQuality)
	{
	case MeshShapeSettings::EBuildQuality::FavorRuntimePerformance:
		splitter = new (&storage.mBinning) TriangleSplitterBinning(inSettings.mTriangleVertices, indexed_triangles);
		break;

	case MeshShapeSettings::EBuildQuality::FavorBuildSpeed:
		splitter = new (&storage.mMean) TriangleSplitterMean(inSettings.mTriangleVertices, indexed_triangles);
		break;

	default:
		MOSS_ASSERT(false);
		break;
	}

	// Build tree
	AABBTreeBuilder builder(*splitter, inSettings.mMaxTrianglesPerLeaf);
	AABBTreeBuilderStats builder_stats;
	const AABBTreeBuilder::Node *root = builder.Build(builder_stats);
	splitter->~TriangleSplitter();

	// Convert to buffer
	AABBTreeToBuffer<TriangleCodec, NodeCodec> buffer;
	const char *error = nullptr;
	if (!buffer.Convert(builder.GetTriangles(), builder.GetNodes(), inSettings.mTriangleVertices, root, inSettings.mPerTriangleUserData, error))
	{
		outResult.SetError(error);
		return;
	}

	// Move data to this class
	mTree.swap(buffer.GetBuffer());

	// Check if we're not exceeding the amount of sub shape id bits
	if (GetSubShapeIDBitsRecursive() > SubShapeID::MaxBits)
	{
		outResult.SetError("Mesh is too big and exceeds the amount of available sub shape ID bits");
		return;
	}

	outResult.Set(this);
}

void MeshShape::sFindActiveEdges(const MeshShapeSettings &inSettings, IndexedTriangleList &ioIndices)
{
	// Check if we're requested to make all edges active
	if (inSettings.mActiveEdgeCosThresholdAngle < 0.0f)
	{
		for (IndexedTriangle &triangle : ioIndices)
			triangle.mMaterialIndex |= 0b111 << FLAGS_ACTIVE_EGDE_SHIFT;
		return;
	}

	// A struct to hold the two vertex indices of an edge
	struct Edge
	{
				Edge(int inIdx1, int inIdx2) : mIdx1(min(inIdx1, inIdx2)), mIdx2(max(inIdx1, inIdx2)) { }

		uint	GetIndexInTriangle(const IndexedTriangle &inTriangle) const
		{
			for (uint edge_idx = 0; edge_idx < 3; ++edge_idx)
			{
				Edge edge(inTriangle.mIdx[edge_idx], inTriangle.mIdx[(edge_idx + 1) % 3]);
				if (*this == edge)
					return edge_idx;
			}

			MOSS_ASSERT(false);
			return ~uint(0);
		}

		bool	operator == (const Edge &inRHS) const
		{
			return mIdx1 == inRHS.mIdx1 && mIdx2 == inRHS.mIdx2;
		}

		uint64	GetHash() const
		{
			static_assert(sizeof(*this) == 2 * sizeof(int), "No padding expected");
			return HashBytes(this, sizeof(*this));
		}

		int		mIdx1;
		int		mIdx2;
	};

	// A struct to hold the triangles that are connected to an edge
	struct TriangleIndices
	{
		uint	mNumTriangles = 0;
		uint	mTriangleIndices[2];
	};

	// Build a list of edge to triangles
	using EdgeToTriangle = TMap<Edge, TriangleIndices>;
	EdgeToTriangle edge_to_triangle;
	edge_to_triangle.reserve(EdgeToTriangle::size_type(ioIndices.size() * 3));
	for (uint triangle_idx = 0; triangle_idx < ioIndices.size(); ++triangle_idx)
	{
		IndexedTriangle &triangle = ioIndices[triangle_idx];
		for (uint edge_idx = 0; edge_idx < 3; ++edge_idx)
		{
			Edge edge(triangle.mIdx[edge_idx], triangle.mIdx[(edge_idx + 1) % 3]);
			EdgeToTriangle::iterator edge_to_triangle_it = edge_to_triangle.try_emplace(edge, TriangleIndices()).first;
			TriangleIndices &indices = edge_to_triangle_it->second;
			if (indices.mNumTriangles < 2)
			{
				// Store index of triangle that connects to this edge
				indices.mTriangleIndices[indices.mNumTriangles] = triangle_idx;
				indices.mNumTriangles++;
			}
			else
			{
				// 3 or more triangles share an edge, mark this edge as active
				uint32 mask = 1 << (edge_idx + FLAGS_ACTIVE_EGDE_SHIFT);
				MOSS_ASSERT((triangle.mMaterialIndex & mask) == 0);
				triangle.mMaterialIndex |= mask;
			}
		}
	}

	// Walk over all edges and determine which ones are active
	for (const EdgeToTriangle::value_type &edge : edge_to_triangle)
	{
		uint num_active = 0;
		if (edge.second.mNumTriangles == 1)
		{
			// Edge is not shared, it is an active edge
			num_active = 1;
		}
		else if (edge.second.mNumTriangles == 2)
		{
			// Simple shared edge, determine if edge is active based on the two adjacent triangles
			const IndexedTriangle &triangle1 = ioIndices[edge.second.mTriangleIndices[0]];
			const IndexedTriangle &triangle2 = ioIndices[edge.second.mTriangleIndices[1]];

			// Find which edge this is for both triangles
			uint edge_idx1 = edge.first.GetIndexInTriangle(triangle1);
			uint edge_idx2 = edge.first.GetIndexInTriangle(triangle2);

			// Construct a plane for triangle 1 (e1 = edge vertex 1, e2 = edge vertex 2, op = opposing vertex)
			Vec3 triangle1_e1 = Vec3(inSettings.mTriangleVertices[triangle1.mIdx[edge_idx1]]);
			Vec3 triangle1_e2 = Vec3(inSettings.mTriangleVertices[triangle1.mIdx[(edge_idx1 + 1) % 3]]);
			Vec3 triangle1_op = Vec3(inSettings.mTriangleVertices[triangle1.mIdx[(edge_idx1 + 2) % 3]]);
			Plane triangle1_plane = Plane::sFromPointsCCW(triangle1_e1, triangle1_e2, triangle1_op);

			// Construct a plane for triangle 2
			Vec3 triangle2_e1 = Vec3(inSettings.mTriangleVertices[triangle2.mIdx[edge_idx2]]);
			Vec3 triangle2_e2 = Vec3(inSettings.mTriangleVertices[triangle2.mIdx[(edge_idx2 + 1) % 3]]);
			Vec3 triangle2_op = Vec3(inSettings.mTriangleVertices[triangle2.mIdx[(edge_idx2 + 2) % 3]]);
			Plane triangle2_plane = Plane::sFromPointsCCW(triangle2_e1, triangle2_e2, triangle2_op);

			// Determine if the edge is active
			num_active = ActiveEdges::IsEdgeActive(triangle1_plane.GetNormal(), triangle2_plane.GetNormal(), triangle1_e2 - triangle1_e1, inSettings.mActiveEdgeCosThresholdAngle)? 2 : 0;
		}
		else
		{
			// More edges incoming, we've already marked all edges beyond the 2nd as active
			num_active = 2;
		}

		// Mark edges of all original triangles active
		for (uint i = 0; i < num_active; ++i)
		{
			uint triangle_idx = edge.second.mTriangleIndices[i];
			IndexedTriangle &triangle = ioIndices[triangle_idx];
			uint edge_idx = edge.first.GetIndexInTriangle(triangle);
			uint32 mask = 1 << (edge_idx + FLAGS_ACTIVE_EGDE_SHIFT);
			MOSS_ASSERT((triangle.mMaterialIndex & mask) == 0);
			triangle.mMaterialIndex |= mask;
		}
	}
}

MassProperties MeshShape::GetMassProperties() const
{
	// We cannot calculate the volume for an arbitrary mesh, so we return invalid mass properties.
	// If you want your mesh to be dynamic, then you should provide the mass properties yourself when
	// creating a Body:
	//
	// BodyCreationSettings::mOverrideMassProperties = EOverrideMassProperties::MassAndInertiaProvided;
	// BodyCreationSettings::mMassPropertiesOverride.SetMassAndInertiaOfSolidBox(Vec3::sOne(), 1000.0f);
	//
	// Note that for a mesh shape to simulate properly, it is best if the mesh is manifold
	// (i.e. closed, all edges shared by only two triangles, consistent winding order).
	return MassProperties();
}

void MeshShape::DecodeSubShapeID(const SubShapeID &inSubShapeID, const void *&outTriangleBlock, uint32 &outTriangleIndex) const
{
	// Get block
	SubShapeID triangle_idx_subshape_id;
	uint32 block_id = inSubShapeID.PopID(NodeCodec::DecodingContext::sTriangleBlockIDBits(sGetNodeHeader(mTree)), triangle_idx_subshape_id);
	outTriangleBlock = NodeCodec::DecodingContext::sGetTriangleBlockStart(&mTree[0], block_id);

	// Fetch the triangle index
	SubShapeID remainder;
	outTriangleIndex = triangle_idx_subshape_id.PopID(NumTriangleBits, remainder);
	MOSS_ASSERT(remainder.IsEmpty(), "Invalid subshape ID");
}

uint MeshShape::GetMaterialIndex(const SubShapeID &inSubShapeID) const
{
	// Decode ID
	const void *block_start;
	uint32 triangle_idx;
	DecodeSubShapeID(inSubShapeID, block_start, triangle_idx);

	// Fetch the flags
	uint8 flags = TriangleCodec::DecodingContext::sGetFlags(block_start, triangle_idx);
	return flags & FLAGS_MATERIAL_MASK;
}

const PhysicsMaterial *MeshShape::GetMaterial(const SubShapeID &inSubShapeID) const
{
	// Return the default material if there are no materials on this shape
	if (mMaterials.empty())
		return PhysicsMaterial::sDefault;

	return mMaterials[GetMaterialIndex(inSubShapeID)];
}

Vec3 MeshShape::GetSurfaceNormal(const SubShapeID &inSubShapeID, Vec3Arg inLocalSurfacePosition) const
{
	// Decode ID
	const void *block_start;
	uint32 triangle_idx;
	DecodeSubShapeID(inSubShapeID, block_start, triangle_idx);

	// Decode triangle
	Vec3 v1, v2, v3;
	const TriangleCodec::DecodingContext triangle_ctx(sGetTriangleHeader(mTree));
	triangle_ctx.GetTriangle(block_start, triangle_idx, v1, v2, v3);

	// Calculate normal
	return (v3 - v2).Cross(v1 - v2).Normalized();
}

void MeshShape::GetSupportingFace(const SubShapeID &inSubShapeID, Vec3Arg inDirection, Vec3Arg inScale, Mat44Arg inCenterOfMassTransform, SupportingFace &outVertices) const
{
	// Decode ID
	const void *block_start;
	uint32 triangle_idx;
	DecodeSubShapeID(inSubShapeID, block_start, triangle_idx);

	// Decode triangle
	const TriangleCodec::DecodingContext triangle_ctx(sGetTriangleHeader(mTree));
	outVertices.resize(3);
	triangle_ctx.GetTriangle(block_start, triangle_idx, outVertices[0], outVertices[1], outVertices[2]);

	// Flip triangle if scaled inside out
	if (ScaleHelpers::IsInsideOut(inScale))
		std::swap(outVertices[1], outVertices[2]);

	// Calculate transform with scale
	Mat44 transform = inCenterOfMassTransform.PreScaled(inScale);

	// Transform to world space
	for (Vec3 &v : outVertices)
		v = transform * v;
}

AABox MeshShape::GetLocalBounds() const
{
	const NodeCodec::Header *header = sGetNodeHeader(mTree);
	return AABox(Vec3::sLoadFloat3Unsafe(header->mRootBoundsMin), Vec3::sLoadFloat3Unsafe(header->mRootBoundsMax));
}

uint MeshShape::GetSubShapeIDBitsRecursive() const
{
	return NodeCodec::DecodingContext::sTriangleBlockIDBits(sGetNodeHeader(mTree)) + NumTriangleBits;
}

template <class Visitor>
MOSS_INLINE void MeshShape::WalkTree(Visitor &ioVisitor) const
{
	const NodeCodec::Header *header = sGetNodeHeader(mTree);
	NodeCodec::DecodingContext node_ctx(header);

	const TriangleCodec::DecodingContext triangle_ctx(sGetTriangleHeader(mTree));
	const uint8 *buffer_start = &mTree[0];
	node_ctx.WalkTree(buffer_start, triangle_ctx, ioVisitor);
}

template <class Visitor>
MOSS_INLINE void MeshShape::WalkTreePerTriangle(const SubShapeIDCreator &inSubShapeIDCreator2, Visitor &ioVisitor) const
{
	struct ChainedVisitor
	{
		MOSS_INLINE			ChainedVisitor(Visitor &ioVisitor, const SubShapeIDCreator &inSubShapeIDCreator2, uint inTriangleBlockIDBits) :
			mVisitor(ioVisitor),
			mSubShapeIDCreator2(inSubShapeIDCreator2),
			mTriangleBlockIDBits(inTriangleBlockIDBits)
		{
		}

		MOSS_INLINE bool		ShouldAbort() const
		{
			return mVisitor.ShouldAbort();
		}

		MOSS_INLINE bool		ShouldVisitNode(int inStackTop) const
		{
			return mVisitor.ShouldVisitNode(inStackTop);
		}

		MOSS_INLINE int		VisitNodes(Vec4Arg inBoundsMinX, Vec4Arg inBoundsMinY, Vec4Arg inBoundsMinZ, Vec4Arg inBoundsMaxX, Vec4Arg inBoundsMaxY, Vec4Arg inBoundsMaxZ, UVec4 &ioProperties, int inStackTop)
		{
			return mVisitor.VisitNodes(inBoundsMinX, inBoundsMinY, inBoundsMinZ, inBoundsMaxX, inBoundsMaxY, inBoundsMaxZ, ioProperties, inStackTop);
		}

		MOSS_INLINE void		VisitTriangles(const TriangleCodec::DecodingContext &ioContext, const void *inTriangles, int inNumTriangles, uint32 inTriangleBlockID)
		{
			// Create ID for triangle block
			SubShapeIDCreator block_sub_shape_id = mSubShapeIDCreator2.PushID(inTriangleBlockID, mTriangleBlockIDBits);

			// Decode vertices and flags
			MOSS_ASSERT(inNumTriangles <= MaxTrianglesPerLeaf);
			Vec3 vertices[MaxTrianglesPerLeaf * 3];
			uint8 flags[MaxTrianglesPerLeaf];
			ioContext.Unpack(inTriangles, inNumTriangles, vertices, flags);

			int triangle_idx = 0;
			for (const Vec3 *v = vertices, *v_end = vertices + inNumTriangles * 3; v < v_end; v += 3, triangle_idx++)
			{
				// Determine active edges
				uint8 active_edges = (flags[triangle_idx] >> FLAGS_ACTIVE_EGDE_SHIFT) & FLAGS_ACTIVE_EDGE_MASK;

				// Create ID for triangle
				SubShapeIDCreator triangle_sub_shape_id = block_sub_shape_id.PushID(triangle_idx, NumTriangleBits);

				mVisitor.VisitTriangle(v[0], v[1], v[2], active_edges, triangle_sub_shape_id.GetID());

				// Check if we should early out now
				if (mVisitor.ShouldAbort())
					break;
			}
		}

		Visitor &			mVisitor;
		SubShapeIDCreator	mSubShapeIDCreator2;
		uint				mTriangleBlockIDBits;
	};

	ChainedVisitor visitor(ioVisitor, inSubShapeIDCreator2, NodeCodec::DecodingContext::sTriangleBlockIDBits(sGetNodeHeader(mTree)));
	WalkTree(visitor);
}

#ifndef MOSS_DEBUG_RENDERER
void MeshShape::Draw(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, Vec3Arg inScale, ColorArg inColor, bool inUseMaterialColors, bool inDrawWireframe) const
{
	// Reset the batch if we switch coloring mode
	if (mCachedTrianglesColoredPerGroup != sDrawTriangleGroups || mCachedUseMaterialColors != inUseMaterialColors)
	{
		mGeometry = nullptr;
		mCachedTrianglesColoredPerGroup = sDrawTriangleGroups;
		mCachedUseMaterialColors = inUseMaterialColors;
	}

	if (mGeometry == nullptr)
	{
		struct Visitor
		{
			MOSS_INLINE bool		ShouldAbort() const
			{
				return false;
			}

			MOSS_INLINE bool		ShouldVisitNode(int inStackTop) const
			{
				return true;
			}

			MOSS_INLINE int		VisitNodes(Vec4Arg inBoundsMinX, Vec4Arg inBoundsMinY, Vec4Arg inBoundsMinZ, Vec4Arg inBoundsMaxX, Vec4Arg inBoundsMaxY, Vec4Arg inBoundsMaxZ, UVec4 &ioProperties, int inStackTop)
			{
				UVec4 valid = UVec4::sOr(UVec4::sOr(Vec4::sLess(inBoundsMinX, inBoundsMaxX), Vec4::sLess(inBoundsMinY, inBoundsMaxY)), Vec4::sLess(inBoundsMinZ, inBoundsMaxZ));
				return CountAndSortTrues(valid, ioProperties);
			}

			MOSS_INLINE void		VisitTriangles(const TriangleCodec::DecodingContext &ioContext, const void *inTriangles, int inNumTriangles, [[maybe_unused]] uint32 inTriangleBlockID)
			{
				MOSS_ASSERT(inNumTriangles <= MaxTrianglesPerLeaf);
				Vec3 vertices[MaxTrianglesPerLeaf * 3];
				ioContext.Unpack(inTriangles, inNumTriangles, vertices);

				if (mDrawTriangleGroups || !mUseMaterialColors || mMaterials.empty())
				{
					// Single color for mesh
					Color color = mDrawTriangleGroups? Color::sGetDistinctColor(mColorIdx++) : (mUseMaterialColors? PhysicsMaterial::sDefault->GetDebugColor() : Color::sWhite);
					for (const Vec3 *v = vertices, *v_end = vertices + inNumTriangles * 3; v < v_end; v += 3)
						mTriangles.push_back({ v[0], v[1], v[2], color });
				}
				else
				{
					// Per triangle color
					uint8 flags[MaxTrianglesPerLeaf];
					TriangleCodec::DecodingContext::sGetFlags(inTriangles, inNumTriangles, flags);

					const uint8 *f = flags;
					for (const Vec3 *v = vertices, *v_end = vertices + inNumTriangles * 3; v < v_end; v += 3, f++)
						mTriangles.push_back({ v[0], v[1], v[2], mMaterials[*f & FLAGS_MATERIAL_MASK]->GetDebugColor() });
				}
			}

			TArray<DebugRenderer::Triangle> &		mTriangles;
			const PhysicsMaterialList &				mMaterials;
			bool									mUseMaterialColors;
			bool									mDrawTriangleGroups;
			int										mColorIdx = 0;
		};

		TArray<DebugRenderer::Triangle> triangles;
		Visitor visitor { triangles, mMaterials, mCachedUseMaterialColors, mCachedTrianglesColoredPerGroup };
		WalkTree(visitor);
		mGeometry = new DebugRenderer::Geometry(inRenderer->CreateTriangleBatch(triangles), GetLocalBounds());
	}

	// Test if the shape is scaled inside out
	DebugRenderer::ECullMode cull_mode = ScaleHelpers::IsInsideOut(inScale)? DebugRenderer::ECullMode::CullFrontFace : DebugRenderer::ECullMode::CullBackFace;

	// Determine the draw mode
	DebugRenderer::EDrawMode draw_mode = inDrawWireframe? DebugRenderer::EDrawMode::Wireframe : DebugRenderer::EDrawMode::Solid;

	// Draw the geometry
	inRenderer->DrawGeometry(inCenterOfMassTransform * Mat44::sScale(inScale), inColor, mGeometry, cull_mode, DebugRenderer::ECastShadow::On, draw_mode);

	if (sDrawTriangleOutlines)
	{
		struct Visitor
		{
			MOSS_INLINE			Visitor(DebugRenderer *inRenderer, RMat44Arg inTransform) :
				mRenderer(inRenderer),
				mTransform(inTransform)
			{
			}

			MOSS_INLINE bool		ShouldAbort() const
			{
				return false;
			}

			MOSS_INLINE bool		ShouldVisitNode(int inStackTop) const
			{
				return true;
			}

			MOSS_INLINE int		VisitNodes(Vec4Arg inBoundsMinX, Vec4Arg inBoundsMinY, Vec4Arg inBoundsMinZ, Vec4Arg inBoundsMaxX, Vec4Arg inBoundsMaxY, Vec4Arg inBoundsMaxZ, UVec4 &ioProperties, int inStackTop)
			{
				UVec4 valid = UVec4::sOr(UVec4::sOr(Vec4::sLess(inBoundsMinX, inBoundsMaxX), Vec4::sLess(inBoundsMinY, inBoundsMaxY)), Vec4::sLess(inBoundsMinZ, inBoundsMaxZ));
				return CountAndSortTrues(valid, ioProperties);
			}

			MOSS_INLINE void		VisitTriangles(const TriangleCodec::DecodingContext &ioContext, const void *inTriangles, int inNumTriangles, uint32 inTriangleBlockID)
			{
				// Decode vertices and flags
				MOSS_ASSERT(inNumTriangles <= MaxTrianglesPerLeaf);
				Vec3 vertices[MaxTrianglesPerLeaf * 3];
				uint8 flags[MaxTrianglesPerLeaf];
				ioContext.Unpack(inTriangles, inNumTriangles, vertices, flags);

				// Loop through triangles
				const uint8 *f = flags;
				for (Vec3 *v = vertices, *v_end = vertices + inNumTriangles * 3; v < v_end; v += 3, ++f)
				{
					// Loop through edges
					for (uint edge_idx = 0; edge_idx < 3; ++edge_idx)
					{
						RVec3 v1 = mTransform * v[edge_idx];
						RVec3 v2 = mTransform * v[(edge_idx + 1) % 3];

						// Draw active edge as a green arrow, other edges as grey
						if (*f & (1 << (edge_idx + FLAGS_ACTIVE_EGDE_SHIFT)))
							mRenderer->DrawArrow(v1, v2, Color::sGreen, 0.01f);
						else
							mRenderer->DrawLine(v1, v2, Color::sGrey);
					}
				}
			}

			DebugRenderer *	mRenderer;
			RMat44			mTransform;
		};

		Visitor visitor { inRenderer, inCenterOfMassTransform.PreScaled(inScale) };
		WalkTree(visitor);
	}
}
#endif // MOSS_DEBUG_RENDERER

bool MeshShape::CastRay(const RayCast &inRay, const SubShapeIDCreator &inSubShapeIDCreator, RayCastResult &ioHit) const
{
	MOSS_PROFILE_FUNCTION();

	struct Visitor
	{
		MOSS_INLINE explicit	Visitor(RayCastResult &ioHit) :
			mHit(ioHit)
		{
		}

		MOSS_INLINE bool		ShouldAbort() const
		{
			return mHit.mFraction <= 0.0f;
		}

		MOSS_INLINE bool		ShouldVisitNode(int inStackTop) const
		{
			return mDistanceStack[inStackTop] < mHit.mFraction;
		}

		MOSS_INLINE int		VisitNodes(Vec4Arg inBoundsMinX, Vec4Arg inBoundsMinY, Vec4Arg inBoundsMinZ, Vec4Arg inBoundsMaxX, Vec4Arg inBoundsMaxY, Vec4Arg inBoundsMaxZ, UVec4 &ioProperties, int inStackTop)
		{
			// Test bounds of 4 children
			Vec4 distance = RayAABox4(mRayOrigin, mRayInvDirection, inBoundsMinX, inBoundsMinY, inBoundsMinZ, inBoundsMaxX, inBoundsMaxY, inBoundsMaxZ);

			// Sort so that highest values are first (we want to first process closer hits and we process stack top to bottom)
			return SortReverseAndStore(distance, mHit.mFraction, ioProperties, &mDistanceStack[inStackTop]);
		}

		MOSS_INLINE void		VisitTriangles(const TriangleCodec::DecodingContext &ioContext, const void *inTriangles, int inNumTriangles, uint32 inTriangleBlockID)
		{
			// Test against triangles
			uint32 triangle_idx;
			float fraction = ioContext.TestRay(mRayOrigin, mRayDirection, inTriangles, inNumTriangles, mHit.mFraction, triangle_idx);
			if (fraction < mHit.mFraction)
			{
				mHit.mFraction = fraction;
				mHit.mSubShapeID2 = mSubShapeIDCreator.PushID(inTriangleBlockID, mTriangleBlockIDBits).PushID(triangle_idx, NumTriangleBits).GetID();
				mReturnValue = true;
			}
		}

		RayCastResult &		mHit;
		Vec3				mRayOrigin;
		Vec3				mRayDirection;
		RayInvDirection		mRayInvDirection;
		uint				mTriangleBlockIDBits;
		SubShapeIDCreator	mSubShapeIDCreator;
		bool				mReturnValue = false;
		float				mDistanceStack[NodeCodec::StackSize];
	};

	Visitor visitor(ioHit);
	visitor.mRayOrigin = inRay.mOrigin;
	visitor.mRayDirection = inRay.mDirection;
	visitor.mRayInvDirection.Set(inRay.mDirection);
	visitor.mTriangleBlockIDBits = NodeCodec::DecodingContext::sTriangleBlockIDBits(sGetNodeHeader(mTree));
	visitor.mSubShapeIDCreator = inSubShapeIDCreator;
	WalkTree(visitor);

	return visitor.mReturnValue;
}

void MeshShape::CastRay(const RayCast &inRay, const RayCastSettings &inRayCastSettings, const SubShapeIDCreator &inSubShapeIDCreator, CastRayCollector &ioCollector, const ShapeFilter &inShapeFilter) const
{
	MOSS_PROFILE_FUNCTION();

	// Test shape filter
	if (!inShapeFilter.ShouldCollide(this, inSubShapeIDCreator.GetID()))
		return;

	struct Visitor
	{
		MOSS_INLINE explicit	Visitor(CastRayCollector &ioCollector) :
			mCollector(ioCollector)
		{
		}

		MOSS_INLINE bool		ShouldAbort() const
		{
			return mCollector.ShouldEarlyOut();
		}

		MOSS_INLINE bool		ShouldVisitNode(int inStackTop) const
		{
			return mDistanceStack[inStackTop] < mCollector.GetEarlyOutFraction();
		}

		MOSS_INLINE int		VisitNodes(Vec4Arg inBoundsMinX, Vec4Arg inBoundsMinY, Vec4Arg inBoundsMinZ, Vec4Arg inBoundsMaxX, Vec4Arg inBoundsMaxY, Vec4Arg inBoundsMaxZ, UVec4 &ioProperties, int inStackTop)
		{
			// Test bounds of 4 children
			Vec4 distance = RayAABox4(mRayOrigin, mRayInvDirection, inBoundsMinX, inBoundsMinY, inBoundsMinZ, inBoundsMaxX, inBoundsMaxY, inBoundsMaxZ);

			// Sort so that highest values are first (we want to first process closer hits and we process stack top to bottom)
			return SortReverseAndStore(distance, mCollector.GetEarlyOutFraction(), ioProperties, &mDistanceStack[inStackTop]);
		}

		MOSS_INLINE void		VisitTriangle(Vec3Arg inV0, Vec3Arg inV1, Vec3Arg inV2, [[maybe_unused]] uint8 inActiveEdges, SubShapeID inSubShapeID2)
		{
			// Back facing check
			if (mBackFaceMode == EBackFaceMode::IgnoreBackFaces && (inV2 - inV0).Cross(inV1 - inV0).Dot(mRayDirection) < 0)
				return;

			// Check the triangle
			float fraction = RayTriangle(mRayOrigin, mRayDirection, inV0, inV1, inV2);
			if (fraction < mCollector.GetEarlyOutFraction())
			{
				RayCastResult hit;
				hit.mBodyID = TransformedShape::sGetBodyID(mCollector.GetContext());
				hit.mFraction = fraction;
				hit.mSubShapeID2 = inSubShapeID2;
				mCollector.AddHit(hit);
			}
		}

		CastRayCollector &	mCollector;
		Vec3				mRayOrigin;
		Vec3				mRayDirection;
		RayInvDirection		mRayInvDirection;
		EBackFaceMode		mBackFaceMode;
		float				mDistanceStack[NodeCodec::StackSize];
	};

	Visitor visitor(ioCollector);
	visitor.mBackFaceMode = inRayCastSettings.mBackFaceModeTriangles;
	visitor.mRayOrigin = inRay.mOrigin;
	visitor.mRayDirection = inRay.mDirection;
	visitor.mRayInvDirection.Set(inRay.mDirection);
	WalkTreePerTriangle(inSubShapeIDCreator, visitor);
}

void MeshShape::CollidePoint(Vec3Arg inPoint, const SubShapeIDCreator &inSubShapeIDCreator, CollidePointCollector &ioCollector, const ShapeFilter &inShapeFilter) const
{
	sCollidePointUsingRayCast(*this, inPoint, inSubShapeIDCreator, ioCollector, inShapeFilter);
}

void MeshShape::CollideSoftBodyVertices(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const CollideSoftBodyVertexIterator &inVertices, uint inNumVertices, int inCollidingShapeIndex) const
{
	MOSS_PROFILE_FUNCTION();

	struct Visitor : public CollideSoftBodyVerticesVsTriangles
	{
		using CollideSoftBodyVerticesVsTriangles::CollideSoftBodyVerticesVsTriangles;

		MOSS_INLINE bool	ShouldAbort() const
		{
			return false;
		}

		MOSS_INLINE bool	ShouldVisitNode([[maybe_unused]] int inStackTop) const
		{
			return mDistanceStack[inStackTop] < mClosestDistanceSq;
		}

		MOSS_INLINE int	VisitNodes(Vec4Arg inBoundsMinX, Vec4Arg inBoundsMinY, Vec4Arg inBoundsMinZ, Vec4Arg inBoundsMaxX, Vec4Arg inBoundsMaxY, Vec4Arg inBoundsMaxZ, UVec4 &ioProperties, int inStackTop)
		{
			// Get distance to vertex
			Vec4 dist_sq = AABox4DistanceSqToPoint(mLocalPosition, inBoundsMinX, inBoundsMinY, inBoundsMinZ, inBoundsMaxX, inBoundsMaxY, inBoundsMaxZ);

			// Sort so that highest values are first (we want to first process closer hits and we process stack top to bottom)
			return SortReverseAndStore(dist_sq, mClosestDistanceSq, ioProperties, &mDistanceStack[inStackTop]);
		}

		MOSS_INLINE void	VisitTriangle(Vec3Arg inV0, Vec3Arg inV1, Vec3Arg inV2, [[maybe_unused]] uint8 inActiveEdges, [[maybe_unused]] SubShapeID inSubShapeID2)
		{
			ProcessTriangle(inV0, inV1, inV2);
		}

		float			mDistanceStack[NodeCodec::StackSize];
	};

	Visitor visitor(inCenterOfMassTransform, inScale);

	for (CollideSoftBodyVertexIterator v = inVertices, sbv_end = inVertices + inNumVertices; v != sbv_end; ++v)
		if (v.GetInvMass() > 0.0f)
		{
			visitor.StartVertex(v);
			WalkTreePerTriangle(SubShapeIDCreator(), visitor);
			visitor.FinishVertex(v, inCollidingShapeIndex);
		}
}

void MeshShape::sCastConvexVsMesh(const ShapeCast &inShapeCast, const ShapeCastSettings &inShapeCastSettings, const Shape *inShape, Vec3Arg inScale, [[maybe_unused]] const ShapeFilter &inShapeFilter, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, CastShapeCollector &ioCollector)
{
	MOSS_PROFILE_FUNCTION();

	struct Visitor : public CastConvexVsTriangles
	{
		using CastConvexVsTriangles::CastConvexVsTriangles;

		MOSS_INLINE bool		ShouldAbort() const
		{
			return mCollector.ShouldEarlyOut();
		}

		MOSS_INLINE bool		ShouldVisitNode(int inStackTop) const
		{
			return mDistanceStack[inStackTop] < mCollector.GetPositiveEarlyOutFraction();
		}

		MOSS_INLINE int		VisitNodes(Vec4Arg inBoundsMinX, Vec4Arg inBoundsMinY, Vec4Arg inBoundsMinZ, Vec4Arg inBoundsMaxX, Vec4Arg inBoundsMaxY, Vec4Arg inBoundsMaxZ, UVec4 &ioProperties, int inStackTop)
		{
			// Scale the bounding boxes of this node
			Vec4 bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z;
			AABox4Scale(mScale, inBoundsMinX, inBoundsMinY, inBoundsMinZ, inBoundsMaxX, inBoundsMaxY, inBoundsMaxZ, bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);

			// Enlarge them by the casted shape's box extents
			AABox4EnlargeWithExtent(mBoxExtent, bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);

			// Test bounds of 4 children
			Vec4 distance = RayAABox4(mBoxCenter, mInvDirection, bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);

			// Sort so that highest values are first (we want to first process closer hits and we process stack top to bottom)
			return SortReverseAndStore(distance, mCollector.GetPositiveEarlyOutFraction(), ioProperties, &mDistanceStack[inStackTop]);
		}

		MOSS_INLINE void		VisitTriangle(Vec3Arg inV0, Vec3Arg inV1, Vec3Arg inV2, uint8 inActiveEdges, SubShapeID inSubShapeID2)
		{
			Cast(inV0, inV1, inV2, inActiveEdges, inSubShapeID2);
		}

		RayInvDirection		mInvDirection;
		Vec3				mBoxCenter;
		Vec3				mBoxExtent;
		float				mDistanceStack[NodeCodec::StackSize];
	};

	MOSS_ASSERT(inShape->GetSubType() == EShapeSubType::Mesh);
	const MeshShape *shape = static_cast<const MeshShape *>(inShape);

	Visitor visitor(inShapeCast, inShapeCastSettings, inScale, inCenterOfMassTransform2, inSubShapeIDCreator1, ioCollector);
	visitor.mInvDirection.Set(inShapeCast.mDirection);
	visitor.mBoxCenter = inShapeCast.mShapeWorldBounds.GetCenter();
	visitor.mBoxExtent = inShapeCast.mShapeWorldBounds.GetExtent();
	shape->WalkTreePerTriangle(inSubShapeIDCreator2, visitor);
}

void MeshShape::sCastSphereVsMesh(const ShapeCast &inShapeCast, const ShapeCastSettings &inShapeCastSettings, const Shape *inShape, Vec3Arg inScale, [[maybe_unused]] const ShapeFilter &inShapeFilter, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, CastShapeCollector &ioCollector)
{
	MOSS_PROFILE_FUNCTION();

	struct Visitor : public CastSphereVsTriangles
	{
		using CastSphereVsTriangles::CastSphereVsTriangles;

		MOSS_INLINE bool		ShouldAbort() const
		{
			return mCollector.ShouldEarlyOut();
		}

		MOSS_INLINE bool		ShouldVisitNode(int inStackTop) const
		{
			return mDistanceStack[inStackTop] < mCollector.GetPositiveEarlyOutFraction();
		}

		MOSS_INLINE int		VisitNodes(Vec4Arg inBoundsMinX, Vec4Arg inBoundsMinY, Vec4Arg inBoundsMinZ, Vec4Arg inBoundsMaxX, Vec4Arg inBoundsMaxY, Vec4Arg inBoundsMaxZ, UVec4 &ioProperties, int inStackTop)
		{
			// Scale the bounding boxes of this node
			Vec4 bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z;
			AABox4Scale(mScale, inBoundsMinX, inBoundsMinY, inBoundsMinZ, inBoundsMaxX, inBoundsMaxY, inBoundsMaxZ, bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);

			// Enlarge them by the radius of the sphere
			AABox4EnlargeWithExtent(Vec3::sReplicate(mRadius), bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);

			// Test bounds of 4 children
			Vec4 distance = RayAABox4(mStart, mInvDirection, bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);

			// Sort so that highest values are first (we want to first process closer hits and we process stack top to bottom)
			return SortReverseAndStore(distance, mCollector.GetPositiveEarlyOutFraction(), ioProperties, &mDistanceStack[inStackTop]);
		}

		MOSS_INLINE void		VisitTriangle(Vec3Arg inV0, Vec3Arg inV1, Vec3Arg inV2, uint8 inActiveEdges, SubShapeID inSubShapeID2)
		{
			Cast(inV0, inV1, inV2, inActiveEdges, inSubShapeID2);
		}

		RayInvDirection		mInvDirection;
		float				mDistanceStack[NodeCodec::StackSize];
	};

	MOSS_ASSERT(inShape->GetSubType() == EShapeSubType::Mesh);
	const MeshShape *shape = static_cast<const MeshShape *>(inShape);

	Visitor visitor(inShapeCast, inShapeCastSettings, inScale, inCenterOfMassTransform2, inSubShapeIDCreator1, ioCollector);
	visitor.mInvDirection.Set(inShapeCast.mDirection);
	shape->WalkTreePerTriangle(inSubShapeIDCreator2, visitor);
}

struct MeshShape::MSGetTrianglesContext
{
	MOSS_INLINE		MSGetTrianglesContext(const MeshShape *inShape, const AABox &inBox, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale) :
		mDecodeCtx(sGetNodeHeader(inShape->mTree)),
		mShape(inShape),
		mLocalBox(Mat44::sInverseRotationTranslation(inRotation, inPositionCOM), inBox),
		mMeshScale(inScale),
		mLocalToWorld(Mat44::sRotationTranslation(inRotation, inPositionCOM) * Mat44::sScale(inScale)),
		mIsInsideOut(ScaleHelpers::IsInsideOut(inScale))
	{
	}

	MOSS_INLINE bool	ShouldAbort() const
	{
		return mShouldAbort;
	}

	MOSS_INLINE bool	ShouldVisitNode([[maybe_unused]] int inStackTop) const
	{
		return true;
	}

	MOSS_INLINE int	VisitNodes(Vec4Arg inBoundsMinX, Vec4Arg inBoundsMinY, Vec4Arg inBoundsMinZ, Vec4Arg inBoundsMaxX, Vec4Arg inBoundsMaxY, Vec4Arg inBoundsMaxZ, UVec4 &ioProperties, [[maybe_unused]] int inStackTop) const
	{
		// Scale the bounding boxes of this node
		Vec4 bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z;
		AABox4Scale(mMeshScale, inBoundsMinX, inBoundsMinY, inBoundsMinZ, inBoundsMaxX, inBoundsMaxY, inBoundsMaxZ, bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);

		// Test which nodes collide
		UVec4 collides = AABox4VsBox(mLocalBox, bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);
		return CountAndSortTrues(collides, ioProperties);
	}

	MOSS_INLINE void	VisitTriangles(const TriangleCodec::DecodingContext &ioContext, const void *inTriangles, int inNumTriangles, [[maybe_unused]] uint32 inTriangleBlockID)
	{
		// When the buffer is full and we cannot process the triangles, abort the tree walk. The next time GetTrianglesNext is called we will continue here.
		if (mNumTrianglesFound + inNumTriangles > mMaxTrianglesRequested)
		{
			mShouldAbort = true;
			return;
		}

		// Decode vertices
		MOSS_ASSERT(inNumTriangles <= MaxTrianglesPerLeaf);
		Vec3 vertices[MaxTrianglesPerLeaf * 3];
		ioContext.Unpack(inTriangles, inNumTriangles, vertices);

		// Store vertices as Float3
		if (mIsInsideOut)
		{
			// Scaled inside out, flip the triangles
			for (const Vec3 *v = vertices, *v_end = v + 3 * inNumTriangles; v < v_end; v += 3)
			{
				(mLocalToWorld * v[0]).StoreFloat3(mTriangleVertices++);
				(mLocalToWorld * v[2]).StoreFloat3(mTriangleVertices++);
				(mLocalToWorld * v[1]).StoreFloat3(mTriangleVertices++);
			}
		}
		else
		{
			// Normal scale
			for (const Vec3 *v = vertices, *v_end = v + 3 * inNumTriangles; v < v_end; ++v)
				(mLocalToWorld * *v).StoreFloat3(mTriangleVertices++);
		}

		if (mMaterials != nullptr)
		{
			if (mShape->mMaterials.empty())
			{
				// No materials, output default
				const PhysicsMaterial *default_material = PhysicsMaterial::sDefault;
				for (int m = 0; m < inNumTriangles; ++m)
					*mMaterials++ = default_material;
			}
			else
			{
				// Decode triangle flags
				uint8 flags[MaxTrianglesPerLeaf];
				TriangleCodec::DecodingContext::sGetFlags(inTriangles, inNumTriangles, flags);

				// Store materials
				for (const uint8 *f = flags, *f_end = f + inNumTriangles; f < f_end; ++f)
					*mMaterials++ = mShape->mMaterials[*f & FLAGS_MATERIAL_MASK].GetPtr();
			}
		}

		// Accumulate triangles found
		mNumTrianglesFound += inNumTriangles;
	}

	NodeCodec::DecodingContext	mDecodeCtx;
	const MeshShape *			mShape;
	OrientedBox					mLocalBox;
	Vec3						mMeshScale;
	Mat44						mLocalToWorld;
	int							mMaxTrianglesRequested;
	Float3 *					mTriangleVertices;
	int							mNumTrianglesFound;
	const PhysicsMaterial **	mMaterials;
	bool						mShouldAbort;
	bool						mIsInsideOut;
};

void MeshShape::GetTrianglesStart(GetTrianglesContext &ioContext, const AABox &inBox, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale) const
{
	static_assert(sizeof(MSGetTrianglesContext) <= sizeof(GetTrianglesContext), "GetTrianglesContext too small");
	MOSS_ASSERT(IsAligned(&ioContext, alignof(MSGetTrianglesContext)));

	new (&ioContext) MSGetTrianglesContext(this, inBox, inPositionCOM, inRotation, inScale);
}

int MeshShape::GetTrianglesNext(GetTrianglesContext &ioContext, int inMaxTrianglesRequested, Float3 *outTriangleVertices, const PhysicsMaterial **outMaterials) const
{
	static_assert(cGetTrianglesMinTrianglesRequested >= MaxTrianglesPerLeaf, "cGetTrianglesMinTrianglesRequested is too small");
	MOSS_ASSERT(inMaxTrianglesRequested >= cGetTrianglesMinTrianglesRequested);

	// Check if we're done
	MSGetTrianglesContext &context = (MSGetTrianglesContext &)ioContext;
	if (context.mDecodeCtx.IsDoneWalking())
		return 0;

	// Store parameters on context
	context.mMaxTrianglesRequested = inMaxTrianglesRequested;
	context.mTriangleVertices = outTriangleVertices;
	context.mMaterials = outMaterials;
	context.mShouldAbort = false; // Reset the abort flag
	context.mNumTrianglesFound = 0;

	// Continue (or start) walking the tree
	const TriangleCodec::DecodingContext triangle_ctx(sGetTriangleHeader(mTree));
	const uint8 *buffer_start = &mTree[0];
	context.mDecodeCtx.WalkTree(buffer_start, triangle_ctx, context);
	return context.mNumTrianglesFound;
}

void MeshShape::sCollideConvexVsMesh(const Shape *inShape1, const Shape *inShape2, Vec3Arg inScale1, Vec3Arg inScale2, Mat44Arg inCenterOfMassTransform1, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, const CollideShapeSettings &inCollideShapeSettings, CollideShapeCollector &ioCollector, [[maybe_unused]] const ShapeFilter &inShapeFilter)
{
	MOSS_PROFILE_FUNCTION();

	// Get the shapes
	MOSS_ASSERT(inShape1->GetType() == EShapeType::Convex);
	MOSS_ASSERT(inShape2->GetType() == EShapeType::Mesh);
	const ConvexShape *shape1 = static_cast<const ConvexShape *>(inShape1);
	const MeshShape *shape2 = static_cast<const MeshShape *>(inShape2);

	struct Visitor : public CollideConvexVsTriangles
	{
		using CollideConvexVsTriangles::CollideConvexVsTriangles;

		MOSS_INLINE bool	ShouldAbort() const
		{
			return mCollector.ShouldEarlyOut();
		}

		MOSS_INLINE bool	ShouldVisitNode([[maybe_unused]] int inStackTop) const
		{
			return true;
		}

		MOSS_INLINE int	VisitNodes(Vec4Arg inBoundsMinX, Vec4Arg inBoundsMinY, Vec4Arg inBoundsMinZ, Vec4Arg inBoundsMaxX, Vec4Arg inBoundsMaxY, Vec4Arg inBoundsMaxZ, UVec4 &ioProperties, [[maybe_unused]] int inStackTop) const
		{
			// Scale the bounding boxes of this node
			Vec4 bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z;
			AABox4Scale(mScale2, inBoundsMinX, inBoundsMinY, inBoundsMinZ, inBoundsMaxX, inBoundsMaxY, inBoundsMaxZ, bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);

			// Test which nodes collide
			UVec4 collides = AABox4VsBox(mBoundsOf1InSpaceOf2, bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);
			return CountAndSortTrues(collides, ioProperties);
		}

		MOSS_INLINE void	VisitTriangle(Vec3Arg inV0, Vec3Arg inV1, Vec3Arg inV2, uint8 inActiveEdges, SubShapeID inSubShapeID2)
		{
			Collide(inV0, inV1, inV2, inActiveEdges, inSubShapeID2);
		}
	};

	Visitor visitor(shape1, inScale1, inScale2, inCenterOfMassTransform1, inCenterOfMassTransform2, inSubShapeIDCreator1.GetID(), inCollideShapeSettings, ioCollector);
	shape2->WalkTreePerTriangle(inSubShapeIDCreator2, visitor);
}

void MeshShape::sCollideSphereVsMesh(const Shape *inShape1, const Shape *inShape2, Vec3Arg inScale1, Vec3Arg inScale2, Mat44Arg inCenterOfMassTransform1, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, const CollideShapeSettings &inCollideShapeSettings, CollideShapeCollector &ioCollector, [[maybe_unused]] const ShapeFilter &inShapeFilter)
{
	MOSS_PROFILE_FUNCTION();

	// Get the shapes
	MOSS_ASSERT(inShape1->GetSubType() == EShapeSubType::Sphere);
	MOSS_ASSERT(inShape2->GetType() == EShapeType::Mesh);
	const SphereShape *shape1 = static_cast<const SphereShape *>(inShape1);
	const MeshShape *shape2 = static_cast<const MeshShape *>(inShape2);

	struct Visitor : public CollideSphereVsTriangles
	{
		using CollideSphereVsTriangles::CollideSphereVsTriangles;

		MOSS_INLINE bool	ShouldAbort() const
		{
			return mCollector.ShouldEarlyOut();
		}

		MOSS_INLINE bool	ShouldVisitNode([[maybe_unused]] int inStackTop) const
		{
			return true;
		}

		MOSS_INLINE int	VisitNodes(Vec4Arg inBoundsMinX, Vec4Arg inBoundsMinY, Vec4Arg inBoundsMinZ, Vec4Arg inBoundsMaxX, Vec4Arg inBoundsMaxY, Vec4Arg inBoundsMaxZ, UVec4 &ioProperties, [[maybe_unused]] int inStackTop) const
		{
			// Scale the bounding boxes of this node
			Vec4 bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z;
			AABox4Scale(mScale2, inBoundsMinX, inBoundsMinY, inBoundsMinZ, inBoundsMaxX, inBoundsMaxY, inBoundsMaxZ, bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);

			// Test which nodes collide
			UVec4 collides = AABox4VsSphere(mSphereCenterIn2, mRadiusPlusMaxSeparationSq, bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);
			return CountAndSortTrues(collides, ioProperties);
		}

		MOSS_INLINE void	VisitTriangle(Vec3Arg inV0, Vec3Arg inV1, Vec3Arg inV2, uint8 inActiveEdges, SubShapeID inSubShapeID2)
		{
			Collide(inV0, inV1, inV2, inActiveEdges, inSubShapeID2);
		}
	};

	Visitor visitor(shape1, inScale1, inScale2, inCenterOfMassTransform1, inCenterOfMassTransform2, inSubShapeIDCreator1.GetID(), inCollideShapeSettings, ioCollector);
	shape2->WalkTreePerTriangle(inSubShapeIDCreator2, visitor);
}

void MeshShape::SaveBinaryState(StreamOut &inStream) const
{
	Shape::SaveBinaryState(inStream);

	inStream.Write(static_cast<const ByteBufferVector &>(mTree)); // Make sure we use the TArray<> overload
}

void MeshShape::RestoreBinaryState(StreamIn &inStream)
{
	Shape::RestoreBinaryState(inStream);

	inStream.Read(static_cast<ByteBufferVector &>(mTree)); // Make sure we use the TArray<> overload
}

void MeshShape::SaveMaterialState(PhysicsMaterialList &outMaterials) const
{
	outMaterials = mMaterials;
}

void MeshShape::RestoreMaterialState(const PhysicsMaterialRefC *inMaterials, uint inNumMaterials)
{
	mMaterials.assign(inMaterials, inMaterials + inNumMaterials);
}

Shape::Stats MeshShape::GetStats() const
{
	// Walk the tree to count the triangles
	struct Visitor
	{
		MOSS_INLINE bool		ShouldAbort() const
		{
			return false;
		}

		MOSS_INLINE bool		ShouldVisitNode([[maybe_unused]] int inStackTop) const
		{
			return true;
		}

		MOSS_INLINE int		VisitNodes(Vec4Arg inBoundsMinX, Vec4Arg inBoundsMinY, Vec4Arg inBoundsMinZ, Vec4Arg inBoundsMaxX, Vec4Arg inBoundsMaxY, Vec4Arg inBoundsMaxZ, UVec4 &ioProperties, [[maybe_unused]] int inStackTop) const
		{
			// Visit all valid children
			UVec4 valid = UVec4::sOr(UVec4::sOr(Vec4::sLess(inBoundsMinX, inBoundsMaxX), Vec4::sLess(inBoundsMinY, inBoundsMaxY)), Vec4::sLess(inBoundsMinZ, inBoundsMaxZ));
			return CountAndSortTrues(valid, ioProperties);
		}

		MOSS_INLINE void		VisitTriangles([[maybe_unused]] const TriangleCodec::DecodingContext &ioContext, [[maybe_unused]] const void *inTriangles, int inNumTriangles, [[maybe_unused]] uint32 inTriangleBlockID)
		{
			mNumTriangles += inNumTriangles;
		}

		uint				mNumTriangles = 0;
	};

	Visitor visitor;
	WalkTree(visitor);

	return Stats(sizeof(*this) + mMaterials.size() * sizeof(Ref<PhysicsMaterial>) + mTree.size() * sizeof(uint8), visitor.mNumTriangles);
}

uint32 MeshShape::GetTriangleUserData(const SubShapeID &inSubShapeID) const
{
	// Decode ID
	const void *block_start;
	uint32 triangle_idx;
	DecodeSubShapeID(inSubShapeID, block_start, triangle_idx);

	// Decode triangle
	const TriangleCodec::DecodingContext triangle_ctx(sGetTriangleHeader(mTree));
	return triangle_ctx.GetUserData(block_start, triangle_idx);
}

void MeshShape::sRegister()
{
	ShapeFunctions &f = ShapeFunctions::sGet(EShapeSubType::Mesh);
	f.mConstruct = []() -> Shape * { return new MeshShape; };
	f.mColor = Color::sRed;

	for (EShapeSubType s : sConvexSubShapeTypes)
	{
		CollisionDispatch::sRegisterCollideShape(s, EShapeSubType::Mesh, sCollideConvexVsMesh);
		CollisionDispatch::sRegisterCastShape(s, EShapeSubType::Mesh, sCastConvexVsMesh);

		CollisionDispatch::sRegisterCastShape(EShapeSubType::Mesh, s, CollisionDispatch::sReversedCastShape);
		CollisionDispatch::sRegisterCollideShape(EShapeSubType::Mesh, s, CollisionDispatch::sReversedCollideShape);
	}

	// Specialized collision functions
	CollisionDispatch::sRegisterCollideShape(EShapeSubType::Sphere, EShapeSubType::Mesh, sCollideSphereVsMesh);
	CollisionDispatch::sRegisterCastShape(EShapeSubType::Sphere, EShapeSubType::Mesh, sCastSphereVsMesh);
}

/*													*/


ShapeSettings::ShapeResult MutableCompoundShapeSettings::Create() const
{
	// Build a mutable compound shape
	if (mCachedResult.IsEmpty())
		Ref<Shape> shape = new MutableCompoundShape(*this, mCachedResult);

	return mCachedResult;
}

MutableCompoundShape::MutableCompoundShape(const MutableCompoundShapeSettings &inSettings, ShapeResult &outResult) :
	CompoundShape(EShapeSubType::MutableCompound, inSettings, outResult)
{
	mSubShapes.reserve(inSettings.mSubShapes.size());
	for (const CompoundShapeSettings::SubShapeSettings &shape : inSettings.mSubShapes)
	{
		// Start constructing the runtime sub shape
		SubShape out_shape;
		if (!out_shape.FromSettings(shape, outResult))
			return;

		mSubShapes.push_back(out_shape);
	}

	AdjustCenterOfMass();

	CalculateSubShapeBounds(0, (uint)mSubShapes.size());

	// Check if we're not exceeding the amount of sub shape id bits
	if (GetSubShapeIDBitsRecursive() > SubShapeID::MaxBits)
	{
		outResult.SetError("Compound hierarchy is too deep and exceeds the amount of available sub shape ID bits");
		return;
	}

	outResult.Set(this);
}

Ref<MutableCompoundShape> MutableCompoundShape::Clone() const
{
	Ref<MutableCompoundShape> clone = new MutableCompoundShape();
	clone->SetUserData(GetUserData());

	clone->mCenterOfMass = mCenterOfMass;
	clone->mLocalBounds = mLocalBounds;
	clone->mSubShapes = mSubShapes;
	clone->mInnerRadius = mInnerRadius;
	clone->mSubShapeBounds = mSubShapeBounds;

	return clone;
}

void MutableCompoundShape::AdjustCenterOfMass()
{
	// First calculate the delta of the center of mass
	float mass = 0.0f;
	Vec3 center_of_mass = Vec3::sZero();
	for (const CompoundShape::SubShape &sub_shape : mSubShapes)
	{
		MassProperties child = sub_shape.mShape->GetMassProperties();
		mass += child.mMass;
		center_of_mass += sub_shape.GetPositionCOM() * child.mMass;
	}
	if (mass > 0.0f)
		center_of_mass /= mass;

	// Now adjust all shapes to recenter around center of mass
	for (CompoundShape::SubShape &sub_shape : mSubShapes)
		sub_shape.SetPositionCOM(sub_shape.GetPositionCOM() - center_of_mass);

	// Update bounding boxes
	for (Bounds &bounds : mSubShapeBounds)
	{
		Vec4 xxxx = center_of_mass.SplatX();
		Vec4 yyyy = center_of_mass.SplatY();
		Vec4 zzzz = center_of_mass.SplatZ();
		bounds.mMinX -= xxxx;
		bounds.mMinY -= yyyy;
		bounds.mMinZ -= zzzz;
		bounds.mMaxX -= xxxx;
		bounds.mMaxY -= yyyy;
		bounds.mMaxZ -= zzzz;
	}
	mLocalBounds.Translate(-center_of_mass);

	// And adjust the center of mass for this shape in the opposite direction
	mCenterOfMass += center_of_mass;
}

void MutableCompoundShape::CalculateLocalBounds()
{
	uint num_blocks = GetNumBlocks();
	if (num_blocks > 0)
	{
		// Initialize min/max for first block
		const Bounds *bounds = mSubShapeBounds.data();
		Vec4 min_x = bounds->mMinX;
		Vec4 min_y = bounds->mMinY;
		Vec4 min_z = bounds->mMinZ;
		Vec4 max_x = bounds->mMaxX;
		Vec4 max_y = bounds->mMaxY;
		Vec4 max_z = bounds->mMaxZ;

		// Accumulate other blocks
		const Bounds *bounds_end = bounds + num_blocks;
		for (++bounds; bounds < bounds_end; ++bounds)
		{
			min_x = Vec4::sMin(min_x, bounds->mMinX);
			min_y = Vec4::sMin(min_y, bounds->mMinY);
			min_z = Vec4::sMin(min_z, bounds->mMinZ);
			max_x = Vec4::sMax(max_x, bounds->mMaxX);
			max_y = Vec4::sMax(max_y, bounds->mMaxY);
			max_z = Vec4::sMax(max_z, bounds->mMaxZ);
		}

		// Calculate resulting bounding box
		mLocalBounds.mMin.SetX(min_x.ReduceMin());
		mLocalBounds.mMin.SetY(min_y.ReduceMin());
		mLocalBounds.mMin.SetZ(min_z.ReduceMin());
		mLocalBounds.mMax.SetX(max_x.ReduceMax());
		mLocalBounds.mMax.SetY(max_y.ReduceMax());
		mLocalBounds.mMax.SetZ(max_z.ReduceMax());
	}
	else
	{
		// There are no subshapes, make the bounding box empty
		mLocalBounds.mMin = mLocalBounds.mMax = Vec3::sZero();
	}

	// Cache the inner radius as it can take a while to recursively iterate over all sub shapes
	CalculateInnerRadius();
}

void MutableCompoundShape::EnsureSubShapeBoundsCapacity()
{
	// Check if we have enough space
	uint new_capacity = ((uint)mSubShapes.size() + 3) >> 2;
	if (mSubShapeBounds.size() < new_capacity)
		mSubShapeBounds.resize(new_capacity);
}

void MutableCompoundShape::CalculateSubShapeBounds(uint inStartIdx, uint inNumber)
{
	// Ensure that we have allocated the required space for mSubShapeBounds
	EnsureSubShapeBoundsCapacity();

	// Loop over blocks of 4 sub shapes
	for (uint sub_shape_idx_start = inStartIdx & ~uint(3), sub_shape_idx_end = inStartIdx + inNumber; sub_shape_idx_start < sub_shape_idx_end; sub_shape_idx_start += 4)
	{
		Mat44 bounds_min;
		Mat44 bounds_max;

		AABox sub_shape_bounds;
		for (uint col = 0; col < 4; ++col)
		{
			uint sub_shape_idx = sub_shape_idx_start + col;
			if (sub_shape_idx < mSubShapes.size()) // else reuse sub_shape_bounds from previous iteration
			{
				const SubShape &sub_shape = mSubShapes[sub_shape_idx];

				// Transform the shape's bounds into our local space
				Mat44 transform = Mat44::sRotationTranslation(sub_shape.GetRotation(), sub_shape.GetPositionCOM());

				// Get the bounding box
				sub_shape_bounds = sub_shape.mShape->GetWorldSpaceBounds(transform, Vec3::sOne());
			}

			// Put the bounds as columns in a matrix
			bounds_min.SetColumn3(col, sub_shape_bounds.mMin);
			bounds_max.SetColumn3(col, sub_shape_bounds.mMax);
		}

		// Transpose to go to structure of arrays format
		Mat44 bounds_min_t = bounds_min.Transposed();
		Mat44 bounds_max_t = bounds_max.Transposed();

		// Store in our bounds array
		Bounds &bounds = mSubShapeBounds[sub_shape_idx_start >> 2];
		bounds.mMinX = bounds_min_t.GetColumn4(0);
		bounds.mMinY = bounds_min_t.GetColumn4(1);
		bounds.mMinZ = bounds_min_t.GetColumn4(2);
		bounds.mMaxX = bounds_max_t.GetColumn4(0);
		bounds.mMaxY = bounds_max_t.GetColumn4(1);
		bounds.mMaxZ = bounds_max_t.GetColumn4(2);
	}

	CalculateLocalBounds();
}

uint MutableCompoundShape::AddShape(Vec3Arg inPosition, QuatArg inRotation, const Shape *inShape, uint32 inUserData, uint inIndex)
{
	SubShape sub_shape;
	sub_shape.mShape = inShape;
	sub_shape.mUserData = inUserData;
	sub_shape.SetTransform(inPosition, inRotation, mCenterOfMass);

	if (inIndex >= mSubShapes.size())
	{
		uint shape_idx = uint(mSubShapes.size());
		mSubShapes.push_back(sub_shape);
		CalculateSubShapeBounds(shape_idx, 1);
		return shape_idx;
	}
	else
	{
		mSubShapes.insert(mSubShapes.begin() + inIndex, sub_shape);
		CalculateSubShapeBounds(inIndex, uint(mSubShapes.size()) - inIndex);
		return inIndex;
	}
}

void MutableCompoundShape::RemoveShape(uint inIndex)
{
	mSubShapes.erase(mSubShapes.begin() + inIndex);

	// We always need to recalculate the bounds of the sub shapes as we test blocks
	// of 4 sub shapes at a time and removed shapes get their bounds updated
	// to repeat the bounds of the previous sub shape
	uint num_bounds = (uint)mSubShapes.size() - inIndex;
	CalculateSubShapeBounds(inIndex, num_bounds);
}

void MutableCompoundShape::ModifyShape(uint inIndex, Vec3Arg inPosition, QuatArg inRotation)
{
	SubShape &sub_shape = mSubShapes[inIndex];
	sub_shape.SetTransform(inPosition, inRotation, mCenterOfMass);

	CalculateSubShapeBounds(inIndex, 1);
}

void MutableCompoundShape::ModifyShape(uint inIndex, Vec3Arg inPosition, QuatArg inRotation, const Shape *inShape)
{
	SubShape &sub_shape = mSubShapes[inIndex];
	sub_shape.mShape = inShape;
	sub_shape.SetTransform(inPosition, inRotation, mCenterOfMass);

	CalculateSubShapeBounds(inIndex, 1);
}

void MutableCompoundShape::ModifyShapes(uint inStartIndex, uint inNumber, const Vec3 *inPositions, const Quat *inRotations, uint inPositionStride, uint inRotationStride)
{
	MOSS_ASSERT(inStartIndex + inNumber <= mSubShapes.size());

	const Vec3 *pos = inPositions;
	const Quat *rot = inRotations;
	for (SubShape *dest = &mSubShapes[inStartIndex], *dest_end = dest + inNumber; dest < dest_end; ++dest)
	{
		// Update transform
		dest->SetTransform(*pos, *rot, mCenterOfMass);

		// Advance pointer in position / rotation buffer
		pos = reinterpret_cast<const Vec3 *>(reinterpret_cast<const uint8 *>(pos) + inPositionStride);
		rot = reinterpret_cast<const Quat *>(reinterpret_cast<const uint8 *>(rot) + inRotationStride);
	}

	CalculateSubShapeBounds(inStartIndex, inNumber);
}

template <class Visitor>
inline void MutableCompoundShape::WalkSubShapes(Visitor &ioVisitor) const
{
	// Loop over all blocks of 4 bounding boxes
	for (uint block = 0, num_blocks = GetNumBlocks(); block < num_blocks; ++block)
	{
		// Test the bounding boxes
		const Bounds &bounds = mSubShapeBounds[block];
		typename Visitor::Result result = ioVisitor.TestBlock(bounds.mMinX, bounds.mMinY, bounds.mMinZ, bounds.mMaxX, bounds.mMaxY, bounds.mMaxZ);

		// Check if any of the bounding boxes collided
		if (ioVisitor.ShouldVisitBlock(result))
		{
			// Go through the individual boxes
			uint sub_shape_start_idx = block << 2;
			for (uint col = 0, max_col = min<uint>(4, (uint)mSubShapes.size() - sub_shape_start_idx); col < max_col; ++col) // Don't read beyond the end of the subshapes array
				if (ioVisitor.ShouldVisitSubShape(result, col)) // Because the early out fraction can change, we need to retest every shape
				{
					// Test sub shape
					uint sub_shape_idx = sub_shape_start_idx + col;
					const SubShape &sub_shape = mSubShapes[sub_shape_idx];
					ioVisitor.VisitShape(sub_shape, sub_shape_idx);

					// If no better collision is available abort
					if (ioVisitor.ShouldAbort())
						break;
				}
		}
	}
}

bool MutableCompoundShape::CastRay(const RayCast &inRay, const SubShapeIDCreator &inSubShapeIDCreator, RayCastResult &ioHit) const
{
	MOSS_PROFILE_FUNCTION();

	struct Visitor : public CastRayVisitor
	{
		using CastRayVisitor::CastRayVisitor;

		using Result = Vec4;

		MOSS_INLINE Result	TestBlock(Vec4Arg inBoundsMinX, Vec4Arg inBoundsMinY, Vec4Arg inBoundsMinZ, Vec4Arg inBoundsMaxX, Vec4Arg inBoundsMaxY, Vec4Arg inBoundsMaxZ) const
		{
			return TestBounds(inBoundsMinX, inBoundsMinY, inBoundsMinZ, inBoundsMaxX, inBoundsMaxY, inBoundsMaxZ);
		}

		MOSS_INLINE bool		ShouldVisitBlock(Vec4Arg inResult) const
		{
			UVec4 closer = Vec4::sLess(inResult, Vec4::sReplicate(mHit.mFraction));
			return closer.TestAnyTrue();
		}

		MOSS_INLINE bool		ShouldVisitSubShape(Vec4Arg inResult, uint inIndexInBlock) const
		{
			return inResult[inIndexInBlock] < mHit.mFraction;
		}
	};

	Visitor visitor(inRay, this, inSubShapeIDCreator, ioHit);
	WalkSubShapes(visitor);
	return visitor.mReturnValue;
}

void MutableCompoundShape::CastRay(const RayCast &inRay, const RayCastSettings &inRayCastSettings, const SubShapeIDCreator &inSubShapeIDCreator, CastRayCollector &ioCollector, const ShapeFilter &inShapeFilter) const
{
	MOSS_PROFILE_FUNCTION();

	// Test shape filter
	if (!inShapeFilter.ShouldCollide(this, inSubShapeIDCreator.GetID()))
		return;

	struct Visitor : public CastRayVisitorCollector
	{
		using CastRayVisitorCollector::CastRayVisitorCollector;

		using Result = Vec4;

		MOSS_INLINE Result	TestBlock(Vec4Arg inBoundsMinX, Vec4Arg inBoundsMinY, Vec4Arg inBoundsMinZ, Vec4Arg inBoundsMaxX, Vec4Arg inBoundsMaxY, Vec4Arg inBoundsMaxZ) const
		{
			return TestBounds(inBoundsMinX, inBoundsMinY, inBoundsMinZ, inBoundsMaxX, inBoundsMaxY, inBoundsMaxZ);
		}

		MOSS_INLINE bool		ShouldVisitBlock(Vec4Arg inResult) const
		{
			UVec4 closer = Vec4::sLess(inResult, Vec4::sReplicate(mCollector.GetEarlyOutFraction()));
			return closer.TestAnyTrue();
		}

		MOSS_INLINE bool		ShouldVisitSubShape(Vec4Arg inResult, uint inIndexInBlock) const
		{
			return inResult[inIndexInBlock] < mCollector.GetEarlyOutFraction();
		}
	};

	Visitor visitor(inRay, inRayCastSettings, this, inSubShapeIDCreator, ioCollector, inShapeFilter);
	WalkSubShapes(visitor);
}

void MutableCompoundShape::CollidePoint(Vec3Arg inPoint, const SubShapeIDCreator &inSubShapeIDCreator, CollidePointCollector &ioCollector, const ShapeFilter &inShapeFilter) const
{
	MOSS_PROFILE_FUNCTION();

	// Test shape filter
	if (!inShapeFilter.ShouldCollide(this, inSubShapeIDCreator.GetID()))
		return;

	struct Visitor : public CollidePointVisitor
	{
		using CollidePointVisitor::CollidePointVisitor;

		using Result = UVec4;

		MOSS_INLINE Result	TestBlock(Vec4Arg inBoundsMinX, Vec4Arg inBoundsMinY, Vec4Arg inBoundsMinZ, Vec4Arg inBoundsMaxX, Vec4Arg inBoundsMaxY, Vec4Arg inBoundsMaxZ) const
		{
			return TestBounds(inBoundsMinX, inBoundsMinY, inBoundsMinZ, inBoundsMaxX, inBoundsMaxY, inBoundsMaxZ);
		}

		MOSS_INLINE bool		ShouldVisitBlock(UVec4Arg inResult) const
		{
			return inResult.TestAnyTrue();
		}

		MOSS_INLINE bool		ShouldVisitSubShape(UVec4Arg inResult, uint inIndexInBlock) const
		{
			return inResult[inIndexInBlock] != 0;
		}
	};

	Visitor visitor(inPoint, this, inSubShapeIDCreator, ioCollector, inShapeFilter);
	WalkSubShapes(visitor);
}

void MutableCompoundShape::sCastShapeVsCompound(const ShapeCast &inShapeCast, const ShapeCastSettings &inShapeCastSettings, const Shape *inShape, Vec3Arg inScale, const ShapeFilter &inShapeFilter, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, CastShapeCollector &ioCollector)
{
	MOSS_PROFILE_FUNCTION();

	struct Visitor : public CastShapeVisitor
	{
		using CastShapeVisitor::CastShapeVisitor;

		using Result = Vec4;

		MOSS_INLINE Result	TestBlock(Vec4Arg inBoundsMinX, Vec4Arg inBoundsMinY, Vec4Arg inBoundsMinZ, Vec4Arg inBoundsMaxX, Vec4Arg inBoundsMaxY, Vec4Arg inBoundsMaxZ) const
		{
			return TestBounds(inBoundsMinX, inBoundsMinY, inBoundsMinZ, inBoundsMaxX, inBoundsMaxY, inBoundsMaxZ);
		}

		MOSS_INLINE bool		ShouldVisitBlock(Vec4Arg inResult) const
		{
			UVec4 closer = Vec4::sLess(inResult, Vec4::sReplicate(mCollector.GetPositiveEarlyOutFraction()));
			return closer.TestAnyTrue();
		}

		MOSS_INLINE bool		ShouldVisitSubShape(Vec4Arg inResult, uint inIndexInBlock) const
		{
			return inResult[inIndexInBlock] < mCollector.GetPositiveEarlyOutFraction();
		}
	};

	MOSS_ASSERT(inShape->GetSubType() == EShapeSubType::MutableCompound);
	const MutableCompoundShape *shape = static_cast<const MutableCompoundShape *>(inShape);

	Visitor visitor(inShapeCast, inShapeCastSettings, shape, inScale, inShapeFilter, inCenterOfMassTransform2, inSubShapeIDCreator1, inSubShapeIDCreator2, ioCollector);
	shape->WalkSubShapes(visitor);
}

void MutableCompoundShape::CollectTransformedShapes(const AABox &inBox, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale, const SubShapeIDCreator &inSubShapeIDCreator, TransformedShapeCollector &ioCollector, const ShapeFilter &inShapeFilter) const
{
	MOSS_PROFILE_FUNCTION();

	// Test shape filter
	if (!inShapeFilter.ShouldCollide(this, inSubShapeIDCreator.GetID()))
		return;

	struct Visitor : public CollectTransformedShapesVisitor
	{
		using CollectTransformedShapesVisitor::CollectTransformedShapesVisitor;

		using Result = UVec4;

		MOSS_INLINE Result	TestBlock(Vec4Arg inBoundsMinX, Vec4Arg inBoundsMinY, Vec4Arg inBoundsMinZ, Vec4Arg inBoundsMaxX, Vec4Arg inBoundsMaxY, Vec4Arg inBoundsMaxZ) const
		{
			return TestBounds(inBoundsMinX, inBoundsMinY, inBoundsMinZ, inBoundsMaxX, inBoundsMaxY, inBoundsMaxZ);
		}

		MOSS_INLINE bool		ShouldVisitBlock(UVec4Arg inResult) const
		{
			return inResult.TestAnyTrue();
		}

		MOSS_INLINE bool		ShouldVisitSubShape(UVec4Arg inResult, uint inIndexInBlock) const
		{
			return inResult[inIndexInBlock] != 0;
		}
	};

	Visitor visitor(inBox, this, inPositionCOM, inRotation, inScale, inSubShapeIDCreator, ioCollector, inShapeFilter);
	WalkSubShapes(visitor);
}

int MutableCompoundShape::GetIntersectingSubShapes(const AABox &inBox, uint *outSubShapeIndices, int inMaxSubShapeIndices) const
{
	MOSS_PROFILE_FUNCTION();

	GetIntersectingSubShapesVisitorMC<AABox> visitor(inBox, outSubShapeIndices, inMaxSubShapeIndices);
	WalkSubShapes(visitor);
	return visitor.GetNumResults();
}

int MutableCompoundShape::GetIntersectingSubShapes(const OrientedBox &inBox, uint *outSubShapeIndices, int inMaxSubShapeIndices) const
{
	MOSS_PROFILE_FUNCTION();

	GetIntersectingSubShapesVisitorMC<OrientedBox> visitor(inBox, outSubShapeIndices, inMaxSubShapeIndices);
	WalkSubShapes(visitor);
	return visitor.GetNumResults();
}

void MutableCompoundShape::sCollideCompoundVsShape(const Shape *inShape1, const Shape *inShape2, Vec3Arg inScale1, Vec3Arg inScale2, Mat44Arg inCenterOfMassTransform1, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, const CollideShapeSettings &inCollideShapeSettings, CollideShapeCollector &ioCollector, const ShapeFilter &inShapeFilter)
{
	MOSS_PROFILE_FUNCTION();

	MOSS_ASSERT(inShape1->GetSubType() == EShapeSubType::MutableCompound);
	const MutableCompoundShape *shape1 = static_cast<const MutableCompoundShape *>(inShape1);

	struct Visitor : public CollideCompoundVsShapeVisitor
	{
		using CollideCompoundVsShapeVisitor::CollideCompoundVsShapeVisitor;

		using Result = UVec4;

		MOSS_INLINE Result	TestBlock(Vec4Arg inBoundsMinX, Vec4Arg inBoundsMinY, Vec4Arg inBoundsMinZ, Vec4Arg inBoundsMaxX, Vec4Arg inBoundsMaxY, Vec4Arg inBoundsMaxZ) const
		{
			return TestBounds(inBoundsMinX, inBoundsMinY, inBoundsMinZ, inBoundsMaxX, inBoundsMaxY, inBoundsMaxZ);
		}

		MOSS_INLINE bool		ShouldVisitBlock(UVec4Arg inResult) const
		{
			return inResult.TestAnyTrue();
		}

		MOSS_INLINE bool		ShouldVisitSubShape(UVec4Arg inResult, uint inIndexInBlock) const
		{
			return inResult[inIndexInBlock] != 0;
		}
	};

	Visitor visitor(shape1, inShape2, inScale1, inScale2, inCenterOfMassTransform1, inCenterOfMassTransform2, inSubShapeIDCreator1, inSubShapeIDCreator2, inCollideShapeSettings, ioCollector, inShapeFilter);
	shape1->WalkSubShapes(visitor);
}

void MutableCompoundShape::sCollideShapeVsCompound(const Shape *inShape1, const Shape *inShape2, Vec3Arg inScale1, Vec3Arg inScale2, Mat44Arg inCenterOfMassTransform1, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, const CollideShapeSettings &inCollideShapeSettings, CollideShapeCollector &ioCollector, const ShapeFilter &inShapeFilter)
{
	MOSS_PROFILE_FUNCTION();

	MOSS_ASSERT(inShape2->GetSubType() == EShapeSubType::MutableCompound);
	const MutableCompoundShape *shape2 = static_cast<const MutableCompoundShape *>(inShape2);

	struct Visitor : public CollideShapeVsCompoundVisitor
	{
		using CollideShapeVsCompoundVisitor::CollideShapeVsCompoundVisitor;

		using Result = UVec4;

		MOSS_INLINE Result	TestBlock(Vec4Arg inBoundsMinX, Vec4Arg inBoundsMinY, Vec4Arg inBoundsMinZ, Vec4Arg inBoundsMaxX, Vec4Arg inBoundsMaxY, Vec4Arg inBoundsMaxZ) const
		{
			return TestBounds(inBoundsMinX, inBoundsMinY, inBoundsMinZ, inBoundsMaxX, inBoundsMaxY, inBoundsMaxZ);
		}

		MOSS_INLINE bool		ShouldVisitBlock(UVec4Arg inResult) const
		{
			return inResult.TestAnyTrue();
		}

		MOSS_INLINE bool		ShouldVisitSubShape(UVec4Arg inResult, uint inIndexInBlock) const
		{
			return inResult[inIndexInBlock] != 0;
		}
	};

	Visitor visitor(inShape1, shape2, inScale1, inScale2, inCenterOfMassTransform1, inCenterOfMassTransform2, inSubShapeIDCreator1, inSubShapeIDCreator2, inCollideShapeSettings, ioCollector, inShapeFilter);
	shape2->WalkSubShapes(visitor);
}

void MutableCompoundShape::SaveBinaryState(StreamOut &inStream) const
{
	CompoundShape::SaveBinaryState(inStream);

	// Write bounds
	uint bounds_size = (((uint)mSubShapes.size() + 3) >> 2) * sizeof(Bounds);
	inStream.WriteBytes(mSubShapeBounds.data(), bounds_size);
}

void MutableCompoundShape::RestoreBinaryState(StreamIn &inStream)
{
	CompoundShape::RestoreBinaryState(inStream);

	// Ensure that we have allocated the required space for mSubShapeBounds
	EnsureSubShapeBoundsCapacity();

	// Read bounds
	uint bounds_size = (((uint)mSubShapes.size() + 3) >> 2) * sizeof(Bounds);
	inStream.ReadBytes(mSubShapeBounds.data(), bounds_size);
}

void MutableCompoundShape::sRegister()
{
	ShapeFunctions &f = ShapeFunctions::sGet(EShapeSubType::MutableCompound);
	f.mConstruct = []() -> Shape * { return new MutableCompoundShape; };
	f.mColor = Color::sDarkOrange;

	for (EShapeSubType s : sAllSubShapeTypes)
	{
		CollisionDispatch::sRegisterCollideShape(EShapeSubType::MutableCompound, s, sCollideCompoundVsShape);
		CollisionDispatch::sRegisterCollideShape(s, EShapeSubType::MutableCompound, sCollideShapeVsCompound);
		CollisionDispatch::sRegisterCastShape(s, EShapeSubType::MutableCompound, sCastShapeVsCompound);
	}
}


/*													*/

ShapeSettings::ShapeResult RotatedTranslatedShapeSettings::Create() const
{
	if (mCachedResult.IsEmpty())
		Ref<Shape> shape = new RotatedTranslatedShape(*this, mCachedResult);
	return mCachedResult;
}

RotatedTranslatedShape::RotatedTranslatedShape(const RotatedTranslatedShapeSettings &inSettings, ShapeResult &outResult) :
	DecoratedShape(EShapeSubType::RotatedTranslated, inSettings, outResult)
{
	if (outResult.HasError())
		return;

	// Calculate center of mass position
	mCenterOfMass = inSettings.mPosition + inSettings.mRotation * mInnerShape->GetCenterOfMass();

	// Store rotation (position is always zero because we center around the center of mass)
	mRotation = inSettings.mRotation;
	mIsRotationIdentity = mRotation.IsClose(Quat::sIdentity());

	outResult.Set(this);
}

RotatedTranslatedShape::RotatedTranslatedShape(Vec3Arg inPosition, QuatArg inRotation, const Shape *inShape) :
	DecoratedShape(EShapeSubType::RotatedTranslated, inShape)
{
	// Calculate center of mass position
	mCenterOfMass = inPosition + inRotation * mInnerShape->GetCenterOfMass();

	// Store rotation (position is always zero because we center around the center of mass)
	mRotation = inRotation;
	mIsRotationIdentity = mRotation.IsClose(Quat::sIdentity());
}

MassProperties RotatedTranslatedShape::GetMassProperties() const
{
	// Rotate inertia of child into place
	MassProperties p = mInnerShape->GetMassProperties();
	p.Rotate(Mat44::sRotation(mRotation));
	return p;
}

AABox RotatedTranslatedShape::GetLocalBounds() const
{
	return mInnerShape->GetLocalBounds().Transformed(Mat44::sRotation(mRotation));
}

AABox RotatedTranslatedShape::GetWorldSpaceBounds(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale) const
{
	Mat44 transform = inCenterOfMassTransform * Mat44::sRotation(mRotation);
	return mInnerShape->GetWorldSpaceBounds(transform, TransformScale(inScale));
}

TransformedShape RotatedTranslatedShape::GetSubShapeTransformedShape(const SubShapeID &inSubShapeID, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale, SubShapeID &outRemainder) const
{
	// We don't use any bits in the sub shape ID
	outRemainder = inSubShapeID;

	TransformedShape ts(RVec3(inPositionCOM), inRotation * mRotation, mInnerShape, BodyID());
	ts.SetShapeScale(TransformScale(inScale));
	return ts;
}

Vec3 RotatedTranslatedShape::GetSurfaceNormal(const SubShapeID &inSubShapeID, Vec3Arg inLocalSurfacePosition) const
{
	// Transform surface position to local space and pass call on
	Mat44 transform = Mat44::sRotation(mRotation.Conjugated());
	Vec3 normal = mInnerShape->GetSurfaceNormal(inSubShapeID, transform * inLocalSurfacePosition);

	// Transform normal to this shape's space
	return transform.Multiply3x3Transposed(normal);
}

void RotatedTranslatedShape::GetSupportingFace(const SubShapeID &inSubShapeID, Vec3Arg inDirection, Vec3Arg inScale, Mat44Arg inCenterOfMassTransform, SupportingFace &outVertices) const
{
	Mat44 transform = Mat44::sRotation(mRotation);
	mInnerShape->GetSupportingFace(inSubShapeID, transform.Multiply3x3Transposed(inDirection), TransformScale(inScale), inCenterOfMassTransform * transform, outVertices);
}

void RotatedTranslatedShape::GetSubmergedVolume(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const Plane &inSurface, float &outTotalVolume, float &outSubmergedVolume, Vec3 &outCenterOfBuoyancy MOSS_IF_DEBUG_RENDERER(, RVec3Arg inBaseOffset)) const
{
	// Get center of mass transform of child
	Mat44 transform = inCenterOfMassTransform * Mat44::sRotation(mRotation);

	// Recurse to child
	mInnerShape->GetSubmergedVolume(transform, TransformScale(inScale), inSurface, outTotalVolume, outSubmergedVolume, outCenterOfBuoyancy MOSS_IF_DEBUG_RENDERER(, inBaseOffset));
}

#ifndef MOSS_DEBUG_RENDERER
void RotatedTranslatedShape::Draw(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, Vec3Arg inScale, ColorArg inColor, bool inUseMaterialColors, bool inDrawWireframe) const
{
	mInnerShape->Draw(inRenderer, inCenterOfMassTransform * Mat44::sRotation(mRotation), TransformScale(inScale), inColor, inUseMaterialColors, inDrawWireframe);
}

void RotatedTranslatedShape::DrawGetSupportFunction(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, Vec3Arg inScale, ColorArg inColor, bool inDrawSupportDirection) const
{
	mInnerShape->DrawGetSupportFunction(inRenderer, inCenterOfMassTransform * Mat44::sRotation(mRotation), TransformScale(inScale), inColor, inDrawSupportDirection);
}

void RotatedTranslatedShape::DrawGetSupportingFace(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, Vec3Arg inScale) const
{
	mInnerShape->DrawGetSupportingFace(inRenderer, inCenterOfMassTransform * Mat44::sRotation(mRotation), TransformScale(inScale));
}
#endif // MOSS_DEBUG_RENDERER

bool RotatedTranslatedShape::CastRay(const RayCast &inRay, const SubShapeIDCreator &inSubShapeIDCreator, RayCastResult &ioHit) const
{
	// Transform the ray
	Mat44 transform = Mat44::sRotation(mRotation.Conjugated());
	RayCast ray = inRay.Transformed(transform);

	return mInnerShape->CastRay(ray, inSubShapeIDCreator, ioHit);
}

void RotatedTranslatedShape::CastRay(const RayCast &inRay, const RayCastSettings &inRayCastSettings, const SubShapeIDCreator &inSubShapeIDCreator, CastRayCollector &ioCollector, const ShapeFilter &inShapeFilter) const
{
	// Test shape filter
	if (!inShapeFilter.ShouldCollide(this, inSubShapeIDCreator.GetID()))
		return;

	// Transform the ray
	Mat44 transform = Mat44::sRotation(mRotation.Conjugated());
	RayCast ray = inRay.Transformed(transform);

	return mInnerShape->CastRay(ray, inRayCastSettings, inSubShapeIDCreator, ioCollector, inShapeFilter);
}

void RotatedTranslatedShape::CollidePoint(Vec3Arg inPoint, const SubShapeIDCreator &inSubShapeIDCreator, CollidePointCollector &ioCollector, const ShapeFilter &inShapeFilter) const
{
	// Test shape filter
	if (!inShapeFilter.ShouldCollide(this, inSubShapeIDCreator.GetID()))
		return;

	// Transform the point
	Mat44 transform = Mat44::sRotation(mRotation.Conjugated());
	mInnerShape->CollidePoint(transform * inPoint, inSubShapeIDCreator, ioCollector, inShapeFilter);
}

void RotatedTranslatedShape::CollideSoftBodyVertices(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const CollideSoftBodyVertexIterator &inVertices, uint inNumVertices, int inCollidingShapeIndex) const
{
	mInnerShape->CollideSoftBodyVertices(inCenterOfMassTransform * Mat44::sRotation(mRotation), inScale, inVertices, inNumVertices, inCollidingShapeIndex);
}

void RotatedTranslatedShape::CollectTransformedShapes(const AABox &inBox, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale, const SubShapeIDCreator &inSubShapeIDCreator, TransformedShapeCollector &ioCollector, const ShapeFilter &inShapeFilter) const
{
	// Test shape filter
	if (!inShapeFilter.ShouldCollide(this, inSubShapeIDCreator.GetID()))
		return;

	mInnerShape->CollectTransformedShapes(inBox, inPositionCOM, inRotation * mRotation, TransformScale(inScale), inSubShapeIDCreator, ioCollector, inShapeFilter);
}

void RotatedTranslatedShape::TransformShape(Mat44Arg inCenterOfMassTransform, TransformedShapeCollector &ioCollector) const
{
	mInnerShape->TransformShape(inCenterOfMassTransform * Mat44::sRotation(mRotation), ioCollector);
}

void RotatedTranslatedShape::sCollideRotatedTranslatedVsShape(const Shape *inShape1, const Shape *inShape2, Vec3Arg inScale1, Vec3Arg inScale2, Mat44Arg inCenterOfMassTransform1, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, const CollideShapeSettings &inCollideShapeSettings, CollideShapeCollector &ioCollector, const ShapeFilter &inShapeFilter)
{
	MOSS_ASSERT(inShape1->GetSubType() == EShapeSubType::RotatedTranslated);
	const RotatedTranslatedShape *shape1 = static_cast<const RotatedTranslatedShape *>(inShape1);

	// Get world transform of 1
	Mat44 transform1 = inCenterOfMassTransform1 * Mat44::sRotation(shape1->mRotation);

	CollisionDispatch::sCollideShapeVsShape(shape1->mInnerShape, inShape2, shape1->TransformScale(inScale1), inScale2, transform1, inCenterOfMassTransform2, inSubShapeIDCreator1, inSubShapeIDCreator2, inCollideShapeSettings, ioCollector, inShapeFilter);
}

void RotatedTranslatedShape::sCollideShapeVsRotatedTranslated(const Shape *inShape1, const Shape *inShape2, Vec3Arg inScale1, Vec3Arg inScale2, Mat44Arg inCenterOfMassTransform1, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, const CollideShapeSettings &inCollideShapeSettings, CollideShapeCollector &ioCollector, const ShapeFilter &inShapeFilter)
{
	MOSS_ASSERT(inShape2->GetSubType() == EShapeSubType::RotatedTranslated);
	const RotatedTranslatedShape *shape2 = static_cast<const RotatedTranslatedShape *>(inShape2);

	// Get world transform of 2
	Mat44 transform2 = inCenterOfMassTransform2 * Mat44::sRotation(shape2->mRotation);

	CollisionDispatch::sCollideShapeVsShape(inShape1, shape2->mInnerShape, inScale1, shape2->TransformScale(inScale2), inCenterOfMassTransform1, transform2, inSubShapeIDCreator1, inSubShapeIDCreator2, inCollideShapeSettings, ioCollector, inShapeFilter);
}

void RotatedTranslatedShape::sCollideRotatedTranslatedVsRotatedTranslated(const Shape *inShape1, const Shape *inShape2, Vec3Arg inScale1, Vec3Arg inScale2, Mat44Arg inCenterOfMassTransform1, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, const CollideShapeSettings &inCollideShapeSettings, CollideShapeCollector &ioCollector, const ShapeFilter &inShapeFilter)
{
	MOSS_ASSERT(inShape1->GetSubType() == EShapeSubType::RotatedTranslated);
	const RotatedTranslatedShape *shape1 = static_cast<const RotatedTranslatedShape *>(inShape1);
	MOSS_ASSERT(inShape2->GetSubType() == EShapeSubType::RotatedTranslated);
	const RotatedTranslatedShape *shape2 = static_cast<const RotatedTranslatedShape *>(inShape2);

	// Get world transform of 1 and 2
	Mat44 transform1 = inCenterOfMassTransform1 * Mat44::sRotation(shape1->mRotation);
	Mat44 transform2 = inCenterOfMassTransform2 * Mat44::sRotation(shape2->mRotation);

	CollisionDispatch::sCollideShapeVsShape(shape1->mInnerShape, shape2->mInnerShape, shape1->TransformScale(inScale1), shape2->TransformScale(inScale2), transform1, transform2, inSubShapeIDCreator1, inSubShapeIDCreator2, inCollideShapeSettings, ioCollector, inShapeFilter);
}

void RotatedTranslatedShape::sCastRotatedTranslatedVsShape(const ShapeCast &inShapeCast, const ShapeCastSettings &inShapeCastSettings, const Shape *inShape, Vec3Arg inScale, const ShapeFilter &inShapeFilter, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, CastShapeCollector &ioCollector)
{
	// Fetch rotated translated shape from cast shape
	MOSS_ASSERT(inShapeCast.mShape->GetSubType() == EShapeSubType::RotatedTranslated);
	const RotatedTranslatedShape *shape1 = static_cast<const RotatedTranslatedShape *>(inShapeCast.mShape);

	// Transform the shape cast and update the shape
	Mat44 transform = inShapeCast.mCenterOfMassStart * Mat44::sRotation(shape1->mRotation);
	Vec3 scale = shape1->TransformScale(inShapeCast.mScale);
	ShapeCast shape_cast(shape1->mInnerShape, scale, transform, inShapeCast.mDirection);

	CollisionDispatch::sCastShapeVsShapeLocalSpace(shape_cast, inShapeCastSettings, inShape, inScale, inShapeFilter, inCenterOfMassTransform2, inSubShapeIDCreator1, inSubShapeIDCreator2, ioCollector);
}

void RotatedTranslatedShape::sCastShapeVsRotatedTranslated(const ShapeCast &inShapeCast, const ShapeCastSettings &inShapeCastSettings, const Shape *inShape, Vec3Arg inScale, const ShapeFilter &inShapeFilter, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, CastShapeCollector &ioCollector)
{
	MOSS_ASSERT(inShape->GetSubType() == EShapeSubType::RotatedTranslated);
	const RotatedTranslatedShape *shape = static_cast<const RotatedTranslatedShape *>(inShape);

	// Determine the local transform
	Mat44 local_transform = Mat44::sRotation(shape->mRotation);

	// Transform the shape cast
	ShapeCast shape_cast = inShapeCast.PostTransformed(local_transform.Transposed3x3());

	CollisionDispatch::sCastShapeVsShapeLocalSpace(shape_cast, inShapeCastSettings, shape->mInnerShape, shape->TransformScale(inScale), inShapeFilter, inCenterOfMassTransform2 * local_transform, inSubShapeIDCreator1, inSubShapeIDCreator2, ioCollector);
}

void RotatedTranslatedShape::sCastRotatedTranslatedVsRotatedTranslated(const ShapeCast &inShapeCast, const ShapeCastSettings &inShapeCastSettings, const Shape *inShape, Vec3Arg inScale, const ShapeFilter &inShapeFilter, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, CastShapeCollector &ioCollector)
{
	MOSS_ASSERT(inShapeCast.mShape->GetSubType() == EShapeSubType::RotatedTranslated);
	const RotatedTranslatedShape *shape1 = static_cast<const RotatedTranslatedShape *>(inShapeCast.mShape);
	MOSS_ASSERT(inShape->GetSubType() == EShapeSubType::RotatedTranslated);
	const RotatedTranslatedShape *shape2 = static_cast<const RotatedTranslatedShape *>(inShape);

	// Determine the local transform of shape 2
	Mat44 local_transform2 = Mat44::sRotation(shape2->mRotation);
	Mat44 local_transform2_transposed = local_transform2.Transposed3x3();

	// Transform the shape cast and update the shape
	Mat44 transform = (local_transform2_transposed * inShapeCast.mCenterOfMassStart) * Mat44::sRotation(shape1->mRotation);
	Vec3 scale = shape1->TransformScale(inShapeCast.mScale);
	ShapeCast shape_cast(shape1->mInnerShape, scale, transform, local_transform2_transposed.Multiply3x3(inShapeCast.mDirection));

	CollisionDispatch::sCastShapeVsShapeLocalSpace(shape_cast, inShapeCastSettings, shape2->mInnerShape, shape2->TransformScale(inScale), inShapeFilter, inCenterOfMassTransform2 * local_transform2, inSubShapeIDCreator1, inSubShapeIDCreator2, ioCollector);
}

void RotatedTranslatedShape::SaveBinaryState(StreamOut &inStream) const
{
	DecoratedShape::SaveBinaryState(inStream);

	inStream.Write(mCenterOfMass);
	inStream.Write(mRotation);
}

void RotatedTranslatedShape::RestoreBinaryState(StreamIn &inStream)
{
	DecoratedShape::RestoreBinaryState(inStream);

	inStream.Read(mCenterOfMass);
	inStream.Read(mRotation);
	mIsRotationIdentity = mRotation.IsClose(Quat::sIdentity());
}

bool RotatedTranslatedShape::IsValidScale(Vec3Arg inScale) const
{
	if (!Shape::IsValidScale(inScale))
		return false;

	if (mIsRotationIdentity || ScaleHelpers::IsUniformScale(inScale))
		return mInnerShape->IsValidScale(inScale);

	if (!ScaleHelpers::CanScaleBeRotated(mRotation, inScale))
		return false;

	return mInnerShape->IsValidScale(ScaleHelpers::RotateScale(mRotation, inScale));
}

Vec3 RotatedTranslatedShape::MakeScaleValid(Vec3Arg inScale) const
{
	Vec3 scale = ScaleHelpers::MakeNonZeroScale(inScale);

	if (mIsRotationIdentity || ScaleHelpers::IsUniformScale(scale))
		return mInnerShape->MakeScaleValid(scale);

	if (ScaleHelpers::CanScaleBeRotated(mRotation, scale))
		return ScaleHelpers::RotateScale(mRotation.Conjugated(), mInnerShape->MakeScaleValid(ScaleHelpers::RotateScale(mRotation, scale)));

	Vec3 abs_uniform_scale = ScaleHelpers::MakeUniformScale(scale.Abs());
	Vec3 uniform_scale = scale.GetSign() * abs_uniform_scale;
	if (ScaleHelpers::CanScaleBeRotated(mRotation, uniform_scale))
		return uniform_scale;

	return Sign(scale.GetX()) * abs_uniform_scale;
}

void RotatedTranslatedShape::sRegister()
{
	ShapeFunctions &f = ShapeFunctions::sGet(EShapeSubType::RotatedTranslated);
	f.mConstruct = []() -> Shape * { return new RotatedTranslatedShape; };
	f.mColor = Color::sBlue;

	for (EShapeSubType s : sAllSubShapeTypes)
	{
		CollisionDispatch::sRegisterCollideShape(EShapeSubType::RotatedTranslated, s, sCollideRotatedTranslatedVsShape);
		CollisionDispatch::sRegisterCollideShape(s, EShapeSubType::RotatedTranslated, sCollideShapeVsRotatedTranslated);
		CollisionDispatch::sRegisterCastShape(EShapeSubType::RotatedTranslated, s, sCastRotatedTranslatedVsShape);
		CollisionDispatch::sRegisterCastShape(s, EShapeSubType::RotatedTranslated, sCastShapeVsRotatedTranslated);
	}

	CollisionDispatch::sRegisterCollideShape(EShapeSubType::RotatedTranslated, EShapeSubType::RotatedTranslated, sCollideRotatedTranslatedVsRotatedTranslated);
	CollisionDispatch::sRegisterCastShape(EShapeSubType::RotatedTranslated, EShapeSubType::RotatedTranslated, sCastRotatedTranslatedVsRotatedTranslated);
}


/*													*/

ShapeSettings::ShapeResult PlaneShapeSettings::Create() const
{
	if (mCachedResult.IsEmpty())
		Ref<Shape> shape = new PlaneShape(*this, mCachedResult);
	return mCachedResult;
}

inline static void sPlaneGetOrthogonalBasis(Vec3Arg inNormal, Vec3 &outPerp1, Vec3 &outPerp2)
{
	outPerp1 = inNormal.Cross(Vec3::sAxisY()).NormalizedOr(Vec3::sAxisX());
	outPerp2 = outPerp1.Cross(inNormal).Normalized();
	outPerp1 = inNormal.Cross(outPerp2);
}

void PlaneShape::GetVertices(Vec3 *outVertices) const
{
	// Create orthogonal basis
	Vec3 normal = mPlane.GetNormal();
	Vec3 perp1, perp2;
	sPlaneGetOrthogonalBasis(normal, perp1, perp2);

	// Scale basis
	perp1 *= mHalfExtent;
	perp2 *= mHalfExtent;

	// Calculate corners
	Vec3 point = -normal * mPlane.GetConstant();
	outVertices[0] = point + perp1 + perp2;
	outVertices[1] = point + perp1 - perp2;
	outVertices[2] = point - perp1 - perp2;
	outVertices[3] = point - perp1 + perp2;
}

void PlaneShape::CalculateLocalBounds()
{
	// Get the vertices of the plane
	Vec3 vertices[4];
	GetVertices(vertices);

	// Encapsulate the vertices and a point mHalfExtent behind the plane
	mLocalBounds = AABox();
	Vec3 normal = mPlane.GetNormal();
	for (const Vec3 &v : vertices)
	{
		mLocalBounds.Encapsulate(v);
		mLocalBounds.Encapsulate(v - mHalfExtent * normal);
	}
}

PlaneShape::PlaneShape(const PlaneShapeSettings &inSettings, ShapeResult &outResult) :
	Shape(EShapeType::Plane, EShapeSubType::Plane, inSettings, outResult),
	mPlane(inSettings.mPlane),
	mMaterial(inSettings.mMaterial),
	mHalfExtent(inSettings.mHalfExtent)
{
	if (!mPlane.GetNormal().IsNormalized())
	{
		outResult.SetError("Plane normal needs to be normalized!");
		return;
	}

	CalculateLocalBounds();

	outResult.Set(this);
}

MassProperties PlaneShape::GetMassProperties() const
{
	// Object should always be static, return default mass properties
	return MassProperties();
}

void PlaneShape::GetSupportingFace(const SubShapeID &inSubShapeID, Vec3Arg inDirection, Vec3Arg inScale, Mat44Arg inCenterOfMassTransform, SupportingFace &outVertices) const
{
	// Get the vertices of the plane
	Vec3 vertices[4];
	GetVertices(vertices);

	// Reverse if scale is inside out
	if (ScaleHelpers::IsInsideOut(inScale))
	{
		std::swap(vertices[0], vertices[3]);
		std::swap(vertices[1], vertices[2]);
	}

	// Transform them to world space
	outVertices.clear();
	Mat44 com = inCenterOfMassTransform.PreScaled(inScale);
	for (const Vec3 &v : vertices)
		outVertices.push_back(com * v);
}

#ifndef MOSS_DEBUG_RENDERER
void PlaneShape::Draw(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, Vec3Arg inScale, ColorArg inColor, bool inUseMaterialColors, bool inDrawWireframe) const
{
	// Get the vertices of the plane
	Vec3 local_vertices[4];
	GetVertices(local_vertices);

	// Reverse if scale is inside out
	if (ScaleHelpers::IsInsideOut(inScale))
	{
		std::swap(local_vertices[0], local_vertices[3]);
		std::swap(local_vertices[1], local_vertices[2]);
	}

	// Transform them to world space
	RMat44 com = inCenterOfMassTransform.PreScaled(inScale);
	RVec3 vertices[4];
	for (uint i = 0; i < 4; ++i)
		vertices[i] = com * local_vertices[i];

	// Determine the color
	Color color = inUseMaterialColors? GetMaterial(SubShapeID())->GetDebugColor() : inColor;

	// Draw the plane
	if (inDrawWireframe)
	{
		inRenderer->DrawWireTriangle(vertices[0], vertices[1], vertices[2], color);
		inRenderer->DrawWireTriangle(vertices[0], vertices[2], vertices[3], color);
	}
	else
	{
		inRenderer->DrawTriangle(vertices[0], vertices[1], vertices[2], color, DebugRenderer::ECastShadow::On);
		inRenderer->DrawTriangle(vertices[0], vertices[2], vertices[3], color, DebugRenderer::ECastShadow::On);
	}
}
#endif // MOSS_DEBUG_RENDERER

bool PlaneShape::CastRay(const RayCast &inRay, const SubShapeIDCreator &inSubShapeIDCreator, RayCastResult &ioHit) const
{
	MOSS_PROFILE_FUNCTION();

	// Test starting inside of negative half space
	float distance = mPlane.SignedDistance(inRay.mOrigin);
	if (distance <= 0.0f)
	{
		ioHit.mFraction = 0.0f;
		ioHit.mSubShapeID2 = inSubShapeIDCreator.GetID();
		return true;
	}

	// Test ray parallel to plane
	float dot = inRay.mDirection.Dot(mPlane.GetNormal());
	if (dot == 0.0f)
		return false;

	// Calculate hit fraction
	float fraction = -distance / dot;
	if (fraction >= 0.0f && fraction < ioHit.mFraction)
	{
		ioHit.mFraction = fraction;
		ioHit.mSubShapeID2 = inSubShapeIDCreator.GetID();
		return true;
	}

	return false;
}

void PlaneShape::CastRay(const RayCast &inRay, const RayCastSettings &inRayCastSettings, const SubShapeIDCreator &inSubShapeIDCreator, CastRayCollector &ioCollector, const ShapeFilter &inShapeFilter) const
{
	MOSS_PROFILE_FUNCTION();

	// Test shape filter
	if (!inShapeFilter.ShouldCollide(this, inSubShapeIDCreator.GetID()))
		return;

	// Inside solid half space?
	float distance = mPlane.SignedDistance(inRay.mOrigin);
	if (inRayCastSettings.mTreatConvexAsSolid
		&& distance <= 0.0f // Inside plane
		&& ioCollector.GetEarlyOutFraction() > 0.0f) // Willing to accept hits at fraction 0
	{
		// Hit at fraction 0
		RayCastResult hit;
		hit.mBodyID = TransformedShape::sGetBodyID(ioCollector.GetContext());
		hit.mFraction = 0.0f;
		hit.mSubShapeID2 = inSubShapeIDCreator.GetID();
		ioCollector.AddHit(hit);
	}

	float dot = inRay.mDirection.Dot(mPlane.GetNormal());
	if (dot != 0.0f // Parallel ray will not hit plane
		&& (inRayCastSettings.mBackFaceModeConvex == EBackFaceMode::CollideWithBackFaces || dot < 0.0f)) // Back face culling
	{
		// Calculate hit with plane
		float fraction = -distance / dot;
		if (fraction >= 0.0f && fraction < ioCollector.GetEarlyOutFraction())
		{
			RayCastResult hit;
			hit.mBodyID = TransformedShape::sGetBodyID(ioCollector.GetContext());
			hit.mFraction = fraction;
			hit.mSubShapeID2 = inSubShapeIDCreator.GetID();
			ioCollector.AddHit(hit);
		}
	}
}

void PlaneShape::CollidePoint(Vec3Arg inPoint, const SubShapeIDCreator &inSubShapeIDCreator, CollidePointCollector &ioCollector, const ShapeFilter &inShapeFilter) const
{
	MOSS_PROFILE_FUNCTION();

	// Test shape filter
	if (!inShapeFilter.ShouldCollide(this, inSubShapeIDCreator.GetID()))
		return;

	// Check if the point is inside the plane
	if (mPlane.SignedDistance(inPoint) < 0.0f)
		ioCollector.AddHit({ TransformedShape::sGetBodyID(ioCollector.GetContext()), inSubShapeIDCreator.GetID() });
}

void PlaneShape::CollideSoftBodyVertices(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const CollideSoftBodyVertexIterator &inVertices, uint inNumVertices, int inCollidingShapeIndex) const
{
	MOSS_PROFILE_FUNCTION();

	// Convert plane to world space
	Plane plane = mPlane.Scaled(inScale).GetTransformed(inCenterOfMassTransform);

	for (CollideSoftBodyVertexIterator v = inVertices, sbv_end = inVertices + inNumVertices; v != sbv_end; ++v)
		if (v.GetInvMass() > 0.0f)
		{
			// Calculate penetration
			float penetration = -plane.SignedDistance(v.GetPosition());
			if (v.UpdatePenetration(penetration))
				v.SetCollision(plane, inCollidingShapeIndex);
		}
}

// This is a version of GetSupportingFace that returns a face that is large enough to cover the shape we're colliding with but not as large as the regular GetSupportedFace to avoid numerical precision issues
inline static void sGetSupportingFace(const ConvexShape *inShape, Vec3Arg inShapeCOM, const Plane &inPlane, Mat44Arg inPlaneToWorld, ConvexShape::SupportingFace &outPlaneFace)
{
	// Project COM of shape onto plane
	Plane world_plane = inPlane.GetTransformed(inPlaneToWorld);
	Vec3 center = world_plane.ProjectPointOnPlane(inShapeCOM);

	// Create orthogonal basis for the plane
	Vec3 normal = world_plane.GetNormal();
	Vec3 perp1, perp2;
	sPlaneGetOrthogonalBasis(normal, perp1, perp2);

	// Base the size of the face on the bounding box of the shape, ensuring that it is large enough to cover the entire shape
	float size = inShape->GetLocalBounds().GetSize().Length();
	perp1 *= size;
	perp2 *= size;

	// Emit the vertices
	outPlaneFace.resize(4);
	outPlaneFace[0] = center + perp1 + perp2;
	outPlaneFace[1] = center + perp1 - perp2;
	outPlaneFace[2] = center - perp1 - perp2;
	outPlaneFace[3] = center - perp1 + perp2;
}

void PlaneShape::sCastConvexVsPlane(const ShapeCast &inShapeCast, const ShapeCastSettings &inShapeCastSettings, const Shape *inShape, Vec3Arg inScale, [[maybe_unused]] const ShapeFilter &inShapeFilter, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, CastShapeCollector &ioCollector)
{
	MOSS_PROFILE_FUNCTION();

	// Get the shapes
	MOSS_ASSERT(inShapeCast.mShape->GetType() == EShapeType::Convex);
	MOSS_ASSERT(inShape->GetType() == EShapeType::Plane);
	const ConvexShape *convex_shape = static_cast<const ConvexShape *>(inShapeCast.mShape);
	const PlaneShape *plane_shape = static_cast<const PlaneShape *>(inShape);

	// Shape cast is provided relative to COM of inShape, so all we need to do is transform our plane with inScale
	Plane plane = plane_shape->mPlane.Scaled(inScale);
	Vec3 normal = plane.GetNormal();

	// Get support function
	ConvexShape::SupportBuffer shape1_support_buffer;
	const ConvexShape::Support *shape1_support = convex_shape->GetSupportFunction(ConvexShape::ESupportMode::Default, shape1_support_buffer, inShapeCast.mScale);

	// Get the support point of the convex shape in the opposite direction of the plane normal in our local space
	Vec3 normal_in_convex_shape_space = inShapeCast.mCenterOfMassStart.Multiply3x3Transposed(normal);
	Vec3 support_point = inShapeCast.mCenterOfMassStart * shape1_support->GetSupport(-normal_in_convex_shape_space);
	float signed_distance = plane.SignedDistance(support_point);
	float convex_radius = shape1_support->GetConvexRadius();
	float penetration_depth = -signed_distance + convex_radius;
	float dot = inShapeCast.mDirection.Dot(normal);

	// Collision output
	Mat44 com_hit;
	Vec3 point1, point2;
	float fraction;

	// Do we start in collision?
	if (penetration_depth > 0.0f)
	{
		// Back face culling?
		if (inShapeCastSettings.mBackFaceModeConvex == EBackFaceMode::IgnoreBackFaces && dot > 0.0f)
			return;

		// Shallower hit?
		if (penetration_depth <= -ioCollector.GetEarlyOutFraction())
			return;

		// We're hitting at fraction 0
		fraction = 0.0f;

		// Get contact point
		com_hit = inCenterOfMassTransform2;
		point1 = inCenterOfMassTransform2 * (support_point - normal * convex_radius);
		point2 = inCenterOfMassTransform2 * (support_point - normal * signed_distance);
	}
	else if (dot < 0.0f) // Moving towards the plane?
	{
		// Calculate hit fraction
		fraction = penetration_depth / dot;
		MOSS_ASSERT(fraction >= 0.0f);

		// Further than early out fraction?
		if (fraction >= ioCollector.GetEarlyOutFraction())
			return;

		// Get contact point
		com_hit = inCenterOfMassTransform2.PostTranslated(fraction * inShapeCast.mDirection);
		point1 = point2 = com_hit * (support_point - normal * convex_radius);
	}
	else
	{
		// Moving away from the plane
		return;
	}

	// Create cast result
	Vec3 penetration_axis_world = com_hit.Multiply3x3(-normal);
	bool back_facing = dot > 0.0f;
	ShapeCastResult result(fraction, point1, point2, penetration_axis_world, back_facing, inSubShapeIDCreator1.GetID(), inSubShapeIDCreator2.GetID(), TransformedShape::sGetBodyID(ioCollector.GetContext()));

	// Gather faces
	if (inShapeCastSettings.mCollectFacesMode == ECollectFacesMode::CollectFaces)
	{
		// Get supporting face of convex shape
		Mat44 shape_to_world = com_hit * inShapeCast.mCenterOfMassStart;
		convex_shape->GetSupportingFace(SubShapeID(), normal_in_convex_shape_space, inShapeCast.mScale, shape_to_world, result.mShape1Face);

		// Get supporting face of plane
		if (!result.mShape1Face.empty())
			sGetSupportingFace(convex_shape, shape_to_world.GetTranslation(), plane, inCenterOfMassTransform2, result.mShape2Face);
	}

	// Notify the collector
	MOSS_IF_TRACK_NARROWPHASE_STATS(TrackNarrowPhaseCollector track;)
	ioCollector.AddHit(result);
}

struct PlaneShape::PSGetTrianglesContext
{
	Float3	mVertices[4];
	bool	mDone = false;
};

void PlaneShape::GetTrianglesStart(GetTrianglesContext &ioContext, const AABox &inBox, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale) const
{
	static_assert(sizeof(PSGetTrianglesContext) <= sizeof(GetTrianglesContext), "GetTrianglesContext too small");
	MOSS_ASSERT(IsAligned(&ioContext, alignof(PSGetTrianglesContext)));

	PSGetTrianglesContext *context = new (&ioContext) PSGetTrianglesContext();

	// Get the vertices of the plane
	Vec3 vertices[4];
	GetVertices(vertices);

	// Reverse if scale is inside out
	if (ScaleHelpers::IsInsideOut(inScale))
	{
		std::swap(vertices[0], vertices[3]);
		std::swap(vertices[1], vertices[2]);
	}

	// Transform them to world space
	Mat44 com = Mat44::sRotationTranslation(inRotation, inPositionCOM).PreScaled(inScale);
	for (uint i = 0; i < 4; ++i)
		(com * vertices[i]).StoreFloat3(&context->mVertices[i]);
}

int PlaneShape::GetTrianglesNext(GetTrianglesContext &ioContext, int inMaxTrianglesRequested, Float3 *outTriangleVertices, const PhysicsMaterial **outMaterials) const
{
	static_assert(cGetTrianglesMinTrianglesRequested >= 2, "cGetTrianglesMinTrianglesRequested is too small");
	MOSS_ASSERT(inMaxTrianglesRequested >= cGetTrianglesMinTrianglesRequested);

	// Check if we're done
	PSGetTrianglesContext &context = (PSGetTrianglesContext &)ioContext;
	if (context.mDone)
		return 0;
	context.mDone = true;

	// 1st triangle
	outTriangleVertices[0] = context.mVertices[0];
	outTriangleVertices[1] = context.mVertices[1];
	outTriangleVertices[2] = context.mVertices[2];

	// 2nd triangle
	outTriangleVertices[3] = context.mVertices[0];
	outTriangleVertices[4] = context.mVertices[2];
	outTriangleVertices[5] = context.mVertices[3];

	if (outMaterials != nullptr)
	{
		// Get material
		const PhysicsMaterial *material = GetMaterial(SubShapeID());
		outMaterials[0] = material;
		outMaterials[1] = material;
	}

	return 2;
}

void PlaneShape::sCollideConvexVsPlane(const Shape *inShape1, const Shape *inShape2, Vec3Arg inScale1, Vec3Arg inScale2, Mat44Arg inCenterOfMassTransform1, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, const CollideShapeSettings &inCollideShapeSettings, CollideShapeCollector &ioCollector, [[maybe_unused]] const ShapeFilter &inShapeFilter)
{
	MOSS_PROFILE_FUNCTION();

	// Get the shapes
	MOSS_ASSERT(inShape1->GetType() == EShapeType::Convex);
	MOSS_ASSERT(inShape2->GetType() == EShapeType::Plane);
	const ConvexShape *shape1 = static_cast<const ConvexShape *>(inShape1);
	const PlaneShape *shape2 = static_cast<const PlaneShape *>(inShape2);

	// Transform the plane to the space of the convex shape
	Plane scaled_plane = shape2->mPlane.Scaled(inScale2);
	Plane plane = scaled_plane.GetTransformed(inCenterOfMassTransform1.InversedRotationTranslation() * inCenterOfMassTransform2);
	Vec3 normal = plane.GetNormal();

	// Get support function
	ConvexShape::SupportBuffer shape1_support_buffer;
	const ConvexShape::Support *shape1_support = shape1->GetSupportFunction(ConvexShape::ESupportMode::Default, shape1_support_buffer, inScale1);

	// Get the support point of the convex shape in the opposite direction of the plane normal
	Vec3 support_point = shape1_support->GetSupport(-normal);
	float signed_distance = plane.SignedDistance(support_point);
	float convex_radius = shape1_support->GetConvexRadius();
	float penetration_depth = -signed_distance + convex_radius;
	if (penetration_depth > -inCollideShapeSettings.mMaxSeparationDistance)
	{
		// Get contact point
		Vec3 point1 = inCenterOfMassTransform1 * (support_point - normal * convex_radius);
		Vec3 point2 = inCenterOfMassTransform1 * (support_point - normal * signed_distance);
		Vec3 penetration_axis_world = inCenterOfMassTransform1.Multiply3x3(-normal);

		// Create collision result
		CollideShapeResult result(point1, point2, penetration_axis_world, penetration_depth, inSubShapeIDCreator1.GetID(), inSubShapeIDCreator2.GetID(), TransformedShape::sGetBodyID(ioCollector.GetContext()));

		// Gather faces
		if (inCollideShapeSettings.mCollectFacesMode == ECollectFacesMode::CollectFaces)
		{
			// Get supporting face of shape 1
			shape1->GetSupportingFace(SubShapeID(), normal, inScale1, inCenterOfMassTransform1, result.mShape1Face);

			// Get supporting face of shape 2
			if (!result.mShape1Face.empty())
				sGetSupportingFace(shape1, inCenterOfMassTransform1.GetTranslation(), scaled_plane, inCenterOfMassTransform2, result.mShape2Face);
		}

		// Notify the collector
		MOSS_IF_TRACK_NARROWPHASE_STATS(TrackNarrowPhaseCollector track;)
		ioCollector.AddHit(result);
	}
}

void PlaneShape::SaveBinaryState(StreamOut &inStream) const
{
	Shape::SaveBinaryState(inStream);

	inStream.Write(mPlane);
	inStream.Write(mHalfExtent);
}

void PlaneShape::RestoreBinaryState(StreamIn &inStream)
{
	Shape::RestoreBinaryState(inStream);

	inStream.Read(mPlane);
	inStream.Read(mHalfExtent);

	CalculateLocalBounds();
}

void PlaneShape::SaveMaterialState(PhysicsMaterialList &outMaterials) const
{
	outMaterials = { mMaterial };
}

void PlaneShape::RestoreMaterialState(const PhysicsMaterialRefC *inMaterials, uint inNumMaterials)
{
	MOSS_ASSERT(inNumMaterials == 1);
	mMaterial = inMaterials[0];
}

void PlaneShape::sRegister()
{
	ShapeFunctions &f = ShapeFunctions::sGet(EShapeSubType::Plane);
	f.mConstruct = []() -> Shape * { return new PlaneShape; };
	f.mColor = Color::sDarkRed;

	for (EShapeSubType s : sConvexSubShapeTypes)
	{
		CollisionDispatch::sRegisterCollideShape(s, EShapeSubType::Plane, sCollideConvexVsPlane);
		CollisionDispatch::sRegisterCastShape(s, EShapeSubType::Plane, sCastConvexVsPlane);

		CollisionDispatch::sRegisterCastShape(EShapeSubType::Plane, s, CollisionDispatch::sReversedCastShape);
		CollisionDispatch::sRegisterCollideShape(EShapeSubType::Plane, s, CollisionDispatch::sReversedCollideShape);
	}
}

/*													*/

ShapeSettings::ShapeResult ScaledShapeSettings::Create() const
{
	if (mCachedResult.IsEmpty())
		Ref<Shape> shape = new ScaledShape(*this, mCachedResult);
	return mCachedResult;
}

ScaledShape::ScaledShape(const ScaledShapeSettings &inSettings, ShapeResult &outResult) :
	DecoratedShape(EShapeSubType::Scaled, inSettings, outResult),
	mScale(inSettings.mScale)
{
	if (outResult.HasError())
		return;

	if (ScaleHelpers::IsZeroScale(inSettings.mScale))
	{
		outResult.SetError("Can't use zero scale!");
		return;
	}

	outResult.Set(this);
}

MassProperties ScaledShape::GetMassProperties() const
{
	MassProperties p = mInnerShape->GetMassProperties();
	p.Scale(mScale);
	return p;
}

AABox ScaledShape::GetLocalBounds() const
{
	return mInnerShape->GetLocalBounds().Scaled(mScale);
}

AABox ScaledShape::GetWorldSpaceBounds(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale) const
{
	return mInnerShape->GetWorldSpaceBounds(inCenterOfMassTransform, inScale * mScale);
}

TransformedShape ScaledShape::GetSubShapeTransformedShape(const SubShapeID &inSubShapeID, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale, SubShapeID &outRemainder) const
{
	// We don't use any bits in the sub shape ID
	outRemainder = inSubShapeID;

	TransformedShape ts(RVec3(inPositionCOM), inRotation, mInnerShape, BodyID());
	ts.SetShapeScale(inScale * mScale);
	return ts;
}

Vec3 ScaledShape::GetSurfaceNormal(const SubShapeID &inSubShapeID, Vec3Arg inLocalSurfacePosition) const
{
	// Transform the surface point to local space and pass the query on
	Vec3 normal = mInnerShape->GetSurfaceNormal(inSubShapeID, inLocalSurfacePosition / mScale);

	// Need to transform the plane normals using inScale
	// Transforming a direction with matrix M is done through multiplying by (M^-1)^T
	// In this case M is a diagonal matrix with the scale vector, so we need to multiply our normal by 1 / scale and renormalize afterwards
	return (normal / mScale).Normalized();
}

void ScaledShape::GetSupportingFace(const SubShapeID &inSubShapeID, Vec3Arg inDirection, Vec3Arg inScale, Mat44Arg inCenterOfMassTransform, SupportingFace &outVertices) const
{
	mInnerShape->GetSupportingFace(inSubShapeID, inDirection, inScale * mScale, inCenterOfMassTransform, outVertices);
}

void ScaledShape::GetSubmergedVolume(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const Plane &inSurface, float &outTotalVolume, float &outSubmergedVolume, Vec3 &outCenterOfBuoyancy MOSS_IF_DEBUG_RENDERER(, RVec3Arg inBaseOffset)) const
{
	mInnerShape->GetSubmergedVolume(inCenterOfMassTransform, inScale * mScale, inSurface, outTotalVolume, outSubmergedVolume, outCenterOfBuoyancy MOSS_IF_DEBUG_RENDERER(, inBaseOffset));
}

#ifndef MOSS_DEBUG_RENDERER
void ScaledShape::Draw(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, Vec3Arg inScale, ColorArg inColor, bool inUseMaterialColors, bool inDrawWireframe) const
{
	mInnerShape->Draw(inRenderer, inCenterOfMassTransform, inScale * mScale, inColor, inUseMaterialColors, inDrawWireframe);
}

void ScaledShape::DrawGetSupportFunction(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, Vec3Arg inScale, ColorArg inColor, bool inDrawSupportDirection) const
{
	mInnerShape->DrawGetSupportFunction(inRenderer, inCenterOfMassTransform, inScale * mScale, inColor, inDrawSupportDirection);
}

void ScaledShape::DrawGetSupportingFace(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, Vec3Arg inScale) const
{
	mInnerShape->DrawGetSupportingFace(inRenderer, inCenterOfMassTransform, inScale * mScale);
}
#endif // MOSS_DEBUG_RENDERER

bool ScaledShape::CastRay(const RayCast &inRay, const SubShapeIDCreator &inSubShapeIDCreator, RayCastResult &ioHit) const
{
	Vec3 inv_scale = mScale.Reciprocal();
	RayCast scaled_ray { inv_scale * inRay.mOrigin, inv_scale * inRay.mDirection };
	return mInnerShape->CastRay(scaled_ray, inSubShapeIDCreator, ioHit);
}

void ScaledShape::CastRay(const RayCast &inRay, const RayCastSettings &inRayCastSettings, const SubShapeIDCreator &inSubShapeIDCreator, CastRayCollector &ioCollector, const ShapeFilter &inShapeFilter) const
{
	// Test shape filter
	if (!inShapeFilter.ShouldCollide(this, inSubShapeIDCreator.GetID()))
		return;

	Vec3 inv_scale = mScale.Reciprocal();
	RayCast scaled_ray { inv_scale * inRay.mOrigin, inv_scale * inRay.mDirection };
	return mInnerShape->CastRay(scaled_ray, inRayCastSettings, inSubShapeIDCreator, ioCollector, inShapeFilter);
}

void ScaledShape::CollidePoint(Vec3Arg inPoint, const SubShapeIDCreator &inSubShapeIDCreator, CollidePointCollector &ioCollector, const ShapeFilter &inShapeFilter) const
{
	// Test shape filter
	if (!inShapeFilter.ShouldCollide(this, inSubShapeIDCreator.GetID()))
		return;

	Vec3 inv_scale = mScale.Reciprocal();
	mInnerShape->CollidePoint(inv_scale * inPoint, inSubShapeIDCreator, ioCollector, inShapeFilter);
}

void ScaledShape::CollideSoftBodyVertices(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const CollideSoftBodyVertexIterator &inVertices, uint inNumVertices, int inCollidingShapeIndex) const
{
	mInnerShape->CollideSoftBodyVertices(inCenterOfMassTransform, inScale * mScale, inVertices, inNumVertices, inCollidingShapeIndex);
}

void ScaledShape::CollectTransformedShapes(const AABox &inBox, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale, const SubShapeIDCreator &inSubShapeIDCreator, TransformedShapeCollector &ioCollector, const ShapeFilter &inShapeFilter) const
{
	// Test shape filter
	if (!inShapeFilter.ShouldCollide(this, inSubShapeIDCreator.GetID()))
		return;

	mInnerShape->CollectTransformedShapes(inBox, inPositionCOM, inRotation, inScale * mScale, inSubShapeIDCreator, ioCollector, inShapeFilter);
}

void ScaledShape::TransformShape(Mat44Arg inCenterOfMassTransform, TransformedShapeCollector &ioCollector) const
{
	mInnerShape->TransformShape(inCenterOfMassTransform * Mat44::sScale(mScale), ioCollector);
}

void ScaledShape::SaveBinaryState(StreamOut &inStream) const
{
	DecoratedShape::SaveBinaryState(inStream);

	inStream.Write(mScale);
}

void ScaledShape::RestoreBinaryState(StreamIn &inStream)
{
	DecoratedShape::RestoreBinaryState(inStream);

	inStream.Read(mScale);
}

float ScaledShape::GetVolume() const
{
	return abs(mScale.GetX() * mScale.GetY() * mScale.GetZ()) * mInnerShape->GetVolume();
}

bool ScaledShape::IsValidScale(Vec3Arg inScale) const
{
	return mInnerShape->IsValidScale(inScale * mScale);
}

Vec3 ScaledShape::MakeScaleValid(Vec3Arg inScale) const
{
	return mInnerShape->MakeScaleValid(mScale * inScale) / mScale;
}

void ScaledShape::sCollideScaledVsShape(const Shape *inShape1, const Shape *inShape2, Vec3Arg inScale1, Vec3Arg inScale2, Mat44Arg inCenterOfMassTransform1, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, const CollideShapeSettings &inCollideShapeSettings, CollideShapeCollector &ioCollector, const ShapeFilter &inShapeFilter)
{
	MOSS_ASSERT(inShape1->GetSubType() == EShapeSubType::Scaled);
	const ScaledShape *shape1 = static_cast<const ScaledShape *>(inShape1);

	CollisionDispatch::sCollideShapeVsShape(shape1->GetInnerShape(), inShape2, inScale1 * shape1->GetScale(), inScale2, inCenterOfMassTransform1, inCenterOfMassTransform2, inSubShapeIDCreator1, inSubShapeIDCreator2, inCollideShapeSettings, ioCollector, inShapeFilter);
}

void ScaledShape::sCollideShapeVsScaled(const Shape *inShape1, const Shape *inShape2, Vec3Arg inScale1, Vec3Arg inScale2, Mat44Arg inCenterOfMassTransform1, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, const CollideShapeSettings &inCollideShapeSettings, CollideShapeCollector &ioCollector, const ShapeFilter &inShapeFilter)
{
	MOSS_ASSERT(inShape2->GetSubType() == EShapeSubType::Scaled);
	const ScaledShape *shape2 = static_cast<const ScaledShape *>(inShape2);

	CollisionDispatch::sCollideShapeVsShape(inShape1, shape2->GetInnerShape(), inScale1, inScale2 * shape2->GetScale(), inCenterOfMassTransform1, inCenterOfMassTransform2, inSubShapeIDCreator1, inSubShapeIDCreator2, inCollideShapeSettings, ioCollector, inShapeFilter);
}

void ScaledShape::sCastScaledVsShape(const ShapeCast &inShapeCast, const ShapeCastSettings &inShapeCastSettings, const Shape *inShape, Vec3Arg inScale, const ShapeFilter &inShapeFilter, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, CastShapeCollector &ioCollector)
{
	MOSS_ASSERT(inShapeCast.mShape->GetSubType() == EShapeSubType::Scaled);
	const ScaledShape *shape = static_cast<const ScaledShape *>(inShapeCast.mShape);

	ShapeCast scaled_cast(shape->GetInnerShape(), inShapeCast.mScale * shape->GetScale(), inShapeCast.mCenterOfMassStart, inShapeCast.mDirection);
	CollisionDispatch::sCastShapeVsShapeLocalSpace(scaled_cast, inShapeCastSettings, inShape, inScale, inShapeFilter, inCenterOfMassTransform2, inSubShapeIDCreator1, inSubShapeIDCreator2, ioCollector);
}

void ScaledShape::sCastShapeVsScaled(const ShapeCast &inShapeCast, const ShapeCastSettings &inShapeCastSettings, const Shape *inShape, Vec3Arg inScale, const ShapeFilter &inShapeFilter, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, CastShapeCollector &ioCollector)
{
	MOSS_ASSERT(inShape->GetSubType() == EShapeSubType::Scaled);
	const ScaledShape *shape = static_cast<const ScaledShape *>(inShape);

	CollisionDispatch::sCastShapeVsShapeLocalSpace(inShapeCast, inShapeCastSettings, shape->mInnerShape, inScale * shape->mScale, inShapeFilter, inCenterOfMassTransform2, inSubShapeIDCreator1, inSubShapeIDCreator2, ioCollector);
}

void ScaledShape::sRegister()
{
	ShapeFunctions &f = ShapeFunctions::sGet(EShapeSubType::Scaled);
	f.mConstruct = []() -> Shape * { return new ScaledShape; };
	f.mColor = Color::sYellow;

	for (EShapeSubType s : sAllSubShapeTypes)
	{
		CollisionDispatch::sRegisterCollideShape(EShapeSubType::Scaled, s, sCollideScaledVsShape);
		CollisionDispatch::sRegisterCollideShape(s, EShapeSubType::Scaled, sCollideShapeVsScaled);
		CollisionDispatch::sRegisterCastShape(EShapeSubType::Scaled, s, sCastScaledVsShape);
		CollisionDispatch::sRegisterCastShape(s, EShapeSubType::Scaled, sCastShapeVsScaled);
	}
}















ShapeSettings::ShapeResult OffsetCenterOfMassShapeSettings::Create() const
{
	if (mCachedResult.IsEmpty())
		Ref<Shape> shape = new OffsetCenterOfMassShape(*this, mCachedResult);
	return mCachedResult;
}

OffsetCenterOfMassShape::OffsetCenterOfMassShape(const OffsetCenterOfMassShapeSettings &inSettings, ShapeResult &outResult) :
	DecoratedShape(EShapeSubType::OffsetCenterOfMass, inSettings, outResult),
	mOffset(inSettings.mOffset)
{
	if (outResult.HasError())
		return;

	outResult.Set(this);
}

AABox OffsetCenterOfMassShape::GetLocalBounds() const
{
	AABox bounds = mInnerShape->GetLocalBounds();
	bounds.mMin -= mOffset;
	bounds.mMax -= mOffset;
	return bounds;
}

AABox OffsetCenterOfMassShape::GetWorldSpaceBounds(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale) const
{
	return mInnerShape->GetWorldSpaceBounds(inCenterOfMassTransform.PreTranslated(-inScale * mOffset), inScale);
}

TransformedShape OffsetCenterOfMassShape::GetSubShapeTransformedShape(const SubShapeID &inSubShapeID, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale, SubShapeID &outRemainder) const
{
	// We don't use any bits in the sub shape ID
	outRemainder = inSubShapeID;

	TransformedShape ts(RVec3(inPositionCOM - inRotation * (inScale * mOffset)), inRotation, mInnerShape, BodyID());
	ts.SetShapeScale(inScale);
	return ts;
}

Vec3 OffsetCenterOfMassShape::GetSurfaceNormal(const SubShapeID &inSubShapeID, Vec3Arg inLocalSurfacePosition) const
{
	// Transform surface position to local space and pass call on
	return mInnerShape->GetSurfaceNormal(inSubShapeID, inLocalSurfacePosition + mOffset);
}

void OffsetCenterOfMassShape::GetSupportingFace(const SubShapeID &inSubShapeID, Vec3Arg inDirection, Vec3Arg inScale, Mat44Arg inCenterOfMassTransform, SupportingFace &outVertices) const
{
	mInnerShape->GetSupportingFace(inSubShapeID, inDirection, inScale, inCenterOfMassTransform.PreTranslated(-inScale * mOffset), outVertices);
}

void OffsetCenterOfMassShape::GetSubmergedVolume(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const Plane &inSurface, float &outTotalVolume, float &outSubmergedVolume, Vec3 &outCenterOfBuoyancy MOSS_IF_DEBUG_RENDERER(, RVec3Arg inBaseOffset)) const
{
	mInnerShape->GetSubmergedVolume(inCenterOfMassTransform.PreTranslated(-inScale * mOffset), inScale, inSurface, outTotalVolume, outSubmergedVolume, outCenterOfBuoyancy MOSS_IF_DEBUG_RENDERER(, inBaseOffset));
}

#ifndef MOSS_DEBUG_RENDERER
void OffsetCenterOfMassShape::Draw(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, Vec3Arg inScale, ColorArg inColor, bool inUseMaterialColors, bool inDrawWireframe) const
{
	mInnerShape->Draw(inRenderer, inCenterOfMassTransform.PreTranslated(-inScale * mOffset), inScale, inColor, inUseMaterialColors, inDrawWireframe);
}

void OffsetCenterOfMassShape::DrawGetSupportFunction(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, Vec3Arg inScale, ColorArg inColor, bool inDrawSupportDirection) const
{
	mInnerShape->DrawGetSupportFunction(inRenderer, inCenterOfMassTransform.PreTranslated(-inScale * mOffset), inScale, inColor, inDrawSupportDirection);
}

void OffsetCenterOfMassShape::DrawGetSupportingFace(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, Vec3Arg inScale) const
{
	mInnerShape->DrawGetSupportingFace(inRenderer, inCenterOfMassTransform.PreTranslated(-inScale * mOffset), inScale);
}
#endif // MOSS_DEBUG_RENDERER

bool OffsetCenterOfMassShape::CastRay(const RayCast &inRay, const SubShapeIDCreator &inSubShapeIDCreator, RayCastResult &ioHit) const
{
	// Transform the ray to local space
	RayCast ray = inRay;
	ray.mOrigin += mOffset;

	return mInnerShape->CastRay(ray, inSubShapeIDCreator, ioHit);
}

void OffsetCenterOfMassShape::CastRay(const RayCast &inRay, const RayCastSettings &inRayCastSettings, const SubShapeIDCreator &inSubShapeIDCreator, CastRayCollector &ioCollector, const ShapeFilter &inShapeFilter) const
{
	// Test shape filter
	if (!inShapeFilter.ShouldCollide(this, inSubShapeIDCreator.GetID()))
		return;

	// Transform the ray to local space
	RayCast ray = inRay;
	ray.mOrigin += mOffset;

	return mInnerShape->CastRay(ray, inRayCastSettings, inSubShapeIDCreator, ioCollector, inShapeFilter);
}

void OffsetCenterOfMassShape::CollidePoint(Vec3Arg inPoint, const SubShapeIDCreator &inSubShapeIDCreator, CollidePointCollector &ioCollector, const ShapeFilter &inShapeFilter) const
{
	// Test shape filter
	if (!inShapeFilter.ShouldCollide(this, inSubShapeIDCreator.GetID()))
		return;

	// Pass the point on to the inner shape in local space
	mInnerShape->CollidePoint(inPoint + mOffset, inSubShapeIDCreator, ioCollector, inShapeFilter);
}

void OffsetCenterOfMassShape::CollideSoftBodyVertices(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const CollideSoftBodyVertexIterator &inVertices, uint inNumVertices, int inCollidingShapeIndex) const
{
	mInnerShape->CollideSoftBodyVertices(inCenterOfMassTransform.PreTranslated(-inScale * mOffset), inScale, inVertices, inNumVertices, inCollidingShapeIndex);
}

void OffsetCenterOfMassShape::CollectTransformedShapes(const AABox &inBox, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale, const SubShapeIDCreator &inSubShapeIDCreator, TransformedShapeCollector &ioCollector, const ShapeFilter &inShapeFilter) const
{
	// Test shape filter
	if (!inShapeFilter.ShouldCollide(this, inSubShapeIDCreator.GetID()))
		return;

	mInnerShape->CollectTransformedShapes(inBox, inPositionCOM - inRotation * (inScale * mOffset), inRotation, inScale, inSubShapeIDCreator, ioCollector, inShapeFilter);
}

void OffsetCenterOfMassShape::TransformShape(Mat44Arg inCenterOfMassTransform, TransformedShapeCollector &ioCollector) const
{
	mInnerShape->TransformShape(inCenterOfMassTransform.PreTranslated(-mOffset), ioCollector);
}

void OffsetCenterOfMassShape::sCollideOffsetCenterOfMassVsShape(const Shape *inShape1, const Shape *inShape2, Vec3Arg inScale1, Vec3Arg inScale2, Mat44Arg inCenterOfMassTransform1, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, const CollideShapeSettings &inCollideShapeSettings, CollideShapeCollector &ioCollector, const ShapeFilter &inShapeFilter)
{
	MOSS_ASSERT(inShape1->GetSubType() == EShapeSubType::OffsetCenterOfMass);
	const OffsetCenterOfMassShape *shape1 = static_cast<const OffsetCenterOfMassShape *>(inShape1);

	CollisionDispatch::sCollideShapeVsShape(shape1->mInnerShape, inShape2, inScale1, inScale2, inCenterOfMassTransform1.PreTranslated(-inScale1 * shape1->mOffset), inCenterOfMassTransform2, inSubShapeIDCreator1, inSubShapeIDCreator2, inCollideShapeSettings, ioCollector, inShapeFilter);
}

void OffsetCenterOfMassShape::sCollideShapeVsOffsetCenterOfMass(const Shape *inShape1, const Shape *inShape2, Vec3Arg inScale1, Vec3Arg inScale2, Mat44Arg inCenterOfMassTransform1, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, const CollideShapeSettings &inCollideShapeSettings, CollideShapeCollector &ioCollector, const ShapeFilter &inShapeFilter)
{
	MOSS_ASSERT(inShape2->GetSubType() == EShapeSubType::OffsetCenterOfMass);
	const OffsetCenterOfMassShape *shape2 = static_cast<const OffsetCenterOfMassShape *>(inShape2);

	CollisionDispatch::sCollideShapeVsShape(inShape1, shape2->mInnerShape, inScale1, inScale2, inCenterOfMassTransform1, inCenterOfMassTransform2.PreTranslated(-inScale2 * shape2->mOffset), inSubShapeIDCreator1, inSubShapeIDCreator2, inCollideShapeSettings, ioCollector, inShapeFilter);
}

void OffsetCenterOfMassShape::sCastOffsetCenterOfMassVsShape(const ShapeCast &inShapeCast, const ShapeCastSettings &inShapeCastSettings, const Shape *inShape, Vec3Arg inScale, const ShapeFilter &inShapeFilter, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, CastShapeCollector &ioCollector)
{
	// Fetch offset center of mass shape from cast shape
	MOSS_ASSERT(inShapeCast.mShape->GetSubType() == EShapeSubType::OffsetCenterOfMass);
	const OffsetCenterOfMassShape *shape1 = static_cast<const OffsetCenterOfMassShape *>(inShapeCast.mShape);

	// Transform the shape cast and update the shape
	ShapeCast shape_cast(shape1->mInnerShape, inShapeCast.mScale, inShapeCast.mCenterOfMassStart.PreTranslated(-inShapeCast.mScale * shape1->mOffset), inShapeCast.mDirection);

	CollisionDispatch::sCastShapeVsShapeLocalSpace(shape_cast, inShapeCastSettings, inShape, inScale, inShapeFilter, inCenterOfMassTransform2, inSubShapeIDCreator1, inSubShapeIDCreator2, ioCollector);
}

void OffsetCenterOfMassShape::sCastShapeVsOffsetCenterOfMass(const ShapeCast &inShapeCast, const ShapeCastSettings &inShapeCastSettings, const Shape *inShape, Vec3Arg inScale, const ShapeFilter &inShapeFilter, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, CastShapeCollector &ioCollector)
{
	MOSS_ASSERT(inShape->GetSubType() == EShapeSubType::OffsetCenterOfMass);
	const OffsetCenterOfMassShape *shape = static_cast<const OffsetCenterOfMassShape *>(inShape);

	// Transform the shape cast
	ShapeCast shape_cast = inShapeCast.PostTransformed(Mat44::sTranslation(inScale * shape->mOffset));

	CollisionDispatch::sCastShapeVsShapeLocalSpace(shape_cast, inShapeCastSettings, shape->mInnerShape, inScale, inShapeFilter, inCenterOfMassTransform2.PreTranslated(-inScale * shape->mOffset), inSubShapeIDCreator1, inSubShapeIDCreator2, ioCollector);
}

void OffsetCenterOfMassShape::SaveBinaryState(StreamOut &inStream) const
{
	DecoratedShape::SaveBinaryState(inStream);

	inStream.Write(mOffset);
}

void OffsetCenterOfMassShape::RestoreBinaryState(StreamIn &inStream)
{
	DecoratedShape::RestoreBinaryState(inStream);

	inStream.Read(mOffset);
}

void OffsetCenterOfMassShape::sRegister()
{
	ShapeFunctions &f = ShapeFunctions::sGet(EShapeSubType::OffsetCenterOfMass);
	f.mConstruct = []() -> Shape * { return new OffsetCenterOfMassShape; };
	f.mColor = Color::sCyan;

	for (EShapeSubType s : sAllSubShapeTypes)
	{
		CollisionDispatch::sRegisterCollideShape(EShapeSubType::OffsetCenterOfMass, s, sCollideOffsetCenterOfMassVsShape);
		CollisionDispatch::sRegisterCollideShape(s, EShapeSubType::OffsetCenterOfMass, sCollideShapeVsOffsetCenterOfMass);
		CollisionDispatch::sRegisterCastShape(EShapeSubType::OffsetCenterOfMass, s, sCastOffsetCenterOfMassVsShape);
		CollisionDispatch::sRegisterCastShape(s, EShapeSubType::OffsetCenterOfMass, sCastShapeVsOffsetCenterOfMass);
	}
}

MOSS_SUPRESS_WARNINGS_END
