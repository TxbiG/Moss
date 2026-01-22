// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT


#include <Moss/Physics/physics_intern.h>
#ifndef MOSS_DEBUG_RENDERER
	#include <Moss/Moss_Renderer.h>
#endif // MOSS_DEBUG_RENDERER

#include <Moss/ObjectStream/TypeDeclarations.h>
#include <Moss/Core/StreamIn.h>
#include <Moss/Core/StreamOut.h>
#include <Moss/ObjectStream/TypeDeclarations.h>
#include <Moss/Core/StreamIn.h>
#include <Moss/Core/StreamOut.h>
#include <Moss/Core/QuickSort.h>
#include <Moss/Core/Variants/TMap.h>
#include <Moss/Core/Variants/TSet.h>
#include <Moss/Core/BinaryHeap.h>
#include <Moss/Core/Profiler.h>
#include <Moss/Physics/Geometry/RayTriangle.h>
#include <Moss/Physics/Collision/RayCast.h>
#include <Moss/Physics/Collision/CastResult.h>
#include <Moss/Physics/Collision/TransformedShape.h>
#include <Moss/Physics/Collision/CastConvexVsTriangles.h>
#include <Moss/Physics/Collision/CastSphereVsTriangles.h>
#include <Moss/Physics/Collision/CollideConvexVsTriangles.h>
#include <Moss/Physics/Collision/CollideSphereVsTriangles.h>
#include <Moss/Physics/Collision/CollisionDispatch.h>
#include <Moss/Physics/Collision/CollideSoftBodyVertexIterator.h>
#include <Moss/Physics/Collision/SimShapeFilterWrapper.h>
#include <Moss/Physics/PhysicsSystem.h>
#include <Moss/Core/ScopeExit.h>

MOSS_SUPRESS_WARNINGS_BEGIN

MOSS_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(SoftBodyCreationSettings)
{
	MOSS_ADD_ATTRIBUTE(SoftBodyCreationSettings, mSettings)
	MOSS_ADD_ATTRIBUTE(SoftBodyCreationSettings, mPosition)
	MOSS_ADD_ATTRIBUTE(SoftBodyCreationSettings, mRotation)
	MOSS_ADD_ATTRIBUTE(SoftBodyCreationSettings, mUserData)
	MOSS_ADD_ENUM_ATTRIBUTE(SoftBodyCreationSettings, mObjectLayer)
	MOSS_ADD_ATTRIBUTE(SoftBodyCreationSettings, mCollisionGroup)
	MOSS_ADD_ATTRIBUTE(SoftBodyCreationSettings, mNumIterations)
	MOSS_ADD_ATTRIBUTE(SoftBodyCreationSettings, mLinearDamping)
	MOSS_ADD_ATTRIBUTE(SoftBodyCreationSettings, mMaxLinearVelocity)
	MOSS_ADD_ATTRIBUTE(SoftBodyCreationSettings, mRestitution)
	MOSS_ADD_ATTRIBUTE(SoftBodyCreationSettings, mFriction)
	MOSS_ADD_ATTRIBUTE(SoftBodyCreationSettings, mPressure)
	MOSS_ADD_ATTRIBUTE(SoftBodyCreationSettings, mGravityFactor)
	MOSS_ADD_ATTRIBUTE(SoftBodyCreationSettings, mUpdatePosition)
	MOSS_ADD_ATTRIBUTE(SoftBodyCreationSettings, mMakeRotationIdentity)
	MOSS_ADD_ATTRIBUTE(SoftBodyCreationSettings, mAllowSleeping)
}

MOSS_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(SoftBodySharedSettings::Vertex)
{
	MOSS_ADD_ATTRIBUTE(SoftBodySharedSettings::Vertex, mPosition)
	MOSS_ADD_ATTRIBUTE(SoftBodySharedSettings::Vertex, mVelocity)
	MOSS_ADD_ATTRIBUTE(SoftBodySharedSettings::Vertex, mInvMass)
}

MOSS_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(SoftBodySharedSettings::Face)
{
	MOSS_ADD_ATTRIBUTE(SoftBodySharedSettings::Face, mVertex)
	MOSS_ADD_ATTRIBUTE(SoftBodySharedSettings::Face, mMaterialIndex)
}

MOSS_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(SoftBodySharedSettings::Edge)
{
	MOSS_ADD_ATTRIBUTE(SoftBodySharedSettings::Edge, mVertex)
	MOSS_ADD_ATTRIBUTE(SoftBodySharedSettings::Edge, mRestLength)
	MOSS_ADD_ATTRIBUTE(SoftBodySharedSettings::Edge, mCompliance)
}

MOSS_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(SoftBodySharedSettings::DihedralBend)
{
	MOSS_ADD_ATTRIBUTE(SoftBodySharedSettings::DihedralBend, mVertex)
	MOSS_ADD_ATTRIBUTE(SoftBodySharedSettings::DihedralBend, mCompliance)
	MOSS_ADD_ATTRIBUTE(SoftBodySharedSettings::DihedralBend, mInitialAngle)
}

MOSS_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(SoftBodySharedSettings::Volume)
{
	MOSS_ADD_ATTRIBUTE(SoftBodySharedSettings::Volume, mVertex)
	MOSS_ADD_ATTRIBUTE(SoftBodySharedSettings::Volume, mSixRestVolume)
	MOSS_ADD_ATTRIBUTE(SoftBodySharedSettings::Volume, mCompliance)
}

MOSS_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(SoftBodySharedSettings::InvBind)
{
	MOSS_ADD_ATTRIBUTE(SoftBodySharedSettings::InvBind, mJointIndex)
	MOSS_ADD_ATTRIBUTE(SoftBodySharedSettings::InvBind, mInvBind)
}

MOSS_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(SoftBodySharedSettings::SkinWeight)
{
	MOSS_ADD_ATTRIBUTE(SoftBodySharedSettings::SkinWeight, mInvBindIndex)
	MOSS_ADD_ATTRIBUTE(SoftBodySharedSettings::SkinWeight, mWeight)
}

MOSS_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(SoftBodySharedSettings::Skinned)
{
	MOSS_ADD_ATTRIBUTE(SoftBodySharedSettings::Skinned, mVertex)
	MOSS_ADD_ATTRIBUTE(SoftBodySharedSettings::Skinned, mWeights)
	MOSS_ADD_ATTRIBUTE(SoftBodySharedSettings::Skinned, mMaxDistance)
	MOSS_ADD_ATTRIBUTE(SoftBodySharedSettings::Skinned, mBackStopDistance)
	MOSS_ADD_ATTRIBUTE(SoftBodySharedSettings::Skinned, mBackStopRadius)
}

MOSS_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(SoftBodySharedSettings::LRA)
{
	MOSS_ADD_ATTRIBUTE(SoftBodySharedSettings::LRA, mVertex)
	MOSS_ADD_ATTRIBUTE(SoftBodySharedSettings::LRA, mMaxDistance)
}

MOSS_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(SoftBodySharedSettings)
{
	MOSS_ADD_ATTRIBUTE(SoftBodySharedSettings, mVertices)
	MOSS_ADD_ATTRIBUTE(SoftBodySharedSettings, mFaces)
	MOSS_ADD_ATTRIBUTE(SoftBodySharedSettings, mEdgeConstraints)
	MOSS_ADD_ATTRIBUTE(SoftBodySharedSettings, mDihedralBendConstraints)
	MOSS_ADD_ATTRIBUTE(SoftBodySharedSettings, mVolumeConstraints)
	MOSS_ADD_ATTRIBUTE(SoftBodySharedSettings, mSkinnedConstraints)
	MOSS_ADD_ATTRIBUTE(SoftBodySharedSettings, mInvBindMatrices)
	MOSS_ADD_ATTRIBUTE(SoftBodySharedSettings, mLRAConstraints)
	MOSS_ADD_ATTRIBUTE(SoftBodySharedSettings, mMaterials)
	MOSS_ADD_ATTRIBUTE(SoftBodySharedSettings, mVertexRadius)
}

void SoftBodyCreationSettings::SaveBinaryState(StreamOut &inStream) const
{
	inStream.Write(mPosition);
	inStream.Write(mRotation);
	inStream.Write(mUserData);
	inStream.Write(mObjectLayer);
	mCollisionGroup.SaveBinaryState(inStream);
	inStream.Write(mNumIterations);
	inStream.Write(mLinearDamping);
	inStream.Write(mMaxLinearVelocity);
	inStream.Write(mRestitution);
	inStream.Write(mFriction);
	inStream.Write(mPressure);
	inStream.Write(mGravityFactor);
	inStream.Write(mUpdatePosition);
	inStream.Write(mMakeRotationIdentity);
	inStream.Write(mAllowSleeping);
}

void SoftBodyCreationSettings::RestoreBinaryState(StreamIn &inStream)
{
	inStream.Read(mPosition);
	inStream.Read(mRotation);
	inStream.Read(mUserData);
	inStream.Read(mObjectLayer);
	mCollisionGroup.RestoreBinaryState(inStream);
	inStream.Read(mNumIterations);
	inStream.Read(mLinearDamping);
	inStream.Read(mMaxLinearVelocity);
	inStream.Read(mRestitution);
	inStream.Read(mFriction);
	inStream.Read(mPressure);
	inStream.Read(mGravityFactor);
	inStream.Read(mUpdatePosition);
	inStream.Read(mMakeRotationIdentity);
	inStream.Read(mAllowSleeping);
}

void SoftBodyCreationSettings::SaveWithChildren(StreamOut &inStream, SharedSettingsToIDMap *ioSharedSettingsMap, MaterialToIDMap *ioMaterialMap, GroupFilterToIDMap *ioGroupFilterMap) const
{
	// Save creation settings
	SaveBinaryState(inStream);

	// Save shared settings
	if (ioSharedSettingsMap != nullptr && ioMaterialMap != nullptr)
		mSettings->SaveWithMaterials(inStream, *ioSharedSettingsMap, *ioMaterialMap);
	else
		inStream.Write(~uint32(0));

	// Save group filter
	StreamUtils::SaveObjectReference(inStream, mCollisionGroup.GetGroupFilter(), ioGroupFilterMap);
}

SoftBodyCreationSettings::SBCSResult SoftBodyCreationSettings::sRestoreWithChildren(StreamIn &inStream, IDToSharedSettingsMap &ioSharedSettingsMap, IDToMaterialMap &ioMaterialMap, IDToGroupFilterMap &ioGroupFilterMap)
{
	SBCSResult result;

	// Read creation settings
	SoftBodyCreationSettings settings;
	settings.RestoreBinaryState(inStream);
	if (inStream.IsEOF() || inStream.IsFailed())
	{
		result.SetError("Error reading body creation settings");
		return result;
	}

	// Read shared settings
	SoftBodySharedSettings::SettingsResult settings_result = SoftBodySharedSettings::sRestoreWithMaterials(inStream, ioSharedSettingsMap, ioMaterialMap);
	if (settings_result.HasError())
	{
		result.SetError(settings_result.GetError());
		return result;
	}
	settings.mSettings = settings_result.Get();

	// Read group filter
	Result gfresult = StreamUtils::RestoreObjectReference(inStream, ioGroupFilterMap);
	if (gfresult.HasError())
	{
		result.SetError(gfresult.GetError());
		return result;
	}
	settings.mCollisionGroup.SetGroupFilter(gfresult.Get());

	result.Set(settings);
	return result;
}


uint SoftBodyShape::GetSubShapeIDBits() const
{
	// Ensure we have enough bits to encode our shape [0, n - 1]
	uint32 n = (uint32)mSoftBodyMotionProperties->GetFaces().size() - 1;
	return 32 - CountLeadingZeros(n);
}

uint32 SoftBodyShape::GetFaceIndex(const SubShapeID &inSubShapeID) const
{
	SubShapeID remainder;
	uint32 face_index = inSubShapeID.PopID(GetSubShapeIDBits(), remainder);
	MOSS_ASSERT(remainder.IsEmpty());
	return face_index;
}

AABox SoftBodyShape::GetLocalBounds() const
{
	return mSoftBodyMotionProperties->GetLocalBounds();
}

bool SoftBodyShape::CastRay(const RayCast &inRay, const SubShapeIDCreator &inSubShapeIDCreator, RayCastResult &ioHit) const
{
	MOSS_PROFILE_FUNCTION();

	uint num_triangle_bits = GetSubShapeIDBits();
	uint triangle_idx = uint(-1);

	const TArray<SoftBodyVertex> &vertices = mSoftBodyMotionProperties->GetVertices();
	for (const SoftBodyMotionProperties::Face &f : mSoftBodyMotionProperties->GetFaces())
	{
		Vec3 x1 = vertices[f.mVertex[0]].mPosition;
		Vec3 x2 = vertices[f.mVertex[1]].mPosition;
		Vec3 x3 = vertices[f.mVertex[2]].mPosition;

		float fraction = RayTriangle(inRay.mOrigin, inRay.mDirection, x1, x2, x3);
		if (fraction < ioHit.mFraction)
		{
			// Store fraction
			ioHit.mFraction = fraction;

			// Store triangle index
			triangle_idx = uint(&f - mSoftBodyMotionProperties->GetFaces().data());
		}
	}

	if (triangle_idx == uint(-1))
		return false;

	ioHit.mSubShapeID2 = inSubShapeIDCreator.PushID(triangle_idx, num_triangle_bits).GetID();
	return true;
}

void SoftBodyShape::CastRay(const RayCast &inRay, const RayCastSettings &inRayCastSettings, const SubShapeIDCreator &inSubShapeIDCreator, CastRayCollector &ioCollector, const ShapeFilter &inShapeFilter) const
{
	MOSS_PROFILE_FUNCTION();

	// Test shape filter
	if (!inShapeFilter.ShouldCollide(this, inSubShapeIDCreator.GetID()))
		return;

	uint num_triangle_bits = GetSubShapeIDBits();

	const TArray<SoftBodyVertex> &vertices = mSoftBodyMotionProperties->GetVertices();
	for (const SoftBodyMotionProperties::Face &f : mSoftBodyMotionProperties->GetFaces())
	{
		Vec3 x1 = vertices[f.mVertex[0]].mPosition;
		Vec3 x2 = vertices[f.mVertex[1]].mPosition;
		Vec3 x3 = vertices[f.mVertex[2]].mPosition;

		// Back facing check
		if (inRayCastSettings.mBackFaceModeTriangles == EBackFaceMode::IgnoreBackFaces && (x2 - x1).Cross(x3 - x1).Dot(inRay.mDirection) > 0.0f)
			continue;

		// Test ray against triangle
		float fraction = RayTriangle(inRay.mOrigin, inRay.mDirection, x1, x2, x3);
		if (fraction < ioCollector.GetEarlyOutFraction())
		{
			// Better hit than the current hit
			RayCastResult hit;
			hit.mBodyID = TransformedShape::sGetBodyID(ioCollector.GetContext());
			hit.mFraction = fraction;
			hit.mSubShapeID2 = inSubShapeIDCreator.PushID(uint(&f - mSoftBodyMotionProperties->GetFaces().data()), num_triangle_bits).GetID();
			ioCollector.AddHit(hit);
		}
	}
}

void SoftBodyShape::CollidePoint(Vec3Arg inPoint, const SubShapeIDCreator &inSubShapeIDCreator, CollidePointCollector &ioCollector, const ShapeFilter &inShapeFilter) const
{
	sCollidePointUsingRayCast(*this, inPoint, inSubShapeIDCreator, ioCollector, inShapeFilter);
}

void SoftBodyShape::CollideSoftBodyVertices(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const CollideSoftBodyVertexIterator &inVertices, uint inNumVertices, int inCollidingShapeIndex) const
{
	/* Not implemented */
}

const PhysicsMaterial *SoftBodyShape::GetMaterial(const SubShapeID &inSubShapeID) const
{
	SubShapeID remainder;
	uint triangle_idx = inSubShapeID.PopID(GetSubShapeIDBits(), remainder);
	MOSS_ASSERT(remainder.IsEmpty());

	const SoftBodyMotionProperties::Face &f = mSoftBodyMotionProperties->GetFace(triangle_idx);
	return mSoftBodyMotionProperties->GetMaterials()[f.mMaterialIndex];
}

Vec3 SoftBodyShape::GetSurfaceNormal(const SubShapeID &inSubShapeID, Vec3Arg inLocalSurfacePosition) const
{
	SubShapeID remainder;
	uint triangle_idx = inSubShapeID.PopID(GetSubShapeIDBits(), remainder);
	MOSS_ASSERT(remainder.IsEmpty());

	const SoftBodyMotionProperties::Face &f = mSoftBodyMotionProperties->GetFace(triangle_idx);
	const TArray<SoftBodyVertex> &vertices = mSoftBodyMotionProperties->GetVertices();

	Vec3 x1 = vertices[f.mVertex[0]].mPosition;
	Vec3 x2 = vertices[f.mVertex[1]].mPosition;
	Vec3 x3 = vertices[f.mVertex[2]].mPosition;

	return (x2 - x1).Cross(x3 - x1).NormalizedOr(Vec3::sAxisY());
}

void SoftBodyShape::GetSupportingFace(const SubShapeID &inSubShapeID, Vec3Arg inDirection, Vec3Arg inScale, Mat44Arg inCenterOfMassTransform, SupportingFace &outVertices) const
{
	SubShapeID remainder;
	uint triangle_idx = inSubShapeID.PopID(GetSubShapeIDBits(), remainder);
	MOSS_ASSERT(remainder.IsEmpty());

	const SoftBodyMotionProperties::Face &f = mSoftBodyMotionProperties->GetFace(triangle_idx);
	const TArray<SoftBodyVertex> &vertices = mSoftBodyMotionProperties->GetVertices();

	for (uint32 i : f.mVertex)
		outVertices.push_back(inCenterOfMassTransform * (inScale * vertices[i].mPosition));
}

void SoftBodyShape::GetSubmergedVolume(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const Plane &inSurface, float &outTotalVolume, float &outSubmergedVolume, Vec3 &outCenterOfBuoyancy MOSS_IF_DEBUG_RENDERER(, RVec3Arg inBaseOffset)) const
{
	outSubmergedVolume = 0.0f;
	outTotalVolume = mSoftBodyMotionProperties->GetVolume();
	outCenterOfBuoyancy = Vec3::sZero();
}

#ifndef MOSS_DEBUG_RENDERER

void SoftBodyShape::Draw(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, Vec3Arg inScale, ColorArg inColor, bool inUseMaterialColors, bool inDrawWireframe) const
{
	const TArray<SoftBodyVertex> &vertices = mSoftBodyMotionProperties->GetVertices();
	for (const SoftBodyMotionProperties::Face &f : mSoftBodyMotionProperties->GetFaces())
	{
		RVec3 x1 = inCenterOfMassTransform * vertices[f.mVertex[0]].mPosition;
		RVec3 x2 = inCenterOfMassTransform * vertices[f.mVertex[1]].mPosition;
		RVec3 x3 = inCenterOfMassTransform * vertices[f.mVertex[2]].mPosition;

		inRenderer->DrawTriangle(x1, x2, x3, inColor, DebugRenderer::ECastShadow::On);
	}
}

#endif // MOSS_DEBUG_RENDERER

struct SoftBodyShape::SBSGetTrianglesContext
{
	Mat44		mCenterOfMassTransform;
	int			mTriangleIndex;
};

void SoftBodyShape::GetTrianglesStart(GetTrianglesContext &ioContext, [[maybe_unused]] const AABox &inBox, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale) const
{
	SBSGetTrianglesContext &context = reinterpret_cast<SBSGetTrianglesContext &>(ioContext);
	context.mCenterOfMassTransform = Mat44::sRotationTranslation(inRotation, inPositionCOM) * Mat44::sScale(inScale);
	context.mTriangleIndex = 0;
}

int SoftBodyShape::GetTrianglesNext(GetTrianglesContext &ioContext, int inMaxTrianglesRequested, Float3 *outTriangleVertices, const PhysicsMaterial **outMaterials) const
{
	SBSGetTrianglesContext &context = reinterpret_cast<SBSGetTrianglesContext &>(ioContext);

	const TArray<SoftBodyMotionProperties::Face> &faces = mSoftBodyMotionProperties->GetFaces();
	const TArray<SoftBodyVertex> &vertices = mSoftBodyMotionProperties->GetVertices();
	const PhysicsMaterialList &materials = mSoftBodyMotionProperties->GetMaterials();

	int num_triangles = min(inMaxTrianglesRequested, (int)faces.size() - context.mTriangleIndex);
	for (int i = 0; i < num_triangles; ++i)
	{
		const SoftBodyMotionProperties::Face &f = faces[context.mTriangleIndex + i];

		Vec3 x1 = context.mCenterOfMassTransform * vertices[f.mVertex[0]].mPosition;
		Vec3 x2 = context.mCenterOfMassTransform * vertices[f.mVertex[1]].mPosition;
		Vec3 x3 = context.mCenterOfMassTransform * vertices[f.mVertex[2]].mPosition;

		x1.StoreFloat3(outTriangleVertices++);
		x2.StoreFloat3(outTriangleVertices++);
		x3.StoreFloat3(outTriangleVertices++);

		if (outMaterials != nullptr)
			*outMaterials++ = materials[f.mMaterialIndex];
	}

	context.mTriangleIndex += num_triangles;
	return num_triangles;
}

Shape::Stats SoftBodyShape::GetStats() const
{
	return Stats(sizeof(*this), (uint)mSoftBodyMotionProperties->GetFaces().size());
}

float SoftBodyShape::GetVolume() const
{
	return mSoftBodyMotionProperties->GetVolume();
}

void SoftBodyShape::sCollideConvexVsSoftBody(const Shape *inShape1, const Shape *inShape2, Vec3Arg inScale1, Vec3Arg inScale2, Mat44Arg inCenterOfMassTransform1, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, const CollideShapeSettings &inCollideShapeSettings, CollideShapeCollector &ioCollector, [[maybe_unused]] const ShapeFilter &inShapeFilter)
{
	MOSS_ASSERT(inShape1->GetType() == EShapeType::Convex);
	const ConvexShape *shape1 = static_cast<const ConvexShape *>(inShape1);
	MOSS_ASSERT(inShape2->GetSubType() == EShapeSubType::SoftBody);
	const SoftBodyShape *shape2 = static_cast<const SoftBodyShape *>(inShape2);

	const TArray<SoftBodyVertex> &vertices = shape2->mSoftBodyMotionProperties->GetVertices();
	const TArray<SoftBodyMotionProperties::Face> &faces = shape2->mSoftBodyMotionProperties->GetFaces();
	uint num_triangle_bits = shape2->GetSubShapeIDBits();

	CollideConvexVsTriangles collider(shape1, inScale1, inScale2, inCenterOfMassTransform1, inCenterOfMassTransform2, inSubShapeIDCreator1.GetID(), inCollideShapeSettings, ioCollector);
	for (const SoftBodyMotionProperties::Face &f : faces)
	{
		Vec3 x1 = vertices[f.mVertex[0]].mPosition;
		Vec3 x2 = vertices[f.mVertex[1]].mPosition;
		Vec3 x3 = vertices[f.mVertex[2]].mPosition;

		collider.Collide(x1, x2, x3, 0b111, inSubShapeIDCreator2.PushID(uint(&f - faces.data()), num_triangle_bits).GetID());
	}
}

void SoftBodyShape::sCollideSphereVsSoftBody(const Shape *inShape1, const Shape *inShape2, Vec3Arg inScale1, Vec3Arg inScale2, Mat44Arg inCenterOfMassTransform1, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, const CollideShapeSettings &inCollideShapeSettings, CollideShapeCollector &ioCollector, [[maybe_unused]] const ShapeFilter &inShapeFilter)
{
	MOSS_ASSERT(inShape1->GetSubType() == EShapeSubType::Sphere);
	const SphereShape *shape1 = static_cast<const SphereShape *>(inShape1);
	MOSS_ASSERT(inShape2->GetSubType() == EShapeSubType::SoftBody);
	const SoftBodyShape *shape2 = static_cast<const SoftBodyShape *>(inShape2);

	const TArray<SoftBodyVertex> &vertices = shape2->mSoftBodyMotionProperties->GetVertices();
	const TArray<SoftBodyMotionProperties::Face> &faces = shape2->mSoftBodyMotionProperties->GetFaces();
	uint num_triangle_bits = shape2->GetSubShapeIDBits();

	CollideSphereVsTriangles collider(shape1, inScale1, inScale2, inCenterOfMassTransform1, inCenterOfMassTransform2, inSubShapeIDCreator1.GetID(), inCollideShapeSettings, ioCollector);
	for (const SoftBodyMotionProperties::Face &f : faces)
	{
		Vec3 x1 = vertices[f.mVertex[0]].mPosition;
		Vec3 x2 = vertices[f.mVertex[1]].mPosition;
		Vec3 x3 = vertices[f.mVertex[2]].mPosition;

		collider.Collide(x1, x2, x3, 0b111, inSubShapeIDCreator2.PushID(uint(&f - faces.data()), num_triangle_bits).GetID());
	}
}

void SoftBodyShape::sCastConvexVsSoftBody(const ShapeCast &inShapeCast, const ShapeCastSettings &inShapeCastSettings, const Shape *inShape, Vec3Arg inScale, [[maybe_unused]] const ShapeFilter &inShapeFilter, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, CastShapeCollector &ioCollector)
{
	MOSS_ASSERT(inShape->GetSubType() == EShapeSubType::SoftBody);
	const SoftBodyShape *shape = static_cast<const SoftBodyShape *>(inShape);

	const TArray<SoftBodyVertex> &vertices = shape->mSoftBodyMotionProperties->GetVertices();
	const TArray<SoftBodyMotionProperties::Face> &faces = shape->mSoftBodyMotionProperties->GetFaces();
	uint num_triangle_bits = shape->GetSubShapeIDBits();

	CastConvexVsTriangles caster(inShapeCast, inShapeCastSettings, inScale, inCenterOfMassTransform2, inSubShapeIDCreator1, ioCollector);
	for (const SoftBodyMotionProperties::Face &f : faces)
	{
		Vec3 x1 = vertices[f.mVertex[0]].mPosition;
		Vec3 x2 = vertices[f.mVertex[1]].mPosition;
		Vec3 x3 = vertices[f.mVertex[2]].mPosition;

		caster.Cast(x1, x2, x3, 0b111, inSubShapeIDCreator2.PushID(uint(&f - faces.data()), num_triangle_bits).GetID());
	}
}

void SoftBodyShape::sCastSphereVsSoftBody(const ShapeCast &inShapeCast, const ShapeCastSettings &inShapeCastSettings, const Shape *inShape, Vec3Arg inScale, [[maybe_unused]] const ShapeFilter &inShapeFilter, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, CastShapeCollector &ioCollector)
{
	MOSS_ASSERT(inShape->GetSubType() == EShapeSubType::SoftBody);
	const SoftBodyShape *shape = static_cast<const SoftBodyShape *>(inShape);

	const TArray<SoftBodyVertex> &vertices = shape->mSoftBodyMotionProperties->GetVertices();
	const TArray<SoftBodyMotionProperties::Face> &faces = shape->mSoftBodyMotionProperties->GetFaces();
	uint num_triangle_bits = shape->GetSubShapeIDBits();

	CastSphereVsTriangles caster(inShapeCast, inShapeCastSettings, inScale, inCenterOfMassTransform2, inSubShapeIDCreator1, ioCollector);
	for (const SoftBodyMotionProperties::Face &f : faces)
	{
		Vec3 x1 = vertices[f.mVertex[0]].mPosition;
		Vec3 x2 = vertices[f.mVertex[1]].mPosition;
		Vec3 x3 = vertices[f.mVertex[2]].mPosition;

		caster.Cast(x1, x2, x3, 0b111, inSubShapeIDCreator2.PushID(uint(&f - faces.data()), num_triangle_bits).GetID());
	}
}

void SoftBodyShape::sRegister()
{
	ShapeFunctions &f = ShapeFunctions::sGet(EShapeSubType::SoftBody);
	f.mConstruct = nullptr; // Not supposed to be constructed by users!
	f.mColor = Color::sDarkGreen;

	for (EShapeSubType s : sConvexSubShapeTypes)
	{
		CollisionDispatch::sRegisterCollideShape(s, EShapeSubType::SoftBody, sCollideConvexVsSoftBody);
		CollisionDispatch::sRegisterCastShape(s, EShapeSubType::SoftBody, sCastConvexVsSoftBody);

		CollisionDispatch::sRegisterCollideShape(EShapeSubType::SoftBody, s, CollisionDispatch::sReversedCollideShape);
		CollisionDispatch::sRegisterCastShape(EShapeSubType::SoftBody, s, CollisionDispatch::sReversedCastShape);
	}

	// Specialized collision functions
	CollisionDispatch::sRegisterCollideShape(EShapeSubType::Sphere, EShapeSubType::SoftBody, sCollideSphereVsSoftBody);
	CollisionDispatch::sRegisterCastShape(EShapeSubType::Sphere, EShapeSubType::SoftBody, sCastSphereVsSoftBody);
}


void SoftBodySharedSettings::CalculateClosestKinematic()
{
	// Check if we already calculated this
	if (!mClosestKinematic.empty())
		return;

	// Reserve output size
	mClosestKinematic.resize(mVertices.size());

	// Create a list of connected vertices
	TArray<TArray<uint32>> connectivity;
	connectivity.resize(mVertices.size());
	for (const Edge &e : mEdgeConstraints)
	{
		connectivity[e.mVertex[0]].push_back(e.mVertex[1]);
		connectivity[e.mVertex[1]].push_back(e.mVertex[0]);
	}

	// Use Dijkstra's algorithm to find the closest kinematic vertex for each vertex
	// See: https://en.wikipedia.org/wiki/Dijkstra's_algorithm
	//
	// An element in the open list
	struct Open
	{
		// Order so that we get the shortest distance first
		bool	operator < (const Open &inRHS) const
		{
			return mDistance > inRHS.mDistance;
		}

		uint32	mVertex;
		float	mDistance;
	};

	// Start with all kinematic elements
	TArray<Open> to_visit;
	for (uint32 v = 0; v < mVertices.size(); ++v)
		if (mVertices[v].mInvMass == 0.0f)
		{
			mClosestKinematic[v].mVertex = v;
			mClosestKinematic[v].mDistance = 0.0f;
			to_visit.push_back({ v, 0.0f });
			BinaryHeapPush(to_visit.begin(), to_visit.end(), std::less<Open> { });
		}

	// Visit all vertices remembering the closest kinematic vertex and its distance
	MOSS_IF_ENABLE_ASSERTS(float last_closest = 0.0f;)
	while (!to_visit.empty())
	{
		// Pop element from the open list
		BinaryHeapPop(to_visit.begin(), to_visit.end(), std::less<Open> { });
		Open current = to_visit.back();
		to_visit.pop_back();
		MOSS_ASSERT(current.mDistance >= last_closest);
		MOSS_IF_ENABLE_ASSERTS(last_closest = current.mDistance;)

		// Loop through all of its connected vertices
		for (uint32 v : connectivity[current.mVertex])
		{
			// Calculate distance from the current vertex to this target vertex and check if it is smaller
			float new_distance = current.mDistance + (Vec3(mVertices[v].mPosition) - Vec3(mVertices[current.mVertex].mPosition)).Length();
			if (new_distance < mClosestKinematic[v].mDistance)
			{
				// Remember new closest vertex
				mClosestKinematic[v].mVertex = mClosestKinematic[current.mVertex].mVertex;
				mClosestKinematic[v].mDistance = new_distance;
				to_visit.push_back({ v, new_distance });
				BinaryHeapPush(to_visit.begin(), to_visit.end(), std::less<Open> { });
			}
		}
	}
}

void SoftBodySharedSettings::CreateConstraints(const VertexAttributes *inVertexAttributes, uint inVertexAttributesLength, EBendType inBendType, float inAngleTolerance)
{
	struct EdgeHelper
	{
		uint32	mVertex[2];
		uint32	mEdgeIdx;
	};

	// Create list of all edges
	TArray<EdgeHelper> edges;
	edges.reserve(mFaces.size() * 3);
	for (const Face &f : mFaces)
		for (int i = 0; i < 3; ++i)
		{
			uint32 v0 = f.mVertex[i];
			uint32 v1 = f.mVertex[(i + 1) % 3];

			EdgeHelper e;
			e.mVertex[0] = min(v0, v1);
			e.mVertex[1] = max(v0, v1);
			e.mEdgeIdx = uint32(&f - mFaces.data()) * 3 + i;
			edges.push_back(e);
		}

	// Sort the edges
	QuickSort(edges.begin(), edges.end(), [](const EdgeHelper &inLHS, const EdgeHelper &inRHS) { return inLHS.mVertex[0] < inRHS.mVertex[0] || (inLHS.mVertex[0] == inRHS.mVertex[0] && inLHS.mVertex[1] < inRHS.mVertex[1]); });

	// Only add edges if one of the vertices is movable
	auto add_edge = [this](uint32 inVtx1, uint32 inVtx2, float inCompliance1, float inCompliance2) {
		if ((mVertices[inVtx1].mInvMass > 0.0f || mVertices[inVtx2].mInvMass > 0.0f)
			&& inCompliance1 < FLT_MAX && inCompliance2 < FLT_MAX)
		{
			Edge temp_edge;
			temp_edge.mVertex[0] = inVtx1;
			temp_edge.mVertex[1] = inVtx2;
			temp_edge.mCompliance = 0.5f * (inCompliance1 + inCompliance2);
			temp_edge.mRestLength = (Vec3(mVertices[inVtx2].mPosition) - Vec3(mVertices[inVtx1].mPosition)).Length();
			MOSS_ASSERT(temp_edge.mRestLength > 0.0f);
			mEdgeConstraints.push_back(temp_edge);
		}
	};

	// Helper function to get the attributes of a vertex
	auto attr = [inVertexAttributes, inVertexAttributesLength](uint32 inVertex) {
		return inVertexAttributes[min(inVertex, inVertexAttributesLength - 1)];
	};

	// Create the constraints
	float sq_sin_tolerance = Square(Sin(inAngleTolerance));
	float sq_cos_tolerance = Square(Cos(inAngleTolerance));
	mEdgeConstraints.clear();
	mEdgeConstraints.reserve(edges.size());
	for (TArray<EdgeHelper>::size_type i = 0; i < edges.size(); ++i)
	{
		const EdgeHelper &e0 = edges[i];

		// Get attributes for the vertices of the edge
		const VertexAttributes &a0 = attr(e0.mVertex[0]);
		const VertexAttributes &a1 = attr(e0.mVertex[1]);

		// Flag that indicates if this edge is a shear edge (if 2 triangles form a quad-like shape and this edge is on the diagonal)
		bool is_shear = false;

		// Test if there are any shared edges
		for (TArray<EdgeHelper>::size_type j = i + 1; j < edges.size(); ++j)
		{
			const EdgeHelper &e1 = edges[j];
			if (e0.mVertex[0] == e1.mVertex[0] && e0.mVertex[1] == e1.mVertex[1])
			{
				// Get opposing vertices
				const Face &f0 = mFaces[e0.mEdgeIdx / 3];
				const Face &f1 = mFaces[e1.mEdgeIdx / 3];
				uint32 vopposite0 = f0.mVertex[(e0.mEdgeIdx + 2) % 3];
				uint32 vopposite1 = f1.mVertex[(e1.mEdgeIdx + 2) % 3];
				const VertexAttributes &a_opposite0 = attr(vopposite0);
				const VertexAttributes &a_opposite1 = attr(vopposite1);

				// Faces should be roughly in a plane
				Vec3 n0 = (Vec3(mVertices[f0.mVertex[2]].mPosition) - Vec3(mVertices[f0.mVertex[0]].mPosition)).Cross(Vec3(mVertices[f0.mVertex[1]].mPosition) - Vec3(mVertices[f0.mVertex[0]].mPosition));
				Vec3 n1 = (Vec3(mVertices[f1.mVertex[2]].mPosition) - Vec3(mVertices[f1.mVertex[0]].mPosition)).Cross(Vec3(mVertices[f1.mVertex[1]].mPosition) - Vec3(mVertices[f1.mVertex[0]].mPosition));
				if (Square(n0.Dot(n1)) > sq_cos_tolerance * n0.LengthSq() * n1.LengthSq())
				{
					// Faces should approximately form a quad
					Vec3 e0_dir = Vec3(mVertices[vopposite0].mPosition) - Vec3(mVertices[e0.mVertex[0]].mPosition);
					Vec3 e1_dir = Vec3(mVertices[vopposite1].mPosition) - Vec3(mVertices[e0.mVertex[0]].mPosition);
					if (Square(e0_dir.Dot(e1_dir)) < sq_sin_tolerance * e0_dir.LengthSq() * e1_dir.LengthSq())
					{
						// Shear constraint
						add_edge(vopposite0, vopposite1, a_opposite0.mShearCompliance, a_opposite1.mShearCompliance);
						is_shear = true;
					}
				}

				// Bend constraint
				switch (inBendType)
				{
				case EBendType::None:
					// Do nothing
					break;

				case EBendType::Distance:
					// Create an edge constraint to represent the bend constraint
					// Use the bend compliance of the shared edge
					if (!is_shear)
						add_edge(vopposite0, vopposite1, a0.mBendCompliance, a1.mBendCompliance);
					break;

				case EBendType::Dihedral:
					// Test if both opposite vertices are free to move
					if ((mVertices[vopposite0].mInvMass > 0.0f || mVertices[vopposite1].mInvMass > 0.0f)
						&& a0.mBendCompliance < FLT_MAX && a1.mBendCompliance < FLT_MAX)
					{
						// Create a bend constraint
						// Use the bend compliance of the shared edge
						mDihedralBendConstraints.emplace_back(e0.mVertex[0], e0.mVertex[1], vopposite0, vopposite1, 0.5f * (a0.mBendCompliance + a1.mBendCompliance));
					}
					break;
				}
			}
			else
			{
				// Start iterating from the first non-shared edge
				i = j - 1;
				break;
			}
		}

		// Create a edge constraint for the current edge
		add_edge(e0.mVertex[0], e0.mVertex[1], is_shear? a0.mShearCompliance : a0.mCompliance, is_shear? a1.mShearCompliance : a1.mCompliance);
	}
	mEdgeConstraints.shrink_to_fit();

	// Calculate the initial angle for all bend constraints
	CalculateBendConstraintConstants();

	// Check if any vertices have LRA constraints
	bool has_lra_constraints = false;
	for (const VertexAttributes *va = inVertexAttributes; va < inVertexAttributes + inVertexAttributesLength; ++va)
		if (va->mLRAType != ELRAType::None)
		{
			has_lra_constraints = true;
			break;
		}
	if (has_lra_constraints)
	{
		// Ensure we have calculated the closest kinematic vertex for each vertex
		CalculateClosestKinematic();

		// Find non-kinematic vertices
		for (uint32 v = 0; v < (uint32)mVertices.size(); ++v)
			if (mVertices[v].mInvMass > 0.0f)
			{
				// Check if a closest vertex was found
				uint32 closest = mClosestKinematic[v].mVertex;
				if (closest != 0xffffffff)
				{
					// Check which LRA constraint to create
					const VertexAttributes &va = attr(v);
					switch (va.mLRAType)
					{
					case ELRAType::None:
						break;

					case ELRAType::EuclideanDistance:
						mLRAConstraints.emplace_back(closest, v, va.mLRAMaxDistanceMultiplier * (Vec3(mVertices[closest].mPosition) - Vec3(mVertices[v].mPosition)).Length());
						break;

					case ELRAType::GeodesicDistance:
						mLRAConstraints.emplace_back(closest, v, va.mLRAMaxDistanceMultiplier * mClosestKinematic[v].mDistance);
						break;
					}
				}
			}
	}
}

void SoftBodySharedSettings::CalculateEdgeLengths()
{
	for (Edge &e : mEdgeConstraints)
	{
		e.mRestLength = (Vec3(mVertices[e.mVertex[1]].mPosition) - Vec3(mVertices[e.mVertex[0]].mPosition)).Length();
		MOSS_ASSERT(e.mRestLength > 0.0f);
	}
}

void SoftBodySharedSettings::CalculateLRALengths(float inMaxDistanceMultiplier)
{
	for (LRA &l : mLRAConstraints)
	{
		l.mMaxDistance = inMaxDistanceMultiplier * (Vec3(mVertices[l.mVertex[1]].mPosition) - Vec3(mVertices[l.mVertex[0]].mPosition)).Length();
		MOSS_ASSERT(l.mMaxDistance > 0.0f);
	}
}

void SoftBodySharedSettings::CalculateBendConstraintConstants()
{
	for (DihedralBend &b : mDihedralBendConstraints)
	{
		// Get positions
		Vec3 x0 = Vec3(mVertices[b.mVertex[0]].mPosition);
		Vec3 x1 = Vec3(mVertices[b.mVertex[1]].mPosition);
		Vec3 x2 = Vec3(mVertices[b.mVertex[2]].mPosition);
		Vec3 x3 = Vec3(mVertices[b.mVertex[3]].mPosition);

		/*
		   x2
		e1/  \e3
		 /    \
		x0----x1
		 \ e0 /
		e2\  /e4
		   x3
		*/

		// Calculate edges
		Vec3 e0 = x1 - x0;
		Vec3 e1 = x2 - x0;
		Vec3 e2 = x3 - x0;

		// Normals of both triangles
		Vec3 n1 = e0.Cross(e1);
		Vec3 n2 = e2.Cross(e0);
		float denom = sqrt(n1.LengthSq() * n2.LengthSq());
		if (denom < 1.0e-12f)
			b.mInitialAngle = 0.0f;
		else
		{
			float sign = Sign(n2.Cross(n1).Dot(e0));
			b.mInitialAngle = sign * ACosApproximate(n1.Dot(n2) / denom); // Runtime uses the approximation too
		}
	}
}

void SoftBodySharedSettings::CalculateVolumeConstraintVolumes()
{
	for (Volume &v : mVolumeConstraints)
	{
		Vec3 x1(mVertices[v.mVertex[0]].mPosition);
		Vec3 x2(mVertices[v.mVertex[1]].mPosition);
		Vec3 x3(mVertices[v.mVertex[2]].mPosition);
		Vec3 x4(mVertices[v.mVertex[3]].mPosition);

		Vec3 x1x2 = x2 - x1;
		Vec3 x1x3 = x3 - x1;
		Vec3 x1x4 = x4 - x1;

		v.mSixRestVolume = abs(x1x2.Cross(x1x3).Dot(x1x4));
	}
}

void SoftBodySharedSettings::CalculateSkinnedConstraintNormals()
{
	// Clear any previous results
	mSkinnedConstraintNormals.clear();

	// If there are no skinned constraints, we're done
	if (mSkinnedConstraints.empty())
		return;

	// First collect all vertices that are skinned
	using VertexIndexSet = TSet<uint32>;
	VertexIndexSet skinned_vertices;
	skinned_vertices.reserve(VertexIndexSet::size_type(mSkinnedConstraints.size()));
	for (const Skinned &s : mSkinnedConstraints)
		skinned_vertices.insert(s.mVertex);

	// Now collect all faces that connect only to skinned vertices
	using ConnectedFacesMap = TMap<uint32, VertexIndexSet>;
	ConnectedFacesMap connected_faces;
	connected_faces.reserve(ConnectedFacesMap::size_type(mVertices.size()));
	for (const Face &f : mFaces)
	{
		// Must connect to only skinned vertices
		bool valid = true;
		for (uint32 v : f.mVertex)
			valid &= skinned_vertices.find(v) != skinned_vertices.end();
		if (!valid)
			continue;

		// Store faces that connect to vertices
		for (uint32 v : f.mVertex)
			connected_faces[v].insert(uint32(&f - mFaces.data()));
	}

	// Populate the list of connecting faces per skinned vertex
	mSkinnedConstraintNormals.reserve(mFaces.size());
	for (Skinned &s : mSkinnedConstraints)
	{
		uint32 start = uint32(mSkinnedConstraintNormals.size());
		MOSS_ASSERT((start >> 24) == 0);
		ConnectedFacesMap::const_iterator connected_faces_it = connected_faces.find(s.mVertex);
		if (connected_faces_it != connected_faces.cend())
		{
			const VertexIndexSet &faces = connected_faces_it->second;
			uint32 num = uint32(faces.size());
			MOSS_ASSERT(num < 256);
			mSkinnedConstraintNormals.insert(mSkinnedConstraintNormals.end(), faces.begin(), faces.end());
			QuickSort(mSkinnedConstraintNormals.begin() + start, mSkinnedConstraintNormals.begin() + start + num);
			s.mNormalInfo = start + (num << 24);
		}
		else
			s.mNormalInfo = 0;
	}
	mSkinnedConstraintNormals.shrink_to_fit();
}

void SoftBodySharedSettings::Optimize(OptimizationResults &outResults)
{
	// Clear any previous results
	mUpdateGroups.clear();

	// Create a list of connected vertices
	struct Connection
	{
		uint32	mVertex;
		uint32	mCount;
	};
	TArray<TArray<Connection>> connectivity;
	connectivity.resize(mVertices.size());
	auto add_connection = [&connectivity](uint inV1, uint inV2) {
			for (int i = 0; i < 2; ++i)
			{
				bool found = false;
				for (Connection &c : connectivity[inV1])
					if (c.mVertex == inV2)
					{
						c.mCount++;
						found = true;
						break;
					}
				if (!found)
					connectivity[inV1].push_back({ inV2, 1 });

				std::swap(inV1, inV2);
			}
		};
	for (const Edge &c : mEdgeConstraints)
		add_connection(c.mVertex[0], c.mVertex[1]);
	for (const LRA &c : mLRAConstraints)
		add_connection(c.mVertex[0], c.mVertex[1]);
	for (const DihedralBend &c : mDihedralBendConstraints)
	{
		add_connection(c.mVertex[0], c.mVertex[1]);
		add_connection(c.mVertex[0], c.mVertex[2]);
		add_connection(c.mVertex[0], c.mVertex[3]);
		add_connection(c.mVertex[1], c.mVertex[2]);
		add_connection(c.mVertex[1], c.mVertex[3]);
		add_connection(c.mVertex[2], c.mVertex[3]);
	}
	for (const Volume &c : mVolumeConstraints)
	{
		add_connection(c.mVertex[0], c.mVertex[1]);
		add_connection(c.mVertex[0], c.mVertex[2]);
		add_connection(c.mVertex[0], c.mVertex[3]);
		add_connection(c.mVertex[1], c.mVertex[2]);
		add_connection(c.mVertex[1], c.mVertex[3]);
		add_connection(c.mVertex[2], c.mVertex[3]);
	}
	// Skinned constraints only update 1 vertex, so we don't need special logic here

	// Maps each of the vertices to a group index
	TArray<int> group_idx;
	group_idx.resize(mVertices.size(), -1);

	// Which group we are currently filling and its vertices
	int current_group_idx = 0;
	TArray<uint> current_group;

	// Start greedy algorithm to group vertices
	for (;;)
	{
		// Find the bounding box of the ungrouped vertices
		AABox bounds;
		for (uint i = 0; i < (uint)mVertices.size(); ++i)
			if (group_idx[i] == -1)
				bounds.Encapsulate(Vec3(mVertices[i].mPosition));

		// If the bounds are invalid, it means that there were no ungrouped vertices
		if (!bounds.IsValid())
			break;

		// Determine longest and shortest axis
		Vec3 bounds_size = bounds.GetSize();
		uint max_axis = bounds_size.GetHighestComponentIndex();
		uint min_axis = bounds_size.GetLowestComponentIndex();
		if (min_axis == max_axis)
			min_axis = (min_axis + 1) % 3;
		uint mid_axis = 3 - min_axis - max_axis;

		// Find the vertex that has the lowest value on the axis with the largest extent
		uint current_vertex = UINT_MAX;
		Float3 current_vertex_position { FLT_MAX, FLT_MAX, FLT_MAX };
		for (uint i = 0; i < (uint)mVertices.size(); ++i)
			if (group_idx[i] == -1)
			{
				const Float3 &vertex_position = mVertices[i].mPosition;
				float max_axis_value = vertex_position[max_axis];
				float mid_axis_value = vertex_position[mid_axis];
				float min_axis_value = vertex_position[min_axis];

				if (max_axis_value < current_vertex_position[max_axis]
					|| (max_axis_value == current_vertex_position[max_axis]
						&& (mid_axis_value < current_vertex_position[mid_axis]
							|| (mid_axis_value == current_vertex_position[mid_axis]
								&& min_axis_value < current_vertex_position[min_axis]))))
				{
					current_vertex_position = mVertices[i].mPosition;
					current_vertex = i;
				}
			}
		if (current_vertex == UINT_MAX)
			break;

		// Initialize the current group with 1 vertex
		current_group.push_back(current_vertex);
		group_idx[current_vertex] = current_group_idx;

		// Fill up the group
		for (;;)
		{
			// Find the vertex that is most connected to the current group
			uint best_vertex = UINT_MAX;
			uint best_num_connections = 0;
			float best_dist_sq = FLT_MAX;
			for (uint i = 0; i < (uint)current_group.size(); ++i) // For all vertices in the current group
				for (const Connection &c : connectivity[current_group[i]]) // For all connections to other vertices
				{
					uint v = c.mVertex;
					if (group_idx[v] == -1) // Ungrouped vertices only
					{
						// Count the number of connections to this group
						uint num_connections = 0;
						for (const Connection &v2 : connectivity[v])
							if (group_idx[v2.mVertex] == current_group_idx)
								num_connections += v2.mCount;

						// Calculate distance to group centroid
						float dist_sq = (Vec3(mVertices[v].mPosition) - Vec3(mVertices[current_group.front()].mPosition)).LengthSq();

						if (best_vertex == UINT_MAX
							|| num_connections > best_num_connections
							|| (num_connections == best_num_connections && dist_sq < best_dist_sq))
						{
							best_vertex = v;
							best_num_connections = num_connections;
							best_dist_sq = dist_sq;
						}
					}
				}

			// Add the best vertex to the current group
			if (best_vertex != UINT_MAX)
			{
				current_group.push_back(best_vertex);
				group_idx[best_vertex] = current_group_idx;
			}

			// Create a new group?
			if (current_group.size() >= SoftBodyUpdateContext::cVertexConstraintBatch // If full, yes
				|| (current_group.size() > SoftBodyUpdateContext::cVertexConstraintBatch / 2 && best_vertex == UINT_MAX)) // If half full and we found no connected vertex, yes
			{
				current_group.clear();
				current_group_idx++;
				break;
			}

			// If we didn't find a connected vertex, we need to find a new starting vertex
			if (best_vertex == UINT_MAX)
				break;
		}
	}

	// If the last group is more than half full, we'll keep it as a separate group, otherwise we merge it with the 'non parallel' group
	if (current_group.size() > SoftBodyUpdateContext::cVertexConstraintBatch / 2)
		++current_group_idx;

	// We no longer need the current group array, free the memory
	current_group.clear();
	current_group.shrink_to_fit();

	// We're done with the connectivity list, free the memory
	connectivity.clear();
	connectivity.shrink_to_fit();

	// Assign the constraints to their groups
	struct Group
	{
		uint			GetSize() const
		{
			return (uint)mEdgeConstraints.size() + (uint)mLRAConstraints.size() + (uint)mDihedralBendConstraints.size() + (uint)mVolumeConstraints.size() + (uint)mSkinnedConstraints.size();
		}

		TArray<uint>		mEdgeConstraints;
		TArray<uint>		mLRAConstraints;
		TArray<uint>		mDihedralBendConstraints;
		TArray<uint>		mVolumeConstraints;
		TArray<uint>		mSkinnedConstraints;
	};
	TArray<Group> groups;
	groups.resize(current_group_idx + 1); // + non parallel group
	for (const Edge &e : mEdgeConstraints)
	{
		int g1 = group_idx[e.mVertex[0]];
		int g2 = group_idx[e.mVertex[1]];
		MOSS_ASSERT(g1 >= 0 && g2 >= 0);
		if (g1 == g2) // In the same group
			groups[g1].mEdgeConstraints.push_back(uint(&e - mEdgeConstraints.data()));
		else // In different groups -> parallel group
			groups.back().mEdgeConstraints.push_back(uint(&e - mEdgeConstraints.data()));
	}
	for (const LRA &l : mLRAConstraints)
	{
		int g1 = group_idx[l.mVertex[0]];
		int g2 = group_idx[l.mVertex[1]];
		MOSS_ASSERT(g1 >= 0 && g2 >= 0);
		if (g1 == g2) // In the same group
			groups[g1].mLRAConstraints.push_back(uint(&l - mLRAConstraints.data()));
		else // In different groups -> parallel group
			groups.back().mLRAConstraints.push_back(uint(&l - mLRAConstraints.data()));
	}
	for (const DihedralBend &d : mDihedralBendConstraints)
	{
		int g1 = group_idx[d.mVertex[0]];
		int g2 = group_idx[d.mVertex[1]];
		int g3 = group_idx[d.mVertex[2]];
		int g4 = group_idx[d.mVertex[3]];
		MOSS_ASSERT(g1 >= 0 && g2 >= 0 && g3 >= 0 && g4 >= 0);
		if (g1 == g2 && g1 == g3 && g1 == g4) // In the same group
			groups[g1].mDihedralBendConstraints.push_back(uint(&d - mDihedralBendConstraints.data()));
		else // In different groups -> parallel group
			groups.back().mDihedralBendConstraints.push_back(uint(&d - mDihedralBendConstraints.data()));
	}
	for (const Volume &v : mVolumeConstraints)
	{
		int g1 = group_idx[v.mVertex[0]];
		int g2 = group_idx[v.mVertex[1]];
		int g3 = group_idx[v.mVertex[2]];
		int g4 = group_idx[v.mVertex[3]];
		MOSS_ASSERT(g1 >= 0 && g2 >= 0 && g3 >= 0 && g4 >= 0);
		if (g1 == g2 && g1 == g3 && g1 == g4) // In the same group
			groups[g1].mVolumeConstraints.push_back(uint(&v - mVolumeConstraints.data()));
		else // In different groups -> parallel group
			groups.back().mVolumeConstraints.push_back(uint(&v - mVolumeConstraints.data()));
	}
	for (const Skinned &s : mSkinnedConstraints)
	{
		int g1 = group_idx[s.mVertex];
		MOSS_ASSERT(g1 >= 0);
		groups[g1].mSkinnedConstraints.push_back(uint(&s - mSkinnedConstraints.data()));
	}

	// Sort the parallel groups from big to small (this means the big groups will be scheduled first and have more time to complete)
	QuickSort(groups.begin(), groups.end() - 1, [](const Group &inLHS, const Group &inRHS) { return inLHS.GetSize() > inRHS.GetSize(); });

	// Make sure we know the closest kinematic vertex so we can sort
	CalculateClosestKinematic();

	// Sort within each group
	for (Group &group : groups)
	{
		// Sort the edge constraints
		QuickSort(group.mEdgeConstraints.begin(), group.mEdgeConstraints.end(), [this](uint inLHS, uint inRHS)
			{
				const Edge &e1 = mEdgeConstraints[inLHS];
				const Edge &e2 = mEdgeConstraints[inRHS];

				// First sort so that the edge with the smallest distance to a kinematic vertex comes first
				float d1 = min(mClosestKinematic[e1.mVertex[0]].mDistance, mClosestKinematic[e1.mVertex[1]].mDistance);
				float d2 = min(mClosestKinematic[e2.mVertex[0]].mDistance, mClosestKinematic[e2.mVertex[1]].mDistance);
				if (d1 != d2)
					return d1 < d2;

				// Order the edges so that the ones with the smallest index go first (hoping to get better cache locality when we process the edges).
				// Note we could also re-order the vertices but that would be much more of a burden to the end user
				uint32 m1 = e1.GetMinVertexIndex();
				uint32 m2 = e2.GetMinVertexIndex();
				if (m1 != m2)
					return m1 < m2;

				return inLHS < inRHS;
			});

		// Sort the LRA constraints
		QuickSort(group.mLRAConstraints.begin(), group.mLRAConstraints.end(), [this](uint inLHS, uint inRHS)
			{
				const LRA &l1 = mLRAConstraints[inLHS];
				const LRA &l2 = mLRAConstraints[inRHS];

				// First sort so that the longest constraint comes first (meaning the shortest constraint has the most influence on the end result)
				// Most of the time there will be a single LRA constraint per vertex and since the LRA constraint only modifies a single vertex,
				// updating one constraint will not violate another constraint.
				if (l1.mMaxDistance != l2.mMaxDistance)
					return l1.mMaxDistance > l2.mMaxDistance;

				// Order constraints so that the ones with the smallest index go first
				uint32 m1 = l1.GetMinVertexIndex();
				uint32 m2 = l2.GetMinVertexIndex();
				if (m1 != m2)
					return m1 < m2;

				return inLHS < inRHS;
			});

		// Sort the dihedral bend constraints
		QuickSort(group.mDihedralBendConstraints.begin(), group.mDihedralBendConstraints.end(), [this](uint inLHS, uint inRHS)
		{
			const DihedralBend &b1 = mDihedralBendConstraints[inLHS];
			const DihedralBend &b2 = mDihedralBendConstraints[inRHS];

			// First sort so that the constraint with the smallest distance to a kinematic vertex comes first
			float d1 = min(
						min(mClosestKinematic[b1.mVertex[0]].mDistance, mClosestKinematic[b1.mVertex[1]].mDistance),
						min(mClosestKinematic[b1.mVertex[2]].mDistance, mClosestKinematic[b1.mVertex[3]].mDistance));
			float d2 = min(
						min(mClosestKinematic[b2.mVertex[0]].mDistance, mClosestKinematic[b2.mVertex[1]].mDistance),
						min(mClosestKinematic[b2.mVertex[2]].mDistance, mClosestKinematic[b2.mVertex[3]].mDistance));
			if (d1 != d2)
				return d1 < d2;

			// Order constraints so that the ones with the smallest index go first
			uint32 m1 = b1.GetMinVertexIndex();
			uint32 m2 = b2.GetMinVertexIndex();
			if (m1 != m2)
				return m1 < m2;

			return inLHS < inRHS;
		});

		// Sort the volume constraints
		QuickSort(group.mVolumeConstraints.begin(), group.mVolumeConstraints.end(), [this](uint inLHS, uint inRHS)
		{
			const Volume &v1 = mVolumeConstraints[inLHS];
			const Volume &v2 = mVolumeConstraints[inRHS];

			// First sort so that the constraint with the smallest distance to a kinematic vertex comes first
			float d1 = min(
						min(mClosestKinematic[v1.mVertex[0]].mDistance, mClosestKinematic[v1.mVertex[1]].mDistance),
						min(mClosestKinematic[v1.mVertex[2]].mDistance, mClosestKinematic[v1.mVertex[3]].mDistance));
			float d2 = min(
						min(mClosestKinematic[v2.mVertex[0]].mDistance, mClosestKinematic[v2.mVertex[1]].mDistance),
						min(mClosestKinematic[v2.mVertex[2]].mDistance, mClosestKinematic[v2.mVertex[3]].mDistance));
			if (d1 != d2)
				return d1 < d2;

			// Order constraints so that the ones with the smallest index go first
			uint32 m1 = v1.GetMinVertexIndex();
			uint32 m2 = v2.GetMinVertexIndex();
			if (m1 != m2)
				return m1 < m2;

			return inLHS < inRHS;
		});

		// Sort the skinned constraints
		QuickSort(group.mSkinnedConstraints.begin(), group.mSkinnedConstraints.end(), [this](uint inLHS, uint inRHS)
			{
				const Skinned &s1 = mSkinnedConstraints[inLHS];
				const Skinned &s2 = mSkinnedConstraints[inRHS];

				// Order the skinned constraints so that the ones with the smallest index go first (hoping to get better cache locality when we process the edges).
				if (s1.mVertex != s2.mVertex)
					return s1.mVertex < s2.mVertex;

				return inLHS < inRHS;
			});
	}

	// Temporary store constraints as we reorder them
	TArray<Edge> temp_edges;
	temp_edges.swap(mEdgeConstraints);
	mEdgeConstraints.reserve(temp_edges.size());
	outResults.mEdgeRemap.reserve(temp_edges.size());

	TArray<LRA> temp_lra;
	temp_lra.swap(mLRAConstraints);
	mLRAConstraints.reserve(temp_lra.size());
	outResults.mLRARemap.reserve(temp_lra.size());

	TArray<DihedralBend> temp_dihedral_bend;
	temp_dihedral_bend.swap(mDihedralBendConstraints);
	mDihedralBendConstraints.reserve(temp_dihedral_bend.size());
	outResults.mDihedralBendRemap.reserve(temp_dihedral_bend.size());

	TArray<Volume> temp_volume;
	temp_volume.swap(mVolumeConstraints);
	mVolumeConstraints.reserve(temp_volume.size());
	outResults.mVolumeRemap.reserve(temp_volume.size());

	TArray<Skinned> temp_skinned;
	temp_skinned.swap(mSkinnedConstraints);
	mSkinnedConstraints.reserve(temp_skinned.size());
	outResults.mSkinnedRemap.reserve(temp_skinned.size());

	// Finalize update groups
	for (const Group &group : groups)
	{
		// Reorder edge constraints for this group
		for (uint idx : group.mEdgeConstraints)
		{
			mEdgeConstraints.push_back(temp_edges[idx]);
			outResults.mEdgeRemap.push_back(idx);
		}

		// Reorder LRA constraints for this group
		for (uint idx : group.mLRAConstraints)
		{
			mLRAConstraints.push_back(temp_lra[idx]);
			outResults.mLRARemap.push_back(idx);
		}

		// Reorder dihedral bend constraints for this group
		for (uint idx : group.mDihedralBendConstraints)
		{
			mDihedralBendConstraints.push_back(temp_dihedral_bend[idx]);
			outResults.mDihedralBendRemap.push_back(idx);
		}

		// Reorder volume constraints for this group
		for (uint idx : group.mVolumeConstraints)
		{
			mVolumeConstraints.push_back(temp_volume[idx]);
			outResults.mVolumeRemap.push_back(idx);
		}

		// Reorder skinned constraints for this group
		for (uint idx : group.mSkinnedConstraints)
		{
			mSkinnedConstraints.push_back(temp_skinned[idx]);
			outResults.mSkinnedRemap.push_back(idx);
		}

		// Store end indices
		mUpdateGroups.push_back({ (uint)mEdgeConstraints.size(), (uint)mLRAConstraints.size(), (uint)mDihedralBendConstraints.size(), (uint)mVolumeConstraints.size(), (uint)mSkinnedConstraints.size() });
	}

	// Free closest kinematic buffer
	mClosestKinematic.clear();
	mClosestKinematic.shrink_to_fit();
}

Ref<SoftBodySharedSettings> SoftBodySharedSettings::Clone() const
{
	Ref<SoftBodySharedSettings> clone = new SoftBodySharedSettings;
	clone->mVertices = mVertices;
	clone->mFaces = mFaces;
	clone->mEdgeConstraints = mEdgeConstraints;
	clone->mDihedralBendConstraints = mDihedralBendConstraints;
	clone->mVolumeConstraints = mVolumeConstraints;
	clone->mSkinnedConstraints = mSkinnedConstraints;
	clone->mSkinnedConstraintNormals = mSkinnedConstraintNormals;
	clone->mInvBindMatrices = mInvBindMatrices;
	clone->mLRAConstraints = mLRAConstraints;
	clone->mMaterials = mMaterials;
	clone->mVertexRadius = mVertexRadius;
	clone->mUpdateGroups = mUpdateGroups;
	return clone;
}

void SoftBodySharedSettings::SaveBinaryState(StreamOut &inStream) const
{
	inStream.Write(mVertices);
	inStream.Write(mFaces);
	inStream.Write(mEdgeConstraints);
	inStream.Write(mDihedralBendConstraints);
	inStream.Write(mVolumeConstraints);
	inStream.Write(mSkinnedConstraints);
	inStream.Write(mSkinnedConstraintNormals);
	inStream.Write(mLRAConstraints);
	inStream.Write(mVertexRadius);
	inStream.Write(mUpdateGroups);

	// Can't write mInvBindMatrices directly because the class contains padding
	inStream.Write(mInvBindMatrices, [](const InvBind &inElement, StreamOut &inS) {
		inS.Write(inElement.mJointIndex);
		inS.Write(inElement.mInvBind);
	});
}

void SoftBodySharedSettings::RestoreBinaryState(StreamIn &inStream)
{
	inStream.Read(mVertices);
	inStream.Read(mFaces);
	inStream.Read(mEdgeConstraints);
	inStream.Read(mDihedralBendConstraints);
	inStream.Read(mVolumeConstraints);
	inStream.Read(mSkinnedConstraints);
	inStream.Read(mSkinnedConstraintNormals);
	inStream.Read(mLRAConstraints);
	inStream.Read(mVertexRadius);
	inStream.Read(mUpdateGroups);

	inStream.Read(mInvBindMatrices, [](StreamIn &inS, InvBind &outElement) {
		inS.Read(outElement.mJointIndex);
		inS.Read(outElement.mInvBind);
	});
}

void SoftBodySharedSettings::SaveWithMaterials(StreamOut &inStream, SharedSettingsToIDMap &ioSettingsMap, MaterialToIDMap &ioMaterialMap) const
{
	SharedSettingsToIDMap::const_iterator settings_iter = ioSettingsMap.find(this);
	if (settings_iter == ioSettingsMap.end())
	{
		// Write settings ID
		uint32 settings_id = ioSettingsMap.size();
		ioSettingsMap[this] = settings_id;
		inStream.Write(settings_id);

		// Write the settings
		SaveBinaryState(inStream);

		// Write materials
		StreamUtils::SaveObjectArray(inStream, mMaterials, &ioMaterialMap);
	}
	else
	{
		// Known settings, just write the ID
		inStream.Write(settings_iter->second);
	}
}

SoftBodySharedSettings::SettingsResult SoftBodySharedSettings::sRestoreWithMaterials(StreamIn &inStream, IDToSharedSettingsMap &ioSettingsMap, IDToMaterialMap &ioMaterialMap)
{
	SettingsResult result;

	// Read settings id
	uint32 settings_id;
	inStream.Read(settings_id);
	if (inStream.IsEOF() || inStream.IsFailed())
	{
		result.SetError("Failed to read settings id");
		return result;
	}

	// Check nullptr settings
	if (settings_id == ~uint32(0))
	{
		result.Set(nullptr);
		return result;
	}

	// Check if we already read this settings
	if (settings_id < ioSettingsMap.size())
	{
		result.Set(ioSettingsMap[settings_id]);
		return result;
	}

	// Create new object
	Ref<SoftBodySharedSettings> settings = new SoftBodySharedSettings;

	// Read state
	settings->RestoreBinaryState(inStream);

	// Read materials
	Result mlresult = StreamUtils::RestoreObjectArray<PhysicsMaterialList>(inStream, ioMaterialMap);
	if (mlresult.HasError())
	{
		result.SetError(mlresult.GetError());
		return result;
	}
	settings->mMaterials = mlresult.Get();

	// Add the settings to the map
	ioSettingsMap.push_back(settings);

	result.Set(settings);
	return result;
}

Ref<SoftBodySharedSettings> SoftBodySharedSettings::sCreateCube(uint inGridSize, float inGridSpacing)
{
	const Vec3 cOffset = Vec3::sReplicate(-0.5f * inGridSpacing * (inGridSize - 1));

	// Create settings
	SoftBodySharedSettings *settings = new SoftBodySharedSettings;
	for (uint z = 0; z < inGridSize; ++z)
		for (uint y = 0; y < inGridSize; ++y)
			for (uint x = 0; x < inGridSize; ++x)
			{
				SoftBodySharedSettings::Vertex v;
				(cOffset + Vec3::sReplicate(inGridSpacing) * Vec3(float(x), float(y), float(z))).StoreFloat3(&v.mPosition);
				settings->mVertices.push_back(v);
			}

	// Function to get the vertex index of a point on the cube
	auto vertex_index = [inGridSize](uint inX, uint inY, uint inZ)
	{
		return inX + inY * inGridSize + inZ * inGridSize * inGridSize;
	};

	// Create edges
	for (uint z = 0; z < inGridSize; ++z)
		for (uint y = 0; y < inGridSize; ++y)
			for (uint x = 0; x < inGridSize; ++x)
			{
				SoftBodySharedSettings::Edge e;
				e.mVertex[0] = vertex_index(x, y, z);
				if (x < inGridSize - 1)
				{
					e.mVertex[1] = vertex_index(x + 1, y, z);
					settings->mEdgeConstraints.push_back(e);
				}
				if (y < inGridSize - 1)
				{
					e.mVertex[1] = vertex_index(x, y + 1, z);
					settings->mEdgeConstraints.push_back(e);
				}
				if (z < inGridSize - 1)
				{
					e.mVertex[1] = vertex_index(x, y, z + 1);
					settings->mEdgeConstraints.push_back(e);
				}
			}
	settings->CalculateEdgeLengths();

	// Tetrahedrons to fill a cube
	const int tetra_indices[6][4][3] = {
		{ {0, 0, 0}, {0, 1, 1}, {0, 0, 1}, {1, 1, 1} },
		{ {0, 0, 0}, {0, 1, 0}, {0, 1, 1}, {1, 1, 1} },
		{ {0, 0, 0}, {0, 0, 1}, {1, 0, 1}, {1, 1, 1} },
		{ {0, 0, 0}, {1, 0, 1}, {1, 0, 0}, {1, 1, 1} },
		{ {0, 0, 0}, {1, 1, 0}, {0, 1, 0}, {1, 1, 1} },
		{ {0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {1, 1, 1} }
	};

	// Create volume constraints
	for (uint z = 0; z < inGridSize - 1; ++z)
		for (uint y = 0; y < inGridSize - 1; ++y)
			for (uint x = 0; x < inGridSize - 1; ++x)
				for (uint t = 0; t < 6; ++t)
				{
					SoftBodySharedSettings::Volume v;
					for (uint i = 0; i < 4; ++i)
						v.mVertex[i] = vertex_index(x + tetra_indices[t][i][0], y + tetra_indices[t][i][1], z + tetra_indices[t][i][2]);
					settings->mVolumeConstraints.push_back(v);
				}

	settings->CalculateVolumeConstraintVolumes();

	// Create faces
	for (uint y = 0; y < inGridSize - 1; ++y)
		for (uint x = 0; x < inGridSize - 1; ++x)
		{
			SoftBodySharedSettings::Face f;

			// Face 1
			f.mVertex[0] = vertex_index(x, y, 0);
			f.mVertex[1] = vertex_index(x, y + 1, 0);
			f.mVertex[2] = vertex_index(x + 1, y + 1, 0);
			settings->AddFace(f);

			f.mVertex[1] = vertex_index(x + 1, y + 1, 0);
			f.mVertex[2] = vertex_index(x + 1, y, 0);
			settings->AddFace(f);

			// Face 2
			f.mVertex[0] = vertex_index(x, y, inGridSize - 1);
			f.mVertex[1] = vertex_index(x + 1, y + 1, inGridSize - 1);
			f.mVertex[2] = vertex_index(x, y + 1, inGridSize - 1);
			settings->AddFace(f);

			f.mVertex[1] = vertex_index(x + 1, y, inGridSize - 1);
			f.mVertex[2] = vertex_index(x + 1, y + 1, inGridSize - 1);
			settings->AddFace(f);

			// Face 3
			f.mVertex[0] = vertex_index(x, 0, y);
			f.mVertex[1] = vertex_index(x + 1, 0, y + 1);
			f.mVertex[2] = vertex_index(x, 0, y + 1);
			settings->AddFace(f);

			f.mVertex[1] = vertex_index(x + 1, 0, y);
			f.mVertex[2] = vertex_index(x + 1, 0, y + 1);
			settings->AddFace(f);

			// Face 4
			f.mVertex[0] = vertex_index(x, inGridSize - 1, y);
			f.mVertex[1] = vertex_index(x, inGridSize - 1, y + 1);
			f.mVertex[2] = vertex_index(x + 1, inGridSize - 1, y + 1);
			settings->AddFace(f);

			f.mVertex[1] = vertex_index(x + 1, inGridSize - 1, y + 1);
			f.mVertex[2] = vertex_index(x + 1, inGridSize - 1, y);
			settings->AddFace(f);

			// Face 5
			f.mVertex[0] = vertex_index(0, x, y);
			f.mVertex[1] = vertex_index(0, x, y + 1);
			f.mVertex[2] = vertex_index(0, x + 1, y + 1);
			settings->AddFace(f);

			f.mVertex[1] = vertex_index(0, x + 1, y + 1);
			f.mVertex[2] = vertex_index(0, x + 1, y);
			settings->AddFace(f);

			// Face 6
			f.mVertex[0] = vertex_index(inGridSize - 1, x, y);
			f.mVertex[1] = vertex_index(inGridSize - 1, x + 1, y + 1);
			f.mVertex[2] = vertex_index(inGridSize - 1, x, y + 1);
			settings->AddFace(f);

			f.mVertex[1] = vertex_index(inGridSize - 1, x + 1, y);
			f.mVertex[2] = vertex_index(inGridSize - 1, x + 1, y + 1);
			settings->AddFace(f);
		}

	// Optimize the settings
	settings->Optimize();

	return settings;
}

void SoftBodyMotionProperties::CalculateMassAndInertia()
{
	MassProperties mp;

	for (const Vertex &v : mVertices)
		if (v.mInvMass > 0.0f)
		{
			Vec3 pos = v.mPosition;

			// Accumulate mass
			float mass = 1.0f / v.mInvMass;
			mp.mMass += mass;

			// Inertia tensor, diagonal
			// See equations https://en.wikipedia.org/wiki/Moment_of_inertia section 'Inertia Tensor'
			for (int i = 0; i < 3; ++i)
				mp.mInertia(i, i) += mass * (Square(pos[(i + 1) % 3]) + Square(pos[(i + 2) % 3]));

			// Inertia tensor off diagonal
			for (int i = 0; i < 3; ++i)
				for (int j = 0; j < 3; ++j)
					if (i != j)
						mp.mInertia(i, j) -= mass * pos[i] * pos[j];
		}
		else
		{
			// If one vertex is kinematic, the entire body will have infinite mass and inertia
			SetInverseMass(0.0f);
			SetInverseInertia(Vec3::sZero(), Quat::sIdentity());
			return;
		}

	SetMassProperties(EAllowedDOFs::All, mp);
}

void SoftBodyMotionProperties::Initialize(const SoftBodyCreationSettings &inSettings)
{
	// Store settings
	mSettings = inSettings.mSettings;
	mNumIterations = inSettings.mNumIterations;
	mPressure = inSettings.mPressure;
	mUpdatePosition = inSettings.mUpdatePosition;

	// Initialize vertices
	mVertices.resize(inSettings.mSettings->mVertices.size());
	Mat44 rotation = inSettings.mMakeRotationIdentity? Mat44::sRotation(inSettings.mRotation) : Mat44::sIdentity();
	for (TArray<Vertex>::size_type v = 0, s = mVertices.size(); v < s; ++v)
	{
		const SoftBodySharedSettings::Vertex &in_vertex = inSettings.mSettings->mVertices[v];
		Vertex &out_vertex = mVertices[v];
		out_vertex.mPreviousPosition = out_vertex.mPosition = rotation * Vec3(in_vertex.mPosition);
		out_vertex.mVelocity = rotation.Multiply3x3(Vec3(in_vertex.mVelocity));
		out_vertex.ResetCollision();
		out_vertex.mInvMass = in_vertex.mInvMass;
		mLocalBounds.Encapsulate(out_vertex.mPosition);
	}

	// Allocate space for skinned vertices
	if (!inSettings.mSettings->mSkinnedConstraints.empty())
		mSkinState.resize(mVertices.size());

	// We don't know delta time yet, so we can't predict the bounds and use the local bounds as the predicted bounds
	mLocalPredictedBounds = mLocalBounds;

	CalculateMassAndInertia();
}

float SoftBodyMotionProperties::GetVolumeTimesSix() const
{
	float six_volume = 0.0f;
	for (const Face &f : mSettings->mFaces)
	{
		Vec3 x1 = mVertices[f.mVertex[0]].mPosition;
		Vec3 x2 = mVertices[f.mVertex[1]].mPosition;
		Vec3 x3 = mVertices[f.mVertex[2]].mPosition;
		six_volume += x1.Cross(x2).Dot(x3); // We pick zero as the origin as this is the center of the bounding box so should give good accuracy
	}
	return six_volume;
}

void SoftBodyMotionProperties::DetermineCollidingShapes(const SoftBodyUpdateContext &inContext, const PhysicsSystem &inSystem, const BodyLockInterface &inBodyLockInterface)
{
	MOSS_PROFILE_FUNCTION();

	// Reset flag prior to collision detection
	mNeedContactCallback.store(false, memory_order_relaxed);

	struct Collector : public CollideShapeBodyCollector
	{
									Collector(const SoftBodyUpdateContext &inContext, const PhysicsSystem &inSystem, const BodyLockInterface &inBodyLockInterface, const AABox &inLocalBounds, SimShapeFilterWrapper &inShapeFilter, TArray<CollidingShape> &ioHits, TArray<CollidingSensor> &ioSensors) :
										mContext(inContext),
										mInverseTransform(inContext.mCenterOfMassTransform.InversedRotationTranslation()),
										mLocalBounds(inLocalBounds),
										mBodyLockInterface(inBodyLockInterface),
										mCombineFriction(inSystem.GetCombineFriction()),
										mCombineRestitution(inSystem.GetCombineRestitution()),
										mShapeFilter(inShapeFilter),
										mHits(ioHits),
										mSensors(ioSensors)
		{
		}

		virtual void				AddHit(const BodyID &inResult) override
		{
			BodyLockRead lock(mBodyLockInterface, inResult);
			if (lock.Succeeded())
			{
				const Body &soft_body = *mContext.mBody;
				const Body &body = lock.GetBody();
				if (body.IsRigidBody() // TODO: We should support soft body vs soft body
					&& soft_body.GetCollisionGroup().CanCollide(body.GetCollisionGroup()))
				{
					SoftBodyContactSettings settings;
					settings.mIsSensor = body.IsSensor();

					if (mContext.mContactListener == nullptr)
					{
						// If we have no contact listener, we can ignore sensors
						if (settings.mIsSensor)
							return;
					}
					else
					{
						// Call the contact listener to see if we should accept this contact
						if (mContext.mContactListener->OnSoftBodyContactValidate(soft_body, body, settings) != SoftBodyValidateResult::AcceptContact)
							return;

						// Check if there will be any interaction
						if (!settings.mIsSensor
							&& settings.mInvMassScale1 == 0.0f
							&& (body.GetMotionType() != EMotionType::Dynamic || settings.mInvMassScale2 == 0.0f))
							return;
					}

					// Calculate transform of this body relative to the soft body
					Mat44 com = (mInverseTransform * body.GetCenterOfMassTransform()).ToMat44();

					// Collect leaf shapes
					mShapeFilter.SetBody2(&body);
					struct LeafShapeCollector : public TransformedShapeCollector
					{
						virtual void		AddHit(const TransformedShape &inResult) override
						{
							mHits.emplace_back(Mat44::sRotationTranslation(inResult.mShapeRotation, Vec3(inResult.mShapePositionCOM)), inResult.GetShapeScale(), inResult.mShape);
						}

						TArray<LeafShape>	mHits;
					};
					LeafShapeCollector collector;
					body.GetShape()->CollectTransformedShapes(mLocalBounds, com.GetTranslation(), com.GetQuaternion(), Vec3::sOne(), SubShapeIDCreator(), collector, mShapeFilter);
					if (collector.mHits.empty())
						return;

					if (settings.mIsSensor)
					{
						CollidingSensor cs;
						cs.mCenterOfMassTransform = com;
						cs.mShapes = std::move(collector.mHits);
						cs.mBodyID = inResult;
						mSensors.push_back(cs);
					}
					else
					{
						CollidingShape cs;
						cs.mCenterOfMassTransform = com;
						cs.mShapes = std::move(collector.mHits);
						cs.mBodyID = inResult;
						cs.mMotionType = body.GetMotionType();
						cs.mUpdateVelocities = false;
						cs.mFriction = mCombineFriction(soft_body, SubShapeID(), body, SubShapeID());
						cs.mRestitution = mCombineRestitution(soft_body, SubShapeID(), body, SubShapeID());
						cs.mSoftBodyInvMassScale = settings.mInvMassScale1;
						if (cs.mMotionType == EMotionType::Dynamic)
						{
							const MotionProperties *mp = body.GetMotionProperties();
							cs.mInvMass = settings.mInvMassScale2 * mp->GetInverseMass();
							cs.mInvInertia = settings.mInvInertiaScale2 * mp->GetInverseInertiaForRotation(cs.mCenterOfMassTransform.GetRotation());
							cs.mOriginalLinearVelocity = cs.mLinearVelocity = mInverseTransform.Multiply3x3(mp->GetLinearVelocity());
							cs.mOriginalAngularVelocity = cs.mAngularVelocity = mInverseTransform.Multiply3x3(mp->GetAngularVelocity());
						}
						mHits.push_back(cs);
					}
				}
			}
		}

	private:
		const SoftBodyUpdateContext &mContext;
		RMat44						mInverseTransform;
		AABox						mLocalBounds;
		const BodyLockInterface &	mBodyLockInterface;
		ContactConstraintManager::CombineFunction mCombineFriction;
		ContactConstraintManager::CombineFunction mCombineRestitution;
		SimShapeFilterWrapper &		mShapeFilter;
		TArray<CollidingShape> &		mHits;
		TArray<CollidingSensor> &	mSensors;
	};

	// Calculate local bounding box
	AABox local_bounds = mLocalBounds;
	local_bounds.Encapsulate(mLocalPredictedBounds);
	local_bounds.ExpandBy(Vec3::sReplicate(mSettings->mVertexRadius));

	// Calculate world space bounding box
	AABox world_bounds = local_bounds.Transformed(inContext.mCenterOfMassTransform);

	// Create shape filter
	SimShapeFilterWrapperUnion shape_filter_union(inContext.mSimShapeFilter, inContext.mBody);
	SimShapeFilterWrapper &shape_filter = shape_filter_union.GetSimShapeFilterWrapper();

	Collector collector(inContext, inSystem, inBodyLockInterface, local_bounds, shape_filter, mCollidingShapes, mCollidingSensors);
	ObjectLayer layer = inContext.mBody->GetObjectLayer();
	DefaultBroadPhaseLayerFilter broadphase_layer_filter = inSystem.GetDefaultBroadPhaseLayerFilter(layer);
	DefaultObjectLayerFilter object_layer_filter = inSystem.GetDefaultLayerFilter(layer);
	inSystem.GetBroadPhaseQuery().CollideAABox(world_bounds, collector, broadphase_layer_filter, object_layer_filter);
	mNumSensors = uint(mCollidingSensors.size()); // Workaround for TSAN false positive: store mCollidingSensors.size() in a separate variable.
}

void SoftBodyMotionProperties::DetermineCollisionPlanes(uint inVertexStart, uint inNumVertices)
{
	MOSS_PROFILE_FUNCTION();

	// Generate collision planes
	for (const CollidingShape &cs : mCollidingShapes)
		for (const LeafShape &shape : cs.mShapes)
			shape.mShape->CollideSoftBodyVertices(shape.mTransform, shape.mScale, CollideSoftBodyVertexIterator(mVertices.data() + inVertexStart), inNumVertices, int(&cs - mCollidingShapes.data()));
}

void SoftBodyMotionProperties::DetermineSensorCollisions(CollidingSensor &ioSensor)
{
	MOSS_PROFILE_FUNCTION();

	Plane collision_plane;
	float largest_penetration = -FLT_MAX;
	int colliding_shape_idx = -1;

	// Collide sensor against all vertices
	CollideSoftBodyVertexIterator vertex_iterator(
		StridedPtr<const Vec3>(&mVertices[0].mPosition, sizeof(SoftBodyVertex)), // The position and mass come from the soft body vertex
		StridedPtr<const float>(&mVertices[0].mInvMass, sizeof(SoftBodyVertex)),
		StridedPtr<Plane>(&collision_plane, 0), // We want all vertices to result in a single collision so we pass stride 0
		StridedPtr<float>(&largest_penetration, 0),
		StridedPtr<int>(&colliding_shape_idx, 0));
	for (const LeafShape &shape : ioSensor.mShapes)
		shape.mShape->CollideSoftBodyVertices(shape.mTransform, shape.mScale, vertex_iterator, uint(mVertices.size()), 0);
	ioSensor.mHasContact = largest_penetration > 0.0f;

	// We need a contact callback if one of the sensors collided
	if (ioSensor.mHasContact)
		mNeedContactCallback.store(true, memory_order_relaxed);
}

void SoftBodyMotionProperties::ApplyPressure(const SoftBodyUpdateContext &inContext)
{
	MOSS_PROFILE_FUNCTION();

	float dt = inContext.mSubStepDeltaTime;
	float pressure_coefficient = mPressure;
	if (pressure_coefficient > 0.0f)
	{
		// Calculate total volume
		float six_volume = GetVolumeTimesSix();
		if (six_volume > 0.0f)
		{
			// Apply pressure
			// p = F / A = n R T / V (see https://en.wikipedia.org/wiki/Pressure)
			// Our pressure coefficient is n R T so the impulse is:
			// P = F dt = pressure_coefficient / V * A * dt
			float coefficient = pressure_coefficient * dt / six_volume; // Need to still multiply by 6 for the volume
			for (const Face &f : mSettings->mFaces)
			{
				Vec3 x1 = mVertices[f.mVertex[0]].mPosition;
				Vec3 x2 = mVertices[f.mVertex[1]].mPosition;
				Vec3 x3 = mVertices[f.mVertex[2]].mPosition;

				Vec3 impulse = coefficient * (x2 - x1).Cross(x3 - x1); // Area is half the cross product so need to still divide by 2
				for (uint32 i : f.mVertex)
				{
					Vertex &v = mVertices[i];
					v.mVelocity += v.mInvMass * impulse; // Want to divide by 3 because we spread over 3 vertices
				}
			}
		}
	}
}

void SoftBodyMotionProperties::IntegratePositions(const SoftBodyUpdateContext &inContext)
{
	MOSS_PROFILE_FUNCTION();

	float dt = inContext.mSubStepDeltaTime;
	float linear_damping = max(0.0f, 1.0f - GetLinearDamping() * dt); // See: MotionProperties::ApplyForceTorqueAndDragInternal

	// Integrate
	Vec3 sub_step_gravity = inContext.mGravity * dt;
	Vec3 sub_step_impulse = GetAccumulatedForce() * dt / max(float(mVertices.size()), 1.0f);
	for (Vertex &v : mVertices)
		if (v.mInvMass > 0.0f)
		{
			// Gravity
			v.mVelocity += sub_step_gravity + sub_step_impulse * v.mInvMass;

			// Damping
			v.mVelocity *= linear_damping;

			// Integrate
			v.mPreviousPosition = v.mPosition;
			v.mPosition += v.mVelocity * dt;
		}
		else
		{
			// Integrate
			v.mPreviousPosition = v.mPosition;
			v.mPosition += v.mVelocity * dt;
		}
}

void SoftBodyMotionProperties::ApplyDihedralBendConstraints(const SoftBodyUpdateContext &inContext, uint inStartIndex, uint inEndIndex)
{
	MOSS_PROFILE_FUNCTION();

	float inv_dt_sq = 1.0f / Square(inContext.mSubStepDeltaTime);

	for (const DihedralBend *b = mSettings->mDihedralBendConstraints.data() + inStartIndex, *b_end = mSettings->mDihedralBendConstraints.data() + inEndIndex; b < b_end; ++b)
	{
		Vertex &v0 = mVertices[b->mVertex[0]];
		Vertex &v1 = mVertices[b->mVertex[1]];
		Vertex &v2 = mVertices[b->mVertex[2]];
		Vertex &v3 = mVertices[b->mVertex[3]];

		// Get positions
		Vec3 x0 = v0.mPosition;
		Vec3 x1 = v1.mPosition;
		Vec3 x2 = v2.mPosition;
		Vec3 x3 = v3.mPosition;

		/*
		   x2
		e1/  \e3
		 /    \
		x0----x1
		 \ e0 /
		e2\  /e4
		   x3
		*/

		// Calculate the shared edge of the triangles
		Vec3 e = x1 - x0;
		float e_len = e.Length();
		if (e_len < 1.0e-6f)
			continue;

		// Calculate the normals of the triangles
		Vec3 x1x2 = x2 - x1;
		Vec3 x1x3 = x3 - x1;
		Vec3 n1 = (x2 - x0).Cross(x1x2);
		Vec3 n2 = x1x3.Cross(x3 - x0);
		float n1_len_sq = n1.LengthSq();
		float n2_len_sq = n2.LengthSq();
		float n1_len_sq_n2_len_sq = n1_len_sq * n2_len_sq;
		if (n1_len_sq_n2_len_sq < 1.0e-24f)
			continue;

		// Calculate constraint equation
		// As per "Strain Based Dynamics" Appendix A we need to negate the gradients when (n1 x n2) . e > 0, instead we make sure that the sign of the constraint equation is correct
		float sign = Sign(n2.Cross(n1).Dot(e));
		float d = n1.Dot(n2) / sqrt(n1_len_sq_n2_len_sq);
		float c = sign * ACosApproximate(d) - b->mInitialAngle;

		// Ensure the range is -PI to PI
		if (c > MOSS_PI)
			c -= 2.0f * MOSS_PI;
		else if (c < -MOSS_PI)
			c += 2.0f * MOSS_PI;

		// Calculate gradient of constraint equation
		// Taken from "Strain Based Dynamics" - Matthias Muller et al. (Appendix A)
		// with p1 = x2, p2 = x3, p3 = x0 and p4 = x1
		// which in turn is based on "Simulation of Clothing with Folds and Wrinkles" - R. Bridson et al. (Section 4)
		n1 /= n1_len_sq;
		n2 /= n2_len_sq;
		Vec3 d0c = (x1x2.Dot(e) * n1 + x1x3.Dot(e) * n2) / e_len;
		Vec3 d2c = e_len * n1;
		Vec3 d3c = e_len * n2;

		// The sum of the gradients must be zero (see "Strain Based Dynamics" section 4)
		Vec3 d1c = -d0c - d2c - d3c;

		// Get masses
		float w0 = v0.mInvMass;
		float w1 = v1.mInvMass;
		float w2 = v2.mInvMass;
		float w3 = v3.mInvMass;

		// Calculate -lambda
		float denom = w0 * d0c.LengthSq() + w1 * d1c.LengthSq() + w2 * d2c.LengthSq() + w3 * d3c.LengthSq() + b->mCompliance * inv_dt_sq;
		if (denom < 1.0e-12f)
			continue;
		float minus_lambda = c / denom;

		// Apply correction
		v0.mPosition = x0 - minus_lambda * w0 * d0c;
		v1.mPosition = x1 - minus_lambda * w1 * d1c;
		v2.mPosition = x2 - minus_lambda * w2 * d2c;
		v3.mPosition = x3 - minus_lambda * w3 * d3c;
	}
}

void SoftBodyMotionProperties::ApplyVolumeConstraints(const SoftBodyUpdateContext &inContext, uint inStartIndex, uint inEndIndex)
{
	MOSS_PROFILE_FUNCTION();

	float inv_dt_sq = 1.0f / Square(inContext.mSubStepDeltaTime);

	// Satisfy volume constraints
	for (const Volume *v = mSettings->mVolumeConstraints.data() + inStartIndex, *v_end = mSettings->mVolumeConstraints.data() + inEndIndex; v < v_end; ++v)
	{
		Vertex &v1 = mVertices[v->mVertex[0]];
		Vertex &v2 = mVertices[v->mVertex[1]];
		Vertex &v3 = mVertices[v->mVertex[2]];
		Vertex &v4 = mVertices[v->mVertex[3]];

		Vec3 x1 = v1.mPosition;
		Vec3 x2 = v2.mPosition;
		Vec3 x3 = v3.mPosition;
		Vec3 x4 = v4.mPosition;

		// Calculate constraint equation
		Vec3 x1x2 = x2 - x1;
		Vec3 x1x3 = x3 - x1;
		Vec3 x1x4 = x4 - x1;
		float c = abs(x1x2.Cross(x1x3).Dot(x1x4)) - v->mSixRestVolume;

		// Calculate gradient of constraint equation
		Vec3 d1c = (x4 - x2).Cross(x3 - x2);
		Vec3 d2c = x1x3.Cross(x1x4);
		Vec3 d3c = x1x4.Cross(x1x2);
		Vec3 d4c = x1x2.Cross(x1x3);

		// Get masses
		float w1 = v1.mInvMass;
		float w2 = v2.mInvMass;
		float w3 = v3.mInvMass;
		float w4 = v4.mInvMass;

		// Calculate -lambda
		float denom = w1 * d1c.LengthSq() + w2 * d2c.LengthSq() + w3 * d3c.LengthSq() + w4 * d4c.LengthSq() + v->mCompliance * inv_dt_sq;
		if (denom < 1.0e-12f)
			continue;
		float minus_lambda = c / denom;

		// Apply correction
		v1.mPosition = x1 - minus_lambda * w1 * d1c;
		v2.mPosition = x2 - minus_lambda * w2 * d2c;
		v3.mPosition = x3 - minus_lambda * w3 * d3c;
		v4.mPosition = x4 - minus_lambda * w4 * d4c;
	}
}

void SoftBodyMotionProperties::ApplySkinConstraints(const SoftBodyUpdateContext &inContext, uint inStartIndex, uint inEndIndex)
{
	// Early out if nothing to do
	if (mSettings->mSkinnedConstraints.empty() || !mEnableSkinConstraints)
		return;

	MOSS_PROFILE_FUNCTION();

	// We're going to iterate multiple times over the skin constraints, update the skinned position accordingly.
	// If we don't do this, the simulation will see a big jump and the first iteration will cause a big velocity change in the system.
	float factor = mSkinStatePreviousPositionValid? inContext.mNextIteration.load(std::memory_order_relaxed) / float(mNumIterations) : 1.0f;
	float prev_factor = 1.0f - factor;

	// Apply the constraints
	Vertex *vertices = mVertices.data();
	const SkinState *skin_states = mSkinState.data();
	for (const Skinned *s = mSettings->mSkinnedConstraints.data() + inStartIndex, *s_end = mSettings->mSkinnedConstraints.data() + inEndIndex; s < s_end; ++s)
	{
		Vertex &vertex = vertices[s->mVertex];
		const SkinState &skin_state = skin_states[s->mVertex];
		float max_distance = s->mMaxDistance * mSkinnedMaxDistanceMultiplier;

		// Calculate the skinned position by interpolating from previous to current position
		Vec3 skin_pos = prev_factor * skin_state.mPreviousPosition + factor * skin_state.mPosition;

		if (max_distance > 0.0f)
		{
			// Move vertex if it violated the back stop
			if (s->mBackStopDistance < max_distance)
			{
				// Center of the back stop sphere
				Vec3 center = skin_pos - skin_state.mNormal * (s->mBackStopDistance + s->mBackStopRadius);

				// Check if we're inside the back stop sphere
				Vec3 delta = vertex.mPosition - center;
				float delta_len_sq = delta.LengthSq();
				if (delta_len_sq < Square(s->mBackStopRadius))
				{
					// Push the vertex to the surface of the back stop sphere
					float delta_len = sqrt(delta_len_sq);
					vertex.mPosition = delta_len > 0.0f?
						center + delta * (s->mBackStopRadius / delta_len)
						: center + skin_state.mNormal * s->mBackStopRadius;
				}
			}

			// Clamp vertex distance to max distance from skinned position
			if (max_distance < FLT_MAX)
			{
				Vec3 delta = vertex.mPosition - skin_pos;
				float delta_len_sq = delta.LengthSq();
				float max_distance_sq = Square(max_distance);
				if (delta_len_sq > max_distance_sq)
					vertex.mPosition = skin_pos + delta * sqrt(max_distance_sq / delta_len_sq);
			}
		}
		else
		{
			// Kinematic: Just update the vertex position
			vertex.mPosition = skin_pos;
		}
	}
}

void SoftBodyMotionProperties::ApplyEdgeConstraints(const SoftBodyUpdateContext &inContext, uint inStartIndex, uint inEndIndex)
{
	MOSS_PROFILE_FUNCTION();

	float inv_dt_sq = 1.0f / Square(inContext.mSubStepDeltaTime);

	// Satisfy edge constraints
	for (const Edge *e = mSettings->mEdgeConstraints.data() + inStartIndex, *e_end = mSettings->mEdgeConstraints.data() + inEndIndex; e < e_end; ++e)
	{
		Vertex &v0 = mVertices[e->mVertex[0]];
		Vertex &v1 = mVertices[e->mVertex[1]];

		// Get positions
		Vec3 x0 = v0.mPosition;
		Vec3 x1 = v1.mPosition;

		// Calculate current length
		Vec3 delta = x1 - x0;
		float length = delta.Length();

		// Apply correction
		float denom = length * (v0.mInvMass + v1.mInvMass + e->mCompliance * inv_dt_sq);
		if (denom < 1.0e-12f)
			continue;
		Vec3 correction = delta * (length - e->mRestLength) / denom;
		v0.mPosition = x0 + v0.mInvMass * correction;
		v1.mPosition = x1 - v1.mInvMass * correction;
	}
}

void SoftBodyMotionProperties::ApplyLRAConstraints(uint inStartIndex, uint inEndIndex)
{
	MOSS_PROFILE_FUNCTION();

	// Satisfy LRA constraints
	Vertex *vertices = mVertices.data();
	for (const LRA *lra = mSettings->mLRAConstraints.data() + inStartIndex, *lra_end = mSettings->mLRAConstraints.data() + inEndIndex; lra < lra_end; ++lra)
	{
		MOSS_ASSERT(lra->mVertex[0] < mVertices.size());
		MOSS_ASSERT(lra->mVertex[1] < mVertices.size());
		const Vertex &vertex0 = vertices[lra->mVertex[0]];
		Vertex &vertex1 = vertices[lra->mVertex[1]];

		Vec3 x0 = vertex0.mPosition;
		Vec3 delta = vertex1.mPosition - x0;
		float delta_len_sq = delta.LengthSq();
		if (delta_len_sq > Square(lra->mMaxDistance))
			vertex1.mPosition = x0 + delta * lra->mMaxDistance / sqrt(delta_len_sq);
	}
}

void SoftBodyMotionProperties::ApplyCollisionConstraintsAndUpdateVelocities(const SoftBodyUpdateContext &inContext)
{
	MOSS_PROFILE_FUNCTION();

	float dt = inContext.mSubStepDeltaTime;
	float restitution_threshold = -2.0f * inContext.mGravity.Length() * dt;
	float vertex_radius = mSettings->mVertexRadius;
	for (Vertex &v : mVertices)
		if (v.mInvMass > 0.0f)
		{
			// Remember previous velocity for restitution calculations
			Vec3 prev_v = v.mVelocity;

			// XPBD velocity update
			v.mVelocity = (v.mPosition - v.mPreviousPosition) / dt;

			// Satisfy collision constraint
			if (v.mCollidingShapeIndex >= 0)
			{
				// Check if there is a collision
				float projected_distance = -v.mCollisionPlane.SignedDistance(v.mPosition) + vertex_radius;
				if (projected_distance > 0.0f)
				{
					// Remember that there was a collision
					v.mHasContact = true;

					// We need a contact callback if one of the vertices collided
					mNeedContactCallback.store(true, memory_order_relaxed);

					// Note that we already calculated the velocity, so this does not affect the velocity (next iteration starts by setting previous position to current position)
					CollidingShape &cs = mCollidingShapes[v.mCollidingShapeIndex];
					Vec3 contact_normal = v.mCollisionPlane.GetNormal();
					v.mPosition += contact_normal * projected_distance;

					// Apply friction as described in Detailed Rigid Body Simulation with Extended Position Based Dynamics - Matthias Muller et al.
					// See section 3.6:
					// Inverse mass: w1 = 1 / m1, w2 = 1 / m2 + (r2 x n)^T I^-1 (r2 x n) = 0 for a static object
					// r2 are the contact point relative to the center of mass of body 2
					// Lagrange multiplier for contact: lambda = -c / (w1 + w2)
					// Where c is the constraint equation (the distance to the plane, negative because penetrating)
					// Contact normal force: fn = lambda / dt^2
					// Delta velocity due to friction dv = -vt / |vt| * min(dt * friction * fn * (w1 + w2), |vt|) = -vt * min(-friction * c / (|vt| * dt), 1)
					// Note that I think there is an error in the paper, I added a mass term, see: https://github.com/matthias-research/pages/issues/29
					// Relative velocity: vr = v1 - v2 - omega2 x r2
					// Normal velocity: vn = vr . contact_normal
					// Tangential velocity: vt = vr - contact_normal * vn
					// Impulse: p = dv / (w1 + w2)
					// Changes in particle velocities:
					// v1 = v1 + p / m1
					// v2 = v2 - p / m2 (no change when colliding with a static body)
					// w2 = w2 - I^-1 (r2 x p) (no change when colliding with a static body)
					if (cs.mMotionType == EMotionType::Dynamic)
					{
						// Calculate normal and tangential velocity (equation 30)
						Vec3 r2 = v.mPosition - cs.mCenterOfMassTransform.GetTranslation();
						Vec3 v2 = cs.GetPointVelocity(r2);
						Vec3 relative_velocity = v.mVelocity - v2;
						Vec3 v_normal = contact_normal * contact_normal.Dot(relative_velocity);
						Vec3 v_tangential = relative_velocity - v_normal;
						float v_tangential_length = v_tangential.Length();

						// Calculate resulting inverse mass of vertex
						float vertex_inv_mass = cs.mSoftBodyInvMassScale * v.mInvMass;

						// Calculate inverse effective mass
						Vec3 r2_cross_n = r2.Cross(contact_normal);
						float w2 = cs.mInvMass + r2_cross_n.Dot(cs.mInvInertia * r2_cross_n);
						float w1_plus_w2 = vertex_inv_mass + w2;
						if (w1_plus_w2 > 0.0f)
						{
							// Calculate delta relative velocity due to friction (modified equation 31)
							Vec3 dv;
							if (v_tangential_length > 0.0f)
								dv = v_tangential * min(cs.mFriction * projected_distance / (v_tangential_length * dt), 1.0f);
							else
								dv = Vec3::sZero();

							// Calculate delta relative velocity due to restitution (equation 35)
							dv += v_normal;
							float prev_v_normal = (prev_v - v2).Dot(contact_normal);
							if (prev_v_normal < restitution_threshold)
								dv += cs.mRestitution * prev_v_normal * contact_normal;

							// Calculate impulse
							Vec3 p = dv / w1_plus_w2;

							// Apply impulse to particle
							v.mVelocity -= p * vertex_inv_mass;

							// Apply impulse to rigid body
							cs.mLinearVelocity += p * cs.mInvMass;
							cs.mAngularVelocity += cs.mInvInertia * r2.Cross(p);

							// Mark that the velocities of the body we hit need to be updated
							cs.mUpdateVelocities = true;
						}
					}
					else if (cs.mSoftBodyInvMassScale > 0.0f)
					{
						// Body is not movable, equations are simpler

						// Calculate normal and tangential velocity (equation 30)
						Vec3 v_normal = contact_normal * contact_normal.Dot(v.mVelocity);
						Vec3 v_tangential = v.mVelocity - v_normal;
						float v_tangential_length = v_tangential.Length();

						// Apply friction (modified equation 31)
						if (v_tangential_length > 0.0f)
							v.mVelocity -= v_tangential * min(cs.mFriction * projected_distance / (v_tangential_length * dt), 1.0f);

						// Apply restitution (equation 35)
						v.mVelocity -= v_normal;
						float prev_v_normal = prev_v.Dot(contact_normal);
						if (prev_v_normal < restitution_threshold)
							v.mVelocity -= cs.mRestitution * prev_v_normal * contact_normal;
					}
				}
			}
		}
}

void SoftBodyMotionProperties::UpdateSoftBodyState(SoftBodyUpdateContext &ioContext, const PhysicsSettings &inPhysicsSettings)
{
	MOSS_PROFILE_FUNCTION();

	// Contact callback
	if (mNeedContactCallback.load(memory_order_relaxed) && ioContext.mContactListener != nullptr)
	{
		// Remove non-colliding sensors from the list
		for (int i = int(mCollidingSensors.size()) - 1; i >= 0; --i)
			if (!mCollidingSensors[i].mHasContact)
			{
				mCollidingSensors[i] = std::move(mCollidingSensors.back());
				mCollidingSensors.pop_back();
			}

		ioContext.mContactListener->OnSoftBodyContactAdded(*ioContext.mBody, SoftBodyManifold(this));
	}

	// Loop through vertices once more to update the global state
	float dt = ioContext.mDeltaTime;
	float max_linear_velocity_sq = Square(GetMaxLinearVelocity());
	float max_v_sq = 0.0f;
	Vec3 linear_velocity = Vec3::sZero(), angular_velocity = Vec3::sZero();
	mLocalPredictedBounds = mLocalBounds = { };
	for (Vertex &v : mVertices)
	{
		// Calculate max square velocity
		float v_sq = v.mVelocity.LengthSq();
		max_v_sq = max(max_v_sq, v_sq);

		// Clamp if velocity is too high
		if (v_sq > max_linear_velocity_sq)
			v.mVelocity *= sqrt(max_linear_velocity_sq / v_sq);

		// Calculate local linear/angular velocity
		linear_velocity += v.mVelocity;
		angular_velocity += v.mPosition.Cross(v.mVelocity);

		// Update local bounding box
		mLocalBounds.Encapsulate(v.mPosition);

		// Create predicted position for the next frame in order to detect collisions before they happen
		mLocalPredictedBounds.Encapsulate(v.mPosition + v.mVelocity * dt + ioContext.mDisplacementDueToGravity);

		// Reset collision data for the next iteration
		v.ResetCollision();
	}

	// Calculate linear/angular velocity of the body by averaging all vertices and bringing the value to world space
	float num_vertices_divider = float(max(int(mVertices.size()), 1));
	SetLinearVelocityClamped(ioContext.mCenterOfMassTransform.Multiply3x3(linear_velocity / num_vertices_divider));
	SetAngularVelocity(ioContext.mCenterOfMassTransform.Multiply3x3(angular_velocity / num_vertices_divider));

	if (mUpdatePosition)
	{
		// Shift the body so that the position is the center of the local bounds
		Vec3 delta = mLocalBounds.GetCenter();
		ioContext.mDeltaPosition = ioContext.mCenterOfMassTransform.Multiply3x3(delta);
		for (Vertex &v : mVertices)
			v.mPosition -= delta;

		// Update the skin state too since we will use this position as the previous position in the next update
		for (SkinState &s : mSkinState)
			s.mPosition -= delta;
		MOSS_IF_DEBUG_RENDERER(mSkinStateTransform.SetTranslation(mSkinStateTransform.GetTranslation() + ioContext.mDeltaPosition);)

		// Offset bounds to match new position
		mLocalBounds.Translate(-delta);
		mLocalPredictedBounds.Translate(-delta);
	}
	else
		ioContext.mDeltaPosition = Vec3::sZero();

	// Test if we should go to sleep
	if (GetAllowSleeping())
	{
		if (max_v_sq > inPhysicsSettings.mPointVelocitySleepThreshold)
		{
			ResetSleepTestTimer();
			ioContext.mCanSleep = ECanSleep::CannotSleep;
		}
		else
			ioContext.mCanSleep = AccumulateSleepTime(dt, inPhysicsSettings.mTimeBeforeSleep);
	}
	else
		ioContext.mCanSleep = ECanSleep::CannotSleep;

	// If SkinVertices is not called after this then don't use the previous position as the skin is static
	mSkinStatePreviousPositionValid = false;

	// Reset force accumulator
	ResetForce();
}

void SoftBodyMotionProperties::UpdateRigidBodyVelocities(const SoftBodyUpdateContext &inContext, BodyInterface &inBodyInterface)
{
	MOSS_PROFILE_FUNCTION();

	// Write back velocity deltas
	for (const CollidingShape &cs : mCollidingShapes)
		if (cs.mUpdateVelocities)
			inBodyInterface.AddLinearAndAngularVelocity(cs.mBodyID, inContext.mCenterOfMassTransform.Multiply3x3(cs.mLinearVelocity - cs.mOriginalLinearVelocity), inContext.mCenterOfMassTransform.Multiply3x3(cs.mAngularVelocity - cs.mOriginalAngularVelocity));

	// Clear colliding shapes/sensors to avoid hanging on to references to shapes
	mCollidingShapes.clear();
	mCollidingSensors.clear();
}

void SoftBodyMotionProperties::InitializeUpdateContext(float inDeltaTime, Body &inSoftBody, const PhysicsSystem &inSystem, SoftBodyUpdateContext &ioContext)
{
	MOSS_PROFILE_FUNCTION();

	// Store body
	ioContext.mBody = &inSoftBody;
	ioContext.mMotionProperties = this;
	ioContext.mContactListener = inSystem.GetSoftBodyContactListener();
	ioContext.mSimShapeFilter = inSystem.GetSimShapeFilter();

	// Convert gravity to local space
	ioContext.mCenterOfMassTransform = inSoftBody.GetCenterOfMassTransform();
	ioContext.mGravity = ioContext.mCenterOfMassTransform.Multiply3x3Transposed(GetGravityFactor() * inSystem.GetGravity());

	// Calculate delta time for sub step
	ioContext.mDeltaTime = inDeltaTime;
	ioContext.mSubStepDeltaTime = inDeltaTime / mNumIterations;

	// Calculate total displacement we'll have due to gravity over all sub steps
	// The total displacement as produced by our integrator can be written as: Sum(i * g * dt^2, i = 0..mNumIterations).
	// This is bigger than 0.5 * g * dt^2 because we first increment the velocity and then update the position
	// Using Sum(i, i = 0..n) = n * (n + 1) / 2 we can write this as:
	ioContext.mDisplacementDueToGravity = (0.5f * mNumIterations * (mNumIterations + 1) * Square(ioContext.mSubStepDeltaTime)) * ioContext.mGravity;
}

void SoftBodyMotionProperties::StartNextIteration(const SoftBodyUpdateContext &ioContext)
{
	ApplyPressure(ioContext);

	IntegratePositions(ioContext);
}

void SoftBodyMotionProperties::StartFirstIteration(SoftBodyUpdateContext &ioContext)
{
	// Start the first iteration
	MOSS_IF_ENABLE_ASSERTS(uint iteration =) ioContext.mNextIteration.fetch_add(1, memory_order_relaxed);
	MOSS_ASSERT(iteration == 0);
	StartNextIteration(ioContext);
	ioContext.mState.store(SoftBodyUpdateContext::EState::ApplyConstraints, memory_order_release);
}

SoftBodyMotionProperties::EStatus SoftBodyMotionProperties::ParallelDetermineCollisionPlanes(SoftBodyUpdateContext &ioContext)
{
	// Do a relaxed read first to see if there is any work to do (this prevents us from doing expensive atomic operations and also prevents us from continuously incrementing the counter and overflowing it)
	uint num_vertices = (uint)mVertices.size();
	if (ioContext.mNextCollisionVertex.load(memory_order_relaxed) < num_vertices)
	{
		// Fetch next batch of vertices to process
		uint next_vertex = ioContext.mNextCollisionVertex.fetch_add(SoftBodyUpdateContext::cVertexCollisionBatch, memory_order_acquire);
		if (next_vertex < num_vertices)
		{
			// Process collision planes
			uint num_vertices_to_process = min(SoftBodyUpdateContext::cVertexCollisionBatch, num_vertices - next_vertex);
			DetermineCollisionPlanes(next_vertex, num_vertices_to_process);
			uint vertices_processed = ioContext.mNumCollisionVerticesProcessed.fetch_add(SoftBodyUpdateContext::cVertexCollisionBatch, memory_order_acq_rel) + num_vertices_to_process;
			if (vertices_processed >= num_vertices)
			{
				// Determine next state
				if (mCollidingSensors.empty())
					StartFirstIteration(ioContext);
				else
					ioContext.mState.store(SoftBodyUpdateContext::EState::DetermineSensorCollisions, memory_order_release);
			}
			return EStatus::DidWork;
		}
	}

	return EStatus::NoWork;
}

SoftBodyMotionProperties::EStatus SoftBodyMotionProperties::ParallelDetermineSensorCollisions(SoftBodyUpdateContext &ioContext)
{
	// Do a relaxed read to see if there are more sensors to process
	if (ioContext.mNextSensorIndex.load(memory_order_relaxed) < mNumSensors)
	{
		// Fetch next sensor to process
		uint sensor_index = ioContext.mNextSensorIndex.fetch_add(1, memory_order_acquire);
		if (sensor_index < mNumSensors)
		{
			// Process this sensor
			DetermineSensorCollisions(mCollidingSensors[sensor_index]);

			// Determine next state
			uint sensors_processed = ioContext.mNumSensorsProcessed.fetch_add(1, memory_order_acq_rel) + 1;
			if (sensors_processed >= mNumSensors)
				StartFirstIteration(ioContext);
			return EStatus::DidWork;
		}
	}

	return EStatus::NoWork;
}

void SoftBodyMotionProperties::ProcessGroup(const SoftBodyUpdateContext &ioContext, uint inGroupIndex)
{
	// Determine start and end
	SoftBodySharedSettings::UpdateGroup start { 0, 0, 0, 0, 0 };
	const SoftBodySharedSettings::UpdateGroup &prev = inGroupIndex > 0? mSettings->mUpdateGroups[inGroupIndex - 1] : start;
	const SoftBodySharedSettings::UpdateGroup &current = mSettings->mUpdateGroups[inGroupIndex];

	// Process volume constraints
	ApplyVolumeConstraints(ioContext, prev.mVolumeEndIndex, current.mVolumeEndIndex);

	// Process bend constraints
	ApplyDihedralBendConstraints(ioContext, prev.mDihedralBendEndIndex, current.mDihedralBendEndIndex);

	// Process skinned constraints
	ApplySkinConstraints(ioContext, prev.mSkinnedEndIndex, current.mSkinnedEndIndex);

	// Process edges
	ApplyEdgeConstraints(ioContext, prev.mEdgeEndIndex, current.mEdgeEndIndex);

	// Process LRA constraints
	ApplyLRAConstraints(prev.mLRAEndIndex, current.mLRAEndIndex);
}

SoftBodyMotionProperties::EStatus SoftBodyMotionProperties::ParallelApplyConstraints(SoftBodyUpdateContext &ioContext, const PhysicsSettings &inPhysicsSettings)
{
	uint num_groups = (uint)mSettings->mUpdateGroups.size();
	MOSS_ASSERT(num_groups > 0, "SoftBodySharedSettings::Optimize should have been called!");
	--num_groups; // Last group is the non-parallel group, we don't want to execute it in parallel

	// Do a relaxed read first to see if there is any work to do (this prevents us from doing expensive atomic operations and also prevents us from continuously incrementing the counter and overflowing it)
	uint next_group = ioContext.mNextConstraintGroup.load(memory_order_relaxed);
	if (next_group < num_groups || (num_groups == 0 && next_group == 0))
	{
		// Fetch the next group process
		next_group = ioContext.mNextConstraintGroup.fetch_add(1, memory_order_acquire);
		if (next_group < num_groups || (num_groups == 0 && next_group == 0))
		{
			uint num_groups_processed = 0;
			if (num_groups > 0)
			{
				// Process this group
				ProcessGroup(ioContext, next_group);

				// Increment total number of groups processed
				num_groups_processed = ioContext.mNumConstraintGroupsProcessed.fetch_add(1, memory_order_acq_rel) + 1;
			}

			if (num_groups_processed >= num_groups)
			{
				// Finish the iteration
				MOSS_PROFILE("FinishIteration");

				// Process non-parallel group
				ProcessGroup(ioContext, num_groups);

				ApplyCollisionConstraintsAndUpdateVelocities(ioContext);

				uint iteration = ioContext.mNextIteration.fetch_add(1, memory_order_relaxed);
				if (iteration < mNumIterations)
				{
					// Start a new iteration
					StartNextIteration(ioContext);

					// Reset group logic
					ioContext.mNumConstraintGroupsProcessed.store(0, memory_order_release);
					ioContext.mNextConstraintGroup.store(0, memory_order_release);
				}
				else
				{
					// On final iteration we update the state
					UpdateSoftBodyState(ioContext, inPhysicsSettings);

					ioContext.mState.store(SoftBodyUpdateContext::EState::Done, memory_order_release);
					return EStatus::Done;
				}
			}

			return EStatus::DidWork;
		}
	}
	return EStatus::NoWork;
}

SoftBodyMotionProperties::EStatus SoftBodyMotionProperties::ParallelUpdate(SoftBodyUpdateContext &ioContext, const PhysicsSettings &inPhysicsSettings)
{
	switch (ioContext.mState.load(memory_order_acquire))
	{
	case SoftBodyUpdateContext::EState::DetermineCollisionPlanes:
		return ParallelDetermineCollisionPlanes(ioContext);

	case SoftBodyUpdateContext::EState::DetermineSensorCollisions:
		return ParallelDetermineSensorCollisions(ioContext);

	case SoftBodyUpdateContext::EState::ApplyConstraints:
		return ParallelApplyConstraints(ioContext, inPhysicsSettings);

	case SoftBodyUpdateContext::EState::Done:
		return EStatus::Done;

	default:
		MOSS_ASSERT(false);
		return EStatus::NoWork;
	}
}

void SoftBodyMotionProperties::SkinVertices([[maybe_unused]] RMat44Arg inCenterOfMassTransform, const Mat44 *inJointMatrices, [[maybe_unused]] uint inNumJoints, bool inHardSkinAll, TempAllocator &ioTempAllocator)
{
	// Calculate the skin matrices
	uint num_skin_matrices = uint(mSettings->mInvBindMatrices.size());
	uint skin_matrices_size = num_skin_matrices * sizeof(Mat44);
	Mat44 *skin_matrices = (Mat44 *)ioTempAllocator.Allocate(skin_matrices_size);
	MOSS_SCOPE_EXIT([&ioTempAllocator, skin_matrices, skin_matrices_size]{ ioTempAllocator.Free(skin_matrices, skin_matrices_size); });
	const Mat44 *skin_matrices_end = skin_matrices + num_skin_matrices;
	const InvBind *inv_bind_matrix = mSettings->mInvBindMatrices.data();
	for (Mat44 *s = skin_matrices; s < skin_matrices_end; ++s, ++inv_bind_matrix)
	{
		MOSS_ASSERT(inv_bind_matrix->mJointIndex < inNumJoints);
		*s = inJointMatrices[inv_bind_matrix->mJointIndex] * inv_bind_matrix->mInvBind;
	}

	// Skin the vertices
	MOSS_IF_DEBUG_RENDERER(mSkinStateTransform = inCenterOfMassTransform;)
	MOSS_IF_ENABLE_ASSERTS(uint num_vertices = uint(mSettings->mVertices.size());)
	MOSS_ASSERT(mSkinState.size() == num_vertices);
	const SoftBodySharedSettings::Vertex *in_vertices = mSettings->mVertices.data();
	for (const Skinned &s : mSettings->mSkinnedConstraints)
	{
		// Get bind pose
		MOSS_ASSERT(s.mVertex < num_vertices);
		Vec3 bind_pos = Vec3::sLoadFloat3Unsafe(in_vertices[s.mVertex].mPosition);

		// Skin vertex
		Vec3 pos = Vec3::sZero();
		for (const SkinWeight &w : s.mWeights)
		{
			// We assume that the first zero weight is the end of the list
			if (w.mWeight == 0.0f)
				break;

			MOSS_ASSERT(w.mInvBindIndex < num_skin_matrices);
			pos += w.mWeight * (skin_matrices[w.mInvBindIndex] * bind_pos);
		}
		SkinState &skin_state = mSkinState[s.mVertex];
		skin_state.mPreviousPosition = skin_state.mPosition;
		skin_state.mPosition = pos;
	}

	// Calculate the normals
	for (const Skinned &s : mSettings->mSkinnedConstraints)
	{
		Vec3 normal = Vec3::sZero();
		uint32 num_faces = s.mNormalInfo >> 24;
		if (num_faces > 0)
		{
			// Calculate normal
			const uint32 *f = &mSettings->mSkinnedConstraintNormals[s.mNormalInfo & 0xffffff];
			const uint32 *f_end = f + num_faces;
			while (f < f_end)
			{
				const Face &face = mSettings->mFaces[*f];
				Vec3 v0 = mSkinState[face.mVertex[0]].mPosition;
				Vec3 v1 = mSkinState[face.mVertex[1]].mPosition;
				Vec3 v2 = mSkinState[face.mVertex[2]].mPosition;
				normal += (v1 - v0).Cross(v2 - v0).NormalizedOr(Vec3::sZero());
				++f;
			}
			normal = normal.NormalizedOr(Vec3::sZero());
		}
		mSkinState[s.mVertex].mNormal = normal;
	}

	if (inHardSkinAll)
	{
		// Hard skin all vertices and reset their velocities
		for (const Skinned &s : mSettings->mSkinnedConstraints)
		{
			Vertex &vertex = mVertices[s.mVertex];
			SkinState &skin_state = mSkinState[s.mVertex];
			skin_state.mPreviousPosition = skin_state.mPosition;
			vertex.mPosition = skin_state.mPosition;
			vertex.mVelocity = Vec3::sZero();
		}
	}
	else if (!mEnableSkinConstraints)
	{
		// Hard skin only the kinematic vertices as we will not solve the skin constraints later
		for (const Skinned &s : mSettings->mSkinnedConstraints)
			if (s.mMaxDistance == 0.0f)
			{
				Vertex &vertex = mVertices[s.mVertex];
				vertex.mPosition = mSkinState[s.mVertex].mPosition;
			}
	}

	// Indicate that the previous positions are valid for the coming update
	mSkinStatePreviousPositionValid = true;
}

void SoftBodyMotionProperties::CustomUpdate(float inDeltaTime, Body &ioSoftBody, PhysicsSystem &inSystem)
{
	MOSS_PROFILE_FUNCTION();

	// Create update context
	SoftBodyUpdateContext context;
	InitializeUpdateContext(inDeltaTime, ioSoftBody, inSystem, context);

	// Determine bodies we're colliding with
	DetermineCollidingShapes(context, inSystem, inSystem.GetBodyLockInterface());

	// Call the internal update until it finishes
	EStatus status;
	const PhysicsSettings &settings = inSystem.GetPhysicsSettings();
	while ((status = ParallelUpdate(context, settings)) == EStatus::DidWork)
		continue;
	MOSS_ASSERT(status == EStatus::Done);

	// Update the state of the bodies we've collided with
	UpdateRigidBodyVelocities(context, inSystem.GetBodyInterface());

	// Update position of the soft body
	if (mUpdatePosition)
		inSystem.GetBodyInterface().SetPosition(ioSoftBody.GetID(), ioSoftBody.GetPosition() + context.mDeltaPosition, EActivation::DontActivate);
}

#ifndef MOSS_DEBUG_RENDERER

void SoftBodyMotionProperties::DrawVertices(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform) const
{
	for (const Vertex &v : mVertices)
		inRenderer->DrawMarker(inCenterOfMassTransform * v.mPosition, v.mInvMass > 0.0f? Color::sGreen : Color::sRed, 0.05f);
}

void SoftBodyMotionProperties::DrawVertexVelocities(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform) const
{
	for (const Vertex &v : mVertices)
		inRenderer->DrawArrow(inCenterOfMassTransform * v.mPosition, inCenterOfMassTransform * (v.mPosition + v.mVelocity), Color::sYellow, 0.01f);
}

template <typename GetEndIndex, typename DrawConstraint>
inline void SoftBodyMotionProperties::DrawConstraints(ESoftBodyConstraintColor inConstraintColor, const GetEndIndex &inGetEndIndex, const DrawConstraint &inDrawConstraint, ColorArg inBaseColor) const
{
	uint start = 0;
	for (uint i = 0; i < (uint)mSettings->mUpdateGroups.size(); ++i)
	{
		uint end = inGetEndIndex(mSettings->mUpdateGroups[i]);

		Color base_color;
		if (inConstraintColor != ESoftBodyConstraintColor::ConstraintType)
			base_color = Color::sGetDistinctColor((uint)mSettings->mUpdateGroups.size() - i - 1); // Ensure that color 0 is always the last group
		else
			base_color = inBaseColor;

		for (uint idx = start; idx < end; ++idx)
		{
			Color color = inConstraintColor == ESoftBodyConstraintColor::ConstraintOrder? base_color * (float(idx - start) / (end - start)) : base_color;
			inDrawConstraint(idx, color);
		}

		start = end;
	}
}

void SoftBodyMotionProperties::DrawEdgeConstraints(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, ESoftBodyConstraintColor inConstraintColor) const
{
	DrawConstraints(inConstraintColor,
		[](const SoftBodySharedSettings::UpdateGroup &inGroup) {
			return inGroup.mEdgeEndIndex;
		},
		[this, inRenderer, &inCenterOfMassTransform](uint inIndex, ColorArg inColor) {
			const Edge &e = mSettings->mEdgeConstraints[inIndex];
			inRenderer->DrawLine(inCenterOfMassTransform * mVertices[e.mVertex[0]].mPosition, inCenterOfMassTransform * mVertices[e.mVertex[1]].mPosition, inColor);
		},
		Color::sWhite);
}

void SoftBodyMotionProperties::DrawBendConstraints(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, ESoftBodyConstraintColor inConstraintColor) const
{
	DrawConstraints(inConstraintColor,
		[](const SoftBodySharedSettings::UpdateGroup &inGroup) {
			return inGroup.mDihedralBendEndIndex;
		},
		[this, inRenderer, &inCenterOfMassTransform](uint inIndex, ColorArg inColor) {
			const DihedralBend &b = mSettings->mDihedralBendConstraints[inIndex];

			RVec3 x0 = inCenterOfMassTransform * mVertices[b.mVertex[0]].mPosition;
			RVec3 x1 = inCenterOfMassTransform * mVertices[b.mVertex[1]].mPosition;
			RVec3 x2 = inCenterOfMassTransform * mVertices[b.mVertex[2]].mPosition;
			RVec3 x3 = inCenterOfMassTransform * mVertices[b.mVertex[3]].mPosition;
			RVec3 c_edge = 0.5_r * (x0 + x1);
			RVec3 c0 = (x0 + x1 + x2) / 3.0_r;
			RVec3 c1 = (x0 + x1 + x3) / 3.0_r;

			inRenderer->DrawArrow(0.9_r * x0 + 0.1_r * x1, 0.1_r * x0 + 0.9_r * x1, inColor, 0.01f);
			inRenderer->DrawLine(c_edge, 0.1_r * c_edge + 0.9_r * c0, inColor);
			inRenderer->DrawLine(c_edge, 0.1_r * c_edge + 0.9_r * c1, inColor);
		},
		Color::sGreen);
}

void SoftBodyMotionProperties::DrawVolumeConstraints(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, ESoftBodyConstraintColor inConstraintColor) const
{
	DrawConstraints(inConstraintColor,
		[](const SoftBodySharedSettings::UpdateGroup &inGroup) {
			return inGroup.mVolumeEndIndex;
		},
		[this, inRenderer, &inCenterOfMassTransform](uint inIndex, ColorArg inColor) {
			const Volume &v = mSettings->mVolumeConstraints[inIndex];

			RVec3 x1 = inCenterOfMassTransform * mVertices[v.mVertex[0]].mPosition;
			RVec3 x2 = inCenterOfMassTransform * mVertices[v.mVertex[1]].mPosition;
			RVec3 x3 = inCenterOfMassTransform * mVertices[v.mVertex[2]].mPosition;
			RVec3 x4 = inCenterOfMassTransform * mVertices[v.mVertex[3]].mPosition;

			inRenderer->DrawTriangle(x1, x3, x2, inColor, DebugRenderer::ECastShadow::On);
			inRenderer->DrawTriangle(x2, x3, x4, inColor, DebugRenderer::ECastShadow::On);
			inRenderer->DrawTriangle(x1, x4, x3, inColor, DebugRenderer::ECastShadow::On);
			inRenderer->DrawTriangle(x1, x2, x4, inColor, DebugRenderer::ECastShadow::On);
		},
		Color::sYellow);
}

void SoftBodyMotionProperties::DrawSkinConstraints(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, ESoftBodyConstraintColor inConstraintColor) const
{
	DrawConstraints(inConstraintColor,
		[](const SoftBodySharedSettings::UpdateGroup &inGroup) {
			return inGroup.mSkinnedEndIndex;
		},
		[this, inRenderer, &inCenterOfMassTransform](uint inIndex, ColorArg inColor) {
			const Skinned &s = mSettings->mSkinnedConstraints[inIndex];
			const SkinState &skin_state = mSkinState[s.mVertex];
			inRenderer->DrawArrow(mSkinStateTransform * skin_state.mPosition, mSkinStateTransform * (skin_state.mPosition + 0.1f * skin_state.mNormal), inColor, 0.01f);
			inRenderer->DrawLine(mSkinStateTransform * skin_state.mPosition, inCenterOfMassTransform * mVertices[s.mVertex].mPosition, Color::sBlue);
		},
		Color::sOrange);
}

void SoftBodyMotionProperties::DrawLRAConstraints(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, ESoftBodyConstraintColor inConstraintColor) const
{
	DrawConstraints(inConstraintColor,
		[](const SoftBodySharedSettings::UpdateGroup &inGroup) {
			return inGroup.mLRAEndIndex;
		},
		[this, inRenderer, &inCenterOfMassTransform](uint inIndex, ColorArg inColor) {
			const LRA &l = mSettings->mLRAConstraints[inIndex];
			inRenderer->DrawLine(inCenterOfMassTransform * mVertices[l.mVertex[0]].mPosition, inCenterOfMassTransform * mVertices[l.mVertex[1]].mPosition, inColor);
		},
		Color::sGrey);
}

void SoftBodyMotionProperties::DrawPredictedBounds(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform) const
{
	inRenderer->DrawWireBox(inCenterOfMassTransform, mLocalPredictedBounds, Color::sRed);
}

#endif // MOSS_DEBUG_RENDERER

void SoftBodyMotionProperties::SaveState(StateRecorder &inStream) const
{
	MotionProperties::SaveState(inStream);

	for (const Vertex &v : mVertices)
	{
		inStream.Write(v.mPreviousPosition);
		inStream.Write(v.mPosition);
		inStream.Write(v.mVelocity);
	}

	for (const SkinState &s : mSkinState)
	{
		inStream.Write(s.mPreviousPosition);
		inStream.Write(s.mPosition);
		inStream.Write(s.mNormal);
	}

	inStream.Write(mLocalBounds.mMin);
	inStream.Write(mLocalBounds.mMax);
	inStream.Write(mLocalPredictedBounds.mMin);
	inStream.Write(mLocalPredictedBounds.mMax);
}

void SoftBodyMotionProperties::RestoreState(StateRecorder &inStream)
{
	MotionProperties::RestoreState(inStream);

	for (Vertex &v : mVertices)
	{
		inStream.Read(v.mPreviousPosition);
		inStream.Read(v.mPosition);
		inStream.Read(v.mVelocity);
	}

	for (SkinState &s : mSkinState)
	{
		inStream.Read(s.mPreviousPosition);
		inStream.Read(s.mPosition);
		inStream.Read(s.mNormal);
	}

	inStream.Read(mLocalBounds.mMin);
	inStream.Read(mLocalBounds.mMax);
	inStream.Read(mLocalPredictedBounds.mMin);
	inStream.Read(mLocalPredictedBounds.mMax);
}

MOSS_SUPRESS_WARNINGS_END
