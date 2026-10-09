// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2023 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Moss/Physics/Collision/ObjectLayer.h>

MOSS_SUPPRESS_WARNINGS_BEGIN

/// Filter class to test if two objects can collide based on their object layer. Used while finding collision pairs.
/// Uses group bits and mask bits. Two layers can collide if Object1.Group & Object2.Mask is non-zero and Object2.Group & Object1.Mask is non-zero.
/// The behavior is similar to that in e.g. Bullet.
/// This implementation works together with BroadPhaseLayerInterfaceMask and ObjectVsBroadPhaseLayerFilterMask
class ObjectLayerPairFilterMask : public ObjectLayerPairFilter
{
public:
	MOSS_OVERRIDE_NEW_DELETE

	/// Number of bits for the group and mask bits
	static constexpr uint32_t cNumBits = MOSS_OBJECT_LAYER_BITS / 2;
	static constexpr uint32_t	cMask = (1 << cNumBits) - 1;

	/// Construct an ObjectLayer from a group and mask bits
	static ObjectLayer		GetObjectLayer(uint32_t inGroup, uint32_t inMask = cMask) {
		MOSS_ASSERT((inGroup & ~cMask) == 0);
		MOSS_ASSERT((inMask & ~cMask) == 0);
		return ObjectLayer((inGroup & cMask) | (inMask << cNumBits));
	}

	/// Get the group bits from an ObjectLayer
	static inline uint32_t	GetGroup(ObjectLayer inObjectLayer) { return uint32_t(inObjectLayer) & cMask; }

	/// Get the mask bits from an ObjectLayer
	static inline uint32_t	GetMask(ObjectLayer inObjectLayer) { return uint32_t(inObjectLayer) >> cNumBits; }

	/// Returns true if two layers can collide
	virtual bool			ShouldCollide(ObjectLayer inObject1, ObjectLayer inObject2) const override {
		return (sGetGroup(inObject1) & sGetMask(inObject2)) != 0 && (sGetGroup(inObject2) & sGetMask(inObject1)) != 0;
	}
};

MOSS_SUPPRESS_WARNINGS_END
