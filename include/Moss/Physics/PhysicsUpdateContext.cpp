// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <Moss/Physics/PhysicsUpdateContext.h>

MOSS_SUPRESS_WARNINGS_BEGIN

PhysicsUpdateContext::PhysicsUpdateContext(TempAllocator &inTempAllocator) :
	mTempAllocator(&inTempAllocator),
	mSteps(inTempAllocator)
{
}

PhysicsUpdateContext::~PhysicsUpdateContext()
{
	MOSS_ASSERT(mBodyPairs == nullptr);
	MOSS_ASSERT(mActiveConstraints == nullptr);
}

MOSS_SUPRESS_WARNINGS_END
