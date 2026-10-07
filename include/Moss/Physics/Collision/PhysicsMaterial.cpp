// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <Moss/Moss.h>

#include <Moss/Physics/Collision/PhysicsMaterial.h>
#include <Moss/Physics/Collision/PhysicsMaterialSimple.h>
#include <Moss/Core/StreamUtils.h>

MOSS_SUPPRESS_WARNINGS_BEGIN

RefConst<PhysicsMaterial> PhysicsMaterial::sDefault;

JPH_IMPLEMENT_SERIALIZABLE_VIRTUAL(PhysicsMaterial)
{
	JPH_ADD_BASE_CLASS(PhysicsMaterial, SerializableObject)
}

void PhysicsMaterial::SaveBinaryState(StreamOut &inStream) const
{
	inStream.Write(GetRTTI()->GetHash());
}

void PhysicsMaterial::RestoreBinaryState(StreamIn &inStream)
{
	// RTTI hash is read in sRestoreFromBinaryState
}

PhysicsMaterial::PhysicsMaterialResult PhysicsMaterial::sRestoreFromBinaryState(StreamIn &inStream)
{
	return StreamUtils::RestoreObject<PhysicsMaterial>(inStream, &PhysicsMaterial::RestoreBinaryState);
}

MOSS_SUPPRESS_WARNINGS_END
