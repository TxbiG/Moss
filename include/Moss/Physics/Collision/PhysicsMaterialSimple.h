// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Moss/Physics/Collision/PhysicsMaterial.h>

MOSS_NAMESPACE_BEGIN

/// Sample implementation of PhysicsMaterial that just holds the needed properties directly
class MOSS_EXPORT PhysicsMaterialSimple : public PhysicsMaterial
{
	MOSS_DECLARE_SERIALIZABLE_VIRTUAL(MOSS_EXPORT, PhysicsMaterialSimple)

public:
	/// Constructor
											PhysicsMaterialSimple() = default;
											PhysicsMaterialSimple(const string_view &inName, ColorArg inColor) : mDebugName(inName), mDebugColor(inColor) { }

	// Properties
	virtual const char *					GetDebugName() const override		{ return mDebugName.c_str(); }
	virtual Color							GetDebugColor() const override		{ return mDebugColor; }

	// See: PhysicsMaterial::SaveBinaryState
	virtual void							SaveBinaryState(StreamOut &inStream) const override;

protected:
	// See: PhysicsMaterial::RestoreBinaryState
	virtual void							RestoreBinaryState(StreamIn &inStream) override;

private:
	String									mDebugName;							///< Name of the material, used for debugging purposes
	Color									mDebugColor = Color::sGrey;			///< Color of the material, used to render the shapes
};

MOSS_NAMESPACE_END
