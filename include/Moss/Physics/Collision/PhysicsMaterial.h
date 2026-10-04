// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Moss/Core/Reference.h>
#include <Moss/Variants/Color.h>
#include <Moss/Core/Result.h>
#include <Moss/Resources/ObjectStream/SerializableObject.h>

MOSS_SUPPRESS_WARNINGS_BEGIN

class StreamIn;
class StreamOut;

/// This structure describes the surface of (part of) a shape. You should inherit from it to define additional
/// information that is interesting for the simulation. The 2 materials involved in a contact could be used
/// to decide which sound or particle effects to play.
///
/// If you inherit from this material, don't forget to create a suitable default material in Default
class MOSS_EXPORT PhysicsMaterial : public SerializableObject, public RefTarget<PhysicsMaterial> {
	MOSS_DECLARE_SERIALIZABLE_VIRTUAL(MOSS_EXPORT, PhysicsMaterial)
public:
	/// Constructor
											PhysicsMaterial() = default;
	virtual									~PhysicsMaterial() override = default;

	/// Default material that is used when a shape has no materials defined
	static RefConst<PhysicsMaterial>		Default;

	// Properties
	virtual const char*					GetDebugName() const			{ return "Unknown"; }
	virtual Color						GetDebugColor() const			{ return Color::Grey; }

	/// Saves the contents of the material in binary form to inStream.
	virtual void							SaveBinaryState(StreamOut &inStream) const;

	using PhysicsMaterialResult = Result<Ref<PhysicsMaterial>>;

	/// Creates a PhysicsMaterial of the correct type and restores its contents from the binary stream inStream.
	static PhysicsMaterialResult			sRestoreFromBinaryState(StreamIn &inStream);

protected:
	/// Don't allow copy constructing this base class, but allow derived classes to copy themselves
											PhysicsMaterial(const PhysicsMaterial &) = default;
	PhysicsMaterial &						operator = (const PhysicsMaterial &) = default;

	/// This function should not be called directly, it is used by sRestoreFromBinaryState.
	virtual void							RestoreBinaryState(StreamIn &inStream);
};

using PhysicsMaterialList = TArray<RefConst<PhysicsMaterial>>;

MOSS_SUPPRESS_WARNINGS_END
