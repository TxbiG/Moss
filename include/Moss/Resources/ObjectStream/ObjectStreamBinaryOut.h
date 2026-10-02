// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Moss/Resources/ObjectStream/ObjectStreamOut.h>

#ifdef MOSS_OBJECT_STREAM

MOSS_NAMESPACE_BEGIN

/// Implementation of ObjectStream binary output stream.
class MOSS_EXPORT ObjectStreamBinaryOut : public ObjectStreamOut
{
public:
	MOSS_OVERRIDE_NEW_DELETE

	/// Constructor and destructor
	explicit					ObjectStreamBinaryOut(ostream &inStream);

	///@name Output type specific operations
	virtual void				WriteDataType(EOSDataType inType) override;
	virtual void				WriteName(const char *inName) override;
	virtual void				WriteIdentifier(Identifier inIdentifier) override;
	virtual void				WriteCount(uint32 inCount) override;

	virtual void				WritePrimitiveData(const uint8 &inPrimitive) override;
	virtual void				WritePrimitiveData(const uint16 &inPrimitive) override;
	virtual void				WritePrimitiveData(const int &inPrimitive) override;
	virtual void				WritePrimitiveData(const uint32 &inPrimitive) override;
	virtual void				WritePrimitiveData(const uint64 &inPrimitive) override;
	virtual void				WritePrimitiveData(const float &inPrimitive) override;
	virtual void				WritePrimitiveData(const double &inPrimitive) override;
	virtual void				WritePrimitiveData(const bool &inPrimitive) override;
	virtual void				WritePrimitiveData(const String &inPrimitive) override;
	virtual void				WritePrimitiveData(const Float3 &inPrimitive) override;
	virtual void				WritePrimitiveData(const Float4 &inPrimitive) override;
	virtual void				WritePrimitiveData(const Double3 &inPrimitive) override;
	virtual void				WritePrimitiveData(const Vec3 &inPrimitive) override;
	virtual void				WritePrimitiveData(const DVec3 &inPrimitive) override;
	virtual void				WritePrimitiveData(const Vec4 &inPrimitive) override;
	virtual void				WritePrimitiveData(const UVec4 &inPrimitive) override;
	virtual void				WritePrimitiveData(const Quat &inPrimitive) override;
	virtual void				WritePrimitiveData(const Mat44 &inPrimitive) override;
	virtual void				WritePrimitiveData(const DMat44 &inPrimitive) override;

private:
	using StringTable = UnorderedMap<String, uint32>;

	StringTable					mStringTable;
	uint32						mNextStringID = 0x80000000;
};

MOSS_NAMESPACE_END

#endif // MOSS_OBJECT_STREAM
