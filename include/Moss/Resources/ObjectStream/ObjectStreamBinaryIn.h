// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Moss/Resources/ObjectStream/ObjectStreamIn.h>

#ifdef MOSS_OBJECT_STREAM

MOSS_SUPPRESS_WARNINGS_END

/// Implementation of ObjectStream binary input stream.
class MOSS_API ObjectStreamBinaryIn : public ObjectStreamIn
{
public:
	MOSS_OVERRIDE_NEW_DELETE

	/// Constructor
	explicit					ObjectStreamBinaryIn(istream &inStream);

	///@name Input type specific operations
	virtual bool				ReadDataType(EOSDataType &outType) override;
	virtual bool				ReadName(String &outName) override;
	virtual bool				ReadIdentifier(Identifier &outIdentifier) override;
	virtual bool				ReadCount(uint32_t &outCount) override;

	virtual bool				ReadPrimitiveData(uint8 &outPrimitive) override;
	virtual bool				ReadPrimitiveData(uint16 &outPrimitive) override;
	virtual bool				ReadPrimitiveData(int &outPrimitive) override;
	virtual bool				ReadPrimitiveData(uint32_t &outPrimitive) override;
	virtual bool				ReadPrimitiveData(uint64 &outPrimitive) override;
	virtual bool				ReadPrimitiveData(float &outPrimitive) override;
	virtual bool				ReadPrimitiveData(double &outPrimitive) override;
	virtual bool				ReadPrimitiveData(bool &outPrimitive) override;
	virtual bool				ReadPrimitiveData(String &outPrimitive) override;
	virtual bool				ReadPrimitiveData(Float3 &outPrimitive) override;
	virtual bool				ReadPrimitiveData(Float4 &outPrimitive) override;
	virtual bool				ReadPrimitiveData(Double3 &outPrimitive) override;
	virtual bool				ReadPrimitiveData(Vec3 &outPrimitive) override;
	virtual bool				ReadPrimitiveData(DVec3 &outPrimitive) override;
	virtual bool				ReadPrimitiveData(Vec4 &outPrimitive) override;
	virtual bool				ReadPrimitiveData(UVec4 &outPrimitive) override;
	virtual bool				ReadPrimitiveData(Quat &outPrimitive) override;
	virtual bool				ReadPrimitiveData(Mat44 &outPrimitive) override;
	virtual bool				ReadPrimitiveData(DMat44 &outPrimitive) override;

private:
	using StringTable = UnorderedMap<uint32_t, String>;

	StringTable					mStringTable;
	uint32_t						mNextStringID = 0x80000000;
};

MOSS_SUPPRESS_WARNINGS_END

#endif // MOSS_OBJECT_STREAM
