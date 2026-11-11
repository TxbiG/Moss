// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <Moss/Resources/ObjectStream/ObjectStream.h>

#ifdef MOSS_OBJECT_STREAM

MOSS_SUPPRESS_WARNINGS_END

// Define macro to declare functions for a specific primitive type
#define MOSS_DECLARE_PRIMITIVE(name)																\
	bool	OSIsType(name *, int inArrayDepth, EOSDataType inDataType, const char *inClassName) \
	{																							\
		return inArrayDepth == 0 && inDataType == EOSDataType::T_##name;						\
	}																							\
	bool	OSReadData(IObjectStreamIn &ioStream, name &outPrimitive)							\
	{																							\
		return ioStream.ReadPrimitiveData(outPrimitive);										\
	}																							\
	void	OSWriteDataType(IObjectStreamOut &ioStream, name *)									\
	{																							\
		ioStream.WriteDataType(EOSDataType::T_##name);											\
	}																							\
	void	OSWriteData(IObjectStreamOut &ioStream, const name &inPrimitive)					\
	{																							\
		ioStream.HintNextItem();																\
		ioStream.WritePrimitiveData(inPrimitive);												\
	}

// This file uses the MOSS_DECLARE_PRIMITIVE macro to define all types
#include <Moss/Resources/ObjectStream/ObjectStreamTypes.h>

MOSS_SUPPRESS_WARNINGS_END

#endif // MOSS_OBJECT_STREAM
