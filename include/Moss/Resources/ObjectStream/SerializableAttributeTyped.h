// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#ifdef MOSS_OBJECT_STREAM

#include <Moss/Resources/ObjectStream/SerializableAttribute.h>
#include <Moss/Resources/ObjectStream/GetPrimitiveTypeOfType.h>
#include <Moss/Resources/ObjectStream/ObjectStream.h>

MOSS_NAMESPACE_BEGIN

//////////////////////////////////////////////////////////////////////////////////////////
// Macros to add properties to be serialized
//////////////////////////////////////////////////////////////////////////////////////////

template <class MemberType>
inline void AddSerializableAttributeTyped(RTTI &inRTTI, uint32 inOffset, const char *inName)
{
	inRTTI.AddAttribute(SerializableAttribute(inName, inOffset,
		[]()
		{
			return GetPrimitiveTypeOfType((MemberType *)nullptr);
		},
		[](int inArrayDepth, EOSDataType inDataType, const char *inClassName)
		{
			return OSIsType((MemberType *)nullptr, inArrayDepth, inDataType, inClassName);
		},
		[](IObjectStreamIn &ioStream, void *inObject)
		{
			return OSReadData(ioStream, *reinterpret_cast<MemberType *>(inObject));
		},
		[](IObjectStreamOut &ioStream, const void *inObject)
		{
			OSWriteData(ioStream, *reinterpret_cast<const MemberType *>(inObject));
		},
		[](IObjectStreamOut &ioStream)
		{
			OSWriteDataType(ioStream, (MemberType *)nullptr);
		}));
}

// MOSS_ADD_ATTRIBUTE
#define MOSS_ADD_ATTRIBUTE_WITH_ALIAS(class_name, member_name, alias_name) \
	AddSerializableAttributeTyped<decltype(class_name::member_name)>(inRTTI, offsetof(class_name, member_name), alias_name);

// MOSS_ADD_ATTRIBUTE
#define MOSS_ADD_ATTRIBUTE(class_name, member_name) \
	MOSS_ADD_ATTRIBUTE_WITH_ALIAS(class_name, member_name, #member_name)

MOSS_NAMESPACE_END

#else

#define MOSS_ADD_ATTRIBUTE_WITH_ALIAS(...)
#define MOSS_ADD_ATTRIBUTE(...)

#endif // MOSS_OBJECT_STREAM
