// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#ifdef MOSS_OBJECT_STREAM

#include <Moss/Resources/ObjectStream/SerializableAttribute.h>
#include <Moss/Resources/ObjectStream/ObjectStream.h>

MOSS_SUPPRESS_WARNINGS_END

//////////////////////////////////////////////////////////////////////////////////////////
// Macros to add properties to be serialized
//////////////////////////////////////////////////////////////////////////////////////////

template <class MemberType>
inline void AddSerializableAttributeEnum(RTTI &inRTTI, uint32 inOffset, const char *inName) {
	inRTTI.AddAttribute(SerializableAttribute(inName, inOffset,
		[]() -> const RTTI *
		{
			return nullptr;
		},
		[](int inArrayDepth, EOSDataType inDataType, [[maybe_unused]] const char *inClassName)
		{
			return inArrayDepth == 0 && inDataType == EOSDataType::T_uint32;
		},
		[](IObjectStreamIn &ioStream, void *inObject)
		{
			uint32 temporary;
			if (OSReadData(ioStream, temporary))
			{
				*reinterpret_cast<MemberType *>(inObject) = static_cast<MemberType>(temporary);
				return true;
			}
			return false;
		},
		[](IObjectStreamOut &ioStream, const void *inObject)
		{
			static_assert(sizeof(MemberType) <= sizeof(uint32));
			uint32 temporary = uint32(*reinterpret_cast<const MemberType *>(inObject));
			OSWriteData(ioStream, temporary);
		},
		[](IObjectStreamOut &ioStream)
		{
			ioStream.WriteDataType(EOSDataType::T_uint32);
		}));
}

// MOSS_ADD_ENUM_ATTRIBUTE_WITH_ALIAS
#define MOSS_ADD_ENUM_ATTRIBUTE_WITH_ALIAS(class_name, member_name, alias_name) \
	AddSerializableAttributeEnum<decltype(class_name::member_name)>(inRTTI, offsetof(class_name, member_name), alias_name);

// MOSS_ADD_ENUM_ATTRIBUTE
#define MOSS_ADD_ENUM_ATTRIBUTE(class_name, member_name) \
	MOSS_ADD_ENUM_ATTRIBUTE_WITH_ALIAS(class_name, member_name, #member_name);

MOSS_SUPPRESS_WARNINGS_END

#else

#define MOSS_ADD_ENUM_ATTRIBUTE_WITH_ALIAS(...)
#define MOSS_ADD_ENUM_ATTRIBUTE(...)

#endif // MOSS_OBJECT_STREAM
