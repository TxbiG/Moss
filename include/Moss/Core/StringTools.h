// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

MOSS_SUPRESS_WARNINGS_BEGIN

/// Create a formatted text string for debugging purposes.
/// Note that this function has an internal buffer of 1024 characters, so long strings will be trimmed.
MOSS_EXPORT String StringFormat(const char *inFMT, ...);

/// Convert type to string
template<typename T>
String ConvertToString(const T &inValue)
{
	using OStringStream = std::basic_ostringstream<char, std::char_traits<char>, STLAllocator<char>>;
	OStringStream oss;
	oss << inValue;
	return oss.str();
}

/// Replace substring with other string
MOSS_EXPORT void StringReplace(String &ioString, const string_view &inSearch, const string_view &inReplace);

/// Convert a delimited string to an array of strings
MOSS_EXPORT void StringToVector(const string_view &inString, TArray<String> &outVector, const string_view &inDelimiter = ",", bool inClearVector = true);

/// Convert an array strings to a delimited string
MOSS_EXPORT void VectorToString(const TArray<String> &inVector, String &outString, const string_view &inDelimiter = ",");

/// Convert a string to lower case
MOSS_EXPORT String ToLower(const string_view &inString);

/// Converts the lower 4 bits of inNibble to a string that represents the number in binary format
MOSS_EXPORT const char *NibbleToBinary(uint32 inNibble);

MOSS_SUPRESS_WARNINGS_END
