// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Moss/Core/STLAllocator.h>
#include <Moss/Moss_stdinc.h>

MOSS_SUPPRESS_WARNINGS_BEGIN

/// Implements the FNV-1a hash algorithm
/// @see https://en.wikipedia.org/wiki/Fowler%E2%80%93Noll%E2%80%93Vo_hash_function
/// @param inData Data block of bytes
/// @param inSize Number of bytes
/// @param inSeed Seed of the hash (can be used to pass in the hash of a previous operation, otherwise leave default)
/// @return Hash
inline uint64_t HashBytes(const void *inData, uint64_t inSize, uint64_t inSeed = 0xcbf29ce484222325UL)
{
	uint64_t hash = inSeed;
	for (const uint8_t *data = reinterpret_cast<const uint8_t *>(inData); data < reinterpret_cast<const uint8_t *>(inData) + inSize; ++data)
	{
		hash ^= uint64_t(*data);
		hash *= 0x100000001b3UL;
	}
	return hash;
}

/// Calculate the FNV-1a hash of inString.
/// @see https://en.wikipedia.org/wiki/Fowler%E2%80%93Noll%E2%80%93Vo_hash_function
constexpr uint64_t HashString(const char *inString, uint64_t inSeed = 0xcbf29ce484222325UL)
{
	uint64_t hash = inSeed;
	for (const char *c = inString; *c != 0; ++c)
	{
		hash ^= uint64_t(*c);
		hash *= 0x100000001b3UL;
	}
	return hash;
}

/// A 64 bit hash function by Thomas Wang, Jan 1997
/// See: http://web.archive.org/web/20071223173210/http://www.concentric.net/~Ttwang/tech/inthash.htm
/// @param inValue Value to hash
/// @return Hash
inline uint64_t Hash64(uint64_t inValue)
{
	uint64_t hash = inValue;
	hash = (~hash) + (hash << 21); // hash = (hash << 21) - hash - 1;
	hash = hash ^ (hash >> 24);
	hash = (hash + (hash << 3)) + (hash << 8); // hash * 265
	hash = hash ^ (hash >> 14);
	hash = (hash + (hash << 2)) + (hash << 4); // hash * 21
	hash = hash ^ (hash >> 28);
	hash = hash + (hash << 31);
	return hash;
}

/// Fallback hash function that calls T::GetHash()
template <class T>
struct Hash
{
	uint64_t		operator () (const T &inValue) const
	{
		return inValue.GetHash();
	}
};

/// A hash function for floats
template <>
struct Hash<float>
{
	uint64_t		operator () (float inValue) const
	{
		float value = inValue == 0.0f? 0.0f : inValue; // Convert -0.0f to 0.0f
		return HashBytes(&value, sizeof(value));
	}
};

/// A hash function for doubles
template <>
struct Hash<double>
{
	uint64_t		operator () (double inValue) const
	{
		double value = inValue == 0.0? 0.0 : inValue; // Convert -0.0 to 0.0
		return HashBytes(&value, sizeof(value));
	}
};

/// A hash function for character pointers
template <>
struct Hash<const char *>
{
	uint64_t		operator () (const char *inValue) const
	{
		return HashString(inValue);
	}
};

/// A hash function for std::string_view
template <>
struct Hash<std::string_view>
{
	uint64_t		operator () (const std::string_view &inValue) const
	{
		return HashBytes(inValue.data(), uint64_t(inValue.size()));
	}
};

/// A hash function for String
template <>
struct Hash<String>
{
	uint64_t		operator () (const String &inValue) const
	{
		return HashBytes(inValue.data(), uint64_t(inValue.size()));
	}
};

/// A fallback function for generic pointers
template <class T>
struct Hash<T *>
{
	uint64_t		operator () (T *inValue) const
	{
		return HashBytes(&inValue, sizeof(inValue));
	}
};

/// Helper macro to define a hash function for trivial types
#define MOSS_DEFINE_TRIVIAL_HASH(type)						\
template <>													\
struct Hash<type>											\
{															\
	uint64_t		operator () (const type &inValue) const		\
	{														\
		return HashBytes(&inValue, sizeof(inValue));		\
	}														\
};

/// Commonly used types
MOSS_DEFINE_TRIVIAL_HASH(char)
MOSS_DEFINE_TRIVIAL_HASH(int)
MOSS_DEFINE_TRIVIAL_HASH(uint32_t)
MOSS_DEFINE_TRIVIAL_HASH(uint64_t)

/// Helper function that hashes a single value into ioSeed
/// Based on https://github.com/jonmaiga/mx3 by Jon Maiga
template <typename T>
inline void HashCombine(uint64_t &ioSeed, const T &inValue)
{
	constexpr uint64_t c = 0xbea225f9eb34556dUL;

	uint64_t h = ioSeed;
	uint64_t x = Hash<T> { } (inValue);

	// See: https://github.com/jonmaiga/mx3/blob/master/mx3.h
	// mix_stream(h, x)
	x *= c;
	x ^= x >> 39;
	h += x * c;
	h *= c;

	// mix(h)
	h ^= h >> 32;
	h *= c;
	h ^= h >> 29;
	h *= c;
	h ^= h >> 32;
	h *= c;
	h ^= h >> 29;

	ioSeed = h;
}

/// Hash combiner to use a custom struct in an unordered map or set
///
/// Usage:
///
///		struct SomeHashKey
///		{
///			std::string key1;
///			std::string key2;
///			bool key3;
///		};
///
///		MOSS_MAKE_HASHABLE(SomeHashKey, t.key1, t.key2, t.key3)
template <typename FirstValue, typename... Values>
inline uint64_t HashCombineArgs(const FirstValue &inFirstValue, Values... inValues)
{
	// Prime the seed by hashing the first value
	uint64_t seed = Hash<FirstValue> { } (inFirstValue);

	// Hash all remaining values together using a fold expression
	(HashCombine(seed, inValues), ...);

	return seed;
}

#define MOSS_MAKE_HASH_STRUCT(type, name, ...)				\
	struct [[nodiscard]] name								\
	{														\
		uint64_t operator()(const type &t) const		\
		{													\
			return HashCombineArgs(__VA_ARGS__);		\
		}													\
	};

#define MOSS_MAKE_STD_HASH(type)								\
	MOSS_SUPPRESS_WARNING_PUSH								\
	MOSS_SUPPRESS_WARNINGS									\
	namespace std											\
	{														\
		template<>											\
		struct [[nodiscard]] hash<type>						\
		{													\
			size_t operator()(const type &t) const			\
			{												\
				return size_t(Hash<type>{ }(t));			\
			}												\
		};													\
	}														\
	MOSS_SUPPRESS_WARNING_POP

#define MOSS_MAKE_HASHABLE(type, ...)						\
	MOSS_SUPPRESS_WARNING_PUSH								\
	MOSS_SUPPRESS_WARNINGS									\
															\
	template<>												\
	MOSS_MAKE_HASH_STRUCT(type, Hash<type>, __VA_ARGS__) 	\
															\
	MOSS_SUPPRESS_WARNING_POP								\
	MOSS_MAKE_STD_HASH(type)

MOSS_SUPPRESS_WARNINGS_END
