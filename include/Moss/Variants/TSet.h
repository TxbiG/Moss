// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2024 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Moss/Core/HashTable.h>

/*
		TSet<> is an UnorderedSet
*/

MOSS_SUPRESS_WARNINGS_BEGIN

/// Internal helper class to provide context for UnorderedSet
template <class Key>
class TSetDetail
{
public:
	/// The key is the key, just return it
	static const Key &		sGetKey(const Key &inKey)
	{
		return inKey;
	}
};

/// Hash Set class
/// @tparam Key Key type
/// @tparam Hash Hash function (note should be 64-bits)
/// @tparam KeyEqual Equality comparison function
template <class Key, class Hash = Hash<Key>, class KeyEqual = std::equal_to<Key>>
class TSet : public HashTable<Key, Key, TSetDetail<Key>, Hash, KeyEqual> { };

MOSS_SUPRESS_WARNINGS_END
