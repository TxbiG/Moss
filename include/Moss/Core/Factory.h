// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Moss/Core/RTTI.h>
#include <Moss/Core/Variants/TMap.h>

MOSS_SUPRESS_WARNINGS_BEGIN

/// This class is responsible for creating instances of classes based on their name or hash and is mainly used for deserialization of saved data.
class MOSS_EXPORT Factory
{
public:
	MOSS_OVERRIDE_NEW_DELETE

	/// Create an object
	void *						CreateObject(const char *inName);

	/// Find type info for a specific class by name
	const RTTI *				Find(const char *inName);

	/// Find type info for a specific class by hash
	const RTTI *				Find(uint32 inHash);

	/// Register an object with the factory. Returns false on failure.
	bool						Register(const RTTI *inRTTI);

	/// Register a list of objects with the factory. Returns false on failure.
	bool						Register(const RTTI **inRTTIs, uint inNumber);

	/// Unregisters all types
	void						Clear();

	/// Get all registered classes
	TArray<const RTTI *>			GetAllClasses() const;

	/// Singleton factory instance
	static Factory *			sInstance;

private:
	using ClassNameMap = TMap<string_view, const RTTI *>;
	using ClassHashMap = TMap<uint32, const RTTI *>;

	ClassNameMap				m_ClassNameMap;			// Map of class names to type info
	ClassHashMap				m_ClassHashMap;			// Map of class hash to type info
};

MOSS_SUPRESS_WARNINGS_END
