// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <Moss/Core/Factory.h>

MOSS_SUPRESS_WARNINGS_BEGIN

Factory *Factory::sInstance = nullptr;

void *Factory::CreateObject(const char *inName)
{
	const RTTI *ci = Find(inName);
	return ci != nullptr? ci->CreateObject() : nullptr;
}

const RTTI *Factory::Find(const char *inName)
{
	ClassNameMap::iterator c = m_ClassNameMap.find(inName);
	return c != m_ClassNameMap.end()? c->second : nullptr;
}

const RTTI *Factory::Find(uint32 inHash)
{
	ClassHashMap::iterator c = m_ClassHashMap.find(inHash);
	return c != m_ClassHashMap.end()? c->second : nullptr;
}

bool Factory::Register(const RTTI *inRTTI)
{
	// Check if we already know the type
	if (Find(inRTTI->GetName()) != nullptr)
		return true;

	// Insert this class by name
	m_ClassNameMap.try_emplace(inRTTI->GetName(), inRTTI);

	// Insert this class by hash
	if (!m_ClassHashMap.try_emplace(inRTTI->GetHash(), inRTTI).second)
	{
		MOSS_ASSERT(false, "Hash collision registering type!");
		return false;
	}

	// Register base classes
	for (int i = 0; i < inRTTI->GetBaseClassCount(); ++i)
		if (!Register(inRTTI->GetBaseClass(i)))
			return false;

#ifdef MOSS_OBJECT_STREAM
	// Register attribute classes
	for (int i = 0; i < inRTTI->GetAttributeCount(); ++i)
	{
		const RTTI *rtti = inRTTI->GetAttribute(i).GetMemberPrimitiveType();
		if (rtti != nullptr && !Register(rtti))
			return false;
	}
#endif // MOSS_OBJECT_STREAM

	return true;
}

bool Factory::Register(const RTTI **inRTTIs, uint inNumber)
{
	m_ClassHashMap.reserve(m_ClassHashMap.size() + inNumber);
	m_ClassNameMap.reserve(m_ClassNameMap.size() + inNumber);

	for (const RTTI **rtti = inRTTIs; rtti < inRTTIs + inNumber; ++rtti)
		if (!Register(*rtti))
			return false;

	return true;
}

void Factory::Clear()
{
	m_ClassNameMap.clear();
	m_ClassHashMap.clear();
}

TArray<const RTTI *> Factory::GetAllClasses() const
{
	TArray<const RTTI *> all_classes;
	all_classes.reserve(m_ClassNameMap.size());
	for (const ClassNameMap::value_type &c : m_ClassNameMap)
		all_classes.push_back(c.second);
	return all_classes;
}

MOSS_SUPRESS_WARNINGS_END
