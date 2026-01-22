// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2023 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Moss/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <Moss/Physics/Collision/ObjectLayerPairFilterMask.h>

MOSS_NAMESPACE_BEGIN

/// Class that determines if an object layer can collide with a broadphase layer.
/// This implementation works together with BroadPhaseLayerInterfaceMask and ObjectLayerPairFilterMask
class ObjectVsBroadPhaseLayerFilterMask : public ObjectVsBroadPhaseLayerFilter
{
public:
	MOSS_OVERRIDE_NEW_DELETE

/// Constructor
					ObjectVsBroadPhaseLayerFilterMask(const BroadPhaseLayerInterfaceMask &inBroadPhaseLayerInterface) :
		mBroadPhaseLayerInterface(inBroadPhaseLayerInterface)
	{
	}

	/// Returns true if an object layer should collide with a broadphase layer
	virtual bool	ShouldCollide(ObjectLayer inLayer1, BroadPhaseLayer inLayer2) const override
	{
		// Just defer to BroadPhaseLayerInterface
		return mBroadPhaseLayerInterface.ShouldCollide(inLayer1, inLayer2);
	}

private:
	const BroadPhaseLayerInterfaceMask &mBroadPhaseLayerInterface;
};

/// BroadPhaseLayerInterface implementation.
/// This defines a mapping between object and broadphase layers.
/// This implementation works together with ObjectLayerPairFilterMask and ObjectVsBroadPhaseLayerFilterMask.
/// A broadphase layer is suitable for an object if its group & inGroupsToInclude is not zero and its group & inGroupsToExclude is zero.
/// The broadphase layers are iterated from lowest to highest value and the first one that matches is taken. If none match then it takes the last layer.
class BroadPhaseLayerInterfaceMask : public BroadPhaseLayerInterface {
public:
	MOSS_OVERRIDE_NEW_DELETE

	explicit				BroadPhaseLayerInterfaceMask(uint inNumBroadPhaseLayers)
	{
		MOSS_ASSERT(inNumBroadPhaseLayers > 0);
		mMapping.resize(inNumBroadPhaseLayers);

#if defined(MOSS_EXTERNAL_PROFILE) || defined(MOSS_PROFILE_ENABLED)
		mBroadPhaseLayerNames.resize(inNumBroadPhaseLayers, "Undefined");
#endif // MOSS_EXTERNAL_PROFILE || MOSS_PROFILE_ENABLED
	}

	// Configures a broadphase layer.
	void					ConfigureLayer(BroadPhaseLayer inBroadPhaseLayer, uint32 inGroupsToInclude, uint32 inGroupsToExclude)
	{
		MOSS_ASSERT((BroadPhaseLayer::Type)inBroadPhaseLayer < (uint)mMapping.size());
		Mapping &m = mMapping[(BroadPhaseLayer::Type)inBroadPhaseLayer];
		m.mGroupsToInclude = inGroupsToInclude;
		m.mGroupsToExclude = inGroupsToExclude;
	}

	virtual uint			GetNumBroadPhaseLayers() const override
	{
		return (uint)mMapping.size();
	}

	virtual BroadPhaseLayer	GetBroadPhaseLayer(ObjectLayer inLayer) const override
	{
		// Try to find the first broadphase layer that matches
		uint32 group = ObjectLayerPairFilterMask::sGetGroup(inLayer);
		for (const Mapping &m : mMapping)
			if ((group & m.mGroupsToInclude) != 0 && (group & m.mGroupsToExclude) == 0)
				return BroadPhaseLayer(BroadPhaseLayer::Type(&m - mMapping.data()));

		// Fall back to the last broadphase layer
		return BroadPhaseLayer(BroadPhaseLayer::Type(mMapping.size() - 1));
	}

	/// Returns true if an object layer should collide with a broadphase layer, this function is being called from ObjectVsBroadPhaseLayerFilterMask
	inline bool				ShouldCollide(ObjectLayer inLayer1, BroadPhaseLayer inLayer2) const
	{
		uint32 mask = ObjectLayerPairFilterMask::sGetMask(inLayer1);
		const Mapping &m = mMapping[(BroadPhaseLayer::Type)inLayer2];
		return &m == &mMapping.back() // Last layer may collide with anything
			|| (m.mGroupsToInclude & mask) != 0; // Mask allows it to collide with objects that could reside in this layer
	}

#if defined(MOSS_EXTERNAL_PROFILE) || defined(MOSS_PROFILE_ENABLED)
	void					SetBroadPhaseLayerName(BroadPhaseLayer inLayer, const char *inName)
	{
		mBroadPhaseLayerNames[(BroadPhaseLayer::Type)inLayer] = inName;
	}

	virtual const char *	GetBroadPhaseLayerName(BroadPhaseLayer inLayer) const override
	{
		return mBroadPhaseLayerNames[(BroadPhaseLayer::Type)inLayer];
	}
#endif // MOSS_EXTERNAL_PROFILE || MOSS_PROFILE_ENABLED

private:
	struct Mapping
	{
		uint32				mGroupsToInclude = 0;
		uint32				mGroupsToExclude = ~uint32(0);
	};
	TArray<Mapping>			mMapping;

#if defined(MOSS_EXTERNAL_PROFILE) || defined(MOSS_PROFILE_ENABLED)
	TArray<const char *>		mBroadPhaseLayerNames;
#endif // MOSS_EXTERNAL_PROFILE || MOSS_PROFILE_ENABLED
};


class BroadPhaseLayerInterfaceTable : public BroadPhaseLayerInterface {
public:
	MOSS_OVERRIDE_NEW_DELETE

							BroadPhaseLayerInterfaceTable(uint inNumObjectLayers, uint inNumBroadPhaseLayers) :
		mNumBroadPhaseLayers(inNumBroadPhaseLayers)
	{
		mObjectToBroadPhase.resize(inNumObjectLayers, BroadPhaseLayer(0));
#if defined(MOSS_EXTERNAL_PROFILE) || defined(MOSS_PROFILE_ENABLED)
		mBroadPhaseLayerNames.resize(inNumBroadPhaseLayers, "Undefined");
#endif // MOSS_EXTERNAL_PROFILE || MOSS_PROFILE_ENABLED
	}

	void					MapObjectToBroadPhaseLayer(ObjectLayer inObjectLayer, BroadPhaseLayer inBroadPhaseLayer)
	{
		MOSS_ASSERT((BroadPhaseLayer::Type)inBroadPhaseLayer < mNumBroadPhaseLayers);
		mObjectToBroadPhase[inObjectLayer] = inBroadPhaseLayer;
	}

	virtual uint			GetNumBroadPhaseLayers() const override
	{
		return mNumBroadPhaseLayers;
	}

	virtual BroadPhaseLayer	GetBroadPhaseLayer(ObjectLayer inLayer) const override
	{
		return mObjectToBroadPhase[inLayer];
	}

#if defined(MOSS_EXTERNAL_PROFILE) || defined(MOSS_PROFILE_ENABLED)
	void					SetBroadPhaseLayerName(BroadPhaseLayer inLayer, const char *inName)
	{
		mBroadPhaseLayerNames[(BroadPhaseLayer::Type)inLayer] = inName;
	}

	virtual const char *	GetBroadPhaseLayerName(BroadPhaseLayer inLayer) const override
	{
		return mBroadPhaseLayerNames[(BroadPhaseLayer::Type)inLayer];
	}
#endif // MOSS_EXTERNAL_PROFILE || MOSS_PROFILE_ENABLED

private:
	uint					mNumBroadPhaseLayers;
	TArray<BroadPhaseLayer>	mObjectToBroadPhase;
#if defined(MOSS_EXTERNAL_PROFILE) || defined(MOSS_PROFILE_ENABLED)
	TArray<const char *>		mBroadPhaseLayerNames;
#endif // MOSS_EXTERNAL_PROFILE || MOSS_PROFILE_ENABLED
};


/// Class that determines if an object layer can collide with a broadphase layer.
/// This implementation uses a table and constructs itself from an ObjectLayerPairFilter and a BroadPhaseLayerInterface.
class ObjectVsBroadPhaseLayerFilterTable : public ObjectVsBroadPhaseLayerFilter
{
private:
	/// Get which bit corresponds to the pair (inLayer1, inLayer2)
	uint					GetBit(ObjectLayer inLayer1, BroadPhaseLayer inLayer2) const
	{
		// Calculate at which bit the entry for this pair resides
		return inLayer1 * mNumBroadPhaseLayers + (BroadPhaseLayer::Type)inLayer2;
	}

public:
	MOSS_OVERRIDE_NEW_DELETE

	/// Construct the table
	/// @param inBroadPhaseLayerInterface The broad phase layer interface that maps object layers to broad phase layers
	/// @param inNumBroadPhaseLayers Number of broad phase layers
	/// @param inObjectLayerPairFilter The object layer pair filter that determines which object layers can collide
	/// @param inNumObjectLayers Number of object layers
							ObjectVsBroadPhaseLayerFilterTable(const BroadPhaseLayerInterface &inBroadPhaseLayerInterface, uint inNumBroadPhaseLayers, const ObjectLayerPairFilter &inObjectLayerPairFilter, uint inNumObjectLayers) :
		mNumBroadPhaseLayers(inNumBroadPhaseLayers)
	{
		// Resize table and set all entries to false
		mTable.resize((inNumBroadPhaseLayers * inNumObjectLayers + 7) / 8, 0);

		// Loop over all object layer pairs
		for (ObjectLayer o1 = 0; o1 < inNumObjectLayers; ++o1)
			for (ObjectLayer o2 = 0; o2 < inNumObjectLayers; ++o2)
			{
				// Get the broad phase layer for the second object layer
				BroadPhaseLayer b2 = inBroadPhaseLayerInterface.GetBroadPhaseLayer(o2);
				MOSS_ASSERT((BroadPhaseLayer::Type)b2 < inNumBroadPhaseLayers);

				// If the object layers collide then so should the object and broadphase layer
				if (inObjectLayerPairFilter.ShouldCollide(o1, o2))
				{
					uint bit = GetBit(o1, b2);
					mTable[bit >> 3] |= 1 << (bit & 0b111);
				}
			}
	}

	/// Returns true if an object layer should collide with a broadphase layer
	virtual bool			ShouldCollide(ObjectLayer inLayer1, BroadPhaseLayer inLayer2) const override
	{
		uint bit = GetBit(inLayer1, inLayer2);
		return (mTable[bit >> 3] & (1 << (bit & 0b111))) != 0;
	}

private:
	uint					mNumBroadPhaseLayers;						///< The total number of broadphase layers
	TArray<uint8>			mTable;										///< The table of bits that indicates which layers collide
};

MOSS_NAMESPACE_END
