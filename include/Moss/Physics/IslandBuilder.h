// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Moss/Physics/Body/Body.h>
#include <Moss/Core/NonCopyable.h>
#include <Moss/Moss_stdinc.h>

MOSS_SUPPRESS_WARNINGS_BEGIN

class TempAllocator;

// Keeps track of connected bodies and builds islands for multithreaded velocity/position update
class MOSS_API IslandBuilder : public NonCopyable
{
public:
	// Destructor
							~IslandBuilder();

	// Initialize the island builder with the maximum amount of bodies that could be active
	void					Init(uint32_t inMaxActiveBodies);

	// Prepare for simulation step by allocating space for the contact constraints
	void					PrepareContactConstraints(uint32_t inMaxContactConstraints, TempAllocator *inTempAllocator);

	// Prepare for simulation step by allocating space for the non-contact constraints
	void					PrepareNonContactConstraints(uint32_t inNumConstraints, TempAllocator *inTempAllocator);

	// Link two bodies by their index in the BodyManager::mActiveBodies list to form islands
	void					LinkBodies(uint32_t inFirst, uint32_t inSecond);

	// Link a constraint to a body by their index in the BodyManager::mActiveBodies
	void					LinkConstraint(uint32_t inConstraintIndex, uint32_t inIndexInActiveBodyList);

	// Link a contact to a body by their index in the BodyManager::mActiveBodies
	void					LinkContact(uint32_t inContactIndex, uint32_t inIndexInActiveBodyList);

	// Finalize the islands after all bodies have been Link()-ed
	void					Finalize(const BodyID *inActiveBodies, uint32_t inNumActiveBodies, uint32_t inNumContacts, TempAllocator *inTempAllocator);

	// Get the amount of islands formed
	uint32_t					GetNumIslands() const							{ return mNumIslands; }

	// Get iterator for a particular island, return false if there are no constraints
	void					GetBodiesInIsland(uint32_t inIslandIndex, BodyID *&outBodiesBegin, BodyID *&outBodiesEnd) const;
	bool					GetConstraintsInIsland(uint32_t inIslandIndex, uint32_t *&outConstraintsBegin, uint32_t *&outConstraintsEnd) const;
	bool					GetContactsInIsland(uint32_t inIslandIndex, uint32_t *&outContactsBegin, uint32_t *&outContactsEnd) const;

	// The number of position iterations for each island
	void					SetNumPositionSteps(uint32_t inIslandIndex, uint inNumPositionSteps)	{ MOSS_ASSERT(inIslandIndex < mNumIslands); MOSS_ASSERT(inNumPositionSteps < 256); mNumPositionSteps[inIslandIndex] = uint8(inNumPositionSteps); }
	uint					GetNumPositionSteps(uint32_t inIslandIndex) const						{ MOSS_ASSERT(inIslandIndex < mNumIslands); return mNumPositionSteps[inIslandIndex]; }

#ifdef MOSS_TRACK_SIMULATION_STATS
	struct IslandStats
	{
		atomic<uint64>		mVelocityConstraintTicks = 0;
		atomic<uint64>		mPositionConstraintTicks = 0;
		atomic<uint64>		mUpdateBoundsTicks = 0;
		uint8				mNumVelocitySteps = 0;
		uint8				mNumPositionSteps = 0;												// Tracking this a 2nd time since IslandBuilder::mNumPositionSteps is not filled in when there are no constraints or for large islands.
		bool				mIsLargeIsland = false;
	};

	// Tracks simulation stats per island
	IslandStats &			GetIslandStats(uint32_t inIslandIndex)								{ return mIslandStats[inIslandIndex]; }
#endif

	// After you're done calling the three functions above, call this function to free associated data
	void					ResetIslands(TempAllocator *inTempAllocator);

private:
	// Returns the index of the lowest body in the group
	uint32_t					GetLowestBodyIndex(uint32_t inActiveBodyIndex) const;

#ifdef MOSS_VALIDATE_ISLAND_BUILDER
	// Helper function to validate all islands so far generated
	void					ValidateIslands(uint32_t inNumActiveBodies) const;
#endif

	// Helper functions to build various islands
	void					BuildBodyIslands(const BodyID *inActiveBodies, uint32_t inNumActiveBodies, TempAllocator *inTempAllocator);
	void					BuildConstraintIslands(const uint32_t *inConstraintToBody, uint32_t inNumConstraints, uint32_t *&outConstraints, uint32_t *&outConstraintsEnd, TempAllocator *inTempAllocator) const;

	// Sorts the islands so that the islands with most constraints go first
	void					SortIslands(TempAllocator *inTempAllocator);

	// Intermediate data structure that for each body keeps track what the lowest index of the body is that it is connected to
	struct BodyLink
	{
		MOSS_OVERRIDE_NEW_DELETE

		atomic<uint32_t>		mLinkedTo;										// An index in mBodyLinks pointing to another body in this island with a lower index than this body
		uint32_t				mIslandIndex;									// The island index of this body (filled in during Finalize)
	};

	// Intermediate data
	BodyLink *				mBodyLinks = nullptr;							// Maps bodies to the first body in the island
	uint32_t *				mConstraintLinks = nullptr;						// Maps constraint index to body index (which maps to island index)
	uint32_t *				mContactLinks = nullptr;						// Maps contact constraint index to body index (which maps to island index)

	// Final data
	BodyID *				mBodyIslands = nullptr;							// Bodies ordered by island
	uint32_t *				mBodyIslandEnds = nullptr;						// End index of each body island

	uint32_t *				mConstraintIslands = nullptr;					// Constraints ordered by island
	uint32_t *				mConstraintIslandEnds = nullptr;				// End index of each constraint island

	uint32_t *				mContactIslands = nullptr;						// Contacts ordered by island
	uint32_t *				mContactIslandEnds = nullptr;					// End index of each contact island

	uint32_t *				mIslandsSorted = nullptr;						// A list of island indices in order of most constraints first

	uint8 *					mNumPositionSteps = nullptr;					// Number of position steps for each island

#ifdef MOSS_TRACK_SIMULATION_STATS
	IslandStats *			mIslandStats = nullptr;							// Per island statistics
#endif

	// Counters
	uint32_t					mMaxActiveBodies;								// Maximum size of the active bodies list (see BodyManager::mActiveBodies)
	uint32_t					mNumActiveBodies = 0;							// Number of active bodies passed to
	uint32_t					mNumConstraints = 0;							// Size of the constraint list (see ConstraintManager::mConstraints)
	uint32_t					mMaxContacts = 0;								// Maximum amount of contacts supported
	uint32_t					mNumContacts = 0;								// Size of the contacts list (see ContactConstraintManager::mNumConstraints)
	uint32_t					mNumIslands = 0;								// Final number of islands

#ifdef MOSS_VALIDATE_ISLAND_BUILDER
	// Structure to keep track of all added links to validate that islands were generated correctly
	struct LinkValidation
	{
		uint32_t				mFirst;
		uint32_t				mSecond;
	};

	LinkValidation*		mLinkValidation = nullptr;
	atomic<uint32_t>			mNumLinkValidation;
#endif
};

MOSS_SUPPRESS_WARNINGS_END
