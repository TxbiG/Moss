#pragma once

#include <Moss/Core/HashCombine.h>

MOSS_SUPPRESS_WARNINGS_BEGIN

class BodyID {
public:
	MOSS_OVERRIDE_NEW_DELETE

	static constexpr uint32_t	cInvalidBodyID = 0xffffffff;	// The value for an invalid body ID
	static constexpr uint32_t	cBroadPhaseBit = 0x80000000;	// This bit is used by the broadphase
	static constexpr uint32_t	cMaxBodyIndex = 0x7fffff;		// Maximum value for body index (also the maximum amount of bodies supported - 1)
	static constexpr uint8_t	cMaxSequenceNumber = 0xff;		// Maximum value for the sequence number
	static constexpr uint32_t cSequenceNumberShift = 23;		// Number of bits to shift to get the sequence number

	// Construct invalid body ID
	BodyID() : mID(cInvalidBodyID) { }

	// Construct from index and sequence number combined in a single uint32_t (use with care!)
	explicit BodyID(uint32_t inID) : mID(inID) { MOSS_ASSERT((inID & cBroadPhaseBit) == 0 || inID == cInvalidBodyID); } // Check bit used by broadphase

	// Construct from index and sequence number
	explicit BodyID(uint32_t inID, uint8_t inSequenceNumber) : mID((uint32_t(inSequenceNumber) << cSequenceNumberShift) | inID) { MOSS_ASSERT(inID <= cMaxBodyIndex); } // Should not overlap with broadphase bit or sequence number

	// Get index in body array
	inline uint32_t GetIndex() const { return mID & cMaxBodyIndex; }

	// Get sequence number of body.
	// The sequence number can be used to check if a body ID with the same body index has been reused by another body.
	// It is mainly used in multi threaded situations where a body is removed and its body index is immediately reused by a body created from another thread.
	// Functions querying the broadphase can (after acquiring a body lock) detect that the body has been removed (we assume that this won't happen more than 128 times in a row).
	inline uint8_t GetSequenceNumber() const { return uint8_t(mID >> cSequenceNumberShift); }

	// Returns the index and sequence number combined in an uint32_t
	inline uint32_t GetIndexAndSequenceNumber() const { return mID; }

	// Check if the ID is valid
	inline bool IsInvalid() const { return mID == cInvalidBodyID; }

	// Equals check
	inline bool operator == (const BodyID &inRHS) const { return mID == inRHS.mID; }

	// Not equals check
	inline bool operator != (const BodyID &inRHS) const { return mID != inRHS.mID; }

	// Smaller than operator, can be used for sorting bodies
	inline bool operator < (const BodyID &inRHS) const { return mID < inRHS.mID; }
	// Greater than operator, can be used for sorting bodies
	inline bool operator > (const BodyID &inRHS) const { return mID > inRHS.mID; }

private:
	uint32_t					mID;
};