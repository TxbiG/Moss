// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

MOSS_SUPPRESS_WARNINGS_BEGIN

///////////////////////////////////////////////////////////////////////////////////
// LFHMAllocator
///////////////////////////////////////////////////////////////////////////////////

inline LFHMAllocator::~LFHMAllocator()
{
	AlignedFree(mObjectStore);
}

inline void LFHMAllocator::Init(uint32_t inObjectStoreSizeBytes)
{
	MOSS_ASSERT(mObjectStore == nullptr);

	mObjectStoreSizeBytes = inObjectStoreSizeBytes;
	mObjectStore = reinterpret_cast<uint8 *>(AlignedAllocate(inObjectStoreSizeBytes, 16));
}

inline void LFHMAllocator::Clear()
{
	mWriteOffset = 0;
}

inline void LFHMAllocator::Allocate(uint32_t inBlockSize, uint32_t &ioBegin, uint32_t &ioEnd)
{
	// If we're already beyond the end of our buffer then don't do an atomic add.
	// It's possible that many keys are inserted after the allocator is full, making it possible
	// for mWriteOffset (uint32_t) to wrap around to zero. When this happens, there will be a memory corruption.
	// This way, we will be able to progress the write offset beyond the size of the buffer
	// worst case by max <CPU count> * inBlockSize.
	if (mWriteOffset.load(memory_order_relaxed) >= mObjectStoreSizeBytes)
		return;

	// Atomically fetch a block from the pool
	uint32_t begin = mWriteOffset.fetch_add(inBlockSize, memory_order_relaxed);
	uint32_t end = min(begin + inBlockSize, mObjectStoreSizeBytes);

	if (ioEnd == begin)
	{
		// Block is allocated straight after our previous block
		begin = ioBegin;
	}
	else
	{
		// Block is a new block
		begin = min(begin, mObjectStoreSizeBytes);
	}

	// Store the begin and end of the resulting block
	ioBegin = begin;
	ioEnd = end;
}

template <class T>
inline uint32_t LFHMAllocator::ToOffset(const T *inData) const
{
	const uint8 *data = reinterpret_cast<const uint8 *>(inData);
	MOSS_ASSERT(data >= mObjectStore && data < mObjectStore + mObjectStoreSizeBytes);
	return uint32_t(data - mObjectStore);
}

template <class T>
inline T *LFHMAllocator::FromOffset(uint32_t inOffset) const
{
	MOSS_ASSERT(inOffset < mObjectStoreSizeBytes);
	return reinterpret_cast<T *>(mObjectStore + inOffset);
}

///////////////////////////////////////////////////////////////////////////////////
// LFHMAllocatorContext
///////////////////////////////////////////////////////////////////////////////////

inline LFHMAllocatorContext::LFHMAllocatorContext(LFHMAllocator &inAllocator, uint32_t inBlockSize) :
	mAllocator(inAllocator),
	mBlockSize(inBlockSize)
{
}

inline bool LFHMAllocatorContext::Allocate(uint32_t inSize, uint32_t inAlignment, uint32_t &outWriteOffset)
{
	// Calculate needed bytes for alignment
	MOSS_ASSERT(IsPowerOf2(inAlignment));
	uint32_t alignment_mask = inAlignment - 1;
	uint32_t alignment = (inAlignment - (mBegin & alignment_mask)) & alignment_mask;

	// Check if we have space
	if (mEnd - mBegin < inSize + alignment)
	{
		// Allocate a new block
		mAllocator.Allocate(mBlockSize, mBegin, mEnd);

		// Update alignment
		alignment = (inAlignment - (mBegin & alignment_mask)) & alignment_mask;

		// Check if we have space again
		if (mEnd - mBegin < inSize + alignment)
			return false;
	}

	// Make the allocation
	mBegin += alignment;
	outWriteOffset = mBegin;
	mBegin += inSize;
	return true;
}

///////////////////////////////////////////////////////////////////////////////////
// LockFreeHashMap
///////////////////////////////////////////////////////////////////////////////////

template <class Key, class Value>
void LockFreeHashMap<Key, Value>::Init(uint32_t inMaxBuckets)
{
	MOSS_ASSERT(inMaxBuckets >= 4 && IsPowerOf2(inMaxBuckets));
	MOSS_ASSERT(mBuckets == nullptr);

	mNumBuckets = inMaxBuckets;
	mMaxBuckets = inMaxBuckets;

	mBuckets = reinterpret_cast<atomic<uint32_t> *>(AlignedAllocate(inMaxBuckets * sizeof(atomic<uint32_t>), 16));

	Clear();
}

template <class Key, class Value>
LockFreeHashMap<Key, Value>::~LockFreeHashMap()
{
	AlignedFree(mBuckets);
}

template <class Key, class Value>
void LockFreeHashMap<Key, Value>::Clear()
{
#ifdef MOSS_DEBUG
	// Reset number of key value pairs
	mNumKeyValues = 0;
#endif // MOSS_DEBUG

	// Reset buckets 4 at a time
	static_assert(sizeof(atomic<uint32_t>) == sizeof(uint32_t));
	UVec4 invalid_handle = UVec4::Replicate(cInvalidHandle);
	uint32_t *start = reinterpret_cast<uint32_t *>(mBuckets);
	const uint32_t *end = start + mNumBuckets;
	MOSS_ASSERT(IsAligned(start, 16));
	while (start < end)
	{
		invalid_handle.StoreInt4Aligned(start);
		start += 4;
	}
}

template <class Key, class Value>
void LockFreeHashMap<Key, Value>::SetNumBuckets(uint32_t inNumBuckets)
{
	MOSS_ASSERT(mNumKeyValues == 0);
	MOSS_ASSERT(inNumBuckets <= mMaxBuckets);
	MOSS_ASSERT(inNumBuckets >= 4 && IsPowerOf2(inNumBuckets));

	mNumBuckets = inNumBuckets;
}

template <class Key, class Value>
template <class... Params>
inline typename LockFreeHashMap<Key, Value>::KeyValue *LockFreeHashMap<Key, Value>::Create(LFHMAllocatorContext &ioContext, const Key &inKey, uint64 inKeyHash, int inExtraBytes, Params &&... inConstructorParams)
{
	// This is not a multi map, test the key hasn't been inserted yet
	MOSS_ASSERT(Find(inKey, inKeyHash) == nullptr);

	// Calculate total size
	uint32_t size = sizeof(KeyValue) + inExtraBytes;

	// Get the write offset for this key value pair
	uint32_t write_offset;
	if (!ioContext.Allocate(size, alignof(KeyValue), write_offset))
		return nullptr;

#ifdef MOSS_DEBUG
	// Increment amount of entries in map
	mNumKeyValues.fetch_add(1, memory_order_relaxed);
#endif // MOSS_DEBUG

	// Construct the key/value pair
	KeyValue *kv = mAllocator.template FromOffset<KeyValue>(write_offset);
	MOSS_ASSERT(intptr_t(kv) % alignof(KeyValue) == 0);
#ifdef MOSS_DEBUG
	memset(kv, 0xcd, size);
#endif
	kv->mKey = inKey;
	new (&kv->mValue) Value(std::forward<Params>(inConstructorParams)...);

	// Get the offset to the first object from the bucket with corresponding hash
	atomic<uint32_t> &offset = mBuckets[inKeyHash & (mNumBuckets - 1)];

	// Add this entry as the first element in the linked list
	uint32_t old_offset = offset.load(memory_order_relaxed);
	for (;;)
	{
		kv->mNextOffset = old_offset;
		if (offset.compare_exchange_weak(old_offset, write_offset, memory_order_release))
			break;
	}

	return kv;
}

template <class Key, class Value>
inline const typename LockFreeHashMap<Key, Value>::KeyValue *LockFreeHashMap<Key, Value>::Find(const Key &inKey, uint64 inKeyHash) const
{
	// Get the offset to the keyvalue object from the bucket with corresponding hash
	uint32_t offset = mBuckets[inKeyHash & (mNumBuckets - 1)].load(memory_order_acquire);
	while (offset != cInvalidHandle)
	{
		// Loop through linked list of values until the right one is found
		const KeyValue *kv = mAllocator.template FromOffset<const KeyValue>(offset);
		if (kv->mKey == inKey)
			return kv;
		offset = kv->mNextOffset;
	}

	// Not found
	return nullptr;
}

template <class Key, class Value>
inline uint32_t LockFreeHashMap<Key, Value>::ToHandle(const KeyValue *inKeyValue) const
{
	return mAllocator.ToOffset(inKeyValue);
}

template <class Key, class Value>
inline const typename LockFreeHashMap<Key, Value>::KeyValue *LockFreeHashMap<Key, Value>::FromHandle(uint32_t inHandle) const
{
	return mAllocator.template FromOffset<const KeyValue>(inHandle);
}

template <class Key, class Value>
inline void LockFreeHashMap<Key, Value>::GetAllKeyValues(TArray<const KeyValue *> &outAll) const
{
	for (const atomic<uint32_t> *bucket = mBuckets; bucket < mBuckets + mNumBuckets; ++bucket)
	{
		uint32_t offset = *bucket;
		while (offset != cInvalidHandle)
		{
			const KeyValue *kv = mAllocator.template FromOffset<const KeyValue>(offset);
			outAll.push_back(kv);
			offset = kv->mNextOffset;
		}
	}
}

template <class Key, class Value>
typename LockFreeHashMap<Key, Value>::Iterator LockFreeHashMap<Key, Value>::begin()
{
	// Start with the first bucket
	Iterator it { this, 0, mBuckets[0] };

	// If it doesn't contain a valid entry, use the ++ operator to find the first valid entry
	if (it.mOffset == cInvalidHandle)
		++it;

	return it;
}

template <class Key, class Value>
typename LockFreeHashMap<Key, Value>::Iterator LockFreeHashMap<Key, Value>::end()
{
	return { this, mNumBuckets, cInvalidHandle };
}

template <class Key, class Value>
typename LockFreeHashMap<Key, Value>::KeyValue &LockFreeHashMap<Key, Value>::Iterator::operator* ()
{
	MOSS_ASSERT(mOffset != cInvalidHandle);

	return *mMap->mAllocator.template FromOffset<KeyValue>(mOffset);
}

template <class Key, class Value>
typename LockFreeHashMap<Key, Value>::Iterator &LockFreeHashMap<Key, Value>::Iterator::operator++ ()
{
	MOSS_ASSERT(mBucket < mMap->mNumBuckets);

	// Find the next key value in this bucket
	if (mOffset != cInvalidHandle)
	{
		const KeyValue *kv = mMap->mAllocator.template FromOffset<const KeyValue>(mOffset);
		mOffset = kv->mNextOffset;
		if (mOffset != cInvalidHandle)
			return *this;
	}

	// Loop over next buckets
	for (;;)
	{
		// Next bucket
		++mBucket;
		if (mBucket >= mMap->mNumBuckets)
			return *this;

		// Fetch the first entry in the bucket
		mOffset = mMap->mBuckets[mBucket];
		if (mOffset != cInvalidHandle)
			return *this;
	}
}

#ifdef MOSS_DEBUG

template <class Key, class Value>
void LockFreeHashMap<Key, Value>::TraceStats() const
{
	const int cMaxPerBucket = 256;

	int max_objects_per_bucket = 0;
	int num_objects = 0;
	int histogram[cMaxPerBucket];
	for (int i = 0; i < cMaxPerBucket; ++i)
		histogram[i] = 0;

	for (atomic<uint32_t> *bucket = mBuckets, *bucket_end = mBuckets + mNumBuckets; bucket < bucket_end; ++bucket)
	{
		int objects_in_bucket = 0;
		uint32_t offset = *bucket;
		while (offset != cInvalidHandle)
		{
			const KeyValue *kv = mAllocator.template FromOffset<const KeyValue>(offset);
			offset = kv->mNextOffset;
			++objects_in_bucket;
			++num_objects;
		}
		max_objects_per_bucket = max(objects_in_bucket, max_objects_per_bucket);
		histogram[min(objects_in_bucket, cMaxPerBucket - 1)]++;
	}

	MOSS_TRACE("max_objects_per_bucket = %d, num_buckets = %u, num_objects = %d", max_objects_per_bucket, mNumBuckets, num_objects);

	for (int i = 0; i < cMaxPerBucket; ++i)
		if (histogram[i] != 0)
			MOSS_TRACE("%d: %d", i, histogram[i]);
}

#endif

MOSS_SUPPRESS_WARNINGS_END
