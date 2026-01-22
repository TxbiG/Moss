// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Moss/Core/Profiler.h>
#include <Moss/Core/NonCopyable.h>

MOSS_SUPPRESS_WARNINGS_STD_BEGIN
#include <mutex>
#include <shared_mutex>
#include <thread>
MOSS_SUPPRESS_WARNINGS_STD_END

MOSS_SUPRESS_WARNINGS_BEGIN

// Things we're using from STL
using std::mutex;
using std::shared_mutex;
using std::thread;
using std::lock_guard;
using std::shared_lock;
using std::unique_lock;

#ifdef MOSS_PLATFORM_BLUE

// On Platform Blue the mutex class is not very fast so we implement it using the official APIs
class MutexBase : public NonCopyable
{
public:
					MutexBase()
	{
		MOSS_PLATFORM_BLUE_MUTEX_INIT(mMutex);
	}

					~MutexBase()
	{
		MOSS_PLATFORM_BLUE_MUTEX_DESTROY(mMutex);
	}

	inline bool		try_lock()
	{
		return MOSS_PLATFORM_BLUE_MUTEX_TRYLOCK(mMutex);
	}

	inline void		lock()
	{
		MOSS_PLATFORM_BLUE_MUTEX_LOCK(mMutex);
	}

	inline void		unlock()
	{
		MOSS_PLATFORM_BLUE_MUTEX_UNLOCK(mMutex);
	}

private:
	MOSS_PLATFORM_BLUE_MUTEX		mMutex;
};

// On Platform Blue the shared_mutex class is not very fast so we implement it using the official APIs
class SharedMutexBase : public NonCopyable
{
public:
					SharedMutexBase()
	{
		MOSS_PLATFORM_BLUE_RWLOCK_INIT(mRWLock);
	}

					~SharedMutexBase()
	{
		MOSS_PLATFORM_BLUE_RWLOCK_DESTROY(mRWLock);
	}

	inline bool		try_lock()
	{
		return MOSS_PLATFORM_BLUE_RWLOCK_TRYWLOCK(mRWLock);
	}

	inline bool		try_lock_shared()
	{
		return MOSS_PLATFORM_BLUE_RWLOCK_TRYRLOCK(mRWLock);
	}

	inline void		lock()
	{
		MOSS_PLATFORM_BLUE_RWLOCK_WLOCK(mRWLock);
	}

	inline void		unlock()
	{
		MOSS_PLATFORM_BLUE_RWLOCK_WUNLOCK(mRWLock);
	}

	inline void		lock_shared()
	{
		MOSS_PLATFORM_BLUE_RWLOCK_RLOCK(mRWLock);
	}

	inline void		unlock_shared()
	{
		MOSS_PLATFORM_BLUE_RWLOCK_RUNLOCK(mRWLock);
	}

private:
	MOSS_PLATFORM_BLUE_RWLOCK	mRWLock;
};

#else

// On other platforms just use the STL implementation
using MutexBase = mutex;
using SharedMutexBase = shared_mutex;

#endif // MOSS_PLATFORM_BLUE

#if defined(MOSS_DEBUG) || defined(MOSS_PROFILE_ENABLED) || defined(MOSS_EXTERNAL_PROFILE)

/// Very simple wrapper around MutexBase which tracks lock contention in the profiler
/// and asserts that locks/unlocks take place on the same thread
class Mutex : public MutexBase
{
public:
	inline bool		try_lock()
	{
		MOSS_ASSERT(mLockedThreadID != std::this_thread::get_id());
		if (MutexBase::try_lock())
		{
			MOSS_IF_ENABLE_ASSERTS(mLockedThreadID = std::this_thread::get_id();)
			return true;
		}
		return false;
	}

	inline void		lock()
	{
		if (!try_lock())
		{
			MOSS_PROFILE("Lock", 0xff00ffff);
			MutexBase::lock();
			MOSS_IF_ENABLE_ASSERTS(mLockedThreadID = std::this_thread::get_id();)
		}
	}

	inline void		unlock()
	{
		MOSS_ASSERT(mLockedThreadID == std::this_thread::get_id());
		MOSS_IF_ENABLE_ASSERTS(mLockedThreadID = thread::id();)
		MutexBase::unlock();
	}

#ifdef MOSS_DEBUG
	inline bool		is_locked()
	{
		return mLockedThreadID != thread::id();
	}
#endif // MOSS_DEBUG

private:
	MOSS_IF_ENABLE_ASSERTS(thread::id mLockedThreadID;)
};

/// Very simple wrapper around SharedMutexBase which tracks lock contention in the profiler
/// and asserts that locks/unlocks take place on the same thread
class SharedMutex : public SharedMutexBase
{
public:
	inline bool		try_lock()
	{
		MOSS_ASSERT(mLockedThreadID != std::this_thread::get_id());
		if (SharedMutexBase::try_lock())
		{
			MOSS_IF_ENABLE_ASSERTS(mLockedThreadID = std::this_thread::get_id();)
			return true;
		}
		return false;
	}

	inline void		lock()
	{
		if (!try_lock())
		{
			MOSS_PROFILE("WLock", 0xff00ffff);
			SharedMutexBase::lock();
			MOSS_IF_ENABLE_ASSERTS(mLockedThreadID = std::this_thread::get_id();)
		}
	}

	inline void		unlock()
	{
		MOSS_ASSERT(mLockedThreadID == std::this_thread::get_id());
		MOSS_IF_ENABLE_ASSERTS(mLockedThreadID = thread::id();)
		SharedMutexBase::unlock();
	}

#ifdef MOSS_DEBUG
	inline bool		is_locked()
	{
		return mLockedThreadID != thread::id();
	}
#endif // MOSS_DEBUG

	inline void		lock_shared()
	{
		if (!try_lock_shared())
		{
			MOSS_PROFILE("RLock", 0xff00ffff);
			SharedMutexBase::lock_shared();
		}
	}

private:
	MOSS_IF_ENABLE_ASSERTS(thread::id mLockedThreadID;)
};

#else

using Mutex = MutexBase;
using SharedMutex = SharedMutexBase;

#endif

MOSS_SUPRESS_WARNINGS_END
