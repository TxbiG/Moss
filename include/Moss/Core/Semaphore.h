// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2023 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Moss/Moss_stdinc.h>

// Determine which platform specific construct we'll use
MOSS_SUPPRESS_WARNINGS_STD_BEGIN
#ifdef MOSS_PLATFORM_WINDOWS
	// We include windows.h in the cpp file, the semaphore itself is a void pointer
#elif defined(MOSS_PLATFORM_LINUX) || defined(MOSS_PLATFORM_ANDROID) || defined(MOSS_PLATFORM_BSD) || defined(MOSS_PLATFORM_WASM)
	#include <semaphore.h>
	#define MOSS_USE_PTHREADS
#elif defined(MOSS_PLATFORM_MACOS) || defined(MOSS_PLATFORM_IOS)
	#include <dispatch/dispatch.h>
	#define MOSS_USE_GRAND_CENTRAL_DISPATCH
#elif defined(MOSS_PLATFORM_BLUE)
	// Jolt/Core/PlatformBlue.h should have defined everything that is needed below
#else
	#include <mutex>
	#include <condition_variable>
#endif
MOSS_SUPPRESS_WARNINGS_STD_END

MOSS_SUPRESS_WARNINGS_BEGIN

/// Implements a semaphore
/// When we switch to C++20 we can use counting_semaphore to unify this
class MOSS_EXPORT Semaphore
{
public:
	/// Constructor
							Semaphore();
							~Semaphore();

	/// Release the semaphore, signaling the thread waiting on the barrier that there may be work
	void					Release(uint inNumber = 1);

	/// Acquire the semaphore inNumber times
	void					Acquire(uint inNumber = 1);

	/// Get the current value of the semaphore
	inline int				GetValue() const								{ return mCount.load(std::memory_order_relaxed); }

private:
#if defined(MOSS_PLATFORM_WINDOWS) || defined(MOSS_USE_PTHREADS) || defined(MOSS_USE_GRAND_CENTRAL_DISPATCH) || defined(MOSS_PLATFORM_BLUE)
#ifdef MOSS_PLATFORM_WINDOWS
	using SemaphoreType = void *;
#elif defined(MOSS_USE_PTHREADS)
	using SemaphoreType = sem_t;
#elif defined(MOSS_USE_GRAND_CENTRAL_DISPATCH)
	using SemaphoreType = dispatch_semaphore_t;
#elif defined(MOSS_PLATFORM_BLUE)
	using SemaphoreType = MOSS_PLATFORM_BLUE_SEMAPHORE;
#endif
	alignas(MOSS_CACHE_LINE_SIZE) atomic<int> mCount { 0 };					///< We increment mCount for every release, to acquire we decrement the count. If the count is negative we know that we are waiting on the actual semaphore.
	SemaphoreType			mSemaphore { };									///< The semaphore is an expensive construct so we only acquire/release it if we know that we need to wait/have waiting threads
#else
	// Other platforms: Emulate a semaphore using a mutex, condition variable and count
	std::mutex				mLock;
	std::condition_variable	mWaitVariable;
	atomic<int>				mCount { 0 };
#endif
};

MOSS_SUPRESS_WARNINGS_END
