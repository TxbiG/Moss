// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

MOSS_SUPPRESS_WARNINGS_STD_BEGIN
#include <mutex>
#include <chrono>
MOSS_SUPPRESS_WARNINGS_STD_END

#include <Moss/Core/NonCopyable.h>
#include <Moss/Core/TickCounter.h>
#include <Moss/Core/Variants/TMap.h>

#if defined(MOSS_EXTERNAL_PROFILE)

MOSS_SUPRESS_WARNINGS_BEGIN

#ifdef MOSS_SHARED_LIBRARY
/// Functions called when a profiler measurement starts or stops, need to be overridden by the user.
using ProfileStartMeasurementFunction = void (*)(const char *inName, uint32 inColor, uint8 *ioUserData);
using ProfileEndMeasurementFunction = void (*)(uint8 *ioUserData);

MOSS_EXPORT extern ProfileStartMeasurementFunction ProfileStartMeasurement;
MOSS_EXPORT extern ProfileEndMeasurementFunction ProfileEndMeasurement;
#endif // MOSS_SHARED_LIBRARY

/// Create this class on the stack to start sampling timing information of a particular scope.
///
/// For statically linked builds, this is left unimplemented intentionally. Needs to be implemented by the user of the library.
/// On construction a measurement should start, on destruction it should be stopped.
/// For dynamically linked builds, the user should override the ProfileStartMeasurement and ProfileEndMeasurement functions.
class alignas(16) ExternalProfileMeasurement : public NonCopyable
{
public:
	/// Constructor
#ifdef MOSS_SHARED_LIBRARY
	MOSS_INLINE						ExternalProfileMeasurement(const char *inName, uint32 inColor = 0) { ProfileStartMeasurement(inName, inColor, mUserData); }
	MOSS_INLINE						~ExternalProfileMeasurement() { ProfileEndMeasurement(mUserData); }
#else
									ExternalProfileMeasurement(const char *inName, uint32 inColor = 0);
									~ExternalProfileMeasurement();
#endif

private:
	uint8							mUserData[64];
};

MOSS_SUPRESS_WARNINGS_END

//////////////////////////////////////////////////////////////////////////////////////////
// Macros to do the actual profiling
//////////////////////////////////////////////////////////////////////////////////////////

MOSS_SUPPRESS_WARNING_PUSH
MOSS_CLANG_SUPPRESS_WARNING("-Wc++98-compat-pedantic")

// Dummy implementations
#define MOSS_PROFILE_START(name)
#define MOSS_PROFILE_END()
#define MOSS_PROFILE_THREAD_START(name)
#define MOSS_PROFILE_THREAD_END()
#define MOSS_PROFILE_NEXTFRAME()
#define MOSS_PROFILE_DUMP(...)

// Scope profiling measurement
#define MOSS_PROFILE_TAG2(line)		profile##line
#define MOSS_PROFILE_TAG(line)		MOSS_PROFILE_TAG2(line)

/// Macro to collect profiling information.
///
/// Usage:
///
///		{
///			MOSS_PROFILE("Operation");
///			do operation;
///		}
///
#define MOSS_PROFILE(...)			ExternalProfileMeasurement MOSS_PROFILE_TAG(__LINE__)(__VA_ARGS__)

// Scope profiling for function
#define MOSS_PROFILE_FUNCTION()		MOSS_PROFILE(MOSS_FUNCTION_NAME)

MOSS_SUPPRESS_WARNING_POP

#elif defined(MOSS_PROFILE_ENABLED)

MOSS_SUPRESS_WARNINGS_BEGIN

class ProfileSample;
class ProfileThread;

/// Singleton class for managing profiling information
class MOSS_EXPORT Profiler : public NonCopyable
{
public:
	MOSS_OVERRIDE_NEW_DELETE

	/// Constructor
								Profiler()															{ UpdateReferenceTime(); }

	/// Increments the frame counter to provide statistics per frame
	void						NextFrame();

	/// Dump profiling statistics at the start of the next frame
	/// @param inTag If not empty, this overrides the auto incrementing number in the filename of the dump file
	void						Dump(const string_view &inTag = string_view());

	/// Add a thread to be instrumented
	void						AddThread(ProfileThread *inThread);

	/// Remove a thread from being instrumented
	void						RemoveThread(ProfileThread *inThread);

	/// Singleton instance
	static Profiler *			sInstance;

private:
	/// Helper class to freeze ProfileSamples per thread while processing them
	struct ThreadSamples
	{
		String					mThreadName;
		ProfileSample *			mSamplesBegin;
		ProfileSample *			mSamplesEnd;
	};

	/// Helper class to aggregate ProfileSamples
	class Aggregator
	{
	public:
		/// Constructor
								Aggregator(const char *inName)										: mName(inName) { }

		/// Accumulate results for a measurement
		void					AccumulateMeasurement(uint64 inCyclesInCallWithChildren)
		{
			mCallCounter++;
			mTotalCyclesInCallWithChildren += inCyclesInCallWithChildren;
			mMinCyclesInCallWithChildren = min(inCyclesInCallWithChildren, mMinCyclesInCallWithChildren);
			mMaxCyclesInCallWithChildren = max(inCyclesInCallWithChildren, mMaxCyclesInCallWithChildren);
		}

		/// Sort descending by total cycles
		bool					operator < (const Aggregator &inRHS) const
		{
			return mTotalCyclesInCallWithChildren > inRHS.mTotalCyclesInCallWithChildren;
		}

		/// Identification
		const char *			mName;																///< User defined name of this item

		/// Statistics
		uint32					mCallCounter = 0;													///< Number of times AccumulateMeasurement was called
		uint64					mTotalCyclesInCallWithChildren = 0;									///< Total amount of cycles spent in this scope
		uint64					mMinCyclesInCallWithChildren = 0xffffffffffffffffUL;				///< Minimum amount of cycles spent per call
		uint64					mMaxCyclesInCallWithChildren = 0;									///< Maximum amount of cycles spent per call
	};

	using Threads = TArray<ThreadSamples>;
	using Aggregators = TArray<Aggregator>;
	using KeyToAggregator = TMap<const char *, size_t>;

	/// Helper function to aggregate profile sample data
	static void					sAggregate(int inDepth, uint32 inColor, ProfileSample *&ioSample, const ProfileSample *inEnd, Aggregators &ioAggregators, KeyToAggregator &ioKeyToAggregator);

	/// We measure the amount of ticks per second, this function resets the reference time point
	void						UpdateReferenceTime();

	/// Get the amount of ticks per second, note that this number will never be fully accurate as the amount of ticks per second may vary with CPU load, so this number is only to be used to give an indication of time for profiling purposes
	uint64						GetProcessorTicksPerSecond() const;

	/// Dump profiling statistics
	void						DumpInternal();
	void						DumpChart(const char *inTag, const Threads &inThreads, const KeyToAggregator &inKeyToAggregators, const Aggregators &inAggregators);

	std::mutex					mLock;																///< Lock that protects mThreads
	uint64						mReferenceTick;														///< Tick count at the start of the frame
	std::chrono::high_resolution_clock::time_point mReferenceTime;									///< Time at the start of the frame
	TArray<ProfileThread *>		mThreads;															///< List of all active threads
	bool						mDump = false;														///< When true, the samples are dumped next frame
	String						mDumpTag;															///< When not empty, this overrides the auto incrementing number of the dump filename
};

// Class that contains the information of a single scoped measurement
class alignas(16) MOSS_EXPORT_GCC_BUG_WORKAROUND ProfileSample : public NonCopyable
{
public:
	MOSS_OVERRIDE_NEW_DELETE

	const char *				mName;																///< User defined name of this item
	uint32						mColor;																///< Color to use for this sample
	uint8						mDepth;																///< Calculated depth
	uint8						mUnused[3];
	uint64						mStartCycle;														///< Cycle counter at start of measurement
	uint64						mEndCycle;															///< Cycle counter at end of measurement
};

/// Collects all samples of a single thread
class ProfileThread : public NonCopyable
{
public:
	MOSS_OVERRIDE_NEW_DELETE

	/// Constructor
	inline						ProfileThread(const string_view &inThreadName);
	inline						~ProfileThread();

	static const uint cMaxSamples = 65536;

	String						mThreadName;														///< Name of the thread that we're collecting information for
	ProfileSample				mSamples[cMaxSamples];												///< Buffer of samples
	uint						mCurrentSample = 0;													///< Next position to write a sample to

#ifdef MOSS_SHARED_LIBRARY
	MOSS_EXPORT static void		sSetInstance(ProfileThread *inInstance);
	MOSS_EXPORT static ProfileThread *sGetInstance();
#else
	static inline void			sSetInstance(ProfileThread *inInstance)								{ sInstance = inInstance; }
	static inline ProfileThread *sGetInstance()														{ return sInstance; }

private:
	static thread_local ProfileThread *sInstance;
#endif
};

/// Create this class on the stack to start sampling timing information of a particular scope
class MOSS_EXPORT ProfileMeasurement : public NonCopyable
{
public:
	/// Constructor
	inline						ProfileMeasurement(const char *inName, uint32 inColor = 0);
	inline						~ProfileMeasurement();

private:
	ProfileSample *				mSample;
	ProfileSample				mTemp;

	static bool					sOutOfSamplesReported;
};

MOSS_SUPRESS_WARNINGS_END

#include "Profiler.inl"

//////////////////////////////////////////////////////////////////////////////////////////
// Macros to do the actual profiling
//////////////////////////////////////////////////////////////////////////////////////////

MOSS_SUPPRESS_WARNING_PUSH
MOSS_CLANG_SUPPRESS_WARNING("-Wc++98-compat-pedantic")

/// Start instrumenting program
#define MOSS_PROFILE_START(name)			do { Profiler::sInstance = new Profiler; MOSS_PROFILE_THREAD_START(name); } while (false)

/// End instrumenting program
#define MOSS_PROFILE_END()				do { MOSS_PROFILE_THREAD_END(); delete Profiler::sInstance; Profiler::sInstance = nullptr; } while (false)

/// Start instrumenting a thread
#define MOSS_PROFILE_THREAD_START(name)	do { if (Profiler::sInstance) ProfileThread::sSetInstance(new ProfileThread(name)); } while (false)

/// End instrumenting a thread
#define MOSS_PROFILE_THREAD_END()		do { delete ProfileThread::sGetInstance(); ProfileThread::sSetInstance(nullptr); } while (false)

/// Scope profiling measurement
#define MOSS_PROFILE_TAG2(line)			profile##line
#define MOSS_PROFILE_TAG(line)			MOSS_PROFILE_TAG2(line)
#define MOSS_PROFILE(...)				ProfileMeasurement MOSS_PROFILE_TAG(__LINE__)(__VA_ARGS__)

/// Scope profiling for function
#define MOSS_PROFILE_FUNCTION()			MOSS_PROFILE(MOSS_FUNCTION_NAME)

/// Update frame counter
#define MOSS_PROFILE_NEXTFRAME()			Profiler::sInstance->NextFrame()

/// Dump profiling info
#define MOSS_PROFILE_DUMP(...)			Profiler::sInstance->Dump(__VA_ARGS__)

MOSS_SUPRESS_WARNINGS_END

#else

//////////////////////////////////////////////////////////////////////////////////////////
// Dummy profiling instructions
//////////////////////////////////////////////////////////////////////////////////////////

MOSS_SUPPRESS_WARNING_PUSH
MOSS_CLANG_SUPPRESS_WARNING("-Wc++98-compat-pedantic")

#define MOSS_PROFILE_START(name)
#define MOSS_PROFILE_END()
#define MOSS_PROFILE_THREAD_START(name)
#define MOSS_PROFILE_THREAD_END()
#define MOSS_PROFILE(...)
#define MOSS_PROFILE_FUNCTION()
#define MOSS_PROFILE_NEXTFRAME()
#define MOSS_PROFILE_DUMP(...)

MOSS_SUPRESS_WARNINGS_END

#endif
