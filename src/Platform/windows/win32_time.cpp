#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <Moss/Moss_Platform.h>
#include <Moss/Moss_stdinc.h>


struct Moss_Timer {
    Moss_Time start;
    Moss_Time end;
};

Moss_Time Moss_get_time() {
    Moss_Time t;
    LARGE_INTEGER freq, counter;
    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&counter);
    t = (counter.QuadPart * 1000000000ULL) / freq.QuadPart;
    return t;
}

void Moss_Timer_start(Moss_Timer* timer) { timer->start = Moss_get_time(); }
void Moss_Timer_stop(Moss_Timer* timer) { timer->end = Moss_get_time(); }


void Moss_SleepMilliseconds(Moss_Time milliseconds) {
    while (milliseconds > 0) {
        DWORD chunk = (milliseconds > (Moss_Time)0xFFFFFFFF) ? 0xFFFFFFFF : (DWORD)milliseconds;
        Sleep(chunk);
        milliseconds -= chunk;
    }
}

void Moss_GetNanoseconds(Moss_Time nanoseconds) {
    DWORD milliseconds = (DWORD)(nanoseconds / 1000000ULL);
    Sleep(milliseconds);
}

void Moss_Yield(void);

double Moss_Delay(double delay);






// Date and Time

/*! @brief Returns "HH:MM:SS"*/
const char* Moss_LocalTime(void);         

/*! @brief Returns "YYYY-MM-DD_HH-MM-SS" (useful for filenames).*/
const char* Moss_TimeStamp(void);         

/*! @brief Returns Unix timestamp as string (seconds since epoch).*/
const char* Moss_TimeNow(void);           

/*! @brief Returns full ctime string: "Sun May 18 12:34:56 2025".*/
const char* Moss_CTimeNow(void);          

/*! @brief Returns "YYYY-MM-DD_HH-MM-SS" in Greenwich Mean Time.*/
const char* Moss_TimeStampGMT(void);

/*! @brief Returns "YYYY-MM-DD_HH-MM-SS" in Eastern Time.*/
const char* Moss_TimeStampET(void);

/*! @brief Returns "YYYY-MM-DD_HH-MM-SS" in Central Time.*/
const char* Moss_TimeStampCT(void);

/*! @brief Returns "YYYY-MM-DD_HH-MM-SS" in Mountain Time.*/
const char* Moss_TimeStampMT(void);

/*! @brief Returns "YYYY-MM-DD_HH-MM-SS" in Pacific Time.*/
const char* Moss_TimeStampPT(void);
// Date & Time Components.
const char* Moss_FormatTime(const char* fmt);