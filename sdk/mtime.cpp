#include "mtime.h"

using namespace std;
using namespace chrono;

time_point<steady_clock> time_start;

int64_t current_times(int precision)
{
	auto durations = std::chrono::steady_clock::now().time_since_epoch();
	switch(precision)
	{
        case TIME_NANOSECOND:
            return durations.count();
        case TIME_MICROSECOND:
            return duration_cast<microseconds>(durations).count();
        case TIME_MILLISECOND:
            return duration_cast<milliseconds>(durations).count();
        case TIME_SECOND:
            return duration_cast<seconds>(durations).count();
        case TIME_MINUTE:
            return duration_cast<minutes>(durations).count();
        case TIME_HOUR:
            return duration_cast<hours>(durations).count();
	}
    return 0;
}

struct tm localdate(int time_zone)
{
    time_t unix_sec = time(NULL);
    struct tm tms;

    static const int kHoursInDay = 24;
    static const int kMinutesInHour = 60;
    static const int kDaysFromUnixTime = 2472632;
    static const int kDaysFromYear = 153;
    static const int kMagicUnkonwnFirst = 146097;
    static const int kMagicUnkonwnSec = 1461;
    tms.tm_sec  =  unix_sec % kMinutesInHour;
    int i      = (unix_sec/kMinutesInHour);
    tms.tm_min  = i % kMinutesInHour; //nn
    i /= kMinutesInHour;
    tms.tm_hour = (i + time_zone) % kHoursInDay; // hh
    tms.tm_mday = (i + time_zone) / kHoursInDay;
    int a = tms.tm_mday + kDaysFromUnixTime;
    int b = (a*4  + 3)/kMagicUnkonwnFirst;
    int c = (-b*kMagicUnkonwnFirst)/4 + a;
    int d =((c*4 + 3) / kMagicUnkonwnSec);
    int e = -d * kMagicUnkonwnSec;
    e = e/4 + c;
    int m = (5*e + 2)/kDaysFromYear;
    tms.tm_mday = -(kDaysFromYear * m + 2)/5 + e + 1;
    tms.tm_mon = (-m/10)*12 + m + 2;
    tms.tm_year = b*100 + d  - 6700 + (m/10);
    return tms;
}


//休眠多少毫秒
void sleep_ms(int ms)
{
    int ts = ms / 1000;
    int ns = ms % 1000;
    struct timespec r {ts, ns * 1000 * 1000};
    nanosleep(&r, NULL);
}

//返回时间字符串
char *time_str(const char *fmt)
{
    static char str[256];
    struct tm tms = localdate();
    strftime(str, 255, fmt, &tms);
    return str;
}
