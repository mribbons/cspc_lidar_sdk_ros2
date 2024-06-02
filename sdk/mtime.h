#ifndef __H_MTIME_H__
#define __H_MTIME_H__

#include <chrono>
#include <unistd.h>
#include <time.h>

enum TIME_PRECISION
{
	TIME_NANOSECOND = 0,	//纳秒
	TIME_MICROSECOND,		//微秒
	TIME_MILLISECOND,		//毫秒
	TIME_SECOND,
	TIME_MINUTE,
	TIME_HOUR
};

//计算用时
extern std::chrono::time_point<std::chrono::steady_clock> time_start;
inline void timestart()
{
	time_start = std::chrono::steady_clock::now();
}
inline int64_t timeused()
{
	return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time_start).count();
}
inline int64_t timeused(std::chrono::time_point<std::chrono::steady_clock>& time_last)
{
	return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time_last).count();
}

//休眠多少毫秒
void sleep_ms(int ms);

//获取tm (只更新年月日时分秒)
struct tm localdate(int time_zone = 8);

//获取时间[整数]
int64_t current_times(int precision = TIME_MILLISECOND);

//获取时间[字符串]
char *time_str(const char *fmt = "%F %T");

#endif
