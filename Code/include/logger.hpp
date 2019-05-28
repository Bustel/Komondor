#ifndef LOGGER_HPP
#define LOGGER_HPP

#include <macros.h>
#include <cstdio>

const int LOG_LVL_DEBUG = 0;
const int LOG_LVL_INFO = 10;
const int LOG_LVL_WARN = 20;
const int LOG_LVL_ERROR = 30;

struct Logger
{
	FILE* file;			// File for writting logs
	int lvl;
	const char* prefix;

	void init(int lvl, FILE* file, const char* prefix); 	
	void log(int l, const char* format_key, ...);
    void v_log(int l, const char* format_key, va_list args);

    void debug(const char* format_key, ...);
    void info(const char* format_key, ...);
    void warn(const char* format_key, ...);
    void error(const char* format_key, ...);

};




#endif
