#ifndef LOGGER_HPP
#define LOGGER_HPP

#include <macros.h>
#include <cstdio>

struct Logger
{
	int save_logs;		// Flag for activating the log writting
	FILE *file;			// File for writting logs
	char head_string[INTEGER_SIZE];	// Header string (to be passed as argument when it is needed to write info from other class or component)

	void SetVoidHeadString();
	// TODO: create 'getter' methods
};


#endif
