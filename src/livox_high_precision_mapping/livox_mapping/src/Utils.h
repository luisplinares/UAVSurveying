/*
 * Copyright 2019 Scott Gardner  scott-gardner@live.com
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

/* inclusion guard */
#ifndef __UTILS_H__
#define __UTILS_H__

#include <string>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <iostream>
#include <fstream>
#include "minIni.h"

// used to pack structures
#define PACKED __attribute__((__packed__))

#ifdef __has_cpp_attribute
#  if __has_cpp_attribute(fallthrough)
#    define FALLTHROUGH [[fallthrough]]
#  elif __has_cpp_attribute(gnu::fallthrough)
#    define FALLTHROUGH [[gnu::fallthrough]]
#  endif
#endif
#ifndef FALLTHROUGH
#  define FALLTHROUGH
#endif

/* Declare and implement const and non-const versions of the array subscript
 * operator. The object is treated as an array of type_ values. */
#define DEFINE_BYTE_ARRAY_METHODS                                                                   \
	inline uint8_t &operator[](size_t i) { return reinterpret_cast<uint8_t *>(this)[i]; }           \
	inline uint8_t operator[](size_t i) const { return reinterpret_cast<const uint8_t *>(this)[i]; }

#define to_radians(x) ((x) * M_PI / 180.0)
using namespace std;
using namespace std;

uint64_t GetCurrentHrUs();
int mkarray ( char *array[], char *str, const char *seperator );
string convertMstotime ( float ms );
std::string exec ( const char *fmt, ... );

std::string ltrim(const std::string& s);
std::string rtrim(const std::string& s);
std::string trim(const std::string& s);

std::string getDiskSpacePct();
extern minIni GaConfig;

#define MYTIMESTAMP 0

class Log
{
private:
	static Log *p_inst;
	Log() {}

	long startTs = 0;
	long lastTS = 0;
	int verbose = 1;
	int firstLog = 1;
	string AppLog = "/home/odroid/Logs/Program.log";
	string AppLog1 = "/home/odroid/Logs/Program1.log";
	string AppLog2 = "/home/odroid/Logs/Program2.log";

	static int exists ( string fname )
	{
		FILE *file;

		if ( ( file = fopen ( fname.c_str(), "r" ) ) )
		{
			fclose ( file );
			return ( 1 );
		}

		return ( 0 );
	}

	void _msg ( const char * strmsg, int first = 0 )
	{
		char timedate[160];

#if MYTIMESTAMP
		uint64_t ts = GetCurrentHrUs();

		if ( startTs == 0 )
			startTs = ts;

		if ( lastTS == 0 )
			lastTS = ts;

		setlocale ( LC_ALL, "" );
		//sprintf(timedate,"%'14lu-%'11lu-%'11lu",ts,ts-startTs,ts-lastTS);

		time_t tt;
		char acttime[50];

		tt = time ( NULL );
		strftime ( acttime, sizeof ( acttime ), "%Oy-%m-%d %H:%M:%S", gmtime ( &tt ) );

		sprintf ( timedate, "%s - %'11lu", acttime, ts - lastTS );

		lastTS = ts;

#else
		time_t tt;

		tt = time ( NULL );
		strftime ( timedate, sizeof ( timedate ), "%Oy-%m-%d %H:%M:%S", gmtime ( &tt ) );
#endif

		if ( verbose )
		{
			if ( first )
				cout << "\n" <<	timedate << " " << strmsg << flush;
			else
				cout << timedate << " " << strmsg << flush;
		}

		if ( firstLog )
		{
			struct stat st;

			stat ( AppLog.c_str(), &st );

			if ( st.st_size > 1000000 )
			{
				if ( exists ( AppLog2 ) )
					remove ( AppLog2.c_str() );

				if ( exists ( AppLog1 ) )
					rename ( AppLog1.c_str(), AppLog2.c_str() );

				if ( exists ( AppLog ) )
					rename ( AppLog.c_str(), AppLog1.c_str() );
			}

			firstLog = 0;
		}

		ofstream fposition ( AppLog, std::ofstream::out | std::ofstream::app );
		if ( first )
			fposition << "\n" << timedate << " " << strmsg;
		else
			fposition << timedate << " " << strmsg;
		fposition.flush();
		fposition.close();
	}

public:
	~Log()
	{
	}

	static void start ( const char *msg )
	{
		static Log L;
		L._msg ( msg, 1 );
	}

	static void msg ( const char *fmt, ... )
	{
		static Log L;
		char outstr[1000];

		va_list ap;
		va_start ( ap, fmt );
		setlocale ( LC_ALL, "" );
		vsprintf ( outstr, fmt, ap );
		va_end ( ap );

		L._msg ( outstr );

	}
};

#endif /* __UTILS_H__ */
