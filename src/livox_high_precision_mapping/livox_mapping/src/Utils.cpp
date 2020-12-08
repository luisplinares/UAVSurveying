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


#include "Utils.h"

minIni GaConfig ( "/home/odroid/GroundSystem.ini" );

uint64_t GetCurrentHrUs()
{
	timeval curTv;

	if ( gettimeofday ( &curTv, NULL ) == -1 )
		Log::msg ( "gettimeofday failed: %d, %s\n", errno, strerror ( errno ) );

	struct tm *pCurTm = gmtime ( &curTv.tv_sec );
	time_t curToHr_t = pCurTm->tm_hour * 3600 + pCurTm->tm_min * 60 + pCurTm->tm_sec;

	return ( curToHr_t * 1000000 + curTv.tv_usec );
}

int mkarray ( char *array[], char *str, const char *seperator )
{
	int cnt = 0;
	char *p = strtok ( str, seperator );

	while ( p != NULL )
	{
		array[cnt++] = p;
		p = strtok ( NULL, seperator );
	}

	return ( cnt );
}

const std::string WHITESPACE = " \n\r\t\f\v";

std::string ltrim(const std::string& s)
{
	size_t start = s.find_first_not_of(WHITESPACE);
	return (start == std::string::npos) ? "" : s.substr(start);
}

std::string rtrim(const std::string& s)
{
	size_t end = s.find_last_not_of(WHITESPACE);
	return (end == std::string::npos) ? "" : s.substr(0, end + 1);
}

std::string trim(const std::string& s)
{
	return rtrim(ltrim(s));
}

std::string getDiskSpacePct()
{
	std::string result = exec ( "df -h" );

	size_t pos = result.find ( "/dev/mmcblk1p2" );

	std::string s = result.substr ( pos + 32, 10 );
	//Log::msg("s:%s\n", s.c_str());


	pos = 0;
	size_t w = 0;

	if ((pos = s.find ( "%" )) != std::string::npos )
	{
		return (trim(s.substr ( 0, pos )));
	}

	return result;
}

std::string exec ( const char *fmt, ... )
{
	char outstr[200];
	std::string result = "";
	char buffer[200];

	va_list ap;
	va_start ( ap, fmt );
	setlocale ( LC_ALL, "" );
	vsprintf ( outstr, fmt, ap );
	va_end ( ap );

	FILE* pipe = popen ( outstr, "r" );

	if ( !pipe )
		return result;

	try
	{
		while ( fgets ( buffer, sizeof ( buffer ) - 1, pipe ) != NULL )
		{
			//Log::msg("exec:%s\n",buffer);
			result += buffer;
		}
	}
	catch ( ... )
	{
		pclose ( pipe );
	}

	return result;
}

#define MSINSEC (1000)
#define MSINMIN (MSINSEC*60)
#define MSINHR  (MSINMIN*60)

string convertMstotime ( float ms )
{
	char buffer[20];
	int hr, min, sec, mili;

	hr   = ms / MSINHR;
	min  = ( ms - hr * MSINHR ) / MSINMIN;
	sec  = ( ms - hr * MSINHR - min * MSINMIN ) / MSINSEC;
	mili = ms - hr * MSINHR - min * MSINMIN - sec * MSINSEC;

	if ( hr > 0 )
		sprintf ( buffer, "%02d:%02d:%02d.%03d", hr, min, sec, mili );
	else //if (min > 0)
		sprintf ( buffer, "%02d:%02d.%03d", min, sec, mili );
	//else
	//	sprintf(buffer,"%02d.%03d",sec,mili);

	return ( string ( buffer ) );
}
