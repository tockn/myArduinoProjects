#ifndef _SD_tockn_
#define _SD_tockn_

#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <string>

class SD_tockn {
	private:
		int logNum = 0;
		char numStr[100];
		char logNumDir[100] = "/";
		char dummy[2048];
		char GPSData[2048];
		char RPYData[2048];

	public:
    int stack = 0;
    char GPSDir[100] = "/GPS.txt";
    char RPYDir[100] = "/RPY.txt";
		char directory[100] ="/flight";
		bool begin();
		bool writeFile(const char *path, const char *message);
		bool createDir(const char *path);
		bool appendFile(const char *path, const char *message);
};

#endif
