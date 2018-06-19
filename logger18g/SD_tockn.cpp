
#include "SD_tockn.h"

bool SD_tockn::begin() {
	if (!SD.begin()) return false;
	File file = SD.open(directory);
	if(!file){
    if(!createDir(directory)){
		  return false;
    }
	}
  sprintf(logNumDir, "%s/lognum.txt", directory);
  file = SD.open(logNumDir);
  if(!file){
    return false;
  }
	int i = 0;
	while(file.available()){
		numStr[i] = file.read();
		i++;
		if(i >= 100) break;
	}
	logNum = atoi(numStr);
  Serial.println();
  Serial.println(numStr);
	logNum++;
 
	sprintf(numStr, "%d", logNum);
	writeFile(logNumDir, numStr);
	sprintf(directory, "%s/%d", directory, logNum);
	createDir(directory);
	sprintf(GPSDir, "%s/gps.txt", directory);
	sprintf(RPYDir, "%s/rpy.txt", directory);

}


bool SD_tockn::writeFile(const char * path, const char * message){
	File file = SD.open(path, FILE_WRITE);
	if(!file){
		return false;
	}
	if(file.print(message)){
		return true;
	} else {
		return false;
	}
}

bool SD_tockn::createDir(const char * path){
    if(!SD.mkdir(path)){
		return false;
    }
}

bool SD_tockn::appendFile(const char * path, const char * message){
    File file = SD.open(path, FILE_APPEND);
    if(!file){
      return false;
    }
    if(file.print(message)){
      stack++;
      if (stack > 100)  stack = 100;
      return true;
    } else {
      return false;
    }
}
