
#ifndef __TIME_CHECK_H__
#define __TIME_CHECK_H__

#include <iostream>

class TimeCheck {
private:
	struct timespec begin, end ;
	bool printOn;
public:
	void setPrint(bool PRINT_ON);
	void setBegin();
	void setEnd();
	void printTime(const char *dispStr);
};

#endif //YOLOV8_YOLOPOSE_H


