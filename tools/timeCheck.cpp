#include "timeCheck.h"

void TimeCheck::setPrint(bool PRINT_ON)
{
	printOn = PRINT_ON;
}


void TimeCheck::setBegin()
{
	clock_gettime(CLOCK_MONOTONIC, &begin);
}


void TimeCheck::setEnd()
{
	clock_gettime(CLOCK_MONOTONIC, &end);
}


void TimeCheck::printTime(const char *dispStr)
{
	if( printOn ) {
		time_t passed_time = (end.tv_sec*1000 + end.tv_nsec/1000000)-(begin.tv_sec*1000 + begin.tv_nsec/1000000);
		printf("%s = %ldms\n", dispStr, passed_time);
	}
}

