/*
 * Copyright (c) 2013, NANOSYSTEMS CORPORATION. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#ifndef __VIDEO_SOURCE_H__
#define __VIDEO_SOURCE_H__

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/ocl.hpp>

#include <atomic>
#include <cmath>
#include <string>
#include <vector>

#include "NSLFrame.h"
#include "lens_transform.h"
#include "YoloPose.h"
#include "YoloDet.h"

class videoSource
{
protected:
	videoSource();

private:

	static void del_arg(int argc, char **argv, int index);
	static int find_arg(int argc, char* argv[], char *arg);
	static int find_int_arg(int argc, char **argv, char *arg, int def);
	static float find_float_arg(int argc, char **argv, char *arg, float def);
	static char *find_char_arg(int argc, char **argv, char *arg, char *def);
	static void callback_mouse_click(int event, int x, int y, int flags, void* user_data);
	static videoSource* Create( int type, char *ipaddr );	

	
	void getMouseEvent( int *mouse_xpos, int *mouse_ypos );
	void mouse_click_func(int event, int x, int y);

	std::string distStringCap;
	std::string nonDistStringCap;
	std::string cntStringCap, timeStringCap;

	std::chrono::steady_clock::time_point timeDelay;
	std::chrono::steady_clock::time_point frameTime;
	std::chrono::steady_clock::time_point fpsTime;
	clock_t beginTime, endTime ;

	std::atomic<int> x_start, y_start;
	cv::dnn::Net dnnNet;
	YoloPose *yoloPose;
	YoloDet  *yoloDet;

	int localFileTest;
	int localFileTotalCnt;
	int captureTotalCnt;
	int detectTotalCnt;
	int nonDetectTotalCnt;
	char *localFileName;

public:
	static videoSource * initAppCfg(int argc, char **argv, CaptureOptions *pAppCfg);
	void setLidarOption(int netType, void *pCapOpt);
	bool captureLidar( int timeout, CaptureOptions *pAppCfg );
	int prockey(CaptureOptions *appCfg);
	void stopLidar();
	void drawCaption(cv::Mat grayMat, cv::Mat distMat, CaptureOptions *appCfg);
	void initDeepLearning( CaptureOptions *pAppCfg );
	int deepLearning( cv::Mat &imageLidar, CaptureOptions *pAppCfg );

	///////////////////// virtual interface ////////////////////////////////////////////////////////////////
	/**
	 * Create videoSource interface from a videoOptions struct that's already been filled out.
	 * It's expected that the supplied videoOptions already contain a valid resource URI.
	 */
	virtual ~videoSource();
	virtual bool Capture( void** image, int timeout=3000 ) = 0;
	virtual void startCaptureCommand( int netType, void *pCapOpt) = 0;
	virtual void setKey(int cmdKey) = 0;
	virtual std::string getDistanceString(int distance ) = 0;
	virtual int getVideoWidth() = 0;
	virtual int getVideoHeight() = 0;	
	virtual int getWidthDiv() = 0;
	virtual int getHeightDiv() = 0;
	virtual void setCameraSize(int width, int height) = 0;
	virtual int getWidth() = 0;
	virtual int getHeight() = 0;
	virtual bool isRotate90() = 0;	
	virtual void closeLidar() = 0;
	virtual std::string getLeftViewName() = 0;
	virtual void drawPointCloud() = 0;
};

#endif // __VIDEO_SOURCE_H__
