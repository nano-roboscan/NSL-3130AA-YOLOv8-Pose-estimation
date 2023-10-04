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

#include <memory>
#include <thread>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "videoSource.h"
#include "NSL3130AA.h"

#ifdef HAVE_CV_CUDA
#include <opencv2/cudawarping.hpp>
#endif

#define WIN_NAME			"nanosystems"

#define NSL1110_TYPE			0
#define NSL3130_TYPE			1

#define VIEW_INFO_UPPER_SIZE	50
#define VIEW_INFO_Y_SIZE		160
#define VIEW_INFO_LOWER_SIZE	30

#define DETECT_DELAY_TIME	2000
#define CHECK_LONG_TIME		(3600 * 24)	// 24 hour
#define MOTION_MAX_CNT		(120 * 20)	//20 min
#define MOTION_SENSITIVITY	40

#define MINIBOX_WIDTH		40
#define MINIBOX_HEIGHT		40


// constructor
videoSource::videoSource()
{
	printf("new videoSource()\n");

	fpsTime = std::chrono::steady_clock::now();
	yoloPose = NULL;
	yoloDet = NULL;
	
	localFileTest = 0;
	localFileTotalCnt = 0;
	captureTotalCnt = 0;	
	detectTotalCnt = 0;
	nonDetectTotalCnt = 0;
	localFileName = NULL;
}

videoSource::~videoSource()
{
	if( yoloPose != NULL ) delete yoloPose;
	if( yoloDet != NULL ) delete yoloDet;
	printf("~videoSource()\n");
}

/////////////////////////////////////// static function ///////////////////////////////////////////////////////////

void print_help()
{
	printf("-nslType : 0 : NSL-1110AA, 1 : NSL-3130AA\n");
	printf("-captureType : 0 : AMPLITUDE_DISTANCE, 1 : AMPLITEDE_DISTANCE_EX_MODE, 2 : DISTANCE_GRAYSCALE_MODE\n");
	printf("-maxDistance : 0 ~ 12500\n");
	printf("-intTime : 0 ~ 4000\n");
	printf("-grayintTime : 0 ~ 50000\n");
	printf("-amplitudeMin : 30 ~ 1000\n");
	printf("-edgeThresHold : 0=disable, 1~5000\n");
	printf("-medianFilterEnable : 0=disable, 1=enable\n");
	printf("-medianFilter : 0=disable, 1~99\n");
	printf("-medianIter : 0=disable, 1~10000\n");
	printf("-gaussIter : 0=disable, 1~10000\n");
	printf("-averageFilterEnable : 0=disable, 1=enable\n");
	printf("-temporalFactor : 0=disable, 1~1000\n");
	printf("-temporalThresHold : 0=disable, 1~1000\n");
	printf("-interferenceEnable : 0=disable, 1=enable\n");
	printf("-interferenceLimit : 0=disable, 1~1000\n");
	printf("-modelType : 0:Yolo8-pose, 1:Yolo8-pose-detection, 2:Yolov8-detection, 3:Yolov4-csp\n");
	printf("-dualBeam : 0:off, 1:6Mhz, 2:3Mhz\n");
	printf("-deepLearning : 0:off, 1:on(amplitude : overflow off)\n");
}

/*
# -nslType : 0 : NSL-1110AA, 1 : NSL-3130AA
# -captureType : 0 ~ 2
#  0	AMPLITEDE_DISTANCE_MODE
#  1	AMPLITEDE_DISTANCE_EX_MODE
#  2	DISTANCE_GRAYSCALE_MODE
# -maxDistance : 0 ~ 12500
# -intTime : 50 ~ 4000
# -grayintTime : 50 ~ 50000
# -amplitudeMin : 30 ~ 1000
# -edgeThresHold : 0=disable, 1~5000
# -medianFilterEnable :  0=disable, 1=enable
# -medianFilter : 0=disable, 1~99
# -medianIter : 0=disable, 1~10000
# -gaussIter :  0=disable, 1~10000
# -averageFilterEnable :  0=disable, 1=enable
# -temporalFactor :  0=disable, 1~1000
# -temporalThresHold :  0=disable, 1~1000
# -interferenceEnable :  0=disable, 1=enable
# -interferenceLimit :  0=disable, 1~1000
*/
videoSource *videoSource::initAppCfg(int argc, char **argv, CaptureOptions *pAppCfg)
{
	memset(pAppCfg, 0, sizeof(CaptureOptions));
	
	pAppCfg->minConfidence= 100.0;
	pAppCfg->maxConfidence = 0;

	// HELP ...
	for(int i = 0; i < argc; ++i){
		if(!argv[i]) continue;
		if(0==strcmp(argv[i], "-help")){
			print_help();
			exit(0);
		}
	}

	
	char *ipAddr = find_char_arg(argc, argv, (char *)"ipaddr", (char *)"192.168.0.220");	

	pAppCfg->lidarType = find_int_arg(argc, argv, (char *)"-nslType", NSL3130_TYPE); // 0 : NSL1110AA, 1 : NSL3130AA
	pAppCfg->inputSize = find_int_arg(argc, argv, (char *)"-inputSize", 0); // 0 : 320, 1 : 416
	pAppCfg->detectThreshold = find_float_arg(argc, argv, (char *)"-thresHold", 0);	// 0: defined YoloPose.h, N: user defined

	pAppCfg->captureType = find_int_arg(argc, argv, (char *)"-captureType", 1);//0 ~ 2
	pAppCfg->integrationTime = find_int_arg(argc, argv, (char *)"-intTime", 1500);//800;
	pAppCfg->grayIntegrationTime = find_int_arg(argc, argv, (char *)"-grayintTime", 100);//100;
	pAppCfg->maxDistance = find_int_arg(argc, argv, (char *)"-maxDistance", 12500);
	pAppCfg->minAmplitude = find_int_arg(argc, argv, (char *)"-amplitudeMin", 50);

	pAppCfg->edgeThresHold = find_int_arg(argc, argv, (char *)"-edgeThresHold", 0);
	pAppCfg->medianFilterSize = find_int_arg(argc, argv, (char *)"-medianFilterSize", 0);
	pAppCfg->medianFilterIterations = find_int_arg(argc, argv, (char *)"-medianIter", 0);
	pAppCfg->gaussIteration = find_int_arg(argc, argv, (char *)"-gaussIter", 0);

	pAppCfg->medianFilterEnable = find_int_arg(argc, argv, (char *)"-medianFilterEnable", 0);
	pAppCfg->averageFilterEnable = find_int_arg(argc, argv, (char *)"-averageFilterEnable", 0);
	pAppCfg->temporalFilterFactorActual = find_int_arg(argc, argv, (char *)"-temporalFactor", 0);
	pAppCfg->temporalFilterThreshold = find_int_arg(argc, argv, (char *)"-temporalThresHold", 0);
	pAppCfg->interferenceUseLashValueEnable = find_int_arg(argc, argv, (char *)"-interferenceEnable", 0);
	pAppCfg->interferenceLimit = find_int_arg(argc, argv, (char *)"-interferenceLimit", 0);


	pAppCfg->dualbeamState = find_int_arg(argc, argv, (char *)"-dualBeam", 0);	// 0:off, 1:6Mhz, 2:3Mhz
	pAppCfg->hdr_mode = find_int_arg(argc, argv, (char *)"-hdrMode", 0);		// 0:off, 1:spatial, 2:temporal
	pAppCfg->deeplearning = find_int_arg(argc, argv, (char *)"-deepLearning", 0);	// 0:off, 1:on(amplitude : overflow off)
	pAppCfg->modelType = find_int_arg(argc, argv, (char *)"-modelType", 0);		// 0:Yolo8-pose, 1:Yolo8-pose-detection, 2:Yolov8-detection, 3:Yolov4-csp

	if( pAppCfg->inputSize == 0 ) pAppCfg->inputSize = 320;
	else if( pAppCfg->inputSize == 1 ) pAppCfg->inputSize = 416;

	if( pAppCfg->detectThreshold > 1 ) pAppCfg->detectThreshold /= 100.0f;

	if( pAppCfg->captureType < 0 || pAppCfg->captureType > 2 ) pAppCfg->captureType = 1;
	if( pAppCfg->maxDistance == 0 ) pAppCfg->maxDistance = 12500;
	if( pAppCfg->integrationTime == 0 ) pAppCfg->integrationTime = 800;
	if( pAppCfg->grayIntegrationTime == 0 ) pAppCfg->grayIntegrationTime = 100;
	if( pAppCfg->minAmplitude == 0 ) pAppCfg->minAmplitude = 50;
	if( pAppCfg->medianFilterSize < 0 || pAppCfg->medianFilterSize > 99 ) pAppCfg->medianFilterSize = 0;
	if( pAppCfg->medianFilterIterations < 0 || pAppCfg->medianFilterIterations > 10000 ) pAppCfg->medianFilterIterations = 0;
	if( pAppCfg->gaussIteration < 0 || pAppCfg->gaussIteration > 10000 ) pAppCfg->gaussIteration = 0;
	if( pAppCfg->edgeThresHold < 0 || pAppCfg->edgeThresHold > 10000 ) pAppCfg->edgeThresHold = 0;
	
	return videoSource::Create(pAppCfg->lidarType, ipAddr);
}

videoSource* videoSource::Create( int type, char *ipaddr )
{
	videoSource* src = NULL;

	src = NSL3130AA::Create(ipaddr);

	return src;
}


void videoSource::callback_mouse_click(int event, int x, int y, int flags, void* user_data)
{
	videoSource* vidSrc = reinterpret_cast<videoSource*>(user_data);
	vidSrc->mouse_click_func(event, x, y);
}

void videoSource::del_arg(int argc, char **argv, int index)
{
    int i;
    for(i = index; i < argc-1; ++i) argv[i] = argv[i+1];
    argv[i] = 0;
}

int videoSource::find_arg(int argc, char* argv[], char *arg)
{
    int i;
    for(i = 0; i < argc; ++i) {
        if(!argv[i]) continue;
        if(0==strcmp(argv[i], arg)) {
            del_arg(argc, argv, i);
            return 1;
        }
    }
    return 0;
}

int videoSource::find_int_arg(int argc, char **argv, char *arg, int def)
{
    int i;

    for(i = 0; i < argc-1; ++i){
        if(!argv[i]) continue;
        if(0==strcmp(argv[i], arg)){
            def = atoi(argv[i+1]);
            del_arg(argc, argv, i);
            del_arg(argc, argv, i);
            break;
        }
    }
    return def;
}

float videoSource::find_float_arg(int argc, char **argv, char *arg, float def)
{
    int i;
    for(i = 0; i < argc-1; ++i){
        if(!argv[i]) continue;
        if(0==strcmp(argv[i], arg)){
            def = atof(argv[i+1]);
            del_arg(argc, argv, i);
            del_arg(argc, argv, i);
            break;
        }
    }
    return def;
}

char *videoSource::find_char_arg(int argc, char **argv, char *arg, char *def)
{
    int i;
    for(i = 0; i < argc-1; ++i){
        if(!argv[i]) continue;
        if(0==strcmp(argv[i], arg)){
            def = argv[i+1];
            del_arg(argc, argv, i);
            del_arg(argc, argv, i);
            break;
        }
    }
    return def;
}


void videoSource::mouse_click_func(int event, int x, int y)
{
    if (event == cv::EVENT_LBUTTONDOWN)
    {
        x_start = x;
        y_start = y;
    }
    else if (event == cv::EVENT_LBUTTONUP)
    {
    }
    else if (event == cv::EVENT_MOUSEMOVE)
    {
    }
}



/////////////////////////////////////// private function ///////////////////////////////////////////////////////////


void videoSource::getMouseEvent( int *mouse_xpos, int *mouse_ypos )
{
	*mouse_xpos = x_start;
	*mouse_ypos = y_start;
}


/////////////////////////////////////// public function ///////////////////////////////////////////////////////////
void videoSource::initDeepLearning( CaptureOptions *pAppCfg )
{

	if( pAppCfg->modelType == YOLO_V8_POSE_TYPE || pAppCfg->modelType == YOLO_V8_POSE_DETECTION_TYPE ){
		cv::Mat initImage(MODEL_WIDTH, MODEL_HEIGHT, CV_8UC3, cv::Scalar(255,255,255)); 
		yoloPose = new YoloPose();
		yoloPose->init("../data/yolov8n-pose.onnx", pAppCfg->detectThreshold, pAppCfg->modelType);
		yoloPose->detect(initImage);
	}
	else{
		yoloDet = new YoloDet();

		if(pAppCfg->modelType == YOLO_V8_DETECTION_TYPE){
			cv::Mat initImage(MODEL_WIDTH, MODEL_HEIGHT, CV_8UC3, cv::Scalar(255,255,255)); 
			yoloDet->init("../data/yolov8n.onnx", "", pAppCfg->detectThreshold, pAppCfg->modelType);
			yoloDet->detect(initImage);
		}
		else{ // pAppCfg->modelType == YOLO_V4_DETECTION_TYPE
			cv::Mat initImage(V4_MODEL_WIDTH, V4_MODEL_HEIGHT, CV_8UC3, cv::Scalar(255,255,255)); 
			yoloDet->init("../data/yolov4-csp.weights", "../data/yolov4-csp.cfg", pAppCfg->detectThreshold, pAppCfg->modelType); // 8fps
			setCameraSize(V4_MODEL_WIDTH, V4_MODEL_HEIGHT);
			yoloDet->detect(initImage);
		}
	}
}


int videoSource::deepLearning( cv::Mat &imageLidar, CaptureOptions *pAppCfg )
{
	int retSize = 0;
	
	if( pAppCfg->modelType == YOLO_V8_POSE_TYPE || pAppCfg->modelType == YOLO_V8_POSE_DETECTION_TYPE ){
		retSize = yoloPose->detect(imageLidar);
	}
	else {
		retSize = yoloDet->detect(imageLidar);
	}

	/*
		calculate accuracy by picture image
	*/
	if( localFileTest != 0 )
	{
		if( retSize == 0 ){
			nonDetectTotalCnt++;
			printf("non-detect :: total:%d/%d,success:%d,fail:%d path = %s\n", localFileTotalCnt, captureTotalCnt, detectTotalCnt, nonDetectTotalCnt, localFileName);
		}
		else{
			detectTotalCnt++;
		}
	}

	return retSize;
}


void videoSource::drawCaption(cv::Mat grayMat, cv::Mat distMat, CaptureOptions *appCfg)
{
	int mouse_xpos, mouse_ypos;
	getMouseEvent(&mouse_xpos, &mouse_ypos);

	cv::Mat drawMat;

	if( !appCfg->isRotate ){
		cv::resize( grayMat, grayMat, cv::Size( DISPLAY_WIDTH, DISPLAY_HEIGHT ));
		cv::resize( distMat, distMat, cv::Size( DISPLAY_WIDTH, DISPLAY_HEIGHT ));
	}
	else{
		cv::resize( grayMat, grayMat, cv::Size( DISPLAY_HEIGHT, DISPLAY_WIDTH ));
		cv::resize( distMat, distMat, cv::Size( DISPLAY_HEIGHT, DISPLAY_WIDTH ));
	}

	// draw people count
	cv::Mat countBox = cv::Mat(MINIBOX_HEIGHT, MINIBOX_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0));
	std::string cntCaption = cv::format("%d", appCfg->detectingCnt);
	if( appCfg->detectingCnt > 9 )
		putText(countBox, cntCaption.c_str(), cv::Point(0, 30), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 255));
	else
		putText(countBox, cntCaption.c_str(), cv::Point(10, 30), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 255));
	cv::Mat imageROI(grayMat, cv::Rect(grayMat.cols-countBox.cols, 0, countBox.cols, countBox.rows)); 
	countBox.copyTo(imageROI);

	// distance + gray image concat
	cv::hconcat(grayMat, distMat, drawMat);

	// title & info 
	cv::Mat viewInfoUpper(VIEW_INFO_UPPER_SIZE, grayMat.cols * 2, CV_8UC3, cv::Scalar(0,0,0));
	cv::Mat viewInfoLower(VIEW_INFO_LOWER_SIZE, grayMat.cols * 2, CV_8UC3, cv::Scalar(255,255,255));
	cv::Mat viewInfo(VIEW_INFO_Y_SIZE, grayMat.cols * 2, CV_8UC3, cv::Scalar(0,0,0));

	std::string dist_caption;
	std::string defaultInfoTitle;

	int width_div = getWidthDiv();
	int height_div = getHeightDiv();

	if( appCfg->lidarType == NSL3130_TYPE ){
		defaultInfoTitle = cv::format("NANOSYSTEMS NSL-3130AA Viewer");
	}
	else{
		defaultInfoTitle = cv::format("NANOSYSTEMS NSL-1110AA Viewer");
	}
	
	std::string defaultInfoLower = cv::format("Nanosystems. co.,Ltd.\u00402022");

	if( isRotate90() == false ){
		cv::line(drawMat, cv::Point(0, 0), cv::Point(0,10), cv::Scalar(0, 255, 255), 1);
		cv::line(drawMat, cv::Point(0, 0), cv::Point(10,0), cv::Scalar(0, 255, 255), 1);
		
		cv::line(drawMat, cv::Point(grayMat.cols, 0), cv::Point(grayMat.cols,10), cv::Scalar(0, 255, 255), 1);
		cv::line(drawMat, cv::Point(grayMat.cols, 0), cv::Point(grayMat.cols+10,0), cv::Scalar(0, 255, 255), 1);
	}
	else{
		cv::line(drawMat, cv::Point(grayMat.cols-1, 0), cv::Point(grayMat.cols-1,10), cv::Scalar(0, 255, 255), 1);
		cv::line(drawMat, cv::Point(grayMat.cols-1, 0), cv::Point(grayMat.cols-1-10, 0), cv::Scalar(0, 255, 255), 1);
		
		cv::line(drawMat, cv::Point(grayMat.cols*2-1, 0), cv::Point(grayMat.cols*2-1,10), cv::Scalar(0, 255, 255), 1);
		cv::line(drawMat, cv::Point(grayMat.cols*2-1, 0), cv::Point(grayMat.cols*2-1-10,0), cv::Scalar(0, 255, 255), 1);

	}


	if( mouse_xpos >= 0 && mouse_ypos >= VIEW_INFO_UPPER_SIZE && mouse_ypos < grayMat.rows + VIEW_INFO_UPPER_SIZE )
	{
		int tofcam_XPos;
		int tofcam_YPos;

		mouse_ypos -= VIEW_INFO_UPPER_SIZE;

		if( mouse_xpos < getWidth() )
		{
			int x_limit_left = mouse_xpos >= 10 ? 10 : mouse_xpos;
			int x_limit_right = mouse_xpos <= (getWidth()-10) ? 10 : getWidth()-mouse_xpos;
			
			int y_limit_left = mouse_ypos >= 10 ? 10 : mouse_ypos;
			int y_limit_right = mouse_ypos <= (getHeight()-10) ? 10 : getHeight()-mouse_ypos;

//				printf("x = %d, %d :: y = %d, %d\n", x_limit_left, x_limit_right, y_limit_left, y_limit_right);
			
			cv::line(drawMat, cv::Point(mouse_xpos-x_limit_left, mouse_ypos), cv::Point(mouse_xpos+x_limit_right, mouse_ypos), cv::Scalar(255, 255, 0), 2);
			cv::line(drawMat, cv::Point(mouse_xpos, mouse_ypos-y_limit_left), cv::Point(mouse_xpos, mouse_ypos+y_limit_right), cv::Scalar(255, 255, 0), 2);
			
			if( isRotate90() == false )
				tofcam_XPos = mouse_xpos/width_div;
			else
				tofcam_YPos = (getWidth()-mouse_xpos)/width_div;
				
		}
		else{

			int x_limit_left = mouse_xpos >= getWidth()+10 ? 10 : mouse_xpos-getWidth();
			int x_limit_right = mouse_xpos <= getWidth()+(getWidth()-10) ? 10 : (getWidth()*2)-mouse_xpos;
			
			int y_limit_left = mouse_ypos >= 10 ? 10 : mouse_ypos;
			int y_limit_right = mouse_ypos <= (getHeight()-10) ? 10 : getHeight()-mouse_ypos;
			
//				printf("x = %d, %d :: y = %d, %d\n", x_limit_left, x_limit_right, y_limit_left, y_limit_right);

			cv::line(drawMat, cv::Point(mouse_xpos-x_limit_left, mouse_ypos), cv::Point(mouse_xpos+x_limit_right, mouse_ypos), cv::Scalar(255, 255, 0), 2);
			cv::line(drawMat, cv::Point(mouse_xpos, mouse_ypos-y_limit_left), cv::Point(mouse_xpos, mouse_ypos+y_limit_right), cv::Scalar(255, 255, 0), 2);

			if( isRotate90() == false )
				tofcam_XPos = (mouse_xpos-getWidth())/width_div;
			else
				tofcam_YPos = (getWidth()-(mouse_xpos-getWidth()))/width_div;
		}

		if( isRotate90() == false )
			tofcam_YPos = mouse_ypos/height_div;
		else
			tofcam_XPos = mouse_ypos/height_div;

		int dist_pos;
		
		if( isRotate90() == false ) dist_pos = tofcam_YPos*getVideoWidth() + tofcam_XPos;
		else dist_pos = tofcam_YPos*getVideoHeight() + tofcam_XPos;

		dist_caption = cv::format("X:%d, Y:%d, %s", tofcam_XPos, tofcam_YPos, getDistanceString(appCfg->pDistanceTable[dist_pos]).c_str());
	}
	else if( distStringCap.length() > 0 ){
		dist_caption = distStringCap;
	}
	else if( nonDistStringCap.length() > 0 ){
		dist_caption = nonDistStringCap;
	}

	std::string defaultInfoCap2;
	std::string defaultInfoCap3;
	std::string defaultInfoCap1 = cv::format("<%s>                           <Distance>", getLeftViewName().c_str());

	
	defaultInfoCap2 = cv::format("position       :     %s", dist_caption.c_str());
	defaultInfoCap3 = cv::format("frame rate    :     %d fps", appCfg->displayFps);

	putText(viewInfoUpper, defaultInfoTitle.c_str(), cv::Point(340, 35), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 255, 255));
	putText(viewInfoLower, defaultInfoLower.c_str(), cv::Point(780, 26), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 0));

	putText(viewInfo, defaultInfoCap1.c_str(), cv::Point(245, 35), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 255, 255));
	putText(viewInfo, defaultInfoCap2.c_str(), cv::Point(90, 90), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 255, 255));
	putText(viewInfo, defaultInfoCap3.c_str(), cv::Point(90, 125), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 255, 255));
	
#ifdef HAVE_CV_CUDA
	cv::cuda::GpuMat gpuUpper(viewInfoUpper), gpuView(viewInfo), gpuLower(viewInfoLower), gpuImage(drawMat), gpuVconcat(drawMat.rows+viewInfoUpper.rows+viewInfoLower.rows+viewInfo.rows, drawMat.cols, drawMat.type());

	gpuUpper.copyTo(gpuVconcat(cv::Rect(0, 0, gpuUpper.cols, gpuUpper.rows)));
	gpuImage.copyTo(gpuVconcat(cv::Rect(0, gpuUpper.rows,gpuImage.cols, gpuImage.rows)));
	gpuView.copyTo(gpuVconcat(cv::Rect(0, gpuUpper.rows+gpuImage.rows, gpuView.cols, gpuView.rows)));
	gpuLower.copyTo(gpuVconcat(cv::Rect(0, gpuUpper.rows+gpuImage.rows+gpuView.rows, gpuLower.cols, gpuLower.rows)));
	gpuVconcat.download(drawMat);
#else
	cv::vconcat(viewInfoUpper, drawMat, drawMat);
	cv::vconcat(drawMat, viewInfo, drawMat);
	cv::vconcat(drawMat, viewInfoLower, drawMat);
#endif

    cv::imshow(WIN_NAME, drawMat);
//	drawPointCloud();


	std::chrono::steady_clock::time_point curTime = std::chrono::steady_clock::now();
	double frame_time = (curTime - frameTime).count() / 1000000.0;
	double fps_time = (curTime - fpsTime).count() / 1000000.0;

	appCfg->fpsCount++;

	if( fps_time >= 1000.0f ){
		appCfg->displayFps = appCfg->fpsCount;

		printf("sample %d fps time = %.3f/%.3f w=%d h=%d\n", appCfg->displayFps, frame_time, fps_time, getWidth(), getHeight());
		appCfg->fpsCount = 0;
		fpsTime = curTime;
	}

	return;
}



void videoSource::stopLidar()
{
	printf("stop Lidar\n");
	closeLidar();
}


bool videoSource::captureLidar( int timeout, CaptureOptions *pAppCfg )
{
	ImageFrame *camImage = NULL;	
	frameTime = std::chrono::steady_clock::now();
	bool ret = Capture((void **)&camImage, timeout);

	if( ret ){
		pAppCfg->isRotate = camImage->isRotate;
		pAppCfg->frameMat = camImage->frameMat;
		pAppCfg->distMat = camImage->distMat;
		pAppCfg->pDistanceTable = camImage->pDistanceTable;

		localFileTest = camImage->localFileTest;
		if( localFileTest != 0 ){
			captureTotalCnt++;
			localFileTotalCnt = camImage->localFileTotalCnt;
			localFileName = camImage->localFileName;
		}
	}
	else{
		if( localFileTest ){
			printf("LOCAL-DETECTION-INFO :: total:%d/%d,success:%d,fail:%d\n", localFileTotalCnt, captureTotalCnt, detectTotalCnt, nonDetectTotalCnt);
		}
	}


	return ret;
}


void videoSource::setLidarOption(int netType, void *pCapOpt)
{
	beginTime = std::clock();

	cv::namedWindow(WIN_NAME, cv::WINDOW_NORMAL);
	cv::setWindowProperty(WIN_NAME, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);	
	cv::setMouseCallback(WIN_NAME, callback_mouse_click, this);

	startCaptureCommand(netType, pCapOpt);
}


int videoSource::prockey(CaptureOptions *appCfg)
{
	int key = cv::waitKey(1);

//	printf("prockey key = %d\n", key);
	switch(key)
	{
		case 'b':
		case 'B':
			// black & white(grayscle) mode
			setKey('b');
			break;
		case '0':
			// HDR Off
			setKey('0');
			break;
		case '1':
			// HDR spatial
			setKey('1');
			break;
		case '2':
			// HDR temporal
			setKey('2');
			break;
		case 'd':
		case 'D':
			//graysca & distance mode
			setKey('d');
			break;
		case 'a':
		case 'A':
			// amplitude & distance mode
			setKey('a');
			break;
		case 'e':
		case 'E':
			// amplitude(Gray) & distance mode
			setKey('e');
			break;
		case 't':
		case 'T':
			// grayscale LED On / Off
			setKey('t');
			break;
		case 'g':
		case 'G':
			//grayscale gain corrected
			setKey('g');
			break;
		case 'o':
		case 'O':
			//grayscale offset corrected
			setKey('o');
			break;
		case 'l':
		case 'L':
			//ambient light enable/disable
			setKey('l');
			break;
		case 's':
		case 'S':
			//saturation enable/disable
			setKey('s');
			break;
		case 'f':
		case 'F':
			//overflow enable/disable
			setKey('f');
			break;
		case 'h':
		case 'H':
			//Help
			setKey('h');
			break;
		case 'u':
		case 'U':
			//DRNU enable / disable
			setKey('u');
			break;
		case 'r':
		case 'R':
			//rotate 90
			setKey('r');
			break;
		case 'p':
		case 'P':
			// point-cloud
			setKey('p');
			break;
			
	}

	return key;
}


