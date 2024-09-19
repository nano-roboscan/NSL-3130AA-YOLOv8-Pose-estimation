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

#ifdef DEEP_LEARNING
	yoloPose = NULL;
	yoloDet = NULL;
#endif	
	localFileTest = 0;
	localFileTotalCnt = 0;
	captureTotalCnt = 0;	
	detectTotalCnt = 0;
	nonDetectTotalCnt = 0;
	localFileName = NULL;
}

videoSource::~videoSource()
{
#ifdef DEEP_LEARNING
	if( yoloPose != NULL ) delete yoloPose;
	if( yoloDet != NULL ) delete yoloDet;
#endif
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
videoSource *videoSource::initAppCfg(int argc, char **argv, CaptureOptions &camOpt)
{
	memset(&camOpt, 0, sizeof(CaptureOptions));
	
	camOpt.minConfidence = 100.0;
	camOpt.maxConfidence = 0;

	// HELP ...
	for(int i = 0; i < argc; ++i){
		if(!argv[i]) continue;
		if(0==strcmp(argv[i], "-help")){
			print_help();
			exit(0);
		}
	}

	
	char *ipAddr = find_char_arg(argc, argv, (char *)"ipaddr", (char *)"192.168.0.220");	

	camOpt.lidarType = find_int_arg(argc, argv, (char *)"-nslType", NSL3130_TYPE); // 0 : NSL1110AA, 1 : NSL3130AA
	camOpt.inputSize = find_int_arg(argc, argv, (char *)"-inputSize", 0); // 0 : 320, 1 : 416
	camOpt.detectThreshold = find_float_arg(argc, argv, (char *)"-thresHold", 0);	// 0: defined YoloPose.h, N: user defined

	camOpt.captureType = find_int_arg(argc, argv, (char *)"-captureType", 1);//0 ~ 2
	camOpt.integrationTime = find_int_arg(argc, argv, (char *)"-intTime", 1500);//800;
	camOpt.grayIntegrationTime = find_int_arg(argc, argv, (char *)"-grayintTime", 1500);//100;
	camOpt.maxDistance = find_int_arg(argc, argv, (char *)"-maxDistance", 12500);
	camOpt.minAmplitude = find_int_arg(argc, argv, (char *)"-amplitudeMin", 50);
	camOpt.detectDistance = find_int_arg(argc, argv, (char *)"-detectDistance", 0);

	camOpt.edgeThresHold = find_int_arg(argc, argv, (char *)"-edgeThresHold", 0);
	camOpt.medianFilterSize = find_int_arg(argc, argv, (char *)"-medianFilterSize", 0);
	camOpt.medianFilterIterations = find_int_arg(argc, argv, (char *)"-medianIter", 0);
	camOpt.gaussIteration = find_int_arg(argc, argv, (char *)"-gaussIter", 0);

	camOpt.medianFilterEnable = find_int_arg(argc, argv, (char *)"-medianFilterEnable", 0);
	camOpt.averageFilterEnable = find_int_arg(argc, argv, (char *)"-averageFilterEnable", 0);
	camOpt.temporalFilterFactorActual = find_int_arg(argc, argv, (char *)"-temporalFactor", 0);
	camOpt.temporalFilterThreshold = find_int_arg(argc, argv, (char *)"-temporalThresHold", 0);
	camOpt.interferenceUseLashValueEnable = find_int_arg(argc, argv, (char *)"-interferenceEnable", 0);
	camOpt.interferenceLimit = find_int_arg(argc, argv, (char *)"-interferenceLimit", 0);


	camOpt.dualbeamState = find_int_arg(argc, argv, (char *)"-dualBeam", 0);	// 0:off, 1:6Mhz, 2:3Mhz
	camOpt.hdr_mode = find_int_arg(argc, argv, (char *)"-hdrMode", 0);		// 0:off, 1:spatial, 2:temporal
	camOpt.deeplearning = find_int_arg(argc, argv, (char *)"-deepLearning", 0);	// 0:off, 1:on(amplitude : overflow off)
	camOpt.modelType = find_int_arg(argc, argv, (char *)"-modelType", 0);		// 0:Yolo8-pose, 1:Yolo8-pose-detection, 2:Yolov8-detection, 3:Yolov4-csp

	if( camOpt.inputSize == 0 ) camOpt.inputSize = 320;
	else if( camOpt.inputSize == 1 ) camOpt.inputSize = 416;

	if( camOpt.detectThreshold > 1 ) camOpt.detectThreshold /= 100.0f;

	if( camOpt.captureType < 0 || camOpt.captureType > 2 ) camOpt.captureType = 1;
	if( camOpt.maxDistance == 0 ) camOpt.maxDistance = 12500;
	if( camOpt.integrationTime == 0 ) camOpt.integrationTime = 800;
	if( camOpt.grayIntegrationTime == 0 ) camOpt.grayIntegrationTime = 100;
	if( camOpt.minAmplitude == 0 ) camOpt.minAmplitude = 50;
	if( camOpt.medianFilterSize < 0 || camOpt.medianFilterSize > 99 ) camOpt.medianFilterSize = 0;
	if( camOpt.medianFilterIterations < 0 || camOpt.medianFilterIterations > 10000 ) camOpt.medianFilterIterations = 0;
	if( camOpt.gaussIteration < 0 || camOpt.gaussIteration > 10000 ) camOpt.gaussIteration = 0;
	if( camOpt.edgeThresHold < 0 || camOpt.edgeThresHold > 10000 ) camOpt.edgeThresHold = 0;
	
	return videoSource::Create(camOpt.lidarType, ipAddr);
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
void videoSource::initDeepLearning( CaptureOptions &camOpt )
{
#ifdef DEEP_LEARNING
	if( camOpt.modelType == YOLO_V8_POSE_TYPE || camOpt.modelType == YOLO_V8_POSE_DETECTION_TYPE ){
		cv::Mat initImage(MODEL_WIDTH, MODEL_HEIGHT, CV_8UC3, cv::Scalar(255,255,255)); 
		yoloPose = new YoloPose();
		yoloPose->init("../data/yolov8n-pose.onnx", camOpt.detectThreshold, camOpt.modelType);
		yoloPose->detect(initImage, camOpt);
	}
	else{
		yoloDet = new YoloDet();

		if(camOpt.modelType == YOLO_V8_DETECTION_TYPE){
			cv::Mat initImage(MODEL_WIDTH, MODEL_HEIGHT, CV_8UC3, cv::Scalar(255,255,255)); 
			yoloDet->init("../data/yolov8n.onnx", "", camOpt.detectThreshold, camOpt.modelType);
			yoloDet->detect(initImage, camOpt);
		}
		else{ // camOpt.modelType == YOLO_V4_DETECTION_TYPE
			cv::Mat initImage(V4_MODEL_WIDTH, V4_MODEL_HEIGHT, CV_8UC3, cv::Scalar(255,255,255)); 
			yoloDet->init("../data/yolov4-csp.weights", "../data/yolov4-csp.cfg", camOpt.detectThreshold, camOpt.modelType); // 8fps
			setCameraSize(V4_MODEL_WIDTH, V4_MODEL_HEIGHT);
			//setCameraSize(512, 384);
			yoloDet->detect(initImage, camOpt);
		}
	}
#endif	
}


int videoSource::deepLearning( cv::Mat &imageLidar, CaptureOptions &camOpt )
{
	int retSize = 0;
	
#ifdef DEEP_LEARNING
	camOpt.nonDetectingCnt = 0;
	if( camOpt.modelType == YOLO_V8_POSE_TYPE || camOpt.modelType == YOLO_V8_POSE_DETECTION_TYPE ){
		retSize = yoloPose->detect(imageLidar, camOpt);
	}
	else {
		retSize = yoloDet->detect(imageLidar, camOpt);
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
#endif
	return retSize;
}


void videoSource::drawCaption(cv::Mat grayMat, cv::Mat distMat, CaptureOptions &camOpt)
{
	int mouse_xpos, mouse_ypos;
	getMouseEvent(&mouse_xpos, &mouse_ypos);

	cv::Mat drawMat;
	int display_width = camOpt.isRotate ? MODEL_HEIGHT : MODEL_WIDTH;
	int display_height = camOpt.isRotate ? MODEL_WIDTH : MODEL_HEIGHT;

	// distance + gray image concat
#ifdef HAVE_CV_CUDA
	cv::cuda::GpuMat gpuGray(grayMat), gpuGrayImage, gpuDist(distMat), gpuDistImage;
	cv::cuda::resize(gpuGray, gpuGrayImage, cv::Size( display_width, display_height ),  cv::INTER_LINEAR);
	cv::cuda::resize(gpuDist, gpuDistImage, cv::Size( display_width, display_height ),  cv::INTER_LINEAR);

	cv::cuda::GpuMat gpuHconcat (gpuGrayImage.rows, gpuGrayImage.cols * 2, gpuGrayImage.type());
	gpuGrayImage.copyTo(gpuHconcat(cv::Rect(0,0,gpuGrayImage.cols, gpuGrayImage.rows)));
	gpuDistImage.copyTo(gpuHconcat(cv::Rect(gpuGrayImage.cols, 0,gpuDistImage.cols, gpuDistImage.rows)));

	gpuGrayImage.download(grayMat);
	gpuHconcat.download(drawMat);
#else
	cv::resize( grayMat, grayMat, cv::Size( display_width, display_height ));
	cv::resize( distMat, distMat, cv::Size( display_width, display_height ));

	cv::hconcat(grayMat, distMat, drawMat);
#endif

	// draw people count
	std::string cntCaption = cv::format("%d", camOpt.detectingCnt);
	if( camOpt.detectingCnt > 9 )
		putText(drawMat, cntCaption.c_str(), cv::Point(display_width-50, 30), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 255, 0));
	else
		putText(drawMat, cntCaption.c_str(), cv::Point(display_width-30, 30), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 255, 0));


	// title & info 
	cv::Mat viewInfoUpper(VIEW_INFO_UPPER_SIZE, drawMat.cols, CV_8UC3, cv::Scalar(0,0,0));
	cv::Mat viewInfoLower(VIEW_INFO_LOWER_SIZE, drawMat.cols, CV_8UC3, cv::Scalar(255,255,255));
	cv::Mat viewInfo(VIEW_INFO_Y_SIZE, drawMat.cols, CV_8UC3, cv::Scalar(0,0,0));

	std::string dist_caption;
	std::string defaultInfoTitle;

	int width_div = getWidthDiv();
	int height_div = getHeightDiv();

	if( camOpt.lidarType == NSL3130_TYPE ){
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

		dist_caption = cv::format("X:%d, Y:%d, %s", tofcam_XPos, tofcam_YPos, getDistanceString(camOpt.pDistanceTable[dist_pos]).c_str());
	}
	else if( distStringCap.length() > 0 ){
		dist_caption = distStringCap;
	}
	else if( nonDistStringCap.length() > 0 ){
		dist_caption = nonDistStringCap;
	}

	std::string defaultInfoCap2 = cv::format("position       :     %s", dist_caption.c_str());
	std::string defaultInfoCap3 = cv::format("frame rate    :     %d fps", camOpt.displayFps);
	std::string defaultInfoCap1 = cv::format("<%s>                           <Distance>", getLeftViewName().c_str());
	if( isRotate90() ){
		defaultInfoCap1 = cv::format("<%s>               <Distance>", getLeftViewName().c_str());
		putText(viewInfo, defaultInfoCap1.c_str(), cv::Point(160, 35), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 255, 255));
	}
	else{
		putText(viewInfo, defaultInfoCap1.c_str(), cv::Point(245, 35), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 255, 255));
	}
	
	putText(viewInfo, defaultInfoCap2.c_str(), cv::Point(90, 90), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 255, 255));
	putText(viewInfo, defaultInfoCap3.c_str(), cv::Point(90, 125), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 255, 255));

	putText(viewInfoUpper, defaultInfoTitle.c_str(), cv::Point(340, 35), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 255, 255));
	putText(viewInfoLower, defaultInfoLower.c_str(), cv::Point(780, 26), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 0));
	
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

	camOpt.fpsCount++;

	if( fps_time >= 1000.0f ){
		camOpt.displayFps = camOpt.fpsCount;

		printf("sample %d fps time = %.3f/%.3f w=%d h=%d\n", camOpt.displayFps, frame_time, fps_time, getDLWidth(), getDLHeight());
		camOpt.fpsCount = 0;
		fpsTime = curTime;
	}

	return;
}



void videoSource::stopLidar()
{
	printf("stop Lidar\n");
	closeLidar();
}


bool videoSource::captureLidar( int timeout, CaptureOptions &camOpt )
{
	ImageFrame *camImage = NULL;	

	frameTime = std::chrono::steady_clock::now();

	bool ret = Capture((void **)&camImage, timeout);

	if( ret ){
		camOpt.isRotate = camImage->isRotate;
		camOpt.frameMat = camImage->frameMat;
		camOpt.distMat = camImage->distMat;
		camOpt.pDistanceTable = camImage->pDistanceTable;

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


void videoSource::setLidarOption(int netType, CaptureOptions &camOpt)
{
	beginTime = std::clock();

	cv::namedWindow(WIN_NAME, cv::WINDOW_NORMAL);
	cv::setWindowProperty(WIN_NAME, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);	
	cv::setMouseCallback(WIN_NAME, callback_mouse_click, this);

	startCaptureCommand(netType, camOpt);
}


int videoSource::prockey(CaptureOptions &camOpt)
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


