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

#ifdef HAVE_CV_CUDA
#include <opencv2/cudawarping.hpp>
#endif

using namespace cv;
using namespace std;
using namespace NslOption;


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

#define MODEL_WIDTH			640
#define MODEL_HEIGHT		480

#define V4_MODEL_WIDTH		512
#define V4_MODEL_HEIGHT		512

void print_help()
{
	printf("-captureType : 1 : DISTANCE_MODE, 2 : GRAYSCALE_MODE, 3 : DISTANCE_AMPLITUDE_MODE, 4 : DISTANCE_GRAYSCALE_MODE\n");
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
	printf("-detectDistance : 0:unlimit, N(mili meter)\n");
}

void callback_mouse_click(int event, int x, int y, int flags, void* user_data)
{
	videoSource* vidSrc = reinterpret_cast<videoSource*>(user_data);
	vidSrc->mouse_click_func(event, x, y);
}


/*
# -captureType : 1 ~ 8
	enum class OPERATION_MODE_OPTIONS
	{
		NONE_MODE = 0,
		DISTANCE_MODE = 1,
		GRAYSCALE_MODE = 2,
		DISTANCE_AMPLITUDE_MODE = 3,
		DISTANCE_GRAYSCALE_MODE = 4,
		RGB_MODE = 5,
		RGB_DISTANCE_MODE = 6,
		RGB_DISTANCE_AMPLITUDE_MODE = 7,
		RGB_DISTANCE_GRAYSCALE_MODE = 8
	};
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
videoSource *createApp(int argc, char **argv, CaptureOptions &camOpt)
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

	videoSource *vidSrc = new videoSource();

	cv::namedWindow(WIN_NAME, cv::WINDOW_NORMAL);
	cv::setWindowProperty(WIN_NAME, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);	
	cv::setMouseCallback(WIN_NAME, callback_mouse_click, vidSrc);
	
	const char *ipAddr = vidSrc->find_char_arg(argc, argv, "ipaddr", "192.168.0.220");	

	camOpt.detectThreshold = vidSrc->find_float_arg(argc, argv, "-thresHold", 0);	// 0: defined YoloPose.h, N: user defined (.6, .4)

	camOpt.captureType = vidSrc->find_int_arg(argc, argv, "-captureType", 3);// 1 ~ 8
	camOpt.integrationTime = vidSrc->find_int_arg(argc, argv, "-intTime", 1000);//800;
	camOpt.grayIntegrationTime = vidSrc->find_int_arg(argc, argv, "-grayintTime", 1500);//100;
	camOpt.maxDistance = vidSrc->find_int_arg(argc, argv, "-maxDistance", 12500);
	camOpt.minAmplitude = vidSrc->find_int_arg(argc, argv, "-minAmplitude", 50);
	camOpt.detectDistance = vidSrc->find_int_arg(argc, argv, "-detectDistance", 0);

	camOpt.medianFilterEnable = vidSrc->find_int_arg(argc, argv, "-medianFilterEnable", 0);
	camOpt.averageFilterEnable = vidSrc->find_int_arg(argc, argv, "-averageFilterEnable", 0);
	camOpt.temporalFilterFactorActual = vidSrc->find_int_arg(argc, argv, "-temporalFactor", 0);
	camOpt.temporalFilterThreshold = vidSrc->find_int_arg(argc, argv, "-temporalThresHold", 0);
	camOpt.edgeThresHold = vidSrc->find_int_arg(argc, argv, "-edgeThresHold", 0);
	camOpt.interferenceUseLashValueEnable = vidSrc->find_int_arg(argc, argv, "-interferenceEnable", 0);
	camOpt.interferenceLimit = vidSrc->find_int_arg(argc, argv, "-interferenceLimit", 0);

	camOpt.dualbeamState = vidSrc->find_int_arg(argc, argv, "-dualBeam", 0);	// 0:off, 1:6Mhz, 2:3Mhz
	camOpt.hdr_mode = vidSrc->find_int_arg(argc, argv, "-hdrMode", 0);		// 0:off, 1:spatial, 2:temporal
	camOpt.deeplearning = vidSrc->find_int_arg(argc, argv, "-deepLearning", 0);	// 0:off, 1:on(amplitude : overflow off)
	camOpt.modelType = vidSrc->find_int_arg(argc, argv, "-modelType", 0);		// 0:Yolo8-pose, 1:Yolo8-pose-detection, 2:Yolov8-detection, 3:Yolov4-csp

	if( camOpt.detectThreshold > 1 ) camOpt.detectThreshold = 0;

	if( camOpt.captureType <= 0 || camOpt.captureType > 8 ) camOpt.captureType = 3;
	if( camOpt.maxDistance == 0 ) camOpt.maxDistance = 12500;
	if( camOpt.integrationTime == 0 ) camOpt.integrationTime = 800;
	if( camOpt.grayIntegrationTime == 0 ) camOpt.grayIntegrationTime = 100;
	if( camOpt.minAmplitude == 0 ) camOpt.minAmplitude = 50;
	if( camOpt.edgeThresHold < 0 || camOpt.edgeThresHold > 10000 ) camOpt.edgeThresHold = 0;
	
	camOpt.nslDevConfig.lidarAngle = 0;
	camOpt.nslDevConfig.lensType = NslOption::LENS_TYPE::LENS_SF;
	vidSrc->handle = nsl_open(ipAddr, &camOpt.nslDevConfig, FUNCTION_OPTIONS::FUNC_ON);
	
	return vidSrc;
}

// constructor
videoSource::videoSource()
{
	printf("new videoSource()\n");

	fpsTime = std::chrono::steady_clock::now();

#ifdef DEEP_LEARNING
	yoloPose = NULL;
	yoloDet = NULL;
	
	DLWidth = 640;
	DLHeight = 480;
#endif	
}

videoSource::~videoSource()
{
#ifdef DEEP_LEARNING
	if( yoloPose != NULL ) delete yoloPose;
	if( yoloDet != NULL ) delete yoloDet;
#endif
	nsl_close();
	printf("~videoSource()\n");
}

/////////////////////////////////////// private function ///////////////////////////////////////////////////////////
std::string videoSource::getDistanceString(int distance )
{
	std::string distStr;

	if( distance == NSL_LOW_AMPLITUDE )
		distStr = "LOW_AMPLITUDE";
	else if( distance == NSL_ADC_OVERFLOW )
		distStr = "ADC_OVERFLOW";
	else if( distance == NSL_SATURATION )
		distStr = "SATURATION";
	else if( distance == NSL_INTERFERENCE )
		distStr = "INTERFERENCE";
	else if( distance == NSL_EDGE_DETECTED )
		distStr = "EDGE_DETECTED";
	else if( distance == NSL_BAD_PIXEL )
		distStr = "BAD_FIXEL";
	else
		distStr = format("%d mm", distance);

	return distStr;
}

int videoSource::getWidthDiv()
{
	return getWidth()/NSL_LIDAR_TYPE_A_WIDTH;
}


int videoSource::getHeightDiv()
{
	return getHeight()/NSL_LIDAR_TYPE_A_HEIGHT;
}

int videoSource::getWidth()
{
	return 640;
}


int videoSource::getHeight()
{
	return 480;
}

void videoSource::setCameraSize(int width, int height)
{
	DLWidth = width;
	DLHeight = height;
}



std::string videoSource::getLeftViewName()
{
	std::string nameStr;
	if( pcdData.operationMode == OPERATION_MODE_OPTIONS::DISTANCE_AMPLITUDE_MODE )
		nameStr = "AMPL&Gray";
	else //if( tofcamInfo.tofcamModeType == DISTANCE_GRAYSCALE_MODE )
		nameStr = "GRAYSCALE";	

	return nameStr;

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

int videoSource::find_int_arg(int argc, char **argv, const char *arg, int def)
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

float videoSource::find_float_arg(int argc, char **argv, const char *arg, float def)
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

const char *videoSource::find_char_arg(int argc, char **argv, const char *arg, const char *def)
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



/////////////////////////////////////// public function ///////////////////////////////////////////////////////////


void videoSource::getMouseEvent( int *mouse_xpos, int *mouse_ypos )
{
	*mouse_xpos = x_start;
	*mouse_ypos = y_start;
}

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
			yoloDet->detect(initImage, camOpt);
		}
	}
#endif	
}

void embed_image(cv::Mat source, cv::Mat dest, int dx, int dy)
{
    int x,y,k;
    for(k = 0; k < source.channels(); ++k){
        for(y = 0; y < source.rows; ++y){
            for(x = 0; x < source.cols; ++x){
                float val = source.at<cv::Vec3b>(y, x)[k];
				dest.at<cv::Vec3b>(dy + y, dx + x)[k] = (uchar)val;
            }
        }
    }
}

cv::Mat resize_image(cv::Mat im, int cols, int rows)	// new_w = 512 new_h = 384
{
    cv::Mat resized(rows, cols, im.type());
    cv::Mat part(im.rows, cols, im.type());
	
    int r, c, k;
    float w_scale = (float)(im.cols - 1) / (cols - 1);
    float h_scale = (float)(im.rows - 1) / (rows - 1);
    for(k = 0; k < im.channels(); ++k){
        for(r = 0; r < im.rows; ++r){
            for(c = 0; c < cols; ++c){
                float val = 0;
                if(c == cols-1 || im.cols == 1){
                     val = im.at<cv::Vec3b>(r, im.cols - 1)[k];
                } else {
                    float sx = c*w_scale;
                    int ix = (int) sx;
                    float dx = sx - ix;
                    val = (1 - dx) * im.at<cv::Vec3b>(r, ix)[k] + dx * im.at<cv::Vec3b>(r, ix + 1)[k];
                }
                part.at<cv::Vec3b>(r, c)[k] = (uchar)val;
            }
        }
    }
    for(k = 0; k < im.channels(); ++k){
        for(r = 0; r < rows; ++r){
            float sy = r*h_scale;
            int iy = (int) sy;
            float dy = sy - iy;
            for(c = 0; c < cols; ++c){
                float val = (1-dy) * part.at<cv::Vec3b>(iy, c)[k];
                resized.at<cv::Vec3b>(r, c)[k] = (uchar)val;
            }
            if(r == rows-1 || im.rows == 1) continue;
            for(c = 0; c < cols; ++c){
                float val = dy * part.at<cv::Vec3b>(iy + 1, c)[k];
                resized.at<cv::Vec3b>(r, c)[k] += (uchar)val;
            }
        }
    }

    return resized;
}


cv::Mat letterbox_image(cv::Mat im, int cols, int rows)
{
    int new_cols = im.cols;
    int new_rows = im.rows;

    if (im.cols == cols && im.rows == rows) return im;
	
    if (((float)cols / im.cols) < ((float)rows / im.rows)) {
        new_cols = cols;
        new_rows = (im.rows * cols) / im.cols;
    }
    else {
        new_rows = rows;
        new_cols = (im.cols * rows) / im.rows;
    }

    cv::Mat resized = resize_image(im, new_cols, new_rows);
    cv::Mat boxed(rows, cols, im.type());

	boxed.setTo(cv::Scalar(.5, .5, .5));
    embed_image(resized, boxed, (cols - new_cols) / 2, (rows - new_rows) / 2);
	
    return boxed;
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
#endif
	return retSize;
}


void videoSource::drawCaption(cv::Mat grayMat, cv::Mat distMat, CaptureOptions &camOpt)
{
	int mouse_xpos, mouse_ypos;
	getMouseEvent(&mouse_xpos, &mouse_ypos);

	cv::Mat drawMat;
	int display_width = MODEL_WIDTH;
	int display_height = MODEL_HEIGHT;

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
	std::string cntCaption = cv::format("%d/ ", camOpt.detectingCnt-camOpt.nonDetectingCnt);
	if( camOpt.detectingCnt > 9 )
		putText(drawMat, cntCaption.c_str(), cv::Point(display_width-100, 30), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 255, 0));
	else
		putText(drawMat, cntCaption.c_str(), cv::Point(display_width-80, 30), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 255, 0));

	std::string nonCntCaption = cv::format("%d", camOpt.nonDetectingCnt);
	if( camOpt.nonDetectingCnt > 9 )
		putText(drawMat, nonCntCaption.c_str(), cv::Point(display_width-50, 30), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(144, 144, 144));
	else
		putText(drawMat, nonCntCaption.c_str(), cv::Point(display_width-30, 30), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(144, 144, 144));

	// title & info 
	cv::Mat viewInfoUpper(VIEW_INFO_UPPER_SIZE, drawMat.cols, CV_8UC3, cv::Scalar(0,0,0));
	cv::Mat viewInfoLower(VIEW_INFO_LOWER_SIZE, drawMat.cols, CV_8UC3, cv::Scalar(255,255,255));
	cv::Mat viewInfo(VIEW_INFO_Y_SIZE, drawMat.cols, CV_8UC3, cv::Scalar(0,0,0));

	std::string dist_caption;
	std::string defaultInfoTitle;

	int width_div = getWidthDiv();
	int height_div = getHeightDiv();

	defaultInfoTitle = cv::format("NANOSYSTEMS NSL-3130AA Viewer");
	std::string defaultInfoLower = cv::format("Nanosystems. co.,Ltd.\u00402022");

	cv::line(drawMat, cv::Point(0, 0), cv::Point(0,10), cv::Scalar(0, 255, 255), 1);
	cv::line(drawMat, cv::Point(0, 0), cv::Point(10,0), cv::Scalar(0, 255, 255), 1);
	
	cv::line(drawMat, cv::Point(grayMat.cols, 0), cv::Point(grayMat.cols,10), cv::Scalar(0, 255, 255), 1);
	cv::line(drawMat, cv::Point(grayMat.cols, 0), cv::Point(grayMat.cols+10,0), cv::Scalar(0, 255, 255), 1);

	if( mouse_xpos >= 0 && mouse_ypos >= VIEW_INFO_UPPER_SIZE && mouse_ypos < grayMat.rows + VIEW_INFO_UPPER_SIZE )
	{
		int tofcam_XPos;
		int tofcam_YPos;

		mouse_ypos -= VIEW_INFO_UPPER_SIZE;

		if( mouse_xpos < MODEL_WIDTH )
		{
			int x_limit_left = mouse_xpos >= 10 ? 10 : mouse_xpos;
			int x_limit_right = mouse_xpos <= (MODEL_WIDTH-10) ? 10 : MODEL_WIDTH-mouse_xpos;
			
			int y_limit_left = mouse_ypos >= 10 ? 10 : mouse_ypos;
			int y_limit_right = mouse_ypos <= (MODEL_HEIGHT-10) ? 10 : MODEL_HEIGHT-mouse_ypos;

//				printf("x = %d, %d :: y = %d, %d\n", x_limit_left, x_limit_right, y_limit_left, y_limit_right);
			
			cv::line(drawMat, cv::Point(mouse_xpos-x_limit_left, mouse_ypos), cv::Point(mouse_xpos+x_limit_right, mouse_ypos), cv::Scalar(255, 255, 0), 2);
			cv::line(drawMat, cv::Point(mouse_xpos, mouse_ypos-y_limit_left), cv::Point(mouse_xpos, mouse_ypos+y_limit_right), cv::Scalar(255, 255, 0), 2);

			tofcam_XPos = mouse_xpos/width_div;
		}
		else{

			int x_limit_left = mouse_xpos >= MODEL_WIDTH+10 ? 10 : mouse_xpos-MODEL_WIDTH;
			int x_limit_right = mouse_xpos <= MODEL_WIDTH+(MODEL_WIDTH-10) ? 10 : (MODEL_WIDTH*2)-mouse_xpos;
			
			int y_limit_left = mouse_ypos >= 10 ? 10 : mouse_ypos;
			int y_limit_right = mouse_ypos <= (MODEL_HEIGHT-10) ? 10 : MODEL_HEIGHT-mouse_ypos;
			
//				printf("x = %d, %d :: y = %d, %d\n", x_limit_left, x_limit_right, y_limit_left, y_limit_right);

			cv::line(drawMat, cv::Point(mouse_xpos-x_limit_left, mouse_ypos), cv::Point(mouse_xpos+x_limit_right, mouse_ypos), cv::Scalar(255, 255, 0), 2);
			cv::line(drawMat, cv::Point(mouse_xpos, mouse_ypos-y_limit_left), cv::Point(mouse_xpos, mouse_ypos+y_limit_right), cv::Scalar(255, 255, 0), 2);

			tofcam_XPos = (mouse_xpos-MODEL_WIDTH)/width_div;
		}

		tofcam_YPos = mouse_ypos/height_div;

		int dist_pos;
		
		dist_pos = tofcam_YPos*NSL_LIDAR_TYPE_A_WIDTH + tofcam_XPos;

		dist_caption = cv::format("X:%d, Y:%d, %s", tofcam_XPos, tofcam_YPos, getDistanceString(pcdData.distance2D[tofcam_YPos][tofcam_XPos]).c_str());
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

	putText(viewInfo, defaultInfoCap1.c_str(), cv::Point(245, 35), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 255, 255));
	
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

		printf("sample %d fps time = %.3f/%.3f DL-Width=%d DL-Height=%d\n", camOpt.displayFps, frame_time, fps_time, DLWidth, DLHeight);
		camOpt.fpsCount = 0;
		fpsTime = curTime;
	}

	return;
}



void videoSource::stopLidar()
{
	printf("stop Lidar\n");
	nsl_streamingOff(handle);
}

void videoSource::setMatrixColor(Mat image, int x, int y, NslVec3b color)
{
	image.at<Vec3b>(y,x)[0] = color.b;
	image.at<Vec3b>(y,x)[1] = color.g;
	image.at<Vec3b>(y,x)[2] = color.r;
}

bool videoSource::captureLidar( int timeout, CaptureOptions &camOpt )
{
	frameTime = std::chrono::steady_clock::now();
	bool ret = false;
	
	if( nsl_getPointCloudData(handle, &pcdData, timeout) == NSL_ERROR_TYPE::NSL_SUCCESS )
	{
		ret = true;

		if( pcdData.includeLidar )
		{
			int width = pcdData.width;
			int height = pcdData.height;
			int xMin = pcdData.roiXMin;
			int yMin = pcdData.roiYMin;
			int lidarWidth = NSL_LIDAR_TYPE_A_WIDTH;
			int lidarHeight = NSL_LIDAR_TYPE_A_HEIGHT;
		
			Mat distanceMat = Mat(lidarHeight, lidarWidth, CV_8UC3, Scalar(255,255,255));
			Mat amplitudeMat = Mat(lidarHeight, lidarWidth, CV_8UC3, Scalar(255,255,255));
		
			for(int y = 0, index = 0; y < height; y++)
			{
				for(int x = 0; x < width; x++, index++)
				{
					setMatrixColor(distanceMat, x+xMin, y+yMin, nsl_getDistanceColor(pcdData.distance2D[y+yMin][x+xMin]));
					setMatrixColor(amplitudeMat, x+xMin, y+yMin, nsl_getAmplitudeColor(pcdData.amplitude[y+yMin][x+xMin]));
				}
			}

			camOpt.pCatesianTable = pcdData.distance3D;
			cv::resize( distanceMat, camOpt.distMat, cv::Size( DLWidth, DLHeight ) );
			cv::resize( amplitudeMat, camOpt.frameMat, cv::Size( DLWidth, DLHeight ));
		}
	}

	return ret;
}


void videoSource::setLidarOption(CaptureOptions &camOpt)
{
	beginTime = std::clock();
	
	int maxDistanceValue = (camOpt.maxDistance <= 0 || camOpt.maxDistance > MAX_DISTANCE_12MHZ) ? MAX_DISTANCE_12MHZ : camOpt.maxDistance;

	camOpt.maxDistance = maxDistanceValue;

	nsl_setIntegrationTime(handle, camOpt.integrationTime, 200, 0, camOpt.grayIntegrationTime);
	nsl_setMinAmplitude(handle, camOpt.minAmplitude);

	nsl_setFilter(handle, camOpt.medianFilterEnable ? FUNCTION_OPTIONS::FUNC_ON : FUNCTION_OPTIONS::FUNC_OFF
						, camOpt.averageFilterEnable ? FUNCTION_OPTIONS::FUNC_ON: FUNCTION_OPTIONS::FUNC_OFF
						, camOpt.temporalFilterFactorActual, camOpt.temporalFilterThreshold
						, camOpt.edgeThresHold
						, camOpt.interferenceLimit, camOpt.interferenceUseLashValueEnable ? FUNCTION_OPTIONS::FUNC_ON : FUNCTION_OPTIONS::FUNC_OFF);


	if( camOpt.captureType == (int)OPERATION_MODE_OPTIONS::DISTANCE_GRAYSCALE_MODE ){
		nsl_setGrayscaleillumination(handle, FUNCTION_OPTIONS::FUNC_ON);
	}
	else{
		nsl_setGrayscaleillumination(handle, FUNCTION_OPTIONS::FUNC_OFF);
	}

	nsl_setColorRange(camOpt.maxDistance, MAX_GRAYSCALE_VALUE, NslOption::FUNCTION_OPTIONS::FUNC_ON);
	nsl_streamingOn(handle, static_cast<OPERATION_MODE_OPTIONS>(camOpt.captureType));
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
			nsl_setColorRange(camOpt.maxDistance, MAX_GRAYSCALE_VALUE, NslOption::FUNCTION_OPTIONS::FUNC_ON);
			if( camOpt.captureType != (int)OPERATION_MODE_OPTIONS::GRAYSCALE_MODE ){
				camOpt.captureType = (int)OPERATION_MODE_OPTIONS::GRAYSCALE_MODE;
				nsl_streamingOn(handle, static_cast<OPERATION_MODE_OPTIONS>(camOpt.captureType));
			}
			
			break;
		case '0':
			// HDR Off
			nsl_setHdrMode(handle, HDR_OPTIONS::HDR_NONE_MODE);
			break;
		case '1':
			// HDR spatial
			nsl_setHdrMode(handle, HDR_OPTIONS::HDR_SPATIAL_MODE);
			break;
		case '2':
			// HDR temporal
			nsl_setHdrMode(handle, HDR_OPTIONS::HDR_TEMPORAL_MODE);
			break;
		case 'd':
		case 'D':
			//graysca & distance mode
			nsl_setColorRange(camOpt.maxDistance, MAX_GRAYSCALE_VALUE, NslOption::FUNCTION_OPTIONS::FUNC_OFF);
			if( camOpt.captureType != (int)OPERATION_MODE_OPTIONS::DISTANCE_GRAYSCALE_MODE ){
				camOpt.captureType = (int)OPERATION_MODE_OPTIONS::DISTANCE_GRAYSCALE_MODE;
				nsl_streamingOn(handle, static_cast<OPERATION_MODE_OPTIONS>(camOpt.captureType));
			}
			break;
		case 'a':
		case 'A':
			// amplitude & distance mode
			nsl_setColorRange(camOpt.maxDistance, MAX_GRAYSCALE_VALUE, NslOption::FUNCTION_OPTIONS::FUNC_OFF);

			if( camOpt.captureType != (int)OPERATION_MODE_OPTIONS::DISTANCE_AMPLITUDE_MODE ){
				camOpt.captureType = (int)OPERATION_MODE_OPTIONS::DISTANCE_AMPLITUDE_MODE;
				nsl_streamingOn(handle, static_cast<OPERATION_MODE_OPTIONS>(camOpt.captureType));
			}
			break;
		case 'e':
		case 'E':
			// amplitude(Gray) & distance mode
			nsl_setColorRange(camOpt.maxDistance, MAX_GRAYSCALE_VALUE, NslOption::FUNCTION_OPTIONS::FUNC_ON);
			break;
		case 't':
		case 'T':
		{
			nsl_getGrayscaleillumination(handle, &camOpt.nslDevConfig.grayscaleIlluminationOpt);
			// grayscale LED On / Off
			if( camOpt.nslDevConfig.grayscaleIlluminationOpt != NslOption::FUNCTION_OPTIONS::FUNC_ON )
				nsl_setGrayscaleillumination(handle, NslOption::FUNCTION_OPTIONS::FUNC_ON);
			else
				nsl_setGrayscaleillumination(handle, NslOption::FUNCTION_OPTIONS::FUNC_OFF);
			break;
		}
		case 's':
		case 'S':
			//saturation enable/disable
			nsl_getAdcOverflowSaturation(handle, &camOpt.nslDevConfig.overflowOpt, &camOpt.nslDevConfig.saturationOpt);

			if( camOpt.nslDevConfig.saturationOpt != NslOption::FUNCTION_OPTIONS::FUNC_ON )
				nsl_setAdcOverflowSaturation(handle, camOpt.nslDevConfig.overflowOpt, NslOption::FUNCTION_OPTIONS::FUNC_ON);
			else
				nsl_setAdcOverflowSaturation(handle, camOpt.nslDevConfig.overflowOpt, NslOption::FUNCTION_OPTIONS::FUNC_OFF);
				
			break;
		case 'f':
		case 'F':
			//overflow enable/disable
			nsl_getAdcOverflowSaturation(handle, &camOpt.nslDevConfig.overflowOpt, &camOpt.nslDevConfig.saturationOpt);

			if( camOpt.nslDevConfig.overflowOpt != NslOption::FUNCTION_OPTIONS::FUNC_ON )
				nsl_setAdcOverflowSaturation(handle, NslOption::FUNCTION_OPTIONS::FUNC_ON, camOpt.nslDevConfig.saturationOpt);
			else
				nsl_setAdcOverflowSaturation(handle, NslOption::FUNCTION_OPTIONS::FUNC_OFF, camOpt.nslDevConfig.saturationOpt);
			break;
		case 'h':
		case 'H':
			//Help
			printf("-----------------------------------------------\n");
			printf("b key : change GRAYSCALE mode\n");
			printf("d key : change DISTANCE & Grayscale mode\n");
			printf("a key : change AMPLITUDE & DISTANCE mode\n");
			printf("e key : change AMPLITUDE(log) & DISTANCE mode\n");
			printf("t key : change Grayscale LED\n");
			printf("s key : change saturation on/off\n");
			printf("f key : change overflow on/off\n");
			printf("0 key : change HDR off\n");
			printf("1 key : change HDR Spatial\n");
			printf("2 key : change HDR Temporal\n");
			printf("-----------------------------------------------\n");
			break;			
	}

	return key;
}



