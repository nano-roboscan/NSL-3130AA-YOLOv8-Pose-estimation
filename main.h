//
// Created by mallumo on 6.5.2023.
//

/*
 * https://github.com/triple-Mu/YOLOv8-TensorRT/tree/main/csrc/pose/normal
 *
 * https://github.com/triple-Mu/YOLOv8-TensorRT/blob/main/csrc/pose/normal/include/yolov8-pose.hpp
 *
 *
 * https://github.com/ultralytics/ultralytics/tree/main/examples/YOLOv8-CPP-Inference
 *
 * yolo export model=yolov8n-pose.pt  format=onnx opset=12  //  1, 56, 8400
 * yolo export model=yolov8n.pt  format=onnx opset=12       //  1, 84, 8400
 *
 * in:  [1, 3, 640, 640]
 * out: [1, 56, 8400] / [1, 84, 8400]
 */

#ifndef YOLOV8_MAIN_H
#define YOLOV8_MAIN_H

#include <opencv2/opencv.hpp>
#include "nanolib.h"

#define HAVE_CV_CUDA
#define DEEP_LEARNING


#define YOLO_V8_POSE_TYPE				0
#define YOLO_V8_POSE_DETECTION_TYPE		1
#define YOLO_V8_DETECTION_TYPE			2
#define YOLO_V4_DETECTION_TYPE			3


typedef struct CaptureOptions_{
	// lidar parameter	
	int captureType;
	int integrationTime;
	int grayIntegrationTime;
	int minAmplitude;

	int medianFilterEnable;
	int averageFilterEnable;
	int	temporalFilterFactorActual;
	int	temporalFilterThreshold;
	int edgeThresHold;	
	int	interferenceUseLashValueEnable;
	int	interferenceLimit;

	int	modelType;
	int deeplearning;
	int	hdr_mode;
	int	dualbeamState;

	cv::Mat	frameMat;
	cv::Mat	distMat;
	double (*pCatesianTable)[NSL_LIDAR_TYPE_B_HEIGHT][NSL_LIDAR_TYPE_B_WIDTH];

	// deep learning parameter
	int inputSize;
	float detectThreshold;
	float minConfidence;
	float maxConfidence;
	float curConfidence;


	// display & running parameter	
	int	fpsCount;
	int displayFps;
	int maxDistance;	
	int detectingCnt;
	int nonDetectingCnt;
	int detectDistance;
	
	NslConfig	nslDevConfig;

}CaptureOptions;


#endif //YOLOV8_MAIN_H
