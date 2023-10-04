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
#include "YoloDet.h"
#include "ImageTools.h"
#include "NSLFrame.h"

void YoloDet::init(const std::string &modelPath, const std::string &modelCfg, float threshold, int modeltype ) {

	modelType = modeltype;

	if( modelType == YOLO_V8_DETECTION_TYPE ){
		net = cv::dnn::readNetFromONNX(modelPath);
	}
	else{ // YOLO_V4_DETECTION_TYPE
		net = cv::dnn::readNetFromDarknet(modelCfg, modelPath);
	}

#ifdef HAVE_CV_CUDA
	net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
	net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
#else
	net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
	net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
#endif
	
	if( threshold > 0 ) modelScoreThreshold = threshold;

	if( modelType == YOLO_V8_DETECTION_TYPE ){
		printf("yolo8 detection threshold = %.1f, NMS = %.1f :: %s\n", modelScoreThreshold, modelNMSThreshold, modelPath.c_str());
	}
	else{
		printf("yolo4 detection threshold = %.1f, NMS = %.1f :: %s\n", modelScoreThreshold, modelNMSThreshold, modelPath.c_str());
	}
}

std::vector<cv::String> YoloDet::getOutputsNames(const cv::dnn::Net& net)
{
    std::vector<cv::String> names;
    if (names.empty())
    {
        //Get the indices of the output layers, i.e. the layers with unconnected outputs
        std::vector<int> outLayers = net.getUnconnectedOutLayers();
        
        //get the names of all the layers in the network
        std::vector<cv::String> layersNames = net.getLayerNames();
        
        // Get the names of the output layers in names
        names.resize(outLayers.size());
        for (int i = 0; i < outLayers.size(); ++i)
            names[i] = layersNames[outLayers[i] - 1];
    }
    return names;
}


int YoloDet::detect(cv::Mat &mat) {
    static cv::Mat blob;
    static std::vector<cv::Mat> outputs;
	std::vector<Detection> detections{};

	if( modelType == YOLO_V8_DETECTION_TYPE ){
		cv::dnn::blobFromImage(mat, blob, 1.0 / 255.0, model640Shape, cv::Scalar(), true, false);
		net.setInput(blob);
		net.forward(outputs, net.getUnconnectedOutLayersNames());
		
		int rows = outputs[0].size[2];
		int dimensions = outputs[0].size[1];
		
		outputs[0] = outputs[0].reshape(1, dimensions);
		cv::transpose(outputs[0], outputs[0]);
		
		float *data = (float *) outputs[0].data;
		
		std::vector<int> class_ids{};
		std::vector<float> confidences{};
		std::vector<cv::Rect> boxes{};
	
		for (int i = 0; i < rows; ++i) {
			float *classes_scores = data + 4;
		
			cv::Mat scores(1, classesCount, CV_32FC1, classes_scores);
			cv::Point class_id;
			double maxClassScore;
		
			minMaxLoc(scores, 0, &maxClassScore, 0, &class_id);

			if (maxClassScore > modelScoreThreshold && class_id.x == 0 ) { //seobi only person :: class_id.x == 0
				confidences.push_back(float(maxClassScore));
				class_ids.push_back(class_id.x);
		
				float x = data[0];
				float y = data[1];
				float w = data[2];
				float h = data[3];
		
				int left = int((x - 0.5 * w) * x_factor);
				int top = int((y - 0.5 * h) * y_factor);
		
				int width = int(w * x_factor);
				int height = int(h * y_factor);
		
				boxes.push_back(cv::Rect(left, top, width, height));
			}
			data += dimensions;
		}
		
		std::vector<int> nms_result;
		cv::dnn::NMSBoxes(boxes, confidences, modelScoreThreshold, modelNMSThreshold, nms_result);
				
		for (int idx: nms_result) {
			Detection result;
			result.class_id = class_ids[idx];
			result.confidence = confidences[idx];
			result.box = boxes[idx];

			detections.push_back(result);
		}

		ImageTools::draw(detections, mat, cv::Scalar(0, 255, 0));
	}
	else{ // YOLO_V4_DETECTION_TYPE
		cv::dnn::blobFromImage(mat, blob, 1.0 / 255.0, model512Shape, cv::Scalar(), true, false);
		net.setInput(blob);
		net.forward(outputs, getOutputsNames(net));
		
		std::vector<int> classIds;
		std::vector<float> confidences;
		std::vector<cv::Rect> boxes;
		
		for (int i = 0; i < outputs.size(); ++i)
		{
			// Scan through all the bounding boxes output from the network and keep only the
			// ones with high confidence scores. Assign the box's class label as the class
			// with the highest score for the box.
			float* data = (float*)outputs[i].data;
			for (int j = 0; j < outputs[i].rows; ++j, data += outputs[i].cols)
			{
				cv::Mat scores = outputs[i].row(j).colRange(5, outputs[i].cols);
				cv::Point classIdPoint;
				double confidence;
				// Get the value and location of the maximum score
				minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
				
				if (confidence > modelScoreThreshold && classIdPoint.x == 0)	//seobi only person :: classIdPoint.x == 0
				{
					int centerX = (int)(data[0] * mat.cols);
					int centerY = (int)(data[1] * mat.rows);
					int width = (int)(data[2] * mat.cols);
					int height = (int)(data[3] * mat.rows);
					int left = centerX - width / 2;
					int top = centerY - height / 2;
					
					classIds.push_back(classIdPoint.x);
					confidences.push_back((float)confidence);
					boxes.push_back(cv::Rect(left, top, width, height));
				}
			}
		}
		
		std::vector<int> indices;
		cv::dnn::NMSBoxes(boxes, confidences, modelScoreThreshold, modelNMSThreshold, indices);
		
		for (int idx = 0; idx < indices.size(); ++idx)
		{
			Detection result;
			result.class_id = classIds[idx];
			result.confidence = confidences[idx];
			result.box = boxes[idx];
					
			detections.push_back(result);
		}
		

		ImageTools::draw(detections, mat, cv::Scalar(128, 0, 255));
	}



    return detections.size();
}
