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
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>

#ifdef DEEP_LEARNING
const int NUM_CLASSES = 1;

void YoloDet::init(const std::string &modelPath, const std::string &modelCfg, float threshold, int modeltype ) 
{
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

int YoloDet::detect(cv::Mat &mat) 
{
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
#if 0
		cv::dnn::DetectionModel model = cv::dnn::DetectionModel(net);
		model.setInputParams(1 / 255.0, model512Shape, cv::Scalar(), true);

		std::vector<int> classIds;
		std::vector<float> scores;
		std::vector<cv::Rect> boxes;
		model.detect(mat, classIds, scores, boxes, modelScoreThreshold, modelNMSThreshold);

		for (int idx = 0; idx < classIds.size() && classIds[idx] == 0 ; ++idx)
		{
			Detection result;
			result.class_id = classIds[idx];
			result.confidence = scores[idx];
			result.box = boxes[idx];
					
			detections.push_back(result);
		}
#else	
		cv::dnn::blobFromImage(mat, blob, 1.0 / 255.0, model512Shape, cv::Scalar(), true, false, CV_32F);
		net.setInput(blob);
		net.forward(outputs, net.getUnconnectedOutLayersNames());

        std::vector<int> indices[NUM_CLASSES];
        std::vector<cv::Rect> boxes[NUM_CLASSES];
        std::vector<float> scores[NUM_CLASSES];

        for (auto& output : outputs)
        {
            const auto num_boxes = output.rows;
            for (int i = 0; i < num_boxes; i++)
            {
                auto x = output.at<float>(i, 0) * mat.cols;
                auto y = output.at<float>(i, 1) * mat.rows;
                auto width = output.at<float>(i, 2) * mat.cols;
                auto height = output.at<float>(i, 3) * mat.rows;
                cv::Rect rect(x - width/2, y - height/2, width, height);

                for (int c = 0; c < NUM_CLASSES; c++)
                {
                    auto confidence = *output.ptr<float>(i, 5 + c);
                    if (confidence >= modelScoreThreshold)
                    {
                        boxes[c].push_back(rect);
                        scores[c].push_back(confidence);
                    }
                }
            }
        }
		
        for (int c = 0; c < NUM_CLASSES; c++)
        {
            cv::dnn::NMSBoxes(boxes[c], scores[c], 0.0, modelNMSThreshold, indices[c]);

			for (int idx = 0; idx < indices[c].size(); ++idx)
			{
				Detection result;
				result.class_id = c;
				result.confidence = scores[c][idx];
				result.box = boxes[c][idx];
						
				detections.push_back(result);
			}
        }
#endif
		ImageTools::draw(detections, mat, cv::Scalar(128, 0, 255));
		
	}



    return detections.size();
}

#endif

