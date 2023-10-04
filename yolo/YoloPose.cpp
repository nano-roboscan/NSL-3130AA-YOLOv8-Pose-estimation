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
#include "YoloPose.h"
#include "ImageTools.h"
#include "NSLFrame.h"

void YoloPose::init(const std::string &modelPath, float threshold, int modeltype) {

	modelType = modeltype;
	if( threshold > 0 ) modelScoreThreshold = threshold;

	net = cv::dnn::readNetFromONNX(modelPath);
#ifdef HAVE_CV_CUDA
	net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
	net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
#else	
	net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
	net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
#endif

	if( modeltype == YOLO_V8_POSE_TYPE ){
		printf("yolo8 pose threshold = %.1f, NMS = %.1f :: %s\n", modelScoreThreshold, modelNMSThreshold, modelPath.c_str());
	}
	else{
		printf("yolo8 pose detection threshold = %.1f, NMS = %.1f :: %s\n", modelScoreThreshold, modelNMSThreshold, modelPath.c_str());
	}
}

int YoloPose::detect(cv::Mat &mat) 
{
    static cv::Mat blob;
    static std::vector<cv::Mat> outputs;

    cv::dnn::blobFromImage(mat, blob, 1.0 / 255.0, modelShape, cv::Scalar(), true, false);
    net.setInput(blob);
    net.forward(outputs, net.getUnconnectedOutLayersNames());

    const int channels = outputs[0].size[2];
    const int anchors = outputs[0].size[1];
    outputs[0] = outputs[0].reshape(1, anchors);
    cv::Mat output = outputs[0].t();


    std::vector<cv::Rect> bboxList;
    std::vector<float> scoreList;
    std::vector<int> indicesList;
    std::vector<std::vector<Keypoint>> kpList;

    for (int i = 0; i < channels; i++) {
        auto row_ptr = output.row(i).ptr<float>();
        auto bbox_ptr = row_ptr;
        auto score_ptr = row_ptr + 4;
        auto kp_ptr = row_ptr + 5;

        float score = *score_ptr;
        if (score > modelScoreThreshold) {
            float x = *bbox_ptr++;
            float y = *bbox_ptr++;
            float w = *bbox_ptr++;
            float h = *bbox_ptr;

            float x0 = clamp((x - 0.5f * w) * 1.0F, 0.f, float(modelShape.width));
            float y0 = clamp((y - 0.5f * h) * 1.0F, 0.f, float(modelShape.height));
            float x1 = clamp((x + 0.5f * w) * 1.0F, 0.f, float(modelShape.width));
            float y1 = clamp((y + 0.5f * h) * 1.0F, 0.f, float(modelShape.height));

            cv::Rect_<float> bbox;
            bbox.x = x0;
            bbox.y = y0;
            bbox.width = x1 - x0;
            bbox.height = y1 - y0;

            std::vector<Keypoint> kps;

			/*
				0 : nose
				1 : left_eye
				2 : right_eye
				3 : left_ear
				4 : right_ear
				5 : left_shoulder
				6 : right_shoulder
				7 : left_elbow
				8 : right_elbow
				9 : left_wrist
				10 : right_wrist
				11 : left_hip
				12 : right_hip
				13 : left_knee
				14 : right_knee
				15 : left_ankle
				16 : right_ankle
			*/
			
            for (int k = 0; k < 17; k++) {

				if( k > 0 && k < 5 ) continue; // discard left_eye ~ right_ear
				
                float kps_x = (*(kp_ptr + 3 * k));
                float kps_y = (*(kp_ptr + 3 * k + 1));
                float kps_s = *(kp_ptr + 3 * k + 2);
                kps_x = clamp(kps_x, 0.f, float(modelShape.width));
                kps_y = clamp(kps_y, 0.f, float(modelShape.height));

                kps.emplace_back(kps_x, kps_y, kps_s);
            }

            bboxList.push_back(bbox);
            scoreList.push_back(score);
            kpList.push_back(kps);
        }
    }

    cv::dnn::NMSBoxes(
            bboxList,
            scoreList,
            modelScoreThreshold,
            modelNMSThreshold,
            indicesList
    );

    std::vector<YoloPose::Person> result{};
    for (auto &i: indicesList) {
        result.emplace_back(bboxList[i], scoreList[i], kpList[i]);
    }

	if( modelType == YOLO_V8_POSE_TYPE )
		ImageTools::drawPose(result, mat);
	else // YOLO_V8_POSE_DETECTION_TYPE
		ImageTools::draw(result, mat);
	
    return result.size();
}

YoloPose::Keypoint::Keypoint(float x, float y, float score) {
    this->position = cv::Point2d(x, y);
    this->conf = score;
}

YoloPose::Person::Person(cv::Rect2i _box, float _score, std::vector<Keypoint> &_kp) {
    this->box = _box;
    this->score = _score;
    this->kp = _kp;
}

inline static float clamp(float val, float min, float max) {
    return val > min ? (val < max ? val : max) : min;
}
