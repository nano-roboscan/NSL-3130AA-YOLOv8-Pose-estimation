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
#ifndef YOLOV8_YOLOPOSE_H
#define YOLOV8_YOLOPOSE_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include "NSLFrame.h"

#ifdef DEEP_LEARNING

class YoloPose {
private:
    cv::dnn::Net net;
#if 1	//seobi
	const cv::Size modelShape = cv::Size(640, 480);
	float modelScoreThreshold{0.60};
	float modelNMSThreshold{0.50};
#else
    const cv::Size modelShape = cv::Size(640, 640);
    float modelScoreThreshold{0.6};
    float modelNMSThreshold{0.50};
#endif
	int modelType;
public:
    struct Keypoint {
        Keypoint(float x, float y, float score);

        cv::Point2d position{};
        float conf{0.0};
    };

    struct Person {
        Person(cv::Rect2i _box, float _score, std::vector<Keypoint> &_kp);

        cv::Rect2i box{};
        float score{0.0};
        std::vector<Keypoint> kp{};
    };

    void init(const std::string &modelPath, float threshold, int modeltype);

    int detect(cv::Mat &mat);
};

inline static float clamp(float val, float min, float max);

#endif

#endif //YOLOV8_YOLOPOSE_H
