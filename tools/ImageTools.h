//
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

#ifndef YOLOV8_IMAGETOOLS_H
#define YOLOV8_IMAGETOOLS_H

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include "YoloDet.h"
#include "YoloPose.h"
#include "NSLFrame.h"

#ifdef DEEP_LEARNING

class ImageTools {
public :

	static void drawSkeleton(cv::Mat &image, YoloPose::Person &item);
	static void drawPose(std::vector<YoloPose::Person> &detections, cv::Mat &image);
    static void draw(std::vector<YoloPose::Person> &detections, cv::Mat &image);
    static void draw(std::vector<YoloDet::Detection> &detections, cv::Mat &image, cv::Scalar boxColor);

};

#endif

#endif //YOLOV8_IMAGETOOLS_H
