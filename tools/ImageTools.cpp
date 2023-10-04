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

#include "ImageTools.h"


void ImageTools::drawSkeleton(cv::Mat &image, YoloPose::Person &item)
{
    auto poseColor = cv::Scalar(0,0,255);
	int thickness = 2;
	
	/*
		0 : nose
		1 : left_shoulder
		2 : right_shoulder
		3 : left_elbow
		4 : right_elbow
		5 : left_wrist
		6 : right_wrist
		7 : left_hip
		8 : right_hip
		9 : left_knee
		10 : right_knee
		11 : left_ankle
		12 : right_ankle
	*/
	for (YoloPose::Keypoint kp:item.kp) {
		cv::circle(image, kp.position, 3, poseColor, cv::FILLED);
	}
	
	cv::Point2d center_shoulder;
	center_shoulder.x = item.kp[1].position.x + (item.kp[2].position.x - item.kp[1].position.x)/2 ;
	center_shoulder.y = item.kp[1].position.y;
	
	cv::Point2d center_hip;
	center_hip.x = item.kp[7].position.x + (item.kp[8].position.x - item.kp[7].position.x)/2 ;
	center_hip.y = item.kp[7].position.y;
	
	cv::circle(image, center_shoulder, 3, poseColor, cv::FILLED);
	cv::circle(image, center_hip, 3, poseColor, cv::FILLED);

	// nose - shoulder
	cv::line(image, item.kp[0].position, center_shoulder, poseColor, thickness);
	
	// shoulder
	cv::line(image, item.kp[1].position, item.kp[2].position, poseColor, thickness);
	// left arm
	cv::line(image, item.kp[1].position, item.kp[3].position, poseColor, thickness);
	cv::line(image, item.kp[3].position, item.kp[5].position, poseColor, thickness);
	// right arm
	cv::line(image, item.kp[2].position, item.kp[4].position, poseColor, thickness);
	cv::line(image, item.kp[4].position, item.kp[6].position, poseColor, thickness);
	
	// center shoulder -> center hip
	cv::line(image, center_shoulder, center_hip, poseColor, thickness);
	
	// hip
	cv::line(image, item.kp[7].position, item.kp[8].position, poseColor, thickness);
	// left leg
	cv::line(image, item.kp[7].position, item.kp[9].position, poseColor, thickness);
	cv::line(image, item.kp[9].position, item.kp[11].position, poseColor, thickness);
	// right leg
	cv::line(image, item.kp[8].position, item.kp[10].position, poseColor, thickness);
	cv::line(image, item.kp[10].position, item.kp[12].position, poseColor, thickness);
}


void ImageTools::drawPose(std::vector<YoloPose::Person> &detections, cv::Mat &image) {
    auto textColor = cv::Scalar(255, 255, 255);
    auto boxColor = cv::Scalar(0, 255, 0);
	int thickness = 2;

    for (YoloPose::Person &item: detections) {
        cv::rectangle(image, item.box, boxColor, thickness);

        std::string infoString = std::to_string(item.score);
        cv::Size textSize = cv::getTextSize(infoString, cv::QT_FONT_NORMAL, 1, 1, nullptr);
        cv::Rect textBox(item.box.x, item.box.y - 40, textSize.width + 10, textSize.height + 20);

        cv::rectangle(image, textBox, boxColor, cv::FILLED);
        cv::putText(image, infoString, cv::Point(item.box.x + 5, item.box.y - 10), cv::FONT_HERSHEY_DUPLEX, 1, textColor, 1, 0);

		drawSkeleton(image, item);
    }
}

void ImageTools::draw(std::vector<YoloPose::Person> &detections, cv::Mat &image) {
    auto textColor = cv::Scalar(255, 255, 255);
    auto boxColor = cv::Scalar(0, 255, 0);
	int thickness = 2;

    for (YoloPose::Person &item: detections) {
        cv::rectangle(image, item.box, boxColor, thickness);

        std::string infoString = std::to_string(item.score);
        cv::Size textSize = cv::getTextSize(infoString, cv::QT_FONT_NORMAL, 1, 1, nullptr);
        cv::Rect textBox(item.box.x, item.box.y - 40, textSize.width + 10, textSize.height + 20);

        cv::rectangle(image, textBox, boxColor, cv::FILLED);
        cv::putText(image, infoString, cv::Point(item.box.x + 5, item.box.y - 10), cv::FONT_HERSHEY_DUPLEX, 1, textColor, 1, 0);
    }
}


void ImageTools::draw(std::vector<YoloDet::Detection> &detections, cv::Mat &image, cv::Scalar boxColor) {
    auto textColor = cv::Scalar(255, 255, 255);
	int thickness = 2;

    for (YoloDet::Detection &item: detections) {
        cv::Rect box = item.box;

        cv::rectangle(image, box, boxColor, thickness);

        std::string infoString = std::to_string(item.confidence).substr(0, 4);
        cv::Size textSize = cv::getTextSize(infoString, cv::FONT_HERSHEY_DUPLEX, 1, 1, nullptr);
        cv::Rect textBox(box.x, box.y - 40, textSize.width + 10, textSize.height + 20);

        cv::rectangle(image, textBox, boxColor, cv::FILLED);
        cv::putText(image, infoString, cv::Point(box.x + 5, box.y - 10), cv::FONT_HERSHEY_DUPLEX, 1, textColor, 1, 0);
    }
}



