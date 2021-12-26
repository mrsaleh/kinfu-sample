#pragma once

#include <opencv2/core.hpp>


class Sensor {
public:
	virtual bool getColorFrame(cv::Mat& colorFrame) = 0;
	virtual bool getDepthFrame(cv::Mat& depthFrame) = 0;
	virtual void getIntrinsics(float& fx, float& fy, float& ppx, float& ppy) = 0;
};