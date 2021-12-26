#ifndef REAL_SENSE_V2
#define REAL_SENSE_V2

#include "Sensor.hpp"

#include "librealsense2/rs.hpp"


class RealsenseV2 : public Sensor {
private:
	rs2::pipeline pipe;
	rs2::pipeline_profile profile;
	rs2::frameset frames;
	rs2_intrinsics depthIntrinsics;
	rs2_intrinsics colorIntrinsics;
	cv::Mat depthFrame;
	cv::Mat colorFrame;
private:
	bool updateFrameset() {
		return pipe.poll_for_frames(&frames);			
	}
public:
	RealsenseV2() {
		try {
			pipe.start();
			profile = pipe.get_active_profile();
			auto depth_stream = profile.get_stream(RS2_STREAM_DEPTH)
				.as<rs2::video_stream_profile>();
			depthIntrinsics = depth_stream.get_intrinsics();

			auto color_stream = profile.get_stream(RS2_STREAM_COLOR)
				.as<rs2::video_stream_profile>();
			colorIntrinsics = color_stream.get_intrinsics();
		}
		catch (std::exception& exp) {
			std::cout << exp.what() << std::endl;
		}
	}


	virtual bool getColorFrame(cv::Mat& colorFrame) override {
		if (!updateFrameset())
			return false;

		auto f = frames.get_color_frame();
		auto w = f.get_width();
		auto h = f.get_height();
		cv::Mat image(cv::Size(w, h), CV_8UC3, (void*)f.get_data(), cv::Mat::AUTO_STEP);
		colorFrame.create(cv::Size(w, h), CV_8UC3);
		image.copyTo(colorFrame);
		return true;
	}

	virtual bool getDepthFrame(cv::Mat& depthFrame) override {
		if (!updateFrameset())
			return false;		
		auto f = frames.get_depth_frame();
		auto w = f.get_width();
		auto h = f.get_height();
		cv::Mat image(cv::Size(w, h), CV_16UC1, (void*)f.get_data(), cv::Mat::AUTO_STEP);
		depthFrame.create(cv::Size(w, h), CV_16UC1);
		image.copyTo(depthFrame);
		return true;
	}
	virtual void getIntrinsics(float& fx, float& fy, float& ppx, float& ppy) {
		fx = depthIntrinsics.fx;
		fy = depthIntrinsics.fy;
		ppx = depthIntrinsics.ppx;
		ppy = depthIntrinsics.ppy;
	}

	~RealsenseV2() {
		pipe.stop();
	}
};

#endif