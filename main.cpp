

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/rgbd.hpp>
#include <opencv2/calib3d.hpp>

#include <fstream>
#include <iostream>
#include <thread>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

#include "Sensor.hpp"
#include "KinectV2.hpp"
#include "RealsenseV2.hpp"


enum struct DepthColorizeMode {
	Color,
	Grayscale
};

void export_to_ply(cv::Mat points, cv::Mat normals)
{
	static int i = 1;
	auto fname = std::to_string(i) + ".ply";
	i++;
	std::cout << "exporting to " << fname << std::endl;


	// Write the ply file
	std::ofstream out(fname);
	out << "ply\n";
	out << "format binary_little_endian 1.0\n";
	out << "comment pointcloud saved from Realsense Viewer\n";
	out << "element vertex " << points.rows << "\n";
	out << "property float" << sizeof(float) * 8 << " x\n";
	out << "property float" << sizeof(float) * 8 << " y\n";
	out << "property float" << sizeof(float) * 8 << " z\n";

	out << "property float" << sizeof(float) * 8 << " nx\n";
	out << "property float" << sizeof(float) * 8 << " ny\n";
	out << "property float" << sizeof(float) * 8 << " nz\n";

	out << "end_header\n";
	out.close();

	out.open(fname, std::ios_base::app | std::ios_base::binary);
	for (int i = 0; i < points.rows; i++)
	{
		// write vertices
		out.write(reinterpret_cast<const char*>(&(points.at<float>(i, 0))), sizeof(float));
		out.write(reinterpret_cast<const char*>(&(points.at<float>(i, 1))), sizeof(float));
		out.write(reinterpret_cast<const char*>(&(points.at<float>(i, 2))), sizeof(float));

		// write normals
		out.write(reinterpret_cast<const char*>(&(normals.at<float>(i, 0))), sizeof(float));
		out.write(reinterpret_cast<const char*>(&(normals.at<float>(i, 1))), sizeof(float));
		out.write(reinterpret_cast<const char*>(&(normals.at<float>(i, 2))), sizeof(float));
	}
}

void saveConfigFile(cv::Ptr<cv::kinfu::Params> kinfu_params) {
	std::ofstream configFile("config.ini");
	configFile << "DepthFactor" << "=" << kinfu_params->depthFactor << std::endl;
	configFile << "BilateralSigmaDepth" << "=" << kinfu_params->bilateral_sigma_depth << std::endl;
	configFile << "BilateralSigmaSpatial" << "=" << kinfu_params->bilateral_sigma_spatial << std::endl;
	configFile << "BilateralKernelSize" << "=" << kinfu_params->bilateral_kernel_size << std::endl;

	configFile << "PyramidLevels" << "=";
	for (int i = 0; i < kinfu_params->icpIterations.size(); i++) {
		if (i != 0)
			configFile << ",";
		configFile << kinfu_params->icpIterations[i];
	}
	configFile << std::endl;

	configFile << "VolumeDimX" << "=" << kinfu_params->volumeDims[0] << std::endl;
	configFile << "VolumeDimY" << "=" << kinfu_params->volumeDims[1] << std::endl;
	configFile << "VolumeDimZ" << "=" << kinfu_params->volumeDims[2] << std::endl;
	configFile << "VoxelSize" << "=" << kinfu_params->voxelSize << std::endl;
	configFile << "TSDFMinCameraMovement" << "=" << kinfu_params->tsdf_min_camera_movement << std::endl;
	configFile << "TSDFMaxWeight" << "=" << kinfu_params->tsdf_max_weight << std::endl;
	configFile << "TSDFTruncateDistance" << "=" << kinfu_params->tsdf_trunc_dist << std::endl;
	configFile << "RaycastStepFactor" << "=" << kinfu_params->raycast_step_factor << std::endl;
	configFile << "LightPoseX" << "=" << kinfu_params->lightPose[0] << std::endl;
	configFile << "LightPoseY" << "=" << kinfu_params->lightPose[1] << std::endl;
	configFile << "LightPoseZ" << "=" << kinfu_params->lightPose[2] << std::endl;
	configFile << "ICPDistanceThresh" << "=" << kinfu_params->icpDistThresh << std::endl;
	configFile << "ICPAngleThresh" << "=" << kinfu_params->icpAngleThresh << std::endl;
	configFile << "TruncateThreshold" << "=" << kinfu_params->truncateThreshold << std::endl;
	configFile.close();


}

void applyConfigFile(cv::Ptr<cv::kinfu::Params> kinfu_params) {
	//Read kinfu config from file
	po::options_description options_desc("Usage:");
	options_desc.add_options()
		("DepthFactor", po::value<float>())
		("BilateralSigmaDepth", po::value<float>())
		("BilateralSigmaSpatial", po::value<float>())
		("BilateralKernelSize", po::value<int>())
		("PyramidLevels", po::value<std::string>())
		("VolumeDimX", po::value<int>())
		("VolumeDimY", po::value<int>())
		("VolumeDimZ", po::value<int>())
		("VoxelSize", po::value<float>())
		("TSDFMinCameraMovement", po::value<float>())
		("TSDFMaxWeight", po::value<int>())
		("TSDFTruncateDistance", po::value<float>())
		("RaycastStepFactor", po::value<float>())
		("LightPoseX", po::value<float>())
		("LightPoseY", po::value<float>())
		("LightPoseZ", po::value<float>())
		("ICPDistanceThresh", po::value<float>())
		("ICPAngleThresh", po::value<float>())
		("TruncateThreshold", po::value<float>())
		;


	auto config = po::parse_config_file("config.ini", options_desc);
	po::variables_map vm;
	po::store(config, vm);
	po::notify(vm);

	if (vm.count("DepthFactor"))
		kinfu_params->depthFactor = vm["DepthFactor"].as<float>();
	if (vm.count("BilateralSigmaDepth"))
		kinfu_params->bilateral_sigma_depth = vm["BilateralSigmaDepth"].as<float>();
	if (vm.count("BilateralSigmaSpatial"))
		kinfu_params->bilateral_sigma_spatial = vm["BilateralSigmaSpatial"].as<float>();
	if (vm.count("BilateralKernelSize"))
		kinfu_params->bilateral_kernel_size = vm["BilateralKernelSize"].as<int>();
	if (vm.count("PyramidLevels")) {
		auto icp_iterations = vm["PyramidLevels"].as<std::string>();
		
		kinfu_params->icpIterations.clear();

		do {
			auto i = icp_iterations.find(',',0);
			if (i == -1)
				break;
			std::string  s = icp_iterations.substr(0, i );
			icp_iterations = icp_iterations.substr(i+1);
			kinfu_params->icpIterations.push_back(std::stoi(s));
		} while (true);
		if (icp_iterations.length() > 0) {
			auto s = icp_iterations;
			kinfu_params->icpIterations.push_back(std::stoi(s));
		}
		kinfu_params->pyramidLevels = kinfu_params->icpIterations.size();
	}

	if (vm.count("VolumeDimX"))
		kinfu_params->volumeDims[0] = vm["VolumeDimX"].as<int>();
	if (vm.count("VolumeDimY"))
		kinfu_params->volumeDims[1] = vm["VolumeDimY"].as<int>();
	if (vm.count("VolumeDimZ"))
		kinfu_params->volumeDims[2] = vm["VolumeDimZ"].as<int>();
	if (vm.count("VoxelSize"))
		kinfu_params->voxelSize = vm["VoxelSize"].as<float>();
	if (vm.count("TSDFMinCameraMovement"))
		kinfu_params->tsdf_min_camera_movement = vm["TSDFMinCameraMovement"].as<float>();
	if (vm.count("TSDFMaxWeight"))
		kinfu_params->tsdf_max_weight = vm["TSDFMaxWeight"].as<int>();
	if (vm.count("TSDFTruncateDistance"))
		kinfu_params->tsdf_trunc_dist = vm["TSDFTruncateDistance"].as<float>();
	if (vm.count("RaycastStepFactor"))
		kinfu_params->raycast_step_factor = vm["RaycastStepFactor"].as<float>();
	if (vm.count("LightPoseX"))
		kinfu_params->lightPose[0] = vm["LightPoseX"].as<float>();
	if (vm.count("LightPoseY"))
		kinfu_params->lightPose[1] = vm["LightPoseY"].as<float>();
	if (vm.count("LightPoseZ"))
		kinfu_params->lightPose[2] = vm["LightPoseZ"].as<float>();
	if (vm.count("ICPDistanceThresh"))
		kinfu_params->icpDistThresh = vm["ICPDistanceThresh"].as<float>();
	if (vm.count("ICPAngleThresh"))
		kinfu_params->icpAngleThresh = vm["ICPAngleThresh"].as<float>();
	if (vm.count("TruncateThreshold"))
		kinfu_params->truncateThreshold = vm["TruncateThreshold"].as<float>();

}

bool fileExist(const char* fileName)
{
	std::ifstream infile(fileName);
	return infile.good();
}

void test_values() {
	Sensor* cam = new KinectV2();
	Sensor* cam2 = new RealsenseV2();
	cv::Mat kin_depth, rs_depth;
	while (!cam->getDepthFrame(kin_depth)) {}
	while (!cam2->getDepthFrame(rs_depth)) {}
	double kin_min = 0;
	double kin_max = 0;
	double rs_min = 0;
	double rs_max = 0;
	cv::minMaxIdx(kin_depth, &kin_min, &kin_max);
	cv::minMaxIdx(rs_depth, &rs_min, &rs_max);

}


float rawDepthToMeters(int depthValue) {
	//if (depthValue < 2047) {
	if (depthValue == 0)
		return 0;

	if (depthValue < 1500) {
		auto result = (float)((double)(depthValue) * 0.001);
		return result;
	}
	return 0.0f;
}

int main(int argc, char** argv) {
	DepthColorizeMode depthColorizeMode = DepthColorizeMode::Color;

	Sensor* cam = new KinectV2();
	//Sensor* cam = new RealsenseV2();
	//test_values();

	cv::Mat depthFrame, colorFrame, prevDepthFrame;
	cv::Mat colorisdDepthFrame, adjustedDepthFrame;

	cv::setUseOptimized(true);

	cv::rgbd::DepthCleaner depthCleaner;
	

	//Waits for sensor to get one depth frame to determine its size
	while (!cam->getDepthFrame(depthFrame));

	auto kinfu_params = cv::kinfu::Params::defaultParams();
	float fx, fy, ppx, ppy;
	kinfu_params->frameSize = cv::Size(depthFrame.cols, depthFrame.rows);

	cam->getIntrinsics(fx, fy, ppx, ppy);
	kinfu_params->intr = cv::Matx33f(
		fx, 0, ppx,
		0, fy, ppy,
		0, 0, 1);
	kinfu_params->depthFactor = 1000;

	//kinfu_params->truncateThreshold = 1.5;

	try {
		if (fileExist("config.ini")) {
			applyConfigFile(kinfu_params);
		}
		else {
			saveConfigFile(kinfu_params);
		}
	}
	catch (std::exception& exp) {
		std::cout << exp.what() << std::endl;
	}

	auto cvkf = cv::kinfu::KinFu::create(kinfu_params);


	cv::Mat _points, _normals;
	bool after_reset = false;
	cv::UMat gpuDepthFrame(depthFrame.rows, depthFrame.cols, CV_16UC1);
	cv::UMat gpuRenderImage;
	cv::UMat renderImage;
	prevDepthFrame = cv::Mat::zeros(depthFrame.rows, depthFrame.cols, CV_16UC1);

	//int i = 0;
	while (cv::waitKey(1) != 27) {
		if (cam->getDepthFrame(depthFrame)) {
			//std::cout << ++i << std::endl;
			//threshold(depthFrame, depthFrame, 1500, 0, cv::THRESH_TOZERO_INV);

			auto h = depthFrame.rows;
			auto w = depthFrame.cols;


			for (int x = 0; x < w; x++) {
				for (int y = 0; y < h; y++) {
					if (depthFrame.at<UINT16>(cv::Point(x, y)) > 2000) {
						depthFrame.at<UINT16>(cv::Point(x, y)) = 0;
					}
					if (depthFrame.at<UINT16>(cv::Point(x, y)) < 500) {
						depthFrame.at<UINT16>(cv::Point(x, y)) = 0;
					}
				}
			}

						
			

			for (int x = 0; x < w; x++) {
				for (int y = 0; y < h; y++) {
					if (depthFrame.at<UINT16>(cv::Point(x, y)) > 1500) {
						depthFrame.at<UINT16>(cv::Point(x, y)) = 0;
					}
				}
			}

			if (depthColorizeMode == DepthColorizeMode::Color) {
				double min = 0;
				double max = 1500;
				//cv::minMaxIdx(depthFrame, &min, &max);
				//cv::Mat dd = prevDepthFrame - depthFrame;
				cv::Mat dd = depthFrame;
				// Histogram Equalization
				float scale = 255.0 / (max - min);
				dd.convertTo(adjustedDepthFrame, CV_8UC1, scale, -min * scale);
				applyColorMap(adjustedDepthFrame, colorisdDepthFrame, cv::COLORMAP_JET);
				imshow("Depth Diff", adjustedDepthFrame);
				//imshow("win", colorisdDepthFrame);
			}
			else {
				depthFrame.convertTo(colorisdDepthFrame, CV_8U, 1.0 / 255.0);
				imshow("Depth", colorisdDepthFrame);
			}

			cv::UMat points, normals;
			depthFrame.copyTo(gpuDepthFrame);
			//Kinect Fusion saves cloud on failure
			if (cvkf->update(gpuDepthFrame)) {
				cvkf->render(gpuRenderImage);
				imshow("render", gpuRenderImage);
			}else {
				cvkf->reset();
				export_to_ply(_points, _normals);
				after_reset = true;
				points.release();
				normals.release();
				std::cout << "Reset" << std::endl;
			}

			if (!after_reset) {
				cvkf->getCloud(points, normals);
			}

			if (!points.empty() && !normals.empty()) {
				points.copyTo(_points);
				points.release();
				normals.copyTo(_normals);
				normals.release();
			}

			after_reset = false;

			depthFrame.copyTo(prevDepthFrame);
		}
	}

	cvkf->getCloud(_points, _normals);
	export_to_ply(_points, _normals);

}