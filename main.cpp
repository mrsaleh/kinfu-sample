#include <Kinect.h>
#include <Windows.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/rgbd.hpp>
using namespace cv;

#include <fstream>
#include <iostream>
#include <thread>


class Sensor {
public:
	virtual bool getColorFrame(Mat& colorFrame) = 0;
	virtual bool getDepthFrame(Mat& depthFrame) = 0;
	virtual void getIntrinsics(float& fx, float& fy, float& px, float& py) = 0;
};

class KinectV2 : public Sensor {
private:
	IKinectSensor* m_pKinectSensor = nullptr;

	IDepthFrameSource* pDepthFrameSource = nullptr;
	IDepthFrameReader* m_pDepthFrameReader = nullptr;
	
	IColorFrameSource* pColorFrameSource = nullptr;
	IColorFrameReader* m_pColorFrameReader = nullptr;
	
	IDepthFrame* pDepthFrame = NULL;
	IColorFrame* pColorFrame = NULL;
	bool depthFrameSizeChanged = true;
	bool colorFrameSizeChanged = true;
	Mat depthMat;
	Mat colorMat;

public:
	KinectV2() {

		if (FAILED(GetDefaultKinectSensor(&m_pKinectSensor))) {
			throw std::runtime_error("Failed to get default sensor!");
		}		
			
		if (FAILED(m_pKinectSensor->Open())) {
			throw std::runtime_error("Failed to open default sensor!");
		}

		std::this_thread::sleep_for(std::chrono::seconds(2));
		BOOLEAN isAvailable = FALSE;
		do {

			if (FAILED(m_pKinectSensor->get_IsAvailable(&isAvailable))) {
				isAvailable = FALSE;
			}
		} while (isAvailable == FALSE);

		if(FAILED(m_pKinectSensor->get_DepthFrameSource(&pDepthFrameSource))){
			throw std::runtime_error("Failed to get depth sensor!");
		}

		if (FAILED(pDepthFrameSource->OpenReader(&m_pDepthFrameReader))) {
			throw std::runtime_error("Failed to open depth sensor!");
		}

		if (FAILED(m_pKinectSensor->get_ColorFrameSource(&pColorFrameSource))) {
			throw std::runtime_error("Failed to get color sensor!");
		}

		if (FAILED(pColorFrameSource->OpenReader(&m_pColorFrameReader))) {
			throw std::runtime_error("Failed to open color sensor!");
		}
		
	}
	virtual bool getColorFrame(Mat& colorMat) override {
		HRESULT hr2 = m_pColorFrameReader->AcquireLatestFrame(&pColorFrame);
		if (SUCCEEDED(hr2)) {
			if (colorFrameSizeChanged) {
				IFrameDescription* pFrameDescription = nullptr;
				int nWidth = 0;
				int nHeight = 0;

				pColorFrame->get_FrameDescription(&pFrameDescription);
				pFrameDescription->get_Width(&nWidth);
				pFrameDescription->get_Height(&nHeight);
				colorMat.create(nHeight, nWidth, CV_8UC4);

				/*pColorFrame->CopyConvertedFrameDataToArray(nWidth * nHeight * 4, depthMat.data, ColorImageFormat_Bgra);*/
				colorFrameSizeChanged = false;
				pFrameDescription->Release();
				pFrameDescription = nullptr;
			}

			pColorFrame->CopyConvertedFrameDataToArray(colorMat.rows * colorMat.cols * 4, (BYTE*)colorMat.data, ColorImageFormat::ColorImageFormat_Bgra);

			pColorFrame->Release();
			pColorFrame = nullptr;

			return true;
		}
		return false;
	}
	virtual bool getDepthFrame(Mat& depthMat) override {
		
		
		HRESULT hr1 = m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);
		
		if (SUCCEEDED(hr1)) {
			if (depthFrameSizeChanged) {
				IFrameDescription* pFrameDescription = nullptr;
				int nWidth = 0;
				int nHeight = 0;

				pDepthFrame->get_FrameDescription(&pFrameDescription);
				pFrameDescription->get_Width(&nWidth);
				pFrameDescription->get_Height(&nHeight);
				depthMat.create(nHeight, nWidth, CV_16UC1);				
				/*pColorFrame->CopyConvertedFrameDataToArray(nWidth * nHeight * 4, depthMat.data, ColorImageFormat_Bgra);*/
				depthFrameSizeChanged = false;
				pFrameDescription->Release();
				pFrameDescription = nullptr;
			}

			pDepthFrame->CopyFrameDataToArray(depthMat.rows * depthMat.cols , (UINT16*)depthMat.data);

			pDepthFrame->Release();
			pDepthFrame = nullptr;

			return true;
		}

		return false;
	}

	virtual void getIntrinsics(float& fx, float& fy, float& px, float& py) override {
		ICoordinateMapper* cm;
		if (FAILED(m_pKinectSensor->get_CoordinateMapper(&cm))) {
			throw std::runtime_error("Failed to get coordinate mapper.");
		}
		CameraIntrinsics depthIntrinsics;
		if (FAILED(cm->GetDepthCameraIntrinsics(&depthIntrinsics))) {
			throw std::runtime_error("Failed to read Kinect v2.0 intrinsics.");
		}		

		fx = depthIntrinsics.FocalLengthX;
		fy = depthIntrinsics.FocalLengthY;
		px = depthIntrinsics.PrincipalPointX;
		py = depthIntrinsics.PrincipalPointY;
	}

	~KinectV2() {

	}
};


enum struct DepthColorizeMode {
	Color,
	Grayscale
};

void export_to_ply(Mat points, Mat normals)
{
	static int i = 1;
	auto fname = std::to_string(i) + ".ply";
	i++;
	std::cout << "exporting to " << fname << std::endl;
	

	// Write the ply file
	std::ofstream out(fname );
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


int main(int argc, char** argv) {
	DepthColorizeMode depthColorizeMode = DepthColorizeMode::Color;

	KinectV2 kinv2;
	Mat depthFrame, colorFrame;
	Mat colorisdDepthFrame,adjustedDepthFrame;
		
	//Waits for sensor to get one depth frame to determine its size
	while (!kinv2.getDepthFrame(depthFrame));

	auto kinfu_params = kinfu::Params::defaultParams();
	float fx, fy, ppx, ppy;
	kinv2.getIntrinsics(fx, fy, ppx, ppy);
	kinfu_params->intr = Matx33f(
		fx, 0, ppx,
		0, fy, ppy,
		0, 0, 1);		
	kinfu_params->frameSize = Size(depthFrame.cols, depthFrame.rows);
	kinfu_params->depthFactor = 1000;			
	auto cvkf = kinfu::KinFu::create(kinfu_params);
	
	Mat points, normals;
	bool after_reset = false;
	UMat gpuDepthFrame(depthFrame.rows, depthFrame.cols, CV_16UC1);

	while (cv::waitKey(1) != 27) {
		if (kinv2.getDepthFrame(depthFrame)) {
			threshold(depthFrame, depthFrame, 1500, 5000, THRESH_TOZERO_INV);
			erode(depthFrame, depthFrame, Mat());
			if (depthColorizeMode == DepthColorizeMode::Color) {
				double min;
				double max;
				cv::minMaxIdx(depthFrame, &min, &max);
				// Histogram Equalization
				float scale = 255 / (max - min);
				depthFrame.convertTo(adjustedDepthFrame, CV_8UC1, scale, -min * scale);
				applyColorMap(adjustedDepthFrame, colorisdDepthFrame, COLORMAP_TURBO);
				imshow("win", colorisdDepthFrame);
			}
			else {
				depthFrame.convertTo(colorisdDepthFrame, CV_8U, 1.0 / 255);
				imshow("Depth", colorisdDepthFrame);
			}
			
			depthFrame.copyTo(gpuDepthFrame);
			
			UMat gpuPoints, gpuNormals;


			//Kinnect Fusion saves cloud on failure
			if (! cvkf->update(gpuDepthFrame)) {
				cvkf->reset();
				if (!points.empty() && !normals.empty()) {
					export_to_ply(points, normals);
				}
				after_reset = true;
				gpuPoints.release();
				gpuNormals.release();
				std::cout << "reset" << std::endl;								
			}
			
			// Get current pointcloud
			if (!after_reset)
			{
				cvkf->getCloud(gpuPoints, gpuNormals);
			}

			if (!gpuPoints.empty() && !gpuNormals.empty()) {

				gpuPoints.copyTo(points);
				gpuNormals.copyTo(normals);
				gpuPoints.release();
				gpuNormals.release();
			}
			after_reset = false;
		}
	}

	cvkf->getCloud(points, normals);
	export_to_ply(points, normals);
	
}