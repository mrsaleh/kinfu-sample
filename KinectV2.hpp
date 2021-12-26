#ifndef KINECT_V2_HPP
#define KINECT_V2_HPP

#include <Kinect.h>

#include "Sensor.hpp"

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
	cv::Mat depthMat;
	cv::Mat colorMat;

	CameraIntrinsics depthIntrinsics;

public:
	KinectV2() {

		depthIntrinsics = { 0 };

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

		if (FAILED(m_pKinectSensor->get_DepthFrameSource(&pDepthFrameSource))) {
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

		ICoordinateMapper* cm;
		if (FAILED(m_pKinectSensor->get_CoordinateMapper(&cm))) {
			throw std::runtime_error("Failed to get coordinate mapper.");
		}

		if (FAILED(cm->GetDepthCameraIntrinsics(&depthIntrinsics))) {
			throw std::runtime_error("Failed to read Kinect v2.0 intrinsics.");
		}	
	}

	virtual bool getColorFrame(cv::Mat& colorMat) override {

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
	virtual bool getDepthFrame(cv::Mat& depthMat) override {

		HRESULT hr1 = m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);

		if (SUCCEEDED(hr1)) {
			//if (depthFrameSizeChanged) {
			//	IFrameDescription* pFrameDescription = nullptr;
			//	int nWidth = 0;
			//	int nHeight = 0;

			//	pDepthFrame->get_FrameDescription(&pFrameDescription);
			//	pFrameDescription->get_Width(&nWidth);
			//	pFrameDescription->get_Height(&nHeight);
			//	depthMat.create(nHeight, nWidth, CV_16UC1);
			//	/*pColorFrame->CopyConvertedFrameDataToArray(nWidth * nHeight * 4, depthMat.data, ColorImageFormat_Bgra);*/
			//	depthFrameSizeChanged = false;
			//	pFrameDescription->Release();
			//	pFrameDescription = nullptr;
			//}
			//pDepthFrame->CopyFrameDataToArray(depthMat.rows * depthMat.cols, (UINT16*)depthMat.data);

				IFrameDescription* pFrameDescription = nullptr;
				int nWidth = 0;
				int nHeight = 0;
				pDepthFrame->get_FrameDescription(&pFrameDescription);
				pFrameDescription->get_Width(&nWidth);
				pFrameDescription->get_Height(&nHeight);
				cv::Mat f;
				f.create(nHeight, nWidth, CV_16UC1);
				pDepthFrame->CopyFrameDataToArray(depthMat.rows * depthMat.cols, (UINT16*)f.data);
				depthMat.create(nHeight, nWidth, CV_16UC1);
				f.copyTo(depthMat);
				pDepthFrame->Release();

			//pDepthFrame->Release();
			//pDepthFrame = nullptr;

			return true;
		}

		return false;
	}

	virtual void getIntrinsics(float& fx, float& fy, float& ppx, float& ppy) override {		
		fx = depthIntrinsics.FocalLengthX;
		fy = depthIntrinsics.FocalLengthY;
		ppx = depthIntrinsics.PrincipalPointX;
		ppy = depthIntrinsics.PrincipalPointY;
	}

	~KinectV2() {

	}
};

#endif