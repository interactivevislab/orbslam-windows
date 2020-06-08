#pragma once

#ifdef ORB_SLAM2_EXPORTS
#define ORB_SLAM2_API __declspec(dllexport)
#else
#define ORB_SLAM2_API __declspec(dllimport)
#endif

#include "opencv2/core.hpp"

	
namespace ORB_SLAM2
{	
	extern "C"
	{
		enum eSensorExp {
			MONOCULAR = 0,
			STEREO = 1,
			RGBD = 2
		};

		class ORB_SLAM2_API ISlamFacade
		{
		public:
			virtual void Init(char* inVocFile, char* inSettingsFile, eSensorExp sensorType = MONOCULAR) = 0;
			virtual void TrackMonocular(const cv::Mat& image, double timestamp) = 0;
			virtual void ShutdownSlam() = 0;
			virtual int GetNumPoints() = 0;
			virtual void GetAllMapPoint(int& outCount, float* pointData, char* fileToWrite = "") = 0;

		};

		ORB_SLAM2_API ISlamFacade* CreateSlamFacade(char* inVocFile, char* inSettingsFile, eSensorExp sensorType = MONOCULAR);
	}	

}