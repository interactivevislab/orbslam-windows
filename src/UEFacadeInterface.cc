#pragma once

#include "System.h"
#include "UEFacadeInterface.h"

#include <Converter.h>

namespace ORB_SLAM2 {
	
	class SlamFacadeImpl : public ISlamFacade
	{

		ORB_SLAM2::System* SLAM;
	public:

		SlamFacadeImpl() {};
		
		void Init(char* inVocFile, char* inSettingsFile, eSensorExp sensorType = MONOCULAR)
		{
			SLAM = new ORB_SLAM2::System(std::string(inVocFile), std::string(inSettingsFile), System::eSensor(sensorType));
		}

		void TrackMonocular(const cv::Mat& image, double timestamp)
		{
			SLAM->TrackMonocular(image, timestamp);
		}

		void ShutdownSlam()
		{
			SLAM->Shutdown();
		}

		int GetNumPoints()
		{
			if (SLAM == nullptr || SLAM->GetMap() == nullptr) return 0;
			return SLAM->GetMap()->GetNumMapPoints();
		}
		
		void GetAllMapPoint(int& outCount, float* pointData, char* fileToWrite = "")
		{
			std::vector<ORB_SLAM2::MapPoint*> allMapPoints = SLAM->GetMap()->GetAllMapPoints();

			outCount = allMapPoints.size();

			//auto fileString = std::string(fileToWrite);
			auto fileString = std::string("D:\Test.ply");
			if (!fileString.empty())
			{
				std::ofstream ply;
				ply.open(fileString);
				ply << "ply" << std::endl;
				ply << "format ascii 1.0" << std::endl;
				ply << "element vertex " << allMapPoints.size() << std::endl;
				ply << "property float x" << std::endl;
				ply << "property float y" << std::endl;
				ply << "property float z" << std::endl;
				ply << "end_header" << std::endl;
				int i = 0;
				for (auto p : allMapPoints) {
					//if (p->isBad()) continue;
					Eigen::Matrix<double, 3, 1> v = ORB_SLAM2::Converter::toVector3d(p->GetWorldPos());
					ply << v.x() << std::endl;
					ply << v.y() << std::endl;
					ply << v.z() << std::endl;
				}
				ply.close();
			}
			
			for (int i = 0; i < outCount; i++)
			{
				auto p = allMapPoints[i];
				if (p->isBad())
				{
					pointData[i * 3 + 0] = 0;
					pointData[i * 3 + 1] = 0;
					pointData[i * 3 + 2] = 0;
				}
				else {
					Eigen::Matrix<double, 3, 1> v = ORB_SLAM2::Converter::toVector3d(p->GetWorldPos());
					pointData[i * 3 + 0] = v.x();
					pointData[i * 3 + 1] = v.y();
					pointData[i * 3 + 2] = v.z();
				}				
			}
		}
	};

	ISlamFacade* CreateSlamFacade(char* inVocFile, char* inSettingsFile, eSensorExp sensorType)
	{
		ISlamFacade* ret = new SlamFacadeImpl();
		ret->Init(inVocFile, inSettingsFile, sensorType);
		return ret;
	}
}
