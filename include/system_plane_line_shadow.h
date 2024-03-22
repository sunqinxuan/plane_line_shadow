/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@gmail.com
#
# Last modified:	2019-05-12 10:38
#
# Filename:		system_plane_line_shadow.h
#
# Description: 
#
************************************************/
#include "types.h"
#include "data_reading.h"
#include "plane_feature_matching.h"
#include "line_feature_matching.h"
#include "motion_estimation_line.h"
#include "plane_param_estimation.h"
#include "plane_extraction.h"
#include "line_extraction.h"
#include "edge_point_extraction.h"
#include "global_map.h"
#include <sys/time.h>

namespace ulysses
{
	class SystemPlaneLineShadow
	{
	public:
		SystemPlaneLineShadow(const std::string &settingFile);

		bool trackCamera(const cv::Mat &rgb, const cv::Mat &depth, const double &timestamp,
							boost::shared_ptr<pcl::visualization::PCLVisualizer> vis,
							Transform Twc);

		void shutDown();
			
		void saveTraj(const std::string &file);
		void saveTraj(const std::string &file, bool between_frames);

		void useShadow(bool p) {motion_estimation->useShadow(p);}
		void useLineResidual(bool p) {motion_estimation->useLineResidual(p);}

		const Scan* getCurrentScan() const {return scan_cur;}
		const Transform getCurrentCamera() const 
		{
			return scan_cur->Tcg;
//			if(map->cameras.size()==0)
//			{
//				Transform Identity;
//				return Identity;
//			}
//			else
//				return map->cameras[map->cameras.size()-1];
		}
		void setDebug(bool d)
		{
			plane_extraction->setDebug(d);
			line_extraction->setDebug(d);
			plane_matching->setDebug(d);
			line_mathing->setDebug(d);
			motion_estimation->setDebug(d);
		}
		
	private:
		Scan *scan_cur, *scan_ref;
		Map *map;

		std::vector<double> timestamps;
		std::vector<Transform> traj;

		IntrinsicParam *intr_cam;
		int num_scan;
		bool isFirstFrame;
		int optimize_frames;

		PlaneExtraction *plane_extraction;
		LineExtraction *line_extraction;
		EdgePointExtraction *edge_extraction;
		PlaneParamEstimation *plane_fitting;
		PlaneFeatureMatching *plane_matching;
		LineFeatureMatching *line_mathing;
		MotionEstimation_Line *motion_estimation;
		GlobalMap *global_map;

		void loadScan(Scan *scan, const cv::Mat &rgb, const cv::Mat &depth);
	};
}
