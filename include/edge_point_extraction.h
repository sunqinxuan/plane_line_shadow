/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2019-05-09 15:16
#
# Filename:		edge_point_extraction.h
#
# Description: 
#
===============================================*/

#include "types.h"

namespace ulysses
{
	void visEdgePoints(Scan *scan, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis);

	class EdgePointExtraction : public pcl::OrganizedEdgeFromRGBNormals<pcl::PointXYZRGBA, pcl::Normal, pcl::Label>, public pcl::OrganizedMultiPlaneSegmentation<pcl::PointXYZRGBA, pcl::Normal, pcl::Label>
	{
	public:
		using OrganizedEdgeBase<pcl::PointXYZRGBA, pcl::Label>::EDGELABEL_NAN_BOUNDARY;
		using OrganizedEdgeBase<pcl::PointXYZRGBA, pcl::Label>::EDGELABEL_OCCLUDING;
		using OrganizedEdgeBase<pcl::PointXYZRGBA, pcl::Label>::EDGELABEL_OCCLUDED;
		using OrganizedEdgeBase<pcl::PointXYZRGBA, pcl::Label>::EDGELABEL_HIGH_CURVATURE;
		using OrganizedEdgeBase<pcl::PointXYZRGBA, pcl::Label>::EDGELABEL_RGB_CANNY;	

		EdgePointExtraction()
		{
			// edge detection;
			setDepthDisconThreshold (0.05);
			setMaxSearchNeighbors (50);
			setEdgeType (EDGELABEL_OCCLUDING | EDGELABEL_OCCLUDED | EDGELABEL_HIGH_CURVATURE); 
			// EDGELABEL_HIGH_CURVATURE | EDGELABEL_OCCLUDING | EDGELABEL_OCCLUDED
			remove("extract_EdgePoints.txt");
			sqRad_ANN=0.01;
			K_ANN=20;
			edge_meas=20;
			thres_ratio=0.5;
			thres_angle=0.5;
		}

		EdgePointExtraction(bool v, bool db, double th, int max, double pxl, double dist, double rad, int k) 
		{
			debug=db;
			visual=v;
			remove("extract_EdgePoints.txt");
			setDepthDisconThreshold (th);
			setMaxSearchNeighbors (max);
			setEdgeType (EDGELABEL_OCCLUDING | EDGELABEL_OCCLUDED | EDGELABEL_HIGH_CURVATURE); 
			thres_pxl_sq=pxl*pxl;
			thres_occluded_dist=dist;
			sqRad_ANN=rad*rad;
			K_ANN=k;
			edge_meas=100;
			thres_ratio=0.2;
			thres_angle=0.2;
		}

		~EdgePointExtraction() {}

		void setDebug(bool d) {debug=d;}

		void setThresPln(double min_inliers, double ang_thres, double dist_thres, double max_curv)
		{
			//double min_inliers=5000, ang_thres=0.017453*5.0, dist_thres=0.05, max_curv=0.01;
			// plane segmentation;
			setMinInliers(min_inliers);
			setAngularThreshold(ang_thres*0.017453); // 5deg
			setDistanceThreshold(dist_thres); // 5cm 
			setMaximumCurvature(max_curv);
			
		}

		void extractEdgePoints(Scan *scan, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis);

	private:
		
		std::ofstream fp;
		bool debug;
		bool visual;

		double thres_pxl_sq;
		double thres_occluded_dist;
		double thres_ratio, thres_angle;

		// radius=0.1m;
//		static constexpr double sqRad_ANN=0.01;
		double sqRad_ANN;
		// K=20;
		int K_ANN;

		double edge_meas;

		pcl::PointCloud<pcl::Label>::Ptr labels_plane;
		pcl::PointCloud<pcl::Label>::Ptr labels_edge;
//		std::vector<pcl::PointIndices> inlier_indices_plane;

		int num_plane;
		void segmentPlanes(Scan *scan);

	};

}

