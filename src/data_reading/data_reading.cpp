/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2019-05-10 12:49
#
# Filename:		data_reading.cpp
#
# Description: 
#
===============================================*/
#include "data_reading.h"

namespace ulysses
{

	void DataReading::loadScan(Scan *scan, const cv::Mat &rgb, const cv::Mat &depth)
	{
		scan->point_cloud=pcl::PointCloud<pcl::PointXYZRGBA>::Ptr (new pcl::PointCloud<pcl::PointXYZRGBA>);
		scan->normal_cloud=pcl::PointCloud<pcl::Normal>::Ptr (new pcl::PointCloud<pcl::Normal>);
		scan->pixel_cloud=pcl::PointCloud<pcl::PointXY>::Ptr (new pcl::PointCloud<pcl::PointXY>);

//		scan->time_stamp=timestamp_depth;
		cv::Mat rgb_image,depth_image;
		uint8_t *depth_ptr,*rgb_ptr;
		pcl::PointXYZRGBA point_tmp;
		unsigned short *depth_tmp_ptr=new unsigned short;
		pcl::PointXY tmp_pointxy;
		// full path of the current rgb and depth image;
//		std::string filename_rgb_full=path+"/"+filename_rgb;
//		std::string filename_depth_full=path+"/"+filename_depth;
		// load the rgb and depth image to cv::Mat;
		// the depth_image is stored as CV_8UC2;
		scan->img_rgb=rgb;//cv::imread(filename_rgb_full);
		scan->img_depth=depth;//cv::imread(filename_depth_full,-1);
		//cv::imshow("rgb",scan->img_rgb);
		//cv::waitKey(0);
		//cv::imshow("dep",scan->img_depth);
		//cv::waitKey(0);

		// pointer to the Mat data;
		rgb_ptr=scan->img_rgb.data;
		depth_ptr=scan->img_depth.data;
		// clear the pointcloud;
		// the allocated memory does not release;
		// the newly pushed elements cover the old ones;
		scan->point_cloud->clear();
		scan->normal_cloud->clear();
		scan->pixel_cloud->clear();
		// generate the point_cloud;
		for(int i=0;i<scan->img_rgb.rows;i++)
		{
			for(int j=0;j<scan->img_rgb.cols;j++)
			{
				// 3 channels for one pixel in rgb image;
				point_tmp.b=*rgb_ptr;
				rgb_ptr++;
				point_tmp.g=*rgb_ptr;
				rgb_ptr++;
				point_tmp.r=*rgb_ptr;
				rgb_ptr++;
				// 2 channels for one pixel in depth image;
				memcpy(depth_tmp_ptr,depth_ptr,2);
				depth_ptr+=2;
//				if(j<300 || j>=800 || i<80 || i>=500)
//					continue;
				point_tmp.z=*depth_tmp_ptr/scan->cam.factor;
				// transformation from pixel coordinate to the camera coordinate;
				// wrong results if considering length of the pixel;
				point_tmp.x=(j-scan->cam.cx)*point_tmp.z/scan->cam.fx;
				point_tmp.y=(i-scan->cam.cy)*point_tmp.z/scan->cam.fy;
				scan->point_cloud->push_back(point_tmp);
			}
		}
		delete depth_tmp_ptr;
		// organize the point_cloud for the normal estimation;
		scan->point_cloud->width=scan->cam.width;//500;//
		scan->point_cloud->height=scan->cam.height;//420;//
		// generate the normal_cloud;
		normal_estimate_integral.setInputCloud(scan->point_cloud);
		normal_estimate_integral.compute (*scan->normal_cloud);
		// generate the pixel_cloud;
		for(int v=0;v<scan->point_cloud->height;v++)
		{
			for(int u=0;u<scan->point_cloud->width;u++)
			{
				tmp_pointxy.x=u;
				tmp_pointxy.y=v;
				scan->pixel_cloud->push_back(tmp_pointxy);
			}
		}

//		Eigen::Quaterniond quat(qw,qx,qy,qz);
//		Eigen::Vector3d vec3d(tx,ty,tz);
//		Transform Tc(quat,vec3d);
//		Transform Tcg(Eigen::Quaterniond(qw,qx,qy,qz), Eigen::Vector3d(tx,ty,tz));
//		Tcg.inverse();
//		scan->Tcg_gt=Tcg;
//
//		scan->Tcg_gt=Tc.inv()*Tg;
	}
}
