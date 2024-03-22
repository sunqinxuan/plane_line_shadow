/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@gmail.com
#
# Last modified: 2019-05-08 09:28
#
# Filename: system_plane_line_shadow.cpp
#
# Description: 
#
************************************************/
#include "system_plane_line_shadow.h"

namespace ulysses 
{
	SystemPlaneLineShadow::SystemPlaneLineShadow(const std::string &settingFile)
	{
		cv::FileStorage settings(settingFile.c_str(),cv::FileStorage::READ);

		bool debug, visual;
		if((int)settings["debug"]==0) debug=false;
		else debug=true;
		if((int)settings["visual"]==0) visual=false;
		else visual=true;

		scan_cur=0;
		scan_ref=0;
//		map=new Map;
		num_scan=0;

		optimize_frames=(int)settings["OptimizeFrames"];

		intr_cam=new IntrinsicParam();
		intr_cam->fx=(double)settings["Camera.fx"];
		intr_cam->fy=(double)settings["Camera.fy"];
		intr_cam->cx=(double)settings["Camera.cx"];
		intr_cam->cy=(double)settings["Camera.cy"];
		intr_cam->width=(int)settings["Camera.width"];
		intr_cam->height=(int)settings["Camera.height"];
		intr_cam->factor=(double)settings["Camera.factor"];
		
		plane_extraction=new PlaneExtraction(visual, debug,
						 (int)settings["PlaneExtraction.min_plane_size"],
						 (double)settings["PlaneExtraction.thres_angle"],
						 (double)settings["PlaneExtraction.thres_dist"],
						 (double)settings["PlaneExtraction.thres_color"]);
		
		line_extraction=new LineExtraction(visual, debug,
						(double)settings["LineExtraction.HoughLinesP.rho"],
						(double)settings["LineExtraction.HoughLinesP.theta"],
						(int)settings["LineExtraction.HoughLinesP.threshold"],
						(double)settings["LineExtraction.HoughLinesP.minLineLength"],
						(double)settings["LineExtraction.HoughLinesP.maxLineGap"],
						(double)settings["LineExtraction.thres_sim_dir"],
						(double)settings["LineExtraction.thres_sim_dist"],
						(double)settings["LineExtraction.thres_split"],
						(int)settings["LineExtraction.min_points_on_line"],
						(double)settings["LineExtraction.thres_line2shadow"],
						(double)settings["LineExtraction.thres_shadow2plane"]);

		edge_extraction=new EdgePointExtraction(visual, debug,
				(double)settings["EdgePointExtraction.PCLOrganizedEdge.DepthDisconThreshold"],
				(int)settings["EdgePointExtraction.PCLOrganizedEdge.MaxSearchNeighbors"],
				(double)settings["EdgePointExtraction.thres_pxl"],
				(double)settings["EdgePointExtraction.thres_occluded_dist"],
				(double)settings["EdgePointExtraction.ANN_search_radius"],
				(int)settings["EdgePointExtraction.ANN_search_K"]);

		plane_fitting=new PlaneParamEstimation();
		plane_fitting->setDebug(debug);

		plane_matching=new PlaneFeatureMatching(visual, debug,
						(double)settings["PlaneFeatureMatching.thres_delta_angle"],
						(double)settings["PlaneFeatureMatching.thres_delta_d"],
						(double)settings["PlaneFeatureMatching.thres_color"]);

		line_mathing=new LineFeatureMatching(visual, debug,
						(double)settings["LineFeatureMatching.thres_dir"],
						(double)settings["LineFeatureMatching.thres_normal"],
						(double)settings["LineFeatureMatching.thres_dist"],
						(double)settings["LineFeatureMatching.thres_delta_dir"],
						(double)settings["LineFeatureMatching.thres_delta_normal"],
						(double)settings["LineFeatureMatching.thres_delta_dist"]);

		motion_estimation=new MotionEstimation_Line(visual, debug,
						(double)settings["MotionEstimationLine.use_plane"],
						(double)settings["MotionEstimationLine.use_line"],
						(double)settings["MotionEstimationLine.use_shadow"]);

		global_map=new GlobalMap();
		global_map->setDebug(debug);

	}

	bool SystemPlaneLineShadow::trackCamera(const cv::Mat &rgb, const cv::Mat &depth, const double &timestamp,
											boost::shared_ptr<pcl::visualization::PCLVisualizer> vis,
											Transform Twc)
	{
		timeval time_start, time_end;
		double time_used;

		scan_cur=new Scan(num_scan,*intr_cam,timestamp);
		num_scan++;
		isFirstFrame=false;

		loadScan(scan_cur,rgb,depth);
		scan_cur->Tcw=Twc.inv();

		if(scan_ref==0) isFirstFrame=true;
		else scan_cur->setRef(scan_ref);

		plane_extraction->extractPlanes(scan_cur,vis);
		for(int i=0;i<scan_cur->observed_planes.size();i++)
			plane_fitting->estimatePlaneParams(scan_cur->observed_planes[i],*intr_cam);

		edge_extraction->extractEdgePoints(scan_cur,vis);
		line_extraction->extractLines(scan_cur,vis);

		if(!isFirstFrame)
		{
			plane_matching->match(scan_cur,vis);
			line_mathing->match(scan_cur,vis);

			map=new Map;
			global_map->addScan(scan_cur,map);

			ulysses::Transform Tcr;
			Tcr.setIdentity();
			if(motion_estimation->alignScans(scan_cur,map,Tcr,optimize_frames,vis))
			{
				scan_cur->Tcr=Tcr;
				scan_cur->Tcg=scan_cur->Tcr*scan_ref->Tcg; // Tcg=Tcr*Trg;

				std::cout<<"Tcr "<<std::endl<<Tcr.getMatrix4f()<<std::endl;

				timestamps.push_back(timestamp);
				traj.push_back(scan_cur->Tcg);
				
			}

			delete scan_ref;
			delete map;
		}

		scan_ref=scan_cur;
		return true;
	}

	void SystemPlaneLineShadow::saveTraj(const std::string &file, bool flag)
	{
		std::cout<<std::endl<<"saving trajectory to "<<file<<std::endl;
		std::ofstream fp;
		fp.open(file.c_str(),std::ios::out);
		for(int i=0;i<traj.size();i++)
		{
			ulysses::Transform Tgc=traj[i].inv();
			fp<<std::fixed<<timestamps[i]<<" "<<Tgc.t.transpose()<<" "<<Tgc.Quaternion().transpose()<<std::endl;
		}
		fp.close();
		std::cout<<"trajectory saved"<<std::endl;
	}


	void SystemPlaneLineShadow::saveTraj(const std::string &file)
	{
		std::cout<<std::endl<<"saving trajectory to "<<file<<std::endl;
		std::ofstream fp;
		fp.open(file.c_str(),std::ios::out);
		for(int i=0;i<map->cameras.size();i++)
		{
			ulysses::Transform Tgc=map->cameras[i].inv();
			fp<<std::fixed<<map->timestamps[i]<<" "<<Tgc.t.transpose()<<" "<<Tgc.Quaternion().transpose()<<std::endl;
		}
		fp.close();
		std::cout<<"trajectory saved"<<std::endl;
	}

	void SystemPlaneLineShadow::loadScan(Scan *scan, const cv::Mat &rgb, const cv::Mat &depth)
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
		pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBA, pcl::Normal> normal_estimate_integral;
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

	void SystemPlaneLineShadow::shutDown()
	{
		delete scan_cur;
//		delete map;
		delete intr_cam;
		delete plane_extraction;
		delete line_extraction;
		delete edge_extraction;
		delete plane_fitting;
		delete plane_matching;
		delete line_mathing;
		delete motion_estimation;
		delete global_map;
	}

}
