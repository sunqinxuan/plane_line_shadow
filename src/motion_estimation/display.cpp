/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@gmail.com
#
# Last modified:	2019-04-25 10:23
#
# Filename:		display.cpp
#
# Description: 
#
===============================================*/
#include "display.h"

namespace ulysses
{

	void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
								void* viewer_void)
	{
		pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
		if (event.getKeySym () == "q" && event.keyDown ())
		{
		   viewer->close();
		}
	}

	// displayPlanes
	// - display planes from one scan in different colors;
	void displayPlanes(Scan *scan, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
		char id[20];
		unsigned char red [14] = {255,   0,   0, 255, 255,   0, 130,   0,   0, 130, 130,   0, 130, 255};
		unsigned char grn [14] = {  0, 255,   0, 255,   0, 255,   0, 130,   0, 130,   0, 130, 130, 255};
		unsigned char blu [14] = {  0,   0, 255,   0, 255, 255,   0,   0, 130,   0, 130, 130, 130, 255};

		vis->removeAllPointClouds();
		vis->removeAllShapes();
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr plane (new pcl::PointCloud<pcl::PointXYZRGBA>);

		// add raw scan data;
		sprintf(id,"scan");
		if (!vis->updatePointCloud (scan->point_cloud, id))
			vis->addPointCloud (scan->point_cloud, id);

		for(size_t i=0;i<scan->observed_planes.size();i++)
		{
			sprintf(id,"plane%d",i);
			plane->resize(scan->observed_planes[i]->points.size());
			for(size_t j=0;j<plane->size();j++)
			{
				plane->at(j).x=scan->observed_planes[i]->points[j].xyz[0];
				plane->at(j).y=scan->observed_planes[i]->points[j].xyz[1];
				plane->at(j).z=scan->observed_planes[i]->points[j].xyz[2];
			}
			pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGBA> color1 (plane, red[i%12], grn[i%12], blu[i%12]);
			if (!vis->updatePointCloud (plane, color1, id))
				vis->addPointCloud (plane, color1, id);
		}

//		// draw the scale of plane->cov_xyz;
//		pcl::PointXYZRGBA pt1,pt2;
//		for(size_t i=0;i<scan->observed_planes.size();i++)
//		{
//			pt1.x=scan->observed_planes[i]->avg_xyz(0);
//			pt1.y=scan->observed_planes[i]->avg_xyz(1);
//			pt1.z=scan->observed_planes[i]->avg_xyz(2);
//			pt2.x=scan->observed_planes[i]->avg_xyz(0)+scan->observed_planes[i]->cov_eigenvectors(0,2)*sqrt(scan->observed_planes[i]->cov_eigenvalues(2))*2;
//			pt2.y=scan->observed_planes[i]->avg_xyz(1)+scan->observed_planes[i]->cov_eigenvectors(1,2)*sqrt(scan->observed_planes[i]->cov_eigenvalues(2))*2;
//			pt2.z=scan->observed_planes[i]->avg_xyz(2)+scan->observed_planes[i]->cov_eigenvectors(2,2)*sqrt(scan->observed_planes[i]->cov_eigenvalues(2))*2;
//			sprintf(id,"%dline1",i);
//			vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,red[13], grn[13], blu[13],id);
//
//			pt2.x=scan->observed_planes[i]->avg_xyz(0)+scan->observed_planes[i]->cov_eigenvectors(0,1)*sqrt(scan->observed_planes[i]->cov_eigenvalues(1))*2;
//			pt2.y=scan->observed_planes[i]->avg_xyz(1)+scan->observed_planes[i]->cov_eigenvectors(1,1)*sqrt(scan->observed_planes[i]->cov_eigenvalues(1))*2;
//			pt2.z=scan->observed_planes[i]->avg_xyz(2)+scan->observed_planes[i]->cov_eigenvectors(2,1)*sqrt(scan->observed_planes[i]->cov_eigenvalues(1))*2;
//			sprintf(id,"%dline2",i);
//			vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,red[13], grn[13], blu[13],id);
//
//		}
	}

	// displayPlanes_2scans
	// - display planes from 2 scans;
	// - different color for each scan;
	// - scan_ref is transformed by Tcr;
	void displayPlanes_2scans(Scan *scan, Scan *scan_ref, Transform Tcr, 
									boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
		char id[20];
		unsigned char red [14] = {255,   0,   0, 255, 255,   0, 130,   0,   0, 130, 130,   0, 130, 255};
		unsigned char grn [14] = {  0, 255,   0, 255,   0, 255,   0, 130,   0, 130,   0, 130, 130, 255};
		unsigned char blu [14] = {  0,   0, 255,   0, 255, 255,   0,   0, 130,   0, 130, 130, 130, 255};

		vis->removeAllPointClouds();
	//	vis->removeAllShapes();
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr plane (new pcl::PointCloud<pcl::PointXYZRGBA>);

		for(size_t i=0;i<scan->observed_planes.size();i++)
		{
			sprintf(id,"plane_cur%d",i);
			plane->resize(scan->observed_planes[i]->points.size());
			for(size_t j=0;j<plane->size();j++)
			{
				plane->at(j).x=scan->observed_planes[i]->points[j].xyz[0];
				plane->at(j).y=scan->observed_planes[i]->points[j].xyz[1];
				plane->at(j).z=scan->observed_planes[i]->points[j].xyz[2];
			}
			pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGBA> color1 (plane, red[0], grn[0], blu[0]);
			if (!vis->updatePointCloud (plane, color1, id))
				vis->addPointCloud (plane, color1, id);
		}
		for(size_t i=0;i<scan_ref->observed_planes.size();i++)
		{
			sprintf(id,"plane_ref%d",i);
			plane->resize(scan_ref->observed_planes[i]->points.size());
			for(size_t j=0;j<plane->size();j++)
			{
				plane->at(j).x=scan_ref->observed_planes[i]->points[j].xyz[0];
				plane->at(j).y=scan_ref->observed_planes[i]->points[j].xyz[1];
				plane->at(j).z=scan_ref->observed_planes[i]->points[j].xyz[2];
			}
			pcl::transformPointCloud(*plane,*plane,Tcr.getMatrix4f());
			pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGBA> color1 (plane, red[2], grn[2], blu[2]);
			if (!vis->updatePointCloud (plane, color1, id))
				vis->addPointCloud (plane, color1, id);
		}
	}

	// displayScans_2scans
	// - display raw scan data from 2 scans;
	// - different color for each scan;
	// - scan_ref is transformed by Tcr;
	void displayScans_2scans(Scan *scan, Scan *scan_ref, Transform Tcr, 
							 boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
		char id[20];
		unsigned char red [14] = {255,   0,   0, 255, 255,   0, 130,   0,   0, 130, 130,   0, 130, 255};
		unsigned char grn [14] = {  0, 255,   0, 255,   0, 255,   0, 130,   0, 130,   0, 130, 130, 255};
		unsigned char blu [14] = {  0,   0, 255,   0, 255, 255,   0,   0, 130,   0, 130, 130, 130, 255};

		vis->removeAllPointClouds();
		vis->removeAllShapes();
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr plane (new pcl::PointCloud<pcl::PointXYZRGBA>);

		sprintf(id,"scan_ref");
		pcl::transformPointCloud(*scan_ref->point_cloud,*plane,Tcr.getMatrix4f());
		pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGBA> color1 (plane, red[2], grn[2], blu[2]);
		if (!vis->updatePointCloud (plane, color1, id))
			vis->addPointCloud (plane, color1, id);

		sprintf(id,"scan_cur");
		pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGBA> color2 (scan->point_cloud, red[0], grn[0], blu[0]);
		if (!vis->updatePointCloud (scan->point_cloud, color2, id))
			vis->addPointCloud (scan->point_cloud, color2, id);

	//	pcl::PointXYZRGBA pt1,pt2;
	//	for(size_t i=0;i<scan->edge_points.size();i++)
	//	{
	//		if(scan->edge_points[i]->cov.determinant()<1e-20 || scan->point_matches[i]==0)
	//			continue;
	//		pt1.x=scan->edge_points[i]->xyz(0);
	//		pt1.y=scan->edge_points[i]->xyz(1);
	//		pt1.z=scan->edge_points[i]->xyz(2);
	//		pt2.x=scan->point_matches[i]->xyz(0);
	//		pt2.y=scan->point_matches[i]->xyz(1);
	//		pt2.z=scan->point_matches[i]->xyz(2);
	//		sprintf(id,"dir%d",i);
	//		vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,red[1], grn[1], blu[1],id);
	//	}
	}

	// display_addScan
	// - add one scan to vis after calling this function;
	// - the scan is transformed to global frame by scan->Tcg;
	void display_addScan(Scan *scan, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
		char id[20];
		unsigned char red [14] = {255,   0,   0, 255, 255,   0, 130,   0,   0, 130, 130,   0, 130, 255};
		unsigned char grn [14] = {  0, 255,   0, 255,   0, 255,   0, 130,   0, 130,   0, 130, 130, 255};
		unsigned char blu [14] = {  0,   0, 255,   0, 255, 255,   0,   0, 130,   0, 130, 130, 130, 255};

	//	vis->removeAllPointClouds();
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr plane (new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr plane_filtered (new pcl::PointCloud<pcl::PointXYZRGBA>);

		Transform Tgc=scan->Tcg.inv();
		//std::cout<<"addScan "<<Tgc.t.transpose()<<" "<<Tgc.Quaternion().transpose()<<std::endl;
		pcl::transformPointCloud(*scan->point_cloud,*plane,Tgc.getMatrix4f());
	//	int count=0;
		for(size_t i=0;i<plane->height;i++)
		{
			for(size_t j=0;j<plane->width;j++)
			{
	//			std::cout<<i<<" "<<i%10<<" "<<j<<" "<<j%10<<std::endl;
				if(i%4==0&&j%4==0)
				{
					plane_filtered->push_back(plane->at(j,i));
	//				count++;
				}
			}
		}
	//	std::cout<<"count "<<count<<std::endl;
	//	std::cout<<"plane "<<plane->size()<<std::endl;
	//	pcl::VoxelGrid<pcl::PointXYZRGBA> filter;
	//	filter.setInputCloud(plane);
	//	filter.setLeafSize(0.01,0.01,0.01);
	//	filter.filter(*plane);

		sprintf(id,"scan%d", scan->id);
		if (!vis->updatePointCloud (plane_filtered, id))
			vis->addPointCloud (plane_filtered, id);
	//	if (!vis->updatePointCloud (plane_filtered, id))
	//		vis->addPointCloud (plane_filtered, id);
	}

	// displayEdgePoints
	// - display edge points in one scan;
	void displayEdgePoints(Scan *scan, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
		char id[50];
		unsigned char red [14] = {255,   0,   0, 255, 255,   0, 130,   0,   0, 130, 130,   0, 130, 255};
		unsigned char grn [14] = {  0, 255,   0, 255,   0, 255,   0, 130,   0, 130,   0, 130, 130, 255};
		unsigned char blu [14] = {  0,   0, 255,   0, 255, 255,   0,   0, 130,   0, 130, 130, 130, 255};

	//	vis->removeAllPointClouds();
		vis->removeAllShapes();

		// add raw scan data;
		sprintf(id,"scan%d",scan->id);
		if (!vis->updatePointCloud (scan->point_cloud, id))
			vis->addPointCloud (scan->point_cloud, id);

	//	vis->removeAllPointClouds();
	//	vis->removeAllShapes();
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr edge (new pcl::PointCloud<pcl::PointXYZRGBA>);
		// add edge points to vis;
		int count=0;
		pcl::PointXYZRGBA pt1,pt2;

		edge->resize(scan->edge_points.size());
		for(size_t i=0;i<scan->edge_points.size();i++)
		{
			if(scan->edge_points[i]->isEdge==false)
				continue;
//			if(scan->edge_points[i]->meas_Edge>20)
//				continue;
			edge->at(i).x=scan->edge_points[i]->xyz(0);
			edge->at(i).y=scan->edge_points[i]->xyz(1);
			edge->at(i).z=scan->edge_points[i]->xyz(2);
//			unsigned int color=(1-scan->edge_points[i]->meas_Edge/100.0)*16777215;
//			unsigned char r=color>>16;
//			unsigned char g=(color>>8)&0xff;
//			unsigned char b=color&0xff;
//
			unsigned int color=(1-scan->edge_points[i]->meas_Edge/20.0)*255.0;
			unsigned char r=color;
			unsigned char g=color;
			unsigned char b=color;
			edge->at(i).r=r;
			edge->at(i).g=g;
			edge->at(i).b=b;
//			edge->at(i).r=scan->edge_points[i]->rgb(0);
//			edge->at(i).g=scan->edge_points[i]->rgb(1);
//			edge->at(i).b=scan->edge_points[i]->rgb(2);

//			pt1.x=edge->at(i).x;
//			pt1.y=edge->at(i).y;
//			pt1.z=edge->at(i).z;
//			pt2.x=pt1.x+scan->edge_points[i]->dir(0)*0.01;
//			pt2.y=pt1.y+scan->edge_points[i]->dir(1)*0.01;
//			pt2.z=pt1.z+scan->edge_points[i]->dir(2)*0.01;
//			sprintf(id,"%dedge_dir",i);
//			vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,red[1], grn[1], blu[1],id);//green
		}
		sprintf(id,"edgePoints%d",scan->id);
//		pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGBA> color_edge (edge, red[13], grn[13], blu[13]);//white
//		if (!vis->updatePointCloud (edge, color_edge, id))
//			vis->addPointCloud (edge, color_edge, id);
		if (!vis->updatePointCloud (edge, id))
			vis->addPointCloud (edge, id);
		vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, id);
		
//		edge->resize(scan->edge_points_occluded.size());
//		for(size_t i=0;i<scan->edge_points_occluded.size();i++)
//		{
//			if(scan->edge_points_occluded[i]->isEdge==false)
//				continue;
//			edge->at(i).x=scan->edge_points_occluded[i]->xyz(0);
//			edge->at(i).y=scan->edge_points_occluded[i]->xyz(1);
//			edge->at(i).z=scan->edge_points_occluded[i]->xyz(2);
//			// display the color of the corresponding plane;
//			if(scan->edge_points_occluded[i]->plane==0)
//			{
//				edge->at(i).r=red[12];
//				edge->at(i).g=grn[12];
//				edge->at(i).b=blu[12];
//			}
//			for(size_t j=0;j<scan->observed_planes.size();j++)
//			{
//				if(scan->edge_points_occluded[i]->plane==scan->observed_planes[j])
//				{
//					edge->at(i).r=red[j];
//					edge->at(i).g=grn[j];
//					edge->at(i).b=blu[j];
//				}
//			}
//		}
//		sprintf(id,"edgePoints_occluded%d",scan->id);
////		pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGBA> color_edge_occluded (edge, red[0], grn[0], blu[0]);//red
//	//	if (!vis->updatePointCloud (edge, color_edge_occluded, id))
//	//		vis->addPointCloud (edge, color_edge_occluded, id);
//		if (!vis->updatePointCloud (edge, id))
//			vis->addPointCloud (edge, id);
//		vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, id);


//		// draw lines between associated occluding and occluded points;
//		pcl::PointXYZRGBA pt1,pt2;
//		for(size_t i=0;i<scan->projective_rays.size();i++)
//		{
//			if(scan->edge_points_occluded[i]->occluding==0)
//			{
//				continue;
//			}
//			pt1.x=scan->projective_rays[i]->occluding->xyz(0);
//			pt1.y=scan->projective_rays[i]->occluding->xyz(1);
//			pt1.z=scan->projective_rays[i]->occluding->xyz(2);
//			pt2.x=scan->projective_rays[i]->occluded->xyz(0);
//			pt2.y=scan->projective_rays[i]->occluded->xyz(1);
//			pt2.z=scan->projective_rays[i]->occluded->xyz(2);
//			sprintf(id,"%dedge",i);
//			vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,red[1], grn[1], blu[1],id);
//		}
//		std::cout<<"occluding points\t"<<scan->edge_points.size()<<std::endl;
//		std::cout<<"occluded points \t"<<scan->edge_points_occluded.size()<<std::endl;
//		std::cout<<"projective rays \t"<<scan->projective_rays.size()<<std::endl;

	}


	void displayMatchedEdgePoints(Scan *scan, Transform Tcr, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
		char id[50];
		// 							0    1    2    3    4    5    6    7    8    9   10   11   12   13
		unsigned char red [14] = {255,   0,   0, 255, 255,   0, 130,   0,   0, 130, 130,   0, 130, 255};
		unsigned char grn [14] = {  0, 255,   0, 255,   0, 255,   0, 130,   0, 130,   0, 130, 130, 255};
		unsigned char blu [14] = {  0,   0, 255,   0, 255, 255,   0,   0, 130,   0, 130, 130, 130, 255};
		pcl::PointXYZRGBA pt1,pt2,pt11,pt22;

		vis->removeAllPointClouds();
		vis->removeAllShapes();

//		sprintf(id,"scan%d",scan->id);
//		if (!vis->updatePointCloud (scan->point_cloud, id))
//			vis->addPointCloud (scan->point_cloud, id);

		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::VoxelGrid<pcl::PointXYZRGBA> filter;
		filter.setLeafSize(0.01f,0.01f,0.01f);

		sprintf(id,"scan_ref");
		pcl::transformPointCloud(*scan->scan_ref->point_cloud,*cloud,Tcr.getMatrix4f());//deep skyblue
		filter.setInputCloud(cloud);
		filter.filter(*cloud_filtered);
		pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGBA> color1 (cloud_filtered,0,191,255);
		if (!vis->updatePointCloud (cloud_filtered, color1, id))
			vis->addPointCloud (cloud_filtered, color1, id);

		sprintf(id,"scan_cur");//deep pink
		filter.setInputCloud(scan->point_cloud);
		filter.filter(*cloud_filtered);
		pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGBA> color2 (scan->point_cloud,255,20,147);
		if (!vis->updatePointCloud (cloud_filtered, color2, id))
			vis->addPointCloud (cloud_filtered, color2, id);


		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr edge_cur (new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr edge_ref (new pcl::PointCloud<pcl::PointXYZRGBA>);

		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr edge_cur_occluded (new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr edge_ref_occluded (new pcl::PointCloud<pcl::PointXYZRGBA>);

		edge_cur->resize(scan->projective_ray_matches.size());
		edge_ref->resize(scan->projective_ray_matches.size());
		edge_cur_occluded->resize(scan->projective_ray_matches.size());
		edge_ref_occluded->resize(scan->projective_ray_matches.size());
		int count_ed=0, count_ing=0;
		// Transform Trc=Tcr.inv();
		// Eigen::Matrix3d R=Trc.R;
		// Eigen::Vector3d t=Trc.t;
		Eigen::Vector3d t=Tcr.t;
		for(size_t i=0;i<scan->projective_ray_matches.size();i++)
		{
			// current occluding point;
			Eigen::Vector3d tmp=scan->projective_ray_matches[i]->cur->occluding->xyz;
			pt1.x=tmp(0);
			pt1.y=tmp(1);
			pt1.z=tmp(2);
			edge_cur->at(i).x=pt1.x;
			edge_cur->at(i).y=pt1.y;
			edge_cur->at(i).z=pt1.z;
			// // reference occluding point;
			tmp=Tcr.transformPoint(scan->projective_ray_matches[i]->ref->occluding->xyz);
//			Eigen::Vector3d dir=Tcr.transformPoint(scan->projective_ray_matches[i]->ref->occluding->xyz)-t;
//			Eigen::Matrix<double,1,1> mu_mat=dir.transpose()*(tmp-t);
//			double mu=mu_mat(0,0)/(dir.norm()*dir.norm());
//			tmp=mu*dir+t;
			pt2.x=tmp(0);
			pt2.y=tmp(1);
			pt2.z=tmp(2);
//			tmp=Tcr.transformPoint(scan->projective_ray_matches[i]->ref->occluding->xyz);
			edge_ref->at(i).x=tmp(0);
			edge_ref->at(i).y=tmp(1);
			edge_ref->at(i).z=tmp(2);
			// draw lines between current and reference occluding points;
			sprintf(id,"%dedge_occluding",i);
			vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,0,255,0,id);//green
			// current occluded point;
			// tmp=scan->projective_ray_matches[i]->cur->occluded->xyz;
			tmp=Tcr.transformPoint(scan->projective_ray_matches[i]->ref->occluded->xyz);
			pt11.x=tmp(0);
			pt11.y=tmp(1);
			pt11.z=tmp(2);
			edge_cur_occluded->at(i).x=pt11.x;
			edge_cur_occluded->at(i).y=pt11.y;
			edge_cur_occluded->at(i).z=pt11.z;
			// // reference occluded point;
			// Eigen::Vector3d nr=scan->projective_ray_matches[i]->ref->plane->normal;
			// double dr=scan->projective_ray_matches[i]->ref->plane->d;
			// Eigen::Vector3d pr=scan->projective_ray_matches[i]->ref->occluding->xyz;
			// double ntd=nr.transpose()*t+dr;
			// double ntp=nr.transpose()*(t-pr);
			// double npd=nr.transpose()*pr+dr;
			// tmp=pr*ntd/ntp-t*npd/ntp;
			// tmp=Tcr.transformPoint(tmp);
//			Eigen::Vector3d n=scan->projective_ray_matches[i]->cur->plane->normal;
//			Eigen::Matrix<double,1,1> tmp1=n.transpose()*tmp;
//			double tmp2=tmp1(0,0)+scan->projective_ray_matches[i]->cur->plane->d;
//			tmp=tmp-tmp2*n;
			tmp=scan->projective_ray_matches[i]->cur->plane->projected_point(scan->projective_ray_matches[i]->cur->occluding->xyz,t);
			pt22.x=tmp(0);
			pt22.y=tmp(1);
			pt22.z=tmp(2);
			edge_ref_occluded->at(i).x=tmp(0);
			edge_ref_occluded->at(i).y=tmp(1);
			edge_ref_occluded->at(i).z=tmp(2);
			// draw lines between current and reference occluded points;
			sprintf(id,"%dres_shadow_ref",i);
			vis->addLine<pcl::PointXYZRGBA>(pt11,pt22,255,255,0,id);//yellow
		}

		sprintf(id,"edge_cur%d",scan->id);//red
		pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGBA> color_cur (edge_cur,255,0,0);
		if (!vis->updatePointCloud (edge_cur, color_cur, id))
			vis->addPointCloud (edge_cur, color_cur, id);
		vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, id);

		sprintf(id,"edge_ref%d",scan->id);//blue
		pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGBA> color_ref (edge_ref,0,0,255);
		if (!vis->updatePointCloud (edge_ref, color_ref, id))
			vis->addPointCloud (edge_ref, color_ref, id);
		vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, id);

		sprintf(id,"edge_cur_occluded%d",scan->id);//magenta
		pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGBA> color_cur_tmp (edge_cur_occluded,255,0,255);
		if (!vis->updatePointCloud (edge_cur_occluded, color_cur_tmp, id))
			vis->addPointCloud (edge_cur_occluded, color_cur_tmp, id);
		vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, id);

		sprintf(id,"edge_ref_occluded%d",scan->id);//cyan
		pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGBA> color_ref_tmp (edge_ref_occluded,0,255,255);
		if (!vis->updatePointCloud (edge_ref_occluded, color_ref_tmp, id))
			vis->addPointCloud (edge_ref_occluded, color_ref_tmp, id);
		vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, id);
	}


	void displayMatchedProjRays(Scan *scan, Transform Tcr, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
		char id[50];
		// 							0    1    2    3    4    5    6    7    8    9   10   11   12   13
		unsigned char red [14] = {255,   0,   0, 255, 255,   0, 130,   0,   0, 130, 130,   0, 130, 255};
		unsigned char grn [14] = {  0, 255,   0, 255,   0, 255,   0, 130,   0, 130,   0, 130, 130, 255};
		unsigned char blu [14] = {  0,   0, 255,   0, 255, 255,   0,   0, 130,   0, 130, 130, 130, 255};
		pcl::PointXYZRGBA pt1,pt2,pt11,pt22;

		vis->removeAllPointClouds();
		vis->removeAllShapes();

//		sprintf(id,"scan%d",scan->id);
//		if (!vis->updatePointCloud (scan->point_cloud, id))
//			vis->addPointCloud (scan->point_cloud, id);

		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::VoxelGrid<pcl::PointXYZRGBA> filter;
		filter.setLeafSize(0.01f,0.01f,0.01f);

		sprintf(id,"scan_ref");
		pcl::transformPointCloud(*scan->scan_ref->point_cloud,*cloud,Tcr.getMatrix4f());//skyblue
		filter.setInputCloud(cloud);
		filter.filter(*cloud_filtered);
		pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGBA> color1 (cloud_filtered,135,206,235);
		if (!vis->updatePointCloud (cloud_filtered, color1, id))
			vis->addPointCloud (cloud_filtered, color1, id);

		sprintf(id,"scan_cur");//pink
		filter.setInputCloud(scan->point_cloud);
		filter.filter(*cloud_filtered);
		pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGBA> color2 (scan->point_cloud,255,192,203);
		if (!vis->updatePointCloud (cloud_filtered, color2, id))
			vis->addPointCloud (cloud_filtered, color2, id);


		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr edge_cur (new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr edge_ref (new pcl::PointCloud<pcl::PointXYZRGBA>);

		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr edge_cur_occluded (new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr edge_ref_occluded (new pcl::PointCloud<pcl::PointXYZRGBA>);

		edge_cur->resize(scan->projective_ray_matches.size());
		edge_ref->resize(scan->projective_ray_matches.size());
		edge_cur_occluded->resize(scan->projective_ray_matches.size());
		edge_ref_occluded->resize(scan->projective_ray_matches.size());
		int count_ed=0, count_ing=0;
		Transform Trc=Tcr.inv();
		for(size_t i=0;i<scan->projective_ray_matches.size();i++)
		{
			// current occluding point;
			Eigen::Vector3d tmp=scan->projective_ray_matches[i]->fused_edge_point_cur;
			pt1.x=tmp(0);
			pt1.y=tmp(1);
			pt1.z=tmp(2);
			edge_cur->at(i).x=pt1.x;
			edge_cur->at(i).y=pt1.y;
			edge_cur->at(i).z=pt1.z;
//			// reference occluding point;
//			tmp=Tcr.transformPoint(scan->projective_ray_matches[i]->fused_edge_point_ref);
//			pt2.x=tmp(0);
//			pt2.y=tmp(1);
//			pt2.z=tmp(2);
//			edge_ref->at(i).x=tmp(0);
//			edge_ref->at(i).y=tmp(1);
//			edge_ref->at(i).z=tmp(2);
//			// draw lines between current and reference occluding points;
//			sprintf(id,"%dedge_occluding",i);
//			vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,0,255,0,id);//green

			// current occluded point;
			tmp=scan->projective_ray_matches[i]->cur->occluded->xyz;
			pt11.x=tmp(0);
			pt11.y=tmp(1);
			pt11.z=tmp(2);
			edge_cur_occluded->at(i).x=pt11.x;
			edge_cur_occluded->at(i).y=pt11.y;
			edge_cur_occluded->at(i).z=pt11.z;
			tmp=scan->projective_ray_matches[i]->cur->plane->projected_point(scan->projective_ray_matches[i]->fused_edge_point_cur);
			pt1.x=tmp(0);
			pt1.y=tmp(1);
			pt1.z=tmp(2);
			sprintf(id,"%dres_shadow_cur",i);
			vis->addLine<pcl::PointXYZRGBA>(pt11,pt1,0,255,0,id);//green
			// reference occluded point;
			tmp=scan->projective_ray_matches[i]->ref->occluded->xyz;
			tmp=Tcr.transformPoint(tmp);
			pt22.x=tmp(0);
			pt22.y=tmp(1);
			pt22.z=tmp(2);
			edge_ref_occluded->at(i).x=tmp(0);
			edge_ref_occluded->at(i).y=tmp(1);
			edge_ref_occluded->at(i).z=tmp(2);
			tmp=scan->projective_ray_matches[i]->ref->plane->projected_point(Trc.transformPoint(scan->projective_ray_matches[i]->fused_edge_point_cur));
			tmp=Tcr.transformPoint(tmp);
//			Eigen::Vector4d pln;
//			pln.block<3,1>(0,0)=scan->projective_ray_matches[i]->ref->plane->normal;
//			pln(3)=scan->projective_ray_matches[i]->ref->plane->d;
//			pln=Tcr.transformPlane(pln);
//			Plane plane;
//			plane.normal=pln.block<3,1>(0,0);
//			plane.d=pln(3);
//			tmp=plane.projected_point(Tcr.transformPoint(scan->projective_ray_matches[i]->fused_edge_point_ref));
			pt2.x=tmp(0);
			pt2.y=tmp(1);
			pt2.z=tmp(2);
			// draw lines between current and reference occluded points;
			sprintf(id,"%dres_shadow_ref",i);
			vis->addLine<pcl::PointXYZRGBA>(pt22,pt2,255,255,0,id);//yellow
		}

		sprintf(id,"edge_cur%d",scan->id);//red - fused edge points;
		pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGBA> color_cur (edge_cur,255,0,0);
		if (!vis->updatePointCloud (edge_cur, color_cur, id))
			vis->addPointCloud (edge_cur, color_cur, id);
		vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, id);

//		sprintf(id,"edge_ref%d",scan->id);//blue
//		pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGBA> color_ref (edge_ref,0,0,255);
//		if (!vis->updatePointCloud (edge_ref, color_ref, id))
//			vis->addPointCloud (edge_ref, color_ref, id);
//		vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, id);

		sprintf(id,"edge_cur_occluded%d",scan->id);//magenta
		pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGBA> color_cur_tmp (edge_cur_occluded,255,0,255);
		if (!vis->updatePointCloud (edge_cur_occluded, color_cur_tmp, id))
			vis->addPointCloud (edge_cur_occluded, color_cur_tmp, id);
		vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, id);

		sprintf(id,"edge_ref_occluded%d",scan->id);//cyan
		pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGBA> color_ref_tmp (edge_ref_occluded,0,255,255);
		if (!vis->updatePointCloud (edge_ref_occluded, color_ref_tmp, id))
			vis->addPointCloud (edge_ref_occluded, color_ref_tmp, id);
		vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, id);


//		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr edge (new pcl::PointCloud<pcl::PointXYZRGBA>);
//		edge->resize(scan->scan_ref->edge_points.size());
//		for(size_t i=0;i<scan->scan_ref->edge_points.size();i++)
//		{
//			if(scan->scan_ref->edge_points[i]->isEdge==false)
//				continue;
//			edge->at(i).x=scan->scan_ref->edge_points[i]->xyz(0);
//			edge->at(i).y=scan->scan_ref->edge_points[i]->xyz(1);
//			edge->at(i).z=scan->scan_ref->edge_points[i]->xyz(2);
//		}
//		sprintf(id,"edgePoints%d",scan->id);
//		pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGBA> color_edge (edge, red[1], grn[1], blu[1]);
//		if (!vis->updatePointCloud (edge, color_edge, id))
//			vis->addPointCloud (edge, color_edge, id);
//		vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, id);

	}




	void displayMatchedShadowPoints(Scan *scan, Transform Tcr, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
		char id[50];
		// 							0    1    2    3    4    5    6    7    8    9   10   11   12   13
		unsigned char red [14] = {255,   0,   0, 255, 255,   0, 130,   0,   0, 130, 130,   0, 130, 255};
		unsigned char grn [14] = {  0, 255,   0, 255,   0, 255,   0, 130,   0, 130,   0, 130, 130, 255};
		unsigned char blu [14] = {  0,   0, 255,   0, 255, 255,   0,   0, 130,   0, 130, 130, 130, 255};
		pcl::PointXYZRGBA pt1,pt2;

		vis->removeAllPointClouds();
		vis->removeAllShapes();

		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr plane (new pcl::PointCloud<pcl::PointXYZRGBA>);

		sprintf(id,"scan_ref");
		pcl::transformPointCloud(*scan->scan_ref->point_cloud,*plane,Tcr.getMatrix4f());
		pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGBA> color1 (plane, red[8], grn[8], blu[8]);
		if (!vis->updatePointCloud (plane, color1, id))
			vis->addPointCloud (plane, color1, id);

		sprintf(id,"scan_cur");
		pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGBA> color2 (scan->point_cloud, red[6], grn[6], blu[6]);
		if (!vis->updatePointCloud (scan->point_cloud, color2, id))
			vis->addPointCloud (scan->point_cloud, color2, id);


//		sprintf(id,"scan%d",scan->id);
//		if (!vis->updatePointCloud (scan->point_cloud, id))
//			vis->addPointCloud (scan->point_cloud, id);

		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr edge_cur (new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr edge_ref (new pcl::PointCloud<pcl::PointXYZRGBA>);

		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr edge_cur_tmp (new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr edge_ref_tmp (new pcl::PointCloud<pcl::PointXYZRGBA>);

		edge_cur->resize(scan->point_matches.size());
		edge_ref->resize(scan->point_matches.size());
		for(size_t i=0;i<scan->point_matches.size();i++)
		{
			pt1.x=scan->point_matches[i].cur->xyz(0);
			pt1.y=scan->point_matches[i].cur->xyz(1);
			pt1.z=scan->point_matches[i].cur->xyz(2);
			edge_cur->at(i).x=scan->point_matches[i].cur->xyz(0);
			edge_cur->at(i).y=scan->point_matches[i].cur->xyz(1);
			edge_cur->at(i).z=scan->point_matches[i].cur->xyz(2);
			Eigen::Vector3d tmp=Tcr.transformPoint(scan->point_matches[i].ref->xyz);
			pt2.x=tmp(0);
			pt2.y=tmp(1);
			pt2.z=tmp(2);
			edge_ref->at(i).x=tmp(0);
			edge_ref->at(i).y=tmp(1);
			edge_ref->at(i).z=tmp(2);
			sprintf(id,"%dedge_occluding",i);
			vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,red[13], grn[13], blu[13],id);//
		}
		sprintf(id,"edge_cur_occluding%d",scan->id);
		pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGBA> color_cur_occluding (edge_cur, red[3], grn[3], blu[3]);//
		if (!vis->updatePointCloud (edge_cur, color_cur_occluding, id))
			vis->addPointCloud (edge_cur, color_cur_occluding, id);
		vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, id);
	
		sprintf(id,"edge_ref_occluding%d",scan->id);
		pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGBA> color_ref_occluding (edge_ref, red[5], grn[5], blu[5]);//
		if (!vis->updatePointCloud (edge_ref, color_ref_occluding, id))
			vis->addPointCloud (edge_ref, color_ref_occluding, id);
		vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, id);
	


//		edge_cur->resize(scan->point_matches_occluded.size());
//		edge_ref->resize(scan->point_matches_occluded.size());
//		edge_cur_tmp->resize(scan->point_matches_occluded.size());
//		edge_ref_tmp->resize(scan->point_matches_occluded.size());
//		int count_ed=0, count_ing=0;
//		for(size_t i=0;i<scan->point_matches_occluded.size();i++)
//		{
//			if(scan->point_matches_occluded[i].cur->plane==0)
//				continue;
//			pt1.x=scan->point_matches_occluded[i].cur->xyz(0);
//			pt1.y=scan->point_matches_occluded[i].cur->xyz(1);
//			pt1.z=scan->point_matches_occluded[i].cur->xyz(2);
//			edge_cur->at(i).x=scan->point_matches_occluded[i].cur->xyz(0);
//			edge_cur->at(i).y=scan->point_matches_occluded[i].cur->xyz(1);
//			edge_cur->at(i).z=scan->point_matches_occluded[i].cur->xyz(2);
//			Eigen::Vector3d tmp=scan->point_matches_occluded[i].cur->plane->projected_point(Tcr.transformPoint(scan->point_matches_occluded[i].ref->xyz));
//			pt2.x=tmp(0);
//			pt2.y=tmp(1);
//			pt2.z=tmp(2);
//			edge_ref->at(i).x=tmp(0);
//			edge_ref->at(i).y=tmp(1);
//			edge_ref->at(i).z=tmp(2);
//			sprintf(id,"%dedge_occluded",i);
//			vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,red[2], grn[2], blu[2],id);//blue
//			count_ed++;
//	//		if(scan->point_matches_occluded[i].cur->occluding==0 || scan->point_matches_occluded[i].ref->occluding==0)
//	//		{
//	//			continue;
//	//		}
//	//		else
//			{
//				pt1.x=scan->point_matches_occluded[i].cur->occluding->xyz(0);
//				pt1.y=scan->point_matches_occluded[i].cur->occluding->xyz(1);
//				pt1.z=scan->point_matches_occluded[i].cur->occluding->xyz(2);
//				edge_cur_tmp->at(i).x=scan->point_matches_occluded[i].cur->occluding->xyz(0);
//				edge_cur_tmp->at(i).y=scan->point_matches_occluded[i].cur->occluding->xyz(1);
//				edge_cur_tmp->at(i).z=scan->point_matches_occluded[i].cur->occluding->xyz(2);
//				Eigen::Vector3d tmp=Tcr.transformPoint(scan->point_matches_occluded[i].ref->xyz);
//				pt2.x=tmp(0);
//				pt2.y=tmp(1);
//				pt2.z=tmp(2);
//				edge_ref_tmp->at(i).x=tmp(0);
//				edge_ref_tmp->at(i).y=tmp(1);
//				edge_ref_tmp->at(i).z=tmp(2);
//				sprintf(id,"%dedge_occluded_ing",i);
//				vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,red[13], grn[13], blu[13],id);//blue
//				count_ing++;
//			}
//		}
//		std::cout<<"occluded matches: "<<count_ed<<std::endl;
//		std::cout<<"occluded_ing matches: "<<count_ing<<std::endl;
//
//		sprintf(id,"edge_cur%d",scan->id);
//		pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGBA> color_cur (edge_cur, red[0], grn[0], blu[0]);//red
//		if (!vis->updatePointCloud (edge_cur, color_cur, id))
//			vis->addPointCloud (edge_cur, color_cur, id);
//		vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, id);
//
//		sprintf(id,"edge_ref%d",scan->id);
//		pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGBA> color_ref (edge_ref, red[1], grn[1], blu[1]);//green
//		if (!vis->updatePointCloud (edge_ref, color_ref, id))
//			vis->addPointCloud (edge_ref, color_ref, id);
//		vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, id);
//
//		sprintf(id,"edge_cur_tmp%d",scan->id);
//		pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGBA> color_cur_tmp (edge_cur_tmp, red[3], grn[3], blu[3]);//red
//		if (!vis->updatePointCloud (edge_cur_tmp, color_cur_tmp, id))
//			vis->addPointCloud (edge_cur_tmp, color_cur_tmp, id);
//		vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, id);
//
//		sprintf(id,"edge_ref_tmp%d",scan->id);
//		pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGBA> color_ref_tmp (edge_ref_tmp, red[5], grn[5], blu[5]);//green
//		if (!vis->updatePointCloud (edge_ref_tmp, color_ref_tmp, id))
//			vis->addPointCloud (edge_ref_tmp, color_ref_tmp, id);
//		vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, id);


	}



	void displayTraj(Map *map, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
	//	vis->removeAllPointClouds();
		vis->removeAllShapes();
		char id[20];
		pcl::PointXYZRGBA pt1,pt2;
		double scale=0.1;
		Transform Tg0=map->scans[0]->Tcg_gt.inv();
		for(size_t i=0;i<map->scans.size();i++)
		{
			Transform Tgc=map->scans[i]->Tcg.inv();
			Eigen::Vector3d x=Tgc.R.block<3,1>(0,0);
			Eigen::Vector3d y=Tgc.R.block<3,1>(0,1);
			Eigen::Vector3d z=Tgc.R.block<3,1>(0,2);
			pt1.x=Tgc.t(0);
			pt1.y=Tgc.t(1);
			pt1.z=Tgc.t(2);
			pt2.x=pt1.x+z(0)*scale;
			pt2.y=pt1.y+z(1)*scale;
			pt2.z=pt1.z+z(2)*scale;
			sprintf(id,"%dz",map->scans[i]->id);
			vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,0,0,255,id);
			if(i>0)
			{
				Transform Tgc_pre=map->scans[i]->scan_ref->Tcg.inv();
				pt2.x=Tgc_pre.t(0);
				pt2.y=Tgc_pre.t(1);
				pt2.z=Tgc_pre.t(2);
				sprintf(id,"%dtraj",map->scans[i]->id);
				vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,255,255,255,id);
			}

			Tgc=Tg0.inv()*map->scans[i]->Tcg_gt.inv();
			if(map->scans[i]->Tcg_gt.t.norm()==0 || map->scans[i]->scan_ref->Tcg_gt.t.norm()==0)
				continue;
	//		std::cout<<map->scans[i]->Tcg_gt.t.transpose()<<std::endl;
			pt1.x=Tgc.t(0);
			pt1.y=Tgc.t(1);
			pt1.z=Tgc.t(2);
			if(i>0)
			{
				Transform Tgc_pre=Tg0.inv()*map->scans[i]->scan_ref->Tcg_gt.inv();
				pt2.x=Tgc_pre.t(0);
				pt2.y=Tgc_pre.t(1);
				pt2.z=Tgc_pre.t(2);
				sprintf(id,"%dtraj_gt",map->scans[i]->id);
				vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,0,255,0,id);
			}
		}
		for(size_t i=0;i<map->loop_closure.size();i++)
		{
			if(map->loop_closure[i].match_score==-1)
				continue;
	//		if(map->loop_closure[i].delta_time<10)
	//			continue;
	//		if(map->loop_closure[i].Tcr.t.norm()>0.5 || acos((map->loop_closure[i].Tcr.R.trace()-1.0)/2.0)>0.3)
	//			continue;
			Transform Tgc=map->loop_closure[i].scan_cur->Tcg.inv();
			Transform Tgr=map->loop_closure[i].scan_ref->Tcg.inv();
			pt1.x=Tgc.t(0);
			pt1.y=Tgc.t(1);
			pt1.z=Tgc.t(2);
			pt2.x=Tgr.t(0);
			pt2.y=Tgr.t(1);
			pt2.z=Tgr.t(2);
			sprintf(id,"%dlc",i);
			vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,255,0,0,id);
		}
		vis->spin();
		for(size_t i=0;i<map->loop_closure.size();i++)
		{
			if(map->loop_closure[i].match_score==-1)
				continue;
			sprintf(id,"%dlc",i);
			vis->removeShape(id);
		}
		for(size_t i=0;i<map->loop_closure.size();i++)
		{
			if(map->loop_closure[i].match_score==-1)
				continue;
	//		if(map->loop_closure[i].delta_time<10)
	//			continue;
			if(map->loop_closure[i].Tcr.t.norm()>0.2 || acos((map->loop_closure[i].Tcr.R.trace()-1.0)/2.0)>0.2)
				continue;
			Transform Tgc=map->loop_closure[i].scan_cur->Tcg.inv();
			Transform Tgr=map->loop_closure[i].scan_ref->Tcg.inv();
			pt1.x=Tgc.t(0);
			pt1.y=Tgc.t(1);
			pt1.z=Tgc.t(2);
			pt2.x=Tgr.t(0);
			pt2.y=Tgr.t(1);
			pt2.z=Tgr.t(2);
			sprintf(id,"%dlc",i);
			vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,255,0,0,id);
		}
	}


	void display_addCameraPose(Scan *scan, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
	//	vis->removeAllPointClouds();
	//	vis->removeAllShapes();
		char id[20];
		pcl::PointXYZRGBA pt1,pt2;
		double scale=0.1;
	//	for(size_t i=0;i<map->scans.size();i++)
		{
			Transform Tgc=scan->Tcg.inv();
			Eigen::Vector3d x=Tgc.R.block<3,1>(0,0);
			Eigen::Vector3d y=Tgc.R.block<3,1>(0,1);
			Eigen::Vector3d z=Tgc.R.block<3,1>(0,2);
			pt1.x=Tgc.t(0);
			pt1.y=Tgc.t(1);
			pt1.z=Tgc.t(2);
			// x - green
			pt2.x=pt1.x+x(0)*scale;
			pt2.y=pt1.y+x(1)*scale;
			pt2.z=pt1.z+x(2)*scale;
			sprintf(id,"%dx",scan->id);
			vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,0,255,0,id);
			// y - blue
			pt2.x=pt1.x+y(0)*scale;
			pt2.y=pt1.y+y(1)*scale;
			pt2.z=pt1.z+y(2)*scale;
			sprintf(id,"%dy",scan->id);
			vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,0,0,255,id);
			// z - red
			pt2.x=pt1.x+z(0)*scale;
			pt2.y=pt1.y+z(1)*scale;
			pt2.z=pt1.z+z(2)*scale;
			sprintf(id,"%dz",scan->id);
			vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,255,0,0,id);
		}
	}


	void display_LoopClosure(LoopClosure loop_closure, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
		char id[20];
		unsigned char red [14] = {255,   0,   0, 255, 255,   0, 130,   0,   0, 130, 130,   0, 130, 255};
		unsigned char grn [14] = {  0, 255,   0, 255,   0, 255,   0, 130,   0, 130,   0, 130, 130, 255};
		unsigned char blu [14] = {  0,   0, 255,   0, 255, 255,   0,   0, 130,   0, 130, 130, 130, 255};

		vis->removeAllPointClouds();
		vis->removeAllShapes();
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr plane (new pcl::PointCloud<pcl::PointXYZRGBA>);

		Scan *scan=loop_closure.scan_cur;
		Scan *scan_ref=loop_closure.scan_ref;

		sprintf(id,"cur");
		plane->resize(scan->edge_points.size());
		for(size_t i=0;i<scan->edge_points.size();i++)
		{
			plane->at(i).x=scan->edge_points[i]->xyz[0];
			plane->at(i).y=scan->edge_points[i]->xyz[1];
			plane->at(i).z=scan->edge_points[i]->xyz[2];
		}
		pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGBA> color1 (plane, red[0], grn[0], blu[0]);
		if (!vis->updatePointCloud (plane, color1, id))
			vis->addPointCloud (plane, color1, id);

		sprintf(id,"ref");
		plane->resize(scan_ref->edge_points.size());
		for(size_t i=0;i<scan_ref->edge_points.size();i++)
		{
			plane->at(i).x=scan_ref->edge_points[i]->xyz[0];
			plane->at(i).y=scan_ref->edge_points[i]->xyz[1];
			plane->at(i).z=scan_ref->edge_points[i]->xyz[2];
		}
		pcl::transformPointCloud(*plane,*plane,loop_closure.Tcr.getMatrix4f());
		pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGBA> color2 (plane, red[2], grn[2], blu[2]);
		if (!vis->updatePointCloud (plane, color2, id))
			vis->addPointCloud (plane, color2, id);
	}


	void displayLines(Scan *scan, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
		char id[20];
		unsigned char red [14] = {255,   0,   0, 255, 255,   0, 130,   0,   0, 130, 130,   0, 130, 255};
		unsigned char grn [14] = {  0, 255,   0, 255,   0, 255,   0, 130,   0, 130,   0, 130, 130, 255};
		unsigned char blu [14] = {  0,   0, 255,   0, 255, 255,   0,   0, 130,   0, 130, 130, 130, 255};

		vis->removeAllPointClouds();
		vis->removeAllShapes();
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr edge (new pcl::PointCloud<pcl::PointXYZRGBA>);

		// add raw scan data;
		sprintf(id,"scan");
		if (!vis->updatePointCloud (scan->point_cloud, id))
			vis->addPointCloud (scan->point_cloud, id);
//		vis->spin();
		
		cv::Mat img=scan->img_rgb;//cv::Mat::zeros(480,640,CV_8UC3);

		// show scan->lines_occluding;
		pcl::PointXYZRGBA pt1,pt2;
		size_t i=0;
//		cout<<"occluding lines: "<<scan->lines_occluding.size()<<endl;
		for(std::list<Line*>::iterator it_line=scan->lines_occluding.begin();it_line!=scan->lines_occluding.end();it_line++)
		{
			pt1.x=(*it_line)->end_point_1(0);
			pt1.y=(*it_line)->end_point_1(1);
			pt1.z=(*it_line)->end_point_1(2);
			pt2.x=(*it_line)->end_point_2(0);
			pt2.y=(*it_line)->end_point_2(1);
			pt2.z=(*it_line)->end_point_2(2);
			sprintf(id,"line_occluding%d",i);
			vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,255,0,0,id);//red

			Eigen::Vector3d u1=scan->cam.project((*it_line)->end_point_1);
			Eigen::Vector3d u2=scan->cam.project((*it_line)->end_point_2);
			cv::Point p1=cv::Point(u1(0),u1(1));
			cv::Point p2=cv::Point(u2(0),u2(1));
			cv::line(img,p1,p2,cv::Scalar(0,0,255));
//			cout<<"\t"<<p1.x<<"\t"<<p1.y<<"\t"<<p2.x<<"\t"<<p2.y<<endl;

			edge->resize((*it_line)->points.size()*2);//+2);
//			for(size_t j=0;j<scan->lines_occluding[i]->points.size();j++)
			size_t j=0;
			for(std::list<EdgePoint*>::iterator it=(*it_line)->points.begin();it!=(*it_line)->points.end();it++)
			{
				edge->at(j).x=(*it)->xyz(0);
				edge->at(j).y=(*it)->xyz(1);
				edge->at(j).z=(*it)->xyz(2);
				edge->at(j).r=255;
				edge->at(j).g=0;
				edge->at(j).b=255;//magenta
				j++;
			}
			i++;

			sprintf(id,"edge_occluding%d",i);
			if (!vis->updatePointCloud (edge, id))
				vis->addPointCloud (edge, id);
			vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, id);
//			vis->spin();
		}

		// show scan->lines_occluded;
//		if(false)
//		cout<<"occluded lines: "<<scan->lines_occluded.size()<<endl;
		/*
		for(std::list<Line*>::iterator it_line=scan->lines_occluded.begin();it_line!=scan->lines_occluded.end();it_line++)
		{
			pt1.x=(*it_line)->end_point_1(0);
			pt1.y=(*it_line)->end_point_1(1);
			pt1.z=(*it_line)->end_point_1(2);
			pt2.x=(*it_line)->end_point_2(0);
			pt2.y=(*it_line)->end_point_2(1);
			pt2.z=(*it_line)->end_point_2(2);
			sprintf(id,"line_occluded%d",i);
			vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,0,255,0,id);//green

			Eigen::Vector3d u1=scan->cam.project((*it_line)->end_point_1);
			Eigen::Vector3d u2=scan->cam.project((*it_line)->end_point_2);
			cv::Point p1=cv::Point(u1(0),u1(1));
			cv::Point p2=cv::Point(u2(0),u2(1));
			cv::line(img,p1,p2,cv::Scalar(0,255,0));
//			cout<<"\t"<<p1.x<<"\t"<<p1.y<<"\t"<<p2.x<<"\t"<<p2.y<<endl;

			edge->resize((*it_line)->points.size());//*2+2);
//			for(size_t j=0;j<scan->lines_occluding[i]->points.size();j++)
			size_t j=0;
			for(std::list<EdgePoint*>::iterator it=(*it_line)->points.begin();it!=(*it_line)->points.end();it++)
			{
				edge->at(j).x=(*it)->xyz(0);
				edge->at(j).y=(*it)->xyz(1);
				edge->at(j).z=(*it)->xyz(2);
				edge->at(j).r=0;
				edge->at(j).g=255;
				edge->at(j).b=255;//cyan
				j++;
			}
			i++;

			sprintf(id,"edge_occluded%d",i);
			if (!vis->updatePointCloud (edge, id))
				vis->addPointCloud (edge, id);
			vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, id);
//			vis->spin();
		}
		*/

	}

	void displayMatchedLines(Scan *scan, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
		char id[20];
		unsigned char red [14] = {255,   0,   0, 255, 255,   0, 130,   0,   0, 130, 130,   0, 130, 255};
		unsigned char grn [14] = {  0, 255,   0, 255,   0, 255,   0, 130,   0, 130,   0, 130, 130, 255};
		unsigned char blu [14] = {  0,   0, 255,   0, 255, 255,   0,   0, 130,   0, 130, 130, 130, 255};

		vis->removeAllPointClouds();
		vis->removeAllShapes();
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr edge (new pcl::PointCloud<pcl::PointXYZRGBA>);

		// add raw scan data;
		sprintf(id,"scan");
		if (!vis->updatePointCloud (scan->point_cloud, id))
			vis->addPointCloud (scan->point_cloud, id);
		
		size_t i=0;
		for(std::vector<LinePair>::iterator it_line=scan->line_matches.begin();it_line!=scan->line_matches.end();it_line++)
		{
			edge->resize((*it_line).cur->points.size()+(*it_line).ref->points.size());
			size_t j=0;
			for(std::list<EdgePoint*>::iterator it=(*it_line).cur->points.begin();it!=(*it_line).cur->points.end();it++)
			{
				edge->at(j).x=(*it)->xyz(0);
				edge->at(j).y=(*it)->xyz(1);
				edge->at(j).z=(*it)->xyz(2);
				edge->at(j).r=0;
				edge->at(j).g=0;
				edge->at(j).b=255;
				j++;
			}
			for(std::list<EdgePoint*>::iterator it=(*it_line).ref->points.begin();it!=(*it_line).ref->points.end();it++)
			{
				edge->at(j).x=(*it)->xyz(0);
				edge->at(j).y=(*it)->xyz(1);
				edge->at(j).z=(*it)->xyz(2);
				edge->at(j).r=255;
				edge->at(j).g=0;
				edge->at(j).b=0;
				j++;
			}
			
			sprintf(id,"edge_occluding%d",i);
			if (!vis->updatePointCloud (edge, id))
				vis->addPointCloud (edge, id);
			vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, id);
			i++;
		}
		
	}

	// **********************************************************************
	// the following two functions are for the debugging of line matching;
	void displayScan(Scan *scan, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
		char id[20];
		unsigned char red [14] = {255,   0,   0, 255, 255,   0, 130,   0,   0, 130, 130,   0, 130, 255};
		unsigned char grn [14] = {  0, 255,   0, 255,   0, 255,   0, 130,   0, 130,   0, 130, 130, 255};
		unsigned char blu [14] = {  0,   0, 255,   0, 255, 255,   0,   0, 130,   0, 130, 130, 130, 255};

		vis->removeAllPointClouds();
		vis->removeAllShapes();

		// add raw scan data;
		sprintf(id,"scan");
		if (!vis->updatePointCloud (scan->point_cloud, id))
			vis->addPointCloud (scan->point_cloud, id);
	}

	void displayLineNode(Node_InterpTree_Line *node, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
		while(!node->isRoot)
		{
			std::cout<<"<"<<node->line_cur->id<<","<<node->line_ref->id<<">,";
			displayLine(node->line_cur,node->line_ref,vis);
			node=node->parent;
		}
		std::cout<<std::endl;
	}

	void removeLineNode(Node_InterpTree_Line *node, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
		char id[20];
		while(!node->isRoot)
		{
			int i=node->line_cur->id*100+node->line_ref->id;
			sprintf(id,"%d",i);
			vis->removePointCloud(id);
			node=node->parent;
		}
	}

	void displayLine(Line *line_cur, Line *line_ref, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
		char id[20];
		unsigned char red [14] = {255,   0,   0, 255, 255,   0, 130,   0,   0, 130, 130,   0, 130, 255};
		unsigned char grn [14] = {  0, 255,   0, 255,   0, 255,   0, 130,   0, 130,   0, 130, 130, 255};
		unsigned char blu [14] = {  0,   0, 255,   0, 255, 255,   0,   0, 130,   0, 130, 130, 130, 255};

		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr edge (new pcl::PointCloud<pcl::PointXYZRGBA>);
		
		edge->resize(line_cur->points.size()+line_ref->points.size());
		int j=0;
		for(std::list<EdgePoint*>::iterator it=line_cur->points.begin();it!=line_cur->points.end();it++)
		{
			edge->at(j).x=(*it)->xyz(0);
			edge->at(j).y=(*it)->xyz(1);
			edge->at(j).z=(*it)->xyz(2);
			edge->at(j).r=0;
			edge->at(j).g=0;
			edge->at(j).b=255;
			j++;
		}
		for(std::list<EdgePoint*>::iterator it=line_ref->points.begin();it!=line_ref->points.end();it++)
		{
			edge->at(j).x=(*it)->xyz(0);
			edge->at(j).y=(*it)->xyz(1);
			edge->at(j).z=(*it)->xyz(2);
			edge->at(j).r=255;
			edge->at(j).g=0;
			edge->at(j).b=0;
			j++;
		}

		int i=line_cur->id*100+line_ref->id;
		sprintf(id,"%d",i);
		if (!vis->updatePointCloud (edge, id))
			vis->addPointCloud (edge, id);
		vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, id);

	}


	void displayMap(Map *map, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
		char id[20];
		unsigned char red [14] = {255,   0,   0, 255, 255,   0, 130,   0,   0, 130, 130,   0, 130, 255};
		unsigned char grn [14] = {  0, 255,   0, 255,   0, 255,   0, 130,   0, 130,   0, 130, 130, 255};
		unsigned char blu [14] = {  0,   0, 255,   0, 255, 255,   0,   0, 130,   0, 130, 130, 130, 255};
		
		vis->removeAllPointClouds();
		vis->removeAllShapes();
		pcl::PointXYZRGBA pt1,pt2;
		double scale=0.1;
		Transform Tg0=map->cameras[0].inv();
		for(size_t i=0;i<map->cameras.size();i++)
		{
			Transform Tgc=map->cameras[i].inv();
			Eigen::Vector3d x=Tgc.R.block<3,1>(0,0);
			Eigen::Vector3d y=Tgc.R.block<3,1>(0,1);
			Eigen::Vector3d z=Tgc.R.block<3,1>(0,2);
			pt1.x=Tgc.t(0);
			pt1.y=Tgc.t(1);
			pt1.z=Tgc.t(2);
			pt2.x=pt1.x+z(0)*scale;
			pt2.y=pt1.y+z(1)*scale;
			pt2.z=pt1.z+z(2)*scale;
//			std::cout<<"camera "<<i<<std::endl;
//			std::cout<<pt1.x<<" "<<pt1.y<<" "<<pt1.z<<std::endl;
//			std::cout<<pt2.x<<" "<<pt2.y<<" "<<pt2.z<<std::endl;
			sprintf(id,"%dz",i);
			vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,0,0,255,id);//z-blue
			pt2.x=pt1.x+x(0)*scale;
			pt2.y=pt1.y+x(1)*scale;
			pt2.z=pt1.z+x(2)*scale;
//			std::cout<<pt2.x<<" "<<pt2.y<<" "<<pt2.z<<std::endl;
			sprintf(id,"%dx",i);
			vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,255,0,0,id);//x-red
			pt2.x=pt1.x+y(0)*scale;
			pt2.y=pt1.y+y(1)*scale;
			pt2.z=pt1.z+y(2)*scale;
//			std::cout<<pt2.x<<" "<<pt2.y<<" "<<pt2.z<<std::endl;
			sprintf(id,"%dy",i);
			vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,0,255,0,id);//y-green
			if(i>0)
			{
				Transform Tgc_pre=map->cameras[i-1].inv();
				pt2.x=Tgc_pre.t(0);
				pt2.y=Tgc_pre.t(1);
				pt2.z=Tgc_pre.t(2);
				sprintf(id,"%dtraj",i);
				vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,255,255,255,id);
			}
		}

		const int plane_size=30;
		const double delta=0.03;//1cm
		for(int i=0;i<map->planes.size();i++)
		{
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr plane (new pcl::PointCloud<pcl::PointXYZRGBA>);
			plane->resize(plane_size*plane_size);
			Eigen::Vector3d n=map->planes[i]->pi.block<3,1>(0,0);
			double d=map->planes[i]->pi(3)/n.norm();
			n.normalize();
//			Eigen::Vector3d p=-d*n;
			Eigen::Vector3d p=map->planes[i]->pG;
			Eigen::Matrix<double,1,1> tmp=p.transpose()*n;
			p=p-(d+tmp(0,0))*n;
			double max=DBL_MIN;
			Eigen::Matrix3d e=Eigen::Matrix3d::Identity();
			int idx=-1;
			for(int j=0;j<3;j++)
			{
				double tmp=fabs(n.transpose()*e.block<3,1>(0,j));
				if(tmp>max)
				{
					max=tmp;
					idx=j;
				}
			}
			if(idx==0) // n ~ x axis
			{
				for(int y=0;y<plane_size;y++)
				{
					for(int z=0;z<plane_size;z++)
					{
						double yy=p(1)+y*delta-double(plane_size)*delta/2.0;
						double zz=p(2)+z*delta-double(plane_size)*delta/2.0;
						plane->at(y*plane_size+z).y=yy;
						plane->at(y*plane_size+z).z=zz;
						plane->at(y*plane_size+z).x=(-d-yy*n(1)-zz*n(2))/n(0);
					}
				}
				sprintf(id,"plane%d",i);
				pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGBA> color1 (plane, 0,255,0);
				if (!vis->updatePointCloud (plane, color1, id))
					vis->addPointCloud (plane, color1, id);
			}
			else if(idx==1) // n ~ y axis
			{
				for(int x=0;x<plane_size;x++)
				{
					for(int z=0;z<plane_size;z++)
					{
						double xx=p(0)+x*delta-double(plane_size)*delta/2.0;
						double zz=p(2)+z*delta-double(plane_size)*delta/2.0;
						plane->at(x*plane_size+z).x=xx;
						plane->at(x*plane_size+z).z=zz;
						plane->at(x*plane_size+z).y=(-d-xx*n(0)-zz*n(2))/n(1);
					}
				}
				sprintf(id,"plane%d",i);
				pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGBA> color1 (plane, 0,255,0);
				if (!vis->updatePointCloud (plane, color1, id))
					vis->addPointCloud (plane, color1, id);
			}
			else if(idx==2) // n ~ z axis
			{
				for(int x=0;x<plane_size;x++)
				{
					for(int y=0;y<plane_size;y++)
					{
						double xx=p(0)+x*delta-double(plane_size)*delta/2.0;
						double yy=p(1)+y*delta-double(plane_size)*delta/2.0;
						plane->at(x*plane_size+y).x=xx;
						plane->at(x*plane_size+y).y=yy;
						plane->at(x*plane_size+y).z=(-d-xx*n(0)-yy*n(1))/n(2);
					}
				}
				sprintf(id,"plane%d",i);
				pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGBA> color1 (plane, 0,255,0);
				if (!vis->updatePointCloud (plane, color1, id))
					vis->addPointCloud (plane, color1, id);
			}
		}

		scale=0.5;
		for(int i=0;i<map->lines.size();i++)
		{
			Eigen::Vector3d u=map->lines[i]->L.block<3,1>(0,0);
			Eigen::Vector3d v=map->lines[i]->L.block<3,1>(3,0);
//			std::cout<<"line"<<std::endl;
//			std::cout<<u.transpose()<<std::endl;
//			std::cout<<v.transpose()<<std::endl;
			u=u-Transform::skew_sym(map->lines[i]->pG)*v;
//			std::cout<<map->lines[i]->pG.transpose()<<std::endl;
//			std::cout<<u.transpose()<<std::endl;
			double d=u.norm();
			Eigen::Vector3d dir=v.cross(u);
			if(dir.norm()<1e-7) dir.setZero();
			else dir.normalize();
//			Eigen::Vector3d p=d*dir;
			Eigen::Vector3d p=d*dir+map->lines[i]->pG;
			pt1.x=p(0);
			pt1.y=p(1);
			pt1.z=p(2);
			pt2.x=p(0)+v(0)*scale;
			pt2.y=p(1)+v(1)*scale;
			pt2.z=p(2)+v(2)*scale;
//			std::cout<<pt1.x<<" "<<pt1.y<<" "<<pt1.z<<std::endl;
			sprintf(id,"line%d",i);
			vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,255,0,0,id);
			vis->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, id);
		}

	}

	void displayPlaneLM(PlaneLM *pl, int plane_size, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
		char id[20];
		const double delta=0.01;//1cm
		sprintf(id,"plane");
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr plane (new pcl::PointCloud<pcl::PointXYZRGBA>);
		plane->resize(plane_size*plane_size);
		Eigen::Vector3d n=pl->pi.block<3,1>(0,0);
		double d=pl->pi(3)/n.norm();
		n.normalize();
		Eigen::Vector3d p=pl->pG;
		Eigen::Matrix<double,1,1> tmp=p.transpose()*n;
		p=p-(d+tmp(0,0))*n;
		double max=DBL_MIN;
		Eigen::Matrix3d e=Eigen::Matrix3d::Identity();
		int idx=-1;
		for(int j=0;j<3;j++)
		{
			double tmp=fabs(n.transpose()*e.block<3,1>(0,j));
			if(tmp>max)
			{
				max=tmp;
				idx=j;
			}
		}
		if(idx==0) // n ~ x axis
		{
			for(int y=0;y<plane_size;y++)
			{
				for(int z=0;z<plane_size;z++)
				{
					double yy=p(1)+y*delta-double(plane_size)*delta/2.0;
					double zz=p(2)+z*delta-double(plane_size)*delta/2.0;
					plane->at(y*plane_size+z).y=yy;
					plane->at(y*plane_size+z).z=zz;
					plane->at(y*plane_size+z).x=(-d-yy*n(1)-zz*n(2))/n(0);
				}
			}
		}
		else if(idx==1) // n ~ y axis
		{
			for(int x=0;x<plane_size;x++)
			{
				for(int z=0;z<plane_size;z++)
				{
					double xx=p(0)+x*delta-double(plane_size)*delta/2.0;
					double zz=p(2)+z*delta-double(plane_size)*delta/2.0;
					plane->at(x*plane_size+z).x=xx;
					plane->at(x*plane_size+z).z=zz;
					plane->at(x*plane_size+z).y=(-d-xx*n(0)-zz*n(2))/n(1);
				}
			}
		}
		else if(idx==2) // n ~ z axis
		{
			for(int x=0;x<plane_size;x++)
			{
				for(int y=0;y<plane_size;y++)
				{
					double xx=p(0)+x*delta-double(plane_size)*delta/2.0;
					double yy=p(1)+y*delta-double(plane_size)*delta/2.0;
					plane->at(x*plane_size+y).x=xx;
					plane->at(x*plane_size+y).y=yy;
					plane->at(x*plane_size+y).z=(-d-xx*n(0)-yy*n(1))/n(2);
				}
			}
		}
		pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGBA> color1 (plane, 0,0,255);
		if (!vis->updatePointCloud (plane, color1, id))
			vis->addPointCloud (plane, color1, id);
		vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, id);
	}

	void displayLineShadow(Scan *scan, Map *map, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
		char id[20];
		unsigned char red [14] = {255,   0,   0, 255, 255,   0, 130,   0,   0, 130, 130,   0, 130, 255};
		unsigned char grn [14] = {  0, 255,   0, 255,   0, 255,   0, 130,   0, 130,   0, 130, 130, 255};
		unsigned char blu [14] = {  0,   0, 255,   0, 255, 255,   0,   0, 130,   0, 130, 130, 130, 255};
		
		vis->removeAllPointClouds();
		vis->removeAllShapes();

		sprintf(id,"scan");
		if (!vis->updatePointCloud (scan->point_cloud, id))
			vis->addPointCloud (scan->point_cloud, id);

		const int plane_size=50;
		const double scale=0.5;
		pcl::PointXYZRGBA pt1,pt2;
		for(int i=0;i<map->observ_shadows.size();i++)
		{
			vis->removeAllShapes();

			double idx_camera=map->observ_shadows[i]->idx_camera;
			double idx_plane=map->observ_shadows[i]->idx_plane;
			double idx_line=map->observ_shadows[i]->idx_line;

			// display plane;
			displayPlaneLM(map->planes[idx_plane],plane_size,vis);//blue

			// display occluding line;
			Eigen::Vector3d u=map->lines[idx_line]->L.block<3,1>(0,0);
			Eigen::Vector3d v=map->lines[idx_line]->L.block<3,1>(3,0);
//			std::cout<<"\noccluding line "<<u.transpose()<<" "<<v.transpose()<<std::endl;
			u=u-Transform::skew_sym(map->lines[idx_line]->pG)*v;
			double d=u.norm();
			Eigen::Vector3d dir=v.cross(u);
			if(dir.norm()<1e-7) dir.setZero();
			else dir.normalize();
			Eigen::Vector3d p=d*dir+map->lines[idx_line]->pG;
			pt1.x=p(0);
			pt1.y=p(1);
			pt1.z=p(2);
			pt2.x=p(0)+v(0)*scale;
			pt2.y=p(1)+v(1)*scale;
			pt2.z=p(2)+v(2)*scale;
//			std::cout<<pt1.x<<" "<<pt1.y<<" "<<pt1.z<<std::endl;
//			std::cout<<pt2.x<<" "<<pt2.y<<" "<<pt2.z<<std::endl;
			sprintf(id,"line");
			vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,255,0,0,id);//red
			vis->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, id);

			// display computed shadow;
			Vector6d L_pi=map->computeLineShadow(idx_camera,idx_plane,idx_line);
			u=L_pi.block<3,1>(0,0);
			v=L_pi.block<3,1>(3,0);
//			std::cout<<"\ncomputed shadow "<<u.transpose()<<" "<<v.transpose()<<std::endl;
			u=u-Transform::skew_sym(map->lines[idx_line]->pG)*v;
			d=u.norm();
			dir=v.cross(u);
			if(dir.norm()<1e-7) dir.setZero();
			else dir.normalize();
			p=d*dir+map->lines[idx_line]->pG;
			pt1.x=p(0);
			pt1.y=p(1);
			pt1.z=p(2);
			pt2.x=p(0)+v(0)*scale;
			pt2.y=p(1)+v(1)*scale;
			pt2.z=p(2)+v(2)*scale;
//			std::cout<<pt1.x<<" "<<pt1.y<<" "<<pt1.z<<std::endl;
//			std::cout<<pt2.x<<" "<<pt2.y<<" "<<pt2.z<<std::endl;
			sprintf(id,"shadow");
			vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,255,0,255,id);//magenta
			vis->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, id);

			// display observed shadow;
			L_pi=map->observ_shadows[i]->L;
			Transform Tgc=map->cameras[idx_camera].inv();
			L_pi=Tgc.getLineTransform()*L_pi;
			u=L_pi.block<3,1>(0,0);
			v=L_pi.block<3,1>(3,0);
//			std::cout<<"\nobserved shadow "<<u.transpose()<<" "<<v.transpose()<<std::endl;
			u=u-Transform::skew_sym(map->lines[idx_line]->pG)*v;
			d=u.norm();
			dir=v.cross(u);
			if(dir.norm()<1e-7) dir.setZero();
			else dir.normalize();
			p=d*dir+map->lines[idx_line]->pG;
			pt1.x=p(0);
			pt1.y=p(1);
			pt1.z=p(2);
			pt2.x=p(0)+v(0)*scale;
			pt2.y=p(1)+v(1)*scale;
			pt2.z=p(2)+v(2)*scale;
//			std::cout<<pt1.x<<" "<<pt1.y<<" "<<pt1.z<<std::endl;
//			std::cout<<pt2.x<<" "<<pt2.y<<" "<<pt2.z<<std::endl;
			sprintf(id,"observeshadow");
			vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,0,255,0,id);//green
			vis->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, id);

			vis->spin();
		}


	}

	void displayLineShadow(Scan *scan, Line *occluding, Line *occluded, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
		char id[20];

		vis->removeAllPointClouds();
		vis->removeAllShapes();

		sprintf(id,"scan");
		if (!vis->updatePointCloud (scan->point_cloud, id))
			vis->addPointCloud (scan->point_cloud, id);


		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr occluding_points (new pcl::PointCloud<pcl::PointXYZRGBA>);
		int j=0;
		{
			occluding_points->resize(occluding->points.size());
			for(std::list<EdgePoint*>::iterator it=occluding->points.begin();
												it!=occluding->points.end();it++)
			{
				occluding_points->at(j).x=(*it)->xyz(0);
				occluding_points->at(j).y=(*it)->xyz(1);
				occluding_points->at(j).z=(*it)->xyz(2);
				occluding_points->at(j).r=255;
				occluding_points->at(j).g=0;
				occluding_points->at(j).b=0;
				j++;
			}
		}
		sprintf(id,"occluding");
		if (!vis->updatePointCloud (occluding_points, id))
			vis->addPointCloud (occluding_points, id);
		vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, id);

		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr occluded_points (new pcl::PointCloud<pcl::PointXYZRGBA>);
		j=0;
		{
			occluded_points->resize(occluded->points.size());
			for(std::list<EdgePoint*>::iterator it=occluded->points.begin();
												it!=occluded->points.end();it++)
			{
				occluded_points->at(j).x=(*it)->xyz(0);
				occluded_points->at(j).y=(*it)->xyz(1);
				occluded_points->at(j).z=(*it)->xyz(2);
				occluded_points->at(j).r=0;
				occluded_points->at(j).g=255;
				occluded_points->at(j).b=0;
				j++;
			}
		}
		sprintf(id,"occluded");
		if (!vis->updatePointCloud (occluded_points, id))
			vis->addPointCloud (occluded_points, id);
		vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, id);
		
	}

	void displayLineShadowPlane(Scan *scan, Plane *plane, Line *occluded, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
		char id[20];

		vis->removeAllPointClouds();
		vis->removeAllShapes();

		sprintf(id,"scan");
		if (!vis->updatePointCloud (scan->point_cloud, id))
			vis->addPointCloud (scan->point_cloud, id);

		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr plane_points (new pcl::PointCloud<pcl::PointXYZRGBA>);
		int j=0;
		{
			plane_points->resize(plane->points.size());
			for(std::vector<Point>::iterator it=plane->points.begin();
											it!=plane->points.end();it++)
			{
				plane_points->at(j).x=(*it).xyz(0);
				plane_points->at(j).y=(*it).xyz(1);
				plane_points->at(j).z=(*it).xyz(2);
				plane_points->at(j).r=0;
				plane_points->at(j).g=0;
				plane_points->at(j).b=255;
				j++;
			}
		}
		sprintf(id,"plane");
		if (!vis->updatePointCloud (plane_points, id))
			vis->addPointCloud (plane_points, id);
		vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, id);

		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr occluded_points (new pcl::PointCloud<pcl::PointXYZRGBA>);
		j=0;
		{
			occluded_points->resize(occluded->points.size());
			for(std::list<EdgePoint*>::iterator it=occluded->points.begin();
												it!=occluded->points.end();it++)
			{
				occluded_points->at(j).x=(*it)->xyz(0);
				occluded_points->at(j).y=(*it)->xyz(1);
				occluded_points->at(j).z=(*it)->xyz(2);
				occluded_points->at(j).r=0;
				occluded_points->at(j).g=255;
				occluded_points->at(j).b=0;
				j++;
			}
		}
		sprintf(id,"occluded");
		if (!vis->updatePointCloud (occluded_points, id))
			vis->addPointCloud (occluded_points, id);
		vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, id);
		
	}


}
