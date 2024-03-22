/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@gmail.com
#
# Last modified:	2019-05-09 14:23
#
# Filename:		line_extraction.cpp
#
# Description: 
#
************************************************/

#include "line_extraction.h"

namespace ulysses
{
	void LineExtraction::extractLines(Scan *scan, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
		fp.open("extract_Lines.txt",std::ios::app);
		if(debug) fp<<std::endl<<"*****************************************************"<<std::endl;

		height=scan->cam.height;
		width=scan->cam.width;
		extractLinesHough(scan->edge_points,scan->lines_occluding,1.0);
		extractLinesHough(scan->edge_points_occluded,scan->lines_occluded,1.5);

		int id=0;
		if(debug) fp<<"\noccluded lines: "<<scan->lines_occluded.size()<<std::endl;
		for(std::list<Line*>::iterator it_line=scan->lines_occluded.begin();it_line!=scan->lines_occluded.end();it_line++)
		{
			(*it_line)->id=id++;
			generatePlucker(*it_line);
			compute_line_cov_inv(*it_line);
			if(debug) fp<<(*it_line)->id<<"\t"<<(*it_line)->u.transpose()<<"\t"<<(*it_line)->v.transpose()<<std::endl;
//			std::cout<<(*it_line)->id<<"\t"<<(*it_line)->u.transpose()<<"\t"<<(*it_line)->v.transpose()<<std::endl;
			// associate occluded lines with planes;
			Eigen::Vector3d u=(*it_line)->u;
			Eigen::Vector3d v=(*it_line)->v;
			double min=DBL_MAX;
			int idx=-1;
			for(int i=0;i<scan->observed_planes.size();i++)
			{
				Eigen::Vector3d n=scan->observed_planes[i]->normal;
				double d=scan->observed_planes[i]->d;
				Eigen::Vector3d tmp=u.cross(n)+d*v;
				double dist=tmp.norm();
				if(dist<min)
				{
					min=dist;
					idx=i;
				}
			}
			if(min<thres_shadow2plane)
			{
				(*it_line)->plane=scan->observed_planes[idx];
				if(debug) fp<<"\tplane\t"<<(*it_line)->plane->normal.transpose()<<std::endl;
//				std::cout<<"|uxn+dv| = "<<min<<std::endl;
//				displayLineShadowPlane(scan,scan->observed_planes[idx],*it_line,vis);
//				vis->spin();
			}
		}

		id=0;
		if(debug) fp<<"\noccluding lines: "<<scan->lines_occluding.size()<<std::endl;
		for(std::list<Line*>::iterator it_line=scan->lines_occluding.begin();it_line!=scan->lines_occluding.end();it_line++)
		{
			(*it_line)->id=id++;
			generatePlucker(*it_line);
			compute_line_cov_inv(*it_line);
			if(debug) fp<<(*it_line)->id<<"\t"<<(*it_line)->u.transpose()<<"\t"<<(*it_line)->v.transpose()<<std::endl;
//			// associate occluding lines with occluded lines;
//			Eigen::Vector3d u=(*it_line)->u;
//			Eigen::Vector3d v=(*it_line)->v;
//			double min=DBL_MAX;
//			std::list<Line*>::iterator idx_min=scan->lines_occluding.end();
//			for(std::list<Line*>::iterator it_occluded=scan->lines_occluded.begin();
//										   it_occluded!=scan->lines_occluded.end();it_occluded++)
//			{
//				Eigen::Vector3d u_pi=(*it_occluded)->u;
//				u.normalize();
//				u_pi.normalize();
//				double dist=u.transpose()*u_pi;
//				dist=fabs(dist);
//				dist=acos(dist)*180.0/M_PI;
//				if(dist<min)
//				{
//					min=dist;
//					idx_min=it_occluded;
//				}
//			}
//			if(idx_min!=scan->lines_occluding.end())
//			{
////				double d=fabs((*idx_min)->u.norm()/(*it_line)->u.norm());
//	//			std::cout<<"min="<<min<<" d="<<d<<std::endl;
//				if(min<thres_line2shadow)// && d>1.0)
//				{
//					(*it_line)->occluded=*idx_min;
//					if(debug) fp<<"\toccluded\t"<<(*it_line)->occluded->u.transpose()<<std::endl;
//	//				std::cout<<"acos(u^T*u_pi) = "<<min<<std::endl;
//	//				displayLineShadow(scan,*idx_min,*it_line,vis);
//	//				vis->spin();
//				}
//			}
		}

		fp.close();

		if(debug)
		{
			visLines(scan,vis);
			vis->spin();
		}
	}

	void LineExtraction::generatePlucker(Line *line)
	{
		// refine the extracted line using the inlier points;
		const int N=line->points.size();
		Eigen::Matrix3d pG_skew=Eigen::Matrix3d::Zero();
		for(std::list<EdgePoint*>::iterator it=line->points.begin();it!=line->points.end();it++)
		{
			pG_skew+=Transform::skew_sym((*it)->xyz);
		}
		pG_skew/=N;
		Eigen::Vector3d pG=Transform::skew_sym_inv(pG_skew);
		Eigen::Matrix3d P=Eigen::Matrix3d::Zero();
		for(std::list<EdgePoint*>::iterator it=line->points.begin();it!=line->points.end();it++)
		{
			Eigen::Vector3d tmp=(*it)->xyz-pG;
			P+=-Transform::skew_sym(tmp)*Transform::skew_sym(tmp);
		}

		Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es;
		es.compute(P);
		Eigen::Vector3d eigenvalues=es.eigenvalues();
		Eigen::Matrix3d eigenvectors=es.eigenvectors();

		double ev_min=DBL_MAX;
		int idx=-1;
		for(int j=0;j<3;j++)
		{
			if(eigenvalues(j)<ev_min)
			{
				ev_min=eigenvalues(j);
				idx=j;
			}
		}

		line->v=eigenvectors.block<3,1>(0,idx);
		line->u=pG_skew*line->v;

		pG.setZero();
		for(std::list<EdgePoint*>::iterator it=line->points.begin();it!=line->points.end();it++)
		{
			pG+=(*it)->xyz;
		}
		pG/=N;
		line->pG=pG;

//		if(debug) fp<<"\t"<<line->u.transpose()<<"\t"<<line->v.transpose()<<std::endl;
	}

	/*
	void LineExtraction::generatePlucker1(Line *line)
	{
		if(debug) fp<<"\n------generatePlucker"<<std::endl;
		// refine the extracted line using the inlier points;
		// compute line-> m,n,x0,y0;
		int N=line->points.size();
		Eigen::Matrix2d A,B;
		A.setZero();
		B.setZero();
		B(1,1)=N;
		for(std::list<EdgePoint*>::iterator it=line->points.begin();it!=line->points.end();it++)
		{
			A(0,0)+=(*it)->xyz(0)*(*it)->xyz(2);
			A(0,1)+=(*it)->xyz(0);
			A(1,0)+=(*it)->xyz(1)*(*it)->xyz(2);
			A(1,1)+=(*it)->xyz(1);
			B(0,0)+=(*it)->xyz(2)*(*it)->xyz(2);
			B(0,1)+=(*it)->xyz(2);
			B(1,0)+=(*it)->xyz(2);
		}
		Eigen::Matrix2d param=A*B.inverse();
		line->m=param(0,0);
		line->n=param(1,0);
		line->x0=param(0,1);
		line->y0=param(1,1);
		if(debug) fp<<"\tm="<<line->m<<"\tn="<<line->n<<"\tx0="<<line->x0<<"\ty0="<<line->y0<<std::endl;

		// refine the endpoints of the line;
		if(debug)
		{
			fp<<"\tend_point_1\t"<<line->end_point_1.transpose()<<std::endl;
			fp<<"\tend_point_2\t"<<line->end_point_2.transpose()<<std::endl;
		}
		double z=line->end_point_1(2);
		line->end_point_1(0)=line->m*z+line->x0;
		line->end_point_1(1)=line->n*z+line->y0;
		z=line->end_point_2(2);
		line->end_point_2(0)=line->m*z+line->x0;
		line->end_point_2(1)=line->n*z+line->y0;
		if(debug)
		{
			fp<<"\tend_point_1\t"<<line->end_point_1.transpose()<<std::endl;
			fp<<"\tend_point_2\t"<<line->end_point_2.transpose()<<std::endl;
		}

		// generate Plucker coordinates for the line;
		line->u=line->end_point_1.cross(line->end_point_2);
		line->v=line->end_point_2-line->end_point_1;
		if(debug) fp<<"\toriginal\t"<<line->u.transpose()<<"\t"<<line->v.transpose()<<std::endl;
		double d=line->u.norm()/line->v.norm();
		line->u.normalize();
		line->u=d*line->u;
		line->v.normalize();
		if(debug) fp<<"\tnormalized\t"<<line->u.transpose()<<"\t"<<line->v.transpose()<<std::endl;
	}*/

	void LineExtraction::compute_line_cov_inv(Line *line)
	{
		line->cov_inv.setZero();
		Matrix6d tmp;
		for(std::list<EdgePoint*>::iterator it=line->points.begin();it!=line->points.end();it++)
		{
			tmp.block<3,3>(0,0)=Eigen::Matrix3d::Identity();
			tmp.block<3,3>(0,3)=-Transform::skew_sym((*it)->xyz);
			tmp.block<3,3>(3,0)=Transform::skew_sym((*it)->xyz);
			tmp.block<3,3>(3,3)=-Transform::skew_sym((*it)->xyz)*Transform::skew_sym((*it)->xyz);
			line->cov_inv=line->cov_inv+tmp;
		}
//		line->cov_inv/=line->points.size();

		Eigen::SelfAdjointEigenSolver<Matrix6d> es;
		es.compute(line->cov_inv);
		Vector6d Lambda=es.eigenvalues();
		Matrix6d U=es.eigenvectors();
		Matrix6d sqrt_Lambda=Matrix6d::Zero();
		Matrix6d inv_Lambda=Matrix6d::Zero();
		for(size_t i=0;i<6;i++)
		{
			sqrt_Lambda(i,i)=sqrt(Lambda(i));
			if(Lambda(i)>0.01)
			{
				inv_Lambda(i,i)=1.0/Lambda(i);
			}
		}
		line->sqrt_cov_inv=U*sqrt_Lambda*U.transpose();
		line->cov=U*inv_Lambda*U.transpose();

	}

	void LineExtraction::extractLinesHough(std::vector<EdgePoint*>& edge_points, std::list<Line*>& lines_occluding, double scale)
	{
		if(debug) fp<<"extractLinesHough-----------------------------------"<<std::endl;
		cv::Mat img_occluding=cv::Mat::zeros(height,width,CV_8UC3);
		for(size_t i=0;i<edge_points.size();i++)
		{
			int x=edge_points[i]->pixel(1);//height
			int y=edge_points[i]->pixel(0);//width
			img_occluding.at<cv::Vec3b>(x,y)[0]=255;//blue
			img_occluding.at<cv::Vec3b>(x,y)[1]=0;
			img_occluding.at<cv::Vec3b>(x,y)[2]=0;
		}
//		cv::imshow("contours",img_occluding);
//		cv::waitKey(0);

		cv::Mat contours=cv::Mat::zeros(height,width,CV_8UC1);
		for(size_t i=0;i<edge_points.size();i++)
		{
			int x=edge_points[i]->pixel(1);
			int y=edge_points[i]->pixel(0);
			contours.at<unsigned char>(x,y)=255;
		}
//		if(debug)
//		{
//			cv::imshow("image",contours);
//			cv::waitKey(0);
//		}

		// extract 2D lines in image using Hough transform (OpenCV);
		// void HoughLinesP(InputArray image, OutputArray lines, double rho, double theta, int threshold, double minLineLength=0, double maxLineGap=0 )
		std::vector<cv::Vec4i> lines;
		cv::HoughLinesP(contours, lines, rho*scale, theta*scale, threshold, minLineLength, maxLineGap);
		std::vector<Line*> lines_tmp;
		lines_tmp.resize(lines.size());
//		for(std::vector<cv::Vec4i>::iterator it=lines.begin();it!=lines.end();it++)
		for(size_t i=0;i<lines.size();i++)
		{
			cv::Point pt1(lines[i][0],lines[i][1]);
			cv::Point pt2(lines[i][2],lines[i][3]);
			cv::line(img_occluding, pt1, pt2, CV_RGB(0,255,0));
			double x1=lines[i][0];
			double y1=lines[i][1];
			double x2=lines[i][2];
			double y2=lines[i][3];
			double tmp=sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
			double a=(y1-y2)/tmp;
			double b=(x2-x1)/tmp;
			double d=(x1*y2-x2*y1)/tmp;
			Line *line=new Line;
			for(size_t j=0;j<edge_points.size();j++)
			{
				int x=edge_points[j]->pixel(0);
				int y=edge_points[j]->pixel(1);
				if(fabs(a*x+y*b+d)<5.0 && edge_points[j]->onLine==false)
				{
					edge_points[j]->onLine=true;
					line->points_tmp.push_back(edge_points[j]);
					img_occluding.at<cv::Vec3b>(int(y),int(x))[0]=0;
					img_occluding.at<cv::Vec3b>(int(y),int(x))[1]=0;
					img_occluding.at<cv::Vec3b>(int(y),int(x))[2]=255;//red
				}
			}
			// lines_occluding.push_back(line);
			lines_tmp[i]=line;
		}
//		if(debug)
//		{
//			cv::imshow("image",img_occluding);
//			cv::waitKey(0);
//		}

		// 3D RANSAC (PCL);
		// merge similar lines;
		// same them in ordered list;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
		Eigen::VectorXf line_coeff;
		std::vector<int> line_inliers;
		Eigen::Matrix<double,1,1> tmp;
		bool merge=false;
		for(size_t i=0;i<lines_tmp.size();i++)
		{
//			if(debug) fp<<i<<"-th line in lines_tmp "<<lines_tmp[i]->points_tmp.size()<<std::endl;
			if(lines_tmp[i]->points_tmp.size()<=10)
			{
				delete lines_tmp[i];
				continue;
			}
			merge=false;
			cloud->points.resize(lines_tmp[i]->points_tmp.size());
			for(size_t j=0;j<lines_tmp[i]->points_tmp.size();j++)
			{
				cloud->points[j].x=lines_tmp[i]->points_tmp[j]->xyz(0);
				cloud->points[j].y=lines_tmp[i]->points_tmp[j]->xyz(1);
				cloud->points[j].z=lines_tmp[i]->points_tmp[j]->xyz(2);
//				fp<<lines_tmp[i]->points[j]->xyz.transpose()<<std::endl;
			}
			line_inliers.clear();
			pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model_line
					(new pcl::SampleConsensusModelLine<pcl::PointXYZ> (cloud));
			pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_line);
			ransac.setDistanceThreshold (.01);
//			model_line->setInputCloud(cloud);
//			ransac.setSampleConsensusModel(model_line);
			ransac.computeModel();
			ransac.getInliers(line_inliers);
//			if(debug) fp<<"inliers "<<line_inliers.size()<<std::endl;
			ransac.getModelCoefficients(line_coeff);
//			if(debug) fp<<"coefficients "<<line_coeff.transpose()<<std::endl;
			lines_tmp[i]->end_point_1(0)=line_coeff(0);
			lines_tmp[i]->end_point_1(1)=line_coeff(1);
			lines_tmp[i]->end_point_1(2)=line_coeff(2);
			lines_tmp[i]->end_point_2(0)=line_coeff(0);
			lines_tmp[i]->end_point_2(1)=line_coeff(1);
			lines_tmp[i]->end_point_2(2)=line_coeff(2);
			lines_tmp[i]->dir(0)=line_coeff(3);
			lines_tmp[i]->dir(1)=line_coeff(4);
			lines_tmp[i]->dir(2)=line_coeff(5);
			lines_tmp[i]->dir.normalize();
//			lines_tmp[i]->u=lines_tmp[i]->end_point_1.cross(lines_tmp[i]->end_point_2);
//			lines_tmp[i]->v=lines_tmp[i]->end_point_2-lines_tmp[i]->end_point_1;
//			lines_tmp[i]->u/=lines_tmp[i]->v.norm();
//			lines_tmp[i]->v.normalize();
			if(debug)
			{
				fp<<"========================================="<<std::endl;
				fp<<i<<"-th line "<<line_coeff.transpose()<<std::endl;
			}
			for(std::list<Line*>::iterator it_line=lines_occluding.begin();it_line!=lines_occluding.end();it_line++)
			{
				double sim_dir=lines_tmp[i]->dir.transpose()*(*it_line)->dir;
				sim_dir=fabs(sim_dir);
				sim_dir=acos(sim_dir)*180/M_PI;

				Eigen::Vector3d vec_sim_dist=lines_tmp[i]->dir.cross((*it_line)->dir);
				vec_sim_dist.normalize();
				Eigen::Matrix<double,1,1> tmp=vec_sim_dist.transpose()*(lines_tmp[i]->end_point_1-(*it_line)->end_point_1);
				double sim_dist=fabs(tmp(0,0));

//				fp<<sim_dir<<"\t"<<sim_dist<<std::endl;

				if(sim_dir<thres_sim_dir*scale && sim_dist<thres_sim_dist*scale)
				{
//					if(debug)
//					{
//						fp<<"similar with a line in lines_occluding"<<std::endl;
//						fp<<"sim_dir ="<<sim_dir<<std::endl;
//						fp<<"sim_dist="<<sim_dist<<std::endl;
//					}
					// merge [i] and [j];
					for(size_t k=0;k<line_inliers.size();k++)
					{
						tmp=(*it_line)->dir.transpose()
							*(lines_tmp[i]->points_tmp[line_inliers[k]]->xyz-(*it_line)->end_point_2);
						if(tmp(0,0)>0)
						{
							(*it_line)->points.push_back(lines_tmp[i]->points_tmp[line_inliers[k]]);
						}
						else
						for(std::list<EdgePoint*>::iterator it=(*it_line)->points.begin();
															it!=(*it_line)->points.end();it++)
						{
							tmp=(*it_line)->dir.transpose()
								*(lines_tmp[i]->points_tmp[line_inliers[k]]->xyz-(*it)->xyz);
							if(tmp(0,0)<0)
							{
								(*it_line)->points.insert(it,lines_tmp[i]->points_tmp[line_inliers[k]]);
								break;
							}
						}
						std::list<EdgePoint*>::iterator it=(*it_line)->points.begin();
						(*it_line)->end_point_1=(*it)->xyz;
//						(*it_line)->end_point_1_img=(*it)->pixel;
						it=(*it_line)->points.end(); it--;
						(*it_line)->end_point_2=(*it)->xyz;
//						(*it_line)->end_point_2_img=(*it)->pixel;
					}
					Eigen::Vector3d vec3d=(*it_line)->end_point_1-(*it_line)->end_point_2;
					(*it_line)->length=vec3d.norm();
					// update lines_occluding[j]->dir here;
					delete lines_tmp[i];
					merge=true;
					break;
				}
			}
			if(merge) continue;
			// else add a new line in lines_occluding
//			if(debug) fp<<"add a new line in lines_occluding"<<std::endl;
			Line *line=new Line;
//			line->points.resize(line_inliers.size());
//			line->end_point_1=lines_tmp[i]->end_point_1;
//			line->end_point_2=lines_tmp[i]->end_point_2;
			line->dir=lines_tmp[i]->dir;
//			line->u=lines_tmp[i]->u;
//			line->v=lines_tmp[i]->v;
//			fp<<"inlier size "<<line_inliers.size()<<std::endl; // more than 10;
			for(size_t k=0;k<line_inliers.size();k++)
			{
				if(line->points.size()==0)
				{
					line->points.push_back(lines_tmp[i]->points_tmp[line_inliers[k]]);
					line->end_point_1=lines_tmp[i]->points_tmp[line_inliers[k]]->xyz;
					line->end_point_2=lines_tmp[i]->points_tmp[line_inliers[k]]->xyz;
//					fp<<"first pushed point "<<lines_tmp[i]->points_tmp[line_inliers[k]]->xyz.transpose()<<std::endl;
				}
				else
				{
					tmp=line->dir.transpose()*(lines_tmp[i]->points_tmp[line_inliers[k]]->xyz-line->end_point_2);
					if(tmp(0,0)>0)
					{
						line->points.push_back(lines_tmp[i]->points_tmp[line_inliers[k]]);
//						fp<<"at the end "<<lines_tmp[i]->points_tmp[line_inliers[k]]->xyz.transpose()<<"\t"<<tmp<<std::endl;
					}
					else
					for(std::list<EdgePoint*>::iterator it=line->points.begin();it!=line->points.end();it++)
					{
						tmp=line->dir.transpose()*(lines_tmp[i]->points_tmp[line_inliers[k]]->xyz-(*it)->xyz);
						if(tmp(0,0)<0)
						{
							line->points.insert(it,lines_tmp[i]->points_tmp[line_inliers[k]]);
//							fp<<"in the middle "<<lines_tmp[i]->points_tmp[line_inliers[k]]->xyz.transpose()<<"\t"<<(*it)->xyz.transpose()<<std::endl;
							break;
						}
					}
					std::list<EdgePoint*>::iterator it=line->points.begin();
					line->end_point_1=(*it)->xyz;
//					line->end_point_1_img=(*it)->pixel;
					it=line->points.end(); it--;
					line->end_point_2=(*it)->xyz;
//					line->end_point_2_img=(*it)->pixel;
//					Eigen::Vector3d pt=lines_tmp[i]->points_tmp[line_inliers[k]]->xyz;
//					Eigen::Vector3d ep1=line->end_point_1;
//					Eigen::Vector3d ep2=line->end_point_2;
//					Eigen::Vector3d dir=line->dir;
//					tmp=dir.transpose()*(pt-ep1);
//					if(tmp(0,0)<0) line->end_point_1=pt;
//					tmp=dir.transpose()*(pt-ep2);
//					if(tmp(0,0)>0) line->end_point_2=pt;
				}
			}
			Eigen::Vector3d vec3d=line->end_point_1-line->end_point_2;
			line->length=vec3d.norm();
			lines_occluding.push_back(line);
			delete lines_tmp[i];

//			if(debug)
//			{
//				std::list<Line*>::iterator itt_line=std::prev(lines_occluding.end());
////				size_t ii=lines_occluding.size()-1;
//				fp<<(*itt_line)->end_point_1.transpose()<<std::endl;
//				for(std::list<EdgePoint*>::iterator it=(*itt_line)->points.begin();it!=(*itt_line)->points.end();it++)
//				{
//					fp<<"\t"<<(*it)->xyz.transpose()<<"\t"<<(*itt_line)->dir.transpose()*((*it)->xyz-(*itt_line)->end_point_1)<<std::endl;
//				}
//				fp<<(*itt_line)->end_point_2.transpose()<<std::endl;
//			}
		}

//		for(size_t i=0;i<lines_occluding.size();i++)
//		if(debug) fp<<std::endl<<"splitting lines "<<std::endl;
		for(std::list<Line*>::iterator it_line=lines_occluding.begin();
									   it_line!=lines_occluding.end();it_line++)
		{
//			if(debug) fp<<"\t"<<lines_occluding.size()<<std::endl;
			for(std::list<EdgePoint*>::iterator it=(*it_line)->points.begin();it!=(*it_line)->points.end();it++)
			{
				if(it==(*it_line)->points.begin()) continue;
				std::list<EdgePoint*>::iterator it_pre=it;
				it_pre--;
				Eigen::Vector3d vec3d=(*it)->xyz-(*it_pre)->xyz;
				if(vec3d.norm()>thres_split*scale)
				{
//					(*it_line)->end_point_2=(*it_pre)->xyz;
					Line *line=new Line;
					line->points.splice(line->points.begin(),(*it_line)->points,it,(*it_line)->points.end());
//					std::list<EdgePoint*>::iterator itt=line->points.begin();
//					line->end_point_1=(*itt)->xyz;
//					line->end_point_2=(*std::prev(line->points.end()))->xyz;
//					vec3d=line->end_point_1-line->end_point_2;
//					line->length=vec3d.norm();
//					line->dir=(*it_line)->dir;
					lines_occluding.push_back(line);
					break;
				}
			}
		}

//		if(debug) fp<<std::endl<<"cunning the lines "<<std::endl;
		for(std::list<Line*>::iterator it_line=lines_occluding.begin();
									   it_line!=lines_occluding.end();)
		{
//			if(debug) fp<<"\t"<<(*it_line)->points.size()<<std::endl;
			// cunning the lines that does not contain enough points;
			if((*it_line)->points.size()<double(min_points_on_line)/scale)
			{
				delete *it_line;
				it_line=lines_occluding.erase(it_line);
			}
			else
			{
				std::list<EdgePoint*>::iterator itt=(*it_line)->points.begin();
				(*it_line)->end_point_1=(*itt)->xyz;
				(*it_line)->end_point_2=(*std::prev((*it_line)->points.end()))->xyz;
				Eigen::Vector3d vec3d=(*it_line)->end_point_1-(*it_line)->end_point_2;
				(*it_line)->length=vec3d.norm();
				(*it_line)->dir=(*it_line)->end_point_2-(*it_line)->end_point_1;
				(*it_line)->dir.normalize();
				it_line++;
			}
		//	compute the dir
		}
//		fp.close();


//		cv::Mat img_occluded=cv::Mat::zeros(480,640,CV_8UC3);
//		for (size_t i=0;i<labels_edge->height;i++)
//		{
//			for (size_t j=0;j<labels_edge->width;j++)
//			{
//				if(labels_edge->at(j,i).label==4)//occluded
//				{
//					img_occluded.at<cv::Vec3b>(i,j)[0]=255;
//					img_occluded.at<cv::Vec3b>(i,j)[1]=0;
//					img_occluded.at<cv::Vec3b>(i,j)[2]=0;//red
//				}
//			}
//		}
//		contours=cv::Mat::zeros(480,640,CV_8UC1);
//		for (size_t i=0;i<labels_edge->height;i++)
//		{
//			for (size_t j=0;j<labels_edge->width;j++)
//			{
//				if(labels_edge->at(j,i).label==4)
//				{
//					contours.at<unsigned char>(i,j)=255;
//				}
//			}
//		}
//		//void HoughLinesP(InputArray image, OutputArray lines, double rho, double theta, int threshold, double minLineLength=0, double maxLineGap=0 )
//		lines.clear();
//		cv::HoughLinesP(contours, lines, rho, theta, threshold, minLineLength, maxLineGap);

	}



//	void LineExtraction::fitLines(Scan *scan)
//	{
//		fp.open("extract_EdgePoints.txt",std::ios::app);
//		int bins_theta=10,bins_phy=20;
////		std::vector<Eigen::Vector2d> dirs_sphere;
////		dirs_sphere.resize(scan->edge_points.size());
//		Eigen::Vector2d dir;
//		std::vector<std::vector<EdgePoint*> > cells;
//		cells.resize(bins_theta*bins_phy);
//		std::vector<std::vector<EdgePoint*> > lines;
//		for(size_t i=0;i<scan->edge_points.size();i++)
//		{
//			if(scan->edge_points[i]->isEdge==false)
//				continue;
//			dir(0)=acos(scan->edge_points[i]->dir(2));
//			dir(1)=atan2(scan->edge_points[i]->dir(1),scan->edge_points[i]->dir(0));
//			int row=dir(0)/(M_PI/bins_theta);
//			int col=(dir(1)+M_PI)/(M_PI*2.0/bins_phy);
//			int index=bins_phy*row+col;
//			cells[index].push_back(scan->edge_points[i]);
//		}
////		for(size_t i=0;i<cells.size();i++)
////		{
////			int row=i/bins_phy;
////			int col=i%bins_phy;
////			if(col==0) fp<<std::endl;
////			fp<<cells[i].size()<<"\t";
////		}
//		std::vector<Sorted_Cell> sorted_cells;
//		sorted_cells.resize(cells.size());
//		for(size_t i=0;i<cells.size();i++)
//		{
//			sorted_cells[i].index=i;
//			sorted_cells[i].num_point=cells[i].size();
//		}
//		std::sort(sorted_cells.begin(),sorted_cells.end());
//		std::vector<Sorted_Cell>::iterator iter_sorted_cells=sorted_cells.end()-1;
//		unsigned char red [14] = {255,   0,   0, 255, 255,   0, 130,   0,   0, 130, 130,   0, 130, 255};
//		unsigned char grn [14] = {  0, 255,   0, 255,   0, 255,   0, 130,   0, 130,   0, 130, 130, 255};
//		unsigned char blu [14] = {  0,   0, 255,   0, 255, 255,   0,   0, 130,   0, 130, 130, 130, 255};
//		int count=0;
//		timeval time_seed;
//
//		while(true)
//		{
////			fp<<"cell "<<count<<" - "<<iter_sorted_cells->num_point<<std::endl;
//			int idx=iter_sorted_cells->index;
////			Eigen::Vector3d n0;
////			n0.setZero();
////			for(size_t i=0;i<cells[idx].size();i++)
////			{
//////				cells[idx][i]->rgb(0)=red[count];
//////				cells[idx][i]->rgb(1)=grn[count];
//////				cells[idx][i]->rgb(2)=blu[count];
////				n0+=cells[idx][i]->xyz;
//////				fp<<"\t"<<cells[idx][i]->xyz.transpose()<<"\t"<<cells[idx][i]->rgb.transpose()<<std::endl;
////			}
////			n0/=iter_sorted_cells->num_point;
//			int iter=0;
//			while(true)
//			{
////				srand((int)time(0));
//				gettimeofday(&time_seed,NULL);
//				srand((int)time_seed.tv_usec);
//				int index=(int)(rand()%iter_sorted_cells->num_point);
//				Eigen::Vector3d p0=cells[idx][index]->xyz;
//				Eigen::Vector3d n0=cells[idx][index]->dir;
//				std::vector<EdgePoint*> tmp_line;
//				for(size_t i=0;i<scan->edge_points.size();i++)
//				{
//					if(scan->edge_points[i]->isEdge==false)
//						continue;
//					if(scan->edge_points[i]->rgb.norm()>0)
//						continue;
//					Eigen::Vector3d p=scan->edge_points[i]->xyz;
//					Eigen::Matrix<double,1,1> mu_tmp=n0.transpose()*(p-p0);
//					double mu=mu_tmp(0,0)/(n0.norm()*n0.norm());
//					p=p-mu*n0-p0;
//					mu=p.norm();
//					double ang=n0.transpose()*scan->edge_points[i]->dir;
//					if(mu<0.03 && ang>0.9)
//						tmp_line.push_back(scan->edge_points[i]);
//				}
//				int thres=(int(iter_sorted_cells->num_point/500.0)+1)*100;
////				fp<<"\t"<<time_seed.tv_usec<<"\t"<<index<<"\t"<<thres<<"\t"<<tmp_line.size()<<std::endl;
//				if(tmp_line.size()>=thres)
//				{
//					for(size_t i=0;i<tmp_line.size();i++)
//					{
//						tmp_line[i]->rgb(0)=red[count];
//						tmp_line[i]->rgb(1)=grn[count];
//						tmp_line[i]->rgb(2)=blu[count];
////						fp<<"\t\t"<<tmp_line[i]->xyz.transpose()<<std::endl;
//					}
//					count++;
//					lines.push_back(tmp_line);
//					break;
//				}
//				if(iter>20)
//					break;
//				iter++;
//			}
//			if(iter_sorted_cells==sorted_cells.begin()) break;
//			iter_sorted_cells--;
//			if(iter_sorted_cells->num_point<50) break;
//		}
//		fp.close();
//	}
//
//	bool LineExtraction::fitSphere(EdgePoint *edge_point)
//	{
//		fp.open("extract_EdgePoints.txt",std::ios::app);
//		Eigen::Vector3d center;
//		double radius;
//
//		double x_bar=0,y_bar=0,z_bar=0;
//		double xy_bar=0,xz_bar=0,yz_bar=0;
//		double x2_bar=0,y2_bar=0,z2_bar=0;
//		double x2y_bar=0,x2z_bar=0,xy2_bar=0,y2z_bar=0,xz2_bar=0,yz2_bar=0;
//		double x3_bar=0,y3_bar=0,z3_bar=0;
//		for(size_t i=0;i<edge_point->neighbors.size();i++)
//		{
//			double x=edge_point->neighbors[i]->xyz(0);
//			double y=edge_point->neighbors[i]->xyz(1);
//			double z=edge_point->neighbors[i]->xyz(2);
//			x_bar  +=x;
//			y_bar  +=y;
//			z_bar  +=z;
//			xy_bar +=x*y;
//			xz_bar +=x*z;
//			yz_bar +=y*z;
//			x2_bar +=x*x;
//			y2_bar +=y*y;
//			z2_bar +=z*z;
//			x2y_bar+=x*x*y;
//			x2z_bar+=x*x*z;
//			xy2_bar+=x*y*y;
//			y2z_bar+=y*y*z;
//			xz2_bar+=x*z*z;
//			yz2_bar+=y*y*z;
//			x3_bar +=x*x*x;
//			y3_bar +=y*y*y;
//			z3_bar +=z*z*z;
//		}
//		x_bar  /=edge_point->neighbors.size();
//		y_bar  /=edge_point->neighbors.size();
//		z_bar  /=edge_point->neighbors.size();
//		xy_bar /=edge_point->neighbors.size();
//		xz_bar /=edge_point->neighbors.size();
//		yz_bar /=edge_point->neighbors.size();
//		x2_bar /=edge_point->neighbors.size();
//		y2_bar /=edge_point->neighbors.size();
//		z2_bar /=edge_point->neighbors.size();
//		x2y_bar/=edge_point->neighbors.size();
//		x2z_bar/=edge_point->neighbors.size();
//		xy2_bar/=edge_point->neighbors.size();
//		y2z_bar/=edge_point->neighbors.size();
//		xz2_bar/=edge_point->neighbors.size();
//		yz2_bar/=edge_point->neighbors.size();
//		x3_bar /=edge_point->neighbors.size();
//		y3_bar /=edge_point->neighbors.size();
//		z3_bar /=edge_point->neighbors.size();
//		
//		Eigen::Matrix3d A;
//		A(0,0)=x2_bar-x_bar*x_bar;
//		A(0,1)=xy_bar-x_bar*y_bar;
//		A(0,2)=xz_bar-x_bar*z_bar;
//		A(1,0)=xy_bar-x_bar*y_bar;
//		A(1,1)=y2_bar-y_bar*y_bar;
//		A(1,2)=yz_bar-y_bar*z_bar;
//		A(2,0)=xz_bar-x_bar*z_bar;
//		A(2,1)=yz_bar-y_bar*z_bar;
//		A(2,2)=z2_bar-z_bar*z_bar;
//
//		Eigen::Vector3d b;
//		b(0)=(x3_bar -x_bar*x2_bar)+(xy2_bar-x_bar*y2_bar)+(xz2_bar-x_bar*z2_bar);
//		b(1)=(x2y_bar-y_bar*x2_bar)+(y3_bar -y_bar*y2_bar)+(yz2_bar-y_bar*z2_bar);
//		b(2)=(x2z_bar-z_bar*x2_bar)+(y2z_bar-z_bar*y2_bar)+(z3_bar -z_bar*z2_bar);
//		b*=0.5;
//
//		Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(A);
//		Eigen::Vector3d Lambda=es.eigenvalues();
//		Eigen::Matrix3d U=es.eigenvectors();
//		
////		fp<<std::endl<<edge_point->neighbors.size()<<"\t"<<Lambda.transpose()<<std::endl;
////		fp<<A<<std::endl;
//
//		Eigen::Matrix3d A_inv;
//		bool invertible;
//		A.computeInverseWithCheck(A_inv,invertible);
//		if(invertible)
//		{
//			center=A_inv*b;
//			double x0=center(0);
//			double y0=center(1);
//			double z0=center(2);
//			radius=sqrt(x2_bar-2*x0*x_bar+x0*x0
//					   +y2_bar-2*y0*y_bar+y0*y0
//					   +z2_bar-2*z0*z_bar+z0*z0);
//			edge_point->meas_Edge=1.0/radius;
//			fp.close();
//			return true;
//		}
//		else
//		{
//			edge_point->meas_Edge=-1;
//			fp.close();
//			return false;
//		}
//
////		edge_point->meas_Edge
//	}

	void visLines(Scan *scan, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
		char id[20];

		vis->removeAllPointClouds();
		vis->removeAllShapes();
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr edge (new pcl::PointCloud<pcl::PointXYZRGBA>);

		// add raw scan data;
		sprintf(id,"scan");
		if (!vis->updatePointCloud (scan->point_cloud, id))
			vis->addPointCloud (scan->point_cloud, id);
		
		// show scan->lines_occluding;
		std::cout<<"\nvisualize occluding lines - red"<<std::endl;
		size_t i=0;
		for(std::list<Line*>::iterator it_line=scan->lines_occluding.begin();it_line!=scan->lines_occluding.end();it_line++)
		{
			edge->resize((*it_line)->points.size()*2);
			size_t j=0;
			for(std::list<EdgePoint*>::iterator it=(*it_line)->points.begin();it!=(*it_line)->points.end();it++)
			{
				edge->at(j).x=(*it)->xyz(0);
				edge->at(j).y=(*it)->xyz(1);
				edge->at(j).z=(*it)->xyz(2);
				edge->at(j).r=255; // red
				edge->at(j).g=0;
				edge->at(j).b=0;
				j++;
			}

			sprintf(id,"line_occluding%d",i);
			if (!vis->updatePointCloud (edge, id))
				vis->addPointCloud (edge, id);
			vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, id);
			i++;
		}

		// show scan->lines_occluded;
		std::cout<<"visualize shadow lines - green"<<std::endl;
		i=0;
		for(std::list<Line*>::iterator it_line=scan->lines_occluded.begin();it_line!=scan->lines_occluded.end();it_line++)
		{
			edge->resize((*it_line)->points.size());
			size_t j=0;
			for(std::list<EdgePoint*>::iterator it=(*it_line)->points.begin();it!=(*it_line)->points.end();it++)
			{
				edge->at(j).x=(*it)->xyz(0);
				edge->at(j).y=(*it)->xyz(1);
				edge->at(j).z=(*it)->xyz(2);
				edge->at(j).r=0;
				edge->at(j).g=255; // green 
				edge->at(j).b=0;
				j++;
			}

			sprintf(id,"line_occluded%d",i);
			if (!vis->updatePointCloud (edge, id))
				vis->addPointCloud (edge, id);
			vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, id);
			i++;
		}

	}


}
