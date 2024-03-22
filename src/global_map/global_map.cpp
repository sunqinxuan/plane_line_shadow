/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@gmail.com
#
# Last modified:	2019-04-24 15:27
#
# Filename:		global_map.cpp
#
# Description: 
#
************************************************/

#include "global_map.h"

namespace ulysses
{
	void GlobalMap::addScan(Scan *scan, Map* map)
	{
		fp.open("global_map.txt",std::ios::app);
		if(debug) fp<<"\nGlobalMap::addScan"<<std::endl;
//		if(map->cameras.size()==0)
//		{
//			map->addCamera(Transform());
//			for(size_t j=0;j<scan->plane_matches.size();j++)
//			{
//				PlaneLM *plane_lm=new PlaneLM(scan->plane_matches[j].ref,
//											  scan->scan_ref->Tcg.inv());
//				plane_lm->id=map->planes.size();
//				plane_lm->indices_cameras.push_back(0);
//				map->planes.push_back(plane_lm);
//				map->addObservationPlane(scan->plane_matches[j].ref,0,plane_lm->id);
//				scan->plane_matches[j].ref->idx_plane=plane_lm->id;
//				if(debug) fp<<"\nadd a new plane landmark (ref)\n"
//							<<"\tidx_camera\t"<<map->cameras.size()-1
//							<<"\tidx_plane\t" <<map->planes.size()-1
//				            <<"\n\tplane_lm\t"<<plane_lm->pi(0)<<"\t"<<plane_lm->pi(1)<<"\t"<<plane_lm->pi(2)<<"\t"<<plane_lm->pi(3)<<std::endl;
//			}
//			for(size_t j=0;j<scan->line_matches.size();j++)
//			{
//				LineLM *line_lm=new LineLM(scan->line_matches[j].ref,
//										   scan->scan_ref->Tcg.inv());
//				line_lm->id=map->lines.size();
//				line_lm->indices_cameras.push_back(0);
//				map->lines.push_back(line_lm);
//				map->addObservationLine(scan->line_matches[j].ref,0,line_lm->id);
//				scan->line_matches[j].ref->idx_line=line_lm->id;
//				if(debug) fp<<"\nadd a new line landmark (ref)\n"
//							<<"\tidx_camera\t"<<map->cameras.size()-1
//							<<"\tidx_line\t" <<map->lines.size()-1
//				            <<"\n\tline_lm\t"<<line_lm->L(0)<<"\t"<<line_lm->L(1)<<"\t"<<line_lm->L(2)<<"\t"
//											 <<line_lm->L(3)<<"\t"<<line_lm->L(4)<<"\t"<<line_lm->L(5)<<std::endl;
//			}
//			// add observation shadows;
//			int id=0;
//			for(std::list<Line*>::iterator it_line=scan->scan_ref->lines_occluded.begin();
//										   it_line!=scan->scan_ref->lines_occluded.end();it_line++)
//			{
//				Eigen::Vector3d u=(*it_line)->u;
//				Eigen::Vector3d v=(*it_line)->v;
//				double min=DBL_MAX;
//				int idx=-1;
//				for(int i=0;i<scan->plane_matches.size();i++)
//				{
//					Eigen::Vector3d n=scan->plane_matches[i].ref->normal;
//					double d=scan->plane_matches[i].ref->d;
//					Eigen::Vector3d tmp=u.cross(n)+d*v;
//					double dist=tmp.norm();
//					if(dist<min)
//					{
//						min=dist;
//						idx=i;
//					}
//					std::cout<<"|uxn+dv| = "<<dist<<std::endl;
//					displayLineShadowPlane(scan->scan_ref,scan->plane_matches[i].ref,*it_line,vis);
//					vis->spin();
//				}
//				if(min<0.1)
//				{
//					(*it_line)->plane=scan->plane_matches[i].ref;
//					min=DBL_MAX;
//					idx=-1;
//					for(int k=0;k<scan->line_matches.size();k++)
//					{
//						Eigen::Vector3d u_pi=scan->line_matches[k].ref->u;
//						u.normalize();
//						u_pi.normalize();
//						double dist=u.transpose()*u_pi;
//						dist=fabs(dist);
//						dist=acos(dist)*180.0/M_PI;
//						if(dist<min)
//						{
//							min=dist;
//							idx=k;
//						}
//						std::cout<<"acos(u^T*u_pi) = "<<dist<<std::endl;
//						displayLineShadow(scan->scan_ref,scan->line_matches[k].ref,*it_line,vis);
//						vis->spin();
//					}
//					if(min<1.0)
//					{
//						scan->line_matches[idx].ref->occluded=(*it_line);
//						addObservationShadow(*it_line,0,(*it_line)->plane->idx_plane,scan->line_matches[idx].ref->idx_line);
//					}
//				}
//			}
//		}
		// for matches[j].cur;
		if(map->cameras.size()==0) map->addCamera(Transform(),scan->scan_ref->time_stamp);
		map->addCamera(map->cameras[map->cameras.size()-1],scan->time_stamp);
		for(size_t j=0;j<scan->plane_matches.size();j++)
		{ // for each plane observation;
//			if(scan->plane_matches[j].ref->idx_plane>=0)
//			{
//				if(debug) fp<<"\nplane observed in map\n"
//							<<"\tidx_camera "<<map->cameras.size()-1
//							<<"\tidx_plane "<<scan->plane_matches[j].ref->idx_plane<<std::endl;
//				map->addObservationPlane(scan->plane_matches[j].cur,map->cameras.size()-1,scan->plane_matches[j].ref->idx_plane);
//				map->planes[scan->plane_matches[j].ref->idx_plane]->indices_cameras.push_back(map->cameras.size()-1);
//				scan->plane_matches[j].cur->idx_plane=scan->plane_matches[j].ref->idx_plane;
//			}
//			else
			{
				PlaneLM *plane_lm=new PlaneLM(scan->plane_matches[j].ref, Transform());
											  //scan->scan_ref->Tcg.inv());
				plane_lm->id=map->planes.size();
				plane_lm->indices_cameras.push_back(map->cameras.size()-2);
				plane_lm->indices_cameras.push_back(map->cameras.size()-1);
				map->planes.push_back(plane_lm);
				map->addObservationPlane(scan->plane_matches[j].ref,map->cameras.size()-2,map->planes.size()-1);
				scan->plane_matches[j].ref->idx_plane=map->planes.size()-1;
				map->addObservationPlane(scan->plane_matches[j].cur,map->cameras.size()-1,map->planes.size()-1);
				scan->plane_matches[j].cur->idx_plane=map->planes.size()-1;
				if(debug) fp<<"\nadd a new plane landmark\n"
							<<"\tidx_camera\t"<<map->cameras.size()-2<<"\t"<<map->cameras.size()-1
							<<"\tidx_plane\t" <<map->planes.size()-1
				            <<"\n\tplane_lm\t"<<plane_lm->pi(0)<<"\t"<<plane_lm->pi(1)<<"\t"<<plane_lm->pi(2)<<"\t"<<plane_lm->pi(3)<<std::endl;
			}
		}
		for(size_t j=0;j<scan->line_matches.size();j++)
		{ // for each line observation;
//			if(scan->line_matches[j].ref->idx_line>=0)
//			{
//				if(debug) fp<<"\nline observed in map\n"
//							<<"\tidx_camera "<<map->cameras.size()-1
//							<<"\tidx_line "<<scan->line_matches[j].ref->idx_line<<std::endl;
//				// add line observation;
//				map->addObservationLine(scan->line_matches[j].cur,map->cameras.size()-1,scan->line_matches[j].ref->idx_line);
//				map->lines[scan->line_matches[j].ref->idx_line]->indices_cameras.push_back(map->cameras.size()-1);
//				scan->line_matches[j].cur->idx_line=scan->line_matches[j].ref->idx_line;
//				// add shadow observation;
//				for(int i=0;i<scan->plane_matches.size();i++)
//				{
//					map->addObservationShadow(scan->line_matches[j].cur,scan->plane_matches[i].cur,map->cameras.size()-1);
//				}
////				if(scan->line_matches[j].cur->occluded!=0)
////				{
////					if(scan->line_matches[j].cur->occluded->plane!=0)
////					{
////						if(scan->line_matches[j].cur->occluded->plane->idx_plane>=0)
////						{
////							map->addObservationShadow(scan->line_matches[j].cur->occluded,
////													  map->cameras.size()-1,//idx_camera
////													  scan->line_matches[j].cur->occluded->plane->idx_plane,//idx_plane
////													  scan->line_matches[j].cur->idx_line);//idx_line
////											 
////						}
////					}
////				}
//			}
//			else
			{
				LineLM *line_lm=new LineLM(scan->line_matches[j].ref, Transform());
										   //scan->scan_ref->Tcg.inv());
				line_lm->id=map->lines.size();
				line_lm->indices_cameras.push_back(map->cameras.size()-2);
				line_lm->indices_cameras.push_back(map->cameras.size()-1);
				map->lines.push_back(line_lm);
				if(debug) fp<<"\nadd a new line landmark and two observations\n"
							<<"\tidx_camera\t"<<map->cameras.size()-2<<"\t"<<map->cameras.size()-1
							<<"\tidx_line\t" <<map->lines.size()-1
				            <<"\n\tline_lm\t"<<line_lm->L(0)<<"\t"<<line_lm->L(1)<<"\t"<<line_lm->L(2)<<"\t"
											 <<line_lm->L(3)<<"\t"<<line_lm->L(4)<<"\t"<<line_lm->L(5)<<std::endl;

				// add line observation (ref);
				map->addObservationLine(scan->line_matches[j].ref,map->cameras.size()-2,map->lines.size()-1);
				scan->line_matches[j].ref->idx_line=map->lines.size()-1;
				for(int i=0;i<scan->plane_matches.size();i++)
				{
					map->addObservationShadow(scan->line_matches[j].ref,scan->plane_matches[i].ref,map->cameras.size()-2);
					int k=map->observ_shadows.size()-1;
					fp<<std::endl<<"shadow "<<k<<std::endl;
					fp<<"L "<<map->observ_shadows[k]->L.transpose()<<std::endl;
					fp<<"sqrt_info "<<std::endl<<map->observ_shadows[k]->sqrt_info<<std::endl;
					fp<<"cov "<<std::endl<<map->observ_shadows[k]->cov<<std::endl;
					fp<<"idx_camera "<<map->observ_shadows[k]->idx_camera<<std::endl;
					fp<<"idx_plane "<<map->observ_shadows[k]->idx_plane<<std::endl;
					fp<<"idx_line "<<map->observ_shadows[k]->idx_line<<std::endl;
				}

				// add line observation (cur);
				map->addObservationLine(scan->line_matches[j].cur,map->cameras.size()-1,map->lines.size()-1);
				scan->line_matches[j].cur->idx_line=map->lines.size()-1;
				for(int i=0;i<scan->plane_matches.size();i++)
				{
					map->addObservationShadow(scan->line_matches[j].cur,scan->plane_matches[i].cur,map->cameras.size()-1);
					int k=map->observ_shadows.size()-1;
					map->observ_shadows[k]->sqrt_info/=double(scan->plane_matches.size());
					fp<<std::endl<<"shadow "<<k<<std::endl;
					fp<<"L "<<map->observ_shadows[k]->L.transpose()<<std::endl;
					fp<<"sqrt_info "<<std::endl<<map->observ_shadows[k]->sqrt_info<<std::endl;
					fp<<"cov "<<std::endl<<map->observ_shadows[k]->cov<<std::endl;
					fp<<"idx_camera "<<map->observ_shadows[k]->idx_camera<<std::endl;
					fp<<"idx_plane "<<map->observ_shadows[k]->idx_plane<<std::endl;
					fp<<"idx_line "<<map->observ_shadows[k]->idx_line<<std::endl;
				}

//				// add shadow observation (ref);
//				if(scan->line_matches[j].ref->occluded!=0)
//				{
//					if(scan->line_matches[j].ref->occluded->plane!=0)
//					{
//						if(scan->line_matches[j].ref->occluded->plane->idx_plane>=0)
//						{
//							map->addObservationShadow(scan->line_matches[j].ref->occluded,
//													  map->cameras.size()-2,//idx_camera
//													  scan->line_matches[j].ref->occluded->plane->idx_plane,//idx_plane
//													  scan->line_matches[j].ref->idx_line);//idx_line
//							if(debug) fp<<"\nadd shadow observation (ref)\n"
//										<<"\tidx_camera\t"<<map->cameras.size()-2//idx_camera
//										<<"\tidx_plane\t"<<scan->line_matches[j].ref->occluded->plane->idx_plane//idx_plane
//										<<"\tidx_line\t"<<scan->line_matches[j].ref->idx_line<<std::endl;//idx_line
//						}
//					}
//				}
//				// add shadow observation (cur);
//				if(scan->line_matches[j].cur->occluded!=0)
//				{
//					if(scan->line_matches[j].cur->occluded->plane!=0)
//					{
//						if(scan->line_matches[j].cur->occluded->plane->idx_plane>=0)
//						{
//							map->addObservationShadow(scan->line_matches[j].cur->occluded,
//													  map->cameras.size()-1,//idx_camera
//													  scan->line_matches[j].cur->occluded->plane->idx_plane,//idx_plane
//													  scan->line_matches[j].cur->idx_line);//idx_line
//							if(debug) fp<<"\nadd shadow observation (cur)\n"
//										<<"\tidx_camera\t"<<map->cameras.size()-1//idx_camera
//										<<"\tidx_plane\t"<<scan->line_matches[j].cur->occluded->plane->idx_plane//idx_plane
//										<<"\tidx_line\t"<<scan->line_matches[j].cur->idx_line<<std::endl;//idx_line
//						}
//					}
//				}
			}
		}
//		// add observation shadows;
//		int id=0;
//		for(std::list<Line*>::iterator it_line=scan->lines_occluded.begin();
//									   it_line!=scan->lines_occluded.end();it_line++)
//		{ // for each occluded line in current frame;
//			Eigen::Vector3d u=(*it_line)->u;
//			Eigen::Vector3d v=(*it_line)->v;
//			double min=DBL_MAX;
//			int idx=-1;
//			for(int i=0;i<scan->plane_matches.size();i++)
//			{
//				Eigen::Vector3d n=scan->plane_matches[i].ref->normal;
//				double d=scan->plane_matches[i].ref->d;
//				Eigen::Vector3d tmp=u.cross(n)+d*v;
//				double dist=tmp.norm();
//				if(dist<min)
//				{
//					min=dist;
//					idx=i;
//				}
//				std::cout<<"|uxn+dv| = "<<dist<<std::endl;
//				displayLineShadowPlane(scan->scan_ref,scan->plane_matches[i].ref,*it_line,vis);
//				vis->spin();
//			}
//			if(min<0.1)
//			{
//				(*it_line)->plane=scan->plane_matches[i].ref;
//				min=DBL_MAX;
//				idx=-1;
//				for(int k=0;k<scan->line_matches.size();k++)
//				{
//					Eigen::Vector3d u_pi=scan->line_matches[k].ref->u;
//					u.normalize();
//					u_pi.normalize();
//					double dist=u.transpose()*u_pi;
//					dist=fabs(dist);
//					dist=acos(dist)*180.0/M_PI;
//					if(dist<min)
//					{
//						min=dist;
//						idx=k;
//					}
//					std::cout<<"acos(u^T*u_pi) = "<<dist<<std::endl;
//					displayLineShadow(scan->scan_ref,scan->line_matches[k].ref,*it_line,vis);
//					vis->spin();
//				}
//				if(min<1.0)
//				{
//					scan->line_matches[idx].ref->occluded=(*it_line);
//					addObservationShadow(*it_line,0,(*it_line)->plane->idx_plane,scan->line_matches[idx].ref->idx_line);
//				}
//			}
//		}

		if(debug)
		{
			fp<<"\nnum_observ_planes_ = "<<map->observ_planes.size()<<std::endl;
			fp<<"num_observ_lines_ = "<<map->observ_lines.size()<<std::endl<<std::endl;
			fp<<"num_cameras_ = "<<map->cameras.size()<<std::endl;
			fp<<"num_planes_ = "<<map->planes.size()<<std::endl;
			fp<<"num_lines_ = "<<map->lines.size()<<std::endl<<std::endl;

			fp<<"\nmap->cameras"<<std::endl;
			for(size_t i=0;i<map->cameras.size();i++)
			{
				Eigen::Quaterniond q=map->cameras[i].Quat();
				fp<<"\t"<<i<<"\t"<<q.w()<<"\t"<<q.vec().transpose()<<"\t"<<map->cameras[i].t.transpose()<<std::endl;
				fp<<"\t\tplanes";
				for(int j=0;j<map->indices_planes[i].size();j++)
				{
					fp<<"\t"<<map->indices_planes[i][j];
				}
				fp<<std::endl;
				fp<<"\t\tlines";
				for(int j=0;j<map->indices_lines[i].size();j++)
				{
					fp<<"\t"<<map->indices_lines[i][j];
				}
				fp<<std::endl;
			}

			fp<<"\nmap->planes"<<std::endl;
			for(size_t i=0;i<map->planes.size();i++)
			{
				fp<<"\t"<<i<<"\t"<<map->planes[i]->pi.transpose()<<std::endl;
				fp<<"\t\tobserved in";
				for(int j=0;j<map->planes[i]->indices_cameras.size();j++)
				{
					fp<<"\t"<<map->planes[i]->indices_cameras[j];
				}
				fp<<std::endl;
			}

			fp<<"\nmap->lines"<<std::endl;
			for(size_t i=0;i<map->lines.size();i++)
			{
				fp<<"\t"<<i<<"\t"<<map->lines[i]->L.transpose()<<std::endl;
				fp<<"\t\tobserved in";
				for(int j=0;j<map->lines[i]->indices_cameras.size();j++)
				{
					fp<<"\t"<<map->lines[i]->indices_cameras[j];
				}
				fp<<std::endl;
			}

			fp<<"\nmap->observ_planes"<<std::endl;
			for(size_t i=0;i<map->observ_planes.size();i++)
			{
				fp<<"\t"<<i<<"\t"<<map->observ_planes[i]->idx_camera<<"\t"<<map->observ_planes[i]->idx_plane<<"\t"<<map->observ_planes[i]->pi.transpose()<<std::endl;
			}

			fp<<"\nmap->observ_lines"<<std::endl;
			for(size_t i=0;i<map->observ_lines.size();i++)
			{
				fp<<"\t"<<i<<"\t"<<map->observ_lines[i]->idx_camera<<"\t"<<map->observ_lines[i]->idx_line<<"\t"<<map->observ_lines[i]->L.transpose()<<std::endl;
			}

			fp<<"\nmap->observ_shadows"<<std::endl;
			for(size_t i=0;i<map->observ_shadows.size();i++)
			{
				fp<<"\t"<<i<<"\t"<<map->observ_shadows[i]->idx_camera<<"\t"<<map->observ_shadows[i]->idx_plane<<"\t"<<map->observ_shadows[i]->idx_line<<"\t"<<map->observ_shadows[i]->L.transpose()<<std::endl;
			}
		}

		fp.close();
	}
}
