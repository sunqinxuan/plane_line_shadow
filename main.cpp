/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2019-05-10 11:08
#
# Filename:		main.cpp
#
# Description: 
#
===============================================*/

#include <sys/time.h>
#include "system_plane_line_shadow.h"
#include "relative_pose_error.h"

using namespace Eigen;
using namespace std;
using namespace ulysses;

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void);

void loadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps, 
				std::vector<ulysses::Transform> &groundtruth);
bool flag;

int main(int argc, char *argv[])
{
	flag=true;
	vtkObject::GlobalWarningDisplayOff();
	// usage: bin/RazorEdge file_settings path_to_seq file_association
	// argv[1] - setting file;
	// argv[2] - sequence path;
	// argv[3] - association file;

	std::vector<std::string> imgFilenames_depth;
	std::vector<std::string> imgFilenames_rgb;
	std::vector<double> timestamps;
	std::vector<ulysses::Transform> groundtruth;
	std::string file_association=std::string(argv[3]);
	loadImages(file_association,imgFilenames_rgb,imgFilenames_depth,timestamps,groundtruth);

	int num_imgs=imgFilenames_depth.size();
	if(imgFilenames_depth.empty())
	{
		std::cerr<<"No images found."<<std::endl;
		return 1;
	}
	else if(imgFilenames_rgb.size()!=imgFilenames_depth.size())
	{
		std::cerr<<"Different number of rgb and depth images."<<std::endl;
		return 1;
	}

	cv::FileStorage settings(argv[1],cv::FileStorage::READ);
	int debug=(int)settings["debug"];
	int start_frame=(int)settings["start_frame"];

	// use both lines and shadows --- red
	ulysses::SystemPlaneLineShadow system(argv[1]);
	system.useShadow((int)settings["useShadow"]);
	system.useLineResidual((int)settings["useLineResidual"]);
	// use only lines --- blue
	ulysses::SystemPlaneLineShadow system2(argv[1]);
	system2.useShadow(false);
	// use only shadows --- green
	ulysses::SystemPlaneLineShadow system3(argv[1]);
	system3.useLineResidual(false);

	ulysses::RelativePoseError RPE((double)settings["Evaluate.time_interval"]);
	ulysses::RelativePoseError RPE2((double)settings["Evaluate.time_interval"]);
	ulysses::RelativePoseError RPE3((double)settings["Evaluate.time_interval"]);

	std::vector<double> times_track;
	times_track.resize(num_imgs);
	// for debugging
	std::vector<int> num_observ;

	std::cout<<"Start processing ..."<<std::endl;
	std::cout<<"number of images: "<<num_imgs<<std::endl<<std::endl;

	boost::shared_ptr<pcl::visualization::PCLVisualizer> vis (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	vis->setBackgroundColor (0, 0, 0);
	vis->initCameraParameters ();
	vis->registerKeyboardCallback (keyboardEventOccurred, (void*)vis.get());

	boost::shared_ptr<pcl::visualization::PCLPainter2D> fig_error (new pcl::visualization::PCLPainter2D ("Error"));
	fig_error->setWindowSize (600, 400);
	fig_error->setPenColor(0,0,0,200);
	fig_error->addLine(0,200,600,200);

	timeval time_start, time_end;
	double time_used;
	cv::Mat img_rgb,img_depth;
	std::ofstream fp;
	fp.open("frames.txt",std::ios::out);
	for(int i=start_frame;i<num_imgs;i+=3)
	{
		if(!flag) break;
		fp<<std::fixed<<timestamps[i]<<"\t"<<i<<std::endl;
		std::cout<<std::endl<<"***************************************************************************"<<std::endl;
		std::cout<<std::fixed<<timestamps[i]<<"\t"<<i<<std::endl;
		img_depth=cv::imread(std::string(argv[2])+"/"+imgFilenames_depth[i],CV_LOAD_IMAGE_UNCHANGED);
		img_rgb=cv::imread(std::string(argv[2])+"/"+imgFilenames_rgb[i],CV_LOAD_IMAGE_UNCHANGED);
		double time_stamp=timestamps[i];
		if(img_depth.empty())
		{
			std::cerr<<"Failed to load image at "<<std::string(argv[2])<<"/"<<imgFilenames_rgb[i]<<std::endl;
			return 1;
		}

		gettimeofday(&time_start,NULL);
		if(system.trackCamera(img_rgb,img_depth,time_stamp,vis,groundtruth[i]))
		{
			fig_error->setPenColor(255,0,0,200);
			RPE.evaluateEachFrame(time_stamp,groundtruth[i],system.getCurrentCamera(),fig_error);
	//		RPE.evaluate(time_stamp,groundtruth[i],system.getCurrentCamera(),fig_error);
			fig_error->spinOnce();
		
		}
		gettimeofday(&time_end,NULL);
		time_used=(1000000*(time_end.tv_sec-time_start.tv_sec)+time_end.tv_usec-time_start.tv_usec)/1000;
		times_track[i]=time_used;
		std::cout<<"processing time per frame - "<<time_used<<std::endl;
		
//		char ch;
//		std::cout<<"Press enter to continue...\n"<<std::endl;
//		ch=std::cin.get();

//		if(debug==0)
		if(false)
		{
			if(system2.trackCamera(img_rgb,img_depth,time_stamp,vis,groundtruth[i]))
			{
				fig_error->setPenColor(0,0,255,200);
				RPE2.evaluateEachFrame(time_stamp,groundtruth[i],system2.getCurrentCamera(),fig_error);
	//			RPE2.evaluate(time_stamp,groundtruth[i],system2.getCurrentCamera(),fig_error);
				fig_error->spinOnce();
			}
			if(system3.trackCamera(img_rgb,img_depth,time_stamp,vis,groundtruth[i]))
			{
				fig_error->setPenColor(0,255,0,200);
				RPE3.evaluateEachFrame(time_stamp,groundtruth[i],system3.getCurrentCamera(),fig_error);
				fig_error->spinOnce();
			}

		}
		
	}

//	system.saveTraj(std::string("traj.txt"));
	system.saveTraj(std::string("traj.txt"),true);
	RPE.saveErrors(std::string("errors.txt"));

//	if(debug==0)
	if(false)
	{
//		system2.saveTraj(std::string("traj2.txt"));
		system2.saveTraj(std::string("traj2.txt"),true);
		RPE2.saveErrors(std::string("errors2.txt"));
		system3.saveTraj(std::string("traj3.txt"),true);
		RPE3.saveErrors(std::string("errors3.txt"));
	}

	system.shutDown();
	system2.shutDown();
	system3.shutDown();
	fp.close();

	return 0;
}

void loadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps, 
				std::vector<ulysses::Transform> &groundtruth)
{
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);
            ss >> t;
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);

			double tx,ty,tz,qx,qy,qz,qw;
			ss>>t;
			ss>>tx>>ty>>tz>>qx>>qy>>qz>>qw;
			Eigen::Quaterniond quat(qw,qx,qy,qz);
			Eigen::Vector3d trans(tx,ty,tz);
			ulysses::Transform Tg(quat,trans);
			groundtruth.push_back(Tg);
        }
    }
}

void saveTimesTrack(const std::vector<double> &times)
{
	std::ofstream fp;
	fp.open("times.txt",std::ios::out);
	for(int i=0;i<times.size();i++)
	{
		fp<<times[i]<<std::endl;
	}
	fp.close();
}


void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
	pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
	if (event.getKeySym () == "q" && event.keyDown ())
	{
	   viewer->close();
	}
	if (event.getKeySym () == "c" && event.keyDown ())
	{
		flag=false;
	}
}

//1305031104.331403 depth/1305031104.331403.png 1305031104.343342 rgb/1305031104.343342.png 1305031104.335800 1.2911 0.6284 1.6501 0.6468 0.6360 -0.2959 -0.2993

/*
int main1 (int argc, char *argv[])
{
	cv::FileStorage settings("settings/settings.yaml",cv::FileStorage::READ);
	bool db;
	if((int)settings["debug"]==0)
		db=false;
	else
		db=true;
	cout<<db<<endl;
//	double camera_fx=settings["Camera.fx"];
//	cout<<camera_fx<<endl;
	return 0;

	vtkObject::GlobalWarningDisplayOff();
	// some default settings;
	std::string sequence_name="";
//	std::string sequence_name="/home/sun/dataset/rgbd_dataset_freiburg3_structure_notexture_far/";
	double time_start=1341836538.6;
	double time_interval=0.1;
	int min_inliers=1000;
	bool usePln=true, usePt=false;
	bool usePlane=true, useLine=true, useShadow=true;;
	int pln_fitting_method=0;
	double m_fp=2.85e-3, sigma_u=0.5, sigma_v=0.5;
	int vis_every_n_frames=10;
	int total_frames=1000;
	double alpha=1, beta=1;
	int max_icp=10, max_lm=10;
	int occluding=2, occluded=4, curvature=0, canny=0;
	bool useWeight=true;
	double delta_time=2, delta_angle=6.0, delta_dist=0.1;
	std::string traj_path="/home/sun/traj/";
	enum Mode {DEBUG, VIS_SCAN, CLOSE_LOOP, CLOSE_LOOP_FILE, TRAJ_PUZZLE, RELEASE, ONLINE, COLLECT, VIEW} mode;
	int key_frame=1;
	int kinect2=0;
	double thres_association=0.01;
	bool visual;
	double pln_angle=0.2, pln_dist=0.05, pln_color=0.5;
	double edge_th=0.05, edge_pxl=10, edge_dist=0.1, edge_rad=0.1, edge_meas=20;
	int edge_max=50, edge_k=20;
	double edge_ratio, edge_angle;
	int pln_cell1=10,pln_cell2=20,pln_cell3=1;
	bool save_forDisplay=false;
	double fitline_rho, fitline_theta, fitline_minLineLength, fitline_maxLineGap;
	int fitline_threshold;
	double fitline_sim_dir=10.0, fitline_sim_dist=0.1;
	double fitline_split=0.05;
	int fitline_min_points_on_line=50;
	double plane_ang_thres=5.0, plane_dist_thres=0.05, plane_max_curv=0.01;
	double matchline_dir=10.0, matchline_normal=5.0, matchline_dist=0.1, matchline_delta_dir=5.0, matchline_delta_normal=1.0, matchline_delta_dist=0.05;
	double line_alpha_l,line_alpha_pi;
	double dist_line2shadow=1.0, dist_shadow2plane=0.1;

	for(int i=1;i<argc;i++)
	{
		if(strcmp(argv[i],"-mode")==0)
		{
			if(strcmp(argv[i+1],"debug")==0)
				mode=DEBUG;
			if(strcmp(argv[i+1],"vis_scan")==0)
				mode=VIS_SCAN;
			if(strcmp(argv[i+1],"close_loop")==0)
				mode=CLOSE_LOOP;
			if(strcmp(argv[i+1],"traj_puzzle")==0)
				mode=TRAJ_PUZZLE;
			if(strcmp(argv[i+1],"close_loop_file")==0)
				mode=CLOSE_LOOP_FILE;
			if(strcmp(argv[i+1],"release")==0)
				mode=RELEASE;
			if(strcmp(argv[i+1],"online")==0)
				mode=ONLINE;
			if(strcmp(argv[i+1],"collect")==0)
				mode=COLLECT;
			if(strcmp(argv[i+1],"view")==0)
				mode=VIEW;
		}
		if(strcmp(argv[i],"-ds")==0) {sequence_name=argv[i+1];}
		if(strcmp(argv[i],"-st")==0) {time_start=atof(argv[i+1]);}
		if(strcmp(argv[i],"-ti")==0) {time_interval=atof(argv[i+1]);}
		if(strcmp(argv[i],"-mi")==0) {min_inliers=atoi(argv[i+1]);}
		if(strcmp(argv[i],"-pln_angle")==0) {pln_angle=atof(argv[i+1]);}
		if(strcmp(argv[i],"-pln_dist")==0) {pln_dist=atof(argv[i+1]);}
		if(strcmp(argv[i],"-pln_color")==0) {pln_color=atof(argv[i+1]);}
		if(strcmp(argv[i],"-pln_cell1")==0) {pln_cell1=atoi(argv[i+1]);}
		if(strcmp(argv[i],"-pln_cell2")==0) {pln_cell2=atoi(argv[i+1]);}
		if(strcmp(argv[i],"-pln_cell3")==0) {pln_cell3=atoi(argv[i+1]);}
		if(strcmp(argv[i],"-edge_th")==0) {edge_th=atof(argv[i+1]);}
		if(strcmp(argv[i],"-edge_pxl")==0) {edge_pxl=atof(argv[i+1]);}
		if(strcmp(argv[i],"-edge_dist")==0) {edge_dist=atof(argv[i+1]);}
		if(strcmp(argv[i],"-edge_max")==0) {edge_max=atoi(argv[i+1]);}
		if(strcmp(argv[i],"-edge_rad")==0) {edge_rad=atof(argv[i+1]);}
		if(strcmp(argv[i],"-edge_k")==0) {edge_k=atoi(argv[i+1]);}
		if(strcmp(argv[i],"-edge_meas")==0) {edge_meas=atof(argv[i+1]);}
		if(strcmp(argv[i],"-edge_ratio")==0) {edge_ratio=atof(argv[i+1]);}
		if(strcmp(argv[i],"-edge_angle")==0) {edge_angle=atof(argv[i+1]);}
		if(strcmp(argv[i],"-fitline_rho")==0) {fitline_rho=atof(argv[i+1]);}
		if(strcmp(argv[i],"-fitline_theta")==0) {fitline_theta=atof(argv[i+1]);fitline_theta=fitline_theta*M_PI/180.0;}
		if(strcmp(argv[i],"-fitline_minLineLength")==0) {fitline_minLineLength=atof(argv[i+1]);}
		if(strcmp(argv[i],"-fitline_maxLineGap")==0) {fitline_maxLineGap=atof(argv[i+1]);}
		if(strcmp(argv[i],"-fitline_threshold")==0) {fitline_threshold=atoi(argv[i+1]);}
		if(strcmp(argv[i],"-fitline_sim_dir")==0) {fitline_sim_dir=atof(argv[i+1]);}
		if(strcmp(argv[i],"-fitline_sim_dist")==0) {fitline_sim_dist=atof(argv[i+1]);}
		if(strcmp(argv[i],"-fitline_split")==0) {fitline_split=atof(argv[i+1]);}
		if(strcmp(argv[i],"-fitline_min_points_on_line")==0) {fitline_min_points_on_line=atoi(argv[i+1]);}
		if(strcmp(argv[i],"-matchline_dir")==0) {matchline_dir=atof(argv[i+1]);}
		if(strcmp(argv[i],"-matchline_normal")==0) {matchline_normal=atof(argv[i+1]);}
		if(strcmp(argv[i],"-matchline_dist")==0) {matchline_dist=atof(argv[i+1]);}
		if(strcmp(argv[i],"-matchline_delta_dir")==0) {matchline_delta_dir=atof(argv[i+1]);}
		if(strcmp(argv[i],"-matchline_delta_normal")==0) {matchline_delta_normal=atof(argv[i+1]);}
		if(strcmp(argv[i],"-matchline_delta_dist")==0) {matchline_delta_dist=atof(argv[i+1]);}
		if(strcmp(argv[i],"-line_alpha_l")==0) {line_alpha_l=atof(argv[i+1]);}
		if(strcmp(argv[i],"-line_alpha_pi")==0) {line_alpha_pi=atof(argv[i+1]);}

		if(strcmp(argv[i],"-icp")==0) {max_icp=atoi(argv[i+1]);}
		if(strcmp(argv[i],"-lm")==0) {max_lm=atoi(argv[i+1]);}
		if(strcmp(argv[i],"-plane_ang_thres")==0) {plane_ang_thres=atof(argv[i+1]);}
		if(strcmp(argv[i],"-plane_dist_thres")==0) {plane_dist_thres=atof(argv[i+1]);}
		if(strcmp(argv[i],"-plane_max_curv")==0) {plane_max_curv=atof(argv[i+1]);}
		if(strcmp(argv[i],"-dist_line2shadow")==0) {dist_line2shadow=atof(argv[i+1]);}
		if(strcmp(argv[i],"-dist_shadow2plane")==0) {dist_shadow2plane=atof(argv[i+1]);}
		if(strcmp(argv[i],"-pln")==0)
		{
			if(strcmp(argv[i+1],"1")==0)
				usePln=true;
			if(strcmp(argv[i+1],"0")==0)
				usePln=false;
		}
		if(strcmp(argv[i],"-pt")==0)
		{
			if(strcmp(argv[i+1],"1")==0)
				usePt=true;
			if(strcmp(argv[i+1],"0")==0)
				usePt=false;
		}
		if(strcmp(argv[i],"-usePlane")==0)
		{
			if(strcmp(argv[i+1],"1")==0)
				usePlane=true;
			if(strcmp(argv[i+1],"0")==0)
				usePlane=false;
		}
		if(strcmp(argv[i],"-useLine")==0)
		{
			if(strcmp(argv[i+1],"1")==0)
				useLine=true;
			if(strcmp(argv[i+1],"0")==0)
				useLine=false;
		}
		if(strcmp(argv[i],"-useShadow")==0)
		{
			if(strcmp(argv[i+1],"1")==0)
				useShadow=true;
			if(strcmp(argv[i+1],"0")==0)
				useShadow=false;
		}
		if(strcmp(argv[i],"-plnfit")==0) {pln_fitting_method=atoi(argv[i+1]);}
		if(strcmp(argv[i],"-vis")==0) {vis_every_n_frames=atoi(argv[i+1]);}
		if(strcmp(argv[i],"-frames")==0) {total_frames=atoi(argv[i+1]);}
		if(strcmp(argv[i],"-alpha")==0) {alpha=atof(argv[i+1]);}
		if(strcmp(argv[i],"-beta")==0) {beta=atof(argv[i+1]);}
		if(strcmp(argv[i],"-associate")==0) {thres_association=atof(argv[i+1]);}
		if(strcmp(argv[i],"-occluding")==0)
		{
			if(strcmp(argv[i+1],"1")==0)
				occluding=2;
			if(strcmp(argv[i+1],"0")==0)
				occluding=0;
		}
		if(strcmp(argv[i],"-occluded")==0)
		{
			if(strcmp(argv[i+1],"1")==0)
				occluded=4;
			if(strcmp(argv[i+1],"0")==0)
				occluded=0;
		}
		if(strcmp(argv[i],"-curvature")==0)
		{
			if(strcmp(argv[i+1],"1")==0)
				curvature=8;
			if(strcmp(argv[i+1],"0")==0)
				curvature=0;
		}
		if(strcmp(argv[i],"-canny")==0)
		{
			if(strcmp(argv[i+1],"1")==0)
				canny=16;
			if(strcmp(argv[i+1],"0")==0)
				canny=0;
		}
		if(strcmp(argv[i],"-useWeight")==0)
		{
			if(strcmp(argv[i+1],"1")==0)
				useWeight=true;
			if(strcmp(argv[i+1],"0")==0)
				useWeight=false;
		}
		if(strcmp(argv[i],"-traj_path")==0) {traj_path=argv[i+1];}
		if(strcmp(argv[i],"-delta_time")==0) {delta_time=atof(argv[i+1]);}
		if(strcmp(argv[i],"-delta_angle")==0) {delta_angle=atof(argv[i+1]);}
		if(strcmp(argv[i],"-delta_dist")==0) {delta_dist=atof(argv[i+1]);}
		if(strcmp(argv[i],"-key_frame")==0) {key_frame=atoi(argv[i+1]);}
		if(strcmp(argv[i],"-kinect2")==0)
		{
			if(strcmp(argv[i+1],"2")==0)
				kinect2=2;
			if(strcmp(argv[i+1],"1")==0)
				kinect2=1;
			if(strcmp(argv[i+1],"0")==0)
				kinect2=0;
		}
		if(strcmp(argv[i],"-visual")==0)
		{
			if(strcmp(argv[i+1],"1")==0)
				visual=true;
			if(strcmp(argv[i+1],"0")==0)
				visual=false;
		}
		if(strcmp(argv[i],"-display")==0)
		{
			if(strcmp(argv[i+1],"1")==0)
				save_forDisplay=true;
			if(strcmp(argv[i+1],"0")==0)
				save_forDisplay=false;
		}
	}

	if(mode==TRAJ_PUZZLE)
	{
		std::cout<<traj_path<<std::endl;
		TrajPuzzle traj_puzzle(traj_path);
		traj_puzzle.readTrajFiles();
		return 0;
	}

	ulysses::Scan *scan_ref;
	ulysses::Scan *scan_cur;
	ulysses::IntrinsicParam cam;
	cam.m_fp=m_fp;
	cam.sigma_u=sigma_u;
	cam.sigma_v=sigma_v;
	if(kinect2==2)
	{
		cam.fx=540.686;
		cam.fy=540.686;
		cam.cx=479.75;
		cam.cy=269.75;
		cam.width=960;
		cam.height=540;
	}
	if(kinect2==1)
	{
		cam.fx=367.933;//params.fx;
		cam.fy=367.933;//params.fy;
		cam.cx=254.169;//params.cx;
		cam.cy=204.267;//params.cy;
		cam.width=512;
		cam.height=424;
		cam.factor=1000.0;
	}
	if(kinect2==0)
	{
		cam.fx=567.6;
		cam.fy=570.2;
		cam.cx=324.7;
		cam.cy=250.1;
		cam.width=640;
		cam.height=480;
	
	}
	std::vector<ulysses::PlanePair> matched_planes;
    ulysses::Transform Tcr_align_planes, Tgc, Tcr, Tcr_gt;
	Eigen::Matrix<double,6,1> xi_cr, xi_cr_gt, delta_xi;
	bool debug=(mode==DEBUG);

	DataReading *data_reading=new ulysses::DataReading();
	data_reading->setPath(sequence_name);
	data_reading->setDebug(true);
	data_reading->setSampleInterval(time_interval);
	
	ulysses::PlaneParamEstimation plane_fitting;
	plane_fitting.setDebug(debug);
	plane_fitting.setPlnFittingMethod(pln_fitting_method);

	ulysses::PlaneFeatureMatching pfm;
	pfm.setDebug(debug);

	ulysses::LineFeatureMatching lfm;
	lfm.setDebug(debug);
	lfm.setThres(matchline_dir,matchline_normal,matchline_dist,matchline_delta_dir,matchline_delta_normal,matchline_delta_dist);

	ulysses::PoseEstimation pe;
	pe.setDebug(debug);
	pe.setVisual(visual);
	pe.usePlnPt(usePln,usePt);
	pe.useEdgeWeight(useWeight);
	pe.setAlpha(alpha);
	pe.setBeta(beta);
	pe.setMaxIterationICP(max_icp);
	pe.setMaxIterationLM(max_lm);
	pe.setThresAssociation(thres_association);
	ulysses::PoseEstimation::ConstraintCase case_cur, case_pre;

	ulysses::MotionEstimation me;
	me.setDebug(debug);
	me.setVisual(visual);
	me.usePlnPt(usePln,usePt);
	me.setMaxIterationICP(max_icp);
	me.setMaxIterationLM(max_lm);
	me.setThresAssociation(thres_association);

	ulysses::MotionEstimation_Line mel;
	mel.setDebug(debug);
	mel.setThres(line_alpha_l,line_alpha_pi);
	mel.usePlane(usePlane);
	mel.useLine(useLine);
	mel.useShadow(useShadow);

	ulysses::PlaneExtraction extract;
	extract.setDebug(debug);
//	extract.setMinPlaneSize(min_inliers);
//	extract.setThres(pln_angle,pln_dist,pln_color);
//	extract.allocBottomGrid(pln_cell1,pln_cell2,pln_cell3);

	ulysses::EdgePointExtraction edge_ext;
	edge_ext.setDebug(debug);
	edge_ext.setThres(edge_th,edge_max,edge_pxl,edge_dist,edge_rad,edge_k,edge_meas,edge_ratio,edge_angle);
	edge_ext.setThresPln(min_inliers,plane_ang_thres,plane_dist_thres,plane_max_curv);
	int edge_type=occluding|occluded|canny;
	edge_ext.setEdgeType(edge_type);

	ulysses::LineExtraction line_ext;
	line_ext.setDebug(debug);
	line_ext.setThresLine(fitline_rho, fitline_theta, fitline_threshold, fitline_minLineLength, fitline_maxLineGap, fitline_sim_dir, fitline_sim_dist,fitline_split,fitline_min_points_on_line,dist_line2shadow,dist_shadow2plane);

	ulysses::Map *map=new ulysses::Map;
	ulysses::GlobalMap gl;
	gl.setDebug(debug);

//	ulysses::BundleAdjustment ba;
//	ba.setDebug(debug);
//	ba.setVisual(visual);


//	ulysses::PlaneFusing pf;
//	pf.setDebug(debug);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> vis (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	vis->setBackgroundColor (0, 0, 0);
//	vis->setBackgroundColor (0.78, 0.78, 0.78);
//	vis->addCoordinateSystem (0.5);
	vis->initCameraParameters ();
	vis->registerKeyboardCallback (keyboardEventOccurred, (void*)vis.get());

//	boost::shared_ptr<pcl::visualization::PCLPainter2D> fig (new pcl::visualization::PCLPainter2D ("Figure"));
//	fig->setWindowSize (600, 400);

	if(mode==VIEW)
	{
//		pcl::visualization::CloudViewer view("point_cloud_map");
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr point_cloud_map(new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
//		pcl::VoxelGrid<pcl::PointXYZRGBA> filter;
//		filter.setLeafSize(0.005,0.005,0.005);
		std::ifstream fp_traj;
		fp_traj.open("traj.txt",std::ios::in);
		double tx,ty,tz,qx,qy,qz,qw,time;
		char ch;
		int count=0;
        ulysses::Transform T0;
		if(time_start!=0)
		{
			while(true)
			{
				fp_traj>>time>>tx>>ty>>tz>>qx>>qy>>qz>>qw;
				ulysses::Transform Tgc(Eigen::Quaterniond(qw,qx,qy,qz), Eigen::Vector3d(tx,ty,tz));
				if(fabs(time-time_start)<0.01)
				{
					T0=Tgc;
					break;
				}
			}
		}
		fp_traj.close();

		vis->removeAllPointClouds();
		vis->removeAllShapes();
//		vis->spin();
		fp_traj.open("traj.txt",std::ios::in);
		while(true)
		{
			fp_traj>>time>>tx>>ty>>tz>>qx>>qy>>qz>>qw;
			if(time<time_start)
			{
				if(count%10!=0) 
				{
					count++;
					continue;
				}
			}
			else
			{
				if(count%key_frame!=0) 
				{
					count++;
					continue;
				}
			}

			scan_cur=new Scan(count,cam);
			data_reading->loadScan_once(scan_cur,time);
			ulysses::Transform Tgc(Eigen::Quaterniond(qw,qx,qy,qz), Eigen::Vector3d(tx,ty,tz));
//			scan_cur->Tcg=Tgc.inv();
//			scan_cur->id=count;
//			if(count==0)
//				T0=Tgc;
			if(fabs(time-time_start)<0.01)
			{
				vis->spin();
			}
			scan_cur->Tcg=Tgc.inv()*T0;
			Tgc=scan_cur->Tcg.inv();
			std::cout<<std::fixed<<time<<" "<<Tgc.t.transpose()<<" "<<Tgc.Quaternion().transpose()<<std::endl;
//			pcl::transformPointCloud(*scan_cur->point_cloud,*point_cloud,Tgc.getMatrix4f());
//			filter.setInputCloud(point_cloud);
//			filter.filter(*point_cloud);
//			*point_cloud_map=*point_cloud_map+*point_cloud;
//			for(size_t i=0;i<scan_cur->point_cloud->size();i++)
//			{
//				if(scan_cur->point_cloud->at(i).y<-0.3 || scan_cur->point_cloud->at(i).y>0.3)
//				{
//					scan_cur->point_cloud->at(i).x=0;
//					scan_cur->point_cloud->at(i).y=0;
//					scan_cur->point_cloud->at(i).z=0;
//				}
//			}
			display_addScan(scan_cur,vis);
			display_addCameraPose(scan_cur,vis);
			vis->spinOnce(200);
//			vis->removeAllPointClouds();
//			view.showCloud(point_cloud_map);
			std::cout<<std::fixed<<time<<std::endl;
			delete scan_cur;
			if(count==0)
			{
				vis->spin();
//				std::cout<<"Press enter to continue...\n"<<std::endl;
//				ch=std::cin.get();
			}
			count++;
			if(fp_traj.eof())
			{
				vis->spin();
//				std::cout<<"Press enter to continue...\n"<<std::endl;
//				ch=std::cin.get();
				break;
			}
		}
		fp_traj.close();
		return 0;
	}

	ofstream fp;
	fp.open("traj.txt",std::ios::out);
	ofstream fp_edge;
	fp_edge.open("traj_edge.txt",std::ios::out);
	ofstream fp_shadow;
	fp_shadow.open("traj_shadow.txt",std::ios::out);
	ofstream fp_forDisplay;
	

	int first=0;
	int filenum = first;
	timeval start, end;
	double timeused;

	ofstream fp_error;
	fp_error.open("error.txt",std::ios::out);
	ofstream fp_residual;
	fp_residual.open("residual.txt",std::ios::out);
	ofstream fp_notes;
	fp_notes.open("notes.txt",std::ios::out);

	boost::shared_ptr<pcl::visualization::PCLPainter2D> fig_error (new pcl::visualization::PCLPainter2D ("Error"));
	fig_error->setWindowSize (600, 400);
	fig_error->setPenColor(0,0,0,200);
	fig_error->addLine(0,200,600,200);

	data_reading->Initialize(time_start);
	double err_pos_pre=0,err_ang_pre=0;
	double err_pos_pre_edge=0,err_ang_pre_edge=0;
	double err_pos_pre_shadow=0,err_ang_pre_shadow=0;
	double err_pos=0, err_ang=0;
	double err_pos_edge=0, err_ang_edge=0;
	double err_pos_shadow=0, err_ang_shadow=0;
	double res_occluding_Tcr=0, res_occluding_Tcr_gt=0;
	double res_occluding_Tcr_pre=0, res_occluding_Tcr_gt_pre=0;
	double res_occluded_Tcr=0, res_occluded_Tcr_gt=0;
	double res_occluded_Tcr_pre=0, res_occluded_Tcr_gt_pre=0;

	std::vector<Scan*> scans;
	PlaneLineBA* plane_line_ba;

	while(!data_reading->isEOF() && filenum<total_frames)
	{
		std::cout<<std::endl<<"***************** frame "<<filenum<<" ******************"<<std::endl;
		fp_notes<<std::endl<<"***************** frame\t"<<filenum<<"\t"<<std::fixed<<data_reading->getTime()<<" ******************"<<std::endl;

		// load point cloud from the image sequence;
		gettimeofday(&start,NULL);
		scan_cur=new Scan(filenum,cam);
//		scan_cur->cam=cam;
//		data_reading->read(scan_cur,cam,rgb,dep);
		if(!data_reading->loadScan(scan_cur)) break;
		gettimeofday(&end,NULL);
		timeused=(1000000*(end.tv_sec-start.tv_sec)+end.tv_usec-start.tv_usec)/1000;
		std::cout<<"time reading data from freiburg:"<<timeused<<std::endl;
//		scan_cur->id=filenum;

//		for(size_t i=0;i<scan_cur->point_cloud->height;i++)
//		{
//			for(size_t j=0;j<scan_cur->point_cloud->width;j++)
//			{
//				fp<<scan_cur->point_cloud->points[i*640+j].x<<"\t"
//				  <<scan_cur->point_cloud->points[i*640+j].y<<"\t"
//				  <<scan_cur->point_cloud->points[i*640+j].z<<"\t"<<std::endl;
//			}
//		}
//		return 0;

//		if (!vis->updatePointCloud (scan_cur->point_cloud, "scan"))
//			vis->addPointCloud (scan_cur->point_cloud, "scan");
//		vis->spin();

		// load points to the grid structure;
		gettimeofday(&start,NULL);
		if(!extract.loadPoints(scan_cur))
		{
			cout<<"no points for PlaneExtraction structure"<<endl;
			return 0;
		}
		gettimeofday(&end,NULL);
		timeused=(1000000*(end.tv_sec-start.tv_sec)+end.tv_usec-start.tv_usec)/1000;
		cout<<"time loading points:"<<timeused<<"ms"<<endl;

		// extract planes;
		gettimeofday(&start,NULL);
		extract.extractPlanes(scan_cur);
		gettimeofday(&end,NULL);
		timeused=(1000000*(end.tv_sec-start.tv_sec)+end.tv_usec-start.tv_usec)/1000;
		cout<<"extracted planes: "<<scan_cur->observed_planes.size()<<std::endl;
		cout<<"time extracting planes:"<<timeused<<"ms"<<endl;
		fp_notes<<"extracted "<<scan_cur->observed_planes.size()<<" planes using "<<timeused<<"ms"<<endl;
		fp_error<<std::endl<<std::fixed<<data_reading->getTime()<<"\t";
//		fp_error<<timeused<<"\t";

//		std::cout<<"after extraction"<<std::endl;
//		for(size_t i=0;i<scan_cur->observed_planes.size();i++)
//		{
//			std::cout<<"plane - "<<i<<std::endl;
//			std::cout<<scan_cur->observed_planes[i]->normal.transpose()<<", "
//					 <<scan_cur->observed_planes[i]->d<<std::endl;
//			std::cout<<scan_cur->observed_planes[i]->points.size()<<std::endl;
//		}

		// estimate plane parameters;
		gettimeofday(&start,NULL);
		for(size_t i=0;i<scan_cur->observed_planes.size();i++)
		{ plane_fitting.estimatePlaneParams(scan_cur->observed_planes[i],cam); }
		gettimeofday(&end,NULL);
		timeused=(1000000*(end.tv_sec-start.tv_sec)+end.tv_usec-start.tv_usec)/1000;
		std::cout<<"time estimating plane parameters:"<<timeused<<std::endl;
//		fp_error<<timeused<<"\t";

		// cout extracted planes;
		for(size_t i=0;i<scan_cur->observed_planes.size();i++)
		{
			std::cout<<"plane - "<<i<<std::endl;
			std::cout<<scan_cur->observed_planes[i]->normal.transpose()<<", "
					 <<scan_cur->observed_planes[i]->d<<std::endl;
			std::cout<<scan_cur->observed_planes[i]->points.size()<<std::endl;
//			std::cout<<"determinant - "<<scan_cur->observed_planes[i]->cov_inv.determinant()<<std::endl;
		}

		fp_notes<<"extracted planes\t(id\tnormal\td\tplane_size)"<<std::endl;
		for(size_t i=0;i<scan_cur->observed_planes.size();i++)
		{
			fp_notes<<"\t"<<i<<"\t";
			fp_notes<<scan_cur->observed_planes[i]->normal.transpose()<<"\t"
					 <<scan_cur->observed_planes[i]->d<<"\t";
			fp_notes<<scan_cur->observed_planes[i]->points.size()<<std::endl;
		}

		// extract edge points;
		gettimeofday(&start,NULL);
		edge_ext.extractEdgePoints(scan_cur);
//		edge_ext.fitLinesHough(scan_cur);
//		for(size_t i=0;i<scan_cur->edge_points.size();i++)
//		{
//			edge_ext.fitSphere(scan_cur->edge_points[i]);
//			fp_notes<<scan_cur->edge_points[i]->meas_Edge<<"\t";
//		}
//		fp_notes<<std::endl;
		gettimeofday(&end,NULL);
		timeused=(1000000*(end.tv_sec-start.tv_sec)+end.tv_usec-start.tv_usec)/1000;
		cout<<"extracted edge points: "<<scan_cur->edge_points.size()<<std::endl;
		cout<<"time extracting edge points:"<<timeused<<"ms"<<endl;
		fp_notes<<"extracted "<<scan_cur->edge_points.size()<<" edge points and "<<scan_cur->edge_points_occluded.size()<<" shadow points using "<<timeused<<"ms"<<endl;

		// extract edge points;
		gettimeofday(&start,NULL);
		line_ext.extractLines(scan_cur,vis);
		gettimeofday(&end,NULL);
		timeused=(1000000*(end.tv_sec-start.tv_sec)+end.tv_usec-start.tv_usec)/1000;
		cout<<"extracted lines: "<<scan_cur->lines_occluding.size()<<std::endl;
		cout<<"time extracting lines:"<<timeused<<"ms"<<endl;
		fp_notes<<"extracted "<<scan_cur->lines_occluding.size()<<" lines and "<<scan_cur->lines_occluded.size()<<" shadow lines using "<<timeused<<"ms"<<endl;

		fp_notes<<"extracted lines"<<endl;
		int count=0;
		for(std::list<Line*>::iterator it_line=scan_cur->lines_occluding.begin();it_line!=scan_cur->lines_occluding.end();it_line++)
		{
			fp_notes<<"\t"<<count++<<"\t"<<(*it_line)->u.transpose()<<"\t"<<(*it_line)->v.transpose()<<endl;
		}
		fp_notes<<"extracted shadow lines"<<endl;
		count=0;
		for(std::list<Line*>::iterator it_line=scan_cur->lines_occluded.begin();it_line!=scan_cur->lines_occluded.end();it_line++)
		{
			fp_notes<<"\t"<<count++<<"\t"<<(*it_line)->u.transpose()<<"\t"<<(*it_line)->v.transpose()<<endl;
		}



//		if(mode==DEBUG)
		if(visual)
		{
//			displayPlanes(scan_cur,vis);
//			vis->spin();
//			displayEdgePoints(scan_cur,vis);
//			vis->spin();
//			displayLines(scan_cur,vis);
//			vis->spin();
		}

		if(filenum>first)
		{
//			scan_cur->scan_ref=scan_ref;
			scan_cur->setRef(scan_ref);
			scans.push_back(scan_cur);

			// plane feature matching;
			gettimeofday(&start,NULL);
			pfm.match(scan_ref->observed_planes, scan_cur->observed_planes, scan_cur->plane_matches);
			gettimeofday(&end,NULL);
			timeused=(1000000*(end.tv_sec-start.tv_sec)+end.tv_usec-start.tv_usec)/1000;
			std::cout<<"time matching plane feature:"<<timeused<<std::endl;
			fp_notes<<"associate "<<scan_cur->plane_matches.size()<<" pairs of planes using "<<timeused<<"ms"<<std::endl;

			// plane feature matching;
			displayScan(scan_cur,vis);
			gettimeofday(&start,NULL);
			lfm.match(scan_ref->lines_occluding, scan_cur->lines_occluding, scan_cur->line_matches,vis);
			gettimeofday(&end,NULL);
			timeused=(1000000*(end.tv_sec-start.tv_sec)+end.tv_usec-start.tv_usec)/1000;
			std::cout<<"time matching line feature:"<<timeused<<std::endl;
			fp_notes<<"associate "<<scan_cur->line_matches.size()<<" pairs of lines using "<<timeused<<"ms"<<std::endl;

			if(visual)
			{
//				displayMatchedLines(scan_cur,vis);
//				vis->spin();
			}

			// cout matched planes;
			std::cout<<"matched planes --- "<<std::endl;
			for(size_t i=0;i<scan_cur->plane_matches.size();i++)
			{
				std::cout<<i<<" --- "<<std::endl;
				std::cout<<"\t<"<<scan_cur->plane_matches[i].cur->id<<","<<scan_cur->plane_matches[i].ref->id<<">"<<std::endl;
				std::cout<<"\t"<<scan_cur->plane_matches[i].cur->normal.transpose()<<", "<<scan_cur->plane_matches[i].cur->d<<std::endl;
				std::cout<<"\t"<<scan_cur->plane_matches[i].ref->normal.transpose()<<", "<<scan_cur->plane_matches[i].ref->d<<std::endl;
			}

			fp_notes<<"matched planes\t(id\t<cur,ref>\tnormal_cur\td_cur\tnormal_ref\td_ref)"<<std::endl;
			for(size_t i=0;i<scan_cur->plane_matches.size();i++)
			{
				fp_notes<<i<<"\t<"<<scan_cur->plane_matches[i].cur->id<<","<<scan_cur->plane_matches[i].ref->id<<">"<<std::endl;
				fp_notes<<"\t"<<scan_cur->plane_matches[i].cur->normal.transpose()<<"\t"<<scan_cur->plane_matches[i].cur->d<<std::endl;
				fp_notes<<"\t"<<scan_cur->plane_matches[i].ref->normal.transpose()<<"\t"<<scan_cur->plane_matches[i].ref->d<<std::endl;
			}

			fp_notes<<"matched lines\t(id\t<cur,ref>\tu_cur\tv_cur\tu_ref\tv_ref)"<<std::endl;
			for(size_t i=0;i<scan_cur->line_matches.size();i++)
			{
				fp_notes<<i<<"\t<"<<scan_cur->line_matches[i].cur->id<<","<<scan_cur->line_matches[i].ref->id<<">"<<std::endl;
				fp_notes<<"\t"<<scan_cur->line_matches[i].cur->u.transpose()<<"\t"<<scan_cur->line_matches[i].cur->v.transpose()<<std::endl;
				fp_notes<<"\t"<<scan_cur->line_matches[i].ref->u.transpose()<<"\t"<<scan_cur->line_matches[i].ref->v.transpose()<<std::endl;
				fp_notes<<"\tvTv="<<scan_cur->line_matches[i].cur->v.transpose()*scan_cur->line_matches[i].ref->v<<std::endl;
				fp_notes<<"\tuTu="<<scan_cur->line_matches[i].cur->u.transpose()*scan_cur->line_matches[i].ref->u<<std::endl;
			}

			int *size=new int[2];
			size=fig_error->getWindowSize();

			Tcr_gt=scan_cur->Tcg_gt*scan_ref->Tcg_gt.inv();
			xi_cr_gt=Tcr_gt.getMotionVector();

//			////////////////////////////////////////////////////////////////////////
//			for(size_t i=0;i<scan_cur->projective_rays.size();i++)
//			{
//				scan_cur->projective_rays[i]->occluded->xyz
//					=scan_cur->projective_rays[i]->occluded_proj->xyz;
//				scan_cur->projective_rays[i]->occluded->xyz
//					=scan_cur->projective_rays[i]->occluded_proj->xyz;
//			}
//			for(size_t i=0;i<scan_ref->projective_rays.size();i++)
//			{
//				scan_ref->projective_rays[i]->occluded->xyz
//					=scan_ref->projective_rays[i]->occluded_proj->xyz;
//				scan_ref->projective_rays[i]->occluded->xyz
//					=scan_ref->projective_rays[i]->occluded_proj->xyz;
//			}

			gl.addScan(scan_cur,map);
			if(visual)
			{
//				displayMap(map,vis);
//				displayLineShadow(scan_cur,map,vis);
//				std::cout<<"map before optimization"<<std::endl;
//				vis->spin();
			}

			////////////////////////////////////////////
			// plane-line-BA
			// align scans;
			gettimeofday(&start,NULL);
			Tcr.setIdentity();
			mel.useShadow(false);
			mel.setDebug(false);
			mel.alignOccludingLines(scan_cur,map,Tcr,vis);
			gettimeofday(&end,NULL);
			timeused=(1000000*(end.tv_sec-start.tv_sec)+end.tv_usec-start.tv_usec)/1000;
			std::cout<<"time aligning occluding lines:"<<timeused<<std::endl;
			fp_notes<<"plane-line-BA in "<<timeused<<" ms"<<std::endl;

			scan_cur->Tcr=Tcr;
			scan_cur->Tcg=scan_cur->Tcr*scan_ref->Tcg; // Tcg=Tcr*Trg;

			xi_cr=Tcr.getMotionVector();
			delta_xi=xi_cr_gt-xi_cr;
			err_pos_edge=delta_xi.block<3,1>(0,0).norm();
			err_ang_edge=delta_xi.block<3,1>(3,0).norm();
			std::cout<<"plane-line-BA\t"<<err_pos_edge<<"\t"<<err_ang_edge<<std::endl;
			fp_notes<<"plane-line-BA"<<std::endl;
			fp_notes<<"\ttranslation error\t"<<err_pos_edge<<std::endl;
			fp_notes<<"\trotation error\t\t"<<err_ang_edge<<std::endl;
			fp_error<<"\t"<<err_pos_edge<<"\t"<<err_ang_edge;
			fig_error->setPenColor(0,0,255,200);
			fig_error->addLine(filenum*2,err_pos_pre_edge*size[0]+0.5*size[1],(filenum+1)*2,err_pos_edge*size[0]+0.5*size[1]);
			fig_error->setPenColor(0,0,255,200);
			fig_error->addLine(filenum*2,err_ang_pre_edge*size[0],(filenum+1)*2,err_ang_edge*size[0]);
			err_pos_pre_edge=err_pos_edge;
			err_ang_pre_edge=err_ang_edge;
			//fig_error->spinOnce();

			std::cout<<"plane-line-BA Tcr "<<std::endl<<Tcr.getMatrix4f()<<std::endl;

			////////////////////////////////////////////
			// plane-line-shadow-BA
			// align scans;
			gettimeofday(&start,NULL);
			Tcr.setIdentity();
			mel.useShadow(true);
			mel.setDebug(true);
			mel.alignOccludingLines(scan_cur,map,Tcr,vis);
			gettimeofday(&end,NULL);
			timeused=(1000000*(end.tv_sec-start.tv_sec)+end.tv_usec-start.tv_usec)/1000;
			std::cout<<"time aligning occluding and occluded lines:"<<timeused<<std::endl;
			fp_notes<<"plane-line-shadow-BA in "<<timeused<<" ms"<<std::endl;

//			scan_cur->Tcr=Tcr;
//			scan_cur->Tcg=scan_cur->Tcr*scan_ref->Tcg; // Tcg=Tcr*Trg;

			xi_cr=Tcr.getMotionVector();
			delta_xi=xi_cr_gt-xi_cr;
			err_pos_shadow=delta_xi.block<3,1>(0,0).norm();
			err_ang_shadow=delta_xi.block<3,1>(3,0).norm();
			std::cout<<"plane-line-shadow-BA\t"<<err_pos_shadow<<"\t"<<err_ang_shadow<<std::endl;
			fp_notes<<"plane-line-shadow-BA"<<std::endl;
			fp_notes<<"\ttranslation error\t"<<err_pos_shadow<<std::endl;
			fp_notes<<"\trotation error\t\t"<<err_ang_shadow<<std::endl;
			fp_error<<"\t"<<err_pos_shadow<<"\t"<<err_ang_shadow;
			fig_error->setPenColor(255,0,0,200);
			fig_error->addLine(filenum*2,err_pos_pre_shadow*size[0]+0.5*size[1],(filenum+1)*2,err_pos_shadow*size[0]+0.5*size[1]);
			fig_error->setPenColor(255,0,0,200);
			fig_error->addLine(filenum*2,err_ang_pre_shadow*size[0],(filenum+1)*2,err_ang_shadow*size[0]);
			err_pos_pre_shadow=err_pos_shadow;
			err_ang_pre_shadow=err_ang_shadow;
			fig_error->spinOnce();

			std::cout<<"plane-line-shadow-BA Tcr "<<std::endl<<Tcr.getMatrix4f()<<std::endl;

//			if(mode==DEBUG)
			if(visual)
			{
//				displayMap(map,vis);
//				std::cout<<"map after optimization"<<std::endl;
//				vis->spin();
				displayScans_2scans(scan_cur,scan_ref,ulysses::Transform(),vis);
//				displayMatchedEdgePoints(scan_cur,ulysses::Transform(),vis);
				std::cout<<"initial pose"<<std::endl;
				vis->spin();
				displayScans_2scans(scan_cur,scan_ref,scan_cur->Tcr,vis);
//				displayMatchedEdgePoints(scan_cur,scan_cur->Tcr_edge,vis);
				std::cout<<"optimized pose"<<std::endl;
				vis->spin();
			}

//			fp_error<<std::endl;


			delete scan_ref;
		}
		else
		{
			scan_cur->Tcg.setIdentity();
//			scans.push_back(scan_cur);
//			scan_cur->Tcg_edge.setIdentity();
		}

		if(mode==VIS_SCAN)
		{
			display_addScan(scan_cur,vis);
			display_addCameraPose(scan_cur,vis);
			if(filenum%vis_every_n_frames==0)
			{
				vis->spin();
				vis->removeAllPointClouds();
			}
		}

		Tgc=scan_cur->Tcg.inv();
		fp<<std::fixed<<data_reading->getTime()<<" "<<Tgc.t.transpose()<<" "<<Tgc.Quaternion().transpose()<<std::endl;
//		Tgc=scan_cur->Tcg_edge.inv();
//		fp_edge<<std::fixed<<data_reading->getTime()<<" "<<Tgc.t.transpose()<<" "<<Tgc.Quaternion().transpose()<<std::endl;

		scan_ref=scan_cur;
		filenum++;
	}

//	fp<<std::endl<<"plane landmarks in the map"<<std::endl;
//	for(size_t i=0;i<map->planes.size();i++)
//	{
//		fp<<map->planes[i]->id<<" "<<map->planes[i]->n.transpose()<<" "<<map->planes[i]->d<<std::endl;
//	}
	fp.close();
	fp_edge.close();
	fp_shadow.close();
	fp_error.close();
	fp_residual.close();
	return 0;
}
*/
