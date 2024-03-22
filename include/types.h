/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2019-05-08 20:10
#
# Filename:		types.h
#
# Description: 
#
===============================================*/

#ifndef _TYPES_H_
#define _TYPES_H_

#include <stdio.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <math.h>
#include <cmath>
#include <list>
#include <limits>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Eigenvalues>
#include <eigen3/Eigen/LU>

#include <pcl-1.8/pcl/point_types.h>
#include <pcl-1.8/pcl/point_cloud.h>
#include <pcl-1.8/pcl/filters/extract_indices.h>
#include <pcl-1.8/pcl/segmentation/planar_region.h>
#include <pcl-1.8/pcl/common/eigen.h>
#include <pcl-1.8/pcl/registration/transforms.h>
#include <pcl-1.8/pcl/sample_consensus/sac_model_line.h>
#include <pcl-1.8/pcl/sample_consensus/ransac.h>
#include <pcl-1.8/pcl/visualization/pcl_visualizer.h>
#include <pcl-1.8/pcl/visualization/pcl_painter2D.h>
#include <pcl-1.8/pcl/features/organized_edge_detection.h>
#include <pcl-1.8/pcl/segmentation/rgb_plane_coefficient_comparator.h>
#include <pcl-1.8/pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl-1.8/pcl/common/centroid.h>
#include <pcl-1.8/pcl/ModelCoefficients.h>
#include <pcl-1.8/pcl/sample_consensus/method_types.h>
#include <pcl-1.8/pcl/sample_consensus/model_types.h>
#include <pcl-1.8/pcl/segmentation/sac_segmentation.h>
#include <pcl-1.8/pcl/features/integral_image_normal.h>
#include <pcl-1.8/pcl/features/normal_3d.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>  
//#include <g2o/types/sba/types_six_dof_expmap.h>

namespace ulysses
{
	typedef Eigen::Matrix<double,6,1> Vector6d;
	typedef Eigen::Matrix<double,6,6> Matrix6d;

	// IntrinsicParam 
	struct IntrinsicParam
	{
		IntrinsicParam() // {fx=525;fy=525;cx=240;cy=320;}
		{
			fx=517.3;//591.1;//567.6;//580.8;//525;
			fy=516.5;//590.1;//570.2;//581.8;//525;
			cx=318.6;//331;//324.7;//308.8;//319.5;
			cy=255.3;//234;//250.1;//253;//239.5;
			m_fp=2.85e-3; //m^-1
			sigma_disparity=0.5; //pixel
			sigma_u=0.5; //pixel
			sigma_v=0.5; //pixel
			factor=5000.0;
			width=640;
			height=480;
		}
		IntrinsicParam(double fx_, double fy_, double cx_, double cy_) {fx=fx_;fy=fy_;cx=cx_;cy=cy_;}
//		IntrinsicParam(const IntrinsicParam int_param) {fx=int_param.fx;fy=int_param.fy;cx=imt_param.cx;cy=int_param.cy;}

		double fx,fy,cx,cy;
		double m_fp;
		double sigma_disparity, sigma_u, sigma_v;
		double factor;
		int width, height;

		// getMatrix
		// - K =
		// - |fx 0  cx|
		//   |0  fy cy|
		//   |0  0  1 |
		Eigen::Matrix3d getMatrix()
		{
			Eigen::Matrix3d mat;
			mat.setIdentity();
			mat(0,0)=fx;
			mat(1,1)=fy;
			mat(0,2)=cx;
			mat(1,2)=cy;
			return mat;
		}

		// project
		// - project the point in 3d camera frame into the pixel frame;
		// - u=Kp;
		// - u is the homogeneous coordinates;
		// - u=[col,row,1]^T;
		Eigen::Vector3d project(Eigen::Vector3d p)
		{
			Eigen::Vector3d tmp,u;
			tmp=getMatrix()*p/p(2);
			u(0)=tmp(0);
			u(1)=tmp(1);
			u(2)=1;
			//u(0) = p[0]*fx/p[2] + cx;
			//u(1) = p[1]*fy/p[2] + cy;
			return u;
		}
	};

	struct Plane;

	struct Point
	{
		// xyz in meters;
		Eigen::Vector3d xyz, normal;

		// coordinate in the PPS for the local plane parameters (after Rotation_PCA);
		Eigen::Vector3d pps;
		void Cartesian2PPS(Eigen::Matrix3d Rotation_PCA=Eigen::Matrix3d::Identity())
		{
			Eigen::Vector3d tmp_vec3d;
			tmp_vec3d=Rotation_PCA*normal;
			if(tmp_vec3d(2)>1.0-1e-20)
			{
				tmp_vec3d(2)=1.0;
			}
			if(tmp_vec3d(2)<-1.0+1e-20)
			{
				tmp_vec3d(2)=-1.0;
			}
			pps(0)=acos(tmp_vec3d(2));
//			if(pps(0)>M_PI)
//			{
//				pps(0)=M_PI*2-pps(0);
//			}
			pps(1)=atan2(tmp_vec3d(1),tmp_vec3d(0));
			pps(2)=-normal.dot(xyz);
		}

		// rgb \in [0,1]^3;
		Eigen::Vector3d rgb;

		// u,v: pixel coordinate;
		int u,v; // u<480, v<640

		// cov of the point;
		// measurement uncertainty;
		Eigen::Matrix3d cov, cov_inv; 
		void compute_cov(IntrinsicParam& cam)
		{
			double sigma_d=cam.m_fp*cam.sigma_disparity*xyz(2)*xyz(2); //m
			double d_fu=xyz(2)/cam.fx;
			double d_fv=xyz(2)/cam.fy;
			double x_d=xyz(0)/xyz(2);
			double y_d=xyz(1)/xyz(2);
			cov(0,0)=d_fu*d_fu*cam.sigma_u*cam.sigma_u+x_d*x_d*sigma_d*sigma_d;
			cov(0,1)=x_d*y_d*sigma_d*sigma_d;
			cov(0,2)=x_d*sigma_d*sigma_d;
			cov(1,0)=cov(0,1);
			cov(1,1)=d_fv*d_fv*cam.sigma_v*cam.sigma_v+y_d*y_d*sigma_d*sigma_d;
			cov(1,2)=y_d*sigma_d*sigma_d;
			cov(2,0)=cov(0,2);
			cov(2,1)=cov(1,2);
			cov(2,2)=sigma_d*sigma_d;
			cov_inv=cov.inverse();
		}

		// weight in the plane fitting;
		// weight = pln_n^T * cov_inv * pln_n;
		// weight_angle = [p_pi/(pln_n^T*p_pi)]^T * cov_inv * [p_pi/(pln_n^T*p_pi)];
		double weight;

	};

	struct Transform
	{
		Transform()
		{
			R=Eigen::Matrix3d::Identity();
			t=Eigen::Vector3d::Zero();
		}

		Transform(const Transform& T)
		{
			R=T.R;
			t=T.t;
			time_stamp=T.time_stamp;
		}

		Transform(Eigen::Matrix3d R_, Eigen::Vector3d t_)
		{
			R=R_;
			t=t_;
		}

		Transform(Eigen::Quaterniond Q_, Eigen::Vector3d t_)
		{
			R=Q_.toRotationMatrix();
			t=t_;
		}

//		Transform(g2o::SE3Quat Tcr)
//		{
//			R=Tcr.rotation().toRotationMatrix();
//			t=Tcr.translation();
//		}

		Eigen::Vector3d t;
		Eigen::Matrix3d R;
		double time_stamp;

//		std::vector<int> indices_planes;
//		std::vector<int> indices_lines;

//		IntrinsicParam intrinsic;

//		std::vector<Plane*> ptr_planes;

		// eular angle: Z-Y-X
		Eigen::Vector3d Eulars() {return R.eulerAngles(2,1,0);}

		const Eigen::Quaterniond Quat() {return Eigen::Quaterniond(R);}
		Eigen::Vector4d Quaternion()
		{
			Eigen::Quaterniond q=Quat();
			Eigen::Vector4d vec;
			vec.block<3,1>(0,0)=q.vec();
			vec(3)=q.w();
			return vec;
		}

		static const Eigen::Matrix3d skew_sym(Eigen::Vector3d u)
		{
			Eigen::Matrix3d U;
			U.setZero();
			U(0,1)=-u(2);
			U(0,2)=u(1);
			U(1,0)=u(2);
			U(1,2)=-u(0);
			U(2,0)=-u(1);
			U(2,1)=u(0);
			return U;
		}

		static const Eigen::Vector3d skew_sym_inv(Eigen::Matrix3d U)
		{
			Eigen::Vector3d u;
			u(1)=U(0,2);
			u(2)=U(1,0);
			u(0)=U(2,1);
			return u;
		}

		void inverse()
		{
			Eigen::Matrix3d R_tmp;
			Eigen::Vector3d t_tmp;
			R_tmp=R.transpose();
			t_tmp=-R.transpose()*t;
			R=R_tmp;
			t=t_tmp;
		}

		Transform inv()
		{
			Eigen::Matrix3d R_tmp;
			Eigen::Vector3d t_tmp;
			R_tmp=R.transpose();
			t_tmp=-R.transpose()*t;
			Transform T(R_tmp,t_tmp);
			T.time_stamp=time_stamp;
			return T;
		}

		void setIdentity()
		{
			R=Eigen::Matrix3d::Identity();
			t=Eigen::Vector3d::Zero();
		}

		Transform operator* (const Transform& tr2) const
		{
			Transform result;
			result.t=t+R*tr2.t;
			result.R=R*tr2.R;
			return result;
		}

		void leftMultiply(Eigen::Isometry3d T)
		{
			Eigen::Matrix3d R_tmp;
			Eigen::Vector3d t_tmp;
			R_tmp=T.rotation()*R;
			t_tmp=T.rotation()*t+T.translation();
			R=R_tmp;
			t=t_tmp;
		}

		Eigen::Vector3d transformPoint(Eigen::Vector3d p)
		{
			return R*p+t;
		}

		Eigen::Vector4d transformPlane(Eigen::Vector4d pln)
		{
			Eigen::Vector4d pln_trans;
			pln_trans.block<3,1>(0,0)=R*pln.block<3,1>(0,0);
			pln_trans(3)=pln(3)-pln_trans.block<3,1>(0,0).transpose()*t;
			return pln_trans;
		}

//		void transformPlane(Plane *pln, Plane *out)
//		{
//			out->normal=R*pln->normal;
//			out->d=pln->d-t.transpose()*R*pln->normal;
//			return out;
//		}

		Eigen::Matrix4f getMatrix4f()
		{
			Eigen::Matrix4f transform;
			transform.setIdentity();
			for(int i=0;i<3;i++)
			{
				transform(i,3)=(float)t(i);
				for(int j=0;j<3;j++)
				{
					transform(i,j)=(float)R(i,j);
				}
			}
			return transform;
		}

		Eigen::Matrix<double,6,1> getMotionVector()
		{
			Eigen::Matrix<double,6,1> xi;
			xi.block<3,1>(0,0)=t;
			double theta=1/cos((R.trace()-1.0)/2.0);
			Eigen::Vector3d w;
			if(theta<1e-7)
			{
				w=Eigen::Vector3d::Zero();
			}
			else
			{
				w(0)=R(2,1)-R(1,2);
				w(1)=R(0,2)-R(2,0);
				w(2)=R(1,0)-R(0,1);
				w=w/(2*sin(theta));
			}
			xi.block<3,1>(3,0)=w;
			return xi;
		}

		const Eigen::Matrix4d getMatrix() const
		{
			Eigen::Matrix4d T;
			T.setIdentity();
			T.block<3,3>(0,0)=R;
			T.block<3,1>(0,3)=t;
			return T;
		}

		const Eigen::Matrix4d getPlaneTransform()
		{
			Eigen::Matrix4d T_pi=inv().getMatrix();
			return T_pi.transpose();
		}

		const Matrix6d getLineTransform() const
		{
			Matrix6d T_L;
			T_L.setZero();
			T_L.block<3,3>(0,0)=R;
			T_L.block<3,3>(3,3)=R;
			T_L.block<3,3>(0,3)=skew_sym(t)*R;
			return T_L;
		}

		static const Eigen::Matrix4d PluckerCoords2Mat(Vector6d l)
		{
			Eigen::Vector3d u=l.block<3,1>(0,0);
			Eigen::Vector3d v=l.block<3,1>(3,0);
			Eigen::Matrix4d L=Eigen::Matrix4d::Zero();
			L.block<3,3>(0,0)=skew_sym(u);
			L.block<3,1>(0,3)=v;
			L.block<1,3>(3,0)=-v.transpose();
			return L;
		}

		static const Eigen::Matrix4d PluckerCoords2DualMat(Vector6d l)
		{
			Eigen::Vector3d u=l.block<3,1>(0,0);
			Eigen::Vector3d v=l.block<3,1>(3,0);
			Eigen::Matrix4d L=Eigen::Matrix4d::Zero();
			L.block<3,3>(0,0)=skew_sym(v);
			L.block<3,1>(0,3)=u;
			L.block<1,3>(3,0)=-u.transpose();
			return L;
		}

		static const Vector6d PluckerMat2Coords(Eigen::Matrix4d L) 
		{
			Vector6d l;
			l.block<3,1>(0,0)=skew_sym_inv(L.block<3,3>(0,0));
			l.block<3,1>(3,0)=L.block<3,1>(0,3);
			return l;
		}

		static const Vector6d PluckerDualMat2Coords(Eigen::Matrix4d L)
		{
			Vector6d l;
			l.block<3,1>(3,0)=skew_sym_inv(L.block<3,3>(0,0));
			l.block<3,1>(0,0)=L.block<3,1>(0,3);
			return l;
		}

	};

	struct EdgePoint
	{
		EdgePoint() 
		{
			isEdge=false;
			occluding=0;
			occluded=0;
			plane=0;
			onLine=false;
		}
		~EdgePoint()
		{
			std::vector<EdgePoint*>().swap(neighbors);
		}

		Eigen::Vector3d xyz;
		Eigen::Vector2d pixel;
		bool isEdge;
		
		Eigen::Matrix3d cov;
		std::vector<EdgePoint*> neighbors;
		double meas_Edge;
		Eigen::Vector3d dir;
		Eigen::Vector3i rgb;
		
		// if this is an occluded point;
		EdgePoint *occluding;
		EdgePoint *occluded;
		Plane *plane;

		bool onLine;
		// scan->point_cloud->at(index);
		int index;
	};

	struct EdgePointPair
	{
		EdgePointPair() 
		{
			weight=1;
			occluded_match=false;
		}

		EdgePointPair(EdgePoint *c, EdgePoint *r)
		{
			cur=c;
			ref=r;
			weight=1;
			occluded_match=false;
		}

		EdgePoint *cur;
		EdgePoint *ref;
		
		// if occluded_match==true
		// then
		// cur->occluded and ref->occluded are matched occluded edge-point;
		bool occluded_match;

		double weight;
		// gradient = {\partial Jpk}/{\partial \xi};
		Eigen::Matrix<double,6,1> gradient;
		Eigen::Matrix<double,6,6> Psi_pk;
		double sq_lambda_pk;
		Eigen::Matrix<double,6,1> v_pk;

		Eigen::Matrix3d computeCov(Transform Tcr)
		{
			return Tcr.R*ref->cov*Tcr.R.transpose()+cur->cov;
//			return cur->cov;
		}

		double pointDist(Transform Tcr)
		{
			Eigen::Vector3d delta=cur->xyz-Tcr.transformPoint(ref->xyz);
			Eigen::Matrix<double,1,1> dist=delta.transpose()*computeCov(Tcr).inverse()*delta;
			return dist(0,0);
		}
	};

	struct Plane
	{
		Plane() {idx_plane=-1;}

		Eigen::Vector3d normal;
		double d;

//		PlaneLM *plane_landmark;

		// cov_inv = hessian
		//         = | H_nn H_nd |
		//           | H_dn H_dd |
		Eigen::Matrix4d cov_inv; // Hessian matrix;
		Eigen::Matrix4d sqrt_cov_inv;
		Eigen::Matrix4d cov;

		Eigen::Vector3d pG;
		int idx_plane;

		Eigen::Vector3d centroid;
		Eigen::Matrix3d scatter_matrix;

		std::vector<Point> points;
		pcl::PointIndices inlier_indices;

		// avg_rgb, cov_rgb: computed from the points
		Eigen::Vector3d avg_rgb,avg_xyz;
		Eigen::Matrix3d cov_rgb,cov_xyz;
		Eigen::Vector3d cov_eigenvalues; // increasing order;
		Eigen::Matrix3d cov_eigenvectors;

		int id;
		
		// if this is a plane in the ref. frame, 
		// then pln_pointer points to the corresponding plane in the cur. frame;
		// if no correspondence, then pln_pointer=0;
		Plane *pln_pointer;

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		~Plane()
		{
			std::vector<Point>().swap(points);
		}

		double similarity_color(Plane *p)
		{
			// color similarity;
			// Bhattachryya distance bwteen the color distribution of two planes;
			Eigen::Matrix3d C=(cov_rgb+p->cov_rgb)/2;
			Eigen::Vector3d m=avg_rgb-p->avg_rgb;
			Eigen::Matrix<double,1,1> M_dist=m.transpose()*C.inverse()*m/8;
			double s_col=M_dist(0,0)+0.5*log(C.determinant()/sqrt(cov_rgb.determinant()*p->cov_rgb.determinant()));
			return s_col;
		}

		double similarity_angle(Plane *p)
		{
			double cos_angle=normal.transpose()*p->normal;
			if(cos_angle>0.9999)
				cos_angle=0.9999;
			double angle=acos(cos_angle);
			return angle;
		}

		double similarity_dist(Plane *p)
		{
			double dist=fabs(d-p->d);
			return dist;
		}

		double point_plane_dist(Eigen::Vector3d p)
		{
			double dist=fabs(normal.transpose()*p+d);
			return dist;
		}

		Eigen::Vector3d projected_point(Eigen::Vector3d pt)
		{
			return -pt*d/(normal.transpose()*pt);
		}

		Eigen::Vector3d projected_point(Eigen::Vector3d p, Eigen::Vector3d t)
		{
			Eigen::Vector3d n=normal;
			Eigen::Matrix<double,1,1> tmp_nt=n.transpose()*t;
			Eigen::Matrix<double,1,1> tmp_np=n.transpose()*p;
			Eigen::Matrix<double,1,1> tmp_npt=n.transpose()*(p-t);
			double mu1=-(tmp_nt(0,0)+d)/tmp_npt(0,0);
			double mu2=(tmp_np(0,0)+d)/tmp_npt(0,0);
			Eigen::Vector3d pp=mu1*p+mu2*t;
			return pp;
		}

	};

	struct PlaneLM
	{
		PlaneLM(Plane *plane, Transform Tgc)
		{
			Eigen::Vector4d pln;
			pln.block<3,1>(0,0)=plane->normal;
			pln(3)=plane->d;
			pi=Tgc.getPlaneTransform()*pln;
			pi.normalize();
			pG=plane->pG;
		}

		~PlaneLM()
		{
			std::vector<int>().swap(indices_cameras);
		}

		Eigen::Vector4d pi;
		Eigen::Vector3d pG;
		int id;
		
		std::vector<int> indices_cameras;

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	};


	struct PlanePair
	{
		PlanePair(){}

		PlanePair(Plane *r, Plane *c)
		{
			ref=r;
			cur=c;
		}

		Plane *ref;
		Plane *cur;

		Eigen::Matrix4d computeCov(Transform Tcr)
		{
			Eigen::Matrix4d cov;
			Eigen::Matrix4d trans;
			trans.setZero();
			trans.block<3,3>(0,0)=-Tcr.R;
			trans.block<1,3>(3,0)=Tcr.t.transpose()*Tcr.R;
			trans(3,3)=-1;
			cov=trans*ref->cov*trans.transpose();
			//cov=cov+cur->cov;
			return cov;
		}

		double planeDist(Transform Tcr)
		{
			Eigen::Vector4d pln_cur, pln_ref;
			pln_cur.block<3,1>(0,0)=cur->normal;
			pln_cur(3)=cur->d;
			pln_ref.block<3,1>(0,0)=ref->normal;
			pln_ref(3)=ref->d;
			pln_ref=Tcr.transformPlane(pln_ref);
			Eigen::Matrix<double,1,1> dist;
			pln_cur=pln_cur-pln_ref;
//			dist=pln_cur.transpose()*computeCov(Tcr).inverse()*pln_cur;
			dist=pln_cur.transpose()*cur->cov_inv*pln_cur;
			return dist(0,0);

		}
	};

	struct Line
	{
		Line()
		{
			occluded=0;
			plane=0;
			idx_line=-1;
		}

		~Line()
		{
			std::vector<EdgePoint*>().swap(points_tmp);
			std::list<EdgePoint*>().swap(points);
		}

		std::vector<EdgePoint*> points_tmp; // for index;
		std::list<EdgePoint*> points;
		Eigen::Vector3d end_point_1, end_point_2;
		Eigen::Vector3d dir;
		double length;

		// Plucker coordinates;
		// u - normal to the plane joining the line and the origin;
		// v - line direction;
		Eigen::Vector3d u,v;
		Matrix6d sqrt_cov_inv;
		Matrix6d cov_inv;
		Matrix6d cov;

		Eigen::Vector3d pG;
		int idx_line; // index of the line observation to the map line;

		int id;

		Line *occluded;
		Plane *plane;

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		// return the angle between two lines in degrees;
		double similarity_dir(Line *l)
		{
			Eigen::Vector3d dir_this = v/v.norm();
			Eigen::Vector3d dir_l    = l->v/l->v.norm();
			double cos_angle=fabs(dir_this.transpose()*dir_l);
			if(cos_angle>0.9999)
				cos_angle=0.9999;
			double angle=acos(cos_angle)*180.0/M_PI;
			return angle;
		}

		double similarity_normal(Line *l)
		{
			Eigen::Vector3d dir_this = u/u.norm();
			Eigen::Vector3d dir_l    = l->u/l->u.norm();
			double cos_angle=fabs(dir_this.transpose()*dir_l);
			if(cos_angle>0.9999)
				cos_angle=0.9999;
			double angle=acos(cos_angle)*180.0/M_PI;
			return angle;
		}

		double similarity_dist(Line *l)
		{
			double d_this = u.norm();
			double d_l    = l->u.norm();
			double dist = fabs(d_this-d_l);
			return dist;
		}
	};

	struct LinePair
	{
		LinePair(){}

		LinePair(Line *r, Line *c)
		{
			ref=r;
			cur=c;
		}

		Line *ref;
		Line *cur;
	};

	struct LineLM
	{
		LineLM(Line *line, Transform Tgc)
		{
			Vector6d ln;
			ln.block<3,1>(0,0)=line->u;
			ln.block<3,1>(3,0)=line->v;
			L=Tgc.getLineTransform()*ln;
			pG=line->pG;
		}

		~LineLM()
		{
			std::vector<int>().swap(indices_cameras);
		}

		// L=[d*\hat{u},\hat{v}];
		// d - vertical distance from the origin to the line;
		// \hat{u} - normalized u;
		// \hat{v} - normalized v;
		Vector6d L; 
		Eigen::Vector3d pG;
		int id;

		std::vector<int> indices_cameras;

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	struct Scan
	{
		Scan(const int &id_cur, const IntrinsicParam &camera, const double &time)
		{
			cam=camera;
			id=id_cur;
			time_stamp=time;
			Tcr.setIdentity();
			Tcg.setIdentity();
		}

		void setRef(Scan* ref)
		{
			scan_ref=ref;
		}

		Scan *scan_ref;
		IntrinsicParam cam;
		double time_stamp;
		int id;

		// transform from global frame to current frame;
		Transform Tcg; 
		Transform Tcr;
		Transform Tcw;

		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr point_cloud;
		pcl::PointCloud<pcl::Normal>::Ptr normal_cloud;
		pcl::PointCloud<pcl::PointXY>::Ptr pixel_cloud;
		cv::Mat img_rgb, img_depth;
		
		Eigen::Matrix3d Rotation_PCA;

		// plane features;
		std::vector<Plane*> observed_planes;
		std::vector<PlanePair> plane_matches;

		// edge points;
		std::vector<EdgePoint*> edge_points;
		std::vector<EdgePoint*> edge_points_occluded;
		std::vector<EdgePointPair> point_matches;

		std::list<Line*> lines_occluding;
		std::list<Line*> lines_occluded;
		std::vector<LinePair> line_matches;

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		~Scan()
		{
			img_rgb.release();
			img_depth.release();

			for(size_t i=0;i<observed_planes.size();i++) delete observed_planes[i];
			for(size_t i=0;i<edge_points.size();i++) delete edge_points[i];
			for(size_t i=0;i<edge_points_occluded.size();i++) delete edge_points_occluded[i];
			for(std::list<Line*>::iterator it=lines_occluding.begin();it!=lines_occluding.end();it++) delete *it;
			for(std::list<Line*>::iterator it=lines_occluded.begin();it!=lines_occluded.end();it++) delete *it;

//			std::vector<Plane*> tmp_pln;
//			std::vector<PlanePair> tmp_pln_pair;
//			std::vector<EdgePoint*> tmp_pt;
//			std::vector<EdgePointPair> tmp_pt_pair;
//			std::list<Line*> tmp_ln;
//			std::vector<LinePair> tmp_ln_pair;
//
//			tmp_pln.swap(observed_planes); 
//			tmp_pln_pair.swap(plane_matches);
//			tmp_pt.swap(edge_points); 
//			tmp_pt.swap(edge_points_occluded); 
//			tmp_pt_pair.swap(point_matches);
//			tmp_ln.swap(lines_occluding); 
//			tmp_ln.swap(lines_occluded); 
//			tmp_ln_pair.swap(line_matches);

			std::vector<Plane*>().swap(observed_planes); 
			std::vector<PlanePair>().swap(plane_matches);
			std::vector<EdgePoint*>().swap(edge_points); 
			std::vector<EdgePoint*>().swap(edge_points_occluded); 
			std::vector<EdgePointPair>().swap(point_matches);
			std::list<Line*>().swap(lines_occluding); 
			std::list<Line*>().swap(lines_occluded); 
			std::vector<LinePair>().swap(line_matches);
		}

	};

	struct Indices
	{
		std::vector<int> indices;

		int size() { return indices.size(); }

		void push_back(int i)
		{
			indices.push_back(i);
		}

		int operator[](int i) const 
		{
			return indices[i];
		}

		~Indices()
		{
			std::vector<int>().swap(indices);
		}
	};

	struct Map
	{
		std::vector<double> timestamps;
		std::vector<Transform> cameras;
		std::vector<PlaneLM*> planes;
		std::vector<LineLM*> lines;

		void addCamera(Transform Tcg, double timestamp)
		{
			cameras.push_back(Tcg);
			indices_planes.resize(cameras.size());
			indices_lines.resize(cameras.size());
			timestamps.push_back(timestamp);
		}

		// index of the cameras 
		// to the planes and lines;
		std::vector<Indices> indices_planes;
		std::vector<Indices> indices_lines;

		struct observation_plane
		{
			Eigen::Vector4d pi; // pi=[n,d];
			Eigen::Matrix4d sqrt_info;
			int idx_camera;
			int idx_plane;
			Plane *plane_ptr;
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		};
		std::vector<observation_plane*> observ_planes;
		void addObservationPlane(Plane *p, int idx_camera, int idx_plane)
		{
			observation_plane *plane=new observation_plane;
			plane->pi.block<3,1>(0,0)=p->normal;
			plane->pi(3)=p->d;
			plane->sqrt_info=p->sqrt_cov_inv;
			plane->idx_camera=idx_camera;
			plane->idx_plane=idx_plane;
			plane->plane_ptr=p;
			observ_planes.push_back(plane);
			indices_planes[idx_camera].push_back(idx_plane);
		}

		struct observation_line
		{
			Vector6d L;
			Matrix6d sqrt_info;
			int idx_camera;
			int idx_line;
			Line *line_ptr;
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		};
		std::vector<observation_line*> observ_lines;
		void addObservationLine(Line *l, int idx_camera, int idx_line)
		{
			observation_line *line=new observation_line;
			line->L.block<3,1>(0,0)=l->u;
			line->L.block<3,1>(3,0)=l->v;
			line->sqrt_info=l->sqrt_cov_inv;
			line->idx_camera=idx_camera;
			line->idx_line=idx_line;
			line->line_ptr=l;

			Vector6d l_cur=cameras[idx_camera].getLineTransform()*lines[idx_line]->L;
			double tmp=l_cur.block<3,1>(0,0).transpose()*l->u;
			if(tmp<0) line->L=-line->L;

			observ_lines.push_back(line);
			indices_lines[idx_camera].push_back(idx_line);
		}

		struct observation_shadow
		{
			Vector6d L;
			Matrix6d sqrt_info;
			Matrix6d cov;
			int idx_camera;
			int idx_plane;
			int idx_line;
			//Line *line_ptr;
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		};
		std::vector<observation_shadow*> observ_shadows;
		//void addObservationShadow(Line *l, int idx_camera, int idx_plane, int idx_line)
		//{
		//	observation_shadow *line=new observation_shadow;
		//	line->L.block<3,1>(0,0)=l->u;
		//	line->L.block<3,1>(3,0)=l->v;
		//	line->sqrt_info=l->sqrt_cov_inv;
		//	line->idx_camera=idx_camera;
		//	line->idx_plane=idx_plane;
		//	line->idx_line=idx_line;
		//	line->line_ptr=l;
		//	observ_shadows.push_back(line);
		//}
		void addObservationShadow(Line *l, Plane *p, int idx_camera)
		{
			observation_shadow *line=new observation_shadow;

			Eigen::Vector3d u=l->u;
			Eigen::Vector3d n=p->normal;
			double d=p->d;

			line->L.block<3,1>(0,0)=d*u;
			line->L.block<3,1>(3,0)=-u.cross(n);
			line->L/=line->L.block<3,1>(3,0).norm();

			Vector6d l_cur=cameras[idx_camera].getLineTransform()*lines[l->idx_line]->L;
			double tmp=l_cur.block<3,1>(0,0).transpose()*l->u;
			if(tmp<0) line->L=-line->L;

			Eigen::Matrix3d Cuu=l->cov.block<3,3>(0,0);
			Eigen::Matrix3d Cnn=p->cov.block<3,3>(0,0);
			Eigen::Vector3d Cnd=p->cov.block<3,1>(0,3);
			double Cdd=p->cov(3,3);

			line->cov.block<3,3>(0,0)=d*d*Cuu+Cdd*u*u.transpose();
			line->cov.block<3,3>(0,3)=d*Cuu*Transform::skew_sym(n).transpose()
									  +u*Cnd.transpose()*Transform::skew_sym(u);
			line->cov.block<3,3>(3,0)=line->cov.block<3,3>(0,3).transpose();
//			line->cov.block<3,3>(3,0)=d*Transform::skew_sym(n)*Cuu
//									  +Transform::skew_sym(u).transpose()*Cnd*u.transpose();
			line->cov.block<3,3>(3,3)=Transform::skew_sym(n)*Cuu*Transform::skew_sym(n).transpose()
									  +Transform::skew_sym(u)*Cnn*Transform::skew_sym(u).transpose();

			Eigen::SelfAdjointEigenSolver<Matrix6d> es;
			es.compute(line->cov);
			Vector6d Lambda=es.eigenvalues();
			Matrix6d U=es.eigenvectors();
			Matrix6d sqrt_inv_Lambda=Matrix6d::Zero();
			for(size_t i=0;i<6;i++)
			{
				if(Lambda(i)>0.01)
				{
					sqrt_inv_Lambda(i,i)=1.0/sqrt(Lambda(i));
				}
			}
			line->sqrt_info=U*sqrt_inv_Lambda*U.transpose();

			line->idx_camera=idx_camera;
			line->idx_plane=p->idx_plane;
			line->idx_line=l->idx_line;
			observ_shadows.push_back(line);
			
		}

		~Map()
		{
			for(int i=0;i<planes.size();i++) delete planes[i];
			for(int i=0;i<lines.size();i++) delete lines[i];
			for(int i=0;i<observ_planes.size();i++) delete observ_planes[i];
			for(int i=0;i<observ_lines.size();i++) delete observ_lines[i];
			for(int i=0;i<observ_shadows.size();i++) delete observ_shadows[i];

			std::vector<double>().swap(timestamps);
			std::vector<Transform>().swap(cameras);
			std::vector<PlaneLM*>().swap(planes);
			std::vector<LineLM*>().swap(lines);
			std::vector<Indices>().swap(indices_planes);
			std::vector<Indices>().swap(indices_lines);
			std::vector<observation_plane*>().swap(observ_planes);
			std::vector<observation_line*>().swap(observ_lines);
			std::vector<observation_shadow*>().swap(observ_shadows);
		}

	};

}

#endif
