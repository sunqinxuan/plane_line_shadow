/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@gmail.com
#
# Last modified: 2019-05-10 10:05
#
# Filename: relative_pose_error.cpp
#
# Description: 
#
************************************************/

#include "relative_pose_error.h"

namespace ulysses
{
	bool RelativePoseError::evaluate(const double &time_stamp, const Transform &gt, const Transform &Tcg, 
									boost::shared_ptr<pcl::visualization::PCLPainter2D> fig)
	{
		bool rpe=false;
		int *size=new int[2];
		size=fig->getWindowSize();

		if(time_stamps.size()!=0)
		{
			for(int i=time_stamps.size()-1;i>=0;i--)
			{
				if(time_stamp-time_stamps[i]>=time_interval)
				{
					rpe=true;
					ulysses::Transform Twr=groundtruth[i];
					ulysses::Transform Twc=gt;
					ulysses::Transform Trg=estimates[i];
					ulysses::Transform Tcr_gt=Twc.inv()*Twr;
					ulysses::Transform Tcr=Tcg*Trg.inv();
					Vector6d delta_xi=Tcr_gt.getMotionVector()-Tcr.getMotionVector();

					errors.push_back(delta_xi);
					time_stamps_error.push_back(time_stamp);

					double error_translation=delta_xi.block<3,1>(0,0).norm();
					double error_rotation=delta_xi.block<3,1>(3,0).norm();
					double x_axis=errors.size();
					fig->addLine((x_axis-1)*2, error_translation_cache*size[0]+0.5*size[1],
								  x_axis*2,    error_translation*size[0]+0.5*size[1]);
					fig->addLine((x_axis-1)*2, error_rotation_cache*size[0],
								  x_axis*2,    error_rotation*size[0]);
					errors_translation.push_back(error_translation);
					errors_rotation.push_back(error_rotation);
					error_translation_cache=error_translation;
					error_rotation_cache=error_rotation;
					break;
				}
			}
		}

		time_stamps.push_back(time_stamp);
		groundtruth.push_back(gt);
		estimates.push_back(Tcg);

		return true;
	}

	bool RelativePoseError::evaluateEachFrame(const double &time_stamp, const Transform &gt, const Transform &Tcg, 
											boost::shared_ptr<pcl::visualization::PCLPainter2D> fig)
	{
		fp.open("relative_pose_error.txt",std::ios::app);
		bool rpe=false;
		int *size=new int[2];
		size=fig->getWindowSize();

		if(time_stamps.size()!=0)
		{
			int i=time_stamps.size()-1;
			rpe=true;
			ulysses::Transform Twr=groundtruth[i];
			ulysses::Transform Twc=gt;
			ulysses::Transform Trg=estimates[i];
			ulysses::Transform Tcr_gt=Twc.inv()*Twr;
			ulysses::Transform Tcr=Tcg*Trg.inv();
			Vector6d delta_xi=Tcr_gt.getMotionVector()-Tcr.getMotionVector();

			errors.push_back(delta_xi);
			time_stamps_error.push_back(time_stamp);

			double error_translation=delta_xi.block<3,1>(0,0).norm();
			double error_rotation=delta_xi.block<3,1>(3,0).norm();
			double x_axis=errors.size();
			fig->addLine((x_axis-1)*2, error_translation_cache*size[0]+0.5*size[1],
						  x_axis*2,    error_translation*size[0]+0.5*size[1]);
			fig->addLine((x_axis-1)*2, error_rotation_cache*size[0],
						  x_axis*2,    error_rotation*size[0]);
			errors_translation.push_back(error_translation);
			errors_rotation.push_back(error_rotation);
			error_translation_cache=error_translation;
			error_rotation_cache=error_rotation;

			fp<<std::fixed<<time_stamp<<"\t"<<error_translation<<"\t"<<error_rotation<<std::endl;
			std::cout<<std::endl<<"errors: "<<error_translation<<" "<<error_rotation<<std::endl;
			std::cout<<std::fixed<<time_stamp<<" "<<Trg.t.transpose()<<" "<<Trg.Quaternion().transpose()<<std::endl;
			Transform Tcgg=Tcg;
			std::cout<<std::fixed<<time_stamp<<" "<<Tcgg.t.transpose()<<" "<<Tcgg.Quaternion().transpose()<<std::endl;
		}

		time_stamps.push_back(time_stamp);
		groundtruth.push_back(gt);
		estimates.push_back(Tcg);

		fp.close();
		return true;
	}

	void RelativePoseError::saveErrors(const std::string &file)
	{
		std::cout<<std::endl<<"saving RPE errors to "<<file<<std::endl;
		std::ofstream fp;
		fp.open(file.c_str(),std::ios::out);
		for(int i=0;i<errors.size();i++)
		{
			fp<<std::fixed<<time_stamps_error[i]<<"\t"<<errors_translation[i]<<"\t"<<errors_rotation[i]<<"\t"<<errors[i].transpose()<<std::endl;
		}
		fp.close();
		std::cout<<"RPE errors saved"<<std::endl;
	}

}
