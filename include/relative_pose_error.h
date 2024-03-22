/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@gmail.com
#
# Last modified: 2019-05-10 10:05
#
# Filename: relative_pose_error.h
#
# Description: 
#
************************************************/

#include "types.h"

namespace ulysses
{
	class RelativePoseError
	{
	public:
		RelativePoseError(double delta)
			: time_interval(delta)
		{
			error_translation_cache=0;
			error_rotation_cache=0;
			remove("relative_pose_error.txt");
		}

		bool evaluate(const double &time_stamp, const Transform &gt, const Transform &Tcg, 
								boost::shared_ptr<pcl::visualization::PCLPainter2D> fig);
		bool evaluateEachFrame(const double &time_stamp, const Transform &gt, const Transform &Tcg, 
								boost::shared_ptr<pcl::visualization::PCLPainter2D> fig); // each frame;
		
		void saveErrors(const std::string &file);

		double getCurrentErrorTrans() 
		{
			if(errors_translation.size()>0)
				return errors_translation[errors_translation.size()-1]; 
			else 
				return 0;
		}

	private:
		const double time_interval;
		std::ofstream fp;

		double error_translation_cache;
		double error_rotation_cache;

		std::vector<double> time_stamps;
		std::vector<ulysses::Transform> groundtruth;
		std::vector<ulysses::Transform> estimates; // Tcg

		std::vector<double> time_stamps_error;
		std::vector<Vector6d> errors;
		std::vector<double> errors_translation;
		std::vector<double> errors_rotation;
	};
}
