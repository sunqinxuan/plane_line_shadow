/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2017-06-08 09:41
#
# Filename:		data_reading.h
#
# Description: 
#
===============================================*/

#include "types.h"

namespace ulysses
{
	class DataReading
	{
	public:

		DataReading() {}

		~DataReading() {}

		void loadScan(Scan *scan, const cv::Mat &rgb, const cv::Mat &depth);

	private:

		// integral image normal estimation method;
		pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBA, pcl::Normal> normal_estimate_integral;

	};
}
