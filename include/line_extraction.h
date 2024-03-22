/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@gmail.com
#
# Last modified:	2019-05-09 14:23
#
# Filename:		line_extraction.h
#
# Description: 
#
************************************************/

#include "types.h"

namespace ulysses
{
	void visLines(Scan *scan, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis);

	class LineExtraction
	{
	public:

		LineExtraction() 
		{
			remove("extract_Lines.txt");
			thres_line2shadow=1.0;
			thres_shadow2plane=0.1;
		}

		LineExtraction(bool v, bool db, double r, double t, int th, double min, double max, 
					double dir, double dist, double split, int min_points,
					double l2s, double s2p)
		{
			remove("extract_Lines.txt");
			visual=v;
			debug=db;
			rho=r; 
			theta=t*M_PI/180.0; 
			threshold=th; 
			minLineLength=min; 
			maxLineGap=max;
			thres_sim_dir=dir;
			thres_sim_dist=dist;
			thres_split=split;
			min_points_on_line=min_points;
			thres_line2shadow=l2s;
			thres_shadow2plane=s2p;
		}

		~LineExtraction() {}

		void setDebug(bool d) {debug=d;}

		void extractLines(Scan *scan, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis);

//		void fitLines(Scan *scan);
//		bool fitSphere(EdgePoint *edge_point);

	private:
		
		std::ofstream fp;
		bool debug;
		bool visual;
		
		size_t height,width;

		double rho; 
		double theta; 
		int threshold; 
		double minLineLength; 
		double maxLineGap;

		double thres_sim_dir, thres_sim_dist;
		double thres_split;
		int min_points_on_line;

		double thres_line2shadow;
		double thres_shadow2plane;

		void extractLinesHough(std::vector<EdgePoint*>& edge_points, std::list<Line*>& lines_occluding, double scale);

//		void generatePlucker1(Line *line);
		void generatePlucker(Line *line);
		void compute_line_cov_inv(Line *line);
	};

}
