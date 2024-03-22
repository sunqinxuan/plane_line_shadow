/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@gmail.com
#
# Last modified:	2019-04-22 11:05
#
# Filename:		global_map.h
#
# Description: 
#
************************************************/

#include "types.h"

namespace ulysses
{
	class GlobalMap
	{
	public:
		GlobalMap() 
		{
			remove("global_map.txt");
			debug=false;
		}

		~GlobalMap() {}

		void setDebug(bool d) {debug=d;}

		void addScan(Scan *scan, Map* map);

	private:
		bool debug;
		std::ofstream fp;
	};

	class LocalMap
	{
	public:
		LocalMap() 
		{
			remove("local_map.txt");
			debug=false;
		}

		~LocalMap() {}

		void setDebug(bool d) {debug=d;}

	private:
		bool debug;
		std::ofstream fp;
	};
}
