/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2019-05-09 20:11
#
# Filename:		plane_feature_matching.h
#
# Description: 
#
===============================================*/

#include "types.h"

namespace ulysses
{
	void visMatchedPlanes(Scan *scan, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis);

	struct Node_InterpTree
	{
		Node_InterpTree() {}
		Node_InterpTree(Plane *plane_ref_, Plane *plane_cur_)
		{
			plane_ref=plane_ref_;
			plane_cur=plane_cur_;
		}

		Plane *plane_ref,*plane_cur;

		Node_InterpTree *parent;
		std::vector<Node_InterpTree*> children;

		void setParent(Node_InterpTree* node)
		{
			parent=node;
			node->children.push_back(this);
			this->layer=node->layer+1;
		}

		void insertChild(Node_InterpTree* node)
		{
			children.push_back(node);
			node->parent=this;
			node->layer=this->layer+1;
		}

		int layer;
	};

	// layers: numbers of planes in referece frame
	// children per node: numbers of planes in current frame
	class InterpTree
	{
	public:

		InterpTree()
		{
			root=new Node_InterpTree;
			thres_color=1.0;
			thres_delta_angle=0.2; // 0.2rad~=11.5deg
			thres_delta_d=0.07; // 10cm
			debug=false;
		}


		InterpTree(bool db, double angle, double d, double color)
		{
			root=new Node_InterpTree;
			thres_delta_angle=angle*M_PI/180.0; // angle~deg
			thres_delta_d=d;
			thres_color=color;
			debug=db;
		}
		~InterpTree() {Release();}

		void Release()
		{
			delete root;
			for(std::vector<Node_InterpTree*>::iterator it=nodes.begin();it!=nodes.end();it++)
			{
				delete *it;
			}
			std::vector<Node_InterpTree*>().swap(nodes);
			leaf_max_interp.clear();
		}

		void setDebug(bool d) {debug=d;}

		int getNodeNum() {return nodes.size();}

		Node_InterpTree* getRoot() {return root;}

		std::vector<Node_InterpTree*> getMaxInterp() 
		{
			return leaf_max_interp;
		}

		bool Construct(std::vector<Plane*> &planes_ref_,std::vector<Plane*> &planes_cur_);
		
	private:

		bool debug;

		std::ofstream fp;

		Node_InterpTree *root;
		std::vector<Node_InterpTree*> nodes;
		std::vector<Node_InterpTree*> leaf_max_interp;

		double thres_color; // thres for consistent_1
		double thres_delta_angle; // (consistent_2) if delta_normal_angle<thres then the planes are parallel
		double thres_delta_d; // (consistent_2) if delta_d<thres then the plane pairs are coincident

		bool consistent_1(Node_InterpTree *node);
		bool consistent_2(Node_InterpTree *node1, Node_InterpTree *node2);
		bool consistent_3(Node_InterpTree *node1, Node_InterpTree *node2, Node_InterpTree *node3);
		bool isLeaf(Node_InterpTree *node);
	};

	class PlaneFeatureMatching
	{
	public:

		PlaneFeatureMatching()
		{
			interp_tree = new InterpTree();
			remove("plane_matching.txt");
		}

		PlaneFeatureMatching(bool v, bool db, double angle, double d, double color)
		{
			debug=db;
			visual=v;
			interp_tree = new InterpTree(db,angle,d,color);
			remove("plane_matching.txt");
		}

		~PlaneFeatureMatching() {delete interp_tree;}

		void setDebug(bool d) {debug=d; interp_tree->setDebug(d);}

		bool match(Scan *scan, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis);

		//void depthFirstSearch();
		//void breadthFirstTravel();
		
	private:

		bool debug;
		bool visual;

		InterpTree *interp_tree;
	};
}
