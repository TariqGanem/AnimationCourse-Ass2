#include "tutorial/sandBox/sandBox.h"
#include "igl/edge_flaps.h"
#include "igl/collapse_edge.h"
#include "Eigen/dense"
#include <functional>
#include "igl/read_triangle_mesh.h"
#include "igl/parallel_for.h"
#include <vector>
#include <igl/vertex_triangle_adjacency.h>
#include "igl/infinite_cost_stopping_condition.h"




SandBox::SandBox()
{
	

}

void SandBox::Init(const std::string &config)
{
	std::string item_name;
	std::ifstream nameFileout;
	Eigen::MatrixXi XF;
	Eigen::MatrixXd XV;
	VectorXi tempEMAP;
	MatrixXi tempE, tempEF, tempEI;
	int dataId = 0;
	nameFileout.open(config);
	if (!nameFileout.is_open())
	{
		std::cout << "Can't open file "<<config << std::endl;
	}
	else
	{
		
		while (nameFileout >> item_name)
		{
			std::cout << "openning " << item_name << std::endl;
			load_mesh_from_file(item_name);
			// reading the data file into faces and vertecies
			igl::read_triangle_mesh(item_name, XV, XF);
			parents.push_back(-1);
			data().add_points(Eigen::RowVector3d(0, 0, 0), Eigen::RowVector3d(0, 0, 1));
			data().show_overlay_depth = false;
			data().point_size = 10;
			if (dataId == 0) data().MyTranslate(Eigen::Vector3d(-1, 0, 0), 0);
			if (dataId == 1) data().MyTranslate(Eigen::Vector3d(1, 0, 0), 0);
			data().line_width = 2;
			data().set_visible(false, 1);
			// given Faces, returns the relevant fields such as Edges, Edges to Faces and so on
			igl::edge_flaps(XF, tempE, tempEMAP, tempEF, tempEI);
			// fill the relevant fields from the edge_flaps
			getEMAP().push_back(tempEMAP);
			getE().push_back(tempE);
			getEF().push_back(tempEF);
			getEI().push_back(tempEI);
			// ordered set < <edge_cost, edge_index> >
			set<pair<double, int>>* priorityQ = new set<pair<double, int>>();
			getQ().push_back(priorityQ);
			// adjust the new positions
			C.push_back(new Eigen::MatrixXd(getE()[dataId].rows(), data().V.cols() + 1));
			vector<PriorityQueue::iterator> Qit;
			Qit.resize(getE()[dataId].rows());
			// scan all edges for the costs	
			data().initQ4(XV.rows());
			for (int edge = 0; edge < getE()[dataId].rows(); edge++) {
				double edgeCost = 0;
				RowVectorXd p;
				// Sets a new position for each vertix of the collapsed edge
				igl::quadric_error(edge, data().V, data().F, getE()[dataId], getEMAP()[dataId], getEF()[dataId], getEI()[dataId], edgeCost, p,data().Q4);
				// update the new position
				C[dataId]->row(edge) = p;
				// inserting the new cost and returning a pair of iterator and something else
				auto tempVec = getQ()[dataId]->insert({ edgeCost, edge });
				// updating the iteratr
				Qit[edge] = tempVec.first;
			}
			num_collapsed.push_back(0);
			getQit().push_back(Qit);
			dataId++;
		}
		nameFileout.close();
	}
	MyTranslate(Eigen::Vector3d(0, 0, -1), true);
	
	data().set_colors(Eigen::RowVector3d(0.9, 0.1, 0.1));

}

SandBox::~SandBox()
{

}

void SandBox::Animate()
{
	if (isActive)
	{
		
		
		
	}
}


