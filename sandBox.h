#pragma once
#include "igl/opengl/glfw/Viewer.h"
#include "igl/aabb.h"
#include <igl/read_triangle_mesh.h>
#include <set>
#include <igl/shortest_edge_and_midpoint.h>
using namespace std;
using namespace Eigen;

typedef  set<pair<double, int>> PriorityQueue;

class SandBox : public igl::opengl::glfw::Viewer
{
public:
	SandBox();
	~SandBox();
	void Init(const std::string& config);
	// Getters for the private fields
	vector<vector<Matrix4d>>& getQs() {
		return Qs;
	}
	vector<VectorXi>& getEMAP() {
		return EMAP;
	}
	vector<MatrixXi>& getE() {
		return E;
	}
	vector<MatrixXi>& getEF() {
		return EF;
	}
	vector<MatrixXi>& getEI() {
		return EI;
	}
	vector<set<pair<double, int>>*>& getQ() {
		return Q;
	}
	vector<vector<PriorityQueue::iterator>>& getQit() {
		return Qit;
	}
	vector<MatrixXd*>& getC() {
		return C;
	}
	vector<int> getNum_collapsed() {
		return num_collapsed;
	}
private:
	// Prepare array-based edge data structures and priority queue
	vector<vector<Matrix4d>> Qs;
	vector<VectorXi> EMAP;
	vector<MatrixXi> E, EF, EI;
	vector<set<pair<double, int>>*> Q;
	vector<vector<PriorityQueue::iterator>> Qit;
	vector<MatrixXd*> C;
	vector<int> num_collapsed;
	void Animate();
};

