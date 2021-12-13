// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2014 Daniele Panozzo <daniele.panozzo@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.

#include "Viewer.h"

//#include <chrono>
#include <thread>

#include <Eigen/LU>


#include <cmath>
#include <cstdio>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <limits>
#include <cassert>

#include <igl/project.h>
//#include <igl/get_seconds.h>
#include <igl/readOBJ.h>
#include <igl/readOFF.h>
#include <igl/adjacency_list.h>
#include <igl/writeOBJ.h>
#include <igl/writeOFF.h>
#include <igl/massmatrix.h>
#include <igl/file_dialog_open.h>
#include <igl/file_dialog_save.h>
#include <igl/quat_mult.h>
#include <igl/axis_angle_to_quat.h>
#include <igl/trackball.h>
#include <igl/two_axis_valuator_fixed_up.h>
#include <igl/snap_to_canonical_view_quat.h>
#include <igl/unproject.h>
#include <igl/serialize.h>
#include <igl/AABB.h>

// Internal global variables used for glfw event handling
//static igl::opengl::glfw::Viewer * __viewer;
static double highdpi = 1;
static double scroll_x = 0;
static double scroll_y = 0;


namespace igl
{
namespace opengl
{
namespace glfw
{

  void Viewer::Init(const std::string config)
  {
	  

  }

  IGL_INLINE Viewer::Viewer():
    data_list(1),
    selected_data_index(0),
    next_data_id(1),
	isPicked(false),
	isActive(false)
  {
    data_list.front().id = 0;

  

    // Temporary variables initialization
   // down = false;
  //  hack_never_moved = true;
    scroll_position = 0.0f;

    // Per face
    data().set_face_based(false);

    
#ifndef IGL_VIEWER_VIEWER_QUIET
    const std::string usage(R"(igl::opengl::glfw::Viewer usage:
  [drag]  Rotate scene
  A,a     Toggle animation (tight draw loop)
  F,f     Toggle face based
  I,i     Toggle invert normals
  L,l     Toggle wireframe
  O,o     Toggle orthographic/perspective projection
  T,t     Toggle filled faces
  [,]     Toggle between cameras
  1,2     Toggle between models
  ;       Toggle vertex labels
  :       Toggle face labels)"
);
    std::cout<<usage<<std::endl;
#endif
  }

  IGL_INLINE Viewer::~Viewer()
  {
  }

  IGL_INLINE bool Viewer::load_mesh_from_file(
      const std::string & mesh_file_name_string)
  {

    // Create new data slot and set to selected
    if(!(data().F.rows() == 0  && data().V.rows() == 0))
    {
      append_mesh();
    }
    data().clear();

    size_t last_dot = mesh_file_name_string.rfind('.');
    if (last_dot == std::string::npos)
    {
      std::cerr<<"Error: No file extension found in "<<
        mesh_file_name_string<<std::endl;
      return false;
    }

    std::string extension = mesh_file_name_string.substr(last_dot+1);

    if (extension == "off" || extension =="OFF")
    {
      Eigen::MatrixXd V;
      Eigen::MatrixXi F;
      if (!igl::readOFF(mesh_file_name_string, V, F))
        return false;
      data().set_mesh(V,F);
    }
    else if (extension == "obj" || extension =="OBJ")
    {
      Eigen::MatrixXd corner_normals;
      Eigen::MatrixXi fNormIndices;

      Eigen::MatrixXd UV_V;
      Eigen::MatrixXi UV_F;
      Eigen::MatrixXd V;
      Eigen::MatrixXi F;

      if (!(
            igl::readOBJ(
              mesh_file_name_string,
              V, UV_V, corner_normals, F, UV_F, fNormIndices)))
      {
        return false;
      }

      data().set_mesh(V,F);
      if (UV_V.rows() > 0)
      {
          data().set_uv(UV_V, UV_F);
      }

    }
    else
    {
      // unrecognized file type
      printf("Error: %s is not a recognized file type.\n",extension.c_str());
      return false;
    }

    data().compute_normals();
    data().uniform_colors(Eigen::Vector3d(51.0/255.0,43.0/255.0,33.3/255.0),
                   Eigen::Vector3d(255.0/255.0,228.0/255.0,58.0/255.0),
                   Eigen::Vector3d(255.0/255.0,235.0/255.0,80.0/255.0));

    // Alec: why?
    if (data().V_uv.rows() == 0)
    {
      data().grid_texture();
    }
    

    //for (unsigned int i = 0; i<plugins.size(); ++i)
    //  if (plugins[i]->post_load())
    //    return true;

    return true;
  }

  IGL_INLINE bool Viewer::save_mesh_to_file(
      const std::string & mesh_file_name_string)
  {
    // first try to load it with a plugin
    //for (unsigned int i = 0; i<plugins.size(); ++i)
    //  if (plugins[i]->save(mesh_file_name_string))
    //    return true;

    size_t last_dot = mesh_file_name_string.rfind('.');
    if (last_dot == std::string::npos)
    {
      // No file type determined
      std::cerr<<"Error: No file extension found in "<<
        mesh_file_name_string<<std::endl;
      return false;
    }
    std::string extension = mesh_file_name_string.substr(last_dot+1);
    if (extension == "off" || extension =="OFF")
    {
      return igl::writeOFF(
        mesh_file_name_string,data().V,data().F);
    }
    else if (extension == "obj" || extension =="OBJ")
    {
      Eigen::MatrixXd corner_normals;
      Eigen::MatrixXi fNormIndices;

      Eigen::MatrixXd UV_V;
      Eigen::MatrixXi UV_F;

      return igl::writeOBJ(mesh_file_name_string,
          data().V,
          data().F,
          corner_normals, fNormIndices, UV_V, UV_F);
    }
    else
    {
      // unrecognized file type
      printf("Error: %s is not a recognized file type.\n",extension.c_str());
      return false;
    }
    return true;
  }
 
  IGL_INLINE bool Viewer::load_scene()
  {
    std::string fname = igl::file_dialog_open();
    if(fname.length() == 0)
      return false;
    return load_scene(fname);
  }

  IGL_INLINE bool Viewer::load_scene(std::string fname)
  {
   // igl::deserialize(core(),"Core",fname.c_str());
    igl::deserialize(data(),"Data",fname.c_str());
    return true;
  }

  IGL_INLINE bool Viewer::save_scene()
  {
    std::string fname = igl::file_dialog_save();
    if (fname.length() == 0)
      return false;
    return save_scene(fname);
  }

  IGL_INLINE bool Viewer::save_scene(std::string fname)
  {
    //igl::serialize(core(),"Core",fname.c_str(),true);
    igl::serialize(data(),"Data",fname.c_str());

    return true;
  }

  IGL_INLINE void Viewer::open_dialog_load_mesh()
  {
    std::string fname = igl::file_dialog_open();

    if (fname.length() == 0)
      return;
    
    this->load_mesh_from_file(fname.c_str());
  }

  IGL_INLINE void Viewer::open_dialog_save_mesh()
  {
    std::string fname = igl::file_dialog_save();

    if(fname.length() == 0)
      return;

    this->save_mesh_to_file(fname.c_str());
  }

  IGL_INLINE ViewerData& Viewer::data(int mesh_id /*= -1*/)
  {
    assert(!data_list.empty() && "data_list should never be empty");
    int index;
    if (mesh_id == -1)
      index = selected_data_index;
    else
      index = mesh_index(mesh_id);

    assert((index >= 0 && index < data_list.size()) &&
      "selected_data_index or mesh_id should be in bounds");
    return data_list[index];
  }

  IGL_INLINE const ViewerData& Viewer::data(int mesh_id /*= -1*/) const
  {
    assert(!data_list.empty() && "data_list should never be empty");
    int index;
    if (mesh_id == -1)
      index = selected_data_index;
    else
      index = mesh_index(mesh_id);

    assert((index >= 0 && index < data_list.size()) &&
      "selected_data_index or mesh_id should be in bounds");
    return data_list[index];
  }

  IGL_INLINE int Viewer::append_mesh(bool visible /*= true*/)
  {
    assert(data_list.size() >= 1);

    data_list.emplace_back();
    selected_data_index = data_list.size()-1;
    data_list.back().id = next_data_id++;
    //if (visible)
    //    for (int i = 0; i < core_list.size(); i++)
    //        data_list.back().set_visible(true, core_list[i].id);
    //else
    //    data_list.back().is_visible = 0;
    return data_list.back().id;
  }

  IGL_INLINE bool Viewer::erase_mesh(const size_t index)
  {
    assert((index >= 0 && index < data_list.size()) && "index should be in bounds");
    assert(data_list.size() >= 1);
    if(data_list.size() == 1)
    {
      // Cannot remove last mesh
      return false;
    }
    data_list[index].meshgl.free();
    data_list.erase(data_list.begin() + index);
    if(selected_data_index >= index && selected_data_index > 0)
    {
      selected_data_index--;
    }

    return true;
  }

  IGL_INLINE size_t Viewer::mesh_index(const int id) const {
    for (size_t i = 0; i < data_list.size(); ++i)
    {
      if (data_list[i].id == id)
        return i;
    }
    return 0;
  }

  Eigen::Matrix4d Viewer::CalcParentsTrans(int indx) 
  {
	  Eigen::Matrix4d prevTrans = Eigen::Matrix4d::Identity();

	  for (int i = indx; parents[i] >= 0; i = parents[i])
	  {
		  //std::cout << "parent matrix:\n" << scn->data_list[scn->parents[i]].MakeTrans() << std::endl;
		  prevTrans = data_list[parents[i]].MakeTransd() * prevTrans;
	  }

	  return prevTrans;
  }

  Eigen::MatrixXd Viewer::setCorners(Eigen::AlignedBox<double, 3> box) {
	  Eigen::MatrixXd V_box(8, 3);
	  V_box << 
		  box.corner(box.TopLeftCeil)[0], box.corner(box.TopLeftCeil)[1], box.corner(box.TopLeftCeil)[2],
		  box.corner(box.TopRightCeil)[0], box.corner(box.TopRightCeil)[1], box.corner(box.TopRightCeil)[2],
		  box.corner(box.TopRightFloor)[0], box.corner(box.TopRightFloor)[1], box.corner(box.TopRightFloor)[2],
		  box.corner(box.TopLeftFloor)[0], box.corner(box.TopLeftFloor)[1], box.corner(box.TopLeftFloor)[2],
		  box.corner(box.BottomLeftCeil)[0], box.corner(box.BottomLeftCeil)[1], box.corner(box.BottomLeftCeil)[2],
		  box.corner(box.BottomRightCeil)[0], box.corner(box.BottomRightCeil)[1], box.corner(box.BottomRightCeil)[2],
		  box.corner(box.BottomRightFloor)[0], box.corner(box.BottomRightFloor)[1], box.corner(box.BottomRightFloor)[2],
		  box.corner(box.BottomLeftFloor)[0], box.corner(box.BottomLeftFloor)[1], box.corner(box.BottomLeftFloor)[2];
	  return V_box;
  }

  void Viewer::drawAllignedBox(Eigen::AlignedBox<double, 3> box1, Eigen::AlignedBox<double, 3> box2, Eigen::RowVector3d color) {
	  for (int i = 0; i < 2; i++) {
		  Eigen::MatrixXd V_box(8, 3);
		  Eigen::MatrixXi E_box(12, 2);
          if (i == 0)
              V_box = setCorners(box1);
          else
              V_box = setCorners(box2);
		  E_box <<
			  0, 1,
			  1, 2,
			  2, 3,
			  3, 0,
			  4, 5,
			  5, 6,
			  6, 7,
			  7, 4,
			  0, 4,
			  1, 5,
			  2, 6,
			  7, 3;
		  for (int j = 0; j < E_box.rows(); ++j)
			  data_list[i].add_edges(V_box.row(E_box(j, 0)), V_box.row(E_box(j, 1)), color);
	  }
  }

  bool Viewer::interactionCheckRecursive(igl::AABB<Eigen::MatrixXd, 3> t1, igl::AABB<Eigen::MatrixXd, 3> t2, Eigen::Matrix4f A, Eigen::Matrix4f B)
  {
	  if (!interactionCheck(t1.m_box, t2.m_box, A, B))
		  return false;
	  if (t1.is_leaf() && t2.is_leaf())
	  {
		  drawAllignedBox(data_list[0].lastTree.m_box, data_list[1].lastTree.m_box, Eigen::RowVector3d(7, 7, 7));
		  drawAllignedBox(t1.m_box, t2.m_box, Eigen::RowVector3d(0, 0, 0));
		  return true;
	  }
	  if (t1.is_leaf())
	  {
		  data_list[0].lastTree = t1;
		  return (interactionCheckRecursive(t1, *t2.m_right, A, B) || interactionCheckRecursive(t1, *t2.m_left, A, B));
	  }

	  if (t2.is_leaf())
	  {
		  data_list[1].lastTree = t2;
		  return (interactionCheckRecursive(*t1.m_right, t2, A, B) || interactionCheckRecursive(*t1.m_left, t2, A, B));
	  }
	  data_list[1].lastTree = t2;
	  data_list[0].lastTree = t1;
	  return (interactionCheckRecursive(*t1.m_right, *t2.m_right, A, B)) ||
		  (interactionCheckRecursive(*t1.m_right, *t2.m_left, A, B)) ||
		  (interactionCheckRecursive(*t1.m_left, *t2.m_left, A, B)) ||
		  (interactionCheckRecursive(*t1.m_left, *t2.m_right, A, B));
  }
  IGL_INLINE void Viewer::animate() {
	  if (isMeshMoving) {
		  if (interactionCheckRecursive(data_list[0].tree, data_list[1].tree, MakeTransScale() * data_list[0].MakeTransScale(), MakeTransScale() * data_list[1].MakeTransScale())) {
			  isMeshMoving = false;
			  return;
		  }
		  data().MyTranslate(data().v, true);
	  }
  }
  bool Viewer::interactionCheck(Eigen::AlignedBox<double, 3> Box1, Eigen::AlignedBox<double, 3> Box2, Eigen::Matrix4f A, Eigen::Matrix4f B)
  {
	  Eigen::Matrix4f c = A * B;
	  Eigen::Vector4f newMatrix = Eigen::Vector4f(Box1.center()(0), Box1.center()(1), Box1.center()(2), 1);
	  newMatrix = A * newMatrix;
	  Eigen::Vector3d c0 = Eigen::Vector3d(newMatrix.x(), newMatrix.y(), newMatrix.z());
	  newMatrix = Eigen::Vector4f(Box2.center()(0), Box2.center()(1), Box2.center()(2), 1);
	  newMatrix = B * newMatrix;
	  Eigen::Vector3d c1 = Eigen::Vector3d(newMatrix.x(), newMatrix.y(), newMatrix.z());
	  Eigen::Vector3d d = c1 - c0;
	  Eigen::Vector3d Ax = Eigen::Vector3d(A(0, 0), A(1, 0), A(2, 0));
	  Eigen::Vector3d Ay = Eigen::Vector3d(A(0, 1), A(1, 1), A(2, 1));
	  Eigen::Vector3d Az = Eigen::Vector3d(A(0, 2), A(1, 2), A(2, 2));
	  Eigen::Vector3d Bx = Eigen::Vector3d(B(0, 0), B(1, 0), B(2, 0));
	  Eigen::Vector3d By = Eigen::Vector3d(B(0, 1), B(1, 1), B(2, 1));
	  Eigen::Vector3d Bz = Eigen::Vector3d(B(0, 2), B(1, 2), B(2, 2));
	  double wA, hA, dA, hB, dB, wB, R0, R1, R;
	  wA = (Box1.sizes()(0) / 2) + 0.0001; 
      hA = (Box1.sizes()(1) / 2) + 0.0001; 
      dA = (Box1.sizes()(2) / 2) + 0.0001;
	  wB = (Box2.sizes()(0) / 2) + 0.0001; 
      hB = (Box2.sizes()(1) / 2) + 0.0001; 
      dB = (Box2.sizes()(2) / 2) + 0.0001;
	  if (std::abs(Ax.dot(d)) > (wB * std::abs(c(0, 0))) + (hB * std::abs(c(0, 1))) + (dB * std::abs(c(0, 2))) + wA)
		  return false;
	  if (std::abs(Ay.dot(d)) > (wB * std::abs(c(1, 0))) + (hB * std::abs(c(1, 1))) + (dB * std::abs(c(1, 2))) + hA)
		  return false;
	  if (std::abs(Az.dot(d)) > (wB * std::abs(c(2, 0))) + (hB * std::abs(c(2, 1))) + (dB * std::abs(c(2, 2))) + dA)
		  return false;
	  if (std::abs(Bx.dot(d)) > wB + (wA * std::abs(c(0, 0))) + (hA * std::abs(c(1, 0))) + (dA * std::abs(c(2, 0))))
		  return false;
	  if (std::abs(By.dot(d)) > hB + (wA * std::abs(c(0, 1))) + (hA * std::abs(c(1, 1))) + (dA * std::abs(c(2, 1))))
		  return false;
	  if (std::abs(Bz.dot(d)) > dB + (wA * std::abs(c(0, 2))) + (hA * std::abs(c(1, 2))) + (dA * std::abs(c(2, 2))))
		  return false;
	  if (std::abs((d.dot(c(1, 0) * Az)) - (d.dot(c(2, 0) * Ay))) > (hB * std::abs(c(0, 2))) + (dB * std::abs(c(0, 1))) + (hA * std::abs(c(2, 0))) + (dA * std::abs(c(1, 0))))
		  return false;
	  if (std::abs((d.dot(c(1, 1) * Az)) - (d.dot(c(2, 1) * Ay))) > (wB * std::abs(c(0, 2))) + (dB * std::abs(c(0, 0))) + (hA * std::abs(c(2, 1))) + (dA * std::abs(c(1, 1))))
		  return false;
	  if (std::abs((d.dot(c(1, 2) * Az)) - (d.dot(c(2, 2) * Ay))) > (wB * std::abs(c(0, 1))) + (hB * std::abs(c(0, 0))) + (hA * std::abs(c(2, 2))) + (dA * std::abs(c(1, 2))))
		  return false;
	  if (std::abs((d.dot(c(2, 0) * Ax)) - (d.dot(c(0, 0) * Az))) > (hB * std::abs(c(1, 2))) + (dB * std::abs(c(1, 1))) + (wA * std::abs(c(2, 0))) + (dA * std::abs(c(0, 0))))
		  return false;
	  if (std::abs((d.dot(c(2, 1) * Ax)) - (d.dot(c(0, 1) * Az))) > (wB * std::abs(c(1, 2))) + (dB * std::abs(c(1, 0))) + (wA * std::abs(c(2, 1))) + (dA * std::abs(c(0, 1))))
		  return false;
	  if (std::abs((d.dot(c(2, 2) * Ax)) - (d.dot(c(0, 2) * Az))) > (wB * std::abs(c(1, 1))) + (hB * std::abs(c(1, 0))) + (wA * std::abs(c(2, 2))) + (dA * std::abs(c(0, 2))))
		  return false;
	  if (std::abs((d.dot(c(0, 0) * Ay)) - (d.dot(c(1, 0) * Ax))) > (hB * std::abs(c(2, 2))) + (dB * std::abs(c(2, 1))) + (wA * std::abs(c(1, 0))) + (hA * std::abs(c(0, 0))))
		  return false;
	  if (std::abs((d.dot(c(0, 1) * Ay)) - (d.dot(c(1, 1) * Ax))) > (wB * std::abs(c(2, 2))) + (dB * std::abs(c(2, 0))) + (wA * std::abs(c(1, 1))) + (hA * std::abs(c(0, 1))))
		  return false;
	  if (std::abs((d.dot(c(0, 2) * Ay)) - (d.dot(c(1, 2) * Ax))) > (wB * std::abs(c(2, 1))) + (hB * std::abs(c(2, 0))) + (wA * std::abs(c(1, 2))) + (hA * std::abs(c(0, 2))))
		  return false;
	  return true;
  }

} // end namespace
} // end namespace
}
