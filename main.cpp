
#include "igl/opengl/glfw/renderer.h"
#include "tutorial/sandBox/inputManager.h"
#include "sandBox.h"

int main(int argc, char *argv[])
{
  Display *disp = new Display(1200, 800, "Wellcome");
  Renderer renderer;

  SandBox viewer;
  
  igl::opengl::glfw::imgui::ImGuiMenu* menu = new igl::opengl::glfw::imgui::ImGuiMenu();
  viewer.Init("configuration.txt");
  for (int i = 0; i < 2; i++) {
	  viewer.selected_data_index = i;
	  renderer.core().toggle(viewer.data().show_lines);
	  viewer.data().set_colors(Eigen::RowVector3d(0.8, 0.8, 0));
	  viewer.data().initBox();
  }
  Init(*disp, menu);
  renderer.init(&viewer,2,menu);
  
  disp->SetRenderer(&renderer);
  disp->launch_rendering(true);
  delete menu;
  delete disp;
}
