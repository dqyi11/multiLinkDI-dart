#include "MultiLinkDI.hpp"
#include "MyWindow.hpp"

int main(int argc, char* argv[])
{
  MultiLinkDI di(2);

  // Set the initial position of the first DegreeOfFreedom so that the di
  // starts to swing right away
  di.setDofPosition(0, 0 * M_PI / 180.0);

  Eigen::Vector3d cube1Pos;
  Eigen::Vector3d cube1Size;
  cube1Pos << 1.0, 1.0, 0.25;
  cube1Size << 0.5, 0.5, 0.5;
  di.addCube(cube1Pos, cube1Size);

  Eigen::Vector3d cube2Pos;
  Eigen::Vector3d cube2Size;
  cube2Pos << 2.0, 1.0, 0.25;
  cube2Size << 0.5, 0.5, 0.5;
  di.addCube(cube2Pos, cube2Size);

  di.render();

  return 0;
}
