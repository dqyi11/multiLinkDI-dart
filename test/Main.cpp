#include "MultiLinkDI.hpp"

int main(int argc, char* argv[])
{
  Eigen::Vector3d diPos;
  //diPos << -2.0, -2.0, 0.0;
  MultiLinkDI di(3, diPos);

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

  /*
  di.setDofPosition(0, 90 * M_PI / 180.0);
  di.setDofPosition(1, 90 * M_PI / 180.0);
  di.setDofPosition(2, 90 * M_PI / 180.0);
  */

  Eigen::VectorXd pos(3);
  pos << 45 * M_PI / 180.0, 0 * M_PI / 180.0, 0 * M_PI / 180.0;
  //di.setPosition(pos);

  std::cout << "COLLISION " << di.isCollided(pos) << std::endl;

  Eigen::Vector3d endPos = di.getEndEffectorPos();
  std::cout << "ENDEFFECTOR " << endPos << std::endl;

  Eigen::VectorXd pos2(3);
  pos2 << 15 * M_PI / 180.0, -15 * M_PI / 180.0, 15 * M_PI / 180.0;
  di.setConfiguration(pos2);

  di.initVisualization();

  return 0;
}
