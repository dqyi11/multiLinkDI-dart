#include "MultiLinkDI.hpp"

int main(int argc, char* argv[])
{
  Eigen::Vector3d diPos;
  //diPos << -2.0, -2.0, 0.0;
  MultiLinkDI di(3, diPos);

  // Add each body to the last BodyNode in the di


  //dart::dynamics::BodyNode* bn = di.makeRootBody( MultiLinkDI::orientationType::X, "body1", 0.1, 1.0, 0.2,
  //                                                Eigen::Vector3d(0.0*M_PI/180, 90.0*M_PI/180, 0.0*M_PI/180));
  dart::dynamics::BodyNode* bn = di.makeRootBody( MultiLinkDI::orientationType::Y, "body1", 0.2, 1.0, 0.1,
                                                  Eigen::Vector3d(90.0*M_PI/180, 0.0*M_PI/180, 0.0*M_PI/180), 90 * M_PI / 180.0);
  //dart::dynamics::BodyNode* bn = di.makeRootBody( MultiLinkDI::orientationType::Z, "body1", 1.0, 0.1, 0.2,
  //                                                Eigen::Vector3d(0.0, 0.0, 0.0));


  for(unsigned int i=1; i<di.getNumOfLinks(); i++)
  {
    std::stringstream ss;
    ss << "body" << i+1;
    Eigen::Vector3d relativeTrans;
    relativeTrans << 0.0, 0.0, 1.0;
    bn = di.addBody(bn, MultiLinkDI::orientationType::Y, ss.str(), 0.2, 1.0, 0.1, Eigen::Vector3d(90.0*M_PI/180, 0.0*M_PI/180, 0.0*M_PI/180),
                    relativeTrans, 0 * M_PI / 180.0);
  }

  di.updateHomeConfig();
  di.addPlane();

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
  //di.setDofConfiguration(0, -45 * M_PI / 180.0);

  Eigen::VectorXd pos(3);
  pos << 45 * M_PI / 180.0, 15 * M_PI / 180.0, 15 * M_PI / 180.0;

  //std::cout << "COLLISION " << di.isCollided(pos) << std::endl;

  Eigen::Vector3d endPos = di.getEndEffectorPos();
  std::cout << "ENDEFFECTOR " << endPos << std::endl;

  Eigen::VectorXd pos2(3);
  pos2 << 15 * M_PI / 180.0, -15 * M_PI / 180.0, 15 * M_PI / 180.0;
  di.setConfiguration(pos2);

  di.initVisualization();

  return 0;
}
