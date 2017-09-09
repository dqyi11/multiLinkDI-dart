#include "MultiLinkDI.hpp"

int main(int argc, char* argv[])
{
  MultiLinkDI di(6, Eigen::Vector3d(0.39, -0.2, 0.0));

  // Add each body to the last BodyNode in the di


  //dart::dynamics::BodyNode* bn = di.makeRootBody( MultiLinkDI::orientationType::X, "body1", 0.1, 1.0, 0.2,
  //                                                Eigen::Vector3d(0.0*M_PI/180, 90.0*M_PI/180, 0.0*M_PI/180));
  //dart::dynamics::BodyNode* bn = di.makeRootBody( MultiLinkDI::orientationType::Y, "body1", 0.2, 1.0, 0.1,
  //                                                Eigen::Vector3d(90.0*M_PI/180, 0.0*M_PI/180, 0.0*M_PI/180), 90 * M_PI / 180.0);
  /*
  dart::dynamics::BodyNode* bn = di.makeRootBody( MultiLinkDI::orientationType::Z, "body1", 1.0, 0.1, 0.2,
                                                  Eigen::Vector3d(0.0, 0.0, 0.0));


  for(unsigned int i=1; i<di.getNumOfLinks(); i++)
  {
    std::stringstream ss;
    ss << "body" << i+1;
    Eigen::Vector3d relativeTrans;
    relativeTrans << 1.0, 0.0, 0.0;
    bn = di.addBody(bn, MultiLinkDI::orientationType::Z, ss.str(), 1.0, 0.1, 0.2, Eigen::Vector3d(0.0*M_PI/180, 0.0*M_PI/180, 0.0*M_PI/180),
                    relativeTrans, 0 * M_PI / 180.0);
  }*/

  dart::dynamics::BodyNode* bn = di.makeRootBody( MultiLinkDI::orientationType::Y, "L1", 0.1, 0.3, 0.1,
                                                  Eigen::Vector3d(90.0*M_PI/180, 0.0*M_PI/180, 0.0*M_PI/180),
                                                  1.57);
  bn = di.addBody(bn, MultiLinkDI::orientationType::X, "L2", 0.1, 0.3, 0.1,
                  Eigen::Vector3d(90.0*M_PI/180, 90.0*M_PI/180, 0.0*M_PI/180),
                  Eigen::Vector3d(0.0, 0.0, 0.3), 0.0);
  bn = di.addBody(bn, MultiLinkDI::orientationType::Y, "L3", 0.1, 0.3, 0.1,
                  Eigen::Vector3d(90.0*M_PI/180, 0.0*M_PI/180, 0.0*M_PI/180),
                  Eigen::Vector3d(0.0, 0.0, 0.3), 0.0);
  bn = di.addBody(bn, MultiLinkDI::orientationType::X, "L4", 0.1, 0.3, 0.1,
                  Eigen::Vector3d(90.0*M_PI/180, 90.0*M_PI/180, 0.0*M_PI/180),
                  Eigen::Vector3d(0.0, 0.0, 0.3), 0.0);
  bn = di.addBody(bn, MultiLinkDI::orientationType::Y, "L5", 0.1, 0.3, 0.1,
                  Eigen::Vector3d(90.0*M_PI/180, 0.0*M_PI/180, 0.0*M_PI/180),
                  Eigen::Vector3d(0.0, 0.0, 0.3), 0.0);
  bn = di.addBody(bn, MultiLinkDI::orientationType::X, "L6", 0.1, 0.3, 0.1,
                  Eigen::Vector3d(90.0*M_PI/180, 90.0*M_PI/180, 0.0*M_PI/180),
                  Eigen::Vector3d(0.0, 0.0, 0.3), 0.0);



  di.updateHomeConfig();
  //di.addPlane();

  /*
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
  */

  di.addCube(Eigen::Vector3d(1.3, 1.5, -0.3), Eigen::Vector3d(1.5, 1.8, 1.0));
  di.addCube(Eigen::Vector3d(1.3, -0.8, -0.3), Eigen::Vector3d(1.5, 1.8, 1.0));

  /*
  di.setDofPosition(0, 90 * M_PI / 180.0);
  di.setDofPosition(1, 90 * M_PI / 180.0);
  di.setDofPosition(2, 90 * M_PI / 180.0);
  */
  //di.setDofConfiguration(0, -45 * M_PI / 180.0);

  //Eigen::VectorXd pos(3);
  //pos << 45 * M_PI / 180.0, 15 * M_PI / 180.0, 15 * M_PI / 180.0;
  //di.setConfiguration(pos);

  //std::cout << "COLLISION " << di.isCollided(pos) << std::endl;

  Eigen::Vector3d endPos = di.getEndEffectorPos();
  std::cout << "ENDEFFECTOR " << endPos << std::endl;

  /*
  Eigen::VectorXd pos2(3);
  pos2 << 15 * M_PI / 180.0, -15 * M_PI / 180.0, 15 * M_PI / 180.0;
  di.setConfiguration(pos2);
  */

  Eigen::VectorXd pos(12);
  pos << -1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  di.setConfiguration(pos);

  std::cout << "COLLISION " << di.isCollided(pos) << std::endl;

  di.initVisualization();

  return 0;
}
