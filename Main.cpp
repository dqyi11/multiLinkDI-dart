#include <dart/dart.hpp>
#include "MyWindow.hpp"

const double default_height = 1.0; // m
const double default_width = 0.2;  // m
const double default_depth = 0.2;  // m

const double default_torque = 15.0; // N-m
const double default_force =  15.0; // N
const int default_countdown = 200;  // Number of timesteps for applying force

const double default_rest_position = 0.0;
const double delta_rest_position = 10.0 * M_PI / 180.0;

const double default_stiffness = 0.0;
const double delta_stiffness = 10;

const double default_damping = 5.0;
const double delta_damping = 1.0;

const double default_cube_mass = 1.0;

using namespace dart::dynamics;
using namespace dart::simulation;

void setGeometry(const BodyNodePtr& bn)
{
  // Create a BoxShape to be used for both visualization and collision checking
  std::shared_ptr<BoxShape> box(new BoxShape(
      Eigen::Vector3d(default_width, default_depth, default_height)));

  // Create a shape node for visualization and collision checking
  auto shapeNode
      = bn->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(box);
  shapeNode->getVisualAspect()->setColor(dart::Color::Blue());

  // Set the location of the shape node
  Eigen::Isometry3d box_tf(Eigen::Isometry3d::Identity());
  Eigen::Vector3d center = Eigen::Vector3d(0, 0, default_height / 2.0);
  box_tf.translation() = center;
  shapeNode->setRelativeTransform(box_tf);

  // Move the center of mass to the center of the object
  bn->setLocalCOM(center);
}

SkeletonPtr createCube(const Eigen::Vector3d& _position,
                       const Eigen::Vector3d& _size,
                       double _mass)
{
  dart::dynamics::SkeletonPtr newCubeSkeleton =
      dart::dynamics::Skeleton::create();

  dart::dynamics::BodyNode::Properties body;
  body.mName = "cube_link";
  body.mInertia.setMass(_mass);
  body.mInertia.setMoment(
      dart::dynamics::BoxShape::computeInertia(_size, _mass));
  dart::dynamics::ShapePtr newBoxShape(new dart::dynamics::BoxShape(_size));

  dart::dynamics::FreeJoint::Properties joint;
  joint.mName = "cube_joint";
  joint.mT_ParentBodyToJoint = Eigen::Translation3d(_position);

  auto pair
      = newCubeSkeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(
        nullptr, joint, body);
  auto shapeNode = pair.second->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(newBoxShape);
  shapeNode->getVisualAspect()->setColor(dart::math::randomVector<3>(0.0, 1.0));

  return newCubeSkeleton;
}

BodyNode* makeRootBody(const SkeletonPtr& di, const std::string& name)
{
    // Set up the properties for the Joint
    RevoluteJoint::Properties properties;
    properties.mName = name + "_joint";
    properties.mAxis = Eigen::Vector3d::UnitY();
    properties.mT_ParentBodyToJoint.translation() =
        Eigen::Vector3d(0, 0, default_height);
    properties.mDampingCoefficients[0] = default_damping;
    properties.mRestPositions[0] = default_rest_position;
    properties.mSpringStiffnesses[0] = default_stiffness;

    // Create a new BodyNode, attached to its parent by a RevoluteJoint
    BodyNodePtr bn = di->createJointAndBodyNodePair<RevoluteJoint>(
          nullptr, properties, BodyNode::AspectProperties(name)).second;

    // Make a shape for the Joint
    const double R = default_width / 2.0;
    const double h = default_depth;
    std::shared_ptr<CylinderShape> cyl(new CylinderShape(R, h));

    // Line up the cylinder with the Joint axis
    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
    tf.linear() = dart::math::eulerXYZToMatrix(Eigen::Vector3d(0.0 * M_PI / 180.0,
                                                               90.0 * M_PI / 180.0,
                                                               0.0 * M_PI / 180.0));

    auto shapeNode = bn->createShapeNodeWith<VisualAspect>(cyl);
    shapeNode->getVisualAspect()->setColor(dart::Color::Blue());
    shapeNode->setRelativeTransform(tf);

    // Set the geometry of the Body
    setGeometry(bn);

    return bn;
}

BodyNode* addBody(const SkeletonPtr& di, BodyNode* parent,
                  const std::string& name)
{
  // Set up the properties for the Joint
  RevoluteJoint::Properties properties;
  properties.mName = name + "_joint";
  properties.mAxis = Eigen::Vector3d::UnitY();
  properties.mT_ParentBodyToJoint.translation() =
      Eigen::Vector3d(0, 0, default_height);
  properties.mRestPositions[0] = default_rest_position;
  properties.mSpringStiffnesses[0] = default_stiffness;
  properties.mDampingCoefficients[0] = default_damping;

  // Create a new BodyNode, attached to its parent by a RevoluteJoint
  BodyNodePtr bn = di->createJointAndBodyNodePair<RevoluteJoint>(
        parent, properties, BodyNode::AspectProperties(name)).second;

  // Make a shape for the Joint
  const double R = default_width / 2.0;
  const double h = default_depth;
  std::shared_ptr<CylinderShape> cyl(new CylinderShape(R, h));

  // Line up the cylinder with the Joint axis
  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.linear() = dart::math::eulerXYZToMatrix(Eigen::Vector3d(0.0 * M_PI / 180.0,
                                                             90.0 * M_PI / 180.0,
                                                             0.0 * M_PI / 180.0));

  auto shapeNode = bn->createShapeNodeWith<VisualAspect>(cyl);
  shapeNode->getVisualAspect()->setColor(dart::Color::Blue());
  shapeNode->setRelativeTransform(tf);

  // Set the geometry of the Body
  setGeometry(bn);

  return bn;
}

int main(int argc, char* argv[])
{
  // create and initialize the world
  auto world = std::make_shared<dart::simulation::World>();
  assert(world != nullptr);

  Eigen::Vector3d gravity(0.0, 0.0, 0.0);
  world->setGravity(gravity);

  SkeletonPtr di = Skeleton::create("di");

  // Add each body to the last BodyNode in the di
  BodyNode* bn = makeRootBody(di, "body1");
  bn = addBody(di, bn, "body2");
  bn = addBody(di, bn, "body3");
  //bn = addBody(di, bn, "body4");

  // Set the initial position of the first DegreeOfFreedom so that the di
  // starts to swing right away
  di->getDof(0)->setPosition(30 * M_PI / 180.0);
  //di->getDof(1)->setPosition(90 * M_PI / 180.0);
  //di->getDof(2)->setPosition(270 * M_PI / 180.0);
  //di->getDof(3)->setPosition(90 * M_PI / 180.0);

  Eigen::Vector3d planePos;
  Eigen::Vector3d planeSize;
  planeSize << 8.0, 8.0, 0.02;
  SkeletonPtr plane = createCube(planePos, planeSize, default_cube_mass);

  Eigen::Vector3d cube1Pos;
  Eigen::Vector3d cube1Size;
  cube1Pos << 1.0, 1.0, 0.25;
  cube1Size << 0.5, 0.5, 0.5;
  SkeletonPtr cube1 = createCube(cube1Pos, cube1Size, default_cube_mass);

  Eigen::Vector3d cube2Pos;
  Eigen::Vector3d cube2Size;
  cube2Pos << 2.0, 1.0, 0.25;
  cube2Size << 0.5, 0.5, 0.5;
  SkeletonPtr cube2 = createCube(cube2Pos, cube2Size, default_cube_mass);

  world->addSkeleton(di);
  world->addSkeleton(plane);
  world->addSkeleton(cube1);
  world->addSkeleton(cube2);

  // create a window and link it to the world
  MyWindow window;
  window.setWorld(world);

  glutInit(&argc, argv);
  window.initWindow(640, 480, "multiLinkDI");
  glutMainLoop();

  return 0;
}
