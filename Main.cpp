#include <dart/dart.hpp>

#include "MyWindow.hpp"

int main(int argc, char* argv[])
{
  // create and initialize the world
  auto world = std::make_shared<dart::simulation::World>();
  assert(world != nullptr);

  Eigen::Vector3d gravity(0.0, 0.0, 0.0);
  world->setGravity(gravity);

  // create a window and link it to the world
  MyWindow window;
  window.setWorld(world);

  glutInit(&argc, argv);
  window.initWindow(640, 480, "multiLinkDI");
  glutMainLoop();

  return 0;
}
