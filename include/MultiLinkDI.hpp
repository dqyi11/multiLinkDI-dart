#ifndef MULTI_LINK_DI_
#define MULTI_LINK_DI_

#include <dart/dart.hpp>
#include "MultiLinkDIWindow.hpp"

class MultiLinkDI
{
public:
    MultiLinkDI(const unsigned int num_of_links, Eigen::Vector3d& pos);
    virtual ~MultiLinkDI();

    Eigen::VectorXd getConfiguration();
    void setConfiguration(const Eigen::VectorXd& config);
    void setDofConfiguration(unsigned int idx, double config);

    void addCube(const Eigen::Vector3d& _poisition,
                 const Eigen::Vector3d& _size);

    bool isCollided(const Eigen::VectorXd& pos);

    void initVisualization();
    void updateVisualization();

    Eigen::Vector3d getEndEffectorPos(const Eigen::VectorXd& config);
    Eigen::Vector3d getEndEffectorPos();

    unsigned int getNumOfLinks() { return num_of_links_; }

    void addWaypoint( Eigen::VectorXd waypoint ) { waypoints_.push_back(waypoint); }
    void clearWaypoints() { waypoints_.clear(); }
    size_t getWaypointNum() { return waypoints_.size(); }
    Eigen::VectorXd getWaypoint(size_t idx) { return waypoints_[idx]; }


    double getResolutionSize() { return default_resolution_size; }

protected:
    void setGeometry(const dart::dynamics::BodyNodePtr& bn);

    dart::dynamics::SkeletonPtr createCube(const Eigen::Vector3d& _position,
                                           const Eigen::Vector3d& _size,
                                           double _mass);

    dart::dynamics::BodyNode* makeRootBody(const dart::dynamics::SkeletonPtr& di,
                                           const std::string& name);

    dart::dynamics::BodyNode* addBody(const dart::dynamics::SkeletonPtr& di,
                                      dart::dynamics::BodyNode* parent,
                                      const std::string& name);

    void initLineSegment();

    unsigned int num_of_links_;

    const double default_distance_to_ground = 0.05;

    const double default_height = 0.1; // m
    const double default_width = 1.0;  // m
    const double default_depth = 0.1;  // m

    const double default_torque = 15.0; // N-m
    const double default_force =  15.0; // N
    const int default_countdown = 200;  // Number of timesteps for applying force

    const double default_rest_position = 0.0;
    const double delta_rest_position = 10.0 * M_PI / 180.0;

    const double default_stiffness = 0.0;
    const double delta_stiffness = 10;

    const double default_damping = 5.0;
    const double delta_damping = 1.0;

    const double default_resolution_size = 0.02;
    const double default_cube_mass = 1.0;
    const Eigen::Vector4d DefaultForceLineColor = Eigen::Vector4d(1.0, 0.63, 0.0, 1.0);

    dart::simulation::WorldPtr world_;
    dart::dynamics::SkeletonPtr di_;
    dart::dynamics::SkeletonPtr plane_;
    std::vector<dart::dynamics::BodyNode*> bodyNodes_;

    Eigen::Vector3d basePos_;

    std::vector<dart::dynamics::SkeletonPtr> objects_;

    std::vector<Eigen::VectorXd> waypoints_;
    std::vector<dart::dynamics::LineSegmentShapePtr> lineSegments_;

    MultiLinkDIWindow window_;
};

#endif // MULTI_LINK_DI_
