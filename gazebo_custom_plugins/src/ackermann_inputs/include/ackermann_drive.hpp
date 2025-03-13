#ifndef GAZEBO_PLUGINS__GAZEBO_ROS_ACKERMANN_DRIVE_HPP_
#define GAZEBO_PLUGINS__GAZEBO_ROS_ACKERMANN_DRIVE_HPP_

#include <gazebo/common/Plugin.hh>

#include <memory>

namespace gazebo_plugins
{
class AckermannDrivePrivate;

/// A ackermann drive plugin for car like robots. Subscribes to geometry_msgs/twist

class AckermannDrive : public gazebo::ModelPlugin
{
public:
  /// Constructor
  AckermannDrive();

  /// Destructor
  ~AckermannDrive();

protected:
  // Documentation inherited
  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

  // Documentation inherited
  void Reset() override;

private:
  /// Private data pointer
  std::unique_ptr<AckermannDrivePrivate> impl_;
};
}  // namespace gazebo_plugins

#endif  // GAZEBO_PLUGINS__GAZEBO_ROS_ACKERMANN_DRIVE_HPP_
