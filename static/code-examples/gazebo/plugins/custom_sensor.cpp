/**
 * Custom Gazebo Sensor Plugin Example
 *
 * This plugin demonstrates how to create a custom sensor for Gazebo.
 * Compile with: catkin_make or colcon build
 */

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
  /// \brief Custom sensor plugin
  class CustomSensorPlugin : public SensorPlugin
  {
    /// \brief Constructor
    public: CustomSensorPlugin() : SensorPlugin()
    {
    }

    /// \brief Destructor
    public: virtual ~CustomSensorPlugin()
    {
      this->parentSensor.reset();
    }

    /// \brief Load the plugin
    public: virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
    {
      // Get the parent sensor
      this->parentSensor = _sensor;

      // Connect to sensor update event
      this->updateConnection = this->parentSensor->ConnectUpdated(
          std::bind(&CustomSensorPlugin::OnUpdate, this));

      // Activate sensor
      this->parentSensor->SetActive(true);

      gzlog << "Custom sensor plugin loaded" << std::endl;
    }

    /// \brief Update callback
    private: void OnUpdate()
    {
      // Read sensor data
      // Process sensor data
      // Publish to ROS topic (if integrated)

      gzlog << "Sensor update at time: "
            << this->parentSensor->LastUpdateTime()
            << std::endl;
    }

    /// \brief Pointer to parent sensor
    private: sensors::SensorPtr parentSensor;

    /// \brief Update connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register plugin
  GZ_REGISTER_SENSOR_PLUGIN(CustomSensorPlugin)
}
