//
// Created by ajahueym on 1/14/24.
//

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/ImuSensor.hh>
#include <gazebo/common/common.hh>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableEntry.h>

#define RADS_TO_DEGREES (180.0 / M_PI)

namespace gazebo {
    class NTIMUPlugin : public SensorPlugin {
    public:
        NTIMUPlugin(): SensorPlugin() {
        }

        void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) override {

            if(_sensor->Type() != "imu"){
                throw std::invalid_argument("NTIMUPlugin is meant for sensors of type \"imu\"!!!");
            }

            std::string robotName;
            if(!_sdf->HasElement("robot_name")){
                throw std::invalid_argument("NTIMUPlugin needs the robot's name!!!");
            }


            imuSensor = dynamic_cast<sensors::ImuSensor*>(_sensor.get());

            updateConnection = imuSensor->ConnectUpdated([this] { Update(); });

            ntInst = nt::NetworkTableInstance::GetDefault();
            ntInst.SetServer("127.0.0.1");
            std::stringstream  ntIdentity;
            ntIdentity << "nt_imu_plugin_" << imuSensor->Name();
            ntInst.StartClient4(ntIdentity.str());

            std::string imuScopedName = imuSensor->ScopedName();
            std::string const result = std::regex_replace( imuScopedName, std::regex( "\\::" ), "[/]\n" );


            const auto ntable = ntInst.GetTable(result);
            rollEntry = ntable->GetEntry("roll");
            pitchEntry = ntable->GetEntry("pitch");
            yawEntry = ntable->GetEntry("yaw");
        }

        void Update() {
            const auto orientation = imuSensor->Orientation();
            rollEntry.SetDouble(orientation.Roll() * RADS_TO_DEGREES);
            pitchEntry.SetDouble(orientation.Pitch() * RADS_TO_DEGREES);
            yawEntry.SetDouble(orientation.Yaw() * RADS_TO_DEGREES);

            ntInst.Flush();
        }

    private:
        sensors::ImuSensor* imuSensor = nullptr;
        event::ConnectionPtr updateConnection;

        nt::NetworkTableInstance ntInst;
        nt::NetworkTableEntry rollEntry, pitchEntry, yawEntry;
    };


    GZ_REGISTER_SENSOR_PLUGIN(NTIMUPlugin);
}