//
// Created by ajahueym on 1/14/24.
//
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableEntry.h>
#include <overture_filters/low_pass_filter/low_pass_filter.h>

namespace gazebo {
    class NTCANCoder : public ModelPlugin {
    public:
        NTCANCoder(): ModelPlugin() {
        }

        void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override {
            std::string jointName;
            if (!sdf->HasElement("joint_name")) {
                throw std::invalid_argument("NTCANCoder plugin not given a joint name!");
            }

            jointName = sdf->Get<std::string>("joint_name");

            sourceJoint = model->GetJoint(jointName);

            if(!sourceJoint) {
                throw std::invalid_argument("NTCANCoder plugin given a joint that does not exist");
            }

            updateConnection = event::Events::ConnectWorldUpdateEnd([this] { Update(); });

            ntInst = nt::NetworkTableInstance::GetDefault();
            ntInst.SetServer("127.0.0.1");
            std::stringstream  ntIdentity;
            ntIdentity << "nt_cancoder_plugin_" << model->GetName() << "_" << jointName;
            ntInst.StartClient4(ntIdentity.str());

            std::string modelScopedName = model->GetScopedName();

            std::string const result = std::regex_replace( modelScopedName, std::regex( "\\::" ), "/" );


            const auto ntable = ntInst.GetTable(result)->GetSubTable("cancoders")->GetSubTable(jointName);
            encoderSpeedEntry = ntable->GetEntry("cancoder_speed");
            encoderPositionEntry = ntable->GetEntry("cancoder_position");

            encoderSpeedEntry.SetDouble(0);
            encoderPositionEntry.SetDouble(0);
        }

        void Update() {
            encoderPositionEntry.SetDouble(sourceJoint->Position() / ( 2 * M_PI));

            double encoderSpeed = speedLPF.Update(sourceJoint->GetVelocity(0)) / (2.0 * M_PI);
            encoderSpeedEntry.SetDouble(encoderSpeed);
        }

    private:
        event::ConnectionPtr updateConnection;
        physics::JointPtr sourceJoint;
        nt::NetworkTableInstance ntInst;
        nt::NetworkTableEntry encoderSpeedEntry, encoderPositionEntry;

        LowPassFilter speedLPF {25,  1.0 / 1000.0};
    };

    GZ_REGISTER_MODEL_PLUGIN(NTCANCoder);
}