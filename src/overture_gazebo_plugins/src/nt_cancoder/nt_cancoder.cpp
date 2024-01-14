//
// Created by ajahueym on 1/14/24.
//
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableEntry.h>

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

            const auto ntable = ntInst.GetTable(model->GetName())->GetSubTable(jointName);
            encoderSpeedEntry = ntable->GetEntry("cancoder_speed");
            encoderPositionEntry = ntable->GetEntry("cancoder_position");

            encoderSpeedEntry.SetDouble(0);
            encoderPositionEntry.SetDouble(0);
        }

        void Update() {
            encoderPositionEntry.SetDouble(sourceJoint->Position() / ( 2 * M_PI));

            double newEncoderSpeed = sourceJoint->GetVelocity(0)  / (2.0 * M_PI);
            double encoderSpeed = beta * lastSpeed + (1 - beta)*newEncoderSpeed;
            lastSpeed = encoderSpeed;

            encoderSpeedEntry.SetDouble(encoderSpeed);
        }

    private:
        event::ConnectionPtr updateConnection;
        physics::JointPtr sourceJoint;
        nt::NetworkTableInstance ntInst;
        nt::NetworkTableEntry encoderSpeedEntry, encoderPositionEntry;

        double lastSpeed = 0;
        const double beta = std::exp(-1 * 25 * 0.001);

    };

    GZ_REGISTER_MODEL_PLUGIN(NTCANCoder);
}