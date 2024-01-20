//
// Created by ajahueym on 1/14/24.
//
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ntcore/networktables/NetworkTableInstance.h>

namespace gazebo {
    class LinkedJoint : public ModelPlugin {
    public:
        LinkedJoint(): ModelPlugin() {
        }

        void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override {
            this->model = model;

            std::string motorName;
            if (!sdf->HasElement("motor_name")) {
                throw std::invalid_argument("LinkedJoint plugin not given a source motor name!");
            }

            motorName = sdf->Get<std::string>("motor_name");


            std::string targetJointName;
            if (!sdf->HasElement("joint_target_name")) {
                throw std::invalid_argument("LinkedJoint plugin not given a target joint name!");
            }

            targetJointName = sdf->Get<std::string>("joint_target_name");

            targetJoint = model->GetJoint(targetJointName);
            targetLink = targetJoint->GetChild();

            if(!targetJoint) {
                throw std::invalid_argument("LinkedJoint plugin given a joint that does not exist");
            }

            if(sdf->HasElement("inverted")){
                inv_mul = -1.0;
            }

            if (sdf->HasElement("torque_axis")) {
                torque_axis = sdf->Get<char>("torque_axis");
            }

            updateConnection = event::Events::ConnectWorldUpdateEnd([this] { Update(); });

            ntInst = nt::NetworkTableInstance::GetDefault();
            ntInst.SetServer("127.0.0.1");
            std::stringstream  ntIdentity;
            ntIdentity << "nt_linked_plugin_" << motorName << "_" << targetJointName;
            ntInst.StartClient4(ntIdentity.str());

            std::string modelScopedName = model->GetScopedName();

            std::string const result = std::regex_replace( modelScopedName, std::regex( "\\::" ), "/" );

            const auto ntable = ntInst.GetTable(result)->GetSubTable("motors")->GetSubTable(motorName);
            torqueAppliedEntry = ntable->GetEntry("torque");
        }

        void Update() {
            double torqueToApply = torqueAppliedEntry.GetDouble(0);
            ignition::math::Vector3d torque;

            switch (torque_axis) {
                case 'x':
                    torque =  {torqueToApply, 0, 0};
                    break;
                case 'y':
                    torque =  {0, torqueToApply, 0};
                    break;
                case 'z':
                    torque =  {0, 0, torqueToApply};
                    break;
                default:
                    torque =  {0, 0, 0};
                    break;

            }

            targetLink->AddRelativeTorque(torque * inv_mul);
        }

    private:
        physics::ModelPtr model;
        event::ConnectionPtr updateConnection;

        physics::JointPtr targetJoint;
        physics::LinkPtr targetLink;
        double inv_mul = 1.0;

        nt::NetworkTableInstance ntInst;
        nt::NetworkTableEntry torqueAppliedEntry;
        char torque_axis = 'z';

    };

    GZ_REGISTER_MODEL_PLUGIN(LinkedJoint);
}//
// Created by ajahueym on 1/19/24.
//
