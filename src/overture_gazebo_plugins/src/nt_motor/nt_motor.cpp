//
// Created by ajahueym on 1/10/24.
//
#include <frc/system/plant/DCMotor.h>
#include <ntcore/networktables/NetworkTableInstance.h>
#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <cmath>
#include <overture_filters/low_pass_filter/low_pass_filter.h>

#define  SIM_UPDATE_PERIOD 0.001

namespace gazebo {
    class NTMotorPlugin : public ModelPlugin {
    public:
        NTMotorPlugin(): ModelPlugin() {
        }

        void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override {
            this->model = model;

            if(!ros::isInitialized()){
                ROS_FATAL_STREAM("Unable to load NTMotorPlugin, ROS is not initialized");
                return;
            }

            if(model->GetJointCount() == 0) {
                ROS_FATAL_STREAM("Unable to load NTMotorPlugin given to model with no joints!!!");
                return;
            }

            std::string modelName = model->GetName();
            std::string jointName;
            if (sdf->HasElement("joint_name")) {
                jointName = sdf->Get<std::string >("joint_name");
            }

            if(jointName.empty()){
                ROS_FATAL("NTMotorPlugin used in %s not given a joint_name!!!!", modelName.c_str());
                return;
            }


            controlledJoint = model->GetJoint(jointName);


            if(!controlledJoint) {
                ROS_FATAL("Joint %s does not exist in model %s!!!", jointName.c_str(), modelName.c_str());
                return;
            }

            childLink = controlledJoint->GetChild();

            if(sdf->HasElement("joint_axis")){
                jointAxis = sdf->Get<unsigned int>("joint_axis");
            }

            std::string motorModel;
            if (sdf->HasElement("motor_model")) {
                motorModel = sdf->Get<std::string >("motor_model");
            }

            if(motorModel.empty()){
                ROS_FATAL("NTMotorPlugin used in %s not given a FRC motor model!!!!", modelName.c_str());
                return;
            }

            if (sdf->HasElement("gear_ratio")) {
                gear_ratio = sdf->Get<double >("gear_ratio");
            }

            if (sdf->HasElement("motor_count")) {
                motor_count = sdf->Get<int>("motor_count");
            }

            if (sdf->HasElement("torque_axis")) {
                torque_axis = sdf->Get<char>("torque_axis");
            }

            if(sdf->HasElement("mechanically_inverted")){
                mechanically_inverted = sdf->Get<bool>("mechanically_inverted");
            }

            if(motorModel == "Kraken") {
                motor = frc::DCMotor::KrakenX60(motor_count);
            } else if (motorModel == "NEO") {
                motor = frc::DCMotor::NEO(motor_count);
            } else if (motorModel == "Falcon"){
                motor = frc::DCMotor::Falcon500(motor_count);
            } else if (motorModel == "FalconFOC"){
                motor = frc::DCMotor::Falcon500FOC(motor_count);
            } else if (motorModel == "Vortex"){
                motor = frc::DCMotor::NeoVortex(motor_count);
            } else {
                ROS_FATAL("NTMotorPlugin used invalid FRC motor model %s!!!!", motorModel.c_str());
                return;
            }

            motor = motor.WithReduction(gear_ratio);

            /// We have the joint that is to be controlled, setup nt listeners and update
            updateConnection = event::Events::ConnectWorldUpdateBegin([this](auto && PH1) { Update(std::forward<decltype(PH1)>(PH1)); });

            ntInst = nt::NetworkTableInstance::GetDefault();
            ntInst.SetServer("127.0.0.1");
            std::stringstream  ntIdentity;
            ntIdentity << "nt_motor_plugin_" << modelName << "_" << jointName;
            ntInst.StartClient4(ntIdentity.str());

            std::string modelScopedName = model->GetScopedName();

            std::string const result = std::regex_replace( modelScopedName, std::regex( "\\::" ), "/" );

            const auto ntable = ntInst.GetTable(result)->GetSubTable("motors")->GetSubTable(jointName);
            encoderSpeedEntry = ntable->GetEntry("encoder_speed");
            encoderPositionEntry = ntable->GetEntry("encoder_position");
            voltageEntry = ntable->GetEntry("voltage_applied");
            currentEntry = ntable->GetEntry("current");
            torqueAppliedEntry = ntable->GetEntry("torque");
            invertedEntry = ntable->GetEntry("inverted");

            encoderSpeedEntry.SetDouble(0);
            encoderPositionEntry.SetDouble(0);
            voltageEntry.SetDouble(0);
            currentEntry.SetDouble(0);
            torqueAppliedEntry.SetDouble(0);
            invertedEntry.SetBoolean(false);
        }

        void Init() override{
            initialPos = controlledJoint->Position(jointAxis);
        }

        void Update(const common::UpdateInfo& info) {
            const auto currentSimTime = model->GetWorld()->SimTime();

            if(currentSimTime.Double() - lastUpdateSimTime.Double() < SIM_UPDATE_PERIOD){
                return;
            }
            lastUpdateSimTime = currentSimTime;

            double jointTurns = (controlledJoint->Position(jointAxis) - initialPos) / (2.0 * M_PI);
            auto appliedVoltage = units::volt_t (voltageEntry.GetDouble(0));

            double jointTurnsPerS = lowPassFilter.Update((jointTurns - lastPos) / SIM_UPDATE_PERIOD);
            lastPos = jointTurns;

            if(mechanically_inverted) {
                appliedVoltage *= -1;
            }

            auto current = motor.Current(units::radians_per_second_t(jointTurnsPerS * 2.0 * M_PI), appliedVoltage);

            auto torqueGenerated = motor.Torque(current);

            double torqueToApply = torqueGenerated.value();
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

            childLink->AddRelativeTorque(torque);


            if(mechanically_inverted) {
                jointTurns *= -1;
                jointTurnsPerS *= -1;
                current *= -1;
            }

            double rotations = jointTurns * gear_ratio;
            encoderPositionEntry.SetDouble(rotations);

            double encoderSpeed = jointTurnsPerS * gear_ratio;
            encoderSpeedEntry.SetDouble(encoderSpeed);

            torqueAppliedEntry.SetDouble(torqueToApply);
            currentEntry.SetDouble(current.value());

            ntInst.Flush();
        }

    private:
        frc::DCMotor motor = frc::DCMotor(0_V, 0_Nm, 0_A, 0_A, 0_rad_per_s);
        physics::JointPtr controlledJoint;
        physics::LinkPtr childLink;
        unsigned int jointAxis = 0;
        physics::ModelPtr model;
        event::ConnectionPtr updateConnection;
        nt::NetworkTableInstance ntInst;
        nt::NetworkTableEntry encoderSpeedEntry, encoderPositionEntry, voltageEntry, currentEntry, torqueAppliedEntry, invertedEntry;

        LowPassFilter lowPassFilter {25, SIM_UPDATE_PERIOD};
        common::Time lastUpdateSimTime;
        double lastPos = 0;

        double initialPos = 0;

        double gear_ratio = 1.0;
        int motor_count = 1;
        char torque_axis = 'z';
        bool mechanically_inverted = false;
    };


    GZ_REGISTER_MODEL_PLUGIN(NTMotorPlugin);
}