//
// Created by ajahueym on 1/11/24.
//
#include <frc/system/plant/DCMotor.h>
#include <ntcore/networktables/NetworkTableInstance.h>
#include <ntcore/networktables/DoubleTopic.h>
#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

namespace gazebo {
    class NTSimWorldTelemetryPlugin : public WorldPlugin {
    public:
        NTSimWorldTelemetryPlugin(): WorldPlugin() {
        }

        void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override {
            this->world = _world;

            ntInst = nt::NetworkTableInstance::GetDefault();
            ntInst.SetServer("127.0.0.1");
            std::stringstream  ntIdentity;
            ntIdentity << "nt_simworld_" << world->Name();
            ntInst.StartClient4(ntIdentity.str());

            const auto ntable = ntInst.GetTable("nt_simworld");
            simTimeEntry = ntable->GetDoubleTopic("sim_time").Publish();

            updateConnection = event::Events::ConnectWorldUpdateEnd([this] { Update(); });
        }

        void Update() {
            simTimeEntry.Set(world->SimTime().Double());
            ntInst.Flush();
        }

    private:
        physics::WorldPtr world;
        event::ConnectionPtr updateConnection;
        nt::NetworkTableInstance ntInst;
        nt::DoublePublisher simTimeEntry;
    };


    GZ_REGISTER_WORLD_PLUGIN(NTSimWorldTelemetryPlugin);
}