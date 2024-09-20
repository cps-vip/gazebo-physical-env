#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

//based on examples from https://github.com/srmainwaring

namespace gazebo
{
    // Derived from the ModelPlugin class
    class MagneticField : public ModelPlugin
    {
    public:
        // load function - for when the plugin is inserted into the simulation
        void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
        {
            // set a magnetic field vector
            ignition::math::Vector3d magneticField(0, 0, 0.00005); //TODO changeme later
            this->model = _model;
            // connect to the world update event
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                    std::bind(&MagneticFieldPlugin::OnUpdate, this));
        }

        // called on every timestep (?)
        void OnUpdate()
        {
            // calc the magnetic force based on the model's position
            ignition::math::Vector3d force = magneticField * this->model->WorldPose().Pos(); 
            // apply the force to the model
            this->model->GetLink()->AddForce(force);
        }

    private:
        physics::ModelPtr model; // Pointer to the model
        event::ConnectionPtr updateConnection; // Pointer to the update event connection
        ignition::math::Vector3d magneticField; // Magnetic field vector
    };
    GZ_REGISTER_MODEL_PLUGIN(MagneticFieldPlugin)
}
