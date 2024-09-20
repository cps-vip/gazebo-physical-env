#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

//based on examples from https://github.com/srmainwaring


namespace gazebo
{
  class MagneticFieldPlugin : public ModelPlugin
  {
  public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
    {
      ignition::math::Vector3d magneticField(0, 0, 0.00005); //TODO changeme later
      this->model = _model;
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&MagneticFieldPlugin::OnUpdate, this));
    }

    void OnUpdate()
    {
      // calc and apply magnetic force
      ignition::math::Vector3d force = magneticField * this->model->WorldPose().Pos(); 
      this->model->GetLink()->AddForce(force);
      // should add to model
    }

  private:
    physics::ModelPtr model;
    event::ConnectionPtr updateConnection;
    ignition::math::Vector3d magneticField; //gz behavior
  };

  GZ_REGISTER_MODEL_PLUGIN(MagneticFieldPlugin)
}
