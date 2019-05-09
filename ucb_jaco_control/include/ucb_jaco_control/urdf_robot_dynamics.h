#ifndef UCB_JACO_CONTROL_URDF_ROBOT_DYNAMICS_H
#define UCB_JACO_CONTROL_URDF_ROBOT_DYNAMICS_H

#include <ucb_jaco_control/robot_dynamics.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>
#include <urdf/model.h>
#include <exception>
#include <iostream>

namespace ucb_jaco_control
{

template <unsigned int DOF>
class URDFRobotDynamics : public RobotDynamics<DOF>
{
public:
  typedef Eigen::Matrix<double, DOF, DOF> Matrix;
  typedef Eigen::Matrix<double, DOF, 1>   Vector;

  URDFRobotDynamics(const std::string& urdf_file,
                    const std::string& root_name,
                    const std::string& tip_name)
    : root_name_(root_name), tip_name_(tip_name), dynamic_params_(nullptr)
  {
    if (!urdf_model_.initFile(urdf_file))
      throw std::invalid_argument("Could not read URDF file " + urdf_file);

    std::vector<urdf::LinkSharedPtr> links;
    urdf_model_.getLinks(links);
    for (urdf::LinkSharedPtr l : links)
      std::cout << l->name << std::endl;

    buildDynamicsModel();
  }

  double getLinkMass(const std::string& link_name) const
  {
    auto link = urdf_model_.getLink(link_name);
    if (link)
      return link->inertial->mass;

    throw std::invalid_argument("Could not get link " + link_name);
  }

  void setLinkMass(const std::string& link_name, double mass)
  {
    urdf::LinkSharedPtr link;
    urdf_model_.getLink(link_name, link);
    if (link)
      link->inertial->mass = mass;
    else
      throw std::invalid_argument("Could not get link " + link_name);

    // Need to update the dynamics model.
    buildDynamicsModel();
  }

  void setLinkMasses(const std::vector<std::string>& link_names,
                     const std::vector<double>&      masses)
  {
    // TODO Check to make sure link_masses.size() == masses.size().
    for (int i = 0; i < link_names.size(); ++i)
    {
      urdf::LinkSharedPtr link;
      urdf_model_.getLink(link_names[i], link);
      if (link)
        link->inertial->mass = masses[i];
      else
        throw std::invalid_argument("Could not get link " + link_names[i]);
    }

    // Need to update the dynamics model.
    buildDynamicsModel();
  }

  ~URDFRobotDynamics()
  {
    delete dynamic_params_;
    dynamic_params_ = nullptr;
  }

  Matrix getInertiaMatrix(const Vector& pos)
  {
    KDL::JntArray pos_array(DOF);
    pos_array.data = pos;

    KDL::JntSpaceInertiaMatrix M(DOF);
    int result = dynamic_params_->JntToMass(pos_array, M);
    // TODO: Check for errors.

    return Eigen::Ref<Matrix>(M.data);
  }

  Vector getCoriolisVector(const Vector& pos,
                           const Vector& vel)
  {
    KDL::JntArray pos_array(DOF);
    pos_array.data = pos;

    KDL::JntArray vel_array(DOF);
    vel_array.data = pos;

    KDL::JntArray C(DOF);
    int result = dynamic_params_->JntToCoriolis(pos_array,
                                                vel_array,
                                                C);
    // TODO: Check for errors.

    return Eigen::Ref<Vector>(C.data);
  }

  Vector getGravityVector(const Vector& pos)
  {
    KDL::JntArray pos_array(DOF);
    pos_array.data = pos;

    KDL::JntArray G(DOF);
    int result = dynamic_params_->JntToGravity(pos_array, G);
    // TODO: Check for errors.

    return Eigen::Ref<Vector>(G.data);
  }

  virtual Vector getFrictionVector(const Vector& vel)
  {
    // Subclass to add custom frictional forces.
    return Vector::Zero();
  }

private:
  void buildDynamicsModel()
  {
    KDL::Tree tree;
    if (!kdl_parser::treeFromUrdfModel(urdf_model_, tree))
      throw std::invalid_argument("Could not parse KDL from URDF model");

    KDL::Chain chain;
    if (!tree.getChain(root_name_, tip_name_, chain))
    {
      throw std::invalid_argument("Could not get chain with root " + root_name_ +
                                  " and tip " + tip_name_);
    }

    // TODO: Do we need to delete and reconstruct?
    if (dynamic_params_)
    {
      delete dynamic_params_;
      dynamic_params_ = nullptr;
    }

    // TODO: Is this correct? Make into a parameter.
    KDL::Vector gravity(0.0, 0.0, -9.8);
    dynamic_params_ = new KDL::ChainDynParam(chain, gravity);
  }

  urdf::Model         urdf_model_;
  std::string         root_name_;
  std::string         tip_name_;
  KDL::ChainDynParam* dynamic_params_;

};

} // namespace ucb_jaco_control

#endif // UCB_JACO_CONTROL_URDF_ROBOT_DYNAMICS_H
