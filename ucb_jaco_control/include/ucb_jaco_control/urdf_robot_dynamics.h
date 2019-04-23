#ifndef UCB_JACO_CONTROL_URDF_ROBOT_DYNAMICS_H
#define UCB_JACO_CONTROL_URDF_ROBOT_DYNAMICS_H

#include <ucb_jaco_control/robot_dynamics.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>
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
  {
    KDL::Tree tree;
    if (!kdl_parser::treeFromFile(urdf_file, tree))
      throw std::invalid_argument("Could not read URDF file " + urdf_file);

    // for (auto s : tree.getSegments())
    //   std::cout << s.first << std::endl;

    KDL::Chain chain;
    if (!tree.getChain("j2s7s300_link_base",
                       "j2s7s300_end_effector",
                       chain))
    {
      throw std::invalid_argument("Could not get chain with root " + root_name +
                                  " and tip " + tip_name);
    }

    // TODO: Is this correct? Make into a parameter.
    KDL::Vector gravity(0.0, 0.0, -9.8);
    dynamic_params_ = new KDL::ChainDynParam(chain, gravity);
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
  KDL::ChainDynParam *dynamic_params_;

};

} // namespace ucb_jaco_control

#endif // UCB_JACO_CONTROL_URDF_ROBOT_DYNAMICS_H
