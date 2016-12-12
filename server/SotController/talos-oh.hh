#ifndef _Talos_OH_HH_
#define _Talos_OH_HH_

#include <string>
#include <vector>
#include <map>

#include <hrpCorba/Controller.hh>
#include <hrpCorba/ViewSimulator.hh>
#include <hrpCorba/DynamicsSimulator.hh>


/** \brief Config variables
 */
struct robot_config_t
{
  /// \brief Name of the controller to load
  std::string libname;
  /// \brief Name of the pid gains to load
  std::string pid_gains_filename;
  /// \brief Robot number of DoFs
  int nb_dofs;
  /// \brief Number of force sensors
  int nb_force_sensors;
  
};

class TalosOH
{
public:
  TalosOH();
  ~TalosOH();

  virtual void initialize();
  void start();
  virtual void input();
  void output();
  virtual void control();
  void stop();
  void restart();
  void shutdown();
  void set_path_to_pid_gains(std::string &);
  OpenHRP::DynamicsSimulator_ptr dynamicsSimulator_;

  std::string modelName_;

protected:

  /// Initialize position and velocity references.
  void initref();

  /// Initialize internal pids of the robot
  void initpids();

  /// Initialize list of link names.
  void initLinkNames();

  /// Read the input values for bushes.
  void input_bushs();

  /// Write the output values for bushes.
  void output_bushs();

  /// Save logs.
  void save_logs();

  /// Store the PID values for each actuator.
  std::vector<double> P_,I_,D_;

  /// Number of degrees of freedom.
  unsigned int nb_dofs_;
  
  /// Store angle values, speed and torque.
  std::vector<double> q_,dq_, qref_, dqref_, ddqref_, pqref_, pdqref_;
  std::vector<double> torques_, forces_;
  std::vector<double> bush_qref_,bush_dqref_,bush_ddqref_;
  std::vector<double> accelerometer_,gyrometer_;
  std::vector<double> angleControl_;
  double dt_;

  OpenHRP::DblSequence sendto_,recvfrom_;

  /// Periodicity of control.
  unsigned int control_ticks_, control_ticks_max_;

  /// Boolean 
  bool perform_control_;

  /// list of link names.
  std::vector<std::string> listOfLinks_;
  std::vector<std::string> listOfBushs_;

  virtual void localControl();

  /// \brief Config variables 
  robot_config_t robot_config_;

  /// Cumulative error.
  double cumul_err_;
};
#endif /* */
