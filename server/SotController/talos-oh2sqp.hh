#ifndef _HRP2_OH_2_SOT_HH_
#define _HRP2_OH_2_SOT_HH_

#include "talos-oh.hh"


class TALOSOH2SQP : public TalosOH 
{
public:
  TALOSOH2SQP();
  ~TALOSOH2SQP();

  // Reimplements the TALOSOH interface.
  void initialize();
  void input();
  void control();

  // Specific interface to this class.

  // Path to the directory where is the trajectories to play back.
  void set_path_to_data(std::string &);
  
protected:

  // Loading the pos data.
  void loadData();
    
  virtual void localControl();

  /// \brief Config variables 
  robot_config_t robot_config_;

  typedef std::vector<double> ConfigurationVector;
  typedef std::vector<ConfigurationVector> Path;

  Path m_RobotPath;

  std::string m_PathToData;

  unsigned long int nb_it_;
};
#endif /* */
