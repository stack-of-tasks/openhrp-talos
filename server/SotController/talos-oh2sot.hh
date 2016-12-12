#ifndef _TALOS_OH_2_SOT_HH_
#define _TALOS_OH_2_SOT_HH_

#include "talos-oh.hh"

#include <sot/core/abstract-sot-external-interface.hh>
#include <dynamic_graph_bridge/sot_loader_basic.hh>

namespace dgsot=dynamicgraph::sot;

class TALOSOH2SOT : public SotLoaderBasic, public TalosOH 
{
public:
  TALOSOH2SOT();
  ~TALOSOH2SOT();

  // Reimplements the TALOSOH interface.
  void initialize();
  void input();
  void control();

  // Specific interface to this class.

  // Path to the SoT library to load.
  void set_path_to_library(std::string &);
  
protected:


  /// Initialize the stack of tasks.
  void initializeSoT();

  virtual void localControl();

  /// \brief the sot-talos controller
  dgsot::AbstractSotExternalInterface * m_sotController;

  void fillSensors(std::map<std::string,dgsot::SensorValues> &sensorsIn);
  void readControl(std::map<std::string,dgsot::ControlValues> &controlValues);

  /// \brief Config variables 
  robot_config_t robot_config_;

  /// Map of sensor readings
  std::map<std::string,dgsot::SensorValues> sensorsIn_;
  /// Map of control values
  std::map<std::string,dgsot::ControlValues> controlValues_;

};
#endif /* */
