#include <iostream>
#include <fstream>
#include <iomanip>
#include <dlfcn.h>

#include "talos-oh2sot.hh"

#if DEBUG
#define ODEBUG(x) std::cout << x << std::endl
#else
#define ODEBUG(x)
#endif
#define ODEBUG3(x) std::cout << x << std::endl

#define DBGFILE "/tmp/talosoh2sot.dat"

#define RESETDEBUG5() { std::ofstream DebugFile;	\
    DebugFile.open(DBGFILE,std::ofstream::out);		\
    DebugFile.close();}
#define ODEBUG5FULL(x) { std::ofstream DebugFile;	\
    DebugFile.open(DBGFILE,std::ofstream::app);		\
    DebugFile << __FILE__ << ":"			\
	      << __FUNCTION__ << "(#"			\
	      << __LINE__ << "):" << x << std::endl;	\
    DebugFile.close();}
#define ODEBUG5(x) { std::ofstream DebugFile;	\
    DebugFile.open(DBGFILE,std::ofstream::app); \
    DebugFile << x << std::endl;		\
    DebugFile.close();}



TALOSOH2SOT::TALOSOH2SOT()
{

  SotLoaderBasic();
  TalosOH();
}

TALOSOH2SOT::~TALOSOH2SOT()
{

}

void TALOSOH2SOT::initializeSoT()
{
  char * sLD_LIBRARY_PATH;
  sLD_LIBRARY_PATH=getenv("LD_LIBRARY_PATH");
  ODEBUG5("LoadSot  Start " << sLD_LIBRARY_PATH);
  char * sPYTHONPATH;
  sPYTHONPATH=getenv("PYTHONPATH");
  ODEBUG5("PYTHONPATH:" << sPYTHONPATH );
  sPYTHONPATH=getenv("PYTHON_PATH");
  ODEBUG5("PYTHON_PATH:" << sPYTHONPATH);

  ODEBUG5("Trying to load the library:" << robot_config_.libname);
  // Load the SotTALOSController library.
  void * SotTALOSControllerLibrary = dlopen(robot_config_.libname.c_str(),
                                           RTLD_GLOBAL | RTLD_NOW);
  if (!SotTALOSControllerLibrary) {
    ODEBUG5("Cannot load library: " << dlerror() );
    return ;
  }
  ODEBUG5("Success in loading the library:" << robot_config_.libname);
  // reset errors
  dlerror();
  
  // Load the symbols.
  createSotExternalInterface_t * createTALOSController =
    (createSotExternalInterface_t *) dlsym(SotTALOSControllerLibrary, 
                                           "createSotExternalInterface");
  ODEBUG5("createHRPController call "<< std::hex
          << std::setbase(10));
  const char* dlsym_error = dlerror();
  if (dlsym_error) {
    ODEBUG5("Cannot load symbol create: " << dlsym_error );
    return ;
  }
  ODEBUG5("Success in getting the controller factory");
  
  // Create taloscontroller
  try 
    {
      ODEBUG5("exception handled createTALOSController call "<< std::hex 
              << std::setbase(10));
      m_sotController = createTALOSController();
      ODEBUG5("After createTALOSController.");

    } 
  catch (std::exception &e)
    {
      ODEBUG5("Exception: " << e.what());
    }
  ODEBUG5("LoadSot - End");
}


void TALOSOH2SOT::initialize()
{
  int argc=1;
  char * argv[1];
  argv[0] = "talosohsot";

  initref();
  initpids();
  initLinkNames();
  initializeSoT();
  SotLoaderBasic::initializeRosNode(argc,argv);
}


void TALOSOH2SOT::input()
{
  TalosOH::input();
  fillSensors(sensorsIn_);
  //input_bushs();
  save_logs();
}

void TALOSOH2SOT::control()
{
  /*   std::cout << "perform_control_ :" << perform_control_ 
	    << "dynamic_graph_stopped_ :" << dynamic_graph_stopped_ 
	    << std::endl; */
  if (perform_control_ & !dynamic_graph_stopped_)
    {
      if (control_ticks_==0)
	{
	  try
	    {
	      m_sotController->setupSetSensors(sensorsIn_);
	      m_sotController->getControl(controlValues_);
	    } 
	  catch (std::exception &e) 
	    {  ODEBUG5("Exception on Execute: " << e.what());throw e; }
	  readControl(controlValues_);
	}
      control_ticks_++;
      if (control_ticks_==control_ticks_max_)
	control_ticks_ = 0;
    }

  for(unsigned int i=0;i<nb_dofs_;i++)
    {
      
      //std::cout << i << " : (" << q_[i] << " , " << qref_[i]  << ")" << std::endl;
      double error = q_[i] - qref_[i];
      cumul_err_ -= error;
      torques_[i] = -(q_[i] - qref_[i] ) *P_[i] 
	//+ cumul_err_ * I_[i]
	- (dq_[i] -dqref_[i]) * D_[i];
      //      torques_[i] =1.0;
    }
}


void 
TALOSOH2SOT::fillSensors(std::map<std::string,dgsot::SensorValues> & 
			sensorsIn)
{
  OpenHRP::SensorState_var aSensorState;
  dynamicsSimulator_->getCharacterSensorState(modelName_.c_str(),aSensorState);

  // Fill in the positions values.
  sensorsIn["joints"].setValues(q_);

  // Update forces
  if (aSensorState->force.length()==4)
    {
      sensorsIn["forces"].setName("force");
      
      forces_.resize(4*6);
      for(unsigned int i=0;i<4;i++)
	{
	  for(unsigned int j=0;j<6;j++)
	    {
	      forces_[i*6+j] = aSensorState->force[i][j];
	    }
	}
      
      sensorsIn["forces"].setValues(forces_);
    }

  // Update torque
  sensorsIn["torques"].setName("torque");
  if (aSensorState->u.length()!=0)
    {
      torques_.resize(aSensorState->u.length());
      for (unsigned int j = 0; j < aSensorState->u.length(); ++j)
        torques_[j] = aSensorState->u[j];
    }
  else 
    {
      torques_.resize(robot_config_.nb_dofs);
      for(unsigned int i=0;i<(unsigned int)robot_config_.nb_dofs;i++)
        torques_[i] = 0.0;
    }
  sensorsIn["torques"].setValues(torques_);

  // Update attitude
  // sensorsIn["attitude"].setName("attitude");
  // if (aSensorState.rateGyro.length()!=0)
  //   {
  //     baseAtt_.resize(aSensorState.rateGyro.length());
  //     for (unsigned int j = 0; j < aSensorState.rateGyro.length(); ++j)
  //       baseAtt_ [j] = aSensorState.rateGyro[j];
  //   }
  // else
  //   {
  //     baseAtt_.resize(9);
  //     for(unsigned int i=0;i<3;i++)
  //       {
  //         for(unsigned int j=0;j<3;j++)
  //           {
  //             if (i==j)
  //               baseAtt_[i*3+j]=1.0;
  //             else 
  //               baseAtt_[i*3+j]=0.0;
  //           }
  //       }
  //   }
  // sensorsIn["attitude"].setValues (baseAtt_);

  // Update accelerometer
  sensorsIn["accelerometer_0"].setName("accelerometer_0");
  if (aSensorState->accel.length()!=0)
    {
      accelerometer_.resize(3);
      for (unsigned int j = 0; j <3; ++j)
        accelerometer_[j] = aSensorState->accel[0][j];
    }
  else 
    {
      accelerometer_.resize(3);
      for(unsigned int i=0;i<3;i++)
        accelerometer_[i] = 0.0;
    }  
  sensorsIn["accelerometer_0"].setValues(accelerometer_);
  
  // Update gyrometer
  sensorsIn["gyrometer_0"].setName("gyrometer_0");
  if (aSensorState->rateGyro.length()!=0)
    {
      gyrometer_.resize(3);
      for (unsigned int j = 0; j < 3; ++j)
        gyrometer_[j] = aSensorState->rateGyro[0][j];
    }
  else 
    {
      gyrometer_.resize(3);
      for(unsigned int i=0;i<3;i++)
        gyrometer_[i] = 0.0;
    }  
  sensorsIn["gyrometer_0"].setValues(gyrometer_);
}

void 
TALOSOH2SOT::readControl(std::map<std::string,dgsot::ControlValues> 
			&controlValues)
{
  double R[9];

  // Update joint values.
  angleControl_ = controlValues["joints"].getValues();
  
  for(unsigned int i=0;i<angleControl_.size();i++)
    { 
      // Update qref from SoT.
      // dqref and ddref should/could be provided by SoT too.
      qref_[i] = angleControl_[i]; 
      ODEBUG("m_qRef["<<i<<"]=" << qRef_[i]);
      
      // Compute speed and acceleration by finite differences.
      dqref_[i] = (qref_[i] - pqref_[i])/dt_;
      ddqref_[i] = (dqref_[i]- pdqref_[i])/dt_;
      
      // Store previous values.
      pqref_[i] = qref_[i];
      pdqref_[i] = dqref_[i];
    }
  if (angleControl_.size()<(unsigned int)robot_config_.nb_dofs)
    {
      for(long unsigned int i=angleControl_.size();
          i<(long unsigned int)robot_config_.nb_dofs
            ;i++)
        {
          qref_[i] = 0.0;
	  dqref_[i] = 0.0;
	  ddqref_[i] = 0.0;
          ODEBUG("m_qRef["<<i<<"]=" << m_qRef.data[i]);
        }
    }
}

void TALOSOH2SOT::set_path_to_library(std::string &aLibName)
{
  robot_config_.libname = aLibName;
}

void TALOSOH2SOT::localControl()
{
}

