#include <iostream>
#include <fstream>
#include <iomanip>
#include <dlfcn.h>

#include "talos-oh.hh"

#if DEBUG
#define ODEBUG(x) std::cout << x << std::endl
#else
#define ODEBUG(x)
#endif
#define ODEBUG3(x) std::cout << x << std::endl

#define DBGFILE "/tmp/talos2oh.dat"

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



TalosOH::TalosOH()
{

  robot_config_.nb_dofs =32;
  nb_dofs_ = 32;
  P_.resize(nb_dofs_);
  I_.resize(nb_dofs_);
  D_.resize(nb_dofs_);

  q_.resize(nb_dofs_);
  qref_.resize(nb_dofs_);
  pqref_.resize(nb_dofs_);
  dq_.resize(nb_dofs_);
  dqref_.resize(nb_dofs_);
  pdqref_.resize(nb_dofs_);
  ddqref_.resize(nb_dofs_);
  torques_.resize(nb_dofs_);

  bush_qref_.resize(6);
  bush_dqref_.resize(6);
  bush_ddqref_.resize(6);

  sendto_.length(1);
  recvfrom_.length(1);

  perform_control_ = false;
  
  control_ticks_ = 0;
  control_ticks_max_ = 5;
  dt_ = 0.005;
  RESETDEBUG5();

  cumul_err_ = 0.0;

  std::ofstream LogFile;
  LogFile.open("/tmp/angle.dat",std::ofstream::out);
  LogFile << "%time ";
  for(unsigned int i=0;i<nb_dofs_;i++)
    LogFile << " Q"<<i << " ";

  for(unsigned int i=0;i<4;i++)
    for(unsigned int j=0;j<6;j++)
      LogFile << " F"<< i << "_" << j;

  LogFile << std::endl;
  LogFile.close();
}

TalosOH::~TalosOH()
{

}

void TalosOH::initref()
{
  // Torso
  qref_[0] =  0.0; dqref_[0] = 0.0;
  qref_[1] =  0.0; dqref_[1] = 0.0;

  // Head
  qref_[2] =  0.0; dqref_[2] = 0.0;
  qref_[3] =  0.0; dqref_[3] = 0.0;
  
  // Left arm
  qref_[4] =  0.0    ; dqref_[4] = 0.0; 
  qref_[5] =  0.25   ; dqref_[5] = 0.0;
  qref_[6] =  0.0    ; dqref_[6] = 0.0;
  qref_[7] =  0.0    ; dqref_[7] = 0.0;
  qref_[8] =  0.0    ; dqref_[8] = 0.0;
  qref_[9] =  0.0    ; dqref_[9] = 0.0;
  qref_[10] = 0.0    ; dqref_[10] = 0.0;

  // Right Arm
  qref_[11] = 0.0    ; dqref_[11] = 0.0;
  qref_[12] =-0.25   ; dqref_[12] = 0.0;
  qref_[13] = 0.0    ; dqref_[13] = 0.0;
  qref_[14] = 0.0    ; dqref_[14] = 0.0;
  qref_[15] = 0.0    ; dqref_[15] = 0.0;
  qref_[16] = 0.0    ; dqref_[16] = 0.0;
  qref_[17] = 0.0    ; dqref_[17] = 0.0;

  // Left leg
  qref_[18] = 0.0    ; dqref_[18] = 0.0;
  qref_[19] = 0.0    ; dqref_[19] = 0.0;
  qref_[20] = 0.0    ; dqref_[20] = 0.0;
  qref_[21] = 0.0    ; dqref_[21] = 0.0;
  qref_[22] = 0.0    ; dqref_[22] = 0.0;
  qref_[23] = 0.0    ; dqref_[23] = 0.0;

  // Right Leg
  qref_[24] = 0.0    ; dqref_[24] = 0.0;
  qref_[25] = 0.0    ; dqref_[25] = 0.0;
  qref_[26] = 0.0    ; dqref_[26] = 0.0;
  qref_[27] = 0.0    ; dqref_[27] = 0.0;
  qref_[28] = 0.0    ; dqref_[28] = 0.0;
  qref_[29] = 0.0    ; dqref_[29] = 0.0;

  for(unsigned int i=0;i<nb_dofs_;i++)
    { 
      pqref_[i] = qref_[i];
      dqref_[i] = 0.0; 
      pdqref_[i] = dqref_[i];
      ddqref_[i] = 0.0; 
    }

  for(unsigned int i=0;i<6;i++)
    {
      bush_qref_[i] =
	bush_dqref_[i] =
	bush_ddqref_[i] = 0.0;
    }
  
}

void TalosOH::initpids()
{
  
  std::ifstream ifs;
  ifs.open(robot_config_.pid_gains_filename.c_str());

  if (!ifs.is_open())
    return;
  
  //  Read PIDs.
  for(int i=0;i<nb_dofs_;i++)
    {
      ifs >> D_[i];
      ifs >> I_[i];
      ifs >> P_[i];
      std::cout << i << " - " << D_[i] << " " << I_[i] << " " << P_[i] << std::endl;
    }
  ifs.close();
}

void TalosOH::initLinkNames()
{
  std::string aLinkName;
  
  aLinkName = "torso_1_joint";listOfLinks_.push_back(aLinkName);
  aLinkName = "torso_2_joint";listOfLinks_.push_back(aLinkName);

  aLinkName = "head_1_joint";listOfLinks_.push_back(aLinkName);
  aLinkName = "head_2_joint";listOfLinks_.push_back(aLinkName);

  aLinkName = "arm_left_1_joint";listOfLinks_.push_back(aLinkName);
  aLinkName = "arm_left_2_joint";listOfLinks_.push_back(aLinkName);
  aLinkName = "arm_left_3_joint";listOfLinks_.push_back(aLinkName);
  aLinkName = "arm_left_4_joint";listOfLinks_.push_back(aLinkName);
  aLinkName = "arm_left_5_joint";listOfLinks_.push_back(aLinkName);
  aLinkName = "arm_left_6_joint";listOfLinks_.push_back(aLinkName);
  aLinkName = "arm_left_7_joint";listOfLinks_.push_back(aLinkName);

  aLinkName = "arm_right_1_joint";listOfLinks_.push_back(aLinkName);
  aLinkName = "arm_right_2_joint";listOfLinks_.push_back(aLinkName);
  aLinkName = "arm_right_3_joint";listOfLinks_.push_back(aLinkName);
  aLinkName = "arm_right_4_joint";listOfLinks_.push_back(aLinkName);
  aLinkName = "arm_right_5_joint";listOfLinks_.push_back(aLinkName);
  aLinkName = "arm_right_6_joint";listOfLinks_.push_back(aLinkName);
  aLinkName = "arm_right_7_joint";listOfLinks_.push_back(aLinkName);
  
  aLinkName = "leg_left_1_joint";listOfLinks_.push_back(aLinkName);
  aLinkName = "leg_left_2_joint";listOfLinks_.push_back(aLinkName);
  aLinkName = "leg_left_3_joint";listOfLinks_.push_back(aLinkName);
  aLinkName = "leg_left_4_joint";listOfLinks_.push_back(aLinkName);
  aLinkName = "leg_left_5_joint";listOfLinks_.push_back(aLinkName);
  aLinkName = "leg_left_6_joint";listOfLinks_.push_back(aLinkName);

  aLinkName = "leg_right_1_joint";listOfLinks_.push_back(aLinkName);
  aLinkName = "leg_right_2_joint";listOfLinks_.push_back(aLinkName);
  aLinkName = "leg_right_3_joint";listOfLinks_.push_back(aLinkName);
  aLinkName = "leg_right_4_joint";listOfLinks_.push_back(aLinkName);
  aLinkName = "leg_right_5_joint";listOfLinks_.push_back(aLinkName);
  aLinkName = "leg_right_6_joint";listOfLinks_.push_back(aLinkName);

}



void TalosOH::initialize()
{

  initref();
  initpids();
  initLinkNames();
}

void TalosOH::input_bushs()
{
  for(unsigned int i=0;i<listOfBushs_.size();i++)
    {
      std::string aLinkName = listOfBushs_[i];
      OpenHRP::DynamicsSimulator::LinkDataType linkDataType =
	OpenHRP::DynamicsSimulator::EXTERNAL_FORCE;
      OpenHRP::DblSequence_var var_q;
      dynamicsSimulator_->getCharacterLinkData(modelName_.c_str(),
					       aLinkName.c_str(),
					       linkDataType,var_q);
      bush_ddqref_[i] = var_q[0];
      std::cout << "input_bushs " << aLinkName << " " << bush_ddqref_[i] << std::endl;
      double T=dt_/control_ticks_max_;

      bush_qref_[i] = bush_qref_[i] + T*bush_dqref_[i] + bush_ddqref_[i]*T*T/2;
      bush_dqref_[i] = bush_dqref_[i] + bush_ddqref_[i]*T;
      
    }
}

void TalosOH::output_bushs()
{
  for(unsigned int i=0;i<listOfBushs_.size();i++)
    {
      std::string aLinkName = listOfBushs_[i];
      sendto_[0] = bush_qref_[i];
      OpenHRP::DynamicsSimulator::LinkDataType linkDataType = OpenHRP::DynamicsSimulator::JOINT_VALUE;
      dynamicsSimulator_->setCharacterLinkData(modelName_.c_str(),
					       aLinkName.c_str(),
					       linkDataType,sendto_);
      sendto_[0] = bush_dqref_[i];
      linkDataType = OpenHRP::DynamicsSimulator::JOINT_VELOCITY;
      dynamicsSimulator_->setCharacterLinkData(modelName_.c_str(),
					       aLinkName.c_str(),
					       linkDataType,sendto_);
      
      sendto_[0] = bush_ddqref_[i];
      linkDataType = OpenHRP::DynamicsSimulator::JOINT_ACCELERATION;
      dynamicsSimulator_->setCharacterLinkData(modelName_.c_str(),
					       aLinkName.c_str(),
					       linkDataType,sendto_);
      
    }					      
}

void TalosOH::save_logs()
{
  std::ofstream LogFile;

  LogFile.open("/tmp/angle.dat",std::ofstream::app);

  for(unsigned int i=0;i<listOfLinks_.size();i++)
    {
      LogFile << q_[i] << " ";
    }

  for(unsigned int i=0;i<listOfBushs_.size();i++)
    {
      LogFile << bush_qref_[i] << " ";
    }

  for(unsigned int i=0;i<4;i++)
    {
      for(unsigned int j=0;j<6;j++)
	{
	  LogFile << forces_[i*6+j] << " ";
	}
    }
     
  LogFile << std::endl;
  LogFile.close();

}
void TalosOH::input()
{
  

  for(unsigned int i=0;i<listOfLinks_.size();i++)
    {
      std::string aLinkName = listOfLinks_[i];
      OpenHRP::DynamicsSimulator::LinkDataType linkDataType =
	OpenHRP::DynamicsSimulator::JOINT_VALUE;
      OpenHRP::DblSequence_var var_q;
      dynamicsSimulator_->getCharacterLinkData(modelName_.c_str(),
					       aLinkName.c_str(),
					       linkDataType,var_q);
      q_[i] = var_q[0];

      linkDataType = OpenHRP::DynamicsSimulator::JOINT_VELOCITY;
      OpenHRP::DblSequence_var var_dq;
      dynamicsSimulator_->getCharacterLinkData(modelName_.c_str(),
					       aLinkName.c_str(),
					       linkDataType,var_dq);
      dq_[i] = var_dq[0];

    }
}

void TalosOH::output()
{
  if (1)
    {
      for(unsigned int i=0;i<listOfLinks_.size();i++)
	{
	  std::string aLinkName = listOfLinks_[i];
	  sendto_[0] = torques_[i];
	  OpenHRP::DynamicsSimulator::LinkDataType linkDataType = OpenHRP::DynamicsSimulator::JOINT_TORQUE;
	  dynamicsSimulator_->setCharacterLinkData(modelName_.c_str(),
						   aLinkName.c_str(),
						   linkDataType,sendto_);
	}					      
    }
  else
    {
      for(unsigned int i=0;i<listOfLinks_.size();i++)
	{
	  std::string aLinkName = listOfLinks_[i];
	  sendto_[0] = qref_[i];
	  OpenHRP::DynamicsSimulator::LinkDataType linkDataType = OpenHRP::DynamicsSimulator::JOINT_VALUE;
	  dynamicsSimulator_->setCharacterLinkData(modelName_.c_str(),
						   aLinkName.c_str(),
						   linkDataType,sendto_);
	  sendto_[0] = dqref_[i];
	  linkDataType = OpenHRP::DynamicsSimulator::JOINT_VELOCITY;
	  dynamicsSimulator_->setCharacterLinkData(modelName_.c_str(),
						   aLinkName.c_str(),
						   linkDataType,sendto_);

	  sendto_[0] = ddqref_[i];
	  linkDataType = OpenHRP::DynamicsSimulator::JOINT_ACCELERATION;
	  dynamicsSimulator_->setCharacterLinkData(modelName_.c_str(),
						   aLinkName.c_str(),
						   linkDataType,sendto_);

	}					      

      //output_bushs();
    }
}


void TalosOH::control()
{
  /* std::cout << "perform_control_ :" << perform_control_ 
	    << "dynamic_graph_stopped_ :" << dynamic_graph_stopped_ 
	    << std::endl;
  */
  if (perform_control_ )
    {
      if (control_ticks_==0)
	{
	  try
	    {
	      localControl();
	    } 
	  catch (std::exception &e) 
	    {  ODEBUG5("Exception on Execute: " << e.what());throw e; }
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

void TalosOH::stop()
{
  perform_control_ = false;
}

void TalosOH::start()
{
  perform_control_ = true;
}

void TalosOH::shutdown()
{
}

void TalosOH::restart()
{
}

void TalosOH::set_path_to_pid_gains(std::string &aFileName)
{
  robot_config_.pid_gains_filename = aFileName;
}

void TalosOH::localControl()
{
}
