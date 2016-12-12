#include <iostream>
#include <fstream>
#include <iomanip>
#include <dlfcn.h>

#include "talos-oh2sqp.hh"

#if DEBUG
#define ODEBUG(x) std::cout << x << std::endl
#else
#define ODEBUG(x)
#endif
#define ODEBUG3(x) std::cout << x << std::endl

#define DBGFILE "/tmp/talosoh2sqp.dat"

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



TALOSOH2SQP::TALOSOH2SQP()
{
  loadData();
  TalosOH();
  nb_it_ = 0;
}

TALOSOH2SQP::~TALOSOH2SQP()
{

}

void TALOSOH2SQP::loadData()
{
  std::string PosFileName = m_PathToData + ".pos";
  std::ifstream aof;
  aof.open(PosFileName);
  if (!aof.is_open())
    return;

  while(!aof.eof())
    {
      ConfigurationVector aCV;
      aCV.resize(40);
      double time;
      aof >> time;
      for(unsigned int i=0;i<40;i++)
	  aof >> aCV[i];
      m_RobotPath.push_back(aCV);
    }
  
}

void TALOSOH2SQP::initialize()
{
  int argc=1;
  char * argv[1];
  argv[0] = "talosoh2sot_server";
  TalosOH::initialize();
}



void TALOSOH2SQP::input()
{
  TalosOH::input();
  save_logs();
}



void TALOSOH2SQP::control()
{
  if (perform_control_ )
    {
      if (control_ticks_==0)
	{
	  for(unsigned int i=0;i<40;i++)
	    qref_[i] = m_RobotPath[nb_it_][i];
	  nb_it_++;
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




void TALOSOH2SQP::localControl()
{
}



void TALOSOH2SQP::set_path_to_data(std::string &aPathToData)
{
  m_PathToData = aPathToData;
}


