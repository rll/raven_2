#include <boost/filesystem.hpp>
#include <exception>
#include <iostream>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <vector>

#include "saveload.h"
#include "DOF_type.h"

#include <raven/state/device.h>

namespace fs = boost::filesystem;
using namespace std;

static std::vector<int> origOffsets;

fs::path getRosDir() {
  char const* home = getenv("HOME");  
  fs::path rosDir = home;
  rosDir /= ".ros"; // rosDir = /home/user/.ros
  return rosDir;  
}

bool loadOffsets(robot_device& dev) {  
  fs::path rosDir = getRosDir();
  fs::path offsetPath = rosDir / "offsets.txt";
  ifstream infile(offsetPath.string().c_str());
  if (infile.fail()) return false;
  
  for (int iMech = 0; iMech < 2; iMech++) {
    for (int iJoint = 0; iJoint < 8; iJoint++) {
      float x;
      infile >> x;
      dev.mech[iMech].joint[iJoint].enc_offset = x;
      origOffsets.push_back(x);
#ifdef USE_NEW_DEVICE
      int joint_ind = iJoint;
	  if (joint_ind != 3) {
		if (joint_ind > 3) joint_ind--;
		Device::beginCurrentUpdate(ros::Time(0));
		ArmPtr arm = Device::currentNoClone()->getArmById(dev.mech[iMech].type);
		MotorPtr motor = arm->motor(joint_ind);
		motor->setEncoderOffset(x);
		Device::finishCurrentUpdate();
	  }
#endif
    }
  }
  assert(!infile.fail());  
  return true;
  
}


bool saveOffsets(robot_device& dev) {
  fs::path rosDir = getRosDir();
  if (!fs::exists(rosDir)) {
    bool success = create_directory(rosDir);
    if (!success) return false;
  }
  
  fs::path offsetPath = rosDir / "offsets.txt";
  ofstream outfile(offsetPath.string().c_str());
  if (outfile.fail()) return false;
  
  int ind=0;
  //std::cout << "Offset diffs:";
  for (int iMech = 0; iMech < 2; iMech++) {
    for (int iJoint = 0; iJoint < 8; iJoint++) {
      outfile << dev.mech[iMech].joint[iJoint].enc_offset<< " ";
      //std::cout << " " << origOffsets[ind] - dev.mech[iMech].joint[iJoint].enc_offset;
    }
  }
  //std::cout << std::endl;
  return true;
  
}

bool saveDOFInfo() {
  fs::path rosDir = getRosDir() / "dof_info.txt";
  fs::path offsetPath = rosDir / "offsets.txt";
  
  
  FILE* outFile = fopen (offsetPath.string().c_str(),"w");
  if (outFile == NULL) return false;
  
  extern DOF_type DOF_types[];  
  
  for (int iDOF = 0; iDOF < 16; iDOF++) {
    DOF_type& dof = DOF_types[iDOF];
    fprintf(outFile, "%i %.5f %.5f %.5f %.5f %.5f", iDOF, dof.home_position, dof.max_position, dof.KP, dof.KD, dof.KI);
  }
  return true;  
}
