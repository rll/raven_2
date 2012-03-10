#include <boost/filesystem.hpp>
#include <exception>
#include <iostream>
#include <cstdio>
#include <fstream>

#include "saveload.h"
#include "DOF_type.h"

namespace fs = boost::filesystem;
using namespace std;

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
  
  for (int iMech = 0; iMech < 2; iMech++) {
    for (int iJoint = 0; iJoint < 8; iJoint++) {
      outfile << dev.mech[iMech].joint[iJoint].enc_offset<< " ";
    }
  }
  return true;
  
}

bool saveDOFInfo() {
  fs::path outPath = getRosDir() / "dof_info.txt";
  fs::path offsetPath = rosDir / "offsets.txt";
  
  
  FILE* outFile = fopen (offsetPath.string().c_str(),"w");
  if (outFile == NULL) return false;
  
  extern DOF_type DOF_types[];  
  
  for (int iDOF = 0; iDOF < 16) {
    DOF_type dof& = DOF_types[iDOF];
    fprintf(outFile, "%i %.5f %.5f %.5f %.5f %.5f", iDOF, dof.home_position, dof.max_position, dof.KP, dof.KD, dof.KI);
  }
  
}
