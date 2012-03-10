#include <boost/filesystem.hpp>
#include <exception>
#include <iostream>
namespace fs = boost::filesystem;
using namespace std;

fs::path getRosDir() {
  char const* home = getenv("HOME");  
  fs::path rosDir = home;
  rosDir /= ".ros"; // rosDir = /home/user/.ros
  return rosDir;  
}

bool loadOffsets(device& dev) {  
  fs::path rosDir = getRosDir();
  fs::path offsetPath = rosDir / "offsets.txt";
  ifstream infile(offsetPath.string().c_str());
  if (infile.fail()) return false;
  
  for (int iMech = 0; iMech < 2; iMech++) {
    for (int iJoint = 0; iJoint < 8; iJoint++) {
      float x;
      infile >> x;
      device.mech[iMech].joint[iJoint].enc_offset = x;
    }
  }
  assert(!infile.fail());  
  return true;
  
}


bool saveOffsets(device& dev) {
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
      outfile << device.mech[iMech].joint[iJoint].enc_offset<< " ";
    }
  }
  return true;
  
  
  
}