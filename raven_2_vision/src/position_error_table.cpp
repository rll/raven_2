/*
 * Table for easy loading and lookup of nearest calibration data 
 * Expects the data to be in XML format and stored in the convention <name>_<number>.xml, starting from number 0
 *
 * @author Jeff Mahler
 * @contact jmahler@berkeley.edu
 */
#include "raven_2_vision/position_error_table.hpp"

#include <iostream>
#include <sstream>

PositionErrorTable::PositionErrorTable(const std::string& calibrationFileDirectory, const std::string& calibrationFilename)
  : fileDirectory_(calibrationFileDirectory), filename_(calibrationFilename)
{
  LoadData();
}

PositionErrorTable::~PositionErrorTable()
{
  delete octree_;
}

void PositionErrorTable::LoadData()
{
  std::vector<std::pair<cv::Point3f, PositionMeasurementNoise> > data;
  int fileNum = 0;
  int prevDataSize = 0;
  bool openSucceeded = true;

  while (openSucceeded) {
    std::stringstream nextFilename;
    nextFilename << filename_ << "_" << fileNum;
    std::string nextFile(fileDirectory_);
    nextFile.append("/");
    nextFile.append(nextFilename.str());
    nextFile.append(ERROR_CALIB_EXT);

    float depth = 0.0f;
    cv::FileStorage fs(nextFile, cv::FileStorage::READ);
    openSucceeded = fs.isOpened();
    if (openSucceeded) {
      fs["depth"] >> depth;

      // load in the xy positions for this file
      cv::FileNode xyPositions = fs["xyPositions"];
      cv::FileNodeIterator it = xyPositions.begin();
      cv::Mat xypos;
      for (; it != xyPositions.end(); it++) {
	(*it) >> xypos;
	cv::Point3f fullPos(xypos.at<float>(0,0), xypos.at<float>(1,0), depth);
	data.push_back(std::make_pair(fullPos, PositionMeasurementNoise()));
      }

      // load in the avg errors for this file
      unsigned int index = prevDataSize;
      cv::FileNode avgErrors = fs["avgErrors"];
      it = avgErrors.begin();
      for (; it != avgErrors.end(); it++, index++) {
	(*it) >> data[index].second.avgError;
      }

      // load in the covariances for this file
      index = prevDataSize;
      cv::FileNode covariance = fs["covariance"];
      it = covariance.begin();
      for (; it != covariance.end(); it++, index++) {
	(*it) >> data[index].second.covariance;
      }
    }
    prevDataSize = data.size();
    fileNum++;
  }

  // create an octree with the new data
  unsigned char depth = 9;
  octree_ = new Octree<PositionMeasurementNoise>(depth, data);
}

PositionMeasurementNoise PositionErrorTable::LookupNoise(cv::Point3f point)
{
  std::pair<cv::Point3f, PositionMeasurementNoise> data = octree_->ClosestPoint(point);
  std::cout << data.first << std::endl;
  return data.second;
}
