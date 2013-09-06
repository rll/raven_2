/*
 * Table for easy loading and lookup of nearest calibration data 
 * Expects the data to be in XML format and stored in the convention <name>_<number>.xml, starting from number 0
 *
 * @author Jeff Mahler
 * @contact jmahler@berkeley.edu
 */
#pragma once
#include <string>
#include <vector>
#include <opencv2/core/core.hpp>
#include <iostream>

#include "raven_2_vision/octree.hpp"

const std::string ERROR_CALIB_EXT = ".xml";

// ! Encapculates the average position measurement error for use in an octree
class PositionMeasurementNoise
{
 public:
  PositionMeasurementNoise() {}
  PositionMeasurementNoise(cv::Mat a, cv::Mat c)
  {
    avgError = a.clone();
    covariance = c.clone();
  }
  PositionMeasurementNoise(const PositionMeasurementNoise& other) 
  {
    avgError = other.avgError.clone();
    covariance = other.covariance.clone();
  }

 public:
  cv::Mat avgError;
  cv::Mat covariance;
};

class PositionErrorTable
{
 public:
  PositionErrorTable(const std::string& calibrationFileDirectory, const std::string& calibrationFilename);
  ~PositionErrorTable();

 public:
  PositionMeasurementNoise LookupNoise(cv::Point3f point);

 private:
  void LoadData();

 private:
  std::string fileDirectory_;
  std::string filename_;
  Octree<PositionMeasurementNoise>* octree_;
};

