/*
 * Octree class to support quick lookups of nearest neighbor points and arbitrary data
 * associated with that point.
 *
 * @author Jeff Mahler
 * @contact jmahler@berkeley.edu
 */
#pragma once
#include <vector>

#include <opencv2/core/core.hpp>
#include <iostream>
const static unsigned int NUM_CHILDREN = 8;
const static unsigned int MIN_DEPTH_OFFSET = 2;

template <class T>
class Octree
{
 private:
  class Octnode {
    friend class Octree;

    Octnode(Octnode* p, unsigned char d, cv::Point3i o);
    ~Octnode();
    
    Octnode* parent;
    Octnode** children;
    unsigned char depth;
    cv::Point3i offset; // number of half width offsets of this node
    std::vector<std::pair<cv::Point3f, T> > data; // point-data pairs falling within this location
};

 public:
  Octree(unsigned char maxDepth, const std::vector<std::pair<cv::Point3f, T> >& data);
  ~Octree();

  std::pair<cv::Point3f, T> ClosestPoint(cv::Point3f query);

 private:
  void ComputeCenter(const std::vector<std::pair<cv::Point3f, T> >& data);
  void BuildTree(const std::vector<std::pair<cv::Point3f, T> >& data);
  void AddPoint(Octnode* node, const std::pair<cv::Point3f, T>& point);
  
  // query helpers
 private:
  std::pair<cv::Point3f, T> ClosestPoint(Octnode* node, unsigned char minDepth, cv::Point3f query);
  bool CompareClosest(Octnode* node, cv::Point3f query, std::pair<cv::Point3f, T>& curClosest, float& curDist);

 private:
  Octnode* root_;
  unsigned char minDepth_;
  unsigned char maxDepth_;
  cv::Point3f center_;
  std::vector<float> scales_;
};

template <class T>
Octree<T>::Octnode::Octnode(Octnode* p, unsigned char d, cv::Point3i o)
  : parent(p), children(NULL), depth(d), offset(o)
{
}

template <class T>
Octree<T>::Octnode::~Octnode()
{
  if (children != NULL) {
    for (unsigned char i = 0; i < NUM_CHILDREN; i++) {
      delete children[i];
    }
  }
}

template <class T>
Octree<T>::Octree(unsigned char maxDepth, const std::vector<std::pair<cv::Point3f, T> >& data)
  : minDepth_(std::max((int)(maxDepth-MIN_DEPTH_OFFSET), 0)), maxDepth_(maxDepth)
{
  root_ = new Octnode(NULL, 0, cv::Point3i(0,0,0));
  ComputeCenter(data);
  BuildTree(data);
}

template <class T>
Octree<T>::~Octree()
{
  delete root_;
}

template <class T>
void Octree<T>::ComputeCenter(const std::vector<std::pair<cv::Point3f, T> >& data)
{
  cv::Point3f min(FLT_MAX, FLT_MAX, FLT_MAX);
  cv::Point3f max(FLT_MIN, FLT_MIN, FLT_MIN);

  for (unsigned int i = 0; i < data.size(); i++) {
    // update min
    if (data[i].first.x < min.x) {
      min.x = data[i].first.x;
    }
    if (data[i].first.y < min.y) {
      min.y = data[i].first.y;
    }
    if (data[i].first.z < min.z) {
      min.z = data[i].first.z;
    }

    // update max
    if (data[i].first.x > max.x) {
      max.x = data[i].first.x;
    }
    if (data[i].first.y > max.y) {
      max.y = data[i].first.y;
    }
    if (data[i].first.z > max.z) {
      max.z = data[i].first.z;
    }
  }

  // compute the center of the tree
  center_.x = (max.x - min.x) / 2;
  center_.y = (max.y - min.y) / 2;
  center_.z = (max.z - min.z) / 2;

  float dim = std::max((max.x - min.x), std::max((max.y - min.y), (max.z - min.z)));
  for (unsigned int i = 0; i < maxDepth_; i++) {
    scales_.push_back(dim);
    dim = dim / 2.0f;
  } 
}

template <class T>
void Octree<T>::BuildTree(const std::vector<std::pair<cv::Point3f, T> >& data)
{
  for (unsigned int i = 0; i < data.size(); i++) {
    AddPoint(root_, data[i]);
  }
}

template <class T>
void Octree<T>::AddPoint(Octnode* node, const std::pair<cv::Point3f, T>& point)
{
  // terminate recursion if we have found the necessary point
  if (node->depth == maxDepth_ -1) {
    node->data.push_back(point);
  }

  // else determine which child node to search, create children if necessary
  else {
    if (node->children == NULL) {
      node->children = new Octnode*[NUM_CHILDREN];

      cv::Point3i offset = 2 * node->offset;
      int a = -1;
      int b = -1;
      int c = -1;
      
      for (unsigned int i = 0; i < maxDepth_; i++) {
	if (i > 0) {
	  a = -a;
	}
	if (b % 2 == 0) {
	  b = -b;
	}
	if (c % 4 == 0) {
	  c = -c;
	}

	node->children[i] = new Octnode(node, node->depth+1, cv::Point3i(offset.x+a, offset.y+b, offset.z+c));
      }
    }

    float halfWidth = scales_[node->depth] / 2.0f;
    cv::Point3f nodeCenter;
    nodeCenter.x = center_.x + halfWidth * node->offset.x;
    nodeCenter.y = center_.y + halfWidth * node->offset.y;
    nodeCenter.z = center_.z + halfWidth * node->offset.z;
    unsigned char nextIndex = 0;
    nextIndex |= ((point.first.x > nodeCenter.x) << 0);
    nextIndex |= ((point.first.y > nodeCenter.y) << 1);
    nextIndex |= ((point.first.z > nodeCenter.z) << 2);
    AddPoint(node->children[nextIndex], point);
  }
}

template <class T>
std::pair<cv::Point3f, T> Octree<T>::ClosestPoint(cv::Point3f query)
{
  return ClosestPoint(root_, minDepth_, query);
}

template <class T>
std::pair<cv::Point3f, T> Octree<T>::ClosestPoint(Octnode* node, unsigned char minDepth, cv::Point3f query)
{
  // go back up the tree if we reach a node without children
  if (node->children == NULL) {
    return ClosestPoint(node->parent, node->depth, query);
  }

  // search the leaves of the tree rooted at the current node once we have reached "min depth"
  else if (node->depth == minDepth - 1) {
    std::pair<cv::Point3f, T> closestPoint;
    float curDist = FLT_MAX;
    CompareClosest(node, query, closestPoint, curDist); 
    return closestPoint;
  }

  // search the next closest child node for the point of interest
  float halfWidth = scales_[node->depth] / 2.0f;
  cv::Point3f nodeCenter;
  nodeCenter.x = center_.x + halfWidth * node->offset.x;
  nodeCenter.y = center_.y + halfWidth * node->offset.y;
  nodeCenter.z = center_.z + halfWidth * node->offset.z;
  unsigned char nextIndex = 0;
  nextIndex |= ((query.x > nodeCenter.x) << 0);
  nextIndex |= ((query.y > nodeCenter.y) << 1);
  nextIndex |= ((query.z > nodeCenter.z) << 2);
  return ClosestPoint(node->children[nextIndex], minDepth, query);
}

template <class T>
bool Octree<T>::CompareClosest(Octnode* node, cv::Point3f query, std::pair<cv::Point3f, T>& curClosest, float& curDist)
{
  // look for a new closest point if we reach a leaf node
  if (node->depth == maxDepth_ - 1) {
    for (unsigned int i = 0; i < node->data.size(); i++) {
      // compute distance between node's data point and the query point
      float dist = 0;
      dist += std::pow((node->data[i].first.x - query.x), 2);
      dist += std::pow((node->data[i].first.y - query.y), 2);
      dist += std::pow((node->data[i].first.z - query.z), 2);
      dist = sqrt(dist);

      // reassign closest if necessary
      if (dist < curDist) {
	curClosest.first = node->data[i].first;
	curClosest.second = node->data[i].second;
	curDist = dist;
      }
    }
    return true;
  }

  // query each of the children (if there are any) if we are not a leaf node
  else if (node->children != NULL) {
    for (unsigned int i = 0; i < NUM_CHILDREN; i++) {
      CompareClosest(node->children[i], query, curClosest, curDist);
    }
    return true;
  }
  return false;
}
