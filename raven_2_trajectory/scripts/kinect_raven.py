#!/usr/bin/env python

import roslib
roslib.load_manifest('raven_2_trajectory')

"""
import argparse
parser = argparse.ArgumentParser(description="e.g. python kinect_raven.py /camera/depth/points cd")
parser.add_argument("cloud_topic")
parser.add_argument("geom_type", choices=["mesh","cd","spheres", "boxes"])
args = parser.parse_args()
"""
import numpy as np
import os
import openravepy as rave
import cloudprocpy,trajoptpy
import json
import trajoptpy.make_kinbodies as mk
from trajoptpy.check_traj import traj_is_safe

assert rave.__version__ >= "0.9"

import rospy
import tfx

# START section of point_cloud2.py
# https://code.google.com/p/ros-by-example/source/browse/trunk/rbx_vol_1/rbx1_apps/src/point_cloud2.py
import roslib.message

import ctypes
import math
import struct

from raven_2_msgs.msg import Constants
from raven_2_utils import raven_util
from raven_2_utils import raven_constants

from sensor_msgs.msg import PointCloud2, PointField

_DATATYPES = {}
_DATATYPES[PointField.INT8]    = ('b', 1)
_DATATYPES[PointField.UINT8]   = ('B', 1)
_DATATYPES[PointField.INT16]   = ('h', 2)
_DATATYPES[PointField.UINT16]  = ('H', 2)
_DATATYPES[PointField.INT32]   = ('i', 4)
_DATATYPES[PointField.UINT32]  = ('I', 4)
_DATATYPES[PointField.FLOAT32] = ('f', 4)
_DATATYPES[PointField.FLOAT64] = ('d', 8)

class EnvironmentGrabber(object):

    def __init__(self):
        self.env = rave.Environment()
        self.env.SetViewer('qtcoin')

        rospy.loginfo('Before loading model')
        ravenFile = os.path.join(roslib.packages.get_pkg_subdir('RavenDebridement','models'),'myRaven.xml')
        
        #ravenFile = '/home/gkahn/ros_workspace/RavenDebridement/models/myRaven.xml'
        self.env.Load(ravenFile)
        rospy.loginfo('After loading model')
        
        self.robot = self.env.GetRobots()[0]
        
        # setup variables for kinect constraint generation
        self.num_cd_components = 50
        self.geom_type = 'cd'
        self.kinect_topic = '/camera/depth/points'
        self.kinect_depth_frame = '/camera_depth_optical_frame'
        self.robot_frame = '/world'
        self.T_kinect_to_robot = tfx.lookupTransform(self.robot_frame, self.kinect_depth_frame, wait=20)
        
        self.cd_body_names = []
        self.cloud_id = 0
        
        rospy.Subscriber(self.kinect_topic, PointCloud2, self.kinect_callback)

    # KINECT-based constraint generation functions
    def read_points(self, cloud, field_names=None, skip_nans=False, uvs=[]):
        """
        Read points from a L{sensor_msgs.PointCloud2} message.
    
        @param cloud: The point cloud to read from.
        @type  cloud: L{sensor_msgs.PointCloud2}
        @param field_names: The names of fields to read. If None, read all fields. [default: None]
        @type  field_names: iterable
        @param skip_nans: If True, then don't return any point with a NaN value.
        @type  skip_nans: bool [default: False]
        @param uvs: If specified, then only return the points at the given coordinates. [default: empty list]
        @type  uvs: iterable
        @return: Generator which yields a list of values for each point.
        @rtype:  generator
        """
        assert isinstance(cloud, roslib.message.Message) and cloud._type == 'sensor_msgs/PointCloud2', 'cloud is not a sensor_msgs.msg.PointCloud2'
        fmt = self._get_struct_fmt(cloud.is_bigendian, cloud.fields, field_names)
        width, height, point_step, row_step, data, isnan = cloud.width, cloud.height, cloud.point_step, cloud.row_step, cloud.data, math.isnan
        unpack_from = struct.Struct(fmt).unpack_from
    
        if skip_nans:
            if uvs:
                for u, v in uvs:
                    p = unpack_from(data, (row_step * v) + (point_step * u))
                    has_nan = False
                    for pv in p:
                        if isnan(pv):
                            has_nan = True
                            break
                    if not has_nan:
                        yield p
            else:
                for v in xrange(height):
                    offset = row_step * v
                    for u in xrange(width):
                        p = unpack_from(data, offset)
                        has_nan = False
                        for pv in p:
                            if isnan(pv):
                                has_nan = True
                                break
                        if not has_nan:
                            yield p
                        offset += point_step
        else:
            if uvs:
                for u, v in uvs:
                    yield unpack_from(data, (row_step * v) + (point_step * u))
            else:
                for v in xrange(height):
                    offset = row_step * v
                    for u in xrange(width):
                        yield unpack_from(data, offset)
                        offset += point_step
    
    def _get_struct_fmt(self, is_bigendian, fields, field_names=None):
        fmt = '>' if is_bigendian else '<'
    
        offset = 0
        for field in (f for f in sorted(fields, key=lambda f: f.offset) if field_names is None or f.name in field_names):
            if offset < field.offset:
                fmt += 'x' * (field.offset - offset)
                offset = field.offset
            if field.datatype not in _DATATYPES:
                print >> sys.stderr, 'Skipping unknown PointField datatype [%d]' % field.datatype
            else:
                datatype_fmt, datatype_length = _DATATYPES[field.datatype]
                fmt    += field.count * datatype_fmt
                offset += field.count * datatype_length
    
        return fmt
    # END section of point_cloud2.py
    
    def generate_mesh(self, cloud):
        #cloud = cloudprocpy.fastBilateralFilter(cloud, 15, .05) # smooth out the depth image
        big_mesh = cloudprocpy.meshOFM(cloud, 3, .1) # use pcl OrganizedFastMesh to make mesh
        simple_mesh = cloudprocpy.quadricSimplifyVTK(big_mesh, .02) # decimate mesh with VTK function
        return simple_mesh
    # END generate_mesh
    
    def get_xyz_world_frame(self, cloud):
        xyz1 = cloud.to2dArray()
        xyz1[:,3] = 1
        return xyz1.dot(self.T_kinect_to_robot.array.T)[:,:3]
    
    # Plots the point cloud to the env
    # Returns a handle to it
    def plotCloud(self, cloud):
        xyz = self.get_xyz_world_frame(cloud)
        goodinds = np.isfinite(xyz[:,0])
        if isinstance(cloud, cloudprocpy.CloudXYZRGB):
            rgbfloats = cloud.to2dArray()[:,4]
            rgb0 = np.ndarray(buffer=rgbfloats.copy(),shape=(raven_constants.Kinect.height * raven_constants.Kinect.width, 4),dtype='uint8')
            cloud_handle = self.env.plot3(xyz[goodinds,:], 2,(rgb0[goodinds,:3][:,::-1])/255. )
        else:
            cloud_handle = self.env.plot3(xyz[goodinds,:], 2)
        return cloud_handle
    
    # Adds the geometric shape(s) to the env
    # Returns a list of the names of the added KinBodies
    def generateBodies(self, cloud, id):
        # BEGIN addtoenv
        names = []
        if self.geom_type == "mesh":
            mesh = self.generate_mesh(cloud)
            name = "simple_mesh_%i"%id
            mesh_body = mk.create_trimesh(self.env, self.get_xyz_world_frame(mesh.getCloud()), np.array(mesh.getFaces()), name=name)
            mesh_body.SetUserData("bt_use_trimesh", True) # Tell collision checker to use the trimesh rather than the convex hull of it
            names.append(name)
            
        elif self.geom_type == "cd":
            big_mesh = self.generate_mesh(cloud)
            convex_meshes = cloudprocpy.convexDecompHACD(big_mesh, self.num_cd_components)
            for (i,mesh) in enumerate(convex_meshes):
                name = "mesh_%i_%i" % (id,i)
                verts = self.get_xyz_world_frame(mesh.getCloud())
                mk.create_trimesh(self.env, verts, mesh.getTriangles(), name=name)
                randcolor = np.random.rand(3)
                self.env.GetKinBody(name).GetLinks()[0].GetGeometries()[0].SetAmbientColor(randcolor)
                self.env.GetKinBody(name).GetLinks()[0].GetGeometries()[0].SetDiffuseColor(randcolor)
                names.append(name)
        # END  addtoenv   
             
        elif self.geom_type == "spheres":
            raise Exception("don't use spheres--there's a performance issue")
            cloud_ds = cloudprocpy.downsampleCloud(cloud, .04)
            name = "spheres_%i"%id
            mk.create_spheres(self.env, self.get_xyz_world_frame(cloud_ds), .02, name=name)
            names.append(name)
            
        elif self.geom_type == "boxes":
            cloud_ds = cloudprocpy.downsampleCloud(cloud, .04)
            name = "boxes_%i"%id
            mk.create_boxes(self.env, self.get_xyz_world_frame(cloud_ds), .02, name=name)
            names.append(name)
        return names
    
    # Updates the bodies and point cloud of the openrave environment
    def kinect_callback(self, cloud_msg):
        # convert from point cloud message to cloudprocpy.CloudXYZ
        points_gen = self.read_points(cloud_msg, skip_nans=False)
        points = []
        for pt in points_gen:
            pt = list(pt)
            pt.append(1)
            points.append(pt)
            
        # reorganize cloud data to construct a cloud object 
        n_dim = 4 # 3 spatial dimension + 1 for homogenity   
        cloud = cloudprocpy.CloudXYZ()
        xyz1 = np.array(points)
        xyz1 = np.reshape(xyz1, (raven_constants.Kinect.height, raven_constants.Kinect.width, n_dim))
        cloud.from3dArray(xyz1)
            
        #save cloud for next run
        np.save('environment_obstacles_%d'%(self.cloud_id), xyz1)
            
        # add new bodies and cloud first
        new_cd_body_names = self.generateBodies(cloud, self.cloud_id)
            
        # remove previous bodies and cloud
        for name in self.cd_body_names:
            self.env.Remove(self.env.GetKinBody(name))
            
        self.cd_body_names = new_cd_body_names
        self.cloud_id = self.cloud_id+1

    
def listener():
    rospy.init_node('kinect_environment_grabber', anonymous=True)
    eg = EnvironmentGrabber()
    rospy.spin()
 
if __name__ == '__main__':
    listener()
