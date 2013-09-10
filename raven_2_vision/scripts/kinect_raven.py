#!/usr/bin/env python

import argparse
parser = argparse.ArgumentParser(description="e.g. python kinect_raven.py /camera/depth/points cd")
parser.add_argument("cloud_topic")
parser.add_argument("geom_type", choices=["mesh","cd","spheres", "boxes"])
args = parser.parse_args()

import numpy as np, os.path as osp
import cloudprocpy,trajoptpy,openravepy
import json
import trajoptpy.make_kinbodies as mk
from trajoptpy.check_traj import traj_is_safe

import IPython

assert openravepy.__version__ >= "0.9"

import rospy

# START section of point_cloud2.py
# https://code.google.com/p/ros-by-example/source/browse/trunk/rbx_vol_1/rbx1_apps/src/point_cloud2.py
import roslib; roslib.load_manifest('sensor_msgs'); roslib.load_manifest('tfx')
import roslib.message
import tfx

import ctypes
import math
import struct

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

def read_points(cloud, field_names=None, skip_nans=False, uvs=[]):
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
    fmt = _get_struct_fmt(cloud.is_bigendian, cloud.fields, field_names)
    width, height, point_step, row_step, data, isnan = cloud.width, cloud.height, cloud.point_step, cloud.row_step, cloud.data, math.isnan
    unpack_from = struct.Struct(fmt).unpack_from
    point_step = 16
    row_step = 10240

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

def _get_struct_fmt(is_bigendian, fields, field_names=None):
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

# BEGIN generate_mesh
def generate_mesh(cloud):
    #cloud = cloudprocpy.fastBilateralFilter(cloud, 15, .05) # smooth out the depth image
    big_mesh = cloudprocpy.meshOFM(cloud, 3, .1) # use pcl OrganizedFastMesh to make mesh
    simple_mesh = cloudprocpy.quadricSimplifyVTK(big_mesh, .02) # decimate mesh with VTK function
    return simple_mesh
# END generate_mesh

def get_xyz_world_frame(cloud):
    xyz1 = cloud.to2dArray()
    xyz1[:,3] = 1
    return xyz1.dot(T_w_k.T)[:,:3]

rospy.init_node('listener', anonymous=True)
#T_w_k = np.eye(4)
T_w_k = np.array(tfx.lookupTransform('/world', '/camera_depth_optical_frame').matrix)


def isValidMesh(mesh):
    vertices = mesh.getVertices()

    for vertex in vertices:
	if vertex[2] > .58:
	    return False

    return True


env = openravepy.Environment()
env.Load('/home/annal/src/RavenDebridement/models/myRaven.xml')
env.SetViewer('qtcoin') # attach viewer (optional)
env.StopSimulation()

# Plots the point cloud to the env
# Returns a handle to it
def plotCloud(cloud):
    xyz=get_xyz_world_frame(cloud)
    goodinds = np.isfinite(xyz[:,0])
    if isinstance(cloud, cloudprocpy.CloudXYZRGB):
        rgbfloats = cloud_orig_colored.to2dArray()[:,4]
        rgb0 = np.ndarray(buffer=rgbfloats.copy(),shape=(480*640,4),dtype='uint8')
        cloud_handle = env.plot3(xyz[goodinds,:], 2,(rgb0[goodinds,:3][:,::-1])/255. )
    else:
        cloud_handle = env.plot3(xyz[goodinds,:], 2)
    return cloud_handle


# Adds the geometric shape(s) to the env
# Returns a list of the names of the added KinBodies
def generateBodies(cloud, id):
    # BEGIN addtoenv
    names = []
    if args.geom_type == "mesh":
        mesh = generate_mesh(cloud)
        name = "simple_mesh_%i"%id
        mesh_body = mk.create_trimesh(env, get_xyz_world_frame(mesh.getCloud()), np.array(mesh.getFaces()), name=name)
        mesh_body.SetUserData("bt_use_trimesh", True) # Tell collision checker to use the trimesh rather than the convex hull of it
        names.append(name)
    elif args.geom_type == "cd":
        big_mesh = generate_mesh(cloud)
        convex_meshes = cloudprocpy.convexDecompHACD(big_mesh,30)
        for (i,mesh) in enumerate(convex_meshes):
            name = "mesh_%i_%i" % (id,i)
            verts = get_xyz_world_frame(mesh.getCloud())
            mk.create_trimesh(env, verts, mesh.getTriangles(), name=name)
            randcolor = np.random.rand(3)

	    print 'vertices'
	    print mesh.getVertices()

	    if not isValidMesh(mesh):
	        print('Mesh invalid. Removing kinbody')
	        env.RemoveKinBody(env.GetKinBody(name))
	        continue
	

            env.GetKinBody(name).GetLinks()[0].GetGeometries()[0].SetAmbientColor(randcolor)
            env.GetKinBody(name).GetLinks()[0].GetGeometries()[0].SetDiffuseColor(randcolor)
            names.append(name)
    # END  addtoenv        
    elif args.geom_type == "spheres":
        raise Exception("don't use spheres--there's a performance issue")
        cloud_ds = cloudprocpy.downsampleCloud(cloud, .04)
        name = "spheres_%i"%id
        mk.create_spheres(env, get_xyz_world_frame(cloud_ds), .02, name=name)
        names.append(name)
    elif args.geom_type == "boxes":
        cloud_ds = cloudprocpy.downsampleCloud(cloud, .04)
        name = "boxes_%i"%id
        mk.create_boxes(env, get_xyz_world_frame(cloud_ds), .02, name=name)
        names.append(name)
    return names

names = []
cloud_handle = None
# alternate between id 0 and 1 so that there is no name conflicts when adding new KinBodies
id = 0

callbackOnceOnly = True

# Updates the bodies and point cloud of the openrave environment
def callback(cloud_msg):
    global callbackOnceOnly
    if not callbackOnceOnly:
	return
    callbackOnceOnly = False

    # convert from point cloud message to cloudprocpy.CloudXYZ
    points_gen = read_points(cloud_msg, skip_nans=False)
    points = []
    for pt in points_gen:
        pt = list(pt)
        pt.append(1)
        points.append(pt)
    cloud = cloudprocpy.CloudXYZ()
    xyz1 = np.array(points)
    # cloud has to be organized for generateBodies to work
    # hardcoded height and width
    xyz1 = np.reshape(xyz1, (480, 640, 4))
    cloud.from3dArray(xyz1)
    
    global cloud_handle
    global names
    global id
    
    # add new bodies and cloud first
    names2 = generateBodies(cloud, id)

    #cloud_handle2 = plotCloud(cloud)
    

    # remove previous bodies and cloud
    for name in names:
        env.Remove(env.GetKinBody(name))
    del cloud_handle

    names = names2
    #cloud_handle = cloud_handle2
    id = (id+1)%2

    
def listener():
    #rospy.init_node('listener', anonymous=True)
    rospy.Subscriber(args.cloud_topic, PointCloud2, callback)
    rospy.spin()
 
if __name__ == '__main__':
    listener()
