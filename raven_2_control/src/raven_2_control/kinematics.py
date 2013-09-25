import roslib
roslib.load_manifest('RavenDebridement')
import rospy

import numpy as np
import tfx

from math import *

SHOULDER   =0
ELBOW      =1
Z_INS      =2
TOOL_ROT   =4
WRIST      =5
GRASP1     =6
GRASP2     =7
YAW        =8
GRASP      =9

A12  = 1.30899694; #Link1 - 75deg in RAD
A23 = 0.907571211  #Link2 - 52deg in RAD - was set to 60

GOLD_ARM_ID = 'L'
GREEN_ARM_ID = 'R'

RAD2DEG = 180. / pi
DEG2RAD = pi / 180.

TOOL_POSE_AXES_TRANSFORM = tfx.transform((1,0,0,  0,-1,0,  0,0,-1));

SHOULDER_OFFSET_GOLD = atan(0.3471/0.9014) #from original URDF
TOOL_ROT_OFFSET_GOLD = -pi/2
WRIST_OFFSET_GOLD = 0

SHOULDER_OFFSET_GREEN = atan(0.3471/0.9014)#from original URDF
TOOL_ROT_OFFSET_GREEN = -pi/2
WRIST_OFFSET_GREEN = 0

THETA_12 = -A12
THETA_23 = -A23
DW = 0.012

Z = lambda theta,d: tfx.transform([[cos(float(theta)),-sin(float(theta)),0], [sin(theta),cos(float(theta)),0], [0,0,1]],(0,0,d))
X = lambda alpha,a: tfx.transform([[1,0,0], [0,cos(float(alpha)),-sin(float(alpha))], [0,sin(float(alpha)),cos(float(alpha))]],(a,0,0))

Tw2b = tfx.transform((0,-1,0, 0,0,-1, 1,0,0));

def THS_TO_IK( armId, ths):
    if armId == GOLD_ARM_ID:
        return fix_angle(ths + SHOULDER_OFFSET_GOLD)
    else:
        return fix_angle(pi - ths - SHOULDER_OFFSET_GREEN)

def THS_FROM_IK(armId, ths):
    if armId == GOLD_ARM_ID:
        return fix_angle(ths - SHOULDER_OFFSET_GOLD)
    else:
        return fix_angle(pi - ths - SHOULDER_OFFSET_GREEN)

Zs = lambda ths: Z(ths,0)

Xu = X(THETA_12,0);

def THE_TO_IK(  armId, the):
    if armId == GOLD_ARM_ID:
        return the
    else:
        return -the

def THE_FROM_IK(armId, the):
    if armId == GOLD_ARM_ID:
        return the
    else:
        return -the

Ze = lambda the: Z(the,0)

Xf = X(THETA_23,0);

def THR_TO_IK(  armId, thr):
    if armId == GOLD_ARM_ID:
        return fix_angle(-thr + TOOL_ROT_OFFSET_GOLD)
    else:
        return fix_angle(thr + TOOL_ROT_OFFSET_GREEN)

def THR_FROM_IK(armId, thr):
    if armId == GOLD_ARM_ID:
        return fix_angle(-thr + TOOL_ROT_OFFSET_GOLD)
    else:
        return fix_angle(thr - TOOL_ROT_OFFSET_GREEN)

Zr = lambda thr: Z(thr,0)

def D_TO_IK(  armId, d):
    return -d

def D_FROM_IK(armId, d):
    return -d
Zi = lambda d: Z(0,d)

Xip = tfx.transform((0,-1,0, 0,0,-1, 1,0,0));

def THP_TO_IK(  armId, thp):
    if armId == GOLD_ARM_ID:
        return fix_angle(-thp + WRIST_OFFSET_GOLD)
    else:
        return thp

def THP_FROM_IK(armId, thp):
    if armId == GOLD_ARM_ID:
        return fix_angle(-thp - WRIST_OFFSET_GOLD)
    else:
        return thp

Zp = lambda thp: Z(thp,0)

Xpy = tfx.transform((1,0,0, 0,0,-1, 0,1,0),(DW,0,0));

def MECH_GRASP_FROM_MECH_FINGERS(armId, g1, g2):
    if armId == GOLD_ARM_ID:
        return (g2 + g1) # * 1000.
    else:
        return (g2 + g1) # * 1000.

def ACTUAL_GRASP_FROM_MECH_GRASP(armId, grasp):
    if armId == GOLD_ARM_ID:
        return grasp #/ 1000.
    else:
        return -grasp #/ 1000.

def GRASP_TO_IK(armId, grasp):
    if armId == GOLD_ARM_ID:
        return grasp #/ 1000.
    else:
        return -grasp #/ 1000.

def FINGER1_FROM_IK(armId, thy, grasp):
    if armId == GOLD_ARM_ID:
        return  thy + grasp/2
    else:
        return -( thy + grasp/2)

def FINGER2_FROM_IK(armId, thy, grasp):
    if armId == GOLD_ARM_ID:
        return -thy + grasp/2
    else:
        return -(-thy + grasp/2)

def THY_MECH_FROM_FINGERS(  armId, g1, g2):
    if armId == GOLD_ARM_ID:
        return (g2 - g1) / 2
    else:
        return -(g2 - g1) / 2

def FINGER1_FROM_THY_AND_GRASP_MECH(armId, thy, grasp):
    if armId == GOLD_ARM_ID:
        return -thy + grasp/2 #(grasp * 1000. / 2)
    else:
        return thy + (-grasp/2) #(-grasp * 1000. / 2)

def FINGER2_FROM_THY_AND_GRASP_MECH(armId, thy, grasp):
    if armId == GOLD_ARM_ID:
        return thy + grasp/2 #(grasp * 1000. / 2)
    else:
        return -thy + (-grasp/2) #(-grasp * 1000. / 2)

def THY_TO_IK_FROM_FINGERS( armId, g1,  g2):
    if armId == GOLD_ARM_ID:
        return -1 * THY_MECH_FROM_FINGERS(armId,g1,g2)
    else:
        return -1 * THY_MECH_FROM_FINGERS(armId,g1,g2)

def THY_FROM_IK(armId, thy, grasp):
    if armId == GOLD_ARM_ID:
        return 1 * thy
    else:
        return 1 * thy

Zy = lambda thy: Z(thy,0)

Tg = tfx.transform((0.01,0,0));

Tikw2g = lambda ths,the,thr,d,thp,thy: (Tw2b * Zs(ths) * Xu * Ze(the) * Xf * Zr(thr) * Zi(d) * Xip * Zp(thp) * Xpy * Zy(thy) * Tg)
Tw2g = lambda armId,ths,the,thr,d,thp,thy: (actual_world_to_ik_world(armId) * Tikw2g(ths,the,thr,d,thp,thy))

Tw2s = lambda armId,ths: (actual_world_to_ik_world(armId) * Tw2b * Zs(ths))
Tw2e = lambda armId,ths,the: (actual_world_to_ik_world(armId) * Tw2b * Zs(ths) * Xu * Ze(the))
Tw2r = lambda armId,ths,the,thr: (actual_world_to_ik_world(armId) * Tw2b * Zs(ths) * Xu * Ze(the) * Xf * Zr(thr))
Tw2i = lambda armId,ths,the,thr,d: (actual_world_to_ik_world(armId) * Tw2b * Zs(ths) * Xu * Ze(the) * Xf * Zr(thr) * Zi(d))
Tw2p = lambda armId,ths,the,thr,d,thp: (actual_world_to_ik_world(armId) * Tw2b * Zs(ths) * Xu * Ze(the) * Xf * Zr(thr) * Zi(d) * Xip * Zp(thp))
Tw2y = lambda armId,ths,the,thr,d,thp,thy: (actual_world_to_ik_world(armId) * Tw2b * Zs(ths) * Xu * Ze(the) * Xf * Zr(thr) * Zi(d) * Xip * Zp(thp) * Xpy * Zy(thy))

def actual_world_to_ik_world(armId):
    if armId == GOLD_ARM_ID:
        return tfx.transform([[0,1,0],[ -1,0,0],[ 0,0,1]])
    else:
        return tfx.transform([[0,-1,0],[1,0,0], [0,0,1]],(-0.149,0.005,-0.006))

def ik_world_to_actual_world(armId):
    return actual_world_to_ik_world(armId).inverse()

TOOL_POSE_AXES_TRANSFORM = tfx.rotation(1,0,0,  0,-1,0,  0,0,-1)

go_dh_al = [0,  -A12, pi - A23,   0, pi/2, -pi/2]
go_dh_a  = [0,     0,        0,   0,    0,     0]
gr_dh_al = [pi,  A12,      A23,  pi, pi/2,  pi/2]
gr_dh_a  = [0,     0,        0,   0,    0,     0]

SHOULDER_MIN_LIMIT =(   0.0 * DEG2RAD)
SHOULDER_MAX_LIMIT =(  90.0 * DEG2RAD)
ELBOW_MIN_LIMIT =(  45.0 * DEG2RAD)
ELBOW_MAX_LIMIT =( 135.0 * DEG2RAD)

Z_INS_MIN_LIMIT =(-0.230)
Z_INS_MAX_LIMIT =( 0.010)

TOOL_ROLL_MIN_LIMIT =(-182.0 * DEG2RAD)
TOOL_ROLL_MAX_LIMIT =( 182.0 * DEG2RAD)
TOOL_WRIST_MIN_LIMIT =(-75.0 * DEG2RAD)
TOOL_WRIST_MAX_LIMIT =( 75.0 * DEG2RAD)

TOOL_GRASP_LIMIT = 89.0 * DEG2RAD
TOOL_GRASP1_MIN_LIMIT = (-TOOL_GRASP_LIMIT)
TOOL_GRASP1_MAX_LIMIT =   TOOL_GRASP_LIMIT
TOOL_GRASP2_MIN_LIMIT = (-TOOL_GRASP_LIMIT)
TOOL_GRASP2_MAX_LIMIT =   TOOL_GRASP_LIMIT

def fix_angle(angle, center=0.):
    test_angle = angle;
    cnt = 1;
    while (test_angle-center) > pi:
        test_angle = angle - cnt * 2*pi;
        cnt += 1
    angle = test_angle;
    cnt = 1;
    while (test_angle-center) < -pi:
        test_angle = angle + cnt * 2*pi;
        cnt += 1
    return test_angle;

def check_joint_limits1(d_act, thp_act, g1_act, g2_act):
    validity = [0,0,0,0]
    
    any_nan = False;
    if (d_act != d_act): any_nan = True
    if (thp_act != thp_act): any_nan = True
    if (g1_act != g1_act): any_nan = True
    if (g2_act != g2_act): any_nan = True
    if (any_nan):
        return False, validity

    bad = False;
    if d_act < Z_INS_MIN_LIMIT:
        validity[0] = -1;
        bad = True;
    if d_act > Z_INS_MAX_LIMIT:
        validity[0] = +1;
        bad = True;
    if thp_act < TOOL_WRIST_MIN_LIMIT:
        validity[1] = -1;
        bad = True;
    if thp_act > TOOL_WRIST_MAX_LIMIT:
        validity[1] = +1;
        bad = True;
    if g1_act < TOOL_GRASP1_MIN_LIMIT:
        validity[2] = -1;
        bad = True;
    if g1_act > TOOL_GRASP1_MAX_LIMIT:
        validity[2] = +1;
        bad = True;
    if g2_act < TOOL_GRASP2_MIN_LIMIT:
        validity[3] = -1;
        bad = True;
    if g2_act > TOOL_GRASP2_MAX_LIMIT:
        validity[3] = +1;
        bad = True;
    return not bad, validity

def check_joint_limits2(ths_act, the_act, thr_act):
    validity = [0,0,0]    
    
    any_nan = False;
    if (ths_act != ths_act):
        #ROS_INFO("ths_act != ths_act, %f",ths_act);
        any_nan = True;
    if (the_act != the_act):
        #ROS_INFO("the_act != the_act, %f",the_act);
        any_nan = True;
    if (thr_act != thr_act):
        #ROS_INFO("thr_act != thr_act, %f",thr_act);
        any_nan = True;
    if (any_nan):
        #ROS_INFO("any_nan is True in check_joint_limits2_new");
        return False, validity
    

    bad = False;
    if (ths_act < SHOULDER_MIN_LIMIT):
        validity[0] = ths_act - SHOULDER_MIN_LIMIT;
        bad = True;
    if (ths_act > SHOULDER_MAX_LIMIT):
        validity[0] = ths_act - SHOULDER_MAX_LIMIT;
        bad = True;
    if (the_act < ELBOW_MIN_LIMIT):
        validity[1] = the_act - ELBOW_MIN_LIMIT;
        bad = True;
    if (the_act > ELBOW_MAX_LIMIT):
        validity[1] = the_act - ELBOW_MAX_LIMIT;
        bad = True;
    if (thr_act < TOOL_ROLL_MIN_LIMIT):
        validity[2] = thr_act - TOOL_ROLL_MIN_LIMIT;
        bad = True;
    if (thr_act > TOOL_ROLL_MAX_LIMIT):
        validity[2] = thr_act - TOOL_ROLL_MAX_LIMIT;
        bad = True;
    return not bad, validity

def get_joints_with_limits1(d_act, thp_act, g1_act, g2_act):
    joints1 = {}
    
    joints1[Z_INS]    = d_act;
    joints1[WRIST]    = thp_act;
    joints1[GRASP1]   = g1_act;
    joints1[GRASP2]   = g2_act;

    limits=0;

    if (joints1[Z_INS]  < Z_INS_MIN_LIMIT):
        print ("hit insertion min limit");
        limits += 1
        joints1[Z_INS] = Z_INS_MIN_LIMIT;
    elif (joints1[Z_INS]  > Z_INS_MAX_LIMIT):
        print ("hit insertion max limit");
        limits += 1
        joints1[Z_INS] = Z_INS_MAX_LIMIT;

    if (joints1[WRIST]  < TOOL_WRIST_MIN_LIMIT):
        print ("hit tool wrist min limit");
        limits += 1
        joints1[WRIST] = TOOL_WRIST_MIN_LIMIT;
    elif (joints1[WRIST]  > TOOL_WRIST_MAX_LIMIT):
        print ("hit tool wrist max limit");
        limits += 1
        joints1[WRIST] = TOOL_WRIST_MAX_LIMIT;

    if (joints1[GRASP1]  < TOOL_GRASP1_MIN_LIMIT):
        print ("hit grasp1 min limit");
        limits += 1
        joints1[GRASP1] = TOOL_GRASP1_MIN_LIMIT;
    elif (joints1[GRASP1]  > TOOL_GRASP1_MAX_LIMIT):
        print ("hit grasp1 max limit");
        limits += 1
        joints1[GRASP1] = TOOL_GRASP1_MAX_LIMIT;

    if (joints1[GRASP2]  < TOOL_GRASP2_MIN_LIMIT):
        print ("hit grasp2 min limit");
        limits += 1
        joints1[GRASP2] = TOOL_GRASP2_MIN_LIMIT;
    elif (joints1[GRASP2]  > TOOL_GRASP2_MAX_LIMIT):
        print ("hit grasp2 max limit");
        limits += 1
        joints1[GRASP2] = TOOL_GRASP2_MAX_LIMIT;

    """
    print ("Z_INS = %f\n", joints1[Z_INS] RAD2DEG);
    print ("WRIST = %f\n", joints1[WRIST] RAD2DEG);
    print ("GRASP1 = %f\n", joints1[GRASP1] RAD2DEG);
    print ("GRASP2 = %f\n", joints1[GRASP2] RAD2DEG);
    """

    return joints1, limits;

def get_joints_with_limits2(ths_act, the_act, thr_act):
    joints1 = {}
    
    joints1[SHOULDER] = ths_act;
    joints1[ELBOW]    = the_act;
    joints1[TOOL_ROT] = thr_act;

    limits = 0;
    if (joints1[SHOULDER] < SHOULDER_MIN_LIMIT):
        print ("hit shoulder min limit");
        limits += 1
        joints1[SHOULDER] = SHOULDER_MIN_LIMIT;
    elif (joints1[SHOULDER] > SHOULDER_MAX_LIMIT):
        print ("hit shoulder max limit");
        limits += 1
        joints1[SHOULDER] = SHOULDER_MAX_LIMIT;

    if (joints1[ELBOW] < ELBOW_MIN_LIMIT):
        print ("hit elbow min limit");
        limits += 1
        joints1[ELBOW] = ELBOW_MIN_LIMIT;
    elif (joints1[ELBOW] > ELBOW_MAX_LIMIT):
        print ("hit elbow max limit");
        limits += 1
        joints1[ELBOW] = ELBOW_MAX_LIMIT;

    if (joints1[TOOL_ROT] < TOOL_ROLL_MIN_LIMIT):
        print ("hit tool roll min limit");
        limits += 1
        joints1[TOOL_ROT] = TOOL_ROLL_MIN_LIMIT;
    elif (joints1[TOOL_ROT] > TOOL_ROLL_MAX_LIMIT):
        print ("hit tool roll max limit");
        limits += 1
        joints1[TOOL_ROT] = TOOL_ROLL_MAX_LIMIT;

    """
    print "SHOULDER = %f" % (joints1[SHOULDER] * RAD2DEG);
    print "ELBOW = %f" % (joints1[ELBOW] * RAD2DEG);
    print "TOOL_ROT = %f" % (joints1[TOOL_ROT] * RAD2DEG);
    """

    return joints1, limits;

def invArmKin(armId, pose, grasp, debug=False):
    pose = tfx.pose(pose).copy()
    
    pose.orientation = pose.orientation * TOOL_POSE_AXES_TRANSFORM.inverse()
    
    if debug:
        print 'pos (%f, %f, %f)' % (pose.position.x, pose.position.y, pose.position.z)
        ang = tfx.tb_angles(pose.orientation)
        print 'ypr (%f, %f, %f)' % (ang.yaw_deg, ang.pitch_deg, ang.roll_deg)
    
    """
     * Actual pose is in the actual world frame, so we have <actual_world to gripper>
     * The ik world frame is the base frame.
     * Therefore, we need <base to actual_world> to get <base to gripper>.
     * <base to actual_world> is inverse of <actual_world to base>
     *
     * Since the ik is based on the yaw frame (to which the gripper is fixed), we
     * take the pose of the yaw frame, not the gripper frame
    """
    
    ik_pose = (ik_world_to_actual_world(armId) * pose.as_tf() * Tg.inverse()).as_pose()

    th12 = THETA_12;
    th23 = THETA_23;

    ks12 = sin(th12);
    kc12 = cos(th12);
    ks23 = sin(th23);
    kc23 = cos(th23);

    dw = DW;
    
    if debug:
        print 'Tw2g'
        print ik_pose.matrix

    Tgripper_to_world = ik_pose.inverse().as_tf();

    origin_in_gripper_frame = Tgripper_to_world.position;
    px = origin_in_gripper_frame.x
    py = origin_in_gripper_frame.y
    pz = origin_in_gripper_frame.z
    

    thy = atan2(py,-px);

    if abs(thy) < 0.001:
        thp = atan2(-pz, -px/cos(thy) - dw)
    else:
        thp = atan2(-pz,  py/sin(thy) - dw);

    d = -pz / sin(thp);
    
    g2_act = FINGER1_FROM_IK(armId,thy,grasp);#FINGER2_FROM_IK(armId,thy,grasp);

    if debug:
        print 'px %f' % px
        print 'py %f' % py
        print 'pz %f' % pz
        print 'thy %f' % thy
        print 'dw %f' % dw
        print 'thp %f' % thp
        print 'd %f' % d

    d_act = D_FROM_IK(armId,d);
    thp_act = THP_FROM_IK(armId,thp);
    thy_act = THY_FROM_IK(armId,thy,grasp);
    g1_act = FINGER1_FROM_IK(armId,thy,grasp);
    g2_act = FINGER2_FROM_IK(armId,thy,grasp);
    
    #check angles
    valid1, validity1 = check_joint_limits1(d_act,thp_act,g1_act,g2_act);
    if not valid1:
        print "check_joint_limits1_new failed";
        print "ik %s invalid --1-- d [%d] % 2.4f \tp [%d] % 3.1f\ty [%d %d] % 3.1f\n" % (
                    armId,
                    validity1[0],              d_act,
                    validity1[1],              thp_act * RAD2DEG,
                    validity1[2],validity1[3], thy_act * RAD2DEG);
        return None;


    #ROS_INFO("d %f",d);
    #ROS_INFO("thp %f",thp);
    #ROS_INFO("thy %f",thy);

    z_roll_in_world = (Zi(d) * Xip * Zp(thp) * Xpy * Zy(thy) * Tg * Tgripper_to_world).inverse() * [0,0,1]
    x_roll_in_world = (Zi(d) * Xip * Zp(thp) * Xpy * Zy(thy) * Tg * Tgripper_to_world).inverse() * [1,0,0]


    zx = z_roll_in_world.x
    zy = z_roll_in_world.y
    zz = z_roll_in_world.z

    xx = x_roll_in_world.x
    xy = x_roll_in_world.y
    xz = x_roll_in_world.z


    cthe = (zy + kc12*kc23) / (ks12*ks23);

    the_1 = acos(cthe);
    the_2 = -acos(cthe);

    the_opt = (the_1,the_2)


    ths_opt = [None,None]
    thr_opt = [None,None]
    
    opts_valid = [None,None]
    validity2 = [[None,None,None,None],[None,None,None,None]]
    
    ths_act = [None,None]
    the_act = [None,None]
    thr_act = [None,None]
    

    for i in xrange(2):
        sthe_tmp = sin(the_opt[i]);
        C1 = ks12*kc23 + kc12*ks23*cthe;
        C2 = ks23 * sthe_tmp;
        C3 = C2 + C1*C1 / C2;

        ths_opt[i] = atan2(
                -np.sign(C3)*(zx - C1 * zz / C2),
                 np.sign(C3)*(zz + C1 * zx / C2));

        sths_tmp = sin(ths_opt[i]);
        cths_tmp = cos(ths_opt[i]);

        C4 = ks12 * sin(the_opt[i]);
        C5 = kc12 * ks23 + ks12 * kc23 * cos(the_opt[i]);
        C6 = kc23*(sthe_tmp * sths_tmp - kc12*cthe*cths_tmp) + cths_tmp*ks12*ks23;
        C7 = cthe*sths_tmp + kc12*cths_tmp*sthe_tmp;
        
        thr_opt[i] = atan2(
                (xx - C7 * xy / C4) / (C6 + C7*C5/C4),
                (xx + C6 * xy / C5) / (-C6*C4/C5 - C7));

        
        ths_act[i] = THS_FROM_IK(armId,ths_opt[i]);
        the_act[i] = THE_FROM_IK(armId,the_opt[i]);
        thr_act[i] = THR_FROM_IK(armId,thr_opt[i]);
        
        if debug:
            print 'ths_act %f' % (ths_act[i] * RAD2DEG)
            print 'the_act %f' % (the_act[i] * RAD2DEG)
            print 'thr_act %f' % (thr_act[i] * RAD2DEG)
            print 'd_act %f' % (d_act)
            print 'thp_act %f' % (thp_act * RAD2DEG)
            print 'thy_act %f' % (thy_act * RAD2DEG)
            print 'grasp %f' % (grasp * RAD2DEG)
            print 'g1_act %f' % (g1_act * RAD2DEG)
            print 'g2_act %f' % (g2_act * RAD2DEG)
    
        valid2, validity2[i] = check_joint_limits2(ths_act[i],the_act[i],thr_act[i]);
        opts_valid[i] = valid2;

        if valid2:
            #set joints1
            joints1, limits1 = get_joints_with_limits1(d_act,thp_act,g1_act,g2_act);
            joints2, limits2 = get_joints_with_limits2(ths_act[i],the_act[i],thr_act[i])
            joints1 = joints1.copy()
            joints1.update(joints2)
            if armId == 'R':
                joints1[YAW] = (joints1[GRASP1] - joints1[GRASP2])/2.
            else:
                joints1[YAW] = (joints1[GRASP2] - joints1[GRASP1])/2.
            joints1[GRASP] = grasp

    if opts_valid[0]:
        return joints1;
    elif opts_valid[1]:
        return joints1;
    else:
        maxValidDist = 3 * DEG2RAD;
        valid_dist = [None,None]
        for i in xrange(2):
            sum = 0;
            for j in xrange(3):
                v = abs(validity2[i][j]);
                sum += v*v;
            valid_dist[i] = sqrt(sum);

        print 'both check_joint_limits2 failed'
        if valid_dist[0] < maxValidDist and valid_dist[0] < valid_dist[1]:
            get_joints_with_limits1(d_act,thp_act,g1_act,g2_act);
            get_joints_with_limits2(ths_act[0],the_act[0],thr_act[0]);
        elif valid_dist[1] < maxValidDist:
            get_joints_with_limits1(d_act,thp_act,g1_act,g2_act);
            get_joints_with_limits2(ths_act[1],the_act[1],thr_act[1]);
            
        print 'Invalid pose {0} for arm {1}'.format(pose, armId)

        return None;

def fwdArmKin(armId, joints):
    tool_tf = actual_world_to_ik_world(armId) \
                    * Tw2b \
                    * Zs(THS_TO_IK(armId,joints[SHOULDER])) \
                    * Xu \
                    * Ze(THE_TO_IK(armId,joints[ELBOW])) \
                    * Xf \
                    * Zr(THR_TO_IK(armId,joints[TOOL_ROT])) \
                    * Zi(D_TO_IK(armId,joints[Z_INS])) \
                    * Xip \
                    * Zp(THP_TO_IK(armId,joints[WRIST])) \
                    * Xpy \
                    * Zy(THY_TO_IK_FROM_FINGERS(armId,joints[GRASP1],joints[GRASP2])) \
                    * Tg
    
    tool_tf.rotation = tool_tf.rotation * TOOL_POSE_AXES_TRANSFORM
    grasp = ACTUAL_GRASP_FROM_MECH_GRASP(armId,MECH_GRASP_FROM_MECH_FINGERS(armId,joints[GRASP1],joints[GRASP2]))
    return tool_tf.as_pose(frame='/0_link'), grasp





#########################
#      TESTS            #
#########################

def jointRad2deg(j):
    j2 = {}
    for k,v in j.iteritems():
        if k == 2:
            j2[k] = v
        else:
            j2[k] = v * 180. / pi
    return j2

def jointDeg2rad(j):
    j2 = {}
    for k,v in j.iteritems():
        if k == 2:
            j2[k] = v
        else:
            j2[k] = v * pi / 180.
    return j2

def joint_str(j):
    return ', '.join("%d: % .3f" % (k,v) for k,v in j.iteritems())


def test_config1():
    jl = jointDeg2rad({0: 20.6, 
                       1: 79.0, 
                       2: -.112, 
                       4: -84.3, 
                       5: 29.5, 
                       6: 32.8, 
                       7: 29.2})
    pl = tfx.pose((0.008,-0.001,-0.131), tfx.tb_angles(24.7,62.2,179.6), frame='/0_link')
    gl = 61.994 * pi / 180.

    jr = jointDeg2rad({0: 6.3, 
                       1: 107.1, 
                       2: -.085, 
                       4: 20.0, 
                       5: -2.8, 
                       6: -41.9, 
                       7: -15.1})
    pr = tfx.pose((-0.109,0.018,-0.098), tfx.tb_angles(54.3,71.0,165.0), frame='/0_link')
    gr = 57.009 * pi / 180.
    return jl, pl, gl, jr, pr, gr

def test_config2():
    jl = {0: .517, 
          1: 1.623, 
          2: -.05, 
          4: .16, 
          5: .161, 
          6: .99, 
          7: 1.023}
    pl = tfx.pose((-.015,-.014,-.069), tfx.tb_angles(-160.6,75.7,-87.2), frame='/0_link')
    gl = 2.013

    jr = {0: .511, 
          1: 1.607, 
          2: -.05, 
          4: .109, 
          5: .110, 
          6: -.652, 
          7: -.634}
    pr = tfx.pose((-.136,-.017,-.068), tfx.tb_angles(-66.3,68.8,40.3), frame='/0_link')
    gr = 1.286
    return jl, pl, gl, jr, pr, gr

def test_config3():
    jl = {0: .517, 
          1: 1.621, 
          2: -.04995, 
          4: .084, 
          5: .123, 
          6: .946, 
          7: .977}
    pl = tfx.pose((-.015,-.015,-.069), tfx.tb_angles(-151.1,76.0,-73.8), frame='/0_link')
    gl = 1.923

    jr = {0: .509, 
          1: 1.609, 
          2: -.04984, 
          4: .111, 
          5: .117, 
          6: -.637, 
          7: -.604}
    pr = tfx.pose((-.136,-.017,-.068), tfx.tb_angles(-67.6,68.8,39.1), frame='/0_link')
    gr = 1.240
    return jl, pl, gl, jr, pr, gr

from raven_2_msgs.msg import *
class RavenStateListener():
    realJoints = [Constants.JOINT_TYPE_SHOULDER,
                     Constants.JOINT_TYPE_ELBOW,
                     Constants.JOINT_TYPE_INSERTION,
                     Constants.JOINT_TYPE_ROTATION,
                     Constants.JOINT_TYPE_PITCH,
                     Constants.JOINT_TYPE_GRASP_FINGER1,
                     Constants.JOINT_TYPE_GRASP_FINGER2]
        
    def __init__(self):
        self.jointsLeft = dict()
        self.poseLeft = None
        self.graspLeft = 0
        
        self.jointsRight = dict()
        self.poseRight = None
        self.graspRight = 0
        
        rospy.Subscriber('/raven_state', RavenState, self._ravenStateCallback)

    def _ravenStateCallback(self, state):
        for arm in state.arms:
            # jpos = joint.position
            # jpos_d = joint.set_point.position
            joints = dict((joint.type, joint.set_point.position) for joint in arm.joints if joint.type in RavenStateListener.realJoints)
            pose = tfx.pose(arm.tool.pose, frame='/0_link')
            grasp = arm.tool.grasp
            
            if arm.name == 'L':
                self.jointsLeft, self.poseLeft, self.graspLeft = joints, pose, grasp
            else:
                self.jointsRight, self.poseRight, self.graspRight = joints, pose, grasp
                
        
def test_config4():
    ravenListener = RavenStateListener()
    
    while not rospy.is_shutdown() and (ravenListener.poseLeft is None or ravenListener.poseRight is None):
        print 'sleeping...'
        rospy.sleep(.05)
        
    jl = ravenListener.jointsLeft
    pl = ravenListener.poseLeft
    gl = ravenListener.graspLeft
    
    jr = ravenListener.jointsRight
    pr = ravenListener.poseRight
    gr = ravenListener.graspRight
    
    
    return jl, pl, gl, jr, pr, gr
    

def main():
    import roslib
    roslib.load_manifest('RavenDebridement')
    from RavenDebridement.srv import InvKinSrv
    import rospy
    from RavenDebridement.RavenCommand.RavenArm import RavenArm
    import IPython
    
    rospy.init_node('testKinematics',anonymous=True)
    
    rospy.loginfo('Waiting for IK server...')
    rospy.wait_for_service('inv_kin_server',timeout=5)
    inv_kin_service = rospy.ServiceProxy('inv_kin_server', InvKinSrv)
    
    from std_msgs.msg import Header
    
    header = Header()
    header.frame_id = '/0_link'
    header.stamp = rospy.Time.now()
    
    min_limits = np.array([SHOULDER_MIN_LIMIT,ELBOW_MIN_LIMIT,Z_INS_MIN_LIMIT,TOOL_ROLL_MIN_LIMIT,TOOL_WRIST_MIN_LIMIT,TOOL_GRASP1_MIN_LIMIT,TOOL_GRASP2_MIN_LIMIT])
    max_limits = np.array([SHOULDER_MAX_LIMIT,ELBOW_MAX_LIMIT,Z_INS_MAX_LIMIT,TOOL_ROLL_MAX_LIMIT,TOOL_WRIST_MAX_LIMIT,TOOL_GRASP1_MAX_LIMIT,TOOL_GRASP2_MAX_LIMIT])
    
    armName = GOLD_ARM_ID
    
    for i in xrange(1):
        r = np.random.rand(len(min_limits))
    
        val = min_limits + r * (max_limits - min_limits)
        
        joints1 = dict(zip([0,1,2,4,5,6,7],val.tolist()))
        
        joints1[TOOL_ROT] = fix_angle(joints1[TOOL_ROT])
        
        pose, grasp = fwdArmKin(armName,joints1)
        
        joints2 = invArmKin(armName, pose, grasp)
        
        joints2[TOOL_ROT] = fix_angle(joints2[TOOL_ROT])
        
        try:
            srvArmName = 0 if armName == GOLD_ARM_ID else 1
            resp = inv_kin_service(header, srvArmName, grasp, pose.msg.Pose())
            joints3 = dict((joint.type, joint.position) for joint in resp.joints)
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.loginfo("IK server failure: %s"%e)
        
#         print '\t'.join("%.3f" % v for v in joints1.values())
#         print '\t'.join("%.3f" % v for v in joints2.values())
#         print '\t'.join("%.3f" % v for v in joints3.values())
#         print np.array(joints1.values()) - np.array(joints2.values())

    def invArmKinSrv(armId, pose, grasp):
        srvArmName = 0 if armId == 'L' else 1
        resp = inv_kin_service(header, srvArmName, grasp, pose.msg.Pose())
        resp_joints = dict((joint.type, joint.position) for joint in resp.joints if joint.type <= 7)
        return resp_joints
    
    
    
    print '-----------------------------------------------'
    
    jl, pl, gl, jr, pr, gr = test_config4()
    
    """
    la = RavenArm('L')
    #ra = RavenArm('R')
    la.start()
    #ra.start()
    rospy.sleep(2)

    la.stop()
    #ra.stop()

    print '\n------p->j------'
    print '--L--'
    jl1 = invArmKin('L', pl, gl)
    jl2 = invArmKinSrv('L', pl, gl)
    print 'actual:', joint_str(jointRad2deg(jl))
    print 'calc:  ', joint_str(jointRad2deg(jl1))
    print 'calc2: ', joint_str(jointRad2deg(jl2))
    """
    
    
    print '\n------j->p------'
    print '--L--'
    pl1,gl1 = fwdArmKin('L',jl)
    print 'actual:', pl
    print 'calc:  ', pl1
    print 'actual:', gl
    print 'calc:  ', gl1
    print '--R--'
    pr1,gr1 = fwdArmKin('R',jr)
    print 'actual:', pr
    print 'calc:  ', pr1
    print 'actual:', gr
    print 'calc:  ', gr1
    
    print '\n------p->j------'
    print '--L--'
    jl1 = invArmKin('L', pl, gl)
    jl2 = invArmKinSrv('L', pl, gl)
    print 'actual:', joint_str(jl)
    print 'calc:  ', joint_str(jl1)
    print 'calc2: ', joint_str(jl2)
    print '--R--'
    jr1 = invArmKin('R', pr, gr)
    jr2 = invArmKinSrv('R', pr, gr)
    print 'actual:', joint_str(jr)
    print 'calc:  ', joint_str(jr1)
    print 'calc2: ', joint_str(jr2)

    print '\n------j->p->j------'
    print '--L--'
    jl1 = invArmKin('L', pl1, gl1)
    print 'actual:', joint_str(jl)
    print 'calc:  ', joint_str(jl1)
    print '--R--'
    jr1 = invArmKin('R', pr1, gr1)
    print 'actual:', joint_str(jr)
    print 'calc:  ', joint_str(jr1)
    
    print '\n------p->j->p------'
    print '--L--'
    pl1,gl1 = fwdArmKin('L',jl1)
    print 'actual:', pl
    print 'calc:  ', pl1
    print 'actual:', gl
    print 'calc:  ', gl1
    print '--R--'
    pr1,gr1 = fwdArmKin('R',jr1)
    print 'actual:', pr
    print 'calc:  ', pr1
    print 'actual:', gr
    print 'calc:  ', gr1
    
    from RavenDebridement.Utils import Util
    d = Util.deltaPose(pl, pl1)
    a = tfx.tb_angles(d.orientation)
    IPython.embed()

if __name__ == '__main__':
    main()