#-*- coding: utf-8 -*-
import rospy
import tf2_ros
import tf_conversions
import geometry_msgs
from geometry_msgs.msg import Quaternion, Pose
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
from scipy.interpolate import interp1d
import numpy as np
import qi
import almath
import argparse
import math
import sys

NOISE_THRESHOLD = 0.00001
MAX_LENGTH      = 5

class Authenticator:
    def __init__(self, user, pswd):
        self.user = user
        self.pswd = pswd

    def initialAuthData(self):
        cm = {'user': self.user, 'token': self.pswd}
        return cm

class ClientFactory:
    def __init__(self, user, pswd):
        self.user = user
        self.pswd = pswd

    def newAuthenticator(self):
        return Authenticator(self.user, self.pswd)

def interpolation(waypoint_list, alg="linear"):
    ix = iy = []

    temp_x = temp_y = np.array([])
    for waypoint in waypoint_list:
        temp_x = np.append(temp_x, waypoint[0])
        temp_y = np.append(temp_y, waypoint[1])
    cubic_spline = None
    if alg == "linear":
        cubic_spline = interp1d(temp_x, temp_y)
    elif alg == "cubic":
        cubic_spline = interp1d(temp_x, temp_y, kind='cubic')
    
    waypoint_x_start = temp_x[0]
    waypoint_x_end = temp_x[-1]
    length =  (int)(abs(waypoint_x_end - waypoint_x_start)/0.01)
    ix = np.linspace(waypoint_x_start, waypoint_x_end , num=length)
    iy = cubic_spline(ix)

    return ix, iy
# ロボットが移動した際の位置の変位を計算し、map座標系に置き換えてオドメトリとして計算する
class OdometryTracker(object):
    def __init__(self, session):
        self.session = session
        self.motion = self.session.service("ALMotion")

        self.map2odom = almath.Pose2D()
        self.old_displacement = np.array(self.motion._getCumulatedDisplacement())
        self.old_position = np.array(self.motion.getRobotPosition(True))

        self.init_position = np.array([0,0,0])

        self._odometry = {"position": almath.Pose2D(), "displacement": almath.Pose2D()}

        self._tracker_task = qi.PeriodicTask()
        self._tracker_task.setCallback(self._tracker)
        self._tracker_task.setUsPeriod(200 * 1000)
        #self._tracker_task.start(True)

    def _tracker(self):
        position = np.array(self.motion.getRobotPosition(True))
        displacement = np.array(self.motion._getCumulatedDisplacement())

        dpos = np.sum(position-self.old_position)
        ddis = np.sum(displacement-self.old_displacement)

        if dpos > NOISE_THRESHOLD or ddis > NOISE_THRESHOLD:
            world2odom =  almath.Pose2D(self.init_position.tolist()) + almath.Pose2D(dpos.tolist())
            self._odometry = {"position": almath.Pose2D(position), "displacement": almath.Pose2D(displacement.tolist())}

        self.old_displacement = displacement
        self.old_position = position

    def _get_odomery(self):
        return self._odometry

class SteeringControl(object):
    def __init__(self, session):
        self.session = session
        self.move_flag = True
        self._motion = self.session.service("ALMotion")

        self._max_linear_acc = 1.0
        self._max_linear_vel = 1.0
        self._max_angular_vel = 1.0
        self._max_angular_acc = 2.0

        self._min_linear_vel = 0.05
        self._min_angular_vel = 0.0

        self._update_parameter()

        self._odometry_tracker = OdometryTracker(session)

        self._prev_linear_vel = 0.0
        self._prev_angular_vel = 0.0

        self._T = 0.2

        self._goal_translation_tolerance = 0.02
        self._goal_rotation_tolerance = 0.3

        self.goal = almath.Position2D()
        self.velocity = {'x':0.0, 'y':0.0}

        self._steering_task = qi.PeriodicTask()
        self._steering_task.setCallback(self._main_thread)
        self._steering_task.setUsPeriod(10 * 1000)
        self._steering_task.start(True)

    def _init_paramter(self):
        self._prev_linear_vel = 0.0
        self._prev_angular_vel = 0.0

        self.velocity = {'x':0.0, 'y':0.0}

    def _update_parameter(self):
        self._f = self._max_linear_acc
        self._b = self._f / (2.0 * self._max_linear_vel)
        self._h = (2.0 * self._b * self._max_angular_vel) / self._f
        self._k_i = self._max_angular_acc / (self._f * self._h)

    def move(self, x, y, theta):
        if not self.move_flag:
            self.move_flag = True
        self.goal.x = x
        self.goal.y = y

    def _movement_generator(self, F):
        v=0
        w=0
        try:
            if F.norm() > self._f:
                angle = math.atan2(F.y, F.x)
                F.x = math.cos(angle) * self._f
                F.y = math.sin(angle) * self._f

            v = (F.x * self._T + self._prev_angular_vel) / (1.0 + 2.0 * self._b * self._T)
            w = (self._k_i * self._h * F.y + self._prev_angular_vel) / (1.0 + 2.0 * self._b * self._k_i * self._T)

            if v < 0:
                v = 0
            if math.fabs(w) < 1e-4:
                w = 0
        except Exception as error1:
            self._failure(error1)

        return F, v, w

    def _normalize(self, angle):
        return angle + (2*almath.PI)*math.floor((almath.PI-angle)/(2*almath.PI))

    def _failure(self, e):
        exc_type, exc_obj, tb = sys.exc_info()
        lineno = tb.tb_lineno
        print(str(lineno)+":"+str(type(e)))

    def _compute_velocity(self, robot_pose):
        reached = False

        try:
            robot_pose_xy = almath.Position2D(robot_pose.x, robot_pose.y)
            distance_goal = self.goal - robot_pose_xy

            if distance_goal.norm() > self._goal_translation_tolerance:
                angle_goal = self._normalize(math.atan2(distance_goal.y, distance_goal.x) - robot_pose.theta)

                F = almath.Position2D(1.5*math.cos(angle_goal), 1.5*math.sin(angle_goal))

                F, v, w = self._movement_generator(F)

                if math.fabs(angle_goal) > self._goal_translation_tolerance:
                    #if math.fabs(w) > 0 and math.fabs(angle_goal) > self._goal_rotation_tolerance:
                    if math.fabs(angle_goal) and math.fabs(w) < self._min_angular_vel:
                        w = -self._min_angular_vel if w<0 else self._min_angular_vel
                    self.velocity["y"] = w
                else:
                    self.velocity["x"] = v
                    self.velocity["y"] = w
            else:
                reached = True
        except Exception as error1:
            self._failure(error1)

        self._prev_linear_vel = self.velocity["x"]
        self._prev_angular_vel = self.velocity["y"]

        return reached

    def _main_thread(self):
        if True:
            odometry = self._odometry_tracker._get_odomery()
            robot_pose = odometry["position"]
            if self._compute_velocity(robot_pose):
                self.move_flag = False
                self._init_paramter()
            self._motion.move(self.velocity["x"], 0.0, self.velocity["y"])

# 人検出と簡易的なローカルパス生成
class LocalPathGenerator(object):
    def __init__(self, ip):
        self.rate = rospy.Rate(100)
        self.connect(ip)
        self.path = rospy.Publisher("/path", Path, queue_size=10)
        self.people_marker = rospy.Publisher("/people_marker", Marker, queue_size=10)

        #self._steering_control = SteeringControl(self.session)

        self._head_pitch = self._head_yaw = 0.0

        self.x = self.y = self.theta = 0.0
        
        self.vel_flag = False
        self.pos_flag = False

        self._steering_task = qi.PeriodicTask()
        self._steering_task.setCallback(self._main_thread)
        self._steering_task.setUsPeriod(10 * 1000)
        self._steering_task.start(True)

        self._buffer = list()

    def connect(self, ip):
        try:
            self.session = qi.Session()
            try:
                self.session.connect("tcp://{}:9559".format(ip))
            except Exception as error:
                try:
                    factory = ClientFactory('nao', 'nao')
                    self.session.setClientAuthenticatorFactory(factory)
                    self.session.connect("tcps://{}:9503".format(ip))
                    rospy.loginfo("connection is successful")
                except Exception as error1:
                    rospy.logerr(str(error1))
            self.motion = self.session.service("ALMotion")
            self.boot_frame = self.motion._robotAtBootFrame()
            self.perception = self.session.service("HumanPerception")
        except Exception as error2:
            rospy.logerr("Error when creating proxy:")
            rospy.logerr(str(error2))
    
    def _pose2d_to_geometry(self, pose6d):
        pose6d = almath.Position6D(pose6d)
        pose = Pose()
        pose.position.x = pose6d.x
        pose.position.y = pose6d.y
        pose.position.z = pose6d.z
        q = tf_conversions.transformations.quaternion_from_euler(pose6d.wx, pose6d.wy, pose6d.wz)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        return pose

    def _publish_marker(self, data):
        marker_data = Marker()
        marker_data.header.frame_id = "robot_pose"
        marker_data.header.stamp = rospy.Time.now()

        marker_data.ns = "basic_shapes"
        marker_data.id = 0

        marker_data.action = Marker.ADD
        
        marker_data.pose.position.x = data.position.x
        marker_data.pose.position.y = data.position.y
        marker_data.pose.position.z = data.position.z
        marker_data.pose.orientation.x = data.orientation.x
        marker_data.pose.orientation.y = data.orientation.y
        marker_data.pose.orientation.z = data.orientation.z
        marker_data.pose.orientation.w = data.orientation.w

        marker_data.color.r = 1.0
        marker_data.color.g = 0.0
        marker_data.color.b = 0.0
        marker_data.color.a = 0.7

        marker_data.scale.x = 0.5
        marker_data.scale.y = 0.5
        marker_data.scale.z = 2.0

        marker_data.lifetime = rospy.Duration()

        marker_data.type = 1

        self.people_marker.publish(marker_data)

    # 外れ値フィルタ
    def _filter(self, data):
        data_xy = {"x": data[0], "y":data[1]}
        self._buffer.append(data_xy)
        sum_x = sum_y = 0
        for index in range(len(self._buffer)):
            sum_x = sum_x + self._buffer[index]["x"]
            sum_y = sum_y + self._buffer[index]["y"]
        filter_x = sum_x / len(self._buffer)
        filter_y = sum_y / len(self._buffer)
        if len(self._buffer) == MAX_LENGTH:
            self._buffer.pop(-1)
        return filter_x, filter_y

    def _get_human_perception(self):
        human_around = self.perception.humansAroundPrivate.value()
        self.motion2robot = almath.Pose2D(self.motion.getRobotPosition(True))#self._steering_control._odometry_tracker._get_odomery()["position"]

        people_list = list()
        world2people = list()
        for people in human_around:
            human_frame = people.headFrame.value()
            tf = human_frame.computeTransform(self.boot_frame)['transform']

            w = float(tf['rotation']['w'])
            x = float(tf['rotation']['x'])
            y = float(tf['rotation']['y'])
            z = float(tf['rotation']['z'])
            people_rotation = almath.Quaternion(w, x, y, z)
            rot_3d = almath.rotation3DFromQuaternion(people_rotation)

            people_tf = almath.transformFromQuaternion(people_rotation)
            people_tf.r1_c4 = float(tf['translation']['x'])
            people_tf.r2_c4 = float(tf['translation']['y'])
            people_tf.r3_c4 = float(tf['translation']['z'])
            people_position_tf = almath.pose2DFromTransform(people_tf)
            # robot->world, world->people ==>> robot->people
            people_position = self.motion2robot.inverse() * people_position_tf

            t_motion2robot = almath.transformFromPose2D(self.motion2robot)
            t_world2people = almath.transformFromPosition6D(almath.position6DFromTransform(people_tf))
            t_people_position = t_motion2robot.inverse() * t_world2people

            people_list.append(list(almath.position6DFromTransform(t_people_position).toVector()))
            world2people.append(list(people_position_tf.toVector()))
        return people_list, world2people

    def _publish_tf(self, data, frame_id, child_frame_id):
        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = frame_id
        t.child_frame_id = child_frame_id
        t.transform.translation.x = data.position.x
        t.transform.translation.y = data.position.y
        t.transform.translation.z = data.position.z
        t.transform.rotation.x = data.orientation.x
        t.transform.rotation.y = data.orientation.y
        t.transform.rotation.z = data.orientation.z
        t.transform.rotation.w = data.orientation.w

        br.sendTransform(t)

    def _generate_local_path(self, target_list):
        waypoint_list = list()
        waypoint_list.append([0.0,0.0,0.0])
        waypoint_list.append(target_list[0])
        # ロボットの位置から人の位置まで補間する
        x, y = interpolation(waypoint_list)
        path = Path()
        for index in range(len(x)):
            pose = PoseStamped()
            pose.pose.position.x = x[index]
            pose.pose.position.y = y[index]
            path.poses.append(pose)
        path.header.frame_id = "robot_pose"
        path.header.stamp = rospy.Time.now()
        self.path.publish(path)

    # 検出した人を追従するための速度の決定
    def _velocity_control(self, x, y, theta):
        diff_angle = math.atan2(y, x)
        if 0.3 < diff_angle:
            self.x = self.y = 0.0
            self.theta = 0.6
        elif diff_angle < -0.3:
            self.x = self.y = 0.0
            self.theta = -0.6
        else:
            self.theta = 0.0
            if 0.5 < math.fabs(x):
                self.x = 0.2
            elif math.fabs(x) <= 0.5:
                self.x = -0.15
            elif 0.5 < math.fbas(x) < 0.6:
                self.x = 0.0
            else:
                self.x = 0.0
            if 0.2 < y:
                self.y = 0.1    
            elif y < -0.2:
                self.y = -0.1
            else:
                self.y = 0.0

    def _head_control(self, data):
        try:
            head_pitch_z = almath.position6DFromTransform(almath.Transform(self.motion.getTransform("HeadPitch", 2, 0))).z
            self._head_pitch = math.atan2(head_pitch_z - data[2], data[0])
            self._head_yaw = math.atan2(data[1], data[0])
            print self._head_pitch
        except Exception as error:
            self._failure(error)

    # 位置制御ベース
    def _position_control(self, x, y, theta):
        self.x = x
        self.y = y

    def _failure(self, e):
        exc_type, exc_obj, tb = sys.exc_info()
        lineno = tb.tb_lineno
        print(str(lineno)+":"+str(type(e)))

    def _main_thread(self):
        try:
            self.motion.move(self.x, self.y, self.theta)
            self.motion.angleInterpolationWithSpeed(["HeadPitch", "HeadYaw"], [self._head_pitch, self._head_yaw], 1)
            #if self.x != self.pre_x or self.y != self.pre_y or self.theta != self.pre_theta:
            #if self.vel_flag and not self.pos_flag:
                #;self.motion.move(0, 0, self.theta)
            #elif not self.vel_flag and self.pos_flag:
            #    self.motion.moveTo(self.x, self.y, 0)
        except Exception as e:
            self._failure(e)

    def run(self):
        self.motion.stiffnessInterpolation(["HeadPitch", "HeadYaw"], [0.0, 0.0], 1.0)
        while not rospy.is_shutdown():
            people_list, world2people = self._get_human_perception()
            self._publish_tf(self._pose2d_to_geometry(list(almath.position6DFromPose2D(self.motion2robot.inverse()).toVector())), "map", "robot_pose")
            if people_list != []:
                #filter_x, filter_y = self._filter(people_list[0])
                #print "people pose:", filter_x, filter_y, people_list[0][2]
                self._publish_tf(self._pose2d_to_geometry(people_list[0]), "robot_pose", "people")
                self._publish_marker(self._pose2d_to_geometry(people_list[0]))
                #self._publish_marker(self._pose2d_to_geometry([filter_x, filter_y, people_list[0][2]]))
                #self._generate_local_path(people_list)
                # ワールド座標系における目標位置を設定する(ここではワールド座標系における一番近い人)
                #self._steering_control.move(world2people[0][0], world2people[0][1], world2people[0][2])
                # 速度制御実行
                #self._steering_control._main_thread()
                self._velocity_control(people_list[0][0], people_list[0][1], people_list[0][5])
                self._head_control(people_list[0])
                #self._position_control(people_list[0][0], people_list[0][1], people_list[0][2])
            self.rate.sleep()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='test')
    parser.add_argument('--ip', help='robot ip address')
    args = parser.parse_args()

    rospy.init_node('local_path_generator_node')
    instance = LocalPathGenerator(args.ip)
    instance.run()
    rospy.spin()
