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
    print temp_x, temp_y
    cubic_spline = None
    if alg == "linear":
        cubic_spline = interp1d(temp_x, temp_y)
    elif alg == "cubic":
        cubic_spline = interp1d(temp_x, temp_y, kind='cubic')
    
    waypoint_x_start = temp_x[0]
    waypoint_x_end = temp_x[-1]
    length =  (int)(abs(waypoint_x_end - waypoint_x_start)/0.01)
    print length
    ix = np.linspace(waypoint_x_start, waypoint_x_end , num=length)
    iy = cubic_spline(ix)

    return ix, iy

class LocalPathGenerator(object):
    def __init__(self, ip):
        self.rate = rospy.Rate(100)
        self.connect(ip)
        self.path = rospy.Publisher("/path", Path, queue_size=10)
        self.people_marker = rospy.Publisher("/people_marker", Marker, queue_size=10)

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
    
    def _pose2d_to_geometry(self, pose2d):
        pose = Pose()
        pose.position.x = pose2d[0]
        pose.position.y = pose2d[1]
        pose.position.z = 0.0
        q = tf_conversions.transformations.quaternion_from_euler(0.0, 0.0, pose2d[2])
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

    def _get_human_perception(self):
        human_around = self.perception.humansAroundPrivate.value()
        self.motion2robot = almath.Pose2D(self.motion.getRobotPosition(True))

        people_list = list()
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
            people_list.append(list(people_position.toVector()))
        return people_list

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

    def run(self):
        while not rospy.is_shutdown():
            people_list = self._get_human_perception()
            #self._publish_tf(self._pose2d_to_geometry(list(self.motion2robot.inverse().toVector())), "robot_pose", "robot_pose")
            if people_list != []:
                self._publish_tf(self._pose2d_to_geometry(people_list[0]), "robot_pose", "people")
                self._publish_marker(self._pose2d_to_geometry(people_list[0]))
                self._generate_local_path(people_list)
            self.rate.sleep()

    def velocity_control(self):
        pass

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='test')
    parser.add_argument('--ip', help='robot ip address')
    args = parser.parse_args()

    rospy.init_node('local_path_generator_node')
    instance = LocalPathGenerator(args.ip)
    instance.run()
    rospy.spin()