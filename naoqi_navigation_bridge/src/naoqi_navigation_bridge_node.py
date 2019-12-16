import qi
import rospy
import tf2_ros
import tf_conversions
import geometry_msgs
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose
from nav_msgs.msg import OccupancyGrid
import almath

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

class NaoqiNavigationBridge:
    def __init__(self):
        self.rate = rospy.Rate(100)
        self.map = rospy.Publisher("/naoqi_exploration_map", OccupancyGrid, queue_size=None)
        self.people_marker = rospy.Publisher("/people_marker", Marker, queue_size=10)
        self.connect("10.40.17.131")

    def connect(self, ip):
        try:
            try:
                self.session = qi.Session()
                self.session.connect("tcp://{}:9559".format(str(ip)))
            except Exception as err1:
                try:
                    self.session = qi.Session()
                    factory = ClientFactory("nao", "nao")
                    self.session.setClientAuthenticatorFactory(factory)
                    self.session.connect("tcps://{}:9503".format(ip))
                    rospy.loginfo("ok connection")
                except Exception as err2:
                    rospy.logerr(str(err2))
            self.navigation = self.session.service("ALNavigation")
            self.perception = self.session.service("HumanPerception")
            self.motion = self.session.service("ALMotion")
            self.boot_frame = self.motion._robotAtBootFrame()
            self.map2robot = almath.Pose2D()
            self.motion2robot = almath.Pose2D()
        except Exception as err3:
            rospy.logerr("Error when creating proxy:")
            rospy.logerr(str(err3))
            self.running = False
    
    def get_quaternion(self, quaternion):
        output = Quaternion()
        output.w = quaternion.w
        output.x = quaternion.x
        output.y = quaternion.y
        output.z = quaternion.z
        return output

    def _map2pose(self, position):
        pose = Pose()
        pose.position.x = position.x
        pose.position.y = position.y
        pose.position.z = 0.0
        q = tf_conversions.transformations.quaternion_from_euler(0.0, 0.0, position.theta)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        return pose

    def _publish_tf(self, data, frame_id):
        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "map"
        t.child_frame_id = frame_id
        t.transform.translation.x = data.position.x
        t.transform.translation.y = data.position.y
        t.transform.translation.z = data.position.z
        t.transform.rotation.x = data.orientation.x
        t.transform.rotation.y = data.orientation.y
        t.transform.rotation.z = data.orientation.z
        t.transform.rotation.w = data.orientation.w

        br.sendTransform(t)

    def _publish_map(self):
        if self.map.get_num_connections() > 0:
            aggregated_map = None
            try:
                aggregated_map = self.navigation.getMetricalMap()
            except Exception as err:
                rospy.logerr(str(err))
            map_marker = OccupancyGrid()
            map_marker.header.stamp = rospy.Time.now()
            map_marker.header.frame_id = "/map"
            map_marker.info.resolution = aggregated_map[0]
            map_marker.info.width = aggregated_map[1]
            map_marker.info.height = aggregated_map[2]
            map_marker.info.origin.orientation = self.get_quaternion(almath.Quaternion_fromAngleAndAxisRotation(-1.57, 0, 0, 1))
            map_marker.info.origin.position.x = aggregated_map[3][0]
            map_marker.info.origin.position.y = aggregated_map[3][1]
            map_marker.data = aggregated_map[4]
            self.map.publish(map_marker)

    def _robot_pose(self):
        try:
            robot_in_map = self.navigation.getRobotPositionInMap()
            self.motion2robot = almath.Pose2D(self.motion.getRobotPosition(True))
            self.map2robot = almath.Pose2D(robot_in_map[0][0], robot_in_map[0][1], robot_in_map[0][2])
            self._publish_tf(self._map2pose(self.map2robot), "robot_pose")
        except Exception as err:
            rospy.logerr(str(err))

    def _publish_marker(self, data):
        marker_data = Marker()
        marker_data.header.frame_id = "map"
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
        marker_data.color.a = 0.8

        marker_data.scale.x = 0.5
        marker_data.scale.y = 0.5
        marker_data.scale.z = 2.0

        marker_data.lifetime = rospy.Duration()

        marker_data.type = 1

        self.people_marker.publish(marker_data)

    def _publish_perception(self):
        human_around = self.perception.humansAroundPrivate.value()

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
            # map->robot, robot->world, world->people ==>> map->people
            people_position = self.map2robot * self.motion2robot.inverse() * people_position_tf
            self._publish_tf(self._map2pose(people_position), "people")
            self._publish_marker(self._map2pose(people_position))
            print people_position

    def run(self):
        rospy.loginfo("start")
        while not rospy.is_shutdown():
            self._publish_map()
            self._robot_pose()
            self._publish_perception()
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node("naoqi_navigation_bridge_node")
    instance = NaoqiNavigationBridge()
    instance.run()
    rospy.spin()
