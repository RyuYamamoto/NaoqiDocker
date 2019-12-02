import qi
import rospy
import tf2_ros
import tf_conversions
import geometry_msgs
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
        self.rate = rospy.Rate(2)
        self.map = rospy.Publisher("/naoqi_exploration_map", OccupancyGrid, queue_size=None)
	self.connect("10.40.16.85")

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
            self.motion = self.session.service("ALMotion")
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
            map2robot = almath.Pose2D(robot_in_map[0][0], robot_in_map[0][1], robot_in_map[0][2])
            self._publish_tf(self._map2pose(map2robot), "robot_pose")
        except Exception as err:
            rospy.logerr(str(err))

    def run(self):
        rospy.loginfo("start")
        while not rospy.is_shutdown():
            self._publish_map()
            self._robot_pose()
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node("naoqi_navigation_bridge_node")
    instance = NaoqiNavigationBridge()
    instance.run()
    rospy.spin()
