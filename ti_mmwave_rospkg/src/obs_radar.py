'''boson robotics radar based obstacle detection based on the 140 degrees FOV
and works only with 2d point cloud cfg file '''

import ros_numpy
import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool


class RadarObjDetct():
    '''class to give the radar'''
    def __init__(self, x_min, x_max, y_min, y_max):
        """Returns:
            Initilization of variables
           Args:
             Get ros subscriber data
        """
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max

        rospy.Subscriber(
            "/ti_mmwave/radar_scan_pcl",
            PointCloud2,
            self.radar_callback)
        self.radar_pub = rospy.Publisher(
            'radar_obj_status', Bool, queue_size=10)

    def radar_callback(self, pc2_msg):
        '''this function is responsible for converting
        point cloud to x,y,z coordinates in meters
        and filter them according to our use case
        as our vehicle marks an square in front
        2 meters width and 4 meters height
        if any point found in that distance it
        passes out from filter gives output in Bool'''
        xyz_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pc2_msg)
        fil = []
        for i, (x, y, z) in enumerate(xyz_array):
            if x > self.x_min and self.y_min < y < self.y_max:
                fil.append(xyz_array[i])
        val = 0
        for x, y, z in fil:
            if self.x_min < x < self.x_max:
                val += 1
        bool_status = Bool()
        if val > 0:
            bool_status = True
        else:
            bool_status = False
        self.radar_pub.publish(bool_status)

    @staticmethod
    def run():
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node('rdar')
    x_min = rospy.get_param('radar/x_min', 1)
    x_max = rospy.get_param('radar/x_max', 4)
    y_min = rospy.get_param('radar/y_min', -1)
    y_max = rospy.get_param('radar/y_max', 1)

    radar_obj = RadarObjDetct(x_min, x_max, y_min, y_max)
    radar_obj.run()
