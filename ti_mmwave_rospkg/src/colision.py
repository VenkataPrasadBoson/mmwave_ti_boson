import rospy
from std_msgs.msg import Bool
from ti_mmwave_rospkg.msg import RadarScan
import time

   
def callback(data):
    
    
    if range1 > 4:
        collision = False
    elif range1 < 1.3:
        collision = False
    elif  1.3 < range1 < 4:
	collision = True
            
    
def start():
    global pub

    pub = rospy.Publisher('/ti_mmwave/obstacle', Bool, queue_size=100)
    rospy.init_node('radar_range')
    rospy.Subscriber("/ti_mmwave/radar_scan", RadarScan, callback)
    
    rospy.spin()

if __name__ == '__main__':
    start()
