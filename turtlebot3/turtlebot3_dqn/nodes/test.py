 #! /usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan

pub = rospy.Publisher('/revised_scan', LaserScan, queue_size = 10)
scann = LaserScan()

def callback(scan):
        scan_ranger = []
        scan_ranger.append(scan.ranges[0])
        scan_ranger.append(scan.ranges[1])
        scan_ranger.append(scan.ranges[2])
        scan_ranger.append(scan.ranges[3])
        scan_ranger.append(scan.ranges[4])
        scan_ranger.append(scan.ranges[5])
        scan_ranger.append(scan.ranges[6])
        scan_ranger.append(scan.ranges[359])
        scan_ranger.append(scan.ranges[358])
        scan_ranger.append(scan.ranges[357])
        scan_ranger.append(scan.ranges[356])
        scan_ranger.append(scan.ranges[355])
        scan_ranger.append(scan.ranges[354])
        scan_ranger.append(scan.ranges[353])
        # scan_ranger.append(max(scan.ranges[128], scan.ranges[129], scan.ranges[130]))
        # scan_ranger.append(max(scan.ranges[148], scan.ranges[149], scan.ranges[150]))
        # scan_ranger.append(max(scan.ranges[158], scan.ranges[159], scan.ranges[160]))
        # scan_ranger.append(max(scan.ranges[168], scan.ranges[169], scan.ranges[170]))
        # scan_ranger.append(max(scan.ranges[178], scan.ranges[179], scan.ranges[180]))

        scann = LaserScan()
        current_time = rospy.Time.now()
        scann.header.stamp = current_time
        scann.header.frame_id = 'rplidar_base_scan'
        scann.angle_min = 0.0
        scann.angle_max = 6.28319
        scann.angle_increment = 0.017501922324299812
        scann.time_increment = 0.0
        scann.range_min = 0.11999999731779099
        scann.range_max = 3.5
        scann.ranges = scan_ranger
        scann.intensities = scan.intensities[0:13]
        pub.publish(scann)


def listener():
    rospy.init_node('revised_scan', anonymous=True)
    sub = rospy.Subscriber('/scan', LaserScan, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()