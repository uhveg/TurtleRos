import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty

def mock_topics():
    rospy.init_node('mock_turtlebot', anonymous=True)
    
    # Publishers for the topics
    cmd_vel_pub = rospy.Publisher('/bot1/cmd_vel', Twist, queue_size=10)
    odom_pub = rospy.Publisher('/bot1/odom', Odometry, queue_size=10)
    motor_power_pub = rospy.Publisher('/bot1/motor_power', Bool, queue_size=10)
    sound_pub = rospy.Publisher('/bot1/sound', String, queue_size=10)

    # Service for reset
    rospy.Service('/bot1/reset', Empty, lambda req: None)

    rate = rospy.Rate(10)  # 10Hz

    while not rospy.is_shutdown():
        # Simulate a command to move the robot
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.1  # Moving forward
        cmd_vel_msg.angular.z = 0.2  # Turning in place
        cmd_vel_pub.publish(cmd_vel_msg)
        
        # Simulate odometry data
        odom_msg = Odometry()
        odom_msg.pose.pose.position.x = 1.0  # Example position
        odom_msg.pose.pose.position.y = 2.0
        odom_pub.publish(odom_msg)

        # Simulate motor power status
        motor_power_msg = Bool()
        motor_power_msg.data = True  # Motors are on
        motor_power_pub.publish(motor_power_msg)

        # Simulate sound data (e.g., "beep" or any command)
        sound_msg = String()
        sound_msg.data = "beep"
        sound_pub.publish(sound_msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        mock_topics()
    except rospy.ROSInterruptException:
        pass
