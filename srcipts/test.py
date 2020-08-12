import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

def talker():
    rospy.init_node('talker', anonymous=True)
    pub = rospy.Publisher('/joint_states', JointState, queue_size=1)
    rate = rospy.Rate(10)
    States=JointState();

    a =0;
    while not rospy.is_shutdown():
        States.header.stamp = rospy.Time.now()
        States.name=['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        a+=0.001
        if(a>3.14):
            a=-3.14
        States.position=[a, -1.57, 0 ,0, 0, 0]
        States.velocity=[]
        States.effort=[]
        pub.publish(States)
        print States

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass