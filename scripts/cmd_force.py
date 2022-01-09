import rospy
from std_msgs.msg import Float32MultiArray, Bool
import time

class cmd_force():
    def __init__(self) -> None:
        rospy.init_node('force_pub')
        # define variables
        self.logger_force = Float32MultiArray()
        self.force_drone1 = Float32MultiArray()
        self.force_drone2 = Float32MultiArray()
        self.force_drone3 = Float32MultiArray()

        # Subscriber
        rospy.Subscriber('/loggerAll/force', Float32MultiArray, self.sub_force0)
        rospy.Subscriber('/drone1/force', Float32MultiArray, self.sub_force1)
        rospy.Subscriber('/drone2/force', Float32MultiArray, self.sub_force2)
        rospy.Subscriber('/drone3/force', Float32MultiArray, self.sub_force3)
        rospy.Subscriber('/cmd_running', Bool, self.sub_run)

        # Publisher
        self.force_publisher0 = rospy.Publisher('/loggerAll/testforce', Float32MultiArray, queue_size=1)
        self.force_publisher4 = rospy.Publisher('/drone1/testforce', Float32MultiArray, queue_size=1)
        self.force_publisher5 = rospy.Publisher('/drone2/testforce', Float32MultiArray, queue_size=1)
        self.force_publisher6 = rospy.Publisher('/drone3/testforce', Float32MultiArray, queue_size=1)

        ## init variables
        self.running = False
        self.logger_force.data = [0 for i in range(5)]
        self.force_drone1.data = [0 for i in range(3)]
        self.force_drone2.data = [0 for i in range(3)]
        self.force_drone3.data = [0 for i in range(3)]


        rospy.loginfo('creating a node to publish force by 1000hz')

    def pub(self):
        rate = rospy.Rate(1000)
        while not rospy.is_shutdown():
            if not self.running:
                print("hahah")
                self.logger_force.data = [0 for i in range(5)]
                self.force_drone1.data = [0 for i in range(3)]
                self.force_drone2.data = [0 for i in range(3)]
                self.force_drone3.data = [0 for i in range(3)]
                # rate.sleep()
                # continue
            start = time.time()
            # print('force: {}\n'.format(self.force.data))
            self.force_publisher0.publish(self.logger_force)
            self.force_publisher4.publish(self.force_drone1)
            self.force_publisher5.publish(self.force_drone2)
            self.force_publisher6.publish(self.force_drone3) # 使用3 uavs时使用
            # rate.sleep()
            end = time.time()
            if(0<time.time()-start < 1e-3):
                time.sleep(abs(9e-4-(time.time()-start)))
            
            print('time: ', time.time() - start)

    def sub_force0(self,data):
        self.logger_force = data
    def sub_force1(self,data):
        self.force_drone1 = data
    def sub_force2(self,data):
        self.force_drone2 = data
    def sub_force3(self,data):
        self.force_drone3 = data

    def sub_run(self, data):
        self.running = data.data

if __name__ == '__main__':
    cmd = cmd_force()
    cmd.pub()
