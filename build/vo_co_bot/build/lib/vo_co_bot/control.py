import rclpy
import rclpy.duration
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import  Int32
from geometry_msgs.msg import Twist


class controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.sub_ =self.create_subscription(String,"command",self.command_callback,20)
        self.pub_ = self.create_publisher(Twist,'/cmd_vel',5)
        self.energy_pub_= self.create_publisher(Int32,'energy_remaining',20)
        self.energy=100
        self.get_logger().info("Cotrol Working")
    
    def command_callback(self,msg):
        command=msg.data.lower()
        twist=Twist()

        if "forward" in command:
            duration= 2
            if "forward" in command and len(command.split())>1:
                try:
                    duration=int(command.split()[1].replace('s',''))
                except ValueError:
                    pass
            twist.linear.x=0.4
            self.move_tb(twist,duration)

        elif "left" in command:
            duration= 2
            if "left" in command and len(command.split())>1:
                try:
                    duration=int(command.split()[1].replace('s',''))
                except ValueError:
                    pass
            twist.angular.z=0.2
            self.move_tb(twist,duration)

        elif "right" in command:
            duration= 2
            if "right" in command and len(command.split())>1:
                try:
                    duration=int(command.split()[1].replace('s',''))
                except ValueError:
                    pass
            twist.angular.z=-0.2
            self.move_tb(twist,duration)
        else:
            self.get_logger().info(f"unknown command: {command}")
        
        self.energy=self.energy-5
        energy_msg= Int32()
        energy_msg.data=self.energy
        self.energy_pub_.publish(energy_msg)





    def move_tb(self,twist,duration):
        self.pub_.publish(twist)
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=duration))
        self.pub_.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node=controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown

if __name__=='__main__' :
    main()


