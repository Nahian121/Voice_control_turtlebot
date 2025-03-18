import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr
import pyaudio


class speaker(Node):
    def __init__(self):
        super().__init__('speaker')
        self.pub_ =self.create_publisher(String,"command",20)
        self.recognize_= sr.Recognizer() #initializing recognizer class
        self.microphone_= sr.Microphone(sample_rate=16000,chunk_size=1024)
        self.timer_= self.create_timer(5,self.listen_commands)
    
    def listen_commands(self):
        with self.microphone_ as source:
            self.get_logger().info("Say where to move..")
            self.recognize_.adjust_for_ambient_noise(source, duration=2)
            try:
                audio= self.recognize_.listen(source,timeout=10)
                command=self.recognize_.recognize_google(audio,language="en-GB").lower()
                self.get_logger().info(f"Command: {command}")

                msg=String()
                msg.data=command
                self.pub_.publish(msg)
            except sr.UnknownValueError:
                self.get_logger().warn("Couldn't understand the input")
            except sr.RequestError:
                self.get_logger().warn("Couldn't request results")

def main(args=None):
    rclpy.init(args=args)
    node=speaker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown

if __name__=='__main__' :
    main()







