#!/usr/bin/env python3
import rclpy
import serial
from rclpy.node import Node
from geometry_msgs.msg import Twist
from threading import Lock
class PoseSubscriberNode(Node):    

    def __init__(self):        
        super().__init__("rasp_subscriber")   
        self.planox = 0.0
        self.planoy = 0.0
        self.pose_publisher = self.create_publisher(Twist, "/turtlebot_position", 10)     
        self.pose_subscriber=self.create_subscription(Twist, "/turtlebot_cmdVel", self.pose_callback, 10 )        
        self.ser = serial.Serial('/dev/ttyACM0', 57600)        
        self.mutex = Lock()        
        self.debug_serial_cmds = True       
        # respuesta = self.ser.write(b'e')       
        # print(respuesta)            

    def pose_callback(self, msg:Twist):        
        cmd = Twist()
        self.get_logger().info("("+str(msg.linear.x)+","+str(msg.angular.z)+")")
        
        if msg.linear.x > 0:
            self.planoy += 3
            self.send_command(f"o {int(msg.linear.x)} {int(msg.angular.z)}") 
        elif msg.linear.x < 0:
            self.planoy -= 3
            self.send_command(f"o {int(msg.linear.x)} {int(msg.angular.z)}") 

        if msg.angular.z > 0:
            self.planox -= 3
            self.send_command(f"o 0 {int(msg.angular.z)}") 
        elif msg.angular.z < 0:
            self.planox += 3
            self.send_command(f"o {int(msg.linear.x)} 0") 

        # Publica continuamente
        self.publish_plan()
        
    def publish_plan(self):
        cmd = Twist()
        cmd.linear.x = self.planox
        cmd.linear.y = self.planoy
        self.pose_publisher.publish(cmd) 

    def send_command(self, cmd_string):
        
        self.mutex.acquire()
        try:
            cmd_string += "\r"
            self.ser.write(cmd_string.encode("utf-8"))
            if (self.debug_serial_cmds):
                print("Sent: " + cmd_string)

            ## Adapted from original
            c = ''
            value = ''
            while c != '\r':
                c = self.ser.read(1).decode("utf-8")
                if (c == ''):
                    print("Error: Serial timeout on command: " + cmd_string)
                    return ''
                value += c

            value = value.strip('\r')

            if (self.debug_serial_cmds):
                print("Received: " + value)
            return value
        finally:
            self.mutex.release()     

          
def main(args=None):    
    rclpy.init(args=args)    
    node = PoseSubscriberNode()    
    rclpy.spin(node)    
    node.ser.close()    
    rclpy.shutdown()

if __name__ == '__main__':    
    main()