#!/usr/bin/env python3
import rclpy
from rclpy.node import Node 
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool

class turtle_bot_player(Node):
    def __init__(self):
        super().__init__("turtle_bot_player")
        self.nombre=""
        self.cli = self.create_client(SetBool, '/request_replay')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('El servicio no está activo, reintentando ...')
        self.req = SetBool.Request()
        self.request_service()
        self.get_logger().info("Nodo turtle_bot_player activado")
        self.turtlebot_cmdVel_player=self.create_publisher(Twist, "/turtlebot_cmdVel", 10)
        self.archivo=open(self.nombre+".txt","r")
        self.mensaje()
        self.l=0
        self.longitud=len(self.enviar)/2
        self.timer_=self.create_timer(0.5, self.send_velocity_command_player)

    def send_velocity_command_player(self):
        msg=Twist()
        if self.l==self.longitud:
            rclpy.shutdown()
        else:
            msg.linear.x=self.enviar[2*self.l]
            msg.angular.z=self.enviar[2*self.l+1]
            self.turtlebot_cmdVel_player.publish(msg)
            self.l+=1
    
    def request_service(self):
        self.req.data = True
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        self.nombre=future.result().message

    def mensaje(self):
        flag=True
        self.enviar=[]
        while flag:
            linea=self.archivo.readline()
            if linea=="":
                flag=False
            else:
                linea=linea.replace("\n","")
                linea=linea.split(",")
                self.enviar.append(float(linea[0]))
                self.enviar.append(float(linea[1]))
            


def main(args=None):
    rclpy.init(args=args)
    node=turtle_bot_player()
    rclpy.spin(node)
    rclpy.shutdown()