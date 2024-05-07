#!/usr/bin/env python3
import rclpy
import termios, tty, sys, select
from rclpy.node import Node 
from geometry_msgs.msg import Twist
import threading

class turtle_bot_teleop(Node):
    def __init__(self):
        super().__init__("turtle_bot_teleop")
        self.get_logger().info("Nodo turtle_bot_teleop activado")
        self.c=""
        self.xlinear=float(input("Seleccione la velocidad linear: "))
        self.zangular=float(input("Seleccione la velocidad angular: "))
        self.turtlebot_cmdVel_=self.create_publisher(Twist, "/turtlebot_cmdVel", 10)
        self.timer_=self.create_timer(0.5, self.send_velocity_command)
        print("Usa las teclas WASD para mover")  # Mensaje para el usuario
        self.key_thread = threading.Thread(target=self.get_key)
        self.key_thread.start()

    def send_velocity_command(self):
        msg=Twist()
        if self.c == 'w':
            msg.linear.x=self.xlinear
        elif self.c == 's':
            msg.linear.x=self.xlinear*-1
        elif self.c == 'a':
            msg.angular.z=self.zangular
        elif self.c == 'd':
            msg.angular.z=self.zangular*-1
        else:  # Cuando no hay teclas presionadas
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        self.turtlebot_cmdVel_.publish(msg)

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            while True:
                rlist, _, _ = select.select([sys.stdin], [], [], 0.1) 
                if rlist:
                    self.c = sys.stdin.read(1)
                else:
                    self.c = ''
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

def main(args=None):
    rclpy.init(args=args)
    node=turtle_bot_teleop()
    rclpy.spin(node)
    rclpy.shutdown()
