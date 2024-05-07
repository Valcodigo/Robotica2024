#!/usr/bin/env python3
import rclpy
from rclpy.node import Node 
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool
import matplotlib.pyplot as plt
import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
import time
from tkinter import messagebox, ttk, StringVar, Label
from tkinter.simpledialog import askstring
from threading import Thread

class turtle_bot_interface(Node):
    def __init__(self):
        super().__init__("turtle_bot_interface")
        self.get_logger().info("Nodo turtle_bot_interface activado")
        self.interfaz=tk.Tk()
        self.interfaz.geometry("1000x700")
        self.interfaz.configure(bg='gainsboro')
        self.interfaz.wm_title("Interfaz Root")
        boton1 = ttk.Button(text="Limpiar Grafica", command=self.limpiar)
        boton1.place(x=50, y=50)
        boton2 = ttk.Button(text="Guardar", command=self.guardar)
        boton2.place(x=50, y=100)
        boton3 = ttk.Button(text="Detener guardado", command=self.no_guardar)
        boton3.place(x=50, y=150)
        self.fig = Figure(figsize=(5, 5), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.x,self.y=[],[]
        self.linx,self.angz=0,0
        self.xpos = StringVar()
        self.ypos = StringVar()
        self.frame = tk.Frame(self.interfaz, width=500, height=500, bg='white',bd=5, relief='sunken')  
        self.frame.pack(side='right')  
        self.canvas = tk.Canvas(self.frame, width=500, height=500, bg='white') 
        self.canvas.pack()
        self.figure_canvas_agg = FigureCanvasTkAgg(self.fig, master=self.canvas)
        self.ax.set_xlabel("x")
        self.ax.set_ylabel("y")
        self.ax.set_xlim([-2.5,2.5])
        self.ax.set_ylim([-2.5,2.5])
        self.ax.set_title("Robot Position")
        self.turtlebot_position_=self.create_subscription(Twist,"/turtlebot_position",self.interface_callback,10)
        self.service_replay=self.create_service(SetBool,"/request_replay",self.pedir_archivo)
        self.figure_canvas_agg.get_tk_widget().pack(side='top', fill='both', expand=1)
        toolbar = NavigationToolbar2Tk(self.figure_canvas_agg, self.canvas)
        toolbar.update()
        self.pos=Thread(target=self.position)
        self.figure_canvas_agg.get_tk_widget().pack(side='top', fill='both', expand=1)
        self.update() 

    def update(self):
        rclpy.spin_once(self)
        self.interfaz.after(100, self.update) 

    def pedir_archivo(self, request, response):
        response.success = True
        msg=askstring('Buscar archivo', '¿Cual archivo de recorrido desea reproducir?: ')
        response.message = msg
        return response

    def interface_callback(self,msg: Twist):
        if self.pos.is_alive():
            None
        else:
            self.pos.start()
        self.x.append(msg.linear.x)
        self.y.append(msg.linear.y)
        self.ax.plot(self.x,self.y,color="b")
        self.figure_canvas_agg.draw()
        time.sleep(0.01)
    
    def limpiar(self):
        self.x,self.y=[],[]
        self.ax.clear()
        self.ax.set_xlabel("x")
        self.ax.set_ylabel("y")
        self.ax.set_xlim([-2.6,2.6])
        self.ax.set_ylim([-2.6,2.6])
        self.ax.set_title("Robot Position")

    def guardar(self):
        self.name = askstring('Guardar archivo', 'Ingrese el nombre del archivo: ')
        self.archivo=open(self.name+".txt","w")
        self.turtlebot_position_=self.create_subscription(Twist,"/turtlebot_cmdVel",self.teleop_callback,10)
        self.flag=True
        
    def no_guardar(self):
        self.flag=False

    def teleop_callback(self,msg: Twist):
        if self.vel.is_alive():
            None
        else:
            self.vel.start()
        if self.flag:
            self.archivo.write(str(msg.linear.x)+","+str(msg.angular.z)+"\n")
            self.linx=msg.linear.x
            self.angz=msg.angular.z
        else:
            self.archivo.close()
    
    def position(self):
        while True:
            if len(self.x)>0:
                self.xpos.set("Pocision X: "+str(round(self.x[-1],4)))
                self.ypos.set("Pocision Y: "+str(round(self.y[-1],4)))
            else:
                self.xpos.set("Pocision X: ")
                self.ypos.set("Pocision Y: ")
            self.etixpos = Label(self.interfaz, textvariable=self.xpos)
            self.etiypos = Label(self.interfaz, textvariable=self.ypos)
            self.etixpos.place(x=50, y=300)
            self.etiypos.place(x=50, y=340)
            time.sleep(0.5)

    

def main(args=None):
    rclpy.init(args=args)
    node=turtle_bot_interface()
    node.interfaz.mainloop()
    rclpy.shutdown()
