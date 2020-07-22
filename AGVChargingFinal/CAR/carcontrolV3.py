import socket_server
import reeds_shepp_path_planning as rspp
import pure_pursuit as pur
import matplotlib.pyplot as plt
import threading
import time
import numpy as np
from pid_control import v_angle
import parameter as par


class Car:
    class Carstate:
        def __init__(self,orgx = None, orgy = None, x = None, y = None, v = None, yaw = None, t = None, ready = False):
            self.x = x
            self.y = y
            self.v = v
            self.yaw = yaw
            self.t = t
            self.ready = ready #运动状态标志
            self.orgx = orgx
            self.orgy = orgy
            
    def __init__(self):
        self.target_speed = par.target_speed  # m/s
        self.T = par.TotalTime  # max simulation time
        self.dt = par.dt
        self.flag = False   #小车运动规划是否成功
        self.carstate = self.Carstate()
        plt.ion()   #将画图模式改为交互模式

    #细调节动作对应表格
    def State(self, string):
        print(string)
        if string == 'R2': 
            t3 = threading.Thread(target=self.R2, args=())
            t3.start()
        elif string== 'R1':
            t3 = threading.Thread(target=self.R1, args=())
            t3.start()
        elif string == 'L2': 
            t3 = threading.Thread(target=self.L2, args=())
            t3.start()
        elif string == 'L1':
            t3 = threading.Thread(target=self.L1, args=())
            t3.start()
        elif string == 'B4':
            t3 = threading.Thread(target=self.B4, args=())
            t3.start()
        elif string == 'B2':
            t3 = threading.Thread(target=self.B2, args=())
            t3.start()
        elif string == 'B1':
            t3 = threading.Thread(target=self.B1, args=())
            t3.start()
        elif string== 'Bs':
            t3 = threading.Thread(target=self.Bs, args=())
            t3.start()
        elif string == 'F':
            t3 = threading.Thread(target=self.F, args=())
            t3.start()
        elif string== 'G4':
            t3 = threading.Thread(target=self.G4, args=())
            t3.start()
        elif string== 'G3':
            t3 = threading.Thread(target=self.G3, args=())
            t3.start()
        elif string == 'G2':
            t3 = threading.Thread(target=self.G2, args=())
            t3.start()
        elif string == 'G1':
            t3 = threading.Thread(target=self.G1, args=())
            t3.start()
        elif string == 'C2':
            t3 = threading.Thread(target=self.C2, args=())
            t3.start()

    #细调节动作组
    def R2(self):  
        v_angle(300, 45)
        t = time.time()
        while time.time() - t < 1: pass
        v_angle(0, 0)
        time.sleep(1)
        v_angle(0, 0)
        time.sleep(1)
        self.carstate.ready = False
        
    def R1(self):  
        v_angle(300, 45)
        t = time.time()
        while time.time() - t < 0.7: pass
        v_angle(0, 0)
        time.sleep(1)
        v_angle(0, 0)
        time.sleep(1)
        self.carstate.ready = False

    def L2(self):  
        v_angle(300, -45)
        t = time.time()
        while time.time() - t < 1: pass
        v_angle(0, 0)
        time.sleep(1)
        self.carstate.ready = False

    def L1(self):  
        v_angle(300, -45)
        t = time.time()
        while time.time() - t < 0.7: pass
        v_angle(0, 0)
        time.sleep(1)
        self.carstate.ready = False

    def B4(self):  
        v_angle(-300, 0)
        t = time.time()
        while time.time() - t < 4: pass
        v_angle(0, 0)
        time.sleep(1)
        self.carstate.ready = False

    def B2(self):  
        v_angle(-300, 0)
        t = time.time()
        while time.time() - t < 2: pass
        v_angle(0, 0)
        time.sleep(1)
        self.carstate.ready = False

    def B1(self):  
        v_angle(-300, 0)
        t = time.time()
        while time.time() - t < 1: pass
        v_angle(0, 0)
        time.sleep(1)
        self.carstate.ready = False

    def Bs(self):  
        v_angle(-300, 0)
        t = time.time()
        while time.time() - t < 0.5: pass
        v_angle(0, 0)
        time.sleep(1)
        self.carstate.ready = False

    def F(self):  
        v_angle(300, 0)
        t = time.time()
        while time.time() - t < 0.7: pass
        v_angle(0, 0)
        time.sleep(1)
        v_angle(0, 0)
        v_angle(0, 0)
        self.carstate.ready = False

    def G4(self):  
        v_angle(300, -45)
        t = time.time()
        while time.time() - t < 4: pass
        v_angle(300, 45)
        t = time.time()
        while time.time() - t < 4: pass
        v_angle(-300, 0)
        t = time.time()
        while time.time() - t < 12: pass
        v_angle(0, 0)
        time.sleep(1)
        v_angle(0, 0)
        time.sleep(1)
        self.carstate.ready = False

    def G3(self):  
        v_angle(300, -45)
        t = time.time()
        while time.time() - t < 3: pass
        v_angle(300, 45)
        t = time.time()
        while time.time() - t < 3: pass
        v_angle(-300, 0)
        t = time.time()
        while time.time() - t < 9: pass
        v_angle(0, 0)
        time.sleep(1)
        v_angle(0, 0)
        time.sleep(1)
        self.carstate.ready = False

    def G2(self):  
        v_angle(300, -45)
        t = time.time()
        while time.time() - t < 2: pass
        v_angle(300, 45)
        t = time.time()
        while time.time() - t < 2: pass
        v_angle(-300, 0)
        t = time.time()
        while time.time() - t < 4: pass
        v_angle(0, 0)
        time.sleep(1)
        v_angle(0, 0)
        time.sleep(1)
        v_angle(0, 0)
        time.sleep(1)
        self.carstate.ready = False

    def C2(self):  
        v_angle(300, 45)
        t = time.time()
        while time.time() - t < 2: pass
        v_angle(300, -45)
        t = time.time()
        while time.time() - t < 2: pass
        v_angle(-300, 0)
        t = time.time()
        while time.time() - t < 4: pass
        v_angle(0, 0)
        time.sleep(1)
        v_angle(0, 0)
        time.sleep(1)
        v_angle(0, 0)
        time.sleep(1)
        self.carstate.ready = False

    def G1(self):  
        v_angle(300, -45)
        t = time.time()
        while time.time() - t < 1: pass
        v_angle(300, 45)
        t = time.time()
        while time.time() - t < 1: pass
        v_angle(-300, 0)
        t = time.time()
        while time.time() - t < 2: pass
        v_angle(0, 0)
        time.sleep(1)
        v_angle(0, 0)
        time.sleep(1)
        self.carstate.ready = False

    # 计算每个节点的速度
    # 输入：reeds_shepp规划后返回的x,y,yaw集合
    # 输出：每个节点到下个节点的运动状态states集合
    def pure_purs(self, x, y, yaw):
	    cx = x
	    cy = y
	    cyaw = yaw
	    target_speed = self.target_speed # [m/s]

	    T = self.T  # max simulation time


	    goal = [cx[-1], cy[-1], cyaw[-1]]

	    cx, cy, cyaw = pur.extend_path(cx, cy, cyaw)

	    speed_profile = pur.calc_speed_profile(
	        cx, cy, cyaw, self.target_speed)

	    t, x, y, yaw, v, a, d, find_goal = pur.closed_loop_prediction(
	        cx[0], cy[0], cyaw[0], cx, cy, cyaw, speed_profile, goal)

	    states = pur.States(x, y, d, v, t)
	    return states

    # 规划两点之间的路径
    # 输入：基站端下发start（x, y, yaw）,end(x, y, yaw)
    # 输出：路径上每个节点的信息point（x,y,yaw），flag 路线规划是否成功
    def reeds_shepp(self, x, y):
        start_x, start_y, start_yaw = x
        start_yaw = np.deg2rad(start_yaw)
        end_x, end_y, end_yaw = y
        end_yaw = np.deg2rad(end_yaw)
        curvature = 1/par.maxcurve
        step_size = par.ReedsShepp_step
        px, py, pyaw, mode, clen = rspp.reeds_shepp_path_planning(
            float(start_x), float(start_y), start_yaw, float(end_x), float(end_y), end_yaw, curvature, step_size)
        if px is not None:
            print("reeds shepp ok")
            flag = True
        else:
            flag = False
        return px, py, pyaw, flag

    # 可视化计算后的数据
    # 输入： states 运动状态集合
    def vision(self, states):
        plt.clf()
        #plt.cla()
        plt.subplot(311)
        plt.plot(states.x, states.y, "-b")
        plt.legend()
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.axis("equal")
        plt.grid(True)

        plt.subplot(312)
        plt.plot(states.t, [iv for iv in states.v], "-r")
        plt.xlabel("Time[s]")
        plt.ylabel("Speed[m/s]")
        plt.grid(True)

        plt.subplot(313)
        plt.plot(states.t, states.yaw, "-r")
        plt.xlabel("Time[s]")
        plt.ylabel("yaw")
        plt.grid(True)
        plt.pause(0.1)
    # 一直运行，控制小车运动
    def carMotion(self):
        while True:
            #轨迹规划结束后将会有一组时间,如果有这组时间，那么开始运动
            if self.carstate.t is not None:
                print("in!!")
                ####################################################
                ##################添加动作代码#######################
                ####################################################
                for i in range(len(self.carstate.t)-1):
                    v_temp = self.carstate.v[i]*par.pid_speed/par.target_speed
                    yaw_temp = -1*self.carstate.yaw[i]
                    print(v_temp)
                    print(self.dt)
                    t = time.time()
                    v_angle(v_temp, yaw_temp)#v是以图像像素为单位的
                    while time.time() - t < self.dt: pass
                    # time.sleep(self.dt)
                    print(time.time() - t)
                self.carstate.t = None #clear all the states
                v_angle(0, 0)#回归静止，前轮摆正
                time.sleep(1)
                v_angle(0, 0)#回归静止，前轮摆正
                time.sleep(1)
                #运动结束后，flag转为false
                self.carstate.ready = False
            #else:
            #   v_angle(0, 0)#回归静止，前轮摆正
                
def main():
    socket = socket_server.socket_Server()
    car = Car()
    t1 = threading.Thread(target=socket.server_open, args=())
    t1.setDaemon(True)
    t1.start()
    # car control command is always running
    t2 = threading.Thread(target=car.carMotion, args=())
    t2.start()
    while True:
        Takeflag, rec = socket.server_takeData()
        if Takeflag:
            string = rec.split("=")
            if string[0] == 'start':
                start_x = string[1].split(",goal")[0]
                start_x = eval(start_x)
                end_y = string[2]
                end_y = eval(end_y)
                print(start_x, end_y)
                x, y, yaw, plannFlag = car.reeds_shepp(start_x, end_y)
                print(yaw)
                if plannFlag:
                    state = car.pure_purs(x, y, yaw)
                    car.vision(state)
                    print("this is t:", state.t)
                    socket.server_send("{};{};{};{};{};{};{};OK".format(x, y, state.x, state.y, state.v, state.yaw, state.t))
                else:
                	socket.server_send("NO")
            elif string[0] == 'Stationary':
             	if car.carstate.ready: socket.server_send("N")
             	else: socket.server_send("Y")
            elif string[0] == 'go':
            	car.carstate = car.Carstate(x, y, state.x, state.y, state.v, state.yaw, state.t, True)
            elif string[0] == 's':
                car.carstate.ready = False
                car.carstate.t = None #clear all the states
                v_angle(0, 0)#回归静止，前轮摆正
                time.sleep(1)
                v_angle(0, 0)#回归静止，前轮摆正
                time.sleep(1)
            elif string[0] == 'R2' or string[0] == 'R1' or string[0] == 'L1'or string[0] == 'L2'or string[0] == 'B4'or string[0] == 'B2'or string[0] == 'B1'or string[0] == 'Bs'or string[0] == 'F'or string[0] == 'G4'or string[0] == 'G3'or string[0] == 'G2' or string[0] == 'C2' or string[0] == 'G1':
                 car.carstate.ready = True  #细调节部分
                 print(string[0])
                 car.State(string[0])       #调用细调节动作
            elif string[0] == 'exit':
            	print("this is exit")
            	break
            else:
            	print("unknown cmd, continue")
    
    socket.server_close()

if __name__ == "__main__":
    main()


