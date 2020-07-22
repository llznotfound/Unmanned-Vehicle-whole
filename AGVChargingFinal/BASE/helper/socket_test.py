# client socket
# 作为通信的客户端
import socket
import numpy as np
import matplotlib.pyplot as plt
class BaseTCP():
    def __init__(self):
        # socket parameter
        # 填写车载电脑IP地址
        self.ip = "192.168.8.11"
        # self.ip = "192.168.1.15"
        self.port = 8888
        self.ready = True  #车辆静止


    # create new socket connection
    def new_socket(self):
        socket_new = socket.socket()
        socket_new.connect((self.ip, self.port))
        return socket_new


    def getxy(self, data):
        string = data.split(";")
        x = string[0]
        x = eval(x)
        y = string[1]
        y = eval(y)
        return x, y


    def getxyzw(self, data):
        string = data.split(";")
        m = string[0]
        m = eval(m)
        n = string[1]
        n = eval(n)
        x = string[2]
        x = eval(x)
        y = string[3]
        y = eval(y)
        z = string[4]
        z = eval(z)
        w = string[5]
        w = eval(w)
        t = string[6]
        t = eval(t)
        return m, n, x, y, z, w, t



    def vision(self, x, y, labelo):
        plt.cla()
        plt.plot(x, y, "-b", label=labelo)
        plt.legend()
        plt.xlabel("x")
        plt.ylabel("y")
        plt.axis("equal")
        plt.grid(True)
        plt.show()


        
        
    def send_xy(self, start, goal):
        # 
        # ("start={},goal={}".format(start, goal))
        Socket = self.new_socket()
        Socket.send("start={},goal={}".format(start, goal).encode(encoding="utf-8"))
        # Socket.close()
        buffer = []
        while True:
            msg = Socket.recv(1024).decode(encoding="utf-8")
            buffer.append(msg)
            # print("buffer", buffer)
            data = ''.join(buffer)
            if msg[-2:] == 'OK':
                Socket.close()
                m, n, x, y, v, yaw, t = self.getxyzw(data)
                self.vision(m, n, "org")
                self.vision(x, y, "trajectory")
                self.vision(t, v, "speed")
                self.vision(t, yaw, "yaw")
                return m, n, x, y

    def questionMotion(self):
        Socket = self.new_socket()
        Socket.send("Stationary".encode(encoding="utf-8"))
        print("Stationary?")
        buffer = []
        while True:
            msg = Socket.recv(1024).decode(encoding="utf-8")
            buffer.append(msg)
            # print("buffer", buffer)
            data = ''.join(buffer)
            print(msg)
            if msg[-1:] == 'Y':
                self.ready = True
                Socket.close()
                break
            elif msg[-1:] == 'N':
                self.ready = False
                Socket.close()
                break

    def CheckState(self):
        return self.ready


    def pid_stop():
        Socket = self.new_socket()
        Socket.send("L".encode(encoding="utf-8"))
        print("Stop")
        Socket.close()

    def exit(self):
        Socket = self.new_socket()
        Socket.send("exit".encode(encoding="utf-8"))
        print("exit")
        Socket.close()

    def GO(self):
        Socket = self.new_socket()
        Socket.send("go".encode(encoding="utf-8"))
        print("go")
        Socket.close()


    def send(self, string1):
        Socket = self.new_socket()
        Socket.send(string1.encode(encoding="utf-8"))
        print(string1)
        Socket.close()


if __name__ == '__main__':
    
    base = BaseTCP()
    # base.exit()
    # base.questionMotion()
    print(base.send_xy((787,30, 0), (787, 1018,  0)))
    # base.GO()
    # base.exit()
    # base.send('C2')
    # pid_stop()
