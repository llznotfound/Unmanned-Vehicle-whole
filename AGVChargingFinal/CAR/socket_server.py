import socket


class socket_Server:

    def __init__(self):
        self.__host = "192.168.1.17"#填写本机IP地址
        
        # self.__host = "192.168.1.15"
        self.__port = 8888
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # 创建 socket 对象
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # self.__host, _ = self.s.getsockname()
        # self.__port = 8888
        self.s.bind((self.__host, self.__port))  # 绑定端口
        self.s.listen(5)  # 等待客户端连接
        self.conn = None
        self.rec = None
        self.closeflag = False

    def server_open(self):
        while True:
            if self.closeflag: break
            print("等待连接....")
            self.conn, addr = self.s.accept()  # 等待链接,多个链接的时候就会出现问题,其实返回了两个值
            # ip, _ = self.s.getpeername()
            # print("来自:{}连接".format(ip))
            ip, _ = self.conn.getpeername()
            print("来自:{}连接".format(ip))
            try:
                # print("conn：{}".format(self.conn))
                if self.conn is not None:
                    data = self.conn.recv(1024)  # 接收数据
                    print('recive:', data.decode(encoding="utf-8"))  # 打印接收到的数据
                    self.rec = data.decode(encoding="utf-8")
                # return None
            except ConnectionResetError as e:
                print('关闭了正在占线的链接！')
        self.s.close()


    def server_takeData(self):
        if self.rec is not None:
            flag = True
            data = self.rec
            self.rec = None
        else:
            flag = False
            data = None
        return flag, data


    def server_send(self, string):
        try:
            self.conn.send(string.encode(encoding="utf-8"))
            print("send {}".format(string))
        except ConnectionResetError as e:
            print('关闭了正在占线的链接！')

    def server_close(self):
        self.closeflag = True
        print("server close!!!")
