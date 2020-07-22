import serial
import time


class Frame:
    def __init__(self):
        self.start = hex(0x5B).split("x")[1].upper()
        self.ctl_flag = hex(0xEB).split("x")[1].upper()
        self.cnt = '01'
        self.is_ctl = None
        self.speed_h = None
        self.speed_l = None
        self.angle = None
        self.end = hex(0x5D).split("x")[1].upper()
        self.auto_flag = True


class Serial:
    def __init__(self, port, bps, time):
        self.port = port
        self.bps = bps
        self.time = time
        try:
            # 打开串口，并得到串口对象
            self.ser = serial.Serial(self.port, self.bps, timeout=self.time)
            # 判断是否打开成功
            if self.ser.is_open:
                print("串口打开成功")
        except Exception as e:
            print("---串口打开异常---：", e)

    def writeFrame(self, Frame1):
        print("设备名称:", self.ser.name)
        data = bytes.fromhex('{}{}{}{}{}{}{}{}'.format(Frame1.start, Frame1.ctl_flag, Frame1.cnt, Frame1.is_ctl,
                                                       Frame1.speed_h, Frame1.speed_l, Frame1.angle, Frame1.end))
        self.ser.write(data)


#  给小车控制板发送命令
def send_cmd(frame1):
    cnt = 0
    frame = frame1
    ser = Serial("COM15", 9600, 0.5)
    while frame.auto_flag:
        cnt = cnt + 1
        if cnt == 0xff:
            cnt = 0
        else:
            if cnt < 16:
                frame.cnt = '0' + hex(cnt).split("x")[1].upper()
                ser.writeFrame(frame)
                time.sleep(0.3)
            else:
                frame.cnt = hex(cnt).split("x")[1].upper()
                ser.writeFrame(frame)
                time.sleep(0.3)
    ser.writeFrame(frame)


#  速度和角度的映射
# def v_angle(v, angle):
#     frame = Frame()
#     frame.auto_flag = False
#     frame.is_ctl = '0' + hex(1).split("x")[1]
#     frame.angle = int(127 + int(angle * 127 / 45))
#     if frame.angle < 16:
#         frame.angle = '0' + hex(frame.angle).split("x")[1].upper()
#     else:
#         frame.angle = hex(frame.angle).split("x")[1].upper()
#     speed = v
#     frame.speed_h = int((speed + 500) / 255)
#     frame.speed_l = int((speed + 500) - frame.speed_h * 255)
#     if frame.speed_h < 16:
#         frame.speed_h = '0' + hex(frame.speed_h).split("x")[1].upper()
#     else:
#         frame.speed_h = hex(frame.speed_h).split("x")[1].upper()
#     if frame.speed_l < 16:
#         frame.speed_l = '0' + hex(frame.speed_l).split("x")[1].upper()
#     else:
#         frame.speed_l = hex(frame.speed_l).split("x")[1].upper()
#     print("angle:{},sh:{},sl{}".format(frame.angle, frame.speed_h, frame.speed_l))
#     send_cmd(frame)

def v_angle(v, angle):
    print(v, angle)
    
if __name__ == "__main__":
	v_angle(300, -45)
	t = time.time()
	while time.time() - t < 0.7: pass
	v_angle(0, 0)
	time.sleep(1)
	v_angle(0, 0)
	time.sleep(1)
    # # v_angle(-350, 0)
    # # time.sleep(3)
  # v_angle(0, 0)


