import pymongo
import os
import time
from time import sleep

def get_dat_filename():     #获取存储电压数据的文件名
    path = r"C:\Users\Administrator\Desktop\sscomsjedjwa\\"
    dirs = os.listdir(path)
    dirs.sort(key=lambda fn:os.path.getmtime(path+'\\'+fn))
    filename = os.path.join(path, dirs[-1])
    print(filename)
    return filename

def mongodb_ini():  # 数据库初始化函数
    mongo_client = pymongo.MongoClient('localhost', 27017)  # 数据库连接实例化
    db = mongo_client.ugv  # 连接到数据库ugv
    ugv_battery_set = db.ugv_battery # 实例化操作并连接到集合ugv_battery
    print("database ini success")
    return ugv_battery_set


def get_voltage_data(path): #读取文件内容并截取最新的电压数据
    #path = r"C:\Users\Administrator\Desktop\sscomsjedjwa\ReceivedTofile-COM1-2020_1_2_10-31-14.DAT"
    #while True
    file = open(path,'r')
    data = str(file.readlines())
    voltage = data[len(data)-7:len(data)-2]
    #print(voltage)
    file.close()
    return voltage

    
def insert_battery_data(path):
    while True:
        sleep(0.5)
        battery_data = get_voltage_data(path)
        battery = float(battery_data)
        #print("_____ %s",battery)
        if(len(battery_data)==5):        #对读取电压进行判断，选择合适的区间转化
            if(battery>=26.10):
                electricity=99
            if(battery>=25.60 and battery_data<=26.04):
                electricity=(battery*11.36)-198
            if(battery>=25.41 and battery<=25.57):
                electricity=(battery*43.75)-1026.68
            if(battery>=25.20 and battery<=25.30):
                electricity=(battery*100)-2450
            if(battery>=24.60 and battery<=25.09):
                electricity=(battery*40.8)-955
            if(battery>=24.40 and battery<=24.55):
                electricity=(battery*20)-443
            if(battery>=24.31 and battery<=24.37):
                electricity=(battery*100)-2398
            if(battery>=23.98 and battery<=24.26):
                electricity=(battery*35.7)-836
            if(battery>=23.80 and battery<=23.94):
                electricity=(battery*57.14)-1347
            if(battery>=23.4 and battery<=23.78):
                electricity=(battery*23.68)-553.21
            print("%s   %s  %s" % (battery_data,electricity,time.ctime()))
            #print("%s   %s" % (battery_data,time.ctime()))
            ugv_battery_set.insert_one({"time": int(time.time()), "battery": battery_data})

        else:
            print("read error")
                
        
        

if __name__ == '__main__':
    path = get_dat_filename()
    #get_voltage_data(path)
    ugv_battery_set = mongodb_ini()
    insert_battery_data(path)
    
    




