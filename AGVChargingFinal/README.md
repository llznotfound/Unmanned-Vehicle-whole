# 通信协议帧格式
1. socket client发送小车位置：start={},goal={}
2. socket server发送规划路线：{x};{y};OK，其中‘OK’为结束标志符。若路线规划不成功，发送‘NO’
3. socket client询问小车是否静止：发送Stationary
4. socket server回复：Y，小车在静止；N，小车在运动
5. socket client发送exit，socket server退出，通信结束

# 文件构成
* BASE含有所有基站侧代码
* CAR含有所有车辆侧代码
