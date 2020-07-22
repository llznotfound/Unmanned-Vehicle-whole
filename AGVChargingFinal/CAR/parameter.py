#-------------------for car control-----------------------------
target_speed = 80/3	#车辆速度（cm/s）
TotalTime = 20		#一次运动时间长度（s）
dt = 0.3			#车辆运动控制间隔（s）
pid_speed = 300		#target speed 对应pid速度控制字

#---------------------for Pure Pursuit --------------------
Lf = 80/3*dt		#预瞄距离ld
goal_dis = 0.5		#车辆距离目标点距离为0.5时认为车辆已经到终点附近
maxcurve = 160		#车辆最大转弯半径
ReedsShepp_step = 10	#ReedsShepp曲线采样间隔

#---------------------for unicycle model--------------------
L = 55				#存在疑问，估计是车辆后轮距离
maxspeed = target_speed


