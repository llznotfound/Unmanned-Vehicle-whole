# AGVCharging
1. 主函数是testV9和adjust。粗调节部分用testV9，细调节部分用adjust。他们有相同的车辆定位部分，但是动作控制代码有所不同。（为了准确起见，细调节部分终点坐标设为图像坐标）
1. 和相机相关的（包括校正和读取视频流）存在camera_cal/下
2. 所有辅助函数（包括utils.py,socket_test.py,helpers.py）都在helper/下
3. Imagehandler包含二维码识别预处理部分，**注意调节31， 32行的low、high值，以及53行的开闭操作参数，具体原理请查阅HSV颜色空间以及图像腐蚀膨胀内容**
4. FindingObject是模版匹配代码，包含了SIFT和ORB两种方法
6. 1.png,2.png,3.png,4.png均为模版图片，所有\*.mp4均为测试用视频
7. 在一个新场地需要完成下面几件事情：
	1. 视角变换参数确定（testV9.py, adjust.py中的modify函数，**注意，modify函数现阶段不是公用的，因此testV9.py, adjust.py中均需要修改**）
	2. 世界坐标与像素坐标对应（utils.py中的Realworld2Camera和camer2Realworld）
	3. 修改socket协议地址（在sockettest.py下第10行）。
	4. 确定摄像头矫正参数（用camera\_cal/helper调用camera\_cal.p, camera\_cal.p由camera\_calibration.py确定。具体过程见[相机标定](https://blog.csdn.net/weixin_41695564/article/details/80422329)）
8. 常见问题Q&A：
	* **无法定位车辆怎么办？** 
		* 如果没有ROI，更新FindingObject采用的图片。
		* 如果ROI中没有定位点，在Imagehandler.py中__convertImagetoBlackWhite模块修改low，up数值。(建议修改代码，把这两个参数作为函数的输入) 
		* 如果还无法检测到定位点，可以用海康威视的4200软件修改摄像头的曝光、对比度等参数。
	* **一般操作顺序？** 
		* 一般操作顺序testV9，粗调节部分。当看到车辆与目标点距离较小的时候，调用细调节adjust。
)

#文件结构
```
│-testV9.py, adjust.py 
	|-ROI中确定二维码朝向代码(AffineTransformation, FindingOrientationOfContours, Imagehandler, PatternFinding)
	|-寻找ROI代码(FindingObject)
	|-helper
		|-socket_test.py
		|-utils.py
	|-camera_cal
		|-helpers.py
		|-ipcamCapture.py
```

