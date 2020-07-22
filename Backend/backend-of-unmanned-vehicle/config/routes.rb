Rails.application.routes.draw do
  # For details on the DSL available within this file, see http://guides.rubyonrails.org/routing.html

  get '/fog/update/:id', to: 'fog#update'
  get '/fog/alives'

  get '/barrage/update', to: 'geo#update'  #演示手机更新数据库弹幕信息
  get '/barrage/get', to: 'geo#get'  #浏览器获取更新后的弹幕接口

  get '/geo/information'  #十月份物联网项目后端接口，提供500个节点的剩余存储空间，计算能力与信道状态三个参数
  get '/geo/storm'  #十月份物联网项目后端接口，提供4个storm节点的进程工作情况，存储能力，计算负荷，各类应用通信占用带宽四个参数

  get '/ocean/all' #无人船全部数据获取接口
  get '/ocean/information' #无人船数据获取接口
  get '/ocean/info_mobile' #无人船数据移动端获取接口
  get '/ocean/water_mobile' #无人船移动端水质数据获取接口
  get '/ocean/mode' #无人船设置工作模式接口
  get '/ocean/download'
  get '/ocean/get_routes' #获取历史csv数据文件
  get '/ocean/get_csv_files'
  get '/ocean/mode_frontend'  #无人船设置工作模式前端接口
  get '/ocean/new_point'#添加路径点接口
  get '/ocean/create_map'
  
  get '/ocean/route_all'
  get '/ocean/route_information'

  get '/ocean/trans_point' # 路径转换
  get '/ocean/send_queue' # 同时发送多个命令
  get '/ocean/bypass' # 绕开
  get '/ocean/remain_point' # 未完成的剩余路径点另存为新文件
  get '/ocean/word_doc' # 保存word文件
  get '/ocean/count_point' # 返回下个路径点
  get '/ocean/upload_file' # 保存上传的bd09格式路径点
  get '/ocean/upload_point' # 保存采集的设备格式路径点为临时文件
  get '/ocean/upload_point2' # 读取临时文件保存为新的csv文件
  get '/ocean/cp_default_file' # 复制web端默认加载的latest.csv文件
  get '/ocean/cp_open_file' # 复制web端打开的文件
  get '/ocean/out_excel' # 导出Excel
  get '/ocean/get_gps_recent'
  get '/ocean/get_status_recent'
  get '/ocean/temp_mongo' # 温度数据存入系统数据库
  get '/ocean/kafka_producer' # kafka测试生产者
  get '/ocean/kafka_consumer' # kafka测试消费者
  get '/ocean/receive_parameter' # 接收小车参数保存在文件内
  get '/ocean/mqtt_pub' # mqtt测试生产者
  get '/ocean/mqtt_sub' # mqtt测试消费者
  get '/ocean/planned_path' # 路径规划

  get '/ocean/position_mobile' #无人船移动端语音输入位置接口
  get '/ocean/position' #无人船前端获取移动端输入位置信息接口
  get '/ocean/warning_mobile' #无人船移动端获取警告信息接口
  get '/ocean/warning' #无人船前端设置警报信息接口
  get '/ocean/get_realtime_loc'
  get '/ocean/mode_return' #无人船前端一键返航指令接口
  get '/ocean/routes' #无人船添加删除工作路径点接口
  get '/ocean/control_relay' #无人船控制继电器接口
  get '/ocean/control_motor' #无人船控制电机接口
  get '/ocean/arm' #电机解锁接口
  get '/ocean/send_message' #send alarm message
  get '/ocean/disarm' #电机上锁接口
  get '/ocean/gps' #另一个GPS模块数据接口
  get '/planstatus/information', to: 'ocean#four_arguments' #无人船数据获取接口
  get '/ocean/control_frontend' #无人船前端控制电机接口 
  get '/ocean/control_manual' #无人船前段控制manual电机接口 

  #测试数据库
  get '/ocean/test_location'
  get '/ocean/test_sensor'

  get '/warehouse/update/:id', to: 'warehouse#update'
  get '/warehouse/alives'

  post '/ocean', to: 'ocean#upload_file'
end