# 快速开始

请确保已安装好 ruby on rails 环境 :-)

>ruby 2.6.3

>rails 5.1.7

使用 git 或直接下载本项目 zip 压缩包

解压后进入项目目录，执行如下命令：

```
$ bundle install        //安装依赖
$ rails s               //启动后台服务
$ rails ocean:init      //连接小车mongodb数据库更新数据
```

特殊场景运行指令：

* 网联温度传感器

```
$ rails ocean:temp
```

* 方向盘

```
$ rails ocean:socket
```

说明：

`rails ocean:xxx` 表示循环运行一个方法

* 对应方法在本项目的 `app\controllers\ocean_controller.rb` 内的 `class << self` 内

* 配置内容在本项目的 `\lib\tasks\ocean.rake` 内。