

#include "pch.h"
#include <iostream>
#include"HK_camera.h"
#include <Windows.h>
#include <thread>
using namespace std;

int main()
{
	
		HK_camera camera;
		if (camera.Init())
		{
			cout << "init success" << endl;
			//方筒摄像头密码为admin+识别号，识别号可在摄像头身上的标签处查看，圆筒摄像头的密码为Hik+识别号
			if (camera.Login("192.168.43.66", "admin", "adminXFITHT", 8000)) //登录
			{
				cout << "login successfully" << endl;
				camera.show();
				//camera.StartRecordThread();
				


			}
			else
			{
				cout << "login fail" << endl;
			}
		}
		else
		{
			cout << "init fail" << endl;
		}
		while (1) 
		{

		}
	
	return 0;
}


