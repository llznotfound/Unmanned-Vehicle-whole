#define _CRT_SECURE_NO_WARNINGS
#include <Windows.h>
#include <thread>
#include <stdio.h>
#include <iostream>
#include<winsock.h>
#include<ctime>
#include "LogitechSteeringWheelLib.h"

#pragma comment(lib,"ws2_32.lib")
#pragma comment(lib, "LogitechSteeringWheelLib.lib")
#pragma comment( linker, "/subsystem:\"console\" /entry:\"WinMainCRTStartup\"")


using namespace std;

int send_flag = 0;
int send_count = 0;

void  Get_Current_Date()
{
	SYSTEMTIME sys;
	GetLocalTime(&sys);
	printf("%4d/%02d/%02d %02d:%02d:%02d.%03d 星期%1d\n", sys.wYear, sys.wMonth, sys.wDay, sys.wHour, sys.wMinute, sys.wSecond, sys.wMilliseconds, sys.wDayOfWeek);
	//return 0;

}

void initialization() {
	//初始化套接字库
	WORD w_req = MAKEWORD(2, 2);//版本号
	WSADATA wsadata;
	int err;
	err = WSAStartup(w_req, &wsadata);
	if (err != 0) {
		cout << "初始化套接字库失败！" << endl;
	}
	else {
		//cout << "初始化套接字库成功！" << endl;
	}
	//检测版本号
	if (LOBYTE(wsadata.wVersion) != 2 || HIBYTE(wsadata.wHighVersion) != 2) {
		cout << "套接字库版本号不符！" << endl;
		WSACleanup();
	}
	else {
		//cout << "套接字库版本正确！" << endl;
	}
	//填充服务端地址信息

}


void SendMessageWithSocket(HWND hwnd)
{
	
	int x, y, z;
	int send_len = 0;
	int recv_len = 0;
	//定义发送缓冲区和接受缓冲区
	char send_buf[100];
	char recv_buf[100];
	//定义服务端套接字，接受请求套接字
	SOCKET s_server;
	//服务端地址客户端地址
	SOCKADDR_IN server_addr;
	initialization();
	//填充服务端信息
	server_addr.sin_family = AF_INET;
	//server_addr.sin_addr.S_un.S_addr = inet_addr("47.97.153.27");
	server_addr.sin_addr.S_un.S_addr = inet_addr("192.168.1.124");
	server_addr.sin_port = htons(1888);
	//server_addr.sin_port = htons(23334);
	//创建套接字
	s_server = socket(AF_INET, SOCK_STREAM, 0);
	if (connect(s_server, (SOCKADDR *)&server_addr, sizeof(SOCKADDR)) == SOCKET_ERROR) {
		//cout << "服务器连接失败！" << endl;
		WSACleanup();
	}
	else {
		//cout << "服务器连接成功！" << endl;
	}


	if (LogiSteeringInitializeWithWindow(true, hwnd))
	{

		printf("init secuss\n");
		//printf("%d\n", hwnd);
		while (LogiUpdate() && LogiIsConnected(1))
		{
			//printf("connect secuss\n");
			Sleep(100);
			DIJOYSTATE2 * wheel = LogiGetState(1);
			if (LogiButtonTriggered(1, 19))
			{
				printf("-----------------------\n");
				printf("Button 19 Pressed\n");
				printf("-----------------------\n");
			}

			if (LogiButtonTriggered(1, 20))
			{
				printf("-----------------------\n");
				printf("Button 20 Pressed\n");
				printf("-----------------------\n");
			}

			if (LogiButtonTriggered(1, 23))
			{
				send_count++;
				cout << "count is : " << send_count << endl;
			}
			//输出角度，油门，刹车信息
			//cout << "获取时间:";
			//Get_Current_Date();
			//printf("Angle = %d  Accelerator = %d  Brake = %d\n", wheel->lX, wheel->lY,wheel->lRz);
			//SendMessage(wheel->lX, wheel->lY, wheel->lRz);
			if (send_count % 2 == 0)
			{
				//SendMessage(0, 0, 0);
				x = 0;
				y = 0;
				z = 0;
				sprintf_s(send_buf, ",%d,%d,%d,", x, y, z);
				send_len = send(s_server, send_buf, strlen(send_buf), 0);


				if (send_len < 0) {

					cout << "发送失败！" << endl;

				}

				printf("x = %d,y = %d,z = %d\n", x, y, z);
			}
			if (send_count % 2 == 1)
			{
				//SendMessage(wheel->lX, wheel->lY, wheel->lRz);
				x = wheel->lX;
				y = wheel->lY;
				z = wheel->lRz;

				sprintf_s(send_buf, ",%d,%d,%d,", x, y, z);
				send_len = send(s_server, send_buf, strlen(send_buf), 0);


				if (send_len < 0) {

					//cout << "发送失败！" << endl;

				}

				printf("x = %d,y = %d,z = %d\n", x, y, z);
			}
			switch (wheel->rgdwPOV[0])
			{
			case(0):
				cout << "D-pad Up Button Pressed" << endl; break;
			case(18000):
				cout << "D-pad Down Button Pressed" << endl; break;
			case(27000):
				cout << "D-pad Left Button Pressed" << endl; break;
			case(9000):
				cout << "D-pad Right Button Pressed" << endl; break;
			case(31500):
				cout << "D-pad Left-up Button Pressed" << endl; break;
			case(4500):
				cout << "D-pad Right-up Button Pressed" << endl; break;
			case(22500):
				cout << "D-pad Left-down Button Pressed" << endl; break;
			case(13500):
				cout << "D-pad Right-down Button Pressed" << endl; break;


			}


		}

	}
	else
	{
		printf("init faild");
	}


	//关闭套接字
	closesocket(s_server);
	//释放DLL资源
	WSACleanup();
}


int SendMessageWithUdp(HWND hwnd)             //使用udp协议发送方向盘数据
{
	WORD wVersionRequested;
	WSADATA wsaData;
	int err;

	wVersionRequested = MAKEWORD(1, 1);

	err = WSAStartup(wVersionRequested, &wsaData);
	if (err != 0) 
	{
		return 0;
	}

	if (LOBYTE(wsaData.wVersion) != 1 || HIBYTE(wsaData.wVersion) != 1) 
	{
		WSACleanup();
		return 0;
	}
	printf("Client is operating!\n\n");

	SOCKET sockSrv = socket(AF_INET, SOCK_DGRAM, 0);

	sockaddr_in  addrSrv;
	addrSrv.sin_addr.S_un.S_addr = inet_addr("47.110.72.141");           //使用natapp穿透工具得到的IP地址
	//addrSrv.sin_addr.S_un.S_addr = inet_addr("127.0.0.1");
	addrSrv.sin_family = AF_INET;
	addrSrv.sin_port = htons(2599);
	//addrSrv.sin_port = htons(2020);

	int len = sizeof(SOCKADDR);
	char sendBuf[100];

	int x, y, z;

	if (LogiSteeringInitializeWithWindow(true, hwnd))
	{

		printf("init secuss\n");
		//printf("%d\n", hwnd);
		while (LogiUpdate() && LogiIsConnected(1))
		{
			//printf("connect secuss\n");
			Sleep(100);
			DIJOYSTATE2 * wheel = LogiGetState(1);         //wheel结构体中存储有相关方向盘的数据

			if (LogiButtonTriggered(1, 19))
			{
				printf("-----------------------\n");
				printf("Button 19 Pressed\n");
				printf("-----------------------\n");
			}

			if (LogiButtonTriggered(1, 20))
			{
				printf("-----------------------\n");
				printf("Button 20 Pressed\n");
				printf("-----------------------\n");
			}

			if (LogiButtonTriggered(1, 23))
			{
				send_count++;
				cout << "count is : " << send_count << endl;
			}
			//输出角度，油门，刹车信息
			//cout << "获取时间:";
			//Get_Current_Date();
			//printf("Angle = %d  Accelerator = %d  Brake = %d\n", wheel->lX, wheel->lY,wheel->lRz);
			//SendMessage(wheel->lX, wheel->lY, wheel->lRz);
			if (send_count % 2 == 0)       //发送逻辑为方向盘默认发送000数据，当红色回车按钮按下，状态进行翻转，发送数据
			{
				//SendMessage(0, 0, 0);
				x = 0;
				y = 0;
				z = 0;
				sprintf_s(sendBuf, "%d,%d,%d", x, y, z);
				//send_len = send(s_server, send_buf, strlen(send_buf), 0);
				int send_len = sendto(sockSrv, sendBuf, strlen(sendBuf) + 1, 0, (SOCKADDR*)&addrSrv, len);

				if (send_len < 0) {

					cout << "发送失败！" << endl;

				}

				printf("x = %d,y = %d,z = %d\n", x, y, z);
			}
			if (send_count % 2 == 1)
			{
				//SendMessage(wheel->lX, wheel->lY, wheel->lRz);
				x = wheel->lX;
				y = wheel->lY;
				z = wheel->lRz;

				sprintf_s(sendBuf, "%d,%d,%d", x, y, z);
				int send_len = sendto(sockSrv, sendBuf, strlen(sendBuf) + 1, 0, (SOCKADDR*)&addrSrv, len);


				if (send_len < 0) {

					//cout << "发送失败！" << endl;

				}

				printf("x = %d,y = %d,z = %d\n", x, y, z);
			}
			switch (wheel->rgdwPOV[0])              //方向盘十字键是否按下判断
			{
			case(0):
				cout << "D-pad Up Button Pressed" << endl; break;
			case(18000):
				cout << "D-pad Down Button Pressed" << endl; break;
			case(27000):
				cout << "D-pad Left Button Pressed" << endl; break;
			case(9000):
				cout << "D-pad Right Button Pressed" << endl; break;
			case(31500):
				cout << "D-pad Left-up Button Pressed" << endl; break;
			case(4500):
				cout << "D-pad Right-up Button Pressed" << endl; break;
			case(22500):
				cout << "D-pad Left-down Button Pressed" << endl; break;
			case(13500):
				cout << "D-pad Right-down Button Pressed" << endl; break;


			}


		}

	}
	else
	{
		printf("init faild");
	}

}










// 必须要进行前导声明  
LRESULT CALLBACK WindowProc(
	_In_  HWND hwnd,
	_In_  UINT uMsg,
	_In_  WPARAM wParam,
	_In_  LPARAM lParam
);

int CALLBACK WinMain(
	_In_  HINSTANCE hInstance,
	_In_  HINSTANCE hPrevInstance,
	_In_  LPSTR lpCmdLine,
	_In_  int nCmdShow
)
{
	// 类名  
	TCHAR cls_Name[] = L"My Class";
	// 设计窗口类  
	WNDCLASS wc = { sizeof(WNDCLASS) };
	wc.hbrBackground = (HBRUSH)COLOR_WINDOW;
	wc.lpfnWndProc = WindowProc;
	wc.lpszClassName = cls_Name;
	wc.hInstance = hInstance;
	wc.style = CS_HREDRAW | CS_VREDRAW;

	// 注册窗口类  
	RegisterClass(&wc);

	// 创建窗口
	HWND hwnd = CreateWindow(
		cls_Name,           //类名，要和刚才注册的一致  
		L"方向盘Demo",          //窗口标题文字  
		WS_OVERLAPPEDWINDOW,        //窗口外观样式  
		38,             //窗口相对于父级的X坐标  
		20,             //窗口相对于父级的Y坐标  
		500,                //窗口的宽度  
		500,                //窗口的高度  
		NULL,               //没有父窗口，为NULL  
		NULL,               //没有菜单，为NULL  
		hInstance,          //当前应用程序的实例句柄  
		NULL);              //没有附加数据，为NULL  
	if (hwnd == NULL)                //检查窗口是否创建成功  
		return 0;

	// 显示窗口  
	ShowWindow(hwnd, SW_SHOW);

	// 更新窗口  
	UpdateWindow(hwnd);

	SetWindowPos(hwnd, HWND_TOPMOST, 0, 0, 0, 0, SWP_NOMOVE | SWP_NOSIZE); //设置窗口最前端
	//thread GetWheelData(WheelInit, hwnd);
	//GetWheelData.detach();
	//SendMessage(0, 0, 0);
	thread SendWheelData(SendMessageWithUdp, hwnd);
	SendWheelData.detach();
	

	//MessageBox(0, "调用了WinMain函数", "测试：", 0);


	// 消息循环  
	MSG msg;
	while (GetMessage(&msg, NULL, 0, 0))
	{
		TranslateMessage(&msg);
		DispatchMessage(&msg);
	}
	return 0;
}

// 在WinMain后实现  
LRESULT CALLBACK WindowProc(
	_In_  HWND hwnd,
	_In_  UINT uMsg,
	_In_  WPARAM wParam,
	_In_  LPARAM lParam
)
{

	switch (uMsg)
	{
	case WM_DESTROY:
	{
		PostQuitMessage(0);
		return 0;
	}
	case WM_PAINT:
	{
		/*PAINTSTRUCT     ps;
		HDC hdc = BeginPaint(hwnd, &ps);
		int length;
		TCHAR buff[1024];
		length = wsprintf(buff, TEXT("the angle is : %d"), x);
		TextOut(hdc, 20, 20, buff, length);
		EndPaint(hwnd, &ps);
		break;*/

	}
	default:
		break;
	}
	return DefWindowProc(hwnd, uMsg, wParam, lParam);
}


