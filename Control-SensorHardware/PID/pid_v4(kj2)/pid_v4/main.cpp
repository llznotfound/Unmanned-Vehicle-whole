#include <iostream>
#include <winsock2.h>  
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#pragma comment(lib,"ws2_32.lib") 
#pragma comment(lib,"Urlmon.lib")

#include "ForwardControl.h"
//using namespace std;

int main()
{
	string str;
	int port_num = 9;
	ForwardControl test(port_num, "ugv", "ugv_status_test", "ugv_angle_status_test", "ugv_point");
	std::cout << "HELLO first??" << std::endl;
	int opt=1;
	std::cout << "select test option (0 - 10): " << endl;
	//std::cin >> opt;
	if (opt == 1) 
	{
		//初始化WSA  
		WORD sockVersion = MAKEWORD(2, 2);
		WSADATA wsaData;
		if (WSAStartup(sockVersion, &wsaData) != 0)
		{
			return 0;
		}

		//创建套接字  
		SOCKET slisten = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		if (slisten == INVALID_SOCKET)
		{
			printf("socket error !");
			return 0;
		}

		//绑定IP和端口  
		sockaddr_in sin;
		sin.sin_family = AF_INET;
		sin.sin_port = htons(8888);
		sin.sin_addr.S_un.S_addr = INADDR_ANY;
		if (::bind(slisten, (LPSOCKADDR)&sin, sizeof(sin)) == SOCKET_ERROR)
		{
			printf("bind error !");
		}

		//开始监听  
		if (listen(slisten, 5) == SOCKET_ERROR)
		{
			printf("listen error !");
			return 0;
		}

		//循环接收数据  
		SOCKET sClient;
		sockaddr_in remoteAddr;
		int nAddrlen = sizeof(remoteAddr);
		char revData[255];
		std::cout << "HELLO??" << std::endl;


	

		while (true)
		{
			std::cout << "HELLO again?" << std::endl;
			printf("等待连接...\n");
			sClient = accept(slisten, (SOCKADDR *)&remoteAddr, &nAddrlen);
			if (sClient == INVALID_SOCKET)
			{
				printf("accept error !");
				continue;
			}
			printf("接受到一个连接：%s \r\n", inet_ntoa(remoteAddr.sin_addr));
			//printf("接受到一个连接：%s \r\n", inet_ntop(remoteAddr.sin_addr));

			//接收数据  
			int ret = recv(sClient, revData, 255, 0);
			cout << "ret=" << ret << endl;
			if (ret > 0)
			{
				revData[ret] = 0x00;
				printf("Received order: %s\n", revData);
				str = revData;
				cout << "str=" << str;
				cout << "str's length=" << str.length() << endl;
				if (str.length() > 20) {
					test.test12(str);
				}
				else if(str.length()==2){



					if (revData[0] == 'M')
					{
						test.test5();
					}
					else if (revData[0] == 'S')
					{
						//test.stop_move_thread();
						test.test9();
					}

					else if (revData[0] == 'D')
					{
						test.test6();
					}
					else if (revData[0] == 'L')
					{
						test.test7();
					}
					else if (revData[0] == 'R')
					{
						test.test8();
					}
					else if (revData[0] == 'B')
					{
						test.test10();
					}
					else if (revData[0] == 'l')
					{
						test.test13();
					}
					else if (revData[0] == 'r')
					{
						test.test14();
					}
				}
				else if (str.length() == 1) {
					if (revData[0] == 'M')
					{
						test.test5();
					}
					else if (revData[0] == 'S')
					{
						//test.stop_move_thread();
						test.test9();
					}

					else if (revData[0] == 'D')
					{
						test.test6();
					}
					else if (revData[0] == 'L')
					{
						test.test7();
					}
					else if (revData[0] == 'R')
					{
						test.test8();
					}
					else if (revData[0] == 'B')
					{
						test.test10();
					}
					else if (revData[0] == 'l')
					{
						test.test13();
					}
					else if (revData[0] == 'r')
					{
						test.test14();
					}
					//test.sendpid(str);
				}

				else if (str.length() >= 6)
				{
					//将方向盘数据帧截取角度，油门，刹车信息并转换为int类型
					//以,为截取符号进行截取
					cout << "===================" << endl;
					cout << endl;
					int angle = atoi((str.substr(0, str.find(","))).c_str());
					int accelerator = atoi((str.substr(str.find(",") + 1, (str.length() - str.rfind(",") - 1))).c_str());
					int brake = atoi((str.substr(str.rfind(",") + 1, str.length())).c_str());


					test.manual_drive(angle, accelerator, brake);
					cout << "angle = " << angle << " " << "accelerator = " << accelerator << " " << "brake = " << brake << endl;
					cout << "corrceting ......" << endl;
					cout << "==================" << endl;
					cout << endl;
				}
			}

			//发送数据  
			//const char * sendData = "你好，TCP客户端！\n";
			//send(sClient, sendData, strlen(sendData), 0);
			closesocket(sClient);
		}

		closesocket(slisten);
		WSACleanup();
	}
	else if (opt == 5)
	{
		test.test5();
	}
	else if(opt==7)
		test.rplidar();
	else
		//test.test11();

	return 0;
}