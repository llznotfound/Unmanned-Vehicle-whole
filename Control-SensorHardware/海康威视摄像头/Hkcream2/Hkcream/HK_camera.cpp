#include "pch.h"

#include <mongoc.h>
#include <bson.h>
#include"HK_camera.h"
#include "PlayM4.h"
#include <windows.h>
#include <string>
#include <ctime>
#include <iostream>
#include <thread>

#pragma comment(lib,"../3rd_x64/mongo-c-driver/usr/lib/bson-1.0.lib")
#pragma comment(lib,"../3rd_x64/mongo-c-driver/usr/lib/mongoc-1.0.lib")

using namespace std;





//全局变量
LONG g_nPort;
Mat g_BGRImage;
LONG lRealHandle;

//数据解码回调函数，
//功能：将YV_12格式的视频数据流转码为可供opencv处理的BGR类型的图片数据，并实时显示。
void CALLBACK DecCBFun(long nPort, char* pBuf, long nSize, FRAME_INFO* pFrameInfo, long nUser, long nReserved2)
{
	if (pFrameInfo->nType == T_YV12)
	{
		std::cout << "the frame infomation is T_YV12" << std::endl;
		if (g_BGRImage.empty())
		{
			g_BGRImage.create(pFrameInfo->nHeight, pFrameInfo->nWidth, CV_8UC3);
		}
		Mat YUVImage(pFrameInfo->nHeight + pFrameInfo->nHeight / 2, pFrameInfo->nWidth, CV_8UC1, (unsigned char*)pBuf);

		cvtColor(YUVImage, g_BGRImage, COLOR_YUV2BGR_YV12);
		imshow("RGBImage1", g_BGRImage);
		waitKey(15);

		YUVImage.~Mat();
	}
}

//实时视频码流数据获取 回调函数
void CALLBACK g_RealDataCallBack_V30(LONG lPlayHandle, DWORD dwDataType, BYTE *pBuffer, DWORD dwBufSize, void* pUser)
{
	if (dwDataType == NET_DVR_STREAMDATA)//码流数据
	{
		if (dwBufSize > 0 && g_nPort != -1)
		{
			if (!PlayM4_InputData(g_nPort, pBuffer, dwBufSize))
			{
				//std::cout << "fail input data" << std::endl;
			}
			else
			{
				std::cout << "success input data" << std::endl;
			}

		}
	}
}
//构造函数
HK_camera::HK_camera(void)
{

}
//析构函数
HK_camera::~HK_camera(void)
{
}
//初始化函数，用作初始化状态检测
bool HK_camera::Init()
{
	if (NET_DVR_Init())
	{
		return true;
	}
	else
	{
		return false;
	}
}

//登录函数，用作摄像头id以及密码输入登录
//bool HK_camera::Login(char* sDeviceAddress, char* sUserName, char* sPassword, WORD wPort)
bool HK_camera::Login(const char* sDeviceAddress, const char* sUserName, const char* sPassword, WORD wPort)       //登陆（VS2017版本）
{
	NET_DVR_USER_LOGIN_INFO pLoginInfo = { 0 };
	NET_DVR_DEVICEINFO_V40 lpDeviceInfo = { 0 };

	pLoginInfo.bUseAsynLogin = 0;     //同步登录方式
	strcpy_s(pLoginInfo.sDeviceAddress, sDeviceAddress);
	strcpy_s(pLoginInfo.sUserName, sUserName);
	strcpy_s(pLoginInfo.sPassword, sPassword);
	pLoginInfo.wPort = wPort;

	lUserID = NET_DVR_Login_V40(&pLoginInfo, &lpDeviceInfo);

	if (lUserID < 0)
	{
		return false;
	}
	else
	{
		return true;
	}
}

//视频流显示函数
void HK_camera::show()
{
	if (PlayM4_GetPort(&g_nPort))            //获取播放库通道号
	{
		if (PlayM4_SetStreamOpenMode(g_nPort, STREAME_REALTIME))      //设置流模式
		{
			if (PlayM4_OpenStream(g_nPort, NULL, 0, 1024 * 1024))         //打开流
			{
				if (PlayM4_SetDecCallBackExMend(g_nPort, DecCBFun, NULL, 0, NULL))
				{
					if (PlayM4_Play(g_nPort, NULL))
					{
						std::cout << "success to set play mode" << std::endl;
					}
					else
					{
						std::cout << "fail to set play mode" << std::endl;
					}
				}
				else
				{
					std::cout << "fail to set dec callback " << std::endl;
				}
			}
			else
			{
				std::cout << "fail to open stream" << std::endl;
			}
		}
		else
		{
			std::cout << "fail to set stream open mode" << std::endl;
		}
	}
	else
	{
		std::cout << "fail to get port" << std::endl;
	}
	//启动预览并设置回调数据流
	NET_DVR_PREVIEWINFO struPlayInfo = { 0 };
	struPlayInfo.hPlayWnd = NULL; //窗口为空，设备SDK不解码只取流
	struPlayInfo.lChannel = 1; //Channel number 设备通道
	struPlayInfo.dwStreamType = 0;// 码流类型，0-主码流，1-子码流，2-码流3，3-码流4, 4-码流5,5-码流6,7-码流7,8-码流8,9-码流9,10-码流10
	struPlayInfo.dwLinkMode = 0;// 0：TCP方式,1：UDP方式,2：多播方式,3 - RTP方式，4-RTP/RTSP,5-RSTP/HTTP 
	struPlayInfo.bBlocked = 1; //0-非阻塞取流, 1-阻塞取流, 如果阻塞SDK内部connect失败将会有5s的超时才能够返回,不适合于轮询取流操作.

	if (NET_DVR_RealPlay_V40(lUserID, &struPlayInfo, g_RealDataCallBack_V30, NULL))
	{
		namedWindow("RGBImage2");
	}
}

inline bool isexists(const std::string& name) {
	struct stat buffer;
	return (stat(name.c_str(), &buffer) == 0);
}

string getlocaldata()
{
	mongoc_client_t *client;
	mongoc_collection_t *collection;
	//mongoc_collection_t  *col_status;
	bson_t *filter;
	bson_t *opts;
	mongoc_cursor_t *cursor;
	const bson_t *doc;
	char *str = NULL;
	string s2;


	mongoc_init();

	client = mongoc_client_new("mongodb://localhost:27017/");
	collection = mongoc_client_get_collection(client, "ugv", "ugv_status_test");

	filter = BCON_NEW();
	opts = BCON_NEW("limit", BCON_INT64(1), "sort", "{", "_id", BCON_INT32(-1), "}");
	cursor = mongoc_collection_find_with_opts(collection, filter, opts, NULL);

	while (mongoc_cursor_next(cursor, &doc)) {
		str = bson_as_json(doc, NULL);
		s2 = str;
		//printf("%s\n", str);
		//cout << s2.substr(72, 21) << endl;
		//cout << s2.substr(95, 22) << endl;
		bson_free(str);
	}

	bson_destroy(filter);
	bson_destroy(opts);
	mongoc_cursor_destroy(cursor);
	mongoc_collection_destroy(collection);
	mongoc_client_destroy(client);
	mongoc_cleanup();


	return s2.substr(81, 10) + "-" + s2.substr(104, 12);

}





void HK_camera::Record()
{



	char videoname[256];
	char localname[64];
	time_t currTime;
	struct tm *mt;
	string localdata;

	NET_DVR_PREVIEWINFO struPlayInfo = { 0 };
	struPlayInfo.hPlayWnd = NULL; //窗口为空，设备SDK不解码只取流
	struPlayInfo.lChannel = 1; //Channel number 设备通道
	struPlayInfo.dwStreamType = 0;// 码流类型，0-主码流，1-子码流，2-码流3，3-码流4, 4-码流5,5-码流6,7-码流7,8-码流8,9-码流9,10-码流10
	struPlayInfo.dwLinkMode = 0;// 0：TCP方式,1：UDP方式,2：多播方式,3 - RTP方式，4-RTP/RTSP,5-RSTP/HTTP 
	struPlayInfo.bBlocked = 1; //0-非阻塞取流, 1-阻塞取流, 如果阻塞SDK内部connect失败将会有5s的超时才能够返回,不适合于轮询取流操作.
	
	

	while(true) {
		currTime = time(NULL);
		mt = localtime(&currTime);
		/*根据日期生成文件名*/
		lRealHandle = NET_DVR_RealPlay_V40(lUserID, &struPlayInfo, g_RealDataCallBack_V30, NULL);
		localdata = "A-" + getlocaldata();
		cout << getlocaldata() << endl;
		for (int i = 0; i < localdata.length(); i++) 
		{
			localname[i] = localdata[i];
		}
		cout << localname << endl;
		
		/*sprintf_s(videoname, "C:\\Users\\Administrator.SC-201905221819\\Desktop\\vidio\\%s-%d-%02d-%02d-%02d-%02d-%02d.mp4", \
			localdata,mt->tm_year + 1900, mt->tm_mon + 1, mt->tm_mday, mt->tm_hour, mt->tm_min, mt->tm_sec);*/

		sprintf_s(videoname, "C:\\Users\\Administrator.SC-201905221819\\Desktop\\vidio\\%s-%d-%02d-%02d-%02d-%02d-%02d.mp4", \
			localname,mt->tm_year + 1900, mt->tm_mon + 1, mt->tm_mday, mt->tm_hour, mt->tm_min, mt->tm_sec);

		NET_DVR_SaveRealData(lRealHandle, videoname);
		cout << "开始录制，文件位置：" << videoname << endl;
		for (int j = 0; j < 6; j++) 
		{
			Sleep(10000);
		}
		
		NET_DVR_StopSaveRealData(lRealHandle);
		if (isexists(videoname))
		{
			cout << "视频录制成功" << endl;
		}
		else
		{
			cout << "视频录制失败" << endl;
		}
		
	}
	
  
}

void HK_camera::StartRecordThread()
{
	RecordThread = thread(&HK_camera::Record, this);
	RecordThread.detach();
}



