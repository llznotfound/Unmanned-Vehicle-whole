#include "ForwardControl.h"
#include <typeinfo>
#include <urlmon.h>
#include <iostream>
#include <cstring>
#include <fstream>
#include<stdio.h>
#include<winsock2.h>
#include<windows.h>
#include "rplidar.h"
#include<cstdio>
#include <queue>
#include <wininet.h>
#pragma comment(lib, "ws2_32.lib")
#pragma comment(lib,"Wininet.lib")


#define PI 3.14159265358979323846

using namespace std;
using namespace rp::standalone::rplidar;
int flag = 0;
bool ctrl_c_pressed = false;
ForwardControl::ForwardControl(int port_num, const char* db_name, const char* col_real_time_name, const char* col_ang_name, const char* col_route_name)
{
	serial_con.InitPort(port_num);
	f_ctl.is_ctl = 0x01;
	f_ctl.speed_h = 0x08;
	f_ctl.speed_l = 0xB4;

	// init db
	// 数据库初始化
	mongoc_init();
	//client = mongoc_client_new("mongodb://192.168.43.4:27017");
	client = mongoc_client_new("mongodb://localhost:27017");
	database = mongoc_client_get_database(client, db_name);
	col_status = mongoc_client_get_collection(client, db_name, col_real_time_name);
	col_angle = mongoc_client_get_collection(client, db_name, col_ang_name);
	col_route = mongoc_client_get_collection(client, db_name, col_route_name);
}

ForwardControl::~ForwardControl(void)
{
	f_ctl.is_ctl = 0x00;
	f_ctl.angle = 0x7f;
	mongoc_collection_destroy(col_angle);
	mongoc_collection_destroy(col_status);
	mongoc_collection_destroy(col_route);
	mongoc_database_destroy(database);
	mongoc_client_destroy(client);
	mongoc_cleanup();
}

// get position of point i in route j from the database
//routePoint ForwardControl::get_route(int n_route, int n_point)
//{
//	string lat, lon;
//	bson_t *filter;
//	bson_t *opts;
//	mongoc_cursor_t *cursor;
//	const bson_t *doc;
//	char *str = NULL;
//	routePoint point;
//
//	filter = BCON_NEW("plan_id", BCON_INT64(n_route), "waypoint_no", BCON_INT64(n_point));
//	//opts = BCON_NEW("limit", BCON_INT64(1), "sort", "{", "time", BCON_INT32(-1), "}");
//	opts = BCON_NEW("limit", BCON_INT64(1), "sort", "{", "_id", BCON_INT32(-1), "}");
//	cursor = mongoc_collection_find_with_opts(col_route, filter, opts, NULL);
//
//	if (mongoc_cursor_next(cursor, &doc)) {
//		str = bson_as_json(doc, NULL);
//		Json::CharReaderBuilder builder;
//		Json::CharReader *reader = builder.newCharReader();
//		Json::Value info;
//		string errors;
//		if (reader->parse(str, str + strlen(str), &info, &errors))
//		{
//			point.lat = gps_m2d(info["lat"].asCString(), "route");
//			point.lon = gps_m2d(info["lng"].asCString(), "route");
//			point.n_point = info["waypoint_sum"].asInt64();
//		}
//		bson_free(str);
//	}
//	return point;
//}

//从数据库中取出最后一条数据，表示小车的位置和小车行驶的角度（经纬度的格式同时转换成实际经纬度）
currStatus ForwardControl::get_curr_status()
{
	//ofstream outfile;
	//outfile.open("C:\\Users\\Administrator\\Desktop\\angle.txt", ios::app);
	//outfile << "lat, lon, current_angle, set_angle, error_angle, out_angle, pid, f_ctl.angle" << endl; // 写入数据的类型
	bson_t *filter;
	bson_t *opts;
	mongoc_cursor_t *cursor;
	const bson_t *doc;
	char *str = NULL;
	currStatus status;

	filter = BCON_NEW();
	//opts = BCON_NEW("limit", BCON_INT64(1), "sort", "{", "time", BCON_INT32(-1), "}");
	opts = BCON_NEW("limit", BCON_INT64(1), "sort", "{", "_id", BCON_INT32(-1), "}");
	cursor = mongoc_collection_find_with_opts(col_status, filter, opts, NULL);

	//cout << "Getting current location..." << endl;
	double start = time(NULL);
	if (mongoc_cursor_next(cursor, &doc)) {
		str = bson_as_json(doc, NULL);
		Json::CharReaderBuilder builder;
		Json::CharReader *reader = builder.newCharReader();
		Json::Value info;
		string errors;
		if (reader->parse(str, str + strlen(str), &info, &errors))
		{
			status.lat = gps_m2d(info["lat"].asCString(), "status");
			status.lon = gps_m2d(info["lng"].asCString(), "status");
		}
		bson_free(str);
	}

	filter = BCON_NEW();
	//opts = BCON_NEW("limit", BCON_INT64(1), "sort", "{", "time", BCON_INT32(-1), "}");
	opts = BCON_NEW("limit", BCON_INT64(1), "sort", "{", "_id", BCON_INT32(-1), "}");
	cursor = mongoc_collection_find_with_opts(col_angle, filter, opts, NULL);

	//cout << "Getting current angle..." << endl;
	if (mongoc_cursor_next(cursor, &doc)) {
		str = bson_as_json(doc, NULL);
		Json::CharReaderBuilder builder;
		Json::CharReader *reader = builder.newCharReader();
		Json::Value info;
		string errors;
		if (reader->parse(str, str + strlen(str), &info, &errors))
		{
			status.angle = info["angle_z"].asFloat()-8;
			//outfile << status.angle << endl;
		}
		bson_free(str);
	}
	
	double end = time(NULL);

	return status;
}

//从数据库中取出最后一条数据，表示小车的位置和小车行驶的角度（经纬度的格式没有转换）
currStatus1 ForwardControl::get_curr_status1()
{
	bson_t *filter;
	bson_t *opts;
	mongoc_cursor_t *cursor;
	const bson_t *doc;
	char *str = NULL;
	currStatus1 status;

	filter = BCON_NEW();
	//opts = BCON_NEW("limit", BCON_INT64(1), "sort", "{", "time", BCON_INT32(-1), "}");
	opts = BCON_NEW("limit", BCON_INT64(1), "sort", "{", "_id", BCON_INT32(-1), "}");
	cursor = mongoc_collection_find_with_opts(col_status, filter, opts, NULL);

	//cout << "Getting current location..." << endl;
	double start = time(NULL);
	if (mongoc_cursor_next(cursor, &doc)) {
		str = bson_as_json(doc, NULL);
		Json::CharReaderBuilder builder;
		Json::CharReader *reader = builder.newCharReader();
		Json::Value info;
		string errors;
		if (reader->parse(str, str + strlen(str), &info, &errors))
		{
			status.lat = info["lat"].asCString();
			status.lon = info["lng"].asCString();
			//status.lon = double(stoi(a.substr(0, a.find(".", sizeof('.')) - 2)))*100 + stod(a.substr(a.find(".", sizeof('.')) - 2, a.length()));
		}
		bson_free(str);
	}

	filter = BCON_NEW();
	//opts = BCON_NEW("limit", BCON_INT64(1), "sort", "{", "time", BCON_INT32(-1), "}");
	opts = BCON_NEW("limit", BCON_INT64(1), "sort", "{", "_id", BCON_INT32(-1), "}");
	cursor = mongoc_collection_find_with_opts(col_angle, filter, opts, NULL);

	//cout << "Getting current angle..." << endl;
	if (mongoc_cursor_next(cursor, &doc)) {
		str = bson_as_json(doc, NULL);
		Json::CharReaderBuilder builder;
		Json::CharReader *reader = builder.newCharReader();
		Json::Value info;
		string errors;
		if (reader->parse(str, str + strlen(str), &info, &errors))
		{
			status.angle = info["angle_z"].asFloat();
		}
		bson_free(str);
	}

	double end = time(NULL);

	return status;
}

// 设置pid参数
bool ForwardControl::set_pid_gains(float kp, float ki, float kd)
{
	pid.set_gains(kp, ki, kd);
	return TRUE;
}

bool ForwardControl::set_pid_sample_t(float sec)
{
	t_sam = sec;
	pid.set_frequency(t_sam);
	return TRUE;
}

// 开始自动行驶线程和激光雷达线程
void ForwardControl::start_move_thread(void)
{
	rplidarThread = thread(&ForwardControl::rplidar, this);
	f_ctl.is_ctl = 0x01;
	f_ctl.angle = 0x7f;
	//sendCmdThread = thread(&ForwardControl::send_cmd, this);
	Sleep(500);
	mvThread = thread(&ForwardControl::move, this);

	if (rplidarThread.joinable())
		rplidarThread.detach();
	if (mvThread.joinable())
		mvThread.detach();
	//if (sendCmdThread.joinable())
		//sendCmdThread.detach();

}

//bool ForwardControl::stop_move_thread(void)
//{
//	switch_to_stop();
//	cout << "Stopping" << endl;
//	if(mvThread.joinable())
//		mvThread.join();
//	if(sendCmdThread.joinable())
//		sendCmdThread.join();
//	cout << "Stopped" << endl;
//	return true;
//}

// 停下小车
void ForwardControl::switch_to_stop()
{
	fmu.lock();
	f_ctl.angle = 0x7f;
	f_ctl.speed_h = (0 + 500) / 255;
	f_ctl.speed_l = (0 + 500) - f_ctl.speed_h * 255;
	f_ctl.is_ctl = 0x00;
	fmu.unlock();
	serial_con.WriteFrame(f_ctl);
	//mv_flag = FALSE;
}

// 向服务器发送小车编号和当前位置的经纬度
void ForwardControl::sendloc(currStatus1 curr) //send location to the database using request
{
	string lat = curr.lat;
	string lng = curr.lon;
	//string lat = "123.111";
	//string lng = "321.222";
	string ip_tail = "101";

	
	string url= "http://192.168.1." + ip_tail + ":8484/ocean/get_realtime_loc?id=A&lat=" +lat+ "&lng=" + lng;
	if (URLOpenStream(NULL, url.c_str(), 0, 0) == S_OK) {
		cout << "Real-time Location Send SUCCEEDED." << endl;
	}
	else
	{
		cout << "Location Send FAIL." << endl;
		return;
	}

}

// 调用后台生成报告的方法
void ForwardControl::sendword()
{
	string ip_tail = "101";

	string url = "http://192.168.1." + ip_tail + ":8484/ocean/word_doc";
	if (URLOpenStream(NULL, url.c_str(), 0, 0) == S_OK) {
		cout << "Send SUCCEEDED." << endl;
	}
	else
	{
		cout << "Send FAIL." << endl;
		return;
	}

}

void ForwardControl::sendpid(string s)
{
	string ip_tail = "5";

	string url = "http://172.20.10." + ip_tail + ":5000/pid_end?flag="+s;
	DeleteUrlCacheEntry(url.c_str());
	if (URLOpenStream(NULL, url.c_str(), 0, 0) == S_OK) {
		cout << "Send SUCCEEDED." << endl;
	}
	else
	{
		cout << "Send FAIL." << endl;
		return;
	}

}

void ForwardControl::socket_send(string s) {
	int send_len = 0;
	int recv_len = 0;
	//发送和接收缓冲区
	char send_buf[100];
	char recv_buf[100];
	SOCKET s_server;
	SOCKADDR_IN server_addr;
	WORD w_req = MAKEWORD(2, 2);
	WSADATA wsadata;
	int err;
	err = WSAStartup(w_req, &wsadata);
	if (err != 0) {
		cout << "初始化套接字库失败！" << endl;
	}
	else {
		cout << "初始化套接字库成功！" << endl;
	}

	if (LOBYTE(wsadata.wVersion) != 2 || HIBYTE(wsadata.wHighVersion) != 2) {
		cout << "套接字库版本号不符！" << endl;
		WSACleanup();
	}
	else {
		cout << "套接字库版本正确！" << endl;
	}


	s_server = socket(AF_INET, SOCK_STREAM, 0);
	server_addr.sin_family = AF_INET;
	server_addr.sin_addr.S_un.S_addr = inet_addr("127.0.0.1");
	server_addr.sin_port = htons(8888);
	s_server = socket(AF_INET, SOCK_STREAM, 0);
	if (connect(s_server, (SOCKADDR *)&server_addr, sizeof(SOCKADDR)) == SOCKET_ERROR) {
		cout << "服务器连接失败" << endl;
		WSACleanup();
	}
	else {
		cout << "服务器连接成功！" << endl;
	}
	cout << "请输入发送信息：";
	strcpy(send_buf, s.c_str());
	//cin >> send_buf;
	send_len = send(s_server, send_buf, 100, 0);
	closesocket(s_server);
	WSACleanup();
	if (send_len < 0) {
		cout << "发送失败！" << endl;
	}

	closesocket(s_server);
	WSACleanup();
}

void ForwardControl::sendAlarm(currStatus curr) //发送短信报警
{
	string lat = to_string(curr.lat);
	string lng = to_string(curr.lon);
	string ip_tail = "101";

	string url = "http://192.168.1." + ip_tail + "/ocean/send_message?id=A&lat=" + lat + "&lng=" + lng;
	if (URLOpenStream(NULL, url.c_str(), 0, 0) == S_OK) {
		cout << "Alarm Send SUCCEEDED." << endl;
	}
	else
	{
		cout << "Alarm Send FAIL." << endl;
		return;
	}

}

// main thread of moving forward
// 计算小车当前位置与下一目标点之间的角度，以北偏角显示，计算应发送给小车控制板的角度值
// 自动行驶的主控制方法
void ForwardControl::move(void)
{
	double out_angle, error, lastdist, currdist, cur_angle, dis1, dis2;
	currStatus curr_status, tmp_status;
	currStatus1 curr_status1;
	int i_route = 0, i_point = 0, n_point = 0 ,count = 0;
	routePoint pA, pB, p;
	time_t t0, t1, t2;
	int speed;
	int cnt = 0;
	/*int ang_cnt = 0;
	int ang_sign = 0;
	int error_cnt = 0;*/

	string pid_cmd;
	ofstream outfile;
	// 角度文件的保存路径
	outfile.open("C:\\Users\\Administrator\\Desktop\\testdata1.txt", ios::app);
	outfile << "lat, lon, current_angle, set_angle, error_angle, out_angle, pid, f_ctl.angle" << endl; // 写入数据的类型
	while (1)
	{
		if (mv_flag == FALSE)
			break;
		i_route = 0;
		i_point = 0;
		n_point = 0;
		while (1)
		{

			tmp_status = get_curr_status();
			pA.lat = tmp_status.lat;
			pA.lon = tmp_status.lon;
			bool changestatus = false;
			// 获得目标点的个数
			n_point = desired_points.size();
			// i_point为正在前往的目标点的点数
			while (i_point < n_point)
			{
			
				pB = desired_points[i_point]; //pB表示正在前往的目标点
				
				cout << "current position: " << fixed << pA.lat << ", " << fixed << pA.lon << endl;
				cout << "desired point is: "<<pB.lat << ", " << pB.lon << endl;
				//while (pB.n_point == 0)
				//	pB = get_route(i_route, i_point + 1);
				set_angle = get_azimuth_desired(pA.lat, pA.lon, pB.lat, pB.lon);
				cout << "desired angle: " << fixed << set_angle << endl;
				// PID controller
				t0 = time(NULL);
				t1 = t0;
				while (1)
				{
					if (auto_flag) {


						t2 = time(NULL);
						if (false && t2 - t1 < t_sam)
							continue;
						else
						{
							// get current position from db
							// curr_status为当前位置的信息，经纬度和角度
							curr_status = get_curr_status();

							/*if (special_points.size() > 0) {
								for (int i = 0; i < special_points.size(); i++) {
									p = special_points[i];
									currdist = get_dist(curr_status.lat, curr_status.lon, p.lat, p.lon);
									if (currdist < 7) {
										curr_status1 = get_curr_status1();
										sendloc(curr_status1);
									}
								}
							}*/
							//sendloc(curr_status);  //send location to the database

							//TODO
							cnt++;
							if (cnt >= 5)
							{
								cnt = 0;
								// 计算当前位置与目标点的角度
								set_angle = get_azimuth_desired(curr_status.lat, curr_status.lon, pB.lat, pB.lon);
							}
							cout << "Going to point " << i_point + 1 << "  remaining points:" << n_point - i_point << endl;
							cout << "desired angle = " << set_angle << "  current angle=" << curr_status.angle << endl;
							insert_point(i_point + 1,0);
							//Sleep(500);
							lastdist = currdist;
							// 计算当前位置与目标点之间的距离
							currdist = get_dist(curr_status.lat, curr_status.lon, pB.lat, pB.lon);
							if (count == 0) {
								dis1 = currdist;
							}
							else if (count == 5) {
								dis2 = currdist;
								if (dis2 - dis1 < 1) {   //连续5次小车记录位置差小于1m
									//sendAlarm(curr_status);      //触发报警
								}
								count = 0;
							}
							count++;
							cout << "current dist from goal: " << currdist << endl;
							// reach desired position
							if (currdist < threshold_dist) //判断小车是否到达目标点
							{
								count = 0;
								if (i_point == n_point - 1) //判断是否为最后一个目标点，为最后一个目标点则结束自动行驶
								{
									ctrl_c_pressed = true;
									Sleep(500);
									switch_to_stop();
									sendword(); //到达最后一个点，调用后台生成报告的方法
									mv_flag = FALSE;
									auto_flag = FALSE;
								}
								cout << "reach point " << i_point + 1<< endl;
								changestatus = true;
								i_point++;
								curr_status = get_curr_status();
								pA.lat = curr_status.lat;
								pA.lon = curr_status.lon;
								pA.n_point = n_point;
								break;
							}
							// PID processing
							// error为小车当前行驶角度与期望角度（小车当前位置与目标点的北偏角度）的差值
							error = curr_status.angle - set_angle;
							if (error > 180)
								error = error - 360;
							if (error < -180)
								error = 360 + error;
							//error1 = e
							// 计算小车应调整的角度，并把最大转动角度控制为40
							out_angle = min(max(pid.process(error), float(-40.0)), float(40.0));
							if (out_angle < 0) {
								cout << "turning LEFT!!!!" << endl;
								pid_cmd = "turn left";
							}
							else {
								cout << "turning RIGHT" << endl;
								pid_cmd = "turn right";
							}
							/*outfile << curr_status.lat << ' ' << curr_status.lon << ' '
								<< curr_status.angle << ' ' << set_angle << ' ' << error << ' ' << pid_1 << ' ' << endl;*/
							fmu.lock();
							if (auto_flag)
							{
								// 将要调整的角度映射给小车控制板
								f_ctl.angle = 127 + int(out_angle * 127 / 45);
								serial_con.WriteFrame(f_ctl); //给小车的控制板发送命令
							}
							fmu.unlock();
							Sleep(500);
							// 输出角度信息
							outfile << curr_status.lat << ' ' << curr_status.lon << ' '
								<< curr_status.angle << ' ' << set_angle << ' ' << error << ' ' << out_angle << " " << pid_cmd << ' ' << int(f_ctl.angle) << ' ' << endl;
							t1 = t2;
							if (mv_flag == false)
								break;
						}
					}
					else {
						//Sleep(500);
						if (mv_flag == FALSE)
							break;
					}

				}
				if (mv_flag == FALSE)
					break;
			}
			i_route++;
			if (mv_flag == FALSE)
				break;
		}
	}
	outfile.close();
}

//给小车控制板发送命令
void ForwardControl::send_cmd(void)
{
	int cnt = 0;
	while (auto_flag)
	{
		cnt++;
		if (cnt == 0xff)
			cnt = 0;
		f_ctl.cnt = cnt;
		serial_con.WriteFrame(f_ctl);
		//cout << "Command given: speed_h" << (int) f_ctl.speed_h << (int) f_ctl.speed_l << endl;
		Sleep(300);
	}
	serial_con.WriteFrame(f_ctl);
}

// 将gps传感器的经纬度格式转换成实际的经纬度
double ForwardControl::gps_m2d(string m, const char* s)
{
	double d = 0.0;
	double tmp;
	double a, b;

	tmp = pow(10, m.length() - m.find(".", sizeof('.')) - 1);
	//cout << typeid(stoi(m.substr(m.find(".", sizeof('.')) + 1, m.length())) / tmp).name() << endl;
	if (s == "route")
	{
		a = double(stoi(m.substr(0, m.find(".", sizeof('.')))));
		b = stod(m.substr(m.find(".", sizeof('.')) + 1, m.length())) / tmp;
		d = a + b;
	}
	else if (s == "status")
		d = double(stoi(m.substr(0, m.find(".", sizeof('.')) - 2))) + stod(m.substr(m.find(".", sizeof('.')) - 2, m.length())) / 60;
	return d;
}

// get the setpoint (angle) for PID from two given positions
// 根据两个点的经纬度，计算两点之间的夹角，以北偏角显示
float ForwardControl::get_azimuth_desired(double lat1, double long1, double lat2, double long2)
{
	// float ang = atan2((long2 - long1), (lat2 - lat1))*180/PI;
	float ang = 0.0;
	double rlat1, rlat2, rlon1, rlon2;
	rlat1 = lat1 * PI / 180;
	rlat2 = lat2 * PI / 180;
	rlon1 = long1 * PI / 180;
	rlon2 = long2 * PI / 180;

	double numerator = sin(rlon2 - rlon1) * cos(rlat2);
	double denominator = cos(rlat1) * sin(rlat2) - sin(rlat1) * cos(rlat2) * cos(rlon2 - rlon1);
	double x = atan2(abs(numerator), abs(denominator));

	if (long2 > long1) // right quadrant
	{
		if (lat2 > lat1) // first quadrant
			ang = x;
		else if (lat2 < lat1) // forth quadrant
			ang = PI - x;
		else
			ang = PI / 2; // in positive-x axis
	}
	else if (long2 < long1) // left quadrant
	{
		if (lat2 > lat1) // second quadrant
			ang = 2 * PI - x;
		else if (lat2 < lat1) // third quadrant
			ang = PI + x;
		else
			ang = PI * 3 / 2; // in negative-x axis
	}

	return ang * 180 / PI;
}

// calculate distance of two point
// 计算两点之间的距离
double ForwardControl::get_dist(double lat1, double lon1, double lat2, double lon2)
{
	double rlat1, rlat2, rlon1, rlon2;// , a, b, s;
	rlat1 = lat1 * PI / 180;
	rlat2 = lat2 * PI / 180;
	rlon1 = lon1 * PI / 180;
	rlon2 = lon2 * PI / 180;


	double havLat = sin((rlat1 - rlat2) / 2);
	double havLon = sin((rlon1 - rlon2) / 2);

	double a = havLat * havLat + cos(rlat1) * cos(rlat2) * havLon * havLon;

	return 2 * 6371000 * atan2(sqrt(a), sqrt(1 - a));

	//return s;
}

void ForwardControl::insert_point(int point,int flag)
{

	bson_error_t error;
	bson_t *doc = bson_new();
	BSON_APPEND_INT64(doc, "id", point);
	BSON_APPEND_INT64(doc, "flag", flag);
	//BSON_APPEND_INT64(doc, "field1", 0);
	//string msg = "test message";
	//BSON_APPEND_BINARY(doc, "field2", BSON_SUBTYPE_BINARY, (const uint8_t*)(msg.c_str()), (uint32_t)(msg.size()));

	bool r = mongoc_collection_insert(col_route, MONGOC_INSERT_NONE, doc, NULL, &error);
	if (!r)
	{
		cout << "Insert Failure:" << error.message;
	}
	bson_destroy(doc);
}

// 激光雷达的部分
void ForwardControl::rplidar()
{
	//ofstream outFile;
	//outFile.open("D:\\10241.txt");

	int k;
	int fre=1;
	double obstacle_angle;
	double obstacle_dist;

	const char * opt_com_path = NULL;
	_u32         opt_com_baudrate = 256000;
	u_result     op_result;

	bool useArgcBaudrate = true;
	bool obstacle;


	if (!opt_com_path) {
#ifdef _WIN32
		// use default com port
		// 默认端口（Windows系统）
		opt_com_path = "\\\\.\\com11";
#else	// Linus系统
		opt_com_path = "/dev/ttyUSB0";
#endif
	}

	// create the driver instance
	RPlidarDriver * drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
	if (!drv) {
		fprintf(stderr, "insufficent memory, exit\n");
		exit(-2);
	}

	rplidar_response_device_info_t devinfo;
	bool connectSuccess = false;
	// make connection...

	if (!drv)
		drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
	// 连接激光雷达
	if (IS_OK(drv->connect(opt_com_path, opt_com_baudrate)))
	{
		op_result = drv->getDeviceInfo(devinfo);

		if (IS_OK(op_result))
		{
			connectSuccess = true;
		}
		else
		{
			delete drv;
			drv = NULL;
		}
	}


	if (!connectSuccess) {

		fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
			, opt_com_path);
		goto on_finished;
	}

	// 启动电机
	drv->startMotor();
	// start scan...
	drv->startScan(0, 1);

	// fetech result and print it out...
	while (1) {
		obstacle = false;

		rplidar_response_measurement_node_t nodes[360];
		size_t   count = _countof(nodes);

		op_result = drv->grabScanData(nodes, count);

		if (IS_OK(op_result)) {
			drv->ascendScanData(nodes, count);
			for (int pos = 0; pos < (int)count; ++pos) {
				/*printf("%s theta: %03.2f Dist: %08.2f Q: %d \n",
					(nodes[pos].sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) ? "S " : "  ",
					(nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f,
					nodes[pos].distance_q2 / 4.0f,
					nodes[pos].sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);*/
				obstacle_angle = (nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f; //获取障碍物的角度信息
				obstacle_dist = nodes[pos].distance_q2 / 4.0f; //获取障碍物的距离
				// 障碍物的判定范围
				if (obstacle_angle > 340 || obstacle_angle < 50)
				{
					if (obstacle_dist != 0 && obstacle_dist < 1000)
					{
						obstacle = true;
						switch_to_stop();
						cout << "obstacle_angle: " << obstacle_angle << "  obstacle_dist: " << obstacle_dist << endl;
						break;
					}
				}
				//ctrl_c_pressed = true;
				//outFile << "theta:  " << (nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f << "  Dist:  " << nodes[pos].distance_q2 / 4.0f << endl;
			}
		}

		if (mv_flag==FALSE) {
			break;
		}

		if (obstacle==true) {
			fre += 1;
			//f_ctl.is_ctl = 0x00;
			//switch_to_stop();
		}
		else {
			fre = 1;
			flag = 0;
			//insert_point(1, 0);
			//f_ctl.angle = 0x7f;
			int speed = 350;
			f_ctl.speed_h = (speed + 500) / 255;
			f_ctl.speed_l = (speed + 500) - f_ctl.speed_h * 255;
			f_ctl.is_ctl = 0x01;
		}
		if (fre > 10) {
			flag = 1;
			//insert_point(1, 1);
		}
		if (fre >= 300)
		{
			mv_flag = FALSE;
			ctrl_c_pressed = true;
		}

	}

	drv->stop();

	drv->stopMotor();
	cout << "finish";

on_finished:
	RPlidarDriver::DisposeDriver(drv);
	drv = NULL;
}

void ForwardControl::test1(string str)
{

	string s1 = str.substr(0, 1);
	string s2 = str.substr(1, str.length());
	cout << "s1=" << s1 << ",s2=" << s2 << endl;
	double d = stod(s2);
	cout << d;
	//Sleep(d * 1000);
	if (s1 == "L") {
		test7();
	}
	else if (s1 == "R") {
		test8();
	}
	else if (s1 == "S") {
		test9();
	}
	else if (s1 == "D") {
		test6();
	}
	else if (s1 == "B") {
		test10();
	}
	Sleep(d * 1000);
}

//void ForwardControl::test2(void)
//{
//	float t_run, t_sample;
//	float kp, ki, kd;
//	int speed;
//	string lat, lon;// , lat1, lon1;
//	freopen("C:\\Users\\Administrator\\Desktop\\UGV\\pid_v4\\pid_v4\\points.txt", "r", stdin);
//
//	speed = 350;
//	f_ctl.speed_h = (speed + 500) / 255;
//	f_ctl.speed_l = (speed + 500) - f_ctl.speed_h * 255;
//	cout << "set distance threshold (m): " << endl;
//	cin >> threshold_dist;
//
//	int n_point;
//	cin >> n_point;
//	cout << "Input " << n_point << "target point(s) please." << endl;
//	cout.precision(15);
//
//	routePoint tmp_point;
//	for (int i = 0; i < n_point; ++i) {
//		cout << "input point No." << i << "\'s latitude: ";
//		//cin >> desired_point.lat
//		cin >> lat;
//		tmp_point.lat = gps_m2d(lat, "status");
//		cout << "input point No." << i << "\'s longitude: ";
//		//cin >> desired_point.lon;
//		cin >> lon;
//		tmp_point.lon = gps_m2d(lon, "status");
//		tmp_point.n_point = n_point;
//
//		desired_points.push_back(tmp_point);
//	}
//
//	set_pid_sample_t(1.0);
//	set_pid_gains(0.7, 0.0, 0.0);
//	mv_flag = TRUE;
//	start_move_thread();
//}



bool FileExistsStatus(const CHAR* path)
{
	DWORD dwAttribute = GetFileAttributes(path);
	if (dwAttribute == 0XFFFFFFFF) return false; //0XFFFFFFFF表示文件不存在
	else return true;
}

bool DownloadFiles(const CHAR* url, const CHAR* downloadPath)
{
	DeleteUrlCacheEntry(url);
	if (URLDownloadToFile(NULL, url, downloadPath, 0, 0) == S_OK && FileExistsStatus(downloadPath)) return true;
	else return false;
}

// 将字符串以“，”为分隔符进行分开
void Split(string source, string divKey, vector<string>& dest)

{

	dest.clear();
	int end = 0;
	int start = 0;

	while (-1 != end) {
		end = source.find(divKey, start);

		dest.push_back(source.substr(start, (end - start)));

		start = end + 1;

	}

}

void ForwardControl::download_special()
{
	string special_path = "C:\\Users\\Administrator\\Desktop\\special_points.csv";
	string ip_tail = "101";
	if (DownloadFiles(("http://192.168.1." + ip_tail + ":8484/ocean/download_special").c_str(), special_path.c_str())) {
		cout << "special point file download success" << endl;
		special_points.clear();
		ifstream specialfile(special_path);
		routePoint tmp_point;
		string temp_line;
		vector<string> items;
		specialfile >> temp_line;
		while (specialfile >> temp_line) {
			Split(temp_line, ",", items);
			tmp_point.lat = gps_m2d(items[1], "status");
			tmp_point.lon = gps_m2d(items[2], "status");
			tmp_point.n_point = 0;
			special_points.push_back(tmp_point);
		}
		specialfile.close();
		/*int size = special_points.size();
		routePoint p;
		cout << "size=" << size;
		for (int i = 0; i < size; i++) {
			p = special_points[i];
			cout << "lat=" << p.lat << ",lon=" << p.lon << endl;
		}*/		
	}
	else {
		special_points.clear();
		cout << "special point file download fail" << endl;
	}
}


// 从服务器下载路径文件并开始自动行驶
void ForwardControl::test5(void)
{
	if (mv_flag == FALSE) {

		float t_run, t_sample;
		float kp, ki, kd;
		int speed;
		string lat, lon;// , lat1, lon1;
		// 路径文件的保存路径
		string target_path_file = "C:\\Users\\Administrator\\Desktop\\points.csv";

		string ip_tail = "101";
		//cin >> ip_tail;
		cout << target_path_file.c_str() << endl;
		// 从服务器下载文件
		if (DownloadFiles(("http://192.168.1." + ip_tail + ":8484/ocean/download").c_str(), target_path_file.c_str()))
			//if (DownloadFiles(("http://22.22.12." + ip_tail + ":8484/ocean/download").c_str(), target_path_file.c_str()))
		{
			cout << "Path Download SUCCEEDED." << endl;
		}
		else
		{
			cout << "Download FAIL." << endl;
			return;
		}

		ifstream datafile(target_path_file);
		speed = 350;
		f_ctl.speed_h = (speed + 500) / 255;
		f_ctl.speed_l = (speed + 500) - f_ctl.speed_h * 255;
		cout << "set distance threshold (m): " << endl;
		threshold_dist = 2;

		cout.precision(15);
		desired_points.clear();
		vector<string> items;
		string tmp_input_line;
		datafile >> tmp_input_line;

		routePoint tmp_point;
		int i = 0;
		// 读取路径文件的路径点并将其存储到desired_points中
		while (datafile >> tmp_input_line)
		{
			//cin >> tmp_input_line;
			cout << tmp_input_line << endl;
			// 将读取的一行数据以"，"分隔开
			Split(tmp_input_line, ",", items);

			tmp_point.lat = gps_m2d(items[1], "status");

			tmp_point.lon = gps_m2d(items[2], "status");
			cout << "Point No." << i << ": " << tmp_point.lat << " " << tmp_point.lon << endl;
			tmp_point.n_point = 0;

			desired_points.push_back(tmp_point);
			++i;
		}
		datafile.close();
		//download_special();
		set_pid_sample_t(1.0);
		set_pid_gains(0.7, 0.0, 0.1);
		mv_flag = TRUE;
		auto_flag = TRUE;
		start_move_thread();
	}
	else if (mv_flag == TRUE && auto_flag == FALSE)
	{
		cout << "continue automatic forward";
		f_ctl.is_ctl = 0x01;
		auto_flag = TRUE;
		int speed = 350;
		f_ctl.speed_h = (speed + 500) / 255;
		f_ctl.speed_l = (speed + 500) - f_ctl.speed_h * 255;
			
	}
}

//void ForwardControl::test5(void)
//{
//	if (mv_flag==TRUE) {
//		mv_flag = FALSE;
//		auto_flag = FALSE;
//		Sleep(700);
//		cout << "stop last auto" << endl;
//	}//
//		time_t t0 = time(NULL);
//		char *t1 = ctime(&t0);
//		cout << "M" << " auto pilot " << t1;
//		float t_run, t_sample;
//		float kp, ki, kd;
//		int speed;
//		string lat, lon;// , lat1, lon1;
//		string target_path_file = "C:\\Users\\Administrator\\Desktop\\1027.csv";
//		//char *file = "C:\\Users\\Administrator\\Desktop\\1027.csv";
//		cout << "Input IP tail of the server: (100~103)" << endl;
//		string ip_tail = "95";
//		//cin >> ip_tail;
//		cout << target_path_file.c_str() << endl;
//		download_special();
//
//		if (DownloadFiles(("http://192.168.43." + ip_tail + ":8484/ocean/download").c_str(), target_path_file.c_str()))
//			//if (DownloadFiles(("http://22.22.12." + ip_tail + ":8484/ocean/download").c_str(), target_path_file.c_str()))
//		{
//			cout << "Path Download SUCCEEDED." << endl;
//		}
//		else
//		{
//			cout << "Download FAIL." << endl;
//			return;
//		}
//		//freopen(target_path_file.c_str(), "r", stdin);
//		ifstream datafile(target_path_file);
//		speed = 350;
//		f_ctl.speed_h = (speed + 500) / 255;
//		f_ctl.speed_l = (speed + 500) - f_ctl.speed_h * 255;
//		cout << "set distance threshold (m): " << endl;
//		threshold_dist = 2;
//		cout.precision(15);
//		desired_points.clear();
//		vector<string> items;
//		string tmp_input_line;
//		datafile >> tmp_input_line;
//
//		routePoint tmp_point;
//		int i = 0;
//		while (datafile >> tmp_input_line)
//		{
//			//cin >> tmp_input_line;
//			cout << tmp_input_line << endl;
//			Split(tmp_input_line, ",", items);
//			//cin >> desired_point.lat
//			//cin >> lat;
//			tmp_point.lat = gps_m2d(items[1], "status");
//
//			//cin >> desired_point.lon;
//			//cin >> lon;
//			tmp_point.lon = gps_m2d(items[2], "status");
//			cout << "Point No." << i << ": " << tmp_point.lat << " " << tmp_point.lon << endl;
//			tmp_point.n_point = 0;
//
//			desired_points.push_back(tmp_point);
//			++i;
//		}
//		datafile.close();
//		//download_special();
//		set_pid_sample_t(1.0);
//		set_pid_gains(0.7, 0.0, 0.1);
//		mv_flag = TRUE;
//		auto_flag = TRUE;
//		//freopen("CON", "r", stdin);
//		start_move_thread();
//	
//	
//}

// 操纵小车直行
void ForwardControl::test6(void)
{
	time_t t0 = time(NULL);
	char *t1 = ctime(&t0);
	cout << "D" << " move forward " << t1;
	float t_run, t_sample;
	float kp, ki, kd;
	int speed;
	//fmu.lock();
	auto_flag = FALSE;
	//fmu.unlock();
	f_ctl.is_ctl = 0x01;
	//f_ctl.angle = 127 + int(45 * 127 / 45);
	f_ctl.angle = 0x7f;
	//f_ctl.speed_h = 1;
	//f_ctl.speed_l = 0xf4;
	speed = 350;
	f_ctl.speed_h = (speed + 500) / 255;
	f_ctl.speed_l = (speed + 500) - f_ctl.speed_h * 255;
	//Sleep(500);

	send_cmd();
	cout << int(f_ctl.speed_h) << endl;
	cout << int(f_ctl.speed_l) << endl;
	//sendpid("D");
}

// 左转
void ForwardControl::test7(void)
{
	time_t t0 = time(NULL);
	char *t1 = ctime(&t0);
	SYSTEMTIME sys;
	GetLocalTime(&sys);
	cout << sys.wHour << ":" << sys.wMinute << ":" << sys.wSecond << ":" << sys.wMilliseconds << endl;
	cout << "L" << " move left " << t1;
	float t_run, t_sample;
	float kp, ki, kd;
	int speed;
	fmu.lock();
	auto_flag = FALSE;
	fmu.unlock();
	f_ctl.is_ctl = 0x01;
	f_ctl.angle = 127 - int(20 * 127 / 45);
	//f_ctl.angle = 0x7f;
	//f_ctl.speed_h = 1;
	//f_ctl.speed_l = 0xf4;
	speed = 350;
	f_ctl.speed_h = (speed + 500) / 255;
	f_ctl.speed_l = (speed + 500) - f_ctl.speed_h * 255;
	//Sleep(500);


	send_cmd();
	cout << int(f_ctl.speed_h) << endl;
	cout << int(f_ctl.speed_l) << endl;
	//sendpid("L");
	//send_cmd();
}

// 右转
void ForwardControl::test8(void)
{
	time_t t0 = time(NULL);
	char *t1 = ctime(&t0);
	cout << "R" << " move right " << t1;
	float t_run, t_sample;
	float kp, ki, kd;
	int speed;
	fmu.lock();
	auto_flag = FALSE;
	fmu.unlock();
	f_ctl.is_ctl = 0x01;
	f_ctl.angle = 127 + int(20 * 127 / 45);
	//f_ctl.angle = 0x7f;
	//f_ctl.speed_h = 1;
	//f_ctl.speed_l = 0xf4;
	speed = 350;
	f_ctl.speed_h = (speed + 500) / 255;
	f_ctl.speed_l = (speed + 500) - f_ctl.speed_h * 255;
	//Sleep(500);

	send_cmd();
	//sendpid("R");
}

// 停止
void ForwardControl::test9(void)
{
	time_t t0 = time(NULL);
	char *t1 = ctime(&t0);
	cout << "S" << " stop " << t1;
	SYSTEMTIME sys;
	GetLocalTime(&sys);
	cout << sys.wHour << ":" << sys.wMinute << ":" << sys.wSecond << ":" << sys.wMilliseconds << endl;
	float t_run, t_sample;
	float kp, ki, kd;
	int speed;
	fmu.lock();
	auto_flag = FALSE;
	fmu.unlock();
	f_ctl.is_ctl = 0x00;
	//f_ctl.angle = 127 + int(30 * 127 / 45);
	f_ctl.angle = 0x7f;
	//f_ctl.speed_h = 1;
	//f_ctl.speed_l = 0xf4;
	speed = 0;
	f_ctl.speed_h = (speed + 500) / 255;
	f_ctl.speed_l = (speed + 500) - f_ctl.speed_h * 255;
	//Sleep(500);

	send_cmd();
	//sendpid("S");
}

// 后退
void ForwardControl::test10(void)
{
	time_t t0 = time(NULL);
	char *t1 = ctime(&t0);
	cout << "B" << " move back " << t1;
	float t_run, t_sample;
	float kp, ki, kd;
	int speed;
	fmu.lock();
	auto_flag = FALSE;
	fmu.unlock();
	f_ctl.is_ctl = 0x01;
	//f_ctl.angle = 127 + int(30 * 127 / 45);
	f_ctl.angle = 0x7f;
	//f_ctl.speed_h = 1;
	//f_ctl.speed_l = 0xf4;
	speed = -300;
	f_ctl.speed_h = (speed + 500) / 255;
	f_ctl.speed_l = (speed + 500) - f_ctl.speed_h * 255;
	//Sleep(500);

	send_cmd();
	cout << "speed_h=" << int(f_ctl.speed_h) << " speed_l=" << int(f_ctl.speed_l) << endl;
	//sendpid("B");
}

//void ForwardControl::test11(void) {
//	while (true) {
//		currStatus1 curr_status;
//		curr_status = get_curr_status1();
//		cout << curr_status.lat << endl;
//		cout << curr_status.lon << endl;
//		sendloc(curr_status);
//		Sleep(500);
//	}
//}


// 接收服务器下发的10个顺序执行的命令，并先依次执行优先级较高的命令，继而执行普通优先级的命令
void ForwardControl::test12(string str) {
	queue<char> q;
	str = str.substr(0, str.length() - 1);
	cout << "str=" << str.length();
	string str1 = "";
	string str2 = "";
	vector<string> items;
	char c;
	int speed;
	Split(str, ",", items);
	for (int i = 0; i < items.size(); i++) {
		if (items[i].length() > 1)
			str2 += items[i].substr(0, 1);
		else
			str1 += items[i];
	}
	string str3 = str2 + str1;
	cout << str3 << endl;
	for (int i = 0; i < str3.length(); i++) {
		q.push(str3[i]);
	}
	int len = q.size();
	for (int i = 0; i < str3.length(); i++) {
		c = q.front();
		if (c == 'L') {
			test7();
		}
		else if (c == 'R') {
			test8();
		}
		else if (c == 'S') {
			test9();
		}
		else if (c == 'D') {
			test6();;
		}
		else if (c == 'B') {
			test10();
		}
		Sleep(2000);
		q.pop();
	}
}


// 左转
void ForwardControl::test13(void)
{
	time_t t0 = time(NULL);
	char *t1 = ctime(&t0);
	SYSTEMTIME sys;
	GetLocalTime(&sys);
	cout << sys.wHour << ":" << sys.wMinute << ":" << sys.wSecond << ":" << sys.wMilliseconds << endl;
	cout << "L" << " move left " << t1;
	float t_run, t_sample;
	float kp, ki, kd;
	int speed;
	fmu.lock();
	auto_flag = FALSE;
	fmu.unlock();
	f_ctl.is_ctl = 0x01;
	f_ctl.angle = 127 - int(20 * 127 / 45);
	//f_ctl.angle = 0x7f;
	//f_ctl.speed_h = 1;
	//f_ctl.speed_l = 0xf4;
	speed = 0;
	f_ctl.speed_h = (speed + 500) / 255;
	f_ctl.speed_l = (speed + 500) - f_ctl.speed_h * 255;
	//Sleep(500);


	send_cmd();
	cout << int(f_ctl.speed_h) << endl;
	cout << int(f_ctl.speed_l) << endl;
	//sendpid("L");
	//send_cmd();
}

// 右转
void ForwardControl::test14(void)
{
	time_t t0 = time(NULL);
	char *t1 = ctime(&t0);
	cout << "R" << " move right " << t1;
	float t_run, t_sample;
	float kp, ki, kd;
	int speed;
	fmu.lock();
	auto_flag = FALSE;
	fmu.unlock();
	f_ctl.is_ctl = 0x01;
	f_ctl.angle = 127 + int(20 * 127 / 45);
	//f_ctl.angle = 0x7f;
	//f_ctl.speed_h = 1;
	//f_ctl.speed_l = 0xf4;
	speed = 0;
	f_ctl.speed_h = (speed + 500) / 255;
	f_ctl.speed_l = (speed + 500) - f_ctl.speed_h * 255;
	//Sleep(500);

	send_cmd();
	//sendpid("R");
}

void ForwardControl::manual_drive(int x,int y,int z)         //接收方向盘数据并驱动小车行驶
{
	time_t t0 = time(NULL);
	char *t1 = ctime(&t0);
	cout << "R" << " move right " << t1;
	float t_run, t_sample;
	float kp, ki, kd;
	int speed;
	fmu.lock();
	auto_flag = FALSE;
	fmu.unlock();
	f_ctl.is_ctl = 0x01;
	f_ctl.angle = 127 + int(x * 80 / 65534 * 127 / 45);  //方向盘选择角度值为-32767~32767,需要将其映射到-43~43
	//f_ctl.angle = 0x7f;
	//f_ctl.speed_h = 1;
	//f_ctl.speed_l = 0xf4;
	if (y == 32767 && z != 32767) //刹车踩下的情况
	{
		speed = int((500 * z / 65534) - 250);           //油门值为-32767~32767 将其映射到0~-500
	}

	if (z == 32767 && y != 32767) //油门踩下的情况
	{
		speed = int(-(500 * y / 65534) + 250);            //油门值为-32767~32767 将其映射到0~500
	}

	if (z == 32767 && y == 32767) //油门刹车都不踩下
	{
		speed = 0;
	}

	if (z != 32767 && y != 32767) //两个都踩下的情况
	{
		speed = 0;
	}
	cout << "speed = " << speed << endl;
	//speed = 350;
	f_ctl.speed_h = (speed + 500) / 255;
	f_ctl.speed_l = (speed + 500) - f_ctl.speed_h * 255;
	//Sleep(500);

	send_cmd();
	//sendpid("R");
}