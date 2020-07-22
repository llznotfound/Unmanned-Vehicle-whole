#ifndef FORWARDCONTROL_H_
#define FORWARDCONTROL_H_

#include <iostream>
#include <mutex>
#include <math.h>
#include <vector>
#include <bson.h>
#include <mongoc.h>
#include <json.h>
#include <time.h>
#include "SerialPort.h"
#include "pid.h"

using namespace std;

struct currStatus
{
	double lat = 0.0;
	double lon = 0.0;
	float angle = 0.0;
};

struct routePoint
{
	double lat = 0.0;
	double lon = 0.0;
	int n_point = 0;
};

class ForwardControl
{
public:
	ForwardControl(int port_num, const char* db_name, const char* col_real_time, const char* col_ang_time, const char* col_route);
	~ForwardControl(void);
	bool start_move_thread(void);
	bool set_pid_gains(float kp, float ki, float kd);
	bool set_pid_sample_t(float sec);
	float t_test = 30;
	void test3();
	void test1();
	void test2();

private:
	CSerialPort serial_con;
	PID pid;
	Frame f_ctl;
	mutex fmu;
	thread mvThread;
	thread sendCmdThread;
	float t_sam = 1;
	float set_angle;
	bool mv_flag = FALSE;
	float threshold_dist = 0;

	// about db
	mongoc_client_t      *client;
	mongoc_database_t    *database;
	mongoc_collection_t  *col_route;
	mongoc_collection_t  *col_angle;
	mongoc_collection_t  *col_status;

	void move(void);
	void send_cmd(void);
	float get_azimuth_desired(double lat1, double lon1, double lat2, double lon2);
	double get_dist(double lat1, double lon1, double lat2, double lon2);
	routePoint get_route(int n_route, int n_point);
	currStatus get_curr_status();
	double gps_m2d(string m, const char* s = "route");
};
#endif
