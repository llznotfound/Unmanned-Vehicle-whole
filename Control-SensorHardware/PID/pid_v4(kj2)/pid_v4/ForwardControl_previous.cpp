#include "ForwardControl.h"

#define PI 3.14159265358979323846

using namespace std;

ForwardControl::ForwardControl(int port_num, const char* db_name, const char* col_real_time_name, const char* col_ang_name, const char* col_route_name)
{
	serial_con.InitPort(port_num);
	f_ctl.is_ctl = 0x01;
	f_ctl.speed_h = 0x03;
	f_ctl.speed_l = 0x84;

	// init db
	mongoc_init();		
	client = mongoc_client_new("mongodb://localhost:27017");
	database = mongoc_client_get_database(client, db_name);
	col_status= mongoc_client_get_collection(client, db_name, col_real_time_name);
	col_angle= mongoc_client_get_collection(client, db_name, col_ang_name);
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
routePoint ForwardControl::get_route(int n_route, int n_point)
{
	string lat, lon;
	bson_t *filter;
	bson_t *opts;
	mongoc_cursor_t *cursor;
	const bson_t *doc;
	char *str = NULL; 
	routePoint point;

	filter = BCON_NEW("plan_id", BCON_INT64(n_route) , "waypoint_no", BCON_INT64(n_point));
	opts = BCON_NEW("limit", BCON_INT64(1), "sort", "{", "time", BCON_INT32(-1), "}");
	cursor = mongoc_collection_find_with_opts(col_route, filter, opts, NULL);

	if (mongoc_cursor_next(cursor, &doc)) {
		str = bson_as_json(doc, NULL);
		Json::CharReaderBuilder builder;
		Json::CharReader *reader = builder.newCharReader();
		Json::Value info;
		string errors;
		if (reader->parse(str, str + strlen(str), &info, &errors))
		{
			point.lat = gps_m2d(info["lat"].asCString(), "route");
			point.lon = gps_m2d(info["lng"].asCString(), "route");
			point.n_point = info["waypoint_sum"].asInt64();
		}
		bson_free(str);
	}
	return point;
}

currStatus ForwardControl::get_curr_status()
{
	bson_t *filter;
	bson_t *opts;
	mongoc_cursor_t *cursor;
	const bson_t *doc;
	char *str = NULL;
	currStatus status;

	filter = BCON_NEW();
	opts = BCON_NEW("limit", BCON_INT64(1), "sort", "{", "time", BCON_INT32(-1), "}");
	cursor = mongoc_collection_find_with_opts(col_status, filter, opts, NULL);

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
	opts = BCON_NEW("limit", BCON_INT64(1), "sort", "{", "time", BCON_INT32(-1), "}");
	cursor = mongoc_collection_find_with_opts(col_angle, filter, opts, NULL);

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
	return status;
}

/*float ForwardControl::get_curr_angle()
{
	bson_t *filter;
	bson_t *opts;
	mongoc_cursor_t *cursor;
	const bson_t *doc;
	char *str = NULL;
	filter = BCON_NEW();
	opts = BCON_NEW("limit", BCON_INT64(1), "sort", "{", "time", BCON_INT32(-1), "}");
	cursor = mongoc_collection_find_with_opts(col_angle, filter, opts, NULL);

	if (mongoc_cursor_next(cursor, &doc)) {
		str = bson_as_json(doc, NULL);
		Json::CharReaderBuilder builder;
		Json::CharReader *reader = builder.newCharReader();
		Json::Value info;
		string errors;
		if (reader->parse(str, str + strlen(str), &info, &errors))
		{
			return info["angle_z"].asFloat();
		}
		bson_free(str);
	}
}*/

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

bool ForwardControl::start_move_thread(void)
{
	mvThread = thread(&ForwardControl::move, this);
	f_ctl.is_ctl = 0x01;
	Sleep(500);
	sendCmdThread = thread(&ForwardControl::send_cmd, this);
}

// main thread of moving forward
void ForwardControl::move(void)
{
	double out_angle, error, dist;
	currStatus curr_status;
	int i_route = 0, i_point = 0, n_point = 0;
	routePoint pA, pB;
	time_t t0, t1, t2;
	int speed;

	/*while (1)
	{
		if (mv_flag = FALSE)
			continue;
		t0 = time(NULL);
		t1 = t0;
		while (1)
		{
			t2 = time(NULL);
			if (t2 - t0 > t_test) //TODO
			{
				f_ctl.is_ctl = 0x00;
				f_ctl.angle = 0x7f;
				break;
			}
			if (t2 - t1 < t_sam)
				continue;
			else
			{
				// get current angle from db
				cur_angle = get_curr_angle();
				// PID processing
				if (abs(cur_angle - set_angle) < 180)
					error = cur_angle - set_angle;
				else if (cur_angle < set_angle)
					error = 360 - abs(cur_angle - set_angle);
				else
					error = abs(cur_angle - set_angle) - 360;
				out_angle = -min(max(pid.process(error), float(-45.0)), float(45.0));
				fmu.lock();
				f_ctl.angle = 127 + int(out_angle * 127 / 45);
				fmu.unlock();
				t1 = t2;
			}
		}
	}*/

	while (1)
	{
		if (mv_flag = FALSE)
			continue;
		i_route = 0;
		i_point = 0;
		n_point = 0;
		while (1)
		{
			pA = get_route(i_route, i_point);
			if (pA.n_point == 0)
				continue;
			else
			{
				n_point = pA.n_point;
				for (int i = 1; i <= n_point; i++)
				{
					// get point B and calculate the setpoint
					pB = get_route(i_route, i_point + 1);
					while (pB.n_point == 0)
						pB = get_route(i_route, i_point + 1);
					set_angle = get_azimuth_desired(pA.lat, pA.lon, pB.lat, pB.lon);
					// PID controller
					t0 = time(NULL);
					t1 = t0;
					while (1)
					{
						t2 = time(NULL);
						if (t2 - t1 < t_sam)
							continue;
						else
						{
							// get current position from db
							curr_status = get_curr_status();
							dist = get_dist(curr_status.lat, curr_status.lon, pB.lat, pB.lon);
							cout << "current dist from goal: " << dist << endl;
							// reach desired position
							if (dist < threshold_dist)
							{
								//f_ctl.is_ctl = 0x00;
								//f_ctl.angle = 0x7f;
								//mv_flag = FALSE;
								i_point++;
								pA.lat = curr_status.lat;
								pA.lon = curr_status.lon;
								pA.n_point = n_point;
								break;
							}
							// PID processing
							error = curr_status.angle - set_angle;
							out_angle = -min(max(pid.process(error), float(-45.0)), float(45.0));
							fmu.lock();
							f_ctl.angle = 127 + int(out_angle * 127 / 45);
							fmu.unlock();
							t1 = t2;
						}
					}
					//if (mv_flag = FALSE)
					//	break;
				}
				i_route++;
			}
		}
	}
}

void ForwardControl::send_cmd(void)
{
	int cnt = 0;
	while (1)
	{
		cnt++;
		if (cnt == 0xff)
			cnt = 0;
		f_ctl.cnt = cnt;
		serial_con.WriteFrame(f_ctl);
		Sleep(300);
	}
}

double ForwardControl::gps_m2d(string m, const char* s)
{
	double d = 0.0, tmp = 10000.0;
	if (s == "route")
		d = stoi(m.substr(0, m.find(".", sizeof('.')))) + stoi(m.substr(m.find(".", sizeof('.')) + 1, m.length())) / tmp;
	else if (s == "status")
		d = stoi(m.substr(0, m.find(".", sizeof('.')) - 2)) + stod(m.substr(m.find(".", sizeof('.')) - 2, m.length())) / 60;
	return d;
}

// get the setpoint (angle) for PID from two given positions
float ForwardControl::get_azimuth_desired(double lat1, double long1, double lat2, double long2)
{
	// float ang = atan2((long2 - long1), (lat2 - lat1))*180/PI;
	float ang = 0.0;
	/*
	int int_lat1, int_lat2, int_lon1, int_lon2;
	int_lat1 = stoi(lat1.substr(0, lat1.find(".", sizeof('.')))) * 1000000 + stoi(lat1.substr(lat1.find(".", sizeof('.')) + 1, lat1.length()));
	int_lat2 = stoi(lat1.substr(0, lat2.find(".", sizeof('.')))) * 1000000 + stoi(lat2.substr(lat2.find(".", sizeof('.')) + 1, lat2.length()));
	int_lon1 = stoi(long1.substr(0, long1.find(".", sizeof('.')))) * 1000000 + stoi(long1.substr(long1.find(".", sizeof('.')) + 1, long1.length()));
	int_lon2 = stoi(long2.substr(0, long2.find(".", sizeof('.')))) * 1000000 + stoi(long2.substr(long2.find(".", sizeof('.')) + 1, long2.length()));

	ang = atan2(int_lon1 - int_lon2, int_lat1 - int_lat2) * 180 / PI;
	cout << "ang: " << ang<<endl;
	if (ang < 0)
		ang = -ang;
	else
		ang = 360 - ang;*/
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
			ang= PI - x;
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

	return 360 - ang * 180 / PI;
}

// calculate distance of two point
double ForwardControl::get_dist(double lat1, double lon1, double lat2, double lon2)
{
	double rlat1, rlat2, rlon1, rlon2;// , a, b, s;
	rlat1 = lat1 * PI / 180;
	rlat2 = lat2 * PI / 180;
	rlon1 = lon1 * PI / 180;
	rlon2 = lon2 * PI / 180;
	/*a = rlat1 - rlat2;
	b = rlon1 - rlon2;
	s = 2 * asin(sqrt(pow(sin(a / 2), 2) + cos(rlat1)*cos(rlat2)*pow(sin(b / 2), 2)));
	s = round(s * 6378.137 * 10000) / 10 * 1.609344;*/

	double havLat = sin((rlat1 - rlat2) / 2);
	double havLon = sin((rlon1 - rlon2) / 2);

	double a = havLat * havLat + cos(rlat1) * cos(rlat2) * havLon * havLon;

	return 2 * 6371000 * atan2(sqrt(a), sqrt(1 - a));

	//return s;
}

// test getting data from database and setpoint calculating
void ForwardControl::test1()
{
	routePoint pA, pB;
	currStatus s;
	double ang = 0.0, ang_curr = 0.0;

	pA = get_route(0, 0);
	pB = get_route(0, 1);
	ang = get_azimuth_desired(pA.lat, pA.lon, pB.lat, pB.lon);
	s = get_curr_status();
	cout << "desired angle: " << ang << endl;
	cout << "current angle:" << s.angle << endl;
}

void ForwardControl::test2(void)
{
	float t_run, t_sample;
	float kp, ki, kd;
	int speed;

	cout << "set speed (0 - 500, default 350 )" << endl;
	cin >> speed;
	f_ctl.speed_h = (speed + 500) / 255;
	f_ctl.speed_l = (speed + 500) - f_ctl.speed_h * 255;
	cout << "set distance threshold (m): " << endl;
	cin >> threshold_dist;
	cout << "set PID sampling time: " << endl;
	cin >> t_sample;
	cout << "set PID gains: " << endl;
	cin >> kp >> ki >> kd;

	set_pid_sample_t(t_sample);
	set_pid_gains(kp, ki, kd);
	mv_flag = TRUE;
	start_move_thread();
}

void ForwardControl::test3(void)
{
	string lat1, lon1, lat2, lon2;
	double a, b, c, d;
	cout << "Pls input: " << endl;
	cin >> lat1 >> lon1 >> lat2 >> lon2;
	a = gps_m2d(lat1);
	b = gps_m2d(lon1);
	c = gps_m2d(lat2);
	d = gps_m2d(lon2);
	cout << a << ", " << b << ", " << c << ", " << d << endl;
	cout << "angle: "<<get_azimuth_desired(a, b, c, d) << endl;
	cout << "dist: " << get_dist(a, b, c, d) << endl;
}