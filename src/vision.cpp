#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Vector3.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <algorithm>

#include <cmath>
#include <iostream>
#include <cv.h>
#include <highgui.h>
#include <time.h>
#include <sys/time.h>

using namespace std;
using namespace cv;

#define FIELD_WIDTH     3.53  // in meters
#define FIELD_HEIGHT    2.39 
#define ROBOT_RADIUS    0.10
#define GUI_NAME        "Soccer Overhead Camera"

// Treshold parameters
int ballColor[3];
int distThreshold;

// Handlers for vision position publishers
ros::Publisher ball_pub;

// Use variables to store position of objects. These variables are useful
// when the ball cannot be seen, otherwise we'll get the position (0, 0)
geometry_msgs::Pose2D ballPose;

vector<double> cpuTimes;
vector<double> actualTimes;

char lastKeyPressed;

Mat img, hsv, dist, distBlurred, bw, homography;

Point mouseLoc;

int blurSize = 5;
void drawPoints(Mat img);
void initPoints(Size imgSize);
void initThresholds();
bool pointsInitialized = false;
bool trackbarShown = false;

double get_wall_time()
{
	struct timeval time;
	gettimeofday(&time, NULL);
	return (double)time.tv_sec + (double)time.tv_usec * .000001;
}

// Source: stackoverflow.com/questions/17432502/how-can-i-measure-cpu-time-and-wall-clock-time-on-both-linux-windows
double get_cpu_time()
{
	return (double)clock() / CLOCKS_PER_SEC;
}

void tic()
{
	cpuTimes.push_back(get_cpu_time());
	actualTimes.push_back(get_wall_time());
}

void toc(string s = "", int count = 1, int sigFigs = 2, bool print = true)
//timeMeasurement toc(string s, int count, int sigFigs, bool print)
{
	if (cpuTimes.size() > 0)
	{
		double cpuTime_ms = (get_cpu_time() - cpuTimes.back()) * 1e3 / count;
		double actualTime_ms = (get_wall_time() - actualTimes.back()) / CLOCKS_PER_SEC * 1000 / count;
		cpuTimes.pop_back();
		actualTimes.pop_back();
		int precision1 = max(sigFigs - 1 - int(floor(log10(actualTime_ms))), 0);
		int precision2 = max(sigFigs - 1 - int(floor(log10(cpuTime_ms))), 0);
		if (print)
		{
			printf("%s: %.*f/%.*f ms\n", s.c_str(), precision1, cpuTime_ms, precision2, actualTime_ms);
		}
	}
	else
		printf("Error: Must call tic before toc.\n");
}

void channelDist(Mat& hsv, Mat& dist, int val, int channel)
{
	Mat channelImg;
	extractChannel(hsv, channelImg, channel);
	if (channel == 0)
	{
		Mat dist1, dist2;
		absdiff(channelImg, val, dist1);
		absdiff(channelImg, val + ((val < 90) ? 180 : -180), dist2);
		min(dist1, dist2, dist);
	}
	else
		absdiff(channelImg, val, dist);
}

void totalDist(Mat& hsv, Mat& dist, int hue, int sat, int val)
{
	Mat hueDist, satDist, valDist;
	channelDist(hsv, hueDist, hue, 0);
	channelDist(hsv, satDist, sat, 1);
	channelDist(hsv, valDist, val, 2);
	dist = hueDist + satDist + valDist;
}

Point2d getCenterOfMass(Moments moment)
{
	double m10 = moment.m10;
	double m01 = moment.m01;
	double mass = moment.m00;
	double x = m10 / mass;
	double y = m01 / mass;
	return Point2d(x, y);
}

bool compareMomentAreas(Moments moment1, Moments moment2)
{
	double area1 = moment1.m00;
	double area2 = moment2.m00;
	return area1 < area2;
}

void processImage()
{
	tic();
	
	// Threshold
	totalDist(hsv, dist, ballColor[0], ballColor[1], ballColor[2]);
	GaussianBlur(dist, distBlurred, Size(blurSize, blurSize), 0);
	threshold(distBlurred, bw, distThreshold, 255, THRESH_BINARY_INV);

	// Find contours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	Mat used = bw.clone();
	findContours(used, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
	if (hierarchy.size() < 1)
		return;

	// Sort contours by size
	vector<Moments> mm;
	for(int i = 0; i < hierarchy.size(); i++)
		mm.push_back(moments((Mat)contours[i]));
	std::sort(mm.begin(), mm.end(), compareMomentAreas);

	// Use largest contour to calculate ball center
	Point2f ballCenter_image = getCenterOfMass(mm[0]);
	
	// Init homography if not yet initialized
	if(!pointsInitialized)
		initPoints(img.size());

	// Tranform the points to world coordinates
	vector<Point2f> pts_image = {ballCenter_image}, pts_world;
	perspectiveTransform(pts_image, pts_world, homography);
	Point2f ballCenter_world = pts_world[0];

	// Publish results
	ballPose.x = ballCenter_world.x;
	ballPose.y = ballCenter_world.y;
	ballPose.theta = 0;
	ball_pub.publish(ballPose);

	toc("Time to process image");	
}

void createTrackbar()
{
	if(trackbarShown)
		return;
	cvDestroyWindow("Control");
	namedWindow("Control", WINDOW_NORMAL);
	printf("Ball values are %d, %d, %d, %d\n", ballColor[0], ballColor[1], ballColor[2], distThreshold);

	createTrackbar("Hue", "Control", &ballColor[0], 179);
	createTrackbar("Sat", "Control", &ballColor[1], 179);

	createTrackbar("Val", "Control", &ballColor[2], 255);
	createTrackbar("Dist", "Control", &distThreshold, 255);

	moveWindow("Control", 0, 0);
	resizeWindow("Control", 400, 100);
	trackbarShown = true;
}

void destroyTrackbar()
{
	cvDestroyWindow("Control");
	trackbarShown = false;
}

void printMenu()
{
	printf("a - Show grayscale\n");
	printf("b - Show threshold without blur\n");
	printf("c - Show threshold with blur\n");
	printf("z - Modify borders of field\n");
	printf("q - Quit\n");
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	img = cv_bridge::toCvShare(msg, "bgr8")->image;
	Mat result = img;
	cvtColor(img, hsv, COLOR_BGR2HSV);
	processImage();

	if(lastKeyPressed == 'a')
	{
		result = 255 - dist * (255.0 / distThreshold);
	}
	else if(lastKeyPressed == 'b')
	{
		result = 255 - distBlurred * (255.0 / distThreshold);
	}
	else if(lastKeyPressed == 'c')
	{
		result = bw;
	}
	else if(lastKeyPressed == 'z')
	{
		drawPoints(result);
	}
	else
	{
		destroyTrackbar();
	}

	imshow(GUI_NAME, result);

	// Wait for key press
	char key = waitKey(30);
	if(key != -1)
		lastKeyPressed = key;
	if(key == 'q')
		ros::shutdown();
}

vector<Point> points;

void initThresholds()
{
	ros::NodeHandle nh;
	nh.param<int>("/soccerref_vision/ball_color/hue", ballColor[0], 32);
	nh.param<int>("/soccerref_vision/ball_color/sat", ballColor[1], 57);
	nh.param<int>("/soccerref_vision/ball_color/val", ballColor[2], 236);
	nh.param<int>("/soccerref_vision/ball_color/dist", distThreshold, 30);
	printf("Initialized thresholds\n");
}

void saveThresholds()
{
	ros::NodeHandle nh;
	nh.setParam("/soccerref_vision/ball_color/hue", ballColor[0]);
	nh.setParam("/soccerref_vision/ball_color/sat", ballColor[1]);
	nh.setParam("/soccerref_vision/ball_color/val", ballColor[2]);
	nh.setParam("/soccerref_vision/ball_color/dist", distThreshold);
}

void calcHomography()
{
	// Image points (convert to float since the perspective transformation requires the points to be floats)
	vector<Point2f> pts_image = vector<Point2f>(points.size());
	for(int i = 0; i < points.size(); i++)
		pts_image[i] = Point2f(points[i]);

	// World points
	vector<Point2f> pts_world = vector<Point2f>(points.size());
	pts_world[0] = Point2f(-FIELD_WIDTH/2, -FIELD_HEIGHT/2);
	pts_world[1] = Point2f(FIELD_WIDTH/2, -FIELD_HEIGHT/2);
	pts_world[2] = Point2f(FIELD_WIDTH/2, FIELD_HEIGHT/2);
	pts_world[3] = Point2f(-FIELD_WIDTH/2, FIELD_HEIGHT/2);
	
	// Calculate homography
	homography = getPerspectiveTransform(pts_image, pts_world);
	vector<Point2f> test;
	perspectiveTransform(pts_image, test, homography);
	for(int i = 0; i < points.size(); i++)
		printf("%d - %f, %f\n", i, test[i].x, test[i].y);
}

void initPoints(Size imgSize)
{
	// load defaults
	points = vector<Point>(4);
	points[0] = Point(imgSize.width * 0.1, imgSize.height * 0.1);
	points[1] = Point(imgSize.width * 0.9, imgSize.height * 0.1);
	points[2] = Point(imgSize.width * 0.9, imgSize.height * 0.9);
	points[3] = Point(imgSize.width * 0.1, imgSize.height * 0.9);

	// overwrite with param server values (if they are available)
	ros::NodeHandle nh;
	for(int i = 0; i < points.size(); i++)
	{
		char buffer[50];
		sprintf(buffer, "/soccerref_vision/field_bounds/corner%d", i);
		string prefix = string(buffer);
		nh.param<int>(prefix + "/x", points[i].x, points[i].x);
		nh.param<int>(prefix + "/y", points[i].y, points[i].y);
	}
	pointsInitialized = true;
	calcHomography();
}

void savePoints()
{
	ros::NodeHandle nh;
	for(int i = 0; i < points.size(); i++)
	{
		char buffer[50];
		sprintf(buffer, "/soccerref_vision/field_bounds/corner%d", i);
		string prefix = string(buffer);
		nh.setParam(prefix + "/x", points[i].x);
		nh.setParam(prefix + "/y", points[i].y);
	}	
}

bool compareDoubles(double d1, double d2)
{
	return d1 < d2;
}

void moveClosestPoint(Point clickPoint)
{
	vector<double> distances = vector<double>(points.size());
	for(int i = 0; i < points.size(); i++)
		distances[i] = norm(points[i] - clickPoint);
	int idx = min_element(distances.begin(), distances.end()) - distances.begin();
	points[idx] = clickPoint;
}

void drawPoints(Mat img)
{
	const Point* ppt[1] = { &points[0] };
	int npt[] = { (int)points.size() };
	Mat mask = Mat(img.size(), CV_8UC1, Scalar(0));
	fillPoly(mask, ppt, npt, 1, Scalar(255), CV_AA);
	//Mat drawing = img.clone();
	Mat drawing = Mat(img.size(), CV_8UC3, Scalar(0));
	img.copyTo(drawing, mask);
	addWeighted(img, .5, drawing, .5, 0, img);
}

int mouseDown;

void mouseCallback(int event, int x, int y, int flags, void* userdata)
{
	if(event == EVENT_LBUTTONDOWN)
		mouseDown = 1;
	else if(event == EVENT_RBUTTONDOWN)
		mouseDown = 2;
	else if(event == EVENT_LBUTTONUP)
		mouseDown = 0;
	else if(event == EVENT_RBUTTONUP)
		mouseDown = 0;
	if (mouseDown && lastKeyPressed == 'z')
	{
		moveClosestPoint(Point(x, y));
		calcHomography();
		Mat result = img.clone();
		drawPoints(result);
		imshow(GUI_NAME, result);
	}
	else
	{
		// Tranform the points to world coordinates
		vector<Point2f> pts_image = {Point2f(x, y)}, pts_world;
		perspectiveTransform(pts_image, pts_world, homography);
		char buffer[100]; 
		sprintf(buffer, "Location in meters: (%.2f, %.2f)\n", pts_world[0].x, pts_world[0].y);
		displayStatusBar(GUI_NAME, buffer, 10000);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "vision_sim");
	ros::NodeHandle nh;

	// Create OpenCV Window and add a mouse callback for clicking
	namedWindow(GUI_NAME, CV_WINDOW_AUTOSIZE);
	setMouseCallback(GUI_NAME, mouseCallback, NULL);
	initThresholds();
	createTrackbar();

	// Subscribe to camera
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber image_sub = it.subscribe("/camera1/image_raw", 1, imageCallback);

	// Create Ball Publisher
	ball_pub = nh.advertise<geometry_msgs::Pose2D>("/vision/ball", 5);
	ros::spin();
	savePoints();
	saveThresholds();
	return 0;
}