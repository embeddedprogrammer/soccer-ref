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

// TODO: At some point we should modify the dimensions to match the actual dimensions of the field
// These changes need to be done in this file, referee.py, the blender model, the soccerBall plugin, and the simulator vision.
// The actual dimenions of the field (not including the goals) are 3.18 x 2.22 meters
// The goals are 0.61 x 0.10 meters.

#define FIELD_WIDTH     3.53  // in meters
#define FIELD_HEIGHT    2.39 
#define GOAL_WIDTH      0.61
#define GOAL_DEPTH      0.14
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
Point2f ballCenter_image;

char lastKeyPressed;

Mat img, hsv, dist, distBlurred, bw, homography, mask;

Point mouseLoc;

bool pointsInitialized = false;
bool trackbarShown = false;

int blurSize = 5;

vector<Point> points, pts_goal1, pts_goal2;

// Function prototypes
void initThresholds();
void initPoints(Size imgSize);
void drawBorders(Mat result);
void drawMask();
void calcHomography();

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
	// Init mask and homography if not yet initialized
	if(!pointsInitialized)
	{
		initPoints(img.size());
		calcHomography();
		drawMask();
	}

	// Threshold
	totalDist(hsv, dist, ballColor[0], ballColor[1], ballColor[2]);
	bitwise_or(dist, mask, dist);
	GaussianBlur(dist, distBlurred, Size(blurSize, blurSize), 0);
	threshold(distBlurred, bw, distThreshold, 255, THRESH_BINARY_INV);

	// Find contours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	Mat used = bw.clone();
	findContours(used, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
	if (hierarchy.size() >= 1)
	{
		// Sort contours by size
		vector<Moments> mm;
		for(int i = 0; i < hierarchy.size(); i++)
			mm.push_back(moments((Mat)contours[i]));
		std::sort(mm.begin(), mm.end(), compareMomentAreas);

		// Use largest contour to calculate ball center
		ballCenter_image = getCenterOfMass(mm[0]);
		
		// Transform the points to world coordinates
		vector<Point2f> pts_image = {ballCenter_image}, pts_world;
		perspectiveTransform(pts_image, pts_world, homography);
		Point2f ballCenter_world = pts_world[0];

		// Store ball position (useful for when ball cannot be seen for a period of time)
		ballPose.x = ballCenter_world.x;
		ballPose.y = ballCenter_world.y;
		ballPose.theta = 0;
	}

	// Publish results
	ball_pub.publish(ballPose);
}

void createTrackbar()
{
	if(trackbarShown)
		return;
	cvDestroyWindow("Control");
	namedWindow("Control", WINDOW_NORMAL);

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
	printf("b - Show grayscale after gaussian blur\n");
	printf("c - Show threshold\n");
	printf("d - Show borders and ball location\n");
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
		createTrackbar();
		result = 255 - dist * (255.0 / distThreshold);
	}
	else if(lastKeyPressed == 'b')
	{
		createTrackbar();
		result = 255 - distBlurred * (255.0 / distThreshold);
	}
	else if(lastKeyPressed == 'c')
	{
		createTrackbar();
		result = bw + mask;
	}
	else if(lastKeyPressed == 'd')
	{
		destroyTrackbar();
		circle(result, ballCenter_image, 10, Scalar(0, 0, 255), 1, CV_AA);
		drawBorders(result);
	}
	else if(lastKeyPressed == 'z')
	{
		destroyTrackbar();
		drawBorders(result);
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

void initThresholds()
{
	ros::NodeHandle nh;
	nh.param<int>("/soccerref_vision/ball_color/hue", ballColor[0], 32);
	nh.param<int>("/soccerref_vision/ball_color/sat", ballColor[1], 57);
	nh.param<int>("/soccerref_vision/ball_color/val", ballColor[2], 236);
	nh.param<int>("/soccerref_vision/ball_color/dist", distThreshold, 30);
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
	pts_world[0] = Point2f(-FIELD_WIDTH/2, FIELD_HEIGHT/2);
	pts_world[1] = Point2f(FIELD_WIDTH/2, FIELD_HEIGHT/2);
	pts_world[2] = Point2f(FIELD_WIDTH/2, -FIELD_HEIGHT/2);
	pts_world[3] = Point2f(-FIELD_WIDTH/2, -FIELD_HEIGHT/2);
	
	// Calculate homography
	homography = getPerspectiveTransform(pts_image, pts_world);

	// Calculate goal dimensions
	vector<Point2f> pts_goal1_world = vector<Point2f>(4);
	pts_goal1_world[0] = Point2f(-FIELD_WIDTH/2 - GOAL_DEPTH, GOAL_WIDTH/2);
	pts_goal1_world[1] = Point2f(-FIELD_WIDTH/2,              GOAL_WIDTH/2);
	pts_goal1_world[2] = Point2f(-FIELD_WIDTH/2,              -GOAL_WIDTH/2);
	pts_goal1_world[3] = Point2f(-FIELD_WIDTH/2 - GOAL_DEPTH, -GOAL_WIDTH/2);
	vector<Point2f> pts_goal1_image;
	perspectiveTransform(pts_goal1_world, pts_goal1_image, homography.inv());

	vector<Point2f> pts_goal2_world = vector<Point2f>(4);
	pts_goal2_world[0] = Point2f(FIELD_WIDTH/2 + GOAL_DEPTH, GOAL_WIDTH/2);
	pts_goal2_world[1] = Point2f(FIELD_WIDTH/2,              GOAL_WIDTH/2);
	pts_goal2_world[2] = Point2f(FIELD_WIDTH/2,              -GOAL_WIDTH/2);
	pts_goal2_world[3] = Point2f(FIELD_WIDTH/2 + GOAL_DEPTH, -GOAL_WIDTH/2);
	vector<Point2f> pts_goal2_image;
	perspectiveTransform(pts_goal2_world, pts_goal2_image, homography.inv());

	// Convet points back to int (required for drawing functions)
	pts_goal1 = vector<Point>(pts_goal1_image.size());
	pts_goal2 = vector<Point>(pts_goal2_image.size());
	for(int i = 0; i < pts_goal1_image.size(); i++)
	{
		pts_goal1[i] = Point(pts_goal1_image[i]);
		pts_goal2[i] = Point(pts_goal2_image[i]);
	}
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

void drawMask()
{
	mask = Mat(img.size(), CV_8UC1, Scalar(255));
	
	const Point* ppt[1] = { &points[0] };
	int npt[] = { (int)points.size() };
	fillPoly(mask, ppt, npt, 1, Scalar(0), CV_AA);

	const Point* ppt_goals[2] = { &pts_goal1[0], &pts_goal2[0] };
	int npt_goals[] = { (int)pts_goal1.size(), (int)pts_goal2.size() };
	fillPoly(mask, ppt_goals, npt_goals, 2, Scalar(0), CV_AA);
}

void drawBorders(Mat result)
{
	const Point* ppt[1] = { &points[0] };
	int npt[] = { (int)points.size() };
	polylines(result, ppt, npt, 1, true, Scalar(0, 0, 255), 1, CV_AA);

	const Point* ppt_goals[2] = { &pts_goal1[0], &pts_goal2[0] };
	int npt_goals[] = { (int)pts_goal1.size(), (int)pts_goal2.size() };
	polylines(result, ppt_goals, npt_goals, 2, true, Scalar(0, 255, 0), 1, CV_AA);
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
		drawMask();
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
	// Init ros
	ros::init(argc, argv, "vision_sim");
	ros::NodeHandle nh;

	// Create OpenCV Window and add a mouse callback for clicking
	namedWindow(GUI_NAME, CV_WINDOW_AUTOSIZE);
	setMouseCallback(GUI_NAME, mouseCallback, NULL);
	printMenu();
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