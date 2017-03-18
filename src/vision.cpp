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

using namespace std;
using namespace cv;

#define FIELD_WIDTH     3.53  // in meters
#define FIELD_HEIGHT    2.39 
#define ROBOT_RADIUS    0.10
#define GUI_NAME        "Soccer Overhead Camera"

//int yellow[] = {20, 128, 128,   30, 255, 255};
int ballColor[3];
int distThreshold;

// Handlers for vision position publishers
ros::Publisher ball_pub;

// Use variables to store position of objects. These variables are useful
// when the ball cannot be seen, otherwise we'll get the position (0, 0)
geometry_msgs::Pose2D poseBall;

void thresholdImage(Mat& imgHSV, Mat& imgGray, Scalar color[])
{
	//inRange(imgHSV, Scalar(yellow[0], yellow[1], yellow[2]), Scalar(yellow[3], yellow[4], yellow[5]), imgGray);

	erode(imgGray, imgGray, getStructuringElement(MORPH_ELLIPSE, Size(2, 2)));
	dilate(imgGray, imgGray, getStructuringElement(MORPH_ELLIPSE, Size(2, 2)));
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

// Point2d imageToWorldCoordinates(Point2d point_i)
// {
// 	//Point2d centerOfField(CAMERA_WIDTH/2, CAMERA_HEIGHT/2);
// 	//Point2d center_w = (point_i - centerOfField);

// 	// You have to split up the pixel to meter conversion
// 	// because it is a rect, not a square!
// 	//center_w.x *= (FIELD_WIDTH/FIELD_WIDTH_PIXELS);
// 	//center_w.y *= (FIELD_HEIGHT/FIELD_HEIGHT_PIXELS);

// 	// Reflect y
// 	//center_w.y = -center_w.y;
	
// 	return center_w;
// }

void getBallPose(Mat& imgHsv, Scalar color[], geometry_msgs::Pose2D& ballPose)
{
	Mat imgGray;
	thresholdImage(imgHsv, imgGray, color);

	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(imgGray, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

	if (hierarchy.size() < 1)
		return;

	// Sort
	vector<Moments> mm;
	for(int i = 0; i < hierarchy.size(); i++)
		mm.push_back(moments((Mat)contours[i]));
	std::sort(mm.begin(), mm.end(), compareMomentAreas);

	// Use largest to calculate ball center
	Point2d ballCenter; // = imageToWorldCoordinates(getCenterOfMass(mm[0]));
	ballPose.x = ballCenter.x;
	ballPose.y = ballCenter.y;
	ballPose.theta = 0;
}

void processImage(Mat img, Mat hsv)
{
	// Threshold, etc
	//getBallPose(hsv,  yellow, poseBall);

	// Publish results
	ball_pub.publish(poseBall);
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

char lastKeyPressed;
Mat hsv, img, bgr;
Point mouseLoc;

void createTrackbar()
{
	cvDestroyWindow("Control");
	namedWindow("Control", WINDOW_NORMAL);

	createTrackbar("Hue", "Control", &ballColor[0], 179);
	createTrackbar("Sat", "Control", &ballColor[1], 179);

	createTrackbar("Val", "Control", &ballColor[2], 255);
	createTrackbar("Dist", "Control", &distThreshold, 255);

	moveWindow("Control", 0, 0);
	resizeWindow("Control", 290, 290);
}

int blurSize = 3;

void drawPoints(Mat img);
void initPoints(Size imgSize);
bool pointsInitialized = false;
bool thresholdsInitialized = false;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	bgr = cv_bridge::toCvShare(msg, "bgr8")->image;
	img = bgr;
	if(!pointsInitialized)
		initPoints(img.size());
	cvtColor(img, hsv, COLOR_BGR2HSV);
	//processImage(img, hsv);

	if(lastKeyPressed == 'h')
		extractChannel(hsv, img, 0);
	if(lastKeyPressed == 's')
		extractChannel(hsv, img, 1);
	if(lastKeyPressed == 'v')
		extractChannel(hsv, img, 2);
	if(lastKeyPressed == 'd')
		channelDist(hsv, img, ballColor[0], 0);
	if(lastKeyPressed == 'a')
		img = hsv;
	if(lastKeyPressed == 'c')
	{
		totalDist(hsv, img, ballColor[0], ballColor[1], ballColor[2]);
		img = 255 - img * (255.0 / distThreshold);
	}
	if(lastKeyPressed == 'e')
	{
		totalDist(hsv, img, ballColor[0], ballColor[1], ballColor[2]);
		threshold(img, img, distThreshold, 255, THRESH_BINARY_INV);
	}
	if(lastKeyPressed == 'f')
	{
		totalDist(hsv, img, ballColor[0], ballColor[1], ballColor[2]);
		GaussianBlur(img, img, Size(blurSize, blurSize), 0);
		threshold(img, img, distThreshold, 255, THRESH_BINARY_INV);
	}
	if(lastKeyPressed == 'z')
	{
		drawPoints(img);
	}
	imshow(GUI_NAME, img);

	// Wait for key press
	char key = waitKey(30);
	if(key != -1)
		lastKeyPressed = key;
	if(key >= '1' && key <= '9')
		blurSize = (key - '0');
	if(key == 'q')
		ros::shutdown();
}

// void mouseCallback(int event, int x, int y, int flags, void* userdata)
// {
// 	Vec3b bgrVal = bgr.at<Vec3b>(y, x);
// 	Vec3b hsvVal = hsv.at<Vec3b>(y, x);
// 	char buffer[100];
// 	sprintf(buffer, "B:%d G:%d R:%d - H:%d S:%d V:%d", bgrVal[0], bgrVal[1], bgrVal[2], hsvVal[0], hsvVal[1], hsvVal[2]);
// 	displayStatusBar(GUI_NAME, buffer, 10000);

// 	if (event == EVENT_LBUTTONDOWN)
// 	{
// 		hueVal = hsvVal[0];
// 		mouseLoc = Point(x, y);
// 	}
// 	// Point2d point_meters = imageToWorldCoordinates(Point2d(x, y));
// 	//sprintf(buffer, "Location: (%.3f m, %.3f m)", point_meters.x, point_meters.y);
// }

vector<Point> points;

void initThresholds()
{
	ros::NodeHandle nh;
	nh.param<int>("/soccerref_vision/ball_color/hue", ballColor[0], 32);
	nh.param<int>("/soccerref_vision/ball_color/sat", ballColor[1], 57);
	nh.param<int>("/soccerref_vision/ball_color/val", ballColor[2], 236);
	nh.param<int>("/soccerref_vision/ball_color/dist", distThreshold, 30);
	thresholdsInitialized = true;
}

void saveThresholds()
{
	ros::NodeHandle nh;
	nh.setParam("/soccerref_vision/ball_color/hue", ballColor[0]);
	nh.setParam("/soccerref_vision/ball_color/sat", ballColor[1]);
	nh.setParam("/soccerref_vision/ball_color/val", ballColor[2]);
	nh.setParam("/soccerref_vision/ball_color/dist", distThreshold);
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

void drawPoints(Mat img)
{
	const Point* ppt[1] = { &points[0] };
	int npt[] = { points.size() };
	Mat mask = Mat(img.size(), CV_8UC1, Scalar(0));
	fillPoly(mask, ppt, npt, 1, Scalar(255), CV_AA);
	//Mat drawing = img.clone();
	Mat drawing = Mat(img.size(), CV_8UC3, Scalar(0));
	img.copyTo(drawing, mask);
	addWeighted(img, .5, drawing, .5, 0, img);
	vector<Point2f> pts_image = vector<Point2f>(points.size());
	for(int i = 0; i < points.size(); i++)
		pts_image[i] = Point2f(points[i]);

	vector<Point2f> pts_world = vector<Point2f>(points.size());
	pts_world[0] = Point2f(-FIELD_WIDTH/2, -FIELD_HEIGHT/2);
	pts_world[1] = Point2f(FIELD_WIDTH/2, -FIELD_HEIGHT/2);
	pts_world[2] = Point2f(FIELD_WIDTH/2, FIELD_HEIGHT/2);
	pts_world[3] = Point2f(-FIELD_WIDTH/2, FIELD_HEIGHT/2);
	
	//Mat H = findHomography()
	Mat H = getPerspectiveTransform(pts_image, pts_world);
	vector<Point2f> test;
	perspectiveTransform(pts_image, test, H);
	for(int i = 0; i < points.size(); i++)
		printf("%d - %f, %f\n", i, test[i].x, test[i].y);
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
	if (mouseDown)
	{
		moveClosestPoint(Point(x, y));
		//drawPoints();
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "vision_sim");
	ros::NodeHandle nh;

	// Create OpenCV Window and add a mouse callback for clicking
	namedWindow(GUI_NAME, CV_WINDOW_AUTOSIZE);
	setMouseCallback(GUI_NAME, mouseCallback, NULL);
	createTrackbar();

	// Subscribe to camera
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber image_sub = it.subscribe("/camera1/image_raw", 1, imageCallback);

	// Create Ball Publisher
	ball_pub = nh.advertise<geometry_msgs::Pose2D>("/vision/ball", 5);
	ros::spin();
	savePoints();
	return 0;
}