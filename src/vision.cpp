#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Vector3.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

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

// Mouse click parameters, empirically found
// The smaller the number, the more positive the error
// (i.e., it will be above the mouse in +y region)
#define FIELD_WIDTH_PIXELS      577.0 // measured from threshold of goal to goal
#define FIELD_HEIGHT_PIXELS     388.0 // measured from inside of wall to wall
#define CAMERA_WIDTH            640.0
#define CAMERA_HEIGHT           480.0

// These colours need to match the Gazebo materials
Scalar yellow[] = {Scalar(20,  128, 128), Scalar(30,  255, 255)};

// Handlers for vision position publishers
ros::Publisher ball_pub;

// Use variables to store position of objects. These variables are very
// useful when the ball cannot be seen, otherwise we'll get the position (0, 0)
geometry_msgs::Pose2D poseBall;

void thresholdImage(Mat& imgHSV, Mat& imgGray, Scalar color[])
{
	inRange(imgHSV, color[0], color[1], imgGray);

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

Point2d imageToWorldCoordinates(Point2d point_i)
{
	Point2d centerOfField(CAMERA_WIDTH/2, CAMERA_HEIGHT/2);
	Point2d center_w = (point_i - centerOfField);

	// You have to split up the pixel to meter conversion
	// because it is a rect, not a square!
	center_w.x *= (FIELD_WIDTH/FIELD_WIDTH_PIXELS);
	center_w.y *= (FIELD_HEIGHT/FIELD_HEIGHT_PIXELS);

	// Reflect y
	center_w.y = -center_w.y;
	
	return center_w;
}

void getBallPose(Mat& imgHsv, Scalar color[], geometry_msgs::Pose2D& ballPose)
{
	Mat imgGray;
	thresholdImage(imgHsv, imgGray, color);

	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours(imgGray, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

	if (hierarchy.size() < 1)
		return;

	for(int i = 0; i < hierarchy.size(); i++)
		mm.push_back(moments((Mat)contours[i]));

	std::sort(mm.begin(), mm.end(), compareMomentAreas);

	Moments mm = moments((Mat)contours[0]);
	Point2d ballCenter = imageToWorldCoordinates(getCenterOfMass(mm));

	ballPose.x = ballCenter.x;
	ballPose.y = ballCenter.y;
	ballPose.theta = 0;
}

void processImage(Mat img)
{
	// Convert to HSV
	Mat imgHsv;
	cvtColor(img, hsv, COLOR_BGR2HSV);

	// Threshold, etc
	getBallPose(hsv,  yellow, poseBall);

	// Publish results
	ball_pub.publish(poseBall);
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	try
	{
		Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
		processImage(frame);
		imshow(GUI_NAME, frame);
		waitKey(30);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
}

void mouseCallback(int event, int x, int y, int flags, void* userdata)
{
	if (event == EVENT_LBUTTONDOWN)
	{
		mouse_left_down = true;
		Point2d point_meters = imageToWorldCoordinates(Point2d(x, y));
		char buffer[50];
		sprintf(buffer, "Location: (%.3f m, %.3f m)", point_meters.x, point_meters.y);
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

	// Subscribe to camera
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber image_sub = it.subscribe("/camera1/image_raw", 1, imageCallback);

	// Create Ball Publisher
	ball_pub = nh.advertise<geometry_msgs::Pose2D>("/vision/ball", 5);
	ros::spin();
	return 0;
}