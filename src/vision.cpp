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

	// Sort
	vector<Moments> mm;
	for(int i = 0; i < hierarchy.size(); i++)
		mm.push_back(moments((Mat)contours[i]));
	std::sort(mm.begin(), mm.end(), compareMomentAreas);

	// Use largest to calculate ball center
	Point2d ballCenter = imageToWorldCoordinates(getCenterOfMass(mm[0]));
	ballPose.x = ballCenter.x;
	ballPose.y = ballCenter.y;
	ballPose.theta = 0;
}

void processImage(Mat img, Mat hsv)
{
	// Threshold, etc
	getBallPose(hsv,  yellow, poseBall);

	// Publish results
	ball_pub.publish(poseBall);
}

void channelDist(Mat& hsv, Mat& dist, int hueVal, int channel)
{
	Mat hue;
	extractChannel(hsv, hue, channel);
	if (channel == 0)
	{
		Mat dist1, dist2;
		absdiff(hue, hueVal, dist1);
		absdiff(hue, hueVal + ((hueVal < 90) ? 180 : -180), dist2);
		min(dist1, dist2, dist);
	}
	else
		absdiff(hue, hueVal, dist);
}

void hueMax(Mat& hsv, Mat& img)
{
	vector<Mat> channels;
	split(hsv, channels);
	channels[1].setTo(255);
	channels[2].setTo(255);
	Mat hsv2;
	merge(channels, hsv2);
	cvtColor(hsv2, img, CV_HSV2BGR);
}

void detectLines(Mat& img)
{
	// Init
	static Ptr<LineSegmentDetector> ls = createLineSegmentDetector(LSD_REFINE_STD); //can use LSD_REFINE_STD or LSD_REFINE_NONE

	// Convert to grayscale
    Mat gray;
    cvtColor(img, gray, COLOR_BGR2GRAY);

    // Detect the lines
    vector<Vec4f> lines_std;
    ls->detect(gray, lines_std);

    // Show found lines
    ls->drawSegments(img, lines_std);
}

void showHistogram(Mat& img)
{
	int bins = 256;             // number of bins
	int nc = img.channels();    // number of channels

	vector<Mat> hist(nc);       // histogram arrays

	// Initalize histogram arrays
	for (int i = 0; i < hist.size(); i++)
		hist[i] = Mat::zeros(1, bins, CV_32SC1);

	// Calculate the histogram of the image
	for (int i = 0; i < img.rows; i++)
	{
		for (int j = 0; j < img.cols; j++)
		{
			for (int k = 0; k < nc; k++)
			{
				uchar val = nc == 1 ? img.at<uchar>(i,j) : img.at<Vec3b>(i,j)[k];
				hist[k].at<int>(val) += 1;
			}
		}
	}

	// For each histogram arrays, obtain the maximum (peak) value
	// Needed to normalize the display later
	int hmax[3] = {0,0,0};
	for (int i = 0; i < nc; i++)
	{
		for (int j = 0; j < bins-1; j++)
			hmax[i] = hist[i].at<int>(j) > hmax[i] ? hist[i].at<int>(j) : hmax[i];
	}

	//const char* wname[3] = { "blue", "green", "red" };
	//Scalar colors[3] = { Scalar(255,0,0), Scalar(0,255,0), Scalar(0,0,255) };
	const char* wname[3] = { "hue", "sat", "val" };
	Scalar colors[3] = { Scalar(255,255,255), Scalar(255,255,255), Scalar(255,255,255) };

	vector<Mat> canvas(nc);

	// Display each histogram in a canvas
	for (int i = 0; i < nc; i++)
	{
		canvas[i] = Mat::ones(125, bins, CV_8UC3);

		for (int j = 0, rows = canvas[i].rows; j < bins-1; j++)
		{
			line(
				canvas[i], 
				Point(j, rows), 
				Point(j, rows - (hist[i].at<int>(j) * rows/hmax[i])), 
				nc == 1 ? Scalar(200,200,200) : colors[i], 
				1, 8, 0
			);
		}
		imshow(nc == 1 ? "value" : wname[i], canvas[i]);
	}
}

char lastKeyPressed;
int hueVal = 0;
Mat hsv, img, bgr;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	bgr = cv_bridge::toCvShare(msg, "bgr8")->image;
	img = bgr;
	cvtColor(img, hsv, COLOR_BGR2HSV);
	processImage(img, hsv);

	if(lastKeyPressed == 'h')
		extractChannel(hsv, img, 0);
	if(lastKeyPressed == 'i')
		hueMax(hsv, img);
	if(lastKeyPressed == 's')
		extractChannel(hsv, img, 1);
	if(lastKeyPressed == 'v')
		extractChannel(hsv, img, 2);
	if(lastKeyPressed == 'd')
		channelDist(hsv, img, hueVal, 0);
	if(lastKeyPressed == 'a')
		img = hsv;
	if(lastKeyPressed == 'l')
		detectLines(img);
	if(lastKeyPressed == 't')
		showHistogram(hsv);

	imshow(GUI_NAME, img);

	// Wait for key press
	char key = waitKey(30);
	if(key != -1)
		lastKeyPressed = key;
	if(key == 'q')
		ros::shutdown();
}

void mouseCallback(int event, int x, int y, int flags, void* userdata)
{
	Vec3b bgrVal = bgr.at<Vec3b>(y, x);
	Vec3b hsvVal = hsv.at<Vec3b>(y, x);
	char buffer[100];
	sprintf(buffer, "B:%d G:%d R:%d - H:%d S:%d V:%d", bgrVal[0], bgrVal[1], bgrVal[2], hsvVal[0], hsvVal[1], hsvVal[2]);
	displayStatusBar(GUI_NAME, buffer, 10000);

	if (event == EVENT_LBUTTONDOWN)
		hueVal = hsvVal[0];
	// Point2d point_meters = imageToWorldCoordinates(Point2d(x, y));
	//sprintf(buffer, "Location: (%.3f m, %.3f m)", point_meters.x, point_meters.y);
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