
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Float64.h>
#include "std_msgs/Float64MultiArray.h"
#include <math.h>

using namespace cv;
using namespace std;

// Limiares da cor azul ( Imagem HSV )
#define MINBLUE         95
#define MAXBLUE         135

#define MINSATBLUE      50
#define MAXSATBLUE      255

#define MINVALBLUE      50
#define MAXVALBLUE      255

// Limiares da cor amarela ( Imagem HSV )
#define MINYELLOW       20
#define MAXYELLOW       45

#define MINSATYELLOW    50
#define MAXSATYELLOW    255

#define MINVALYELLOW    50
#define MAXVALYELLOW    255

#define cvCOLOR_RED Scalar(0, 0, 255)
#define cvCOLOR_GREEN Scalar(0, 255, 0)



int ARR_MAXBLUE[3] = {MAXBLUE, MAXSATBLUE, MAXVALBLUE};
int ARR_MINBLUE[3] = {MINBLUE, MINSATBLUE, MINVALBLUE};

int ARR_MAXYELLOW[3] = {MAXYELLOW, MAXSATYELLOW, MAXVALYELLOW};
int ARR_MINYELLOW[3] = {MINYELLOW, MINSATYELLOW, MINVALYELLOW};

//parametros de filtros
#define GAUSSIANFILTER 3
#define KERNELSIZE 9

size_t countoursSize;
Mat pointsf;
RotatedRect box;

bool IMAGEM = false;
bool VIDEO = true;



class LandingMark
{
public:
    //// Variaveis ////
    cv::Mat main_img_C3, main_imgHSV_C3, img_blue_C1, img_yellow_C1, img_final_C1;
    cv::Mat morph_kernel;

    int rows, cols;
    int centerX, centerY;
    int majorEllipseWidth, majorEllipseHeight;
	int minEllipseWidth, minEllipseHeight;

    bool success, fstTime;

	cv::RotatedRect majorEllipse;
	cv::Rect mark;
	cv::Rect closestBase;

    std::vector< std::vector<cv::Point>> contours;

	std::vector<cv::Rect> basesList;

	std_msgs::Float64 dx, dy;


	LandingMark()
	{	
		this->morph_kernel = Mat::ones(KERNELSIZE, KERNELSIZE, CV_8U);

		this->success = false;
		this->fstTime = true;
	}


	Mat imfill(Mat img)
	{
		morphologyEx(img, img, MORPH_CLOSE, morph_kernel, Point(-1, -1), 3);

		findContours(img, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

		vector<vector<Point>> hull( contours.size() );

		for( size_t i = 0; i < contours.size(); i++ )
		{
			convexHull( contours[i], hull[i] );
		}

		if (hull.size() == 1)
		{
			drawContours(img, hull, 0, 255, -1);
		}
		else if (hull.size() > 1)
		{
			float biggestArea = 0;
			vector<Point> biggestContour;

			for ( size_t i = 0; i < hull.size(); i++ )
			{
				float area = contourArea(hull[i]);

				if (area > biggestArea)
				{
					biggestArea = area;
					biggestContour = hull[i];
				}
			}
			vector<vector<Point>> bigContours;
			bigContours.push_back(biggestContour);
			drawContours(img, bigContours, 0, 255, -1);
		}

		return img;
	}


	Mat imlimiares(Mat hsv, int hsvMin[3], int hsvMax[3])
	{
		Mat hsvtresh;

		inRange(hsv, Scalar(hsvMin[0], hsvMin[1], hsvMin[2]), Scalar(hsvMax[0], hsvMax[1], hsvMax[2]), hsvtresh);

		hsvtresh = imfill(hsvtresh);

		return hsvtresh;
	}


	void camParam(Mat img)
	{
		rows = img.rows;
		cols = img.cols;

		centerX = img.size().width/2;
		centerY = img.size().height/2;
	}


	void setImage(Mat img)
	{
		GaussianBlur(img, img, Size(GAUSSIANFILTER, GAUSSIANFILTER), 0);

		if (fstTime)
		{
			this->camParam(img);
			fstTime = false;
		}

		main_img_C3 = img;
	}


	void processImage()
	{
		cvtColor(main_img_C3, main_imgHSV_C3, COLOR_BGR2HSV);

		Mat hsv, output, bitwise_base_final;

		// Pega a area azul
		img_blue_C1 = imlimiares(main_imgHSV_C3, ARR_MINBLUE, ARR_MAXBLUE);
		bitwise_and(main_imgHSV_C3, main_imgHSV_C3, hsv, img_blue_C1);

		// Pega a area amarela
		img_yellow_C1 = imlimiares(main_imgHSV_C3, ARR_MINYELLOW, ARR_MAXYELLOW);
		bitwise_and(hsv, hsv, output, img_yellow_C1);

		// Pega apenas a area do mark
		bitwise_and(img_blue_C1, img_yellow_C1, img_final_C1);	
	}


	bool findSquare()
	{
		this->processImage();

		findContours(img_final_C1, this->contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

		bool success = false;
		int minArea = 400;

		for (int i=0; i<this->contours.size(); i++)
		{
			Rect currentRect = boundingRect( this->contours[i] );

			int areaRect = currentRect.width * currentRect.height;

			if (areaRect >= minArea)
			{
				this->basesList.push_back(currentRect);

				success = true;
			}
		}

		return success;
	}


	void drawClosestBase()
	{
		Rect closest;
		int distance;
		int lowestDistance = -1;

		for (size_t i = 0; i < this->basesList.size(); i++)
		{
			int currentBaseX = this->basesList[i].x;
			int currentBaseY = this->basesList[i].y;

			int dx = currentBaseX - this->centerX;
			int dy = currentBaseY - this->centerY;

			distance = hypot(dx, dy);

			// VERIFICA SE É A PRIMEIRA BASE A SER CALCULADA
			if (lowestDistance == -1)
			{
				lowestDistance = distance;
				closest = this->basesList[i];
			}
			else
			{
				if (distance < lowestDistance)
				{
					lowestDistance = distance;
					closest = this->basesList[i];
				}
			}

			cv::rectangle(this->main_img_C3, this->basesList[i], cvCOLOR_RED, 2, LINE_8);
		}

		this->basesList.clear();
		cv::rectangle(this->main_img_C3, closest, cvCOLOR_GREEN, 3);
		this->closestBase = closest;
	}


	void setClosestBaseDistance()
	{
		this->dx.data = this->closestBase.x + ( this->closestBase.width/2 ) - this->centerX;
		this->dy.data = this->closestBase.y + ( this->closestBase.height/2 ) - this->centerY;
	}


	bool existBaseAround()
	{
		bool success = false;

		for (size_t i = 0; i < this->basesList.size(); i++)
		{
			int coordX = this->basesList[i].x;
			int coordY = this->basesList[i].y;

			// Verifica se não é a base atual
			if ( coordX == this->closestBase.x && coordY == this->closestBase.y )
			{
				continue;
			}
			else
			{
				success = true;
				
				this->closestBase = this->basesList[i];

				break;
			}
		}

		return success;
	}

	// bool findSquare()
	// {
	// 	this->processImage();

	// 	findContours(img_final_C1, this->contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

	// 	bool success = false;
	// 	int currentArea = 0;

	// 	for (int i=0; i<this->contours.size(); i++)
	// 	{
	// 		Rect currentRect = boundingRect( this->contours[i] );

	// 		int areaRect = currentRect.width * currentRect.height;

	// 		if (areaRect >= currentArea)
	// 		{
	// 			currentArea = areaRect;

	// 			this->mark = currentRect;
	// 			this->dx.data = (currentRect.x + (currentRect.width/2)) - this->centerX;
	// 			this->dy.data = (currentRect.y + (currentRect.height/2)) - this->centerY;

	// 			success = true;
	// 		}
	// 	}

	// 	return success;
	// }


	void drawSquare()
	{
		rectangle(main_img_C3, mark, cvCOLOR_RED, 2);
	}


	void show()
	{
		imshow("Main_image", main_img_C3);
	}
};

LandingMark mark;

Mat frame;

ros::Publisher pub;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	try
	{
		Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;

		cv::resize(img, img, Size(), 0.75, 0.75);

		mark.setImage(img);

		std_msgs::Float64MultiArray array;
		array.data.resize(2);
		
		// Verifica se achou alguma elipse
		if ( mark.findSquare() )
		{
			mark.drawClosestBase();
			mark.setClosestBaseDistance();

			array.data[0] = mark.dx.data;
			array.data[1] = mark.dy.data;

			pub.publish(array);
		}  

		mark.show();
			
		waitKey(20);

	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
}

int main(int argc, char **argv)
{
 	ros::init(argc, argv, "image_listener");

	ros::NodeHandle n;

	pub = n.advertise<std_msgs::Float64MultiArray>("/base_detector/px_py", 10);

	image_transport::ImageTransport it(n);
	image_transport::Subscriber sub = it.subscribe("/hydrone/camera_camera/image_raw", 1, imageCallback);
	
	ros::spin();
		
	destroyWindow("view");
}