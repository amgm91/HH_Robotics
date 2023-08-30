#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <unistd.h>
#include <math.h>

#include <ctime>
#include <iostream>
#include <raspicam/raspicam_cv.h>
#include <raspicam/raspicam.h>
#include "global.h"

// g++ $(pkg-config --libs --cflags opencv) -o camera camera.cpp

using namespace std;
using namespace cv;


int d = 0;
int box_found = 0;
int box_N = -1;
double dArea;
double box_area;

// box_N = 1 --> the box has 1 on it
// box_N = 0 --> the box has 0 on it
// box_N = -1 --> the box not detected


int *Camera_Client(void *)
{	
	VideoCapture cap(0);
	
	if(!cap.isOpened())
	{
		cout<<"Cannot open the web cam" <<endl;
		return 0;
	}
	// Create a window for the color sliders
	namedWindow("Control", CV_WINDOW_AUTOSIZE);
	
	int iLowH =40;
	int iHighH = 179;
	
	
	int iLowS = 100;
	int iHighS = 255;
	
	int iLowV = 40;
	int iHighV = 255;
	
	// Create trackbars in "Control Window"
	cvCreateTrackbar("LowH","Control",&iLowH,179);
	cvCreateTrackbar("HighH","Control",&iHighH,179);
	
	cvCreateTrackbar("LowS","Control",&iLowS,255);
	cvCreateTrackbar("HighS","Control",&iHighS,255);
	
	cvCreateTrackbar("LowV","Control",&iLowV,255);
	cvCreateTrackbar("HighV","Control",&iHighV,255);
	
	int iLastX = -1;
	int iLastY = -1;
	
		
	while(true){
		
		Mat imgOriginal;
		bool bSuccess = cap.read(imgOriginal);
		
				
		if(!bSuccess) 
		{
			cout<<"Cannot read a frame from video stream" << endl;
			break;
		}
		
		Mat imgHSV;
		
		cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV);
		
		Mat imgThresholded;
		
		inRange(imgHSV, Scalar(iLowH, iLowS,iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded);
		
		// Remove purple objects from frame
		Mat imgThresholded_purple;
		
		inRange(imgHSV, Scalar(140, 38, 33), Scalar(179, 255, 255), imgThresholded_purple);
		
		bitwise_not(imgThresholded_purple, imgThresholded_purple);
		
		bitwise_and(imgThresholded, imgThresholded_purple, imgThresholded);
		
		// remove yellow objects fror frame
		
		Mat imgThresholded_yellow;
		
		inRange(imgHSV, Scalar(19, 57, 48), Scalar(63, 255, 255), imgThresholded_yellow);
		
		bitwise_not(imgThresholded_yellow, imgThresholded_yellow);
		
		bitwise_and(imgThresholded, imgThresholded_yellow, imgThresholded);
		
		
		Size s = imgThresholded.size();
		int imgWidth = s.width;
		
		//Morphological opening (remove small objects from foreground)
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE,Size(6,6)));
		dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE,Size(6,6)));
		
		// Morphological closing (remove small holes from foreground)
		dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE,Size(6,6)));
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE,Size(6,6)));
		
		
		threshold(imgThresholded,imgThresholded,128,255,THRESH_BINARY);
		
		Moments oMoments = moments(imgThresholded);
		
		double dM01 = oMoments.m01;
		double dM10 = oMoments.m10;
		dArea = oMoments.m00;
		
		//cout << "dArea: " << dArea << endl;
		
		if(dArea > 22000)
		{
			int posX = dM10/dArea;
			int posY = dM01/dArea;
			d = posX - imgWidth/2;
			//cout<< "d = " << (posX - imgWidth/2) << endl;
		}
		else
		{
			d = 0;
			box_found = 0;
			box_N = -1;
		}
		
		//cout << "d: " << d << endl;
		
		
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;
		findContours(imgThresholded, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
		
		
		size_t largestContourIndex = 0;
		double largestContourArea = 0.0;
		
		if(!contours.empty())
		{	
			for(size_t i = 0; i < contours.size();i++)
			{
				double contourArea = cv::contourArea(contours[i]);
				if(contourArea > largestContourArea)
				{
					largestContourArea = contourArea;
					largestContourIndex = i;
				}
			}
			
			//cout << "Largest Contour Area: " << contourArea(contours[largestContourIndex]) << endl;
			
			box_area = contourArea(contours[largestContourIndex]);
			
			Rect roi = boundingRect(contours[largestContourIndex]);
			Mat roiImage = imgThresholded(roi).clone();
			
			//cout << "roi size: " << roiImage.size() << endl;
			
			vector<vector<Point> > contours2;
			vector<Vec4i> hierarchy2;
			findContours(roiImage, contours2, hierarchy2, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
			
			//cout << "Contours 2 Size: "<< contours2.size() << endl;
			if (contours2.size() > 1)
			{
				box_N = 0;
				//cout << "Detected Box : "<< box_N << endl; 
				
			}
			else
			{
				box_N = 1;
				//cout << "Detected Box : "<< box_N << endl; 
				
			}
			for (size_t i = 0; i < contours2.size(); i++) 
			{
				drawContours(imgOriginal, contours2, i, cv::Scalar(0, 0, 255), 2);
			}
			imshow("Detected Box", roiImage);
		}
		
		//imshow("yellow Binary",imgThresholded_yellow);
		//imshow("Purple Binary",imgThresholded_purple);
		imshow("Original", imgOriginal);
		//imshow("Thresholded Image", imgThresholded);
		
		
		
		if(waitKey(30) == 27)
		{
			cout<<"esc key is pressed by user" <<endl;
			cap.release();
			destroyAllWindows();
			break;
		}

	}
	return 0;
}


