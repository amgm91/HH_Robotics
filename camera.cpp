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
		
		//Mat imgLines = Mat::zeros(imgOriginal.size(), CV_8UC3);
		
		if(!bSuccess) 
		{
			cout<<"Cannot read a frame from video stream" << endl;
			break;
		}
		
		Mat imgHSV;
		
		cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV);
		
		Mat imgThresholded;
		
		inRange(imgHSV, Scalar(iLowH, iLowS,iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded);
		
		Size s = imgThresholded.size();
		int imgWidth = s.width;
		//int imgheight = s.height;
		//cout << "cols = " << imgWidth << endl;
		//cout << "rows = " << imgheight << endl;
		
		//Morphological opening (remove small objects from foreground)
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE,Size(6,6)));
		dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE,Size(6,6)));
		
		// Morphological closing (remove small holes from foreground)
		dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE,Size(6,6)));
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE,Size(6,6)));
		
		//GaussianBlur(imgThresholded,imgThresholded, Size(3,3),0);
		
		threshold(imgThresholded,imgThresholded,128,255,THRESH_BINARY);
		
		Moments oMoments = moments(imgThresholded);
		
		double dM01 = oMoments.m01;
		double dM10 = oMoments.m10;
		dArea = oMoments.m00;
		
		cout << "dArea: " << dArea << endl;
		
		if(dArea > 22000)
		{
			int posX = dM10/dArea;
			int posY = dM01/dArea;
			d = posX - imgWidth/2;
			box_found = 1;
			//cout<< "d = " << (posX - imgWidth/2) << endl;
		}
		else
		{
			d = 0;
			box_found = 0;
			box_N = -1;
		}
		
		cout << "d: " << d << endl;
		
		/*
		Mat result;
		matchTemplate(imgOriginal, oneTemplate, result, TM_CCOEFF_NORMED);
		
		normalize(result, result, 0,255, NORM_MINMAX, CV_8UC1);
		
		//cvtColor(result, result, COLOR_BGR2GRAY);
		
		double thres = 0.8;
		threshold(result, result, thres,1.0,THRESH_BINARY);
		
		vector<Point> locations;
		findNonZero(result, locations);
		
		cout <<"HERE"<<endl
		for(vector<Point>::const_iterator it = locations.begin();it !=locations.end(); ++it)
		{
			Rect matchRect(it->x,it->y, oneTemplate.cols, oneTemplate.rows);
			rectangle(imgOriginal, matchRect, Scalar(2,255,0),2);
			imshow("Template Matching", imgOriginal);
		}
			*/
		
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;
		findContours(imgThresholded, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
		
		//cout<<"Number of Countours" << contours.size() << endl;
		
		size_t largestContourIndex = 0;
		double largestContourArea = 0.0;
		
		if(!contours.empty())
		{
			//no_box = 0;
			/*for(const vector<Point>& contour : contours)
			{
			double area = contourArea(contour);
			
			if(area > 1000)
			{
				cout << "Contour Area= " << area<<endl; 
				//RotatedRect boundingRect = minAreaRect(contour);
				//Rect rect = boundingRect.boundingRect();
				/*
				if(rect.x>= 0 && rect.y >=0 && rect.x + rect.width <= imgOriginal.cols && rect.y + rect.height <= imgOriginal.rows)
				{
					Mat roi = imgOriginal(rect);
					Mat gray;
					cvtColor(roi, gray, COLOR_BGR2GRAY);
				
					threshold(gray, gray, 0,255, THRESH_BINARY | THRESH_OTSU);
				
					double whitePixelRatio = countNonZero(gray)/static_cast<double>(gray.rows * gray.cols);
				
					cout<<"whitePixelRatio = "<< whitePixelRatio <<endl;
				}
				
			}
			}*/
			
			for(size_t i = 0; i < contours.size();i++)
			{
				double contourArea = cv::contourArea(contours[i]);
				if(contourArea > largestContourArea)
				{
					largestContourArea = contourArea;
					largestContourIndex = i;
				}
			}
			
			//cout << "Largest countour Area: " << largestContourArea << endl;
			/*
			vector<Point> approx;
			
			approxPolyDP(contours[largestContourIndex], approx, arcLength(contours[largestContourIndex], true) * 0.02, true);
			//cout << "approx.size" << approx.size() <<endl;
			if (approx.size() > 8) {
				//cout << "Zero." << endl;
				box_N = 0;
				//cout << "approx.size" << approx.size() <<endl;
				//drawContours(imOriginal, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(0, 0, 255), 2);
			}
			else if(approx.size() < 8 && approx.size() > 4)
			{
				//cout << "One" << endl;
				box_N = 1;
			}
			else
			{
				box_N = -1;
			}
			*/
			//cout << "Box_N: " << box_N<<endl;
			//Rect boundingRect = cv::boundingRect(contours[largestContourIndex]);
		
			//rectangle(imgOriginal, boundingRect, Scalar(0,255,0),2);
			
			//float aspectRatio = static_cast<float>(boundingRect.width) / boundingRect.height;
			//cout << "aspect Ratio: " << aspectRatio << endl;
			
			//vector<Point> largestContour = contours[largestContourIndex];
			/*
			Rect boundingRect2 = boundingRect(contours[largestContourIndex]);
			
			Mat roi2 = imgOriginal(boundingRect2);
			
			Mat gray;
			cvtColor(roi2, gray, cv::COLOR_BGR2GRAY);
			
			Mat binary;
			threshold(gray, binary, 0, 255, THRESH_BINARY | THRESH_OTSU);
			
			vector<std::vector<cv::Point>> roiContours;
			findContours(binary, roiContours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

			double maxAreaROI = 0;
			int largestContourIndexROI = -1;
			for (size_t i = 0; i < roiContours.size(); i++) {
				double areaROI = cv::contourArea(roiContours[i]);
				if (areaROI > maxAreaROI) {
					maxAreaROI = areaROI;
					largestContourIndexROI = i;
				}
			}
			
			if (largestContourIndexROI != -1) 
			{
				drawContours(imgOriginal, roiContours, largestContourIndexROI, Scalar(0,0,255), 2);
				double contourAreaROI = cv::contourArea(roiContours[largestContourIndexROI]);
				cout << "Inside Contour Area: " << contourAreaROI << endl;
			}
			cout<<"Number of Countours" << roiContours.size() << endl;
			*/
			vector<Point> largestContour = contours[largestContourIndex];
			Rect boundingRect = cv::boundingRect(largestContour);
			rectangle(imgOriginal, boundingRect, Scalar(255,0,0),2);
			
			Mat roi = imgOriginal(boundingRect);
			
			Point center(boundingRect.x + boundingRect.width /2, boundingRect.y + boundingRect.height/2);
			Point center_1(boundingRect.x + boundingRect.width /2 + 50 , boundingRect.y + boundingRect.height/2);
			Point center_2(boundingRect.x + boundingRect.width /2 - 50 , boundingRect.y + boundingRect.height/2);
			
			Scalar pixelValue1 = imgThresholded.at<Vec3b>(center);
			Scalar pixelValue2 = imgThresholded.at<Vec3b>(center_1);
			Scalar pixelValue3 = imgThresholded.at<Vec3b>(center_2);
			
			cout<<"Pixel value at center = "<< pixelValue1 <<endl;
			cout<<"Pixel value at center + 25 = "<< pixelValue2 <<endl;
			cout<<"Pixel value at center - 25 = "<< pixelValue3 <<endl;
			cout<<"avg = "<< (pixelValue1[0]+ pixelValue2[0]+ pixelValue3[0]) / 3 <<endl;
			int avg  = (pixelValue1[0]+ pixelValue2[0]+ pixelValue3[0])/3;
			if(avg == 0)
			{
				box_N = 1;
			}
			else
			{
				box_N = 0;
			}
			
			/*
			int roiSize = 10;
			
			int roiX = max(0, center.x - roiSize/2);
			int roiY = max(0, center.y - roiSize/2);
			int roiWidth = min(roiSize, imgOriginal.cols-roiX);
			int roiHeight = min(roiSize, imgOriginal.rows - roiY);
			
			Rect centerRect(roiX, roiY, roiWidth, roiHeight);
			rectangle(imgOriginal, centerRect, Scalar(255,0,0),2);
			//Rect centerRect(center.x - roiSize/2, center.y - roiSize/2, roiSize, roiSize);
			
			Mat centerRoi = imgOriginal(centerRect);
			
			if(pixelValue[0] == 0)
				{
					box_N = 1;
				}
				else if(pixelValue[0] == 255)
				{
					box_N = 0;
				}
				else
				{
					box_N = -1;
				}
		
				//cout<<"Average HSV: "<< avgHSV[0] << ", " <<avgHSV[1]<<", "<< avgHSV[2]<<endl;
				cout << "Box Number: " << box_N << endl;
			
			if(centerRoi.cols>0 && centerRoi.rows>0)
			{
				Mat hsv;
				//cvtColor(roi, hsv, COLOR_BGR2HSV);
				cvtColor(centerRoi, hsv, COLOR_BGR2HSV);
		
				Scalar avgHSV = mean(hsv);
				
			}
			*/
			
		}
		
		
		imshow("Original", imgOriginal);
		
		//cout<< "Largest Area = "<<largestContourArea<<endl;
		//Mat contourImage = Mat::zeros(imgOriginal.size(), CV_8UC3);
		//drawContours(contourImage, contours, largestContourIndex, Scalar(0,255,0),2);
		//Mat image_copy = imgOriginal.clone();
		//drawContours(image_copy, contours, -1, Scalar(0, 255, 0), 2);
		//imshow("hello", image_copy);
	
	
		//imshow("Contours", contourImage);
		imshow("Thresholded Image", imgThresholded);
		//imshow("Original", imgOriginal);
		
		
		
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


