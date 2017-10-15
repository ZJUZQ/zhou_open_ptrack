

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <time.h>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <sensor_msgs/image_encodings.h>

#include "opencv2/opencv.hpp"
//#include "opencv2/highgui.hpp"

#include "camera_dh.hpp"      
#include "log.h"
#include "comdef.h"

// use computer's camera for code debug
#define USE_DEBUG_CAMERA

enum Status
{
    UNSUBCRIBED = 0,
    RAW,
};

void createImage(const cv::Mat &image, const std_msgs::Header &header, sensor_msgs::Image &msgImage);

void publishImages(const std_msgs::Header &header, sensor_msgs::CameraInfo &infoMsg, 
				   const std::string &cameraName, const cv::Mat &image, 
				   const Status &statu, const ros::Publisher &imagePub, const ros::Publisher &infoPub);
void createCameraInfo(const cv::Size &size, const cv::Mat &cameraMatrix, const cv::Mat &distortion, const cv::Mat &rotation, const cv::Mat &projection, sensor_msgs::CameraInfo &cameraInfo);
void createHeader(std_msgs::Header& header, const std::string& cameraName, int cout);

int main(int argc, char** argv){

	cv::Mat image;

	ros::init(argc, argv, "dh_bridge");
	ros::NodeHandle nh;

	int debug = 0;
	std::string debug_code_flow_file;
	std::ofstream debug_file;
	ros::param::get("~debug", debug);
	ros::param::get("~debug_code_flow_file", debug_code_flow_file);
	std::cout << std::endl << "debug = " << debug <<std::endl;
	std::cout << "debug_code_flow_file = " << debug_code_flow_file << std::endl;

	if(debug == 1){
		debug_file.open(debug_code_flow_file.c_str(), std::ios::app);
		//debug_file << "time ms: " << clock() * 1000 / CLOCKS_PER_SEC << "\n";
		debug_file << "ros time: " << ros::Time::now().toSec() << " (s) : " << ros::Time::now().toNSec() << "\n";
		debug_file << "dh_bridge.cpp -> main() : Begin\n\n";
	}

	std::string cameraName, cameraIP;

	int cameraPORT = 37777;
	double fx, fy, cx, cy;

	ros::param::get("~sensor_name", cameraName); // that is: /node_name/cameraName
	ros::param::get("~camera_ip", cameraIP);
	// read camera info
	ros::param::get("~fx", fx);
	ros::param::get("~fy", fy);
	ros::param::get("~cx", cx);
	ros::param::get("~cy", cy);

	if(debug == 1){
		//debug_file << "time ms: " << clock() * 1000 / CLOCKS_PER_SEC << "\n";
		debug_file << "ros time: " << ros::Time::now().toSec() << " (s) : " << ros::Time::now().toNSec() << "\n";
		debug_file << "		dh_bridge.cpp : sensor_name = " << cameraName << "\n\n";
		debug_file << "		dh_bridge.cpp : camera_ip = " << cameraIP << "\n\n";
		debug_file << "		dh_bridge.cpp : fx = " << fx << "\n\n";
		debug_file << "		dh_bridge.cpp : fy = " << fy << "\n\n";
		debug_file << "		dh_bridge.cpp : cx = " << cx << "\n\n";
		debug_file << "		dh_bridge.cpp : cy = " << cy << "\n\n";
	}
	
	ros::Publisher imagePub = nh.advertise<sensor_msgs::Image>("/" + cameraName + "/" + "image", 2);
    ros::Publisher infoPub = nh.advertise<sensor_msgs::CameraInfo>("/" + cameraName + "/" + "camera_info", 2);

    if(debug == 1){
    	//debug_file << "time ms: " << clock() * 1000 / CLOCKS_PER_SEC << "\n";
    	debug_file << "ros time: " << ros::Time::now().toSec() << " (s) : " << ros::Time::now().toNSec() << "\n";
		debug_file << "		dh_bridge.cpp : imagePub, topic = /" << cameraName << "/image\n\n";
		debug_file << "		dh_bridge.cpp : infoPub, topic = /" << cameraName << "/camera_info\n\n";
		debug_file.close();
	}

    #ifdef USE_DEBUG_CAMERA
    cv::VideoCapture cap(0); // open computer's default camera
    if(!cap.isOpened())
    	return -1;
    cap >> image;
    #endif

    #ifndef USE_DEBUG_CAMERA
    // Create Camera handle
    CAMERA_DH *camera = new CAMERA_DH();
    void *ipc = camera->createIPC(cameraIP.c_str(), cameraPORT, 1);
    if(ipc === 0)
    {
        printf("Create IPC failed: %s:%d\n",cameraIP.c_str(), cameraPORT);
        return -1;
    }

    usleep(1e6); // Wait 1s to init camera configuration
    image = camera->getImage(ipc);
	#endif

	// camera_info
	sensor_msgs::CameraInfo camera_info;
	cv::Mat cameraMatrixColor = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat distortionColor = cv::Mat::zeros(1, 5, CV_64F);

	cameraMatrixColor.at<double>(0, 0) = fx;
	cameraMatrixColor.at<double>(1, 1) = fy;
	cameraMatrixColor.at<double>(0, 2) = cx;
	cameraMatrixColor.at<double>(1, 2) = cy;
	cameraMatrixColor.at<double>(2, 2) = 1;

	cv::Mat projColor = cv::Mat::zeros(3, 4, CV_64F); // Projection/camera matrix
    cameraMatrixColor.copyTo(projColor(cv::Rect(0, 0, 3, 3)));
    	
    createCameraInfo(cv::Size(image.cols, image.rows), cameraMatrixColor, distortionColor, cv::Mat::eye(3, 3, CV_64F), projColor, camera_info);
    
    std::cout << "debug_0_0" << std::endl;
	int counter = 0;
	//cv::namedWindow("display", 0);

	ros::Rate loop(50);
	while(ros::ok){

		#ifndef USE_DEBUG_CAMERA
		image = camera->getImage(ipc);
		#endif

		#ifdef USE_DEBUG_CAMERA
		cap >> image;
		#endif
		/* 
		cv::imshow( "display", image);
    	char c = (char)cv::waitKey(1);
    	if(27==c || 'q'==c)
            break;
        */

        if(debug == 1 && counter == 1){
        	debug_file.open(debug_code_flow_file.c_str(), std::ios::app);
      		//debug_file << "time ms: " << clock() * 1000 / CLOCKS_PER_SEC << "\n";
      		debug_file << "ros time: " << ros::Time::now().toSec() << " (s) : " << ros::Time::now().toNSec() << "\n";
			debug_file << "		dh_bridge.cpp : starting receive image from camera driver...\n\n";
			debug_file.close();
		}


        std_msgs::Header header;
        createHeader(header, cameraName, counter);

        Status statu = UNSUBCRIBED;
    	if(imagePub.getNumSubscribers() > 0) // Returns the number of subscribers that are currently connected to this Publisher. 
      		statu = RAW;

      	if(statu == UNSUBCRIBED)
      		continue;

      	publishImages(header, camera_info, cameraName, image, statu, imagePub, infoPub);

      	if(debug == 1 && counter == 1){
      		debug_file.open(debug_code_flow_file.c_str(), std::ios::app);
      		//debug_file << "time ms: " << clock() * 1000 / CLOCKS_PER_SEC << "\n";
      		debug_file << "ros time: " << ros::Time::now().toSec() << " (s) : " << ros::Time::now().toNSec() << "\n";
			debug_file << "		dh_bridge.cpp : imagePub, 	starting publish image...\n\n";
			debug_file << "		dh_bridge.cpp : infoPub, 	starting publish camera_info...\n\n";
			debug_file.close();
		}

		/*
		cv_bridge::CvImage img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, image);
		sensor_msgs::Image img_msg;
		img_bridge.toImageMsg(img_msg);

		camera_info.header = header;
		camera_info.header.frame_id = header.frame_id;

		switch(statu)
		{
		case UNSUBCRIBED:
		  	break;
		case RAW:
			imagePub.publish(img_msg);
		  	infoPub.publish(camera_info);
		  	break;
		}
		*/
      	
		counter++;
		ros::spinOnce();
        loop.sleep();
	}
	if(debug == 1){
		debug_file.open(debug_code_flow_file.c_str(), std::ios::app);
		//debug_file << "time ms: " << clock() * 1000 / CLOCKS_PER_SEC << "\n";
		debug_file << "ros time: " << ros::Time::now().toSec() << " (s) : " << ros::Time::now().toNSec() << "\n";
		debug_file << "dh_bridge.cpp -> main() : End\n\n";
		debug_file.close();
	}
	return 0;
}
void publishImages(const std_msgs::Header &header, sensor_msgs::CameraInfo &infoMsg, 
				   const std::string &cameraName, const cv::Mat &image, 
				   const Status &statu, const ros::Publisher &imagePub, const ros::Publisher &infoPub)
{
	sensor_msgs::Image imageMsg;
	//sensor_msgs::CameraInfo infoMsg;
	infoMsg.header = header;
	infoMsg.header.frame_id = header.frame_id;

	switch(statu)
	{
	case UNSUBCRIBED:
	  	break;
	case RAW:
	  	createImage(image, infoMsg.header, imageMsg);
		imagePub.publish(imageMsg);
	  	infoPub.publish(infoMsg);
	  	break;
	}
}

// input image, output msgImage
void createImage(const cv::Mat &image, const std_msgs::Header &header, sensor_msgs::Image &msgImage)
{
    size_t step, size;
    step = image.cols * image.elemSize();
    size = image.rows * step;

    msgImage.encoding = sensor_msgs::image_encodings::BGR8;

    msgImage.header = header;
    msgImage.height = image.rows;
    msgImage.width = image.cols;
    msgImage.is_bigendian = false;
    msgImage.step = step;
    msgImage.data.resize(size); // uint8[] data
    memcpy(msgImage.data.data(), image.data, size);
}

void createHeader(std_msgs::Header& header, const std::string& cameraName, int cout)
{
	header.seq = cout;
	header.stamp = ros::Time::now();
	header.frame_id = "/" + cameraName + "_optical_frame";
}

void createCameraInfo(const cv::Size &size, const cv::Mat &cameraMatrix, const cv::Mat &distortion, const cv::Mat &rotation, const cv::Mat &projection, sensor_msgs::CameraInfo &cameraInfo)
{
	cameraInfo.height = size.height;
	cameraInfo.width = size.width;

	const double *itC = cameraMatrix.ptr<double>(0, 0);
	for(size_t i = 0; i < 9; ++i, ++itC)
	{
	  cameraInfo.K[i] = *itC; // Intrinsic camera matrix for the raw (distorted) images
	}

	const double *itR = rotation.ptr<double>(0, 0); // (stereo cameras only)
	for(size_t i = 0; i < 9; ++i, ++itR)
	{
	  cameraInfo.R[i] = *itR;
	}

	const double *itP = projection.ptr<double>(0, 0);
	for(size_t i = 0; i < 12; ++i, ++itP)
	{
	  cameraInfo.P[i] = *itP;
	}

	cameraInfo.distortion_model = "plumb_bob";
	cameraInfo.D.resize(distortion.cols);
	const double *itD = distortion.ptr<double>(0, 0);
	for(size_t i = 0; i < (size_t)distortion.cols; ++i, ++itD)
	{
	  cameraInfo.D[i] = *itD;
	}
}


