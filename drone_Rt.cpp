//#include "stdafx.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "vo_features.h"
#include <math.h>

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

int main(){

//カメラのパラメータなど
	Mat K=(Mat_<double>(3,3,CV_64FC1) <<484.9279,0,285.1439,0,482.8537,237.0575,0,0,1); 
	//Mat distortionCoefficient=(cv::Mat_<float>(4,1) <<0.1331,-0.5386,0.0119,-0.0056,1.6022);

//準備用のMat
	Mat img_1,img_2;//画像
	Mat R=cv::Mat::eye(3, 3, CV_64FC1);
	Mat t=cv::Mat::eye(3, 1, CV_64FC1);
	int i;
	i=1;

	VideoCapture cap(0);
for(int h=0;h<10;h++){



	if (i==0){
		img_2=img_1.clone();
		}
	cap >> img_1;
	
	if(i==0){
		Rt(img_1,img_2,R,t,K);//画像１、画像２、Rの出力用、tの出力用、内部パラメータ行列
		cout<<"R="<<R<<endl<<endl;
		cout<<"t="<<t<<endl<<endl;
	}

	i=0;
		
	cout<<"---------------------------------------------------------"<<endl;
}
}
	




			
