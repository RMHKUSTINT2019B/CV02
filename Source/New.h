#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "math.h"
#include "stdio.h"
#include "cv_basic.h"

using namespace std;
using namespace cv;

const int thres_min_R = 190;
const int thres_max_G = 160;
const int thres_max_B = 160;
inline auto R(cv::Vec3b i) noexcept { return i[2]; }
inline auto G(cv::Vec3b i) noexcept { return i[1]; }
inline auto B(cv::Vec3b i) noexcept { return i[0]; }

inline Mat Tubeidentify(const Mat& input)
{
	Mat cx_src=input;
	int result_shumaguan;
//	result_shumaguan = 0;
//	Mat cx_src=imread("2017.png");
//	cvtColor(cx_src,cx_src,CV_BGR2GRAY);
//	threshold(cx_src,cx_src,200,255,THRESH_BINARY);
	auto view = cx_src.begin<cv::Vec3b>();
	auto _masked = cv::Mat1b(cx_src.rows, cx_src.cols);
	for (auto&& x : _masked) {
		auto pixel = *(view++);
		x = static_cast<uchar>((R(pixel)>thres_min_R && G(pixel)<thres_max_G && B(pixel)<thres_max_B) * 255);
	}
	//Mat element=getStructuringElement(MORPH_RECT,Size(3,3));
	//morphologyEx(cx_src,cx_src,MORPH_DILATE,element);
	//morphologyEx(cx_src,cx_src,MORPH_ERODE,element);
	//morphologyEx(cx_src,cx_src,MORPH_ERODE,element);
	cx_src = _masked;
	imshow("cx_src",cx_src);
	Mat row1,row2,col1;
	row1 = cx_src.rowRange(cx_src.rows/3,cx_src.rows/3+1);
	row2 = cx_src.rowRange(2*cx_src.rows/3,2*cx_src.rows/3+1);
	col1 = cx_src.colRange(cx_src.cols/2,cx_src.cols/2+1);
	cout<<row1<<endl<<endl;
	int flag_row1=0;
	int flag_row2=0;
	int flag_col1=0;
	int point_row1[10],point_row2[10],point_col1[10];
	for(int i=0;i<row1.cols-1;i++)
	{
		if(abs(row1.at<uchar>(0,i)-row1.at<uchar>(0,i+1))==255)
		{
			point_row1[flag_row1]=i;
			flag_row1++;
		}
		if(abs(row2.at<uchar>(0,i)-row2.at<uchar>(0,i+1))==255)
		{
			point_row2[flag_row2]=i;
			flag_row2++;
		}
	}
	for(int j=0;j<col1.rows-1;j++)
	{
		if(abs(col1.at<uchar>(j,0)-col1.at<uchar>(j+1,0))==255)
		{
			point_col1[flag_col1]=j;
			flag_col1++;
		}
	}
	cout<<flag_row1<<endl;
	cout<<flag_row2<<endl;
	cout<<flag_col1<<endl;
	if(flag_row1==2&&flag_row2==2&&flag_col1==2)
	{
		result_shumaguan=1;
	}
	if(flag_row1==2&&flag_row2==2&&flag_col1==6)
	{
		if(point_row1[0]>row1.cols/2)
		{
			if(point_row2[0]>row2.cols/2)
			{
				result_shumaguan=3;
			}
			else
				result_shumaguan=2;
		}
		else
			result_shumaguan=1;

	}
	if(flag_row1==4&&flag_row2==4&&flag_col1==6)
	{
		result_shumaguan=1;
	}
	if(flag_row1==2&&flag_row2==4&&flag_col1==6)
	{
		result_shumaguan=1;
	}
	if(flag_row1==4&&flag_row2==2&&flag_col1==6)
	{
		result_shumaguan=1;
	}
	if(flag_row1==2&&flag_row2==2&&point_row2[1]>row2.cols/2&&point_row1[1]>row1.cols/2&&flag_col1<6)
	{
		result_shumaguan=1;
	}
	cout<<result_shumaguan<<endl;
	waitKey(0);
	return cv::Mat1b(1, 1);
}