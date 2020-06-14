
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/imgproc.hpp"
#include "iostream"
#include<windows.h>
#include "omp.h"


#include<conio.h>
//#include<fstream>
using namespace cv;
using namespace std;

#define T_ANGLE_THRE 5//6
#define T_SIZE_THRE 1.5//3
#define RED -1
#define BLUE 1
#define CILLING 5

static unsigned char function_flag;
static int target_color = RED;
int up_cap_count = 0;

static HANDLE hCom;
static HANDLE thread;

//void read_ini(wchar_t* text,int target);
void update();
Point2f get_arror(RotatedRect target);
vector<vector<Point>> judge_contours(vector<vector<Point>> test_white, vector<vector<Point>> test_color, vector <Vec4i>  hiararchy);
bool fun2(Point2f pt[]);
bool parallel(Point2f point_a, Point2f point_b);
float get_cos_angle(Point2f point_a, Point2f point_b);
float get_point_to_point(Point2f point_a, Point2f point_b);
RotatedRect fun(vector<RotatedRect> test);
void find_2(Mat *target, vector<Vec4i> hi, vector <vector<Point>> temp);
void find_white_color(Mat *src1, Mat *dst_white, int R, int G, int B, Mat *dst_color, int target_color, int min, int max);
void find_white(Mat *src1, Mat *dst, int R, int G, int B);
void DesColor(Mat src1, int a, Mat dst, int min, int max);
void brightAdjust(Mat src, Mat dst, double dContrast, double dBright); //亮度调节函数
vector<RotatedRect> armorDetect(vector<RotatedRect> vEllipse); //检测装甲
void drawBox(RotatedRect box, Mat img, Scalar color); //标记装甲 
RotatedRect find_best(vector<RotatedRect> vEllipse);
vector<vector<Point>> judge_contours(vector<vector<Point>> test, vector<vector<Point>> test2, int target);
void receive() {
	PurgeComm(hCom, PURGE_TXCLEAR | PURGE_RXCLEAR);//对串口的缓冲区进行处理
	DWORD back1;
	unsigned char a = 0;
	while (1) {
		ReadFile(hCom, &a, 2, &back1, NULL);
		switch (a) {
		case 0x00:
			function_flag = 0x00;
			break;
		case 0x01:
			function_flag = 0x01;
			target_color = RED;
			update();
			break;
		case 0x02:
			function_flag = 0x02;
			target_color = BLUE;
			update();
			break;
		case 0x03:
			function_flag = 0x03;
			break;
		case 0x04:
			function_flag = 0x04;
			break;
		case 0x05:
			function_flag = 0x05;
			break;
		}
		//cout << "abc" << endl;
		Sleep(50);
	}
}
RotatedRect up(Point2f(0, 0), Size2f(0, 0), 0);
int up_flag = -1;
int b = 0;
int g = 0;
int r = 0;
Size imgSize;
int main()
{


	wchar_t* text = L"COM3";
	hCom = CreateFile(text, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
	if (hCom == INVALID_HANDLE_VALUE)
	{
		cout << "port open fail" << endl;
		//return FALSE;
	}
	
	SetupComm(hCom, 1024, 1024); //输入缓冲区和输出缓冲区的大小都是1024
	COMMTIMEOUTS TimeOuts; //设定读超时
	TimeOuts.ReadIntervalTimeout = 1000;
	TimeOuts.ReadTotalTimeoutMultiplier = 500;
	TimeOuts.ReadTotalTimeoutConstant = 5000; //设定写超时
	TimeOuts.WriteTotalTimeoutMultiplier = 500;
	TimeOuts.WriteTotalTimeoutConstant = 2000;
	SetCommTimeouts(hCom, &TimeOuts); //设置超时
									  //采用同步发送
	DCB dcb;
	GetCommState(hCom, &dcb);
	dcb.BaudRate = CBR_115200; //波特率为115200
	dcb.ByteSize = 8; //每个字节有8位
	dcb.Parity = NOPARITY; //无奇偶校验位
	dcb.StopBits = ONESTOPBIT; //一个停止位
	SetCommState(hCom, &dcb);
	PurgeComm(hCom, PURGE_TXCLEAR | PURGE_RXCLEAR);//对串口的缓冲区进行处理

												   //thread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)receive, NULL, 0, 0);


	//VideoCapture cap0(1);
	VideoCapture cap0("test.avi");
	//VideoCapture cap0("ytg.mp4");//1：USB摄像头，0，电脑自带摄像头
	cap0.set(CV_CAP_PROP_FPS, 60);//强制60帧
	cap0.set(CV_CAP_PROP_FRAME_WIDTH, 480);
	cap0.set(CV_CAP_PROP_FRAME_HEIGHT,360);
	Mat frame0;
	RotatedRect s;
	vector<RotatedRect> vEllipse;
	vector<RotatedRect> vRlt;
	vector<RotatedRect> vArmor;
	bool bFlag = false;

	vector<vector<Point> > contour;
	vector<vector<Point> > contour2;
	vector<vector<Point> > contour3;
	cap0 >> frame0;
	imgSize = frame0.size();

	vector <Vec4i>  hiararchy;

	Mat change = Mat(imgSize, CV_8UC3);
	Mat rawImg = Mat(imgSize, CV_8UC3, Scalar(255, 255, 255));
	Mat grayImage = Mat(imgSize, CV_8UC1);
	Mat Image1 = Mat(imgSize, CV_8UC1);
	Mat Image2 = Mat(imgSize, CV_8UC1);
	Mat binary = Mat(imgSize, CV_8UC1);
	Mat rlt = Mat(imgSize, CV_8UC1);
	namedWindow("Raw");

	while (1)
	{
		if (cap0.read(frame0))
		{
			//find_white_color(&frame0, &binary, 230, 230, 230, &grayImage, target_color, 200, 255);
			find_white_color(&frame0, &binary, 170, 170, 170, &grayImage, target_color, 200, 255);
			blur(binary, Image1, Size(3, 3));
			blur(grayImage, Image2, Size(3, 3));
			findContours(Image1, contour, RETR_LIST, CHAIN_APPROX_NONE);
			findContours(Image2, contour2, hiararchy, RETR_TREE, CHAIN_APPROX_NONE);
			for (int i = 0; i<contour.size(); i++)
			{
				if (contour[i].size()< 30)  //判断当前轮廓是否大于10个像素点
				{
					contour.erase(contour.begin() + i);
					i--;
				}
			}

			contour3 = judge_contours(contour, contour2, 1);
			for (int i = 0; i<contour3.size(); i++)
			{
				if (contour3[i].size()> 10)
				{
					bFlag = true;
					s = fitEllipse(contour3[i]);
					if (bFlag)
					{
						vEllipse.push_back(s); //将发现的目标保存
						drawBox(s, frame0, Scalar(0, 0, 255));
					}
				}
			}
			drawContours(rawImg, contour, -1, Scalar(0, 0, 0), -1);
			drawContours(rawImg,contour2 ,-1, Scalar(0, 255, 0), -1);
			drawContours(rawImg, contour3, -1, Scalar(0, 0, 255), -1);
			imshow("binary", rawImg);
			rawImg.setTo(255);

			vRlt = armorDetect(vEllipse);
			for (unsigned int nI = 0; nI < vRlt.size(); nI++) //在当前图像中标出装甲的位置
				drawBox(vRlt[nI], frame0, Scalar(0, 255, 255));
			if (vRlt.size() > 0) {
				RotatedRect asd = find_best(vRlt);
				if (asd.size.area() > 20) {
					drawBox(asd, frame0, Scalar(0, 255, 0));
					DWORD back1;
					int set[3] = { asd.center.x  ,asd.center.y ,1 };
					up_cap_count = CILLING;
					//system("cls");
					//cout << set[0] << endl;
					//cout << set[1] << endl;
					//cout << set[2] << endl;
					WriteFile(hCom, &set, 12, &back1, NULL);
				}
				else {
					up_cap_count--;

					if (up_cap_count == 0) {
						DWORD back1;
						int set[3] = { 0  ,0 ,0 };
						WriteFile(hCom, &set, 12, &back1, NULL);
					}
				}
			}
			cv::imshow("Raw", frame0);
			cv::waitKey(1);
			vEllipse.clear();
			vRlt.clear();
			vArmor.clear();
		}
		else
		{
			break;
		}
	}
	CloseHandle(hCom);
	//delete thread;
	cap0.release();
	return 0;

}
void update() {
	system("cls");
	cout << "target_color:" << target_color << endl;

}


vector<vector<Point>> judge_contours(vector<vector<Point>> test_white, vector<vector<Point>> test_color, int target) {
	//	double pointPolygonTest(InputArray contour, Point2f pt, bool measureDist)
	vector<vector<Point>> back;
	//	1...................
	//	vector<vector<Point>>white = test_white;
	//	vector <vector <Point>>color = test_color;
	//	vector<vector<Point>>::iterator it_end_white = test_white.end();
	//	vector<vector<Point>>::iterator it_end_color = test_color.end();
	//	omp_set_num_threads(8);
	//#pragma omp parallel for	
	//	for (vector<vector<Point>>::iterator it_1=test_white.begin(); it_1!=it_end_white; ++it_1) {
	//		int length = 0;
	//		//Point *temp_point;
	//		int count = 0;
	//
	//		for (vector<vector<Point>>::iterator it_2 = test_color.begin(); it_2!=it_end_color; ++it_2) {
	//			for (vector<Point>::iterator it_point=it_2->begin(); it_point!=it_2->end(); ++it_point) {
	//				it_2->erase(it_point);
	//				length = (int)pointPolygonTest(*it_1, *it_point, true);
	//				if ((abs(length) <5)&&(length<=0)) {
	//					count++;
	//					//it_point = it_2->erase(it_point);
	//				}
	//			//	else {
	//				//	++it_point;
	//			//	}
	//			}
	//		}
	//		if (count >= (it_1->size()>>1)) {
	//			back.push_back(*it_1);
	//		}
	//		count = 0;
	//	}


	//----------------------------------------------------------------------------------		
	omp_set_num_threads(8);
#pragma omp parallel for 	

	for (int temp_i = test_white.size() - 1; temp_i >= 0; temp_i--) {
		int count = 0;
		bool flag = false;
		int temp_count = (test_white[temp_i].size() >> 1);
		for (int i = test_color.size() - 1; i >= 0; i--) {
			if (count >= temp_count) {
				i = 0;
				continue;
			}

			for (int p = test_color[i].size() - 1; p >= 0; p--) {
				if (count >= temp_count) {
					flag = true;
					p = 0;
					continue;
				}
				int length = 0;
				Point *	temp_point = &(test_color[i])[p];
				length = (int)pointPolygonTest(test_white[temp_i], *temp_point, true);
				if ((abs(length) <5)
					//&& (length <= 0)
					) {
					//#pragma omp critical
					count++;
				}
			}
		}
		if (flag) {
#pragma omp critical
			back.push_back(test_white[temp_i]);
		}
	}

	//----------------------------------------------------------------------------------------------------------------
	//	vector <Point> total_color_point;
	//	
	//	omp_set_num_threads(8);
	//#pragma omp parallel for 	
	//
	//	for (int temp_i = test_white.size() - 1; temp_i >= 0; temp_i--) {
	//			int count = 0;
	//			bool flag = false;
	//			vector <vector <Point>>color_temp = color;
	//			int temp_count = (test_white[temp_i].size() >> 1);
	//		for (int i = test_color.size() - 1; i >= 0; i--) {
	//
	//
	//			for (int p = test_color[i].size() - 1; p >= 0; p--) {
	//							//if (count >= temp_count) {
	//							//	flag = true;
	//							//	continue;
	//							//}
	//							int length = 0;
	//							Point *	temp_point = &(test_color[i])[p];
	//							length = (int)pointPolygonTest(test_white[temp_i], *temp_point, true);
	//							if ((abs(length) <5)
	//								//&& (length <= 0)
	//								) {
	//								//#pragma omp critical
	//								count++;
	//							}
	//						}
	//					}
	//					if (flag) {
	//			#pragma omp critical
	//						back.push_back(test_white[temp_i]);
	//					}
	//			}
	//
	//----------------------------------------------------------------------------------------------

	return back;
}
vector<vector<Point>> judge_contours(vector<vector<Point>> test_white, vector<vector<Point>> test_color, vector <Vec4i>  hiararchy) {
	vector<vector<Point>> back;
	vector<vector<Point>>convex;
	Mat whilte(imgSize, CV_8UC1);
	Mat color(imgSize, CV_8UC1);
	Mat imgback(imgSize, CV_8UC1);
	drawContours(whilte, test_white, -1, Scalar(255, 255, 255), -1);
	for (int a = test_color.size() - 1, p = 0; a >= 0; a--) {
		vector<Point>temp;
		if (hiararchy[a][3] == -1) {
			vector<Point>temp;
			convexHull(test_color[a], temp);
			convex.push_back(temp);
		}
	}
	drawContours(color, convex, -1, Scalar(255, 255, 255), -1);
	bitwise_and(whilte, color, imgback);
	findContours(imgback, back, RETR_LIST, CHAIN_APPROX_NONE);
	return back;


}
void brightAdjust(Mat src, Mat dst, double dContrast, double dBright)
{
	int nVal;
	omp_set_num_threads(8);
#pragma omp parallel for

	for (int nI = 0; nI<src.rows; nI++)
	{
		Vec3b* p1 = src.ptr<Vec3b>(nI);
		Vec3b* p2 = dst.ptr<Vec3b>(nI);
		for (int nJ = 0; nJ <src.cols; nJ++)
		{
			for (int nK = 0; nK < 3; nK++)
			{
				//	每个像素的每个通道的值都进行线性变换
				nVal = (int)(dContrast * p1[nJ][nK] + dBright);
				if (nVal < 0)
					nVal = 0;
				if (nVal > 255)
					nVal = 255;
				p2[nJ][nK] = nVal;
			}
		}
	}
}
RotatedRect find_best(vector<RotatedRect> test) {

	float threshold_ = 0;
	int max = 0;
	int count = -1;
	int temp_area = 0;
	int _long, _short;
	float min = 5;
	for (int temp = test.size() - 1; temp >= 0; temp--) {

		if (test[temp].size.height >= test[temp].size.width) {
			_long = test[temp].size.height;
			_short = test[temp].size.width;
		}
		else {
			_short = test[temp].size.height;
			_long = test[temp].size.width;
		}
		if (
			//temp_area > max
			//	&&
			_long / _short < min
			) {
			min = _long / _short;
			//max = temp_area;
			count = temp;
		}
	}
	if (count<0) {
		Point2f back_center;
		Size back_size;
		RotatedRect back;
		back_center.x = 0;
		back_center.y = 0;
		back_size.height = 0;
		back_size.width = 0;
		back.size = back_size;
		back.center = back_center;
		back.angle = 0;
		return back;
	}
	else {
		return test[count];
	}
}


bool parallel(Point2f point_a, Point2f point_b) {



}

vector<RotatedRect> armorDetect(vector<RotatedRect> vEllipse)
{
	vector<RotatedRect> vRlt;
	RotatedRect armor;
	int nL, nW;
	int _long1 = 0, _short1 = 0, _long2 = 0, _short2 = 0;
	double dAngle;
	vRlt.clear();

	if (vEllipse.size() < 2)
		return vRlt;
	for (unsigned int nI = 0; nI < vEllipse.size() - 1; nI++)
	{
		Point2f temp_p_point = get_arror(vEllipse[nI]);
		if (abs(temp_p_point.y) / abs(temp_p_point.x) < 0.25)continue;

		for (unsigned int nJ = nI + 1; nJ < vEllipse.size(); nJ++)
		{
			Point2f temp_p_point = get_arror(vEllipse[nI]);
			if (abs(temp_p_point.y) / abs(temp_p_point.x) < 0.25)continue;


			dAngle = abs(vEllipse[nI].angle - vEllipse[nJ].angle);
			while (dAngle > 180)
				dAngle -= 180;

			if (
				//	abs(get_cos_angle(get_arror(vEllipse[nI]),get_arror(vEllipse[nJ])))>cos(5/180*CV_PI)
				(dAngle < T_ANGLE_THRE || 180 - dAngle < T_ANGLE_THRE)
				&&
				abs(vEllipse[nI].size.height - vEllipse[nJ].size.height) < (vEllipse[nI].size.height + vEllipse[nJ].size.height) / T_SIZE_THRE
				&&
				abs(vEllipse[nI].size.width - vEllipse[nJ].size.width) < (vEllipse[nI].size.width + vEllipse[nJ].size.width) / T_SIZE_THRE
				)
			{
				armor.center.x = (vEllipse[nI].center.x + vEllipse[nJ].center.x) / 2; //装甲中心的x坐标 
				armor.center.y = (vEllipse[nI].center.y + vEllipse[nJ].center.y) / 2; //装甲中心的y坐标
				armor.angle = (vEllipse[nI].angle + vEllipse[nJ].angle) / 2;   //装甲所在旋转矩形的旋转角度
				if (180 - dAngle < T_ANGLE_THRE)
					armor.angle += 90;
				nL = (vEllipse[nI].size.height + vEllipse[nJ].size.height) / 2; //装甲的高度
				nW = sqrt((vEllipse[nI].center.x - vEllipse[nJ].center.x) * (vEllipse[nI].center.x - vEllipse[nJ].center.x) +
					(vEllipse[nI].center.y - vEllipse[nJ].center.y) * (vEllipse[nI].center.y - vEllipse[nJ].center.y));
				armor.size.height = nL < nW ? nL : nW;
				armor.size.width = nL >= nW ? nL : nW;

				Point2f pt[12];
				vEllipse[nI].points(pt);
				vEllipse[nJ].points(pt + 4);
				armor.points(pt + 8);
				if (fun2(pt)) {
					if (armor.size.width / armor.size.height<3 && armor.size.width / armor.size.height>2)
					{
						//		Point2f temp_p_point = get_arror(armor);
						//	if(abs(temp_p_point.y)/abs(temp_p_point.x)<1.7321)
						vRlt.push_back(armor); //将找出的装甲的旋转矩形保存到vector
						//cout << armor.size.width / armor.size.height << endl;
					}
				}

			}
		}
	}
	return vRlt;
}
bool fun2(Point2f pt[]) {

	vector<Point>check1;
	vector<Point>check2;
	int count_point = 0;
	for (int count = 0; count < 8; count++) {
		if (count < 4) {
			check1.push_back((Point)pt[count]);
		}
		else {
			check2.push_back((Point)pt[count]);
		}
	}
	for (int count = 8; count < 12; count++) {
		if (pointPolygonTest(check1, pt[count], false) >= 0 || pointPolygonTest(check2, pt[count], false) >= 0) {
			count_point++;
		}
	}
	if (count_point >= 2)
	{
		return true;
	}
	else {
		return  false;
	}
}

void drawBox(RotatedRect box, Mat img, Scalar color)
{
	Point2f pt[4];
	box.points(pt);
	//Scalar(B,G,R)
	circle(img, box.center, 3, color, -1);//中心点
										  //circle(img, pt[0], 5, Scalar(255,255,255), -1);
										  //circle(img, pt[1], 5, color, -1);
										  //box.points(pt); //计算二维盒子顶点 
	line(img, pt[0], pt[1], color, 3, 8, 0);
	line(img, pt[1], pt[2], color, 3, 8, 0);
	line(img, pt[2], pt[3], color, 3, 8, 0);
	line(img, pt[3], pt[0], color, 3, 8, 0);
}

float get_cos_angle(Point2f point_a, Point2f point_b) {
	float long_a = sqrtf(pow(point_a.x, 2) + pow(point_a.y, 2));
	float long_b = sqrtf(pow(point_b.x, 2) + pow(point_b.y, 2));
	float _up = point_a.x*point_b.x + point_b.y*point_b.y;
	return _up / (long_a*long_b);
}

void DesColor(Mat src1, int a, Mat dst, int min, int max) {
	if (a == RED) {


		int height = src1.rows;
		int width = src1.cols;
		omp_set_num_threads(8);
#pragma omp parallel for

		//int nc = src.channels();	
		for (int row = 0; row < height; row++) {
			for (int col = 0; col < width; col++) {
				//if (nc == 1) {			
				//int gray = gray_src.at<uchar>(row, col);	
				//gray_src.at<uchar>(row, col) = 255 - gray;		
				//}			else if (nc == 3) {			
				int b = src1.at<Vec3b>(row, col)[0];
				int g = src1.at<Vec3b>(row, col)[1];
				int r = src1.at<Vec3b>(row, col)[2];
				//dst.at<Vec3b>(row, col)[0] = 255 - b;		
				//	dst.at<Vec3b>(row, col)[1] = 255 - g;		
				//dst.at<Vec3b>(row, col)[2] = 255 - r;
				if ((r >= 150 && r <= max))
				{
					dst.at<uchar>(row, col) = 255;
				}
				else {
					dst.at<uchar>(row, col) = 0;
				}
			}
		}

		//for (int nI = 0; nI<src1.rows; nI++)
		//{
		//	uchar* pchar1 = src1.ptr<uchar>(nI);
		//	uchar* pchar2 = src2.ptr<uchar>(nI);
		//	uchar* pchar3 = dst.ptr<uchar>(nI);
		//	for (int nJ = 0; nJ <src1.cols; nJ++)
		//	{
		//		if (pchar1[nJ] - pchar2[nJ]> nThre) //
		//		{
		//			pchar3[nJ] = 255;
		//		}
		//		else
		//		{
		//			pchar3[nJ] = 0;
		//		}
		//	}
		//}


	}
	else if (a == BLUE) {


		int height = src1.rows;
		int width = src1.cols;
		omp_set_num_threads(8);
#pragma omp parallel for

		for (int row = 0; row < height; row++) {
			for (int col = 0; col < width; col++) {
				int b = src1.at<Vec3b>(row, col)[0];
				int g = src1.at<Vec3b>(row, col)[1];
				int r = src1.at<Vec3b>(row, col)[2];

				if (
					//	(b >= 200 &&b <= 255)&& 
					(g >= 100 && g <= 255) &&
					(r >= 0 && r <= 5) || (b - r) >= 100)
				{
					dst.at<uchar>(row, col) = 255;
				}
				else {
					dst.at<uchar>(row, col) = 0;
				}
			}
		}

	}
	else {
		cvtColor(src1, dst, CV_BGR2GRAY);
		//dst = src1;
	}



}

void find_white(Mat *src1, Mat *dst, int R, int G, int B) {
	int height = src1->rows;
	int width = src1->cols * src1->channels();
	int b = 0, g = 0, r = 0;

	omp_set_num_threads(8);
#pragma omp parallel for
	for (int row = 0; row < height; row++) {
		uchar * mid_line = src1->ptr<uchar>(row);
		uchar * cur_line = dst->ptr<uchar>(row);
		for (int col = 0; col < width; col += 3) {
			/*b = mid_line[col];
			g = mid_line[col+1];
			r = mid_line[col + 2];

			if ((b >= B && b <= 255) && (g >= G && g <= 255) && (r >=R && r <= 255))
			{
			*cur_line = 255;
			cur_line++;
			}
			else {
			*cur_line = 0;
			cur_line++;
			}*/
			if ((mid_line[col] >= B) && (mid_line[col + 1] >= G) && (mid_line[col + 2] >= R))
			{
				*cur_line = 255;
				cur_line++;
			}
			else {
				*cur_line = 0;
				cur_line++;
			}

		}
	}






}

void find_white_color(Mat *src1, Mat *dst_white, int R, int G, int B, Mat *dst_color, int target_color, int min, int max) {
	int height = src1->rows;
	int width = src1->cols * src1->channels();
	//int b = 0, g = 0, r = 0,



	//	omp_set_num_threads(8);
	//#pragma omp parallel for
	if (target_color == BLUE) {

		omp_set_num_threads(8);
#pragma omp parallel for
		for (int row = 0; row < height; row++) {
			uchar * raw = src1->ptr<uchar>(row);
			uchar * white = dst_white->ptr<uchar>(row);
			uchar * color = dst_color->ptr<uchar>(row);
			for (int col = 0; col < width; col += 3) {
				/*int b = raw[col];
				int g = raw[col + 1];
				int r = raw[col + 2];*/
				//find_whilt
				if (
					/*(raw[col] >= B) && (raw[col + 1] >= G) && (raw[col + 2] >= R)
					||
					(raw[col] <= 170) && (raw[col + 1] >= 200) && (raw[col + 2] >= 200)*/
					(raw[col] >= 200) && (raw[col + 1] >= 200) && (raw[col + 2] >= 200)
					)
				{
					*white = 255;
				}
				else {
					*white = 0;
				}
				white++;

				if (
					//	(b >= 200 &&b <= 255)&& 
					//(raw[col + 1] >= 100 && raw[col + 1] <= 255) &&
					//(raw[col + 2] >= 0 && raw[col + 2] <= 5) ||
					(raw[col] - raw[col + 2]) >= 130)
				{
					*color = 255;
				}
				else {
					*color = 0;
				}
				color++;
			}

		}
	}
	else if (target_color == RED) {
		omp_set_num_threads(8);
#pragma omp parallel for

		for (int row = 0; row < height; row++) {
			uchar * raw = src1->ptr<uchar>(row);
			uchar * white = dst_white->ptr<uchar>(row);
			uchar * color = dst_color->ptr<uchar>(row);
			for (int col = 0; col < width; col += 3) {
				int	b = raw[col];
				int	g = raw[col + 1];
				int	r = raw[col + 2];
				if ((raw[col] >= B) && (raw[col + 1] >= G) && (raw[col + 2] >= R)

					||
					(raw[col] <= 170) && (raw[col + 1] >= 200) && (raw[col + 2] >= 200))
				{
					*white = 255;
				}
				else {
					*white = 0;
				}
				white++;

				if (
					//	(b >= 200 &&b <= 255)&& 
					(raw[col + 1] >= 100 && raw[col + 1] <= 255) &&
					(raw[col] >= 0 && raw[col] <= 5) ||
					(raw[col + 2] - raw[col]) >= 100)
				{
					*color = 255;
				}
				else {
					*color = 0;
				}
				color++;
			}
		}
	}
	else {
		cvtColor(*src1, *dst_color, CV_BGR2GRAY);
		cvtColor(*src1, *dst_white, CV_BGR2GRAY);
	}
}

void find_2(Mat *target, vector<Vec4i> hi, vector <vector<Point>> temp) {
	for (int a = hi.size() - 1; a >= 0; --a) {
		bool flag = true;
		Vec4i y = hi[a];
		if (y[3] != -1 && y[2] == -1) {
			drawContours(*target, temp, a, Scalar(0, 0, 0), 1);
		}

	}
}
RotatedRect fun(vector<RotatedRect >test) {
	if (up_flag < 0) {
		Point2f back_center;
		Size back_size;
		RotatedRect back(Point2f(0, 0), Size2f(0, 0), 0);
		int count = -1;
		float _long, _short;
		float min = 5;
		for (int temp = test.size() - 1; temp >= 0; temp--) {
			_long = test[temp].size.height >= test[temp].size.width ? test[temp].size.height : test[temp].size.width;
			_short = test[temp].size.height < test[temp].size.width ? test[temp].size.height : test[temp].size.width;
			if (_long / _short < min) {
				min = _long / _short;
				count = temp;
			}
		}
		if (count<0) {
			return back;
		}
		else {
			up = test[count];
			up_flag = CILLING;
			return test[count];
		}
	}
	else {
		vector<RotatedRect >temp_Rect;
		vector<RotatedRect >group1;
		vector<RotatedRect >group2;
		float chang_kuan_bi = 3;
		float _long, _short;
		float max_cong_die_lv = 0;
		int back;
		float cong_die_lv;
		for (int count = test.size() - 1; count >= 0; count--) {
			_long = test[count].size.height >= test[count].size.width ? test[count].size.height : test[count].size.width;
			_short = test[count].size.height < test[count].size.width ? test[count].size.height : test[count].size.width;
			if (_long / _short <= chang_kuan_bi) {
				temp_Rect.push_back(test[count]);
			}
		}
		Rect temp_rect = up.boundingRect();
		Rect now_rect, _rect;
		int area_rect, area_up = temp_rect.area();

		for (int count = temp_Rect.size() - 1; count >= 0; count--) {
			now_rect = temp_Rect[count].boundingRect();
			_rect = now_rect&temp_rect;
			area_rect = _rect.area();

			cong_die_lv = area_rect / now_rect.area();
			if (cong_die_lv > max_cong_die_lv) {
				max_cong_die_lv = cong_die_lv;
				back = count;
			}
		}
		if (max_cong_die_lv >= 0.3) {
			up = temp_Rect[back];
			up_flag = CILLING;
			return temp_Rect[back];
		}
		if (max_cong_die_lv > 0 && max_cong_die_lv < 0.3) {
			up_flag--;
			return up;
		}
		if (max_cong_die_lv <= 0) {
			up_flag -= 2;
			return up;
		}

	}
}

float get_point_to_point(Point2f point_a, Point2f point_b) {
	return sqrtf(pow(point_a.x - point_b.x, 2) + pow(point_a.y - point_b.y, 2));
}
Point2f get_arror(RotatedRect target) {
	Point2f pt[4];
	target.points(pt);
	float _long = target.size.height>target.size.width ? target.size.height : target.size.width;
	float _short = target.size.height < target.size.width ? target.size.height : target.size.width;

	if (get_point_to_point(pt[0], pt[1]) == _short)
	{
		Point2f back(target.center.x - (pt[0].x + pt[1].x) / 2, target.center.y - (pt[0].y + pt[1].y) / 2);
		return back;
	}
	else {
		Point2f back(target.center.x - (pt[0].x + pt[3].x) / 2, target.center.y - (pt[0].y + pt[3].y) / 2);
		return back;
	}


}

void read_ini(wchar_t* text, int *target) {
	GetPrivateProfileString(L"cap_config", L"COM", L"N", text, 16, L"./RM_config.ini");
	*target=GetPrivateProfileInt(L"cap_config", L"target", 0, L"./RM_config.ini");
}