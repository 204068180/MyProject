#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/calib3d.hpp>
#include<opencv2/imgcodecs.hpp>
#include<opencv2/core/utility.hpp>
#include<opencv2/video/video.hpp>
#include<iostream>

using namespace std;

cv::Mat imgl,imgls, imgr;

const int imagewidth = 640;
const int imagehight = 480;
const cv::Size imagesize = cv::Size(imagewidth,imagehight);

cv::Mat disp, disp8;
cv::Mat xyz;

cv::Mat left_gray, right_gray;
cv::Mat R1, R2, P1, P2, Q;
cv::Mat rectifyImageL2, rectifyImageR2;
cv::Rect validROIL, validROIR;


bool selectObject = false;    //是否选择对象
cv::Rect selection;      //定义矩形选框
cv::Point origin;
cv::Vec3f  point3;
float d;

cv::Point ptL, ptR;
cv::Rect rect;
long double sum = 0;
long int n;

/*给深度图上色*/
void GenerateFalseMap(cv::Mat &src, cv::Mat &disp)
{
	// color map  
	float max_val = 255.0f;
	float map[8][4] = { { 0,0,0,114 },{ 0,0,1,185 },{ 1,0,0,114 },{ 1,0,1,174 },
	{ 0,1,0,114 },{ 0,1,1,185 },{ 1,1,0,114 },{ 1,1,1,0 } };
	float sum = 0;
	for (int i = 0; i < 8; i++)
		sum += map[i][3];

	float weights[8]; // relative   weights  
	float cumsum[8];  // cumulative weights  
	cumsum[0] = 0;
	for (int i = 0; i < 7; i++) {
		weights[i] = sum / map[i][3];
		cumsum[i + 1] = cumsum[i] + map[i][3] / sum;
	}

	int height_ = src.rows;
	int width_ = src.cols;
	// for all pixels do  
	for (int v = 0; v < height_; v++) {
		for (int u = 0; u < width_; u++) {

			// get normalized value  
			float val = std::min(std::max(src.data[v*width_ + u] / max_val, 0.0f), 1.0f);

			// find bin  
			int i;
			for (i = 0; i < 7; i++)
				if (val < cumsum[i + 1])
					break;

			// compute red/green/blue values  
			float   w = 1.0 - (val - cumsum[i])*weights[i];
			uchar r = (uchar)((w*map[i][0] + (1.0 - w)*map[i + 1][0]) * 255.0);
			uchar g = (uchar)((w*map[i][1] + (1.0 - w)*map[i + 1][1]) * 255.0);
			uchar b = (uchar)((w*map[i][2] + (1.0 - w)*map[i + 1][2]) * 255.0);
			//rgb内存连续存放  
			disp.data[v*width_ * 3 + 3 * u + 0] = b;
			disp.data[v*width_ * 3 + 3 * u + 1] = g;
			disp.data[v*width_ * 3 + 3 * u + 2] = r;
		}
	}
}


/*****描述：鼠标操作回调*****/
void OnMouse(int event, int x, int y, int flag, void *ustg)
{
	if (selectObject)
	{
		selection.x = MIN(x, origin.x);
		selection.y = MIN(y, origin.y);
		selection.width = std::abs(x - origin.x);
		selection.height = std::abs(y - origin.y);
	}

	switch (event)
	{
	case cv::EVENT_LBUTTONDOWN:   //鼠标左按钮按下的事件
		origin = cv::Point(x, y);
		selection = cv::Rect(x, y, 0, 0);
		selectObject = true;
		//cout << origin << "in world coordinate is: " << xyz.at<Vec3f>(origin) << endl;
		point3 = xyz.at<cv::Vec3f>(origin);
		point3[0];
		//cout << "point3[0]:" << point3[0] << "point3[1]:" << point3[1] << "point3[2]:" << point3[2]<<endl;
		cout << "世界坐标：" << endl;
		cout << "x: " << point3[0] << "  y: " << point3[1] << "  z: " << point3[2] << endl;
		d = point3[0] * point3[0] + point3[1] * point3[1] + point3[2] * point3[2];
		d = sqrt(d);   //mm
	   // cout << "距离是:" << d << "mm" << endl;

		d = d / 10.0;   //cm
		cout << "距离是:" << d << "cm" << endl;

		// d = d/1000.0;   //m
		// cout << "距离是:" << d << "m" << endl;

		break;
	case cv::EVENT_LBUTTONUP:    //鼠标左按钮释放的事件
		selectObject = false;
		if (selection.width > 0 && selection.height > 0)
			break;
	}
}

cv::Mat cameraMatrixL, cameraMatrixR, distcoeffL, distcoeffR;
cv::Mat R, T;
cv::Mat maplx, maply, maprx, mapry;
cv::Mat canvas;
double sf;
int w, h;
char* imagenameL = new char[3000];
char* imagenameR = new char[3000];
int a = 0;
/*----------------立体矫正--------------*/
void stereo_calib(cv::Mat &imagel,cv::Mat &imager)
{
	cv::FileStorage fs("intrinsics.yml", cv::FileStorage::READ);

	fs["cameraMatrixL"] >> cameraMatrixL;
	fs["cameraDistcoeffL"] >> distcoeffL;
	fs["cameraMatrixR"] >> cameraMatrixR;
	fs["cameraDistcoeffR"] >> distcoeffR;

	fs.open("extrinsics.yml", cv::FileStorage::READ);
	fs["R"] >> R;
	fs["T"] >> T;


	cv::stereoRectify(cameraMatrixL, distcoeffL, cameraMatrixR, distcoeffR, imagesize, R, T, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, -1, imagesize, &validROIL, &validROIR);
	
	cv::initUndistortRectifyMap(cameraMatrixL, distcoeffL, R1, P1, imagesize, CV_32FC1, maplx, maply);
	cv::initUndistortRectifyMap(cameraMatrixR, distcoeffR, R2, P2, imagesize, CV_32FC1, maprx, mapry);
	
	cv::remap(imagel, rectifyImageL2, maplx, maply, cv::InterpolationFlags::INTER_LINEAR);
	cv::remap(imager, rectifyImageR2, maprx, mapry, cv::InterpolationFlags::INTER_LINEAR);

	cv::imshow("L1", rectifyImageL2);
	cv::imshow("R1", rectifyImageR2);

	sprintf_s(imagenameL, 3000, "D:\\桌面\\project1\\stero_match\\99\\L\\l_text1%d.jpg", a);   //保存视频的地址和格式
	sprintf_s(imagenameR, 3000, "D:\\桌面\\project1\\stero_match\\99\\R\\r_text1%d.jpg", a);   //保存视频的地址和格式
	a++;
	cv::imwrite(imagenameL, rectifyImageL2);
	cv::imwrite(imagenameR, rectifyImageR2);

	///*-------------左右图像放在同个窗口，并用平行线标齐---------------*/

	/*sf = 600. / MAX(imagesize.width, imagesize.height);
	w = cvRound(imagesize.width * sf);
	h = cvRound(imagesize.height * sf);
	canvas.create(h, w * 2, CV_8UC3);

	cv::Mat canvasPart = canvas(cv::Rect(w * 0, 0, w, h));
	resize(rectifyImageL2, canvasPart, canvasPart.size(), 0, 0, cv::INTER_AREA);
	cv::Rect vroiL(cvRound(validROIL.x*sf), cvRound(validROIL.y*sf),
		cvRound(validROIL.width*sf), cvRound(validROIL.height*sf));
	cv::rectangle(canvasPart, vroiL, cv::Scalar(0, 0, 255), 3, 8);

	canvasPart = canvas(cv::Rect(w, 0, w, h));
	resize(rectifyImageR2, canvasPart, canvasPart.size(), 0, 0, cv::INTER_LINEAR);
	cv::Rect vroiR(cvRound(validROIR.x * sf), cvRound(validROIR.y*sf),
		cvRound(validROIR.width * sf), cvRound(validROIR.height * sf));
	cv::rectangle(canvasPart, vroiR, cv::Scalar(0, 255, 0), 3, 8);

	for (int i = 0; i < canvas.rows; i += 16)
		line(canvas, cv::Point(0, i), cv::Point(canvas.cols, i), cv::Scalar(0, 255, 0), 1, 8);

	cv::imshow("rectified", canvas);*/
}

/*----------------立体匹配--------------*/

int blockSize = 1, uniquenessRatio = 66, numDisparities = 2;
void stereo_match_sgbm(int, void*)
{	
	int numberOfDisparities = ((imagesize.width / 8) + 15) & -16;
	cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0, 16, 3);
	sgbm->setPreFilterCap(32);
	//int blockSize = 9;
	int sgbmWinSize = blockSize > 0 ? blockSize : 3;
	sgbm->setBlockSize(sgbmWinSize);
	int cn = 1;
	sgbm->setP1(8 * cn*sgbmWinSize*sgbmWinSize);
	sgbm->setP2(32 * cn*sgbmWinSize*sgbmWinSize);
	sgbm->setMinDisparity(0);
	sgbm->setNumDisparities(numberOfDisparities);
	sgbm->setUniquenessRatio(uniquenessRatio);
	sgbm->setSpeckleWindowSize(100);
	sgbm->setSpeckleRange(32);
	sgbm->setDisp12MaxDiff(1);

	

	sgbm->setMode(cv::StereoSGBM::MODE_SGBM);

	sgbm->compute(left_gray, right_gray, disp);
	
	disp.convertTo(disp8, CV_8U, 255 / ((numDisparities * 16 + 16)*16.));
	
	cv::Mat structure = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5), cv::Point(-1, -1));
	cv::dilate(disp8, disp8, structure, cv::Point(-1, -1), 1);
	
	cv::medianBlur(disp8, disp8, 3);
	cv::normalize(disp8, disp8, 0, 255,cv::NormTypes::NORM_MINMAX);
	

	cv::imshow("disparity", disp8);

	//cv::Mat color(disp8.size(), CV_8UC3);
	//GenerateFalseMap(disp8, color);//转成彩图
	//imshow("disparity_color", color);

	reprojectImageTo3D(disp, xyz, Q, true);
	xyz = xyz * 16;
	
}

cv::Ptr<cv::StereoBM> bm = cv::StereoBM::create(16, 3);
void stereo_match_bm(int, void*)
{
	bm->setBlockSize(2 * blockSize + 5);     //SAD窗口大小，5~21之间为宜
	bm->setROI1(validROIL);
	bm->setROI2(validROIR);
	bm->setPreFilterCap(31);
	bm->setMinDisparity(0);  //最小视差，默认值为0, 可以是负值，int型
	bm->setNumDisparities(numDisparities * 16 + 16);//视差窗口，即最大视差值与最小视差值之差,窗口大小必须是16的整数倍，int型
	bm->setTextureThreshold(10);
	bm->setUniquenessRatio(uniquenessRatio);//uniquenessRatio主要可以防止误匹配
	bm->setSpeckleWindowSize(100);
	bm->setSpeckleRange(32);
	bm->setDisp12MaxDiff(1);
	bm->compute(left_gray, right_gray, disp);//输入图像必须为灰度图
	disp.convertTo(disp8, CV_8U, 255 / ((numDisparities * 16 + 16)*16.));//计算出的视差是CV_16S格式
	reprojectImageTo3D(disp, xyz, Q, true); //在实际求距离时，ReprojectTo3D出来的X / W, Y / W, Z / W都要乘以16(也就是W除以16)，才能得到正确的三维坐标信息。
	xyz = xyz * 16;
	imshow("disparity", disp8);
}


//void OnMouse(int event, int x, int y, int flag, void *ustg);


char left_filename[500];
char right_filename[500];
int ns = 1;
int m = 1;
clock_t start, finish;
int main()
{
	/*cv::VideoCapture capl("7.avi");
	capl.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
	capl.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
	capl.set(cv::CAP_PROP_FPS, 30);
	cv::Mat frame, imgl, imgr;*/

	imgl= cv::imread("l_text12.jpg");
	imgr= cv::imread("r_text12.jpg");
	
	
	/*----------------图片处理-----------------------*/
	cv::GaussianBlur(imgl, imgl, cv::Size(5, 5), 0, 0);
	cv::GaussianBlur(imgr, imgr, cv::Size(5, 5), 0, 0);
	cv::Mat kernel = (cv::Mat_<char>(3, 3) << 0, -1, 0, -1, 5, -1, 0, -1, 0);
	cv::filter2D(imgl, imgl, -1, kernel);
	cv::filter2D(imgr, imgr, -1, kernel);
	/*----------------图片处理-----------------------*/

	cv::namedWindow("disparity", cv::WINDOW_AUTOSIZE);

	
	/*while (true)
	{
		capl >> frame;
		int w = frame.cols;
		int h = frame.rows;

		imgl = cv::Mat(frame, cv::Rect(0, 0, w / 2, h));
		imgr = cv::Mat(frame, cv::Rect(w / 2, 0, w / 2, h));
		cv::GaussianBlur(imgl, imgl, cv::Size(5, 5), 0, 0);
		cv::GaussianBlur(imgr, imgr, cv::Size(5, 5), 0, 0);
		cv::Mat kernel = (cv::Mat_<char>(3, 3) << 0, -1, 0, -1, 5, -1, 0, -1, 0);
		cv::filter2D(imgl, imgl, -1, kernel);
		cv::filter2D(imgr, imgr, -1, kernel);*/
		

		/*cout << "L = " << capl.get(cv::CAP_PROP_FPS) << endl;
		cout << "R = " << capr.get(cv::CAP_PROP_FPS) << endl;*/

		

		/*cv::imshow("L",imgl);
		cv::imshow("R", imgr);*/

		stereo_calib(imgl, imgr);

		/*if (cv::waitKey(30)== 13)
		{
			sprintf_s(left_filename, "li%d.jpg", ns++);
			sprintf_s(right_filename, "ri%d.jpg", m++);
			cv::imwrite(left_filename, rectifyImageL2);
			cv::imwrite(right_filename, rectifyImageR2);
		}*/

		cv::cvtColor(rectifyImageL2, left_gray, cv::ColorConversionCodes::COLOR_BGR2GRAY);
		cv::cvtColor(rectifyImageR2, right_gray, cv::ColorConversionCodes::COLOR_BGR2GRAY);

		
		// 创建SAD窗口 Trackbar
		cv::createTrackbar("BlockSize:\n", "disparity", &blockSize, 10, stereo_match_sgbm);
		// 创建视差唯一性百分比窗口 Trackbar
	    cv::createTrackbar("UniquenessRatio:\n", "disparity", &uniquenessRatio, 80, stereo_match_sgbm);
		// 创建视差窗口 Trackbar
		cv::createTrackbar("NumDisparities:\n", "disparity", &numDisparities, 5, stereo_match_sgbm);
			start = clock();
			stereo_match_sgbm(0, 0);

			cv::setMouseCallback("L1", OnMouse, 0);
			finish = clock();
			//std::cout << "time = " << double(finish - start) / CLOCKS_PER_SEC << "s" << std::endl;
		

			/*if(cv::waitKey(30)>0)
				break;

	}*/

	cv::waitKey(0);
	return 0;
}



//void OnMouse(int event, int x, int y, int flag, void *ustg)
//{
//	if (event == cv::EVENT_LBUTTONDOWN)
//	{
//		ptL = cv::Point(x, y);
//
//	}
//	if (flag == cv::EVENT_FLAG_LBUTTON)
//	{
//		ptR = cv::Point(x, y);                //终点
//		imgls = rectifyImageL2.clone();
//		cv::rectangle(imgls, ptL, ptR, cv::Scalar(255, 0, 0));
//
//		cv::imshow("L1", imgls);
//	}
//	if (event == cv::EVENT_LBUTTONUP)
//	{
//		if (ptL != ptR)
//		{
//			rect = cv::Rect(ptL, ptR);
//
//
//
//			for (int i = 0; i < rect.width; i++)
//			{
//				for (int j = 0; j < rect.height; j++)
//				{
//					origin = cv::Point(ptL.x + i,ptL.y + j);
//					point3 = xyz.at<cv::Vec3f>(origin);
//					point3[0];
//					cout << "世界坐标：" << endl;
//					cout << "x: " << point3[0] << "  y: " << point3[1] << "  z: " << point3[2] << endl;
//					d = point3[0] * point3[0] + point3[1] * point3[1] + point3[2] * point3[2];
//					d = sqrt(d);   //mm
//					d = d / 10.0;   //cm
//					
//
//					if (d < 1600)
//					{
//						sum = sum + d;
//						n++;
//					}
//				}
//			}
//
//			cout << "距离是:" << sum / n << "cm" << endl;
//			sum = 0;
//			n = 0;
//		}
//	}
//}
