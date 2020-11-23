#include<iostream>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/video/video.hpp>

using namespace std;

int main()
{
	char left_filename[500];
	char right_filename[500];
	int n = 0;
	int m = 0;

	cv::VideoCapture cap("7.avi");
	cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
	cap.set(cv::CAP_PROP_FPS, 60);
	/*cv::VideoCapture cap(0);
	cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
	cap.set(cv::CAP_PROP_FPS, 60);*/
	cv::Mat frame,frame_r, frame_l;

	if (!cap.isOpened())
	{
		cout << "cann't open the camera" << endl;
		return -1;
	}

	while (true)
	{
		cap >> frame;
		int w = frame.cols;
		int h = frame.rows;
		if (!frame.empty())
		{
			frame_l = cv::Mat(frame, cv::Rect(0, 0, w / 2, h));
			frame_r = cv::Mat(frame, cv::Rect(w / 2, 0, w / 2, h));
			/*cv::putText(frame_r, "right", cv::Point(0, 25), cv::FONT_HERSHEY_COMPLEX, 0.8, 255, 1);
			cv::putText(frame_l, "left", cv::Point(0, 25), cv::FONT_HERSHEY_COMPLEX, 0.8, 255, 1);*/
			cv::imshow("camera", frame);
			//cv::imshow("right", frame_r);
			//cv::imshow("left", frame_l);
		}
		
		switch (cv::waitKey(30))
		{
		case 27:   //按Esc键 退出  

			return 0;

		case 13:   //按 Enter键 拍照

			sprintf_s(left_filename, "D:\\桌面\\project1\\biaoding\\left_frame\\l_text1%d.jpg", n++);
			sprintf_s(right_filename, "D:\\桌面\\project1\\biaoding\\right_frame\\r_text1%d.jpg", m++);
			cv::imwrite(left_filename, frame_l);
			cv::imwrite(right_filename, frame_r);
			break;
			
		}

	}

	return 0;
}