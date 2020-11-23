#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

int main()
{
	cout << "Built with opencv" << CV_VERSION << endl;
	VideoCapture cap(0);
	cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
	cap.set(cv::CAP_PROP_FPS, 60);
	cout << cap.get(cv::CAP_PROP_FPS) << endl;
	cv::namedWindow("right");
	cv::namedWindow("left");
	Mat frame, right_f, left_f;
	
	if (!cap.isOpened())
	{
		cout << "open camera failed. " << endl;
		return -1;
	}

	while (true)
	{		
		cap >> frame;
		
		int w = frame.cols;
		int h = frame.rows;
			
		if (!frame.empty())
		{
			right_f = cv::Mat(frame, cv::Rect(0, 0, w / 2, h));
			left_f = cv::Mat(frame, cv::Rect(w / 2, 0, w / 2, h));
			cv::putText(right_f, "right", cv::Point(0, 25), cv::FONT_HERSHEY_COMPLEX, 0.8, 255, 1);
			cv::putText(left_f, "left", cv::Point(0, 25), cv::FONT_HERSHEY_COMPLEX, 0.8, 255, 1);
			imshow("camera", frame);
			cv::imshow("right", right_f);
			cv::imshow("left", left_f);
			
		}		
		if (waitKey(30) > 0)
		{
			break;
		}
	}
	return 0;


}