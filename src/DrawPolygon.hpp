#include <boost/shared_ptr.hpp> 
#include <opencv2/opencv.hpp>
#include <iostream>
#include <Windows.h>
using namespace std;

class PolygonDrawer {
public:

	std::string window_name_;
	bool done_;
	bool l_pushed;
	cv::Point current_;
	std::vector<cv::Point> points_;
	boost::shared_ptr<cv::Mat> imgPtr;
	cv::Scalar WORKING_LINE_COLOR;
	cv::Scalar FINAL_LINE_COLOR;
	int width;
	int height;

	PolygonDrawer(const std::string window_name, cv::Mat img, int w_width, int w_height) {
		WORKING_LINE_COLOR = (255, 0, 0);
		FINAL_LINE_COLOR = (0, 255, 0);
		window_name_ = window_name;
		done_ = false;
		l_pushed = false;
		current_ = cv::Point(0, 0); // Current position, so we can draw the line-in-progress
		imgPtr.reset(new cv::Mat(img));
		width = w_width;
		height = w_height;

	}

	static void onMouse(int event, int x, int y, int f, void* data) {
		PolygonDrawer *curobj = reinterpret_cast<PolygonDrawer*>(data);
		if (curobj->done_) // Nothing more to do
			return;
		if (event == cv::EVENT_MOUSEMOVE)
			curobj->current_ = cv::Point(x, y);
		else if (event == cv::EVENT_LBUTTONDOWN) {
			// Left click means adding a point at current position to the list of points
			printf("Adding point #%zu with position(%d,%d) \n", curobj->points_.size(), x, y);
			curobj->points_.push_back(cv::Point(x, y));
			curobj->l_pushed = true; 
		}
		else if (event == cv::EVENT_RBUTTONDOWN)
		{
			curobj->points_.pop_back();
			//if (curobj->l_pushed)
			//	cv::line(*curobj->imgPtr, curobj->points_[curobj->points_.size() - 1], cv::Point(x, y), cv::Scalar(255, 0, 0), 3.0);
			//curobj->l_pushed = false;
		}
		else if (event == cv::EVENT_FLAG_SHIFTKEY) {
			// Right click means we're done
			printf("Completing polygon with %zu points \n", curobj->points_.size());
			curobj->done_ = true;
		}
	}

	int run() {
		cv::namedWindow(window_name_);
		cv::resizeWindow(window_name_, height, width);
		cv::setMouseCallback(window_name_, onMouse, this);
		

		while (!done_) {
			
			cv::Mat img;
			imgPtr->copyTo(img);
			if (points_.size() > 0)
			{
				const cv::Point *pts = (const cv::Point*) cv::Mat(points_).data;
				int npts = cv::Mat(points_).rows;
				cv::polylines(img, &pts, &npts, 1, false, cv::Scalar(255, 0, 0));
				cv::line(img, points_[points_.size() - 1], current_, cv::Scalar(0, 255, 0), 2.0);
			}
			cv::imshow(window_name_, img);
			//cv::waitKey(1);
			if (cv::waitKey(1) == VK_SPACE)
				done_ = true;
		}
		const cv::Point *pts = (const cv::Point*) cv::Mat(points_).data;
		int npts = cv::Mat(points_).rows;

		// user finished entering the polygon points
		if (points_.size() > 0) {
			cv::Mat img;
			imgPtr->copyTo(img);
			cv::Mat img2;
			imgPtr->copyTo(img2);
			//cv::fillPoly(img, &pts, &npts, 1, cv::Scalar(255, 0, 255));
			cv::fillConvexPoly(img, pts, npts, cv::Scalar(255, 0, 255));
			cv::Mat show_img = 0.3 * img + 0.7 * img2;
			cv::imshow(window_name_, show_img);
			if (cv::waitKey() == VK_SPACE)
			{
				cv::destroyWindow(window_name_);
				return 1;
			}
			else if (cv::waitKey() == VK_ESCAPE)
			{
				cv::destroyWindow(window_name_);
				return 0;
			}
		}
	}

	vector<cv::Point> getMaskData()
	{
		vector<cv::Point> data;

		const cv::Point *pts = (const cv::Point*) cv::Mat(points_).data;
		int npts = cv::Mat(points_).rows;
		cout << npts << endl;
		cv::Mat img;
		imgPtr->copyTo(img);
		//cv::fillPoly(img, &pts, &npts, 1, cv::Scalar(0, 0, 0));
		cv::fillConvexPoly(img, pts, npts, cv::Scalar(0, 0, 0));
		for (int m = 0; m < img.rows; m++)
			for (int n = 0; n < img.cols; n++)
				if (img.at<cv::Vec3b>(m, n) == cv::Vec3b(0, 0, 0))
					data.push_back(cv::Point(n, m));

		return data;
	}

};