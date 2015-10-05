#include <pylon_camera/image_rectifier.h>

using namespace cv;

namespace pylon_camera
{

ImageRectifier::ImageRectifier() :
                    rect_map_x_(),
                    rect_map_y_()
{
}

ImageRectifier::~ImageRectifier()
{
}

void ImageRectifier::setupRectifyingMap(const Mat& cam_matrix, const Mat& dist_coefficients,
    const int& img_width,
    const int&img_height)
{
    cv::Mat I = cv::Mat_<double>::eye(3, 3);
    // md 20.05.15
    // newCameraMatrix (4th param of initUndistortRectifyMap) should be the cameraMatrix
    // if not -> shift in the undistorted img
    cv::initUndistortRectifyMap(cam_matrix,
                                dist_coefficients,
                                I,
                                cam_matrix,
                                cv::Size(img_width, img_height),
                                CV_32FC1,
                                rect_map_x_,
                                rect_map_y_);
}

void ImageRectifier::rectify(const Mat& src, Mat& dst)
{
//	if(rect_map_x_.empty() || rect_map_y_.empty())
//	{
//		std::cerr << "ERROR while trying to rectify: No map available. Call 'setupRectifyingMap()' fist!" << std::endl;
//		return;
//	}
    // md 20.05.15
    // cv::undistort generates for each img 'n_rows' times the map for rectifiyng the row (cv::initUndistortRectifyMap())
    // faster: cv::initUndistortRectifyMap() only once, and then cv::remap() for each img

    cv::remap(src, dst, rect_map_x_, rect_map_y_, INTER_LINEAR, BORDER_CONSTANT);
}

} /* namespace pylon_camera */
