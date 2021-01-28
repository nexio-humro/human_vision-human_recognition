#ifndef OPENCV_TO_DLIB_FUNCTIONS_HPP
#define OPENCV_TO_DLIB_FUNCTIONS_HPP

#include <iostream>
#include <opencv2/core/mat.hpp>
#include <dlib/matrix.h>
#include <dlib/image_io.h>
#include <dlib/opencv/cv_image.h>

namespace OTDF
{
	dlib::matrix<dlib::rgb_pixel> cvMat2dlibMatrix(cv::Mat& input);
}

#endif
