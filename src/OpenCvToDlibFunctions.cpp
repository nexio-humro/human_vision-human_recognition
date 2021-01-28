#include <OpenCvToDlibFunctions.hpp>

namespace OTDF
{
	dlib::matrix<dlib::rgb_pixel> cvMat2dlibMatrix(cv::Mat& input)
	{
		dlib::matrix<dlib::rgb_pixel> matrix_image;
		
		if( (input.empty() == false) && (input.channels() == 3) )
		{
			dlib::assign_image(matrix_image, dlib::cv_image<dlib::rgb_pixel>(input));
		}
		
		return matrix_image;
	}
}
