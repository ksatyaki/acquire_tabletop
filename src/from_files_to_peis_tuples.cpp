#include <boost/filesystem.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <cv_bridge/cv_bridge.h>
#include <doro_msgs/SiftDescriptor.h>

#include <ros_over_peis/subscriber.h>
#include <ros_over_peis/publisher.h>

#include <string>
#include <string.h>

void processFile (std::string file_name)
{
	printf("\nReading file: %s.", file_name.c_str());
	cv_bridge::CvImage _image;
	_image.encoding = "mono8";
	_image.image = cv::imread(file_name, CV_LOAD_IMAGE_GRAYSCALE);

	size_t position_of_end = file_name.substr(file_name.find_last_of("/") + 1).find_first_of("_") == std::string::npos ? file_name.substr(file_name.find_last_of("/") + 1).find(".") : file_name.substr(file_name.find_last_of("/") + 1).find_first_of("_");

	std::string object_name = file_name.substr(file_name.find_last_of("/") + 1, position_of_end);
	printf("\nObject name is: %s.", object_name.c_str());

	ros_over_peis::Publisher <doro_msgs::SiftDescriptor> image_to_peis_pub ( (object_name + ".sift_descriptor.header.update").c_str(), 9898);

	peiskmt_subscribe(9898, (object_name + ".sift_descriptor.header.update").c_str());
	peiskmt_subscribe(9898, (object_name + ".sift_descriptor.data.update").c_str());

	// Compute sift features.
	// Increase this value if each of the images contain more than one view of the object.
	cv::SiftFeatureDetector detector(300);
	cv::SiftDescriptorExtractor extractor;

	std::vector <cv::KeyPoint> keypoint_;
	cv::Mat descriptor_;

	detector.detect(_image.image, keypoint_);
	extractor.compute(_image.image, keypoint_, descriptor_);

	doro_msgs::SiftDescriptor s;
	s.elem_size = descriptor_.elemSize();
	s.elem_type = descriptor_.type();
	s.cols = descriptor_.cols;
	s.rows = descriptor_.rows;
	// This field is the most important. This is the object name!!!
	s.id = object_name;
	//s.flags = descriptor_.flags;

	if(object_name == "ibumetin")
		std::cout<<descriptor_;

	const size_t data_size = s.cols * s.rows * s.elem_size;
	uchar* actual_data = descriptor_.ptr();

	boost::shared_ptr<uchar> copy_ = boost::shared_ptr<uchar> (new uchar[data_size]);
	memcpy(copy_.get(), actual_data, data_size);

	peiskmt_setRemoteTuple(9898, (object_name + ".sift_descriptor.data.update").c_str(), data_size, copy_.get(), "descriptor_matrix", PEISK_ENCODING_BINARY);
	image_to_peis_pub.publish(s);

	printf("\n***************\n$$$ Success $$$\n***************\n");

	usleep(200);

}

void traverseDirectories(const boost::filesystem::path &base_dir, const std::string &extension)
{
	if (!boost::filesystem::exists (base_dir) && !boost::filesystem::is_directory (base_dir))
	{
		printf("Non-existent directory man!\n");
		return;
	}

	for (boost::filesystem::directory_iterator it (base_dir); it != boost::filesystem::directory_iterator (); ++it)
	{
		if (boost::filesystem::is_directory (it->status ()))
		{
			std::stringstream ss;
			ss << it->path ();
			//pcl::console::print_highlight ("Loading %s (%lu models loaded so far).\n", ss.str ().c_str (), (unsigned long)models.size ());
			traverseDirectories (it->path (), extension);
		}
		if (boost::filesystem::is_regular_file (it->status ()) && boost::filesystem::extension (it->path ()) == extension)
		{
			processFile( (base_dir / it->path().filename()).string() );
		}
	}
}

int main(int argn, char* args[])
{
	if(argn < 2)
	{
		printf("Must provide a directory to traverse. Usage: from_files_to_peis_tuples <dir_with_images>");
		return 0;
	}
	else
	{
		argn --;
		peiskmt_initialize(&argn, args);
		traverseDirectories(args[1], ".jpg");
	}

}
