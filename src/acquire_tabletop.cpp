/*
  Copyright (C) 2015  Chittaranjan Srinivas Swaminathan

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>
  
*/

#include "acquire_tabletop/acquire_tabletop.h"

namespace acquire_tabletop {

AcquireTabletopServer::AcquireTabletopServer()
{
	clusters_sub_ = nh_.subscribe("/clusters", 1, &AcquireTabletopServer::clustersCB, this);
	server_ = nh_.advertiseService("acquire_tabletop", &AcquireTabletopServer::serverCB, this);

	camera_info_sub_ = nh_.subscribe ("camera_info", 1, &AcquireTabletopServer::cameraInfoCallback, this);

	cam_interface::subscribeToCAM();
	tf_listener_ = boost::shared_ptr <tf::TransformListener> (new tf::TransformListener (ros::Duration(10.0)));
	prepare();
	subscribed_ = false;
	ROS_INFO("Server Started!");
}

void AcquireTabletopServer::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& camera_info)
{
	projection_matrix_ = boost::shared_ptr <tf::Matrix3x3> (new tf::Matrix3x3);
	for(int i = 0; i < 3; i ++)
	{
		for(int j = 0; j < 3; j ++)
		{
			(*projection_matrix_)[i][j] = camera_info->P[i*4 + j];
		}
	}

	image_width = camera_info->width;
	image_height = camera_info->height;
	camera_info_sub_.shutdown();
}

void AcquireTabletopServer::clustersCB(const doro_msgs::ClusterArrayConstPtr& _clusters)
{
	// Create space for new message
	//clusters_ptr_ = doro_msgs::ClusterArrayPtr (new doro_msgs::ClusterArray);

	ROS_INFO("Fetching clusters.");
	// Copy
	clusters_ptr_ = _clusters;
}

void AcquireTabletopServer::imageCallback(const sensor_msgs::ImageConstPtr& _msg)
{
	//image_ = sensor_msgs::ImagePtr(new sensor_msgs::Image(*_msg));
	image_ = _msg;
}


AcquireTabletopServer::~AcquireTabletopServer()
{
	clusters_sub_.shutdown();
	server_.shutdown();
	ROS_INFO("Subscriptions cancelled.");
	ROS_INFO("Server shutdown.");
}

void AcquireTabletopServer::prepare()
{
	ROS_INFO("Preparing the registry of descriptors...");
	getDescriptorsFromTuples();
	//flase_prep();
}

void AcquireTabletopServer::getDescriptorsFromTuples()
{
	std::vector<std::string> object_names = cam_interface::getAllObjectNamesFromCAM();
	for(std::vector <std::string>::iterator it = object_names.begin(); it != object_names.end(); it++)
		std::cout << *it << " | ";

	int id_for_tuple = CAM_PEIS_ID; //peiskmt_peisid();

	ROS_INFO("\nPreparing ...");
	int i = 0;
	for(std::vector<std::string>::iterator it = object_names.begin(); it != object_names.end(); it++)
	{
		if(it->find("banana") != std::string::npos)
			continue;
		ros_over_peis::Subscriber <doro_msgs::SiftDescriptor> image_from_peis_sub ((*it + ".sift_descriptor.header").c_str());
		doro_msgs::SiftDescriptor::Ptr desc;
		PeisTuple* recd_tuple;

		image_from_peis_sub.getMsg(desc);
		while(!desc)
		{
			image_from_peis_sub.getMsg(desc);
			printf("Getting descriptor tuple %d...\n", i);
		}

		recd_tuple = peiskmt_getTuple(id_for_tuple, std::string(*it + ".sift_descriptor.data").c_str(), PEISK_KEEP_OLD);
		while(!recd_tuple)
		{
			recd_tuple = peiskmt_getTuple(id_for_tuple, std::string(*it + ".sift_descriptor.data").c_str(), PEISK_KEEP_OLD);
			printf("Getting data tuple %d...\n", i);
			sleep(1);
		}
		printf("Deserializing...\n");
		cv::Mat recd_mat(desc->rows, desc->cols, desc->elem_type, (uchar *) recd_tuple->data);
		descriptors_.push_back(recd_mat.clone());
		ids_.push_back(desc->id);
		i++;
	}

	sleep(1);

	//std::cout<<descriptors_[0];

	ROS_INFO("\nDone: Succeeded reading PEIS Tuples for SIFT Descriptors...\n");
}

void AcquireTabletopServer::matchWithRegistry(const cv::Mat& _input_descriptors, const float& tolerance)
{
	cv::FlannBasedMatcher matcher;
	//ROS_INFO("Matcher called with tolerance: %f", tolerance);

	int max_size = 0;

	matches_.clear();
	match_indices_.clear();
	match_numbers_.clear();

	for(int i = 0; i < descriptors_.size(); i++)
	{
		std::vector<cv::DMatch> matches;
		std::vector<cv::DMatch> matches_filtered;

		//ROS_INFO("MATCHING...");
		if(_input_descriptors.empty())
		{
			ROS_INFO("Input descriptors are ruined.");
			return;
		}
		if(descriptors_[i].empty())
		{
			ROS_INFO("Original Descriptors are ruined.");
			return;
		}
		matcher.match(_input_descriptors, descriptors_[i], matches);
		//ROS_INFO("****(((%s)))**** has %d features b4 filtering.", ids_[i].c_str(), matches.size());

		for(std::vector <cv::DMatch>::iterator it = matches.begin(); it != matches.end(); it++)
			for(std::vector <cv::DMatch>::iterator jt = matches.begin() + 1; jt != matches.end(); jt++)
			{
				if(it->distance < 0.55*jt->distance)
				{
					matches_filtered.push_back(*it);
					break;
				}
			}

		if(matches_filtered.size() < 10 * (tolerance))
		{
		  ROS_INFO("[ACQUIRE] (%s has %d features)", ids_[i].c_str(), matches_filtered.size());
			matches_filtered.clear();
		}
		else
		{
		  ROS_INFO("[ACQUIRE] (%s has %d features) --- OK!", ids_[i].c_str(), matches_filtered.size());
		        matches_.push_back(matches_filtered);
			match_indices_.push_back(i);
			match_numbers_.push_back(matches_filtered.size());
		}
		
	}

	int max_match_number = 0;
	for(int i = 0; i < match_numbers_.size(); i++) {
	  if(match_numbers_[i] > max_match_number) {
	    max_match_number = match_numbers_[i];
	    match_numbers_[0] = match_numbers_[i];
	    match_indices_[0] = match_indices_[i];
	  }
	}

	if(match_indices_.size() > 0)
	  ROS_INFO("Multiple matches. But %s wins out.", ids_[match_indices_[0]].c_str());
	ROS_INFO("[ACQUIRE] --------------------------");
	// matches_ = matches_filtered_all[match_index_];

}

cv::Mat AcquireTabletopServer::cutImage(const cv::Mat& input_image, int row_offset, int col_offset, int row_size, int col_size)
{
	if( (input_image.rows < row_offset + row_size) || (input_image.cols < col_offset + col_size) )
	{
		//printf("%d, %d, %d, %d, %d, %d\n", input_image.rows, row_offset, row_size, input_image.cols, col_offset, col_size);
		row_size = input_image.rows - row_offset;
		col_size = input_image.cols - col_offset;

		//std::cout<<"burr";
	}

	cv::Mat temp(row_size, col_size, 0);

	for(int i = 0; i<row_size; i++)
		for(int j = 0; j<col_size; j++)
			temp.data[i*temp.step[0] + j*temp.step[1]] = input_image.data[((row_offset + i)*input_image.step[0])  + (j + col_offset)*input_image.step[1]];

	return temp;
}

bool AcquireTabletopServer::serverCB(AcquireTabletopRequest& request, AcquireTabletopResponse& response)
{
	ROS_INFO("Server Called!");

	for(int ii = 0; ii < 3; ii++)
	{
		for(int jj = 0; jj < 3; jj++)
		{
			std::cout << (*projection_matrix_)[ii][jj] <<" | ";
		}
		std::cout << std::endl;
	}
	float tole;

	if(request.tolerance == 0.0 || request.tolerance > 1.0 || request.tolerance < 0.0)
		tole = 1.0;
	else
		tole = request.tolerance;

	bool return_value;
	std::string object_name = "unknown_object_";

	clusters_ptr_.reset();

	ros::param::set("/cluster_extraction_enable", true);

	while( !clusters_ptr_ && ros::ok())
	{
		if(clusters_ptr_)
			ROS_INFO("We have the clusters.");
		usleep(100000);
	}

	ros::param::set("/cluster_extraction_enable", false);

	// Get the image from the rgb/image_raw topic.

	image_raw_sub_ = nh_.subscribe("image", 4, &AcquireTabletopServer::imageCallback, this);
	while(!image_)
	{
		//ROS_INFO("Waiting for the camera image...");
		usleep(100000);
	}

	image_raw_sub_.shutdown();

	//ROS_INFO("Processing image...");
	cv_bridge::CvImagePtr test_image;

	sensor_msgs::ImageConstPtr _image_for_processing_ = image_;
	test_image = cv_bridge::toCvCopy(_image_for_processing_);

	bool pruned = false;

	std::vector <std::string> cluster_names;
	int unknown_num = 0;
	int unseen_num = 0;

	tf::StampedTransform camera_to_base_link;

	try
	{
		tf_listener_->waitForTransform(image_->header.frame_id, "ptu_tilt_motor_link", ros::Time(0), ros::Duration(1));
		tf_listener_->lookupTransform(image_->header.frame_id, "ptu_tilt_motor_link", ros::Time(0), camera_to_base_link);
	}

	catch(tf::TransformException& ex)
	{
		ROS_INFO("COCKED-UP TRANSFORM. ACQUIRE FAILED! Why: %s", ex.what());
		return true;
	}

	for(int i = 0; i < clusters_ptr_->clusters.size(); i++)
	{

		tf::Vector3 ptA(clusters_ptr_->clusters[i].a.x, clusters_ptr_->clusters[i].a.y, clusters_ptr_->clusters[i].a.z);
		tf::Vector3 ptB(clusters_ptr_->clusters[i].b.x, clusters_ptr_->clusters[i].b.y, clusters_ptr_->clusters[i].b.z);

		tf::Vector3 ptAInCameraFrame = camera_to_base_link * ptA;
		tf::Vector3 ptBInCameraFrame = camera_to_base_link * ptB;

		// Now that ptA and ptB are in camera frame, we can project.
		tf::Vector3 resultant_pixels_ptA = (*projection_matrix_) * ptAInCameraFrame;
		tf::Vector3 resultant_pixels_ptB = (*projection_matrix_) * ptBInCameraFrame;

		int start_x = (int) (resultant_pixels_ptA[0]/resultant_pixels_ptA[2] - 70);
		int start_y = (int) (resultant_pixels_ptA[1]/resultant_pixels_ptA[2] - 50);
		int end_x = (int) (resultant_pixels_ptB[0]/resultant_pixels_ptB[2] - 40);
		int end_y = (int) (resultant_pixels_ptB[1]/resultant_pixels_ptB[2] - 30);

		if( (start_x < 0 && end_x < 0) || (start_x >= image_width && end_x >= image_width) ||
				(start_y < 0 && end_y < 0) || (end_y >= image_height && start_y >= image_height) )
		{
			//ROS_INFO("UnSeen: %s", cluster_names[i].c_str());

			char object_name_with_num[20];
			sprintf(object_name_with_num, "unseen_object_%d", unseen_num);
			cluster_names.push_back(object_name_with_num);
			unseen_num++;
			continue;
		}

		if(start_x < 0)
			start_x = 0;
		if(start_y < 0)
			start_y = 0;
		if(end_x >= image_width)
			end_x = image_width - 1;
		if(end_y >= image_height)
			end_y = image_height - 1;

		//ROS_INFO("Widths: (%d,%d) and (%d,%d)", cam_s_x, cam_s_y, cam_e_x, cam_e_y);

		cv::Mat new_part = cutImage(test_image->image, start_y, start_x, end_y-start_y, end_x-start_x);

		//ROS_INFO("SHOWING CUT IMAGE: SIZE: %d, %d", end_x-start_x, end_y-start_y);

		//cv::imshow("sucks", new_part);
		//cv::waitKey(0);
		std::string name_from_sift = processImage(new_part, tole);
		if(name_from_sift.find("unknown_object") != std::string::npos)
		{
			char object_number[4];
			sprintf(object_number, "%d", unknown_num);
			name_from_sift += object_number;
			unknown_num++;
		}
		cluster_names.push_back(name_from_sift);
		//ROS_INFO("Seen: %s", cluster_names[i].c_str());
	}

	if(clusters_ptr_->clusters.size() != 0)
		response.objects.header.frame_id = clusters_ptr_->clusters[0].centroid.header.frame_id;

	// Before we Anchor, let's retrive the signatures from CAM. //
	cam_objects_ = cam_interface::getAllObjectSignaturesFromCAM(tf_listener_);

	for(int i = 0; i < clusters_ptr_->clusters.size(); i++)
	{
		doro_msgs::TableObject __object;
		__object.centroid = clusters_ptr_->clusters[i].centroid.point;
		__object.cluster_size = clusters_ptr_->clusters[i].cluster_size;
		__object.color = clusters_ptr_->clusters[i].color;

		/* ********************************************************************** *
		 * This is the part where we anchor! We set the id (symbol) to a value by *
		 * using the sift matching. If sift can't find it, we use other signature *
		 * values and compare it to the features here.                            *
		 * ********************************************************************** */

		// Stage 1: Using sift recognized id.
		__object.id = cluster_names[i];

		// Stage 2: Using signatures from CAM.
		if(cluster_names[i].find("unseen") != std::string::npos ||
		   cluster_names[i].find("unknown") != std::string::npos ||
		   cluster_names[i].empty())
		{
		  //anchorUsingSignature(__object, tole);
		}

		response.objects.table_objects.push_back(__object);
	}

	// Pruning out.

	response.objects.number_of_objects = response.objects.table_objects.size();

	if(request.signature.cluster_size.size() != 0)
	{
		for(std::vector<doro_msgs::TableObject>::iterator iter_obj = response.objects.table_objects.begin();
				iter_obj != response.objects.table_objects.end();
				iter_obj++)
		{
			if(iter_obj->cluster_size[0] > (0.025 + request.signature.cluster_size[0]) ||
					iter_obj->cluster_size[0] < (request.signature.cluster_size[0] - 0.025) ||
					iter_obj->cluster_size[1] > (0.025 + request.signature.cluster_size[1]) ||
					iter_obj->cluster_size[1] < (request.signature.cluster_size[1] - 0.025) ||
					iter_obj->cluster_size[2] > (0.025 + request.signature.cluster_size[2]) ||
					iter_obj->cluster_size[2] < (request.signature.cluster_size[2] - 0.025) )
			{
				// If the size is out of bounds, remove it.
				response.objects.table_objects.erase(iter_obj);
				iter_obj--;
			}
		}
		ROS_INFO("We pruned for size. There are %d objects now.", response.objects.table_objects.size());
		pruned = true;
	}
	else
		ROS_INFO("Size Ignored.");


	if(request.signature.color.size() == 3)
	{
		//int color = (request.signature.color[0] << 16) | (request.signature.color[1] << 8) | (request.signature.color[2]);

		for(std::vector<doro_msgs::TableObject>::iterator iter_obj = response.objects.table_objects.begin();
				iter_obj != response.objects.table_objects.end();
				iter_obj++)
		{
			if( (iter_obj->color[0] > (50 + request.signature.color[0]) || iter_obj->color[0] < (request.signature.color[0] - 50) ) ||
					(iter_obj->color[1] > (50 + request.signature.color[1]) || iter_obj->color[1] < (request.signature.color[1] - 50) ) ||
					(iter_obj->color[2] > (50 + request.signature.color[2]) || iter_obj->color[2] < (request.signature.color[2] - 50) ) )
			{
				response.objects.table_objects.erase(iter_obj);
				iter_obj--;
			}
		}

		ROS_INFO("We pruned for color. There are %d objects now.", response.objects.table_objects.size());
		pruned = true;

	}
	else
		ROS_INFO("Color Ignored.");


	// In this case return only the element requested.
	// That is, we return only that element which has the same id as in the signature.
	for(std::vector<doro_msgs::TableObject>::iterator iter_obj = response.objects.table_objects.begin();
			iter_obj != response.objects.table_objects.end();
			iter_obj++)
	{
		if(request.type == request.KNOWN && (iter_obj->id.empty() || iter_obj->id.find("unknown") != std::string::npos || iter_obj->id.find("unseen") != std::string::npos) )
		{
			response.objects.table_objects.erase(iter_obj);
			iter_obj--;
			continue;
		}

		else if(request.type == request.NAME &&
				iter_obj->id.compare(request.signature.id) != 0)
		{
			//ROS_INFO("%s != %s", iter_obj->id.c_str(), request.signature.id.c_str());
			response.objects.table_objects.erase(iter_obj);
			iter_obj--;
			continue;
		}
	}


	return_value = true;

	ROS_INFO("****************");
	ROS_INFO("* Response set *");
	ROS_INFO("****************");

	clusters_ptr_.reset();
	image_.reset();
	return return_value;

}

void AcquireTabletopServer::anchorUsingSignature(doro_msgs::TableObject& object, const float& tolerance)
{
	doro_msgs::TableObjectArray::_table_objects_type::iterator last_match = cam_objects_.table_objects.end();

	// ***************************************************** //
	// Increment these after every match.                   //
	// Set id to the object that has most number of matches. //
	// If none match, do nothing.                            //
	// ***************************************************** //
	std::vector <int> match_count;
	match_count.resize(cam_objects_.table_objects.size());

	for(int i = 0; i < match_count.size(); i++)
		match_count[i] = 0;

	//std::cout<<cam_objects_;
	int max_match_count_index = 0;
	int total_tolerance = (int) (50.00 * (1.00 + (1.00 - tolerance) ));
	float total_size_tolerance = (0.005* (1.00 + (1.00 - tolerance) ));

	int k = 0;
	for(doro_msgs::TableObjectArray::_table_objects_type::iterator it = cam_objects_.table_objects.begin();
			it != cam_objects_.table_objects.end();
			it++)
	{

		// ************** //
		// BASED ON COLOR //
		// ************** //

		if(it->color.size() == 3)
		{

				if( ((int) (object.color[0]) < (total_tolerance + (int)it->color[0]) && (int) (object.color[0]) > ((int) (it->color[0]) - total_tolerance) ) &&
						((int) (object.color[1]) < (total_tolerance + (int)it->color[1]) && (int) (object.color[1]) > ((int) (it->color[1]) - total_tolerance) ) &&
						((int) (object.color[2]) < (total_tolerance + (int)it->color[2]) && (int) (object.color[2]) > ((int) (it->color[2]) - total_tolerance) ) )
				{
					match_count[k]++;
					//ROS_INFO("COLOR MATCH");
					//std::cout<<object;
				}
		}


		// ************* //
		// BASED ON SIZE //
		// ************* //

		if(it->cluster_size.size() != 0)
		{

			if(object.cluster_size[0] < (total_size_tolerance + it->cluster_size[0]) &&
					object.cluster_size[0] > (it->cluster_size[0] - total_size_tolerance) &&
					object.cluster_size[1] < (total_size_tolerance + it->cluster_size[1]) &&
					object.cluster_size[1] > (it->cluster_size[1] - total_size_tolerance) &&
					object.cluster_size[2] < (total_size_tolerance + it->cluster_size[2]) &&
					object.cluster_size[2] > (it->cluster_size[2] - total_size_tolerance) )
			{
				match_count[k]++;
				//ROS_INFO("SIZE MATCH: %f < %f", );
			}
		}

		// If the current one has the best matches till now //
		if(match_count[k] > match_count[max_match_count_index])
			max_match_count_index = k;

		k++;
	}

	std::sort(match_count.begin(), match_count.end());

	// Finally see if two matches have equal match count.
	if(match_count[match_count.size()-1] > match_count[match_count.size()-2])
	{
		// Anchoring can take place if a single best match has occured.
		object.id = cam_objects_.table_objects[max_match_count_index].id;
	}
	else
	{
		//printf("\nNo match in anchoring process.\nmatch_count(last): %d, match_count(lastButOne): %d", match_count[match_count.size()-1], match_count[match_count.size()-2]);
	}
}

std::string AcquireTabletopServer::processImage(const cv::Mat& test_image, const float& tolerance)
{
	// detecting keypoints
	cv::SiftFeatureDetector detector(300);
	std::vector<cv::KeyPoint> keypoints_input_img;

	detector.detect(test_image, keypoints_input_img);

	// computing descriptors
	cv::SiftDescriptorExtractor extractor;
	cv::Mat descriptors_input_img;

	extractor.compute(test_image, keypoints_input_img, descriptors_input_img);

	// matching descriptors
	matchWithRegistry(descriptors_input_img, tolerance);

	if(matches_.size() == 0)
	{
		//ROS_INFO("This object is new.");
		return "unknown_object_";
	}

	std::vector<double> our_x, our_y;
	our_x.resize(matches_.size());
	our_y.resize(matches_.size());


	for(int j = 0; j < matches_.size(); j++)
	{
		cv::Mat img_the = test_image.clone();

		int i;
		for(i = 0; i < matches_[j].size(); i++)
		{
			our_x[j] += keypoints_input_img[matches_[j][i].queryIdx].pt.x;
			our_y[j] += keypoints_input_img[matches_[j][i].queryIdx].pt.y;

			//cv::circle(img_the, cv::Point(keypoints_input_img[matches_[j][i].queryIdx].pt.x, keypoints_input_img[matches_[j][i].queryIdx].pt.y), 10, cv::Scalar(0xff0000));
		}

		our_x[j] = our_x[j] / double(i);
		our_y[j] = our_y[j] / double(i);

		/*
		_object.id = ids_[match_indices_[j]];
		_object.x = (our_x[j]*258/640) + 195;
		_object.y = (our_y[j]*196/480) + 162;
		*/
	}
	image_.reset();

	ROS_INFO("[ACQUIRE] (Saw: %s)", ids_[match_indices_[0]].c_str());
	ROS_INFO("[ACQUIRE] --------------------------");
	return ids_[match_indices_[0]];
}

} /* namespace acquire_tabletop */

int main(int argn, char* args[])
{
	ros::init(argn, args, "acquire_tabletop_server");
	std::vector <std::string> peis_args;
	ros::removeROSArgs(argn, args, peis_args);
	std::vector <char*> peis_args_c_str;
	
	for(int i = 0; i < peis_args.size(); i++) {
	  peis_args_c_str.push_back(new char[peis_args[i].size()+1]);
	  strcpy(peis_args_c_str[i],peis_args[i].c_str());
	}

	argn = peis_args.size();
	args = peis_args_c_str.data();

	for(int i = 0; i < argn; i++) {
	  ROS_INFO("(%s)",args[i]);
	}

	peiskmt_initialize(&argn, args);
	acquire_tabletop::AcquireTabletopServer ATS;
	ros::MultiThreadedSpinner m_t_spinner(4);
	m_t_spinner.spin();
}
