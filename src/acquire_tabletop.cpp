/*
 * acquire_tabletop.cpp
 *
 *  Created on: Sep 29, 2014
 *      Author: ace
 */

#include "acquire_tabletop/acquire_tabletop.h"

namespace acquire_tabletop {

AcquireTabletopServer::AcquireTabletopServer()
{
	clusters_sub_ = nh_.subscribe("/clusters", 1, &AcquireTabletopServer::clustersCB, this);
	server_ = nh_.advertiseService("acquire_tabletop", &AcquireTabletopServer::serverCB, this);

	tf_listener_ = boost::shared_ptr <tf::TransformListener> (new tf::TransformListener (ros::Duration(10.0)));
	prepare();
	subscribed_ = false;
	ROS_INFO("Server Started!");
}

void AcquireTabletopServer::clustersCB(const doro_msgs::ClusterArrayConstPtr& _clusters)
{
	// Create space for new message
	clusters_ptr_ = doro_msgs::ClusterArrayPtr (new doro_msgs::ClusterArray);

	ROS_INFO("Fetching clusters.");
	// Copy
	*clusters_ptr_ = *_clusters;
}

void AcquireTabletopServer::imageCallback(const sensor_msgs::ImageConstPtr& _msg)
{
	image_ = sensor_msgs::ImagePtr(new sensor_msgs::Image(*_msg));
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

/*

void AcquireTabletopServer::flase_prep()
{
	images_.push_back(cv::imread("/home/ace/chittaranjan_ros/catkin_ws/src/obj_detection/img_registry/tropicana_front.jpg", CV_LOAD_IMAGE_GRAYSCALE));
	images_.push_back(cv::imread("/home/ace/chittaranjan_ros/catkin_ws/src/obj_detection/img_registry/ibumetin.jpg", CV_LOAD_IMAGE_GRAYSCALE));
	images_.push_back(cv::imread("/home/ace/chittaranjan_ros/catkin_ws/src/obj_detection/img_registry/pandonil.jpg", CV_LOAD_IMAGE_GRAYSCALE));
	images_.push_back(cv::imread("/home/ace/chittaranjan_ros/catkin_ws/src/obj_detection/img_registry/yoggi.jpg", CV_LOAD_IMAGE_GRAYSCALE));

	cv::SiftFeatureDetector detector(300);
	cv::SiftDescriptorExtractor extractor;

	ids_.push_back("tropicana");
	ids_.push_back("ibumetin");
	ids_.push_back("pandonil");
	ids_.push_back("yoggi");

	keypoints_.resize(IMAGES_SIZE);
	descriptors_.resize(IMAGES_SIZE);

	detector.detect(images_[0], keypoints_[0]);
	detector.detect(images_[1], keypoints_[1]);
	detector.detect(images_[2], keypoints_[2]);
	detector.detect(images_[3], keypoints_[3]);

	extractor.compute(images_[0], keypoints_[0], descriptors_[0]);
	extractor.compute(images_[1], keypoints_[1], descriptors_[1]);
	extractor.compute(images_[2], keypoints_[2], descriptors_[2]);
	extractor.compute(images_[3], keypoints_[3], descriptors_[3]);
}


*
void AcquireTabletopServer::getDescriptorsFromTuples()
{

	char _key[30];
	//descriptors_.resize(image_numbers);

	int NUM_OF_IMAGES = 4;
	std::vector<std::string> object_names;
	object_names.push_back("tropicana");
	object_names.push_back("ibumetin");
	object_names.push_back("pandonil");
	object_names.push_back("yoggi");

	int id_for_tuple = 111; //peiskmt_peisid();

	ros_over_peis::Subscriber <doro_msgs::SiftDescriptor> image_from_peis_sub[NUM_OF_IMAGES];
	doro_msgs::SiftDescriptor::Ptr desc[NUM_OF_IMAGES];
	PeisTuple* recd_tuple[NUM_OF_IMAGES];

	ROS_INFO("Preparing ...");

	for(int i = 0; i < NUM_OF_IMAGES; i++)
	{
		// Subscribe to data tuple;
		peiskmt_subscribe(id_for_tuple, std::string(object_names[i] + ".sift_descriptor.data").c_str());

		image_from_peis_sub[0].setTupleKey(object_names[i] + ".sift_descriptor.header");
		image_from_peis_sub[0].subscribe();

		desc[i] = doro_msgs::SiftDescriptor::Ptr(new doro_msgs::SiftDescriptor);
		image_from_peis_sub[i].getMsg(desc[i]);
		while(!desc[i])
		{
			image_from_peis_sub[i].getMsg(desc[i]);
			printf("Getting descriptor tuple %d...\n", i);
		}

		recd_tuple[i] = peiskmt_getTuple(id_for_tuple, std::string(object_names[i] + ".sift_descriptor.data").c_str(), PEISK_KEEP_OLD);
		while(!recd_tuple[i])
		{
			recd_tuple[i] = peiskmt_getTuple(id_for_tuple, std::string(object_names[i] + ".sift_descriptor.data").c_str(), PEISK_KEEP_OLD);
			printf("Getting data tuple %d...\n", i);
			sleep(1);
		}
		printf("Deserializing...\n");
		cv::Mat recd_mat(desc[i]->rows, desc[i]->cols, desc[i]->elem_type, (uchar *) recd_tuple[i]->data);
		descriptors_.push_back(recd_mat.clone());
		ids_.push_back(desc[i]->id);
	}

	sleep(1);


	std::cout<<descriptors_[0];

	ROS_INFO("Done: Succeeded reading PEIS Tuples for SIFT Descriptors...");
}
*/

void AcquireTabletopServer::getDescriptorsFromTuples()
{

	ROS_INFO("Preparing...");
	const int image_numbers = 2;
	char _key[30];
	//descriptors_.resize(image_numbers);

	int id_for_tuple = CAM_PEIS_ID; //peiskmt_peisid();

	ros_over_peis::Subscriber <doro_msgs::SiftDescriptor> image_from_peis_sub1 ("tropicana.sift_descriptor.header");
	ros_over_peis::Subscriber <doro_msgs::SiftDescriptor> image_from_peis_sub2 ("ibumetin.sift_descriptor.header");
	ros_over_peis::Subscriber <doro_msgs::SiftDescriptor> image_from_peis_sub3 ("pandonil.sift_descriptor.header");
	ros_over_peis::Subscriber <doro_msgs::SiftDescriptor> image_from_peis_sub4 ("yoggi.sift_descriptor.header");

	doro_msgs::SiftDescriptor::Ptr desc1 = doro_msgs::SiftDescriptor::Ptr(new doro_msgs::SiftDescriptor);
	doro_msgs::SiftDescriptor::Ptr desc2 = doro_msgs::SiftDescriptor::Ptr(new doro_msgs::SiftDescriptor);
	doro_msgs::SiftDescriptor::Ptr desc3 = doro_msgs::SiftDescriptor::Ptr(new doro_msgs::SiftDescriptor);
	doro_msgs::SiftDescriptor::Ptr desc4 = doro_msgs::SiftDescriptor::Ptr(new doro_msgs::SiftDescriptor);

	image_from_peis_sub1.getMsg(desc1);
	image_from_peis_sub2.getMsg(desc2);
	image_from_peis_sub3.getMsg(desc3);
	image_from_peis_sub4.getMsg(desc4);

	while(!desc1)
		image_from_peis_sub1.getMsg(desc1);
	while(!desc2)
		image_from_peis_sub2.getMsg(desc2);
	while(!desc3)
		image_from_peis_sub3.getMsg(desc3);
	while(!desc4)
		image_from_peis_sub4.getMsg(desc4);

	peiskmt_subscribe(id_for_tuple, "tropicana.sift_descriptor.data");
	peiskmt_subscribe(id_for_tuple, "ibumetin.sift_descriptor.data");
	peiskmt_subscribe(id_for_tuple, "pandonil.sift_descriptor.data");
	peiskmt_subscribe(id_for_tuple, "yoggi.sift_descriptor.data");

	sleep(1);


	PeisTuple* recd_tuple1 = peiskmt_getTuple(id_for_tuple, "tropicana.sift_descriptor.data", PEISK_KEEP_OLD);
	printf("Getting data tuple 0...");
	while(!recd_tuple1)
	{
		recd_tuple1 = peiskmt_getTuple(id_for_tuple, "tropicana.sift_descriptor.data", PEISK_KEEP_OLD);
		sleep(1);
	}

	PeisTuple* recd_tuple2 = peiskmt_getTuple(id_for_tuple, "ibumetin.sift_descriptor.data", PEISK_KEEP_OLD);
	printf("Getting data tuple 1...\n");
	while(!recd_tuple2)
	{
		recd_tuple2 = peiskmt_getTuple(id_for_tuple, "ibumetin.sift_descriptor.data", PEISK_KEEP_OLD);
		sleep(1);
	}

	PeisTuple* recd_tuple3 = peiskmt_getTuple(id_for_tuple, "pandonil.sift_descriptor.data", PEISK_KEEP_OLD);
	printf("Getting data tuple 2...\n");
	while(!recd_tuple3)
	{
		recd_tuple3 = peiskmt_getTuple(id_for_tuple, "pandonil.sift_descriptor.data", PEISK_KEEP_OLD);
		sleep(1);
	}

	PeisTuple* recd_tuple4 = peiskmt_getTuple(id_for_tuple, "yoggi.sift_descriptor.data", PEISK_KEEP_OLD);
	printf("Getting data tuple 3...\n");
	while(!recd_tuple4)
	{
		recd_tuple4 = peiskmt_getTuple(id_for_tuple, "yoggi.sift_descriptor.data", PEISK_KEEP_OLD);
		sleep(1);
	}

	/** DESERIALIZE TO CV_MAT **/
	cv::Mat recd_mat1(desc1->rows, desc1->cols, desc1->elem_type, (uchar *) recd_tuple1->data);
	cv::Mat recd_mat2(desc2->rows, desc2->cols, desc2->elem_type, (uchar *) recd_tuple2->data);
	cv::Mat recd_mat3(desc3->rows, desc3->cols, desc3->elem_type, (uchar *) recd_tuple3->data);
	cv::Mat recd_mat4(desc4->rows, desc4->cols, desc4->elem_type, (uchar *) recd_tuple4->data);

	descriptors_.push_back(recd_mat1.clone());
	descriptors_.push_back(recd_mat2.clone());
	descriptors_.push_back(recd_mat3.clone());
	descriptors_.push_back(recd_mat4.clone());

	ids_.push_back(desc1->id);
	ids_.push_back(desc2->id);
	ids_.push_back(desc3->id);
	ids_.push_back(desc4->id);

	//std::cout<<descriptors_[0];

	ROS_INFO("Done: Succeeded reading PEIS Tuples for SIFT Descriptors...");
}

void AcquireTabletopServer::matchWithRegistry(const cv::Mat& _input_descriptors, const float& tolerance)
{
	cv::FlannBasedMatcher matcher;
	//ROS_INFO("Matcher called with tolerance: %f", tolerance);

	int max_size = 0;

	matches_.clear();
	match_indices_.clear();

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
		//ROS_INFO("Match number %d has %d features b4 filtering.", i, matches.size());

		for(std::vector <cv::DMatch>::iterator it = matches.begin(); it != matches.end(); it++)
			for(std::vector <cv::DMatch>::iterator jt = matches.begin() + 1; jt != matches.end(); jt++)
			{
				if(it->distance < 0.30*jt->distance)
				{
					matches_filtered.push_back(*it);
					break;
				}
			}

		if(matches_filtered.size() < 10 * (tolerance))
		{
			//ROS_INFO("Match number %d has %d features. Not enough.", i, matches_filtered.size());
			matches_filtered.clear();
		}
		else
		{
			//ROS_INFO("Match number %d has %d features.", i, matches_filtered.size());
			matches_.push_back(matches_filtered);
			match_indices_.push_back(i);
		}
	}
	// matches_ = matches_filtered_all[match_index_];

}

cv::Mat AcquireTabletopServer::cutImage(const cv::Mat& input_image, int row_offset, int col_offset, int row_size, int col_size)
{
	if( (input_image.rows < row_offset + row_size) || (input_image.cols < col_offset + col_size) )
	{
		printf("%d, %d, %d, %d, %d, %d\n", input_image.rows, row_offset, row_size, input_image.cols, col_offset, col_size);
		row_size = input_image.rows - row_offset;
		col_size = input_image.cols - col_offset;

		std::cout<<"burr";
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

		sleep(1);
	}

	ros::param::set("/cluster_extraction_enable", false);

	// Get the image from the rgb/image_raw topic.

	image_raw_sub_ = nh_.subscribe("image", 1, &AcquireTabletopServer::imageCallback, this);
	while(!image_)
	{
		//ROS_INFO("Waiting for the camera image...");
		sleep(1);
	}

	image_raw_sub_.shutdown();

	//ROS_INFO("Processing image...");
	cv_bridge::CvImagePtr test_image;
	test_image = cv_bridge::toCvCopy(image_);

	bool pruned = false;

	std::vector <std::string> cluster_names;
	int unknown_num = 0;
	int unseen_num = 0;

	for(int i = 0; i < clusters_ptr_->clusters.size(); i++)
	{
		int start_x = clusters_ptr_->clusters[i].window[0];
		int end_x = clusters_ptr_->clusters[i].window[2];
		int start_y = clusters_ptr_->clusters[i].window[1];
		int end_y = clusters_ptr_->clusters[i].window[3];;

		//start_x -= 30;
		//end_x -= 30;


		//ROS_INFO("Widths: (%d,%d) and (%d,%d)", start_x, start_y, end_x, end_y);

		if( (start_x < 195 && end_x < 195) || (start_x > 453 && end_x > 453) ||
				(start_y < 162 && end_y < 162) || (end_y > 358 && start_y > 358) )
		{
			//ROS_INFO("UnSeen: %s", cluster_names[i].c_str());

			char object_name_with_num[20];
			sprintf(object_name_with_num, "unseen_object_%d", unseen_num);
			cluster_names.push_back(object_name_with_num);
			unseen_num++;
			continue;
		}

		// ********************************************************************** //
		// Transform the window from xtion camera indices to conventional camera indices //
		// ********************************************************************** //

		int cam_s_x = (int) ( (start_x - 195.00)*640.00/258.00 ) - 80;
		int cam_s_y = (int) ( (start_y - 162.00)*480.00/196.00 ) - 80;
		int cam_e_x = (int) ( (end_x - 195.00)*640.00/258.00 ) + 20;
		int cam_e_y = (int) ( (end_y - 162.00)*480.00/196.00 ) + 20;

		if(cam_s_x < 0)
			cam_s_x = 0;
		if(cam_s_y < 0)
			cam_s_y = 0;
		if(cam_e_x > 639)
			cam_e_x = 639;
		if(cam_e_y > 479)
			cam_e_y = 479;

		//ROS_INFO("Widths: (%d,%d) and (%d,%d)", cam_s_x, cam_s_y, cam_e_x, cam_e_y);

		cv::Mat new_part = cutImage(test_image->image, cam_s_y, cam_s_x, cam_e_y-cam_s_y, cam_e_x-cam_s_x);

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
			anchorUsingSignature(__object, tole);
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

	if(request.signature.centroid.x != 0.0 && request.signature.centroid.y != 0.0 && request.signature.centroid.z != 0.0)
	{
		for(std::vector<doro_msgs::TableObject>::iterator iter_obj = response.objects.table_objects.begin();
				iter_obj != response.objects.table_objects.end();
				iter_obj++)
		{
			if( ( (iter_obj->centroid.x > (request.signature.centroid.x + request.signature.centroid_tolerance.x) ) ||
					(iter_obj->centroid.x < (request.signature.centroid.x - request.signature.centroid_tolerance.x) ) ) ||

					( (iter_obj->centroid.y > (request.signature.centroid.y + request.signature.centroid_tolerance.y) ) ||
							(iter_obj->centroid.y < (request.signature.centroid.y - request.signature.centroid_tolerance.y) ) ) ||

							( (iter_obj->centroid.z > (request.signature.centroid.z + request.signature.centroid_tolerance.z) ) ||
									(iter_obj->centroid.z < (request.signature.centroid.z - request.signature.centroid_tolerance.z) ) ) )
			{
				ROS_INFO("IS HERE");
				ROS_INFO("We pruned for location. There are %d objects now.", response.objects.table_objects.size());
				response.objects.table_objects.erase(iter_obj);
				iter_obj--;
			}
		}
		ROS_INFO("We pruned for location. There are %d objects now.", response.objects.table_objects.size());
		pruned = true;
	}
	else
		ROS_INFO("Location Ignored.");



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

	int max_match_count_index = 0;
	int total_tolerance = (int) (50.00 * (1.00 + tolerance));
	float total_size_tolerance = (0.025 * (1.00 + tolerance));

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
					ROS_INFO("COLOR MATCH");
					std::cout<<object;
				}
		}


		// ************* //
		// BASED ON SIZE //
		// ************* //

		if(it->cluster_size.size() != 0)
		{

			if(object.cluster_size[0] < (total_size_tolerance + it->cluster_size[0]) ||
					object.cluster_size[0] > (it->cluster_size[0] - total_size_tolerance) ||
					object.cluster_size[1] < (total_size_tolerance + it->cluster_size[1]) ||
					object.cluster_size[1] > (it->cluster_size[1] - total_size_tolerance) ||
					object.cluster_size[2] < (total_size_tolerance + it->cluster_size[2]) ||
					object.cluster_size[2] > (it->cluster_size[2] - total_size_tolerance) )
			{
				match_count[k]++;
				ROS_INFO("SIZE MATCH");
				std::cout<<object;
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
		printf("\nNo match in anchoring process.\nmatch_count(last): %d, match_count(lastButOne): %d", match_count[match_count.size()-1], match_count[match_count.size()-2]);
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

	//ROS_INFO("returning match string for index: %s", ids_[match_indices_[0]].c_str());
	return ids_[match_indices_[0]];
}

} /* namespace acquire_tabletop */

int main(int argn, char* args[])
{
	peiskmt_initialize(&argn, args);
	ros::init(argn, args, "acquire_tabletop_server");
	acquire_tabletop::AcquireTabletopServer ATS;
	ros::MultiThreadedSpinner m_t_spinner(4);

	m_t_spinner.spin();
}
