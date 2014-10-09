#include <ros/ros.h>
#include <acquire_tabletop/AcquireTabletop.h>

int main(int argn, char *args[])
{
	ros::init(argn, args, "acquire_tabletop_client_test");

	ros::NodeHandle nh;
	ros::ServiceClient sc = nh.serviceClient <acquire_tabletop::AcquireTabletop> ("acquire_tabletop");

	acquire_tabletop::AcquireTabletop message;
	message.request.type = acquire_tabletop::AcquireTabletopRequest::ALL;

	//message.request.signature.cluster_size = 6000.00;
	message.request.signature.centroid.x = 0.23;
	message.request.signature.centroid.y = 0.17;
	message.request.signature.centroid.z = 0.86;

	message.request.signature.centroid_tolerance.x = 0.55;
	message.request.signature.centroid_tolerance.y = 0.55;
	message.request.signature.centroid_tolerance.z = 0.05;

	ROS_INFO("Client_calls");
	if (sc.call(message))
	{
		std::cout<<message.response;
	}

	else
	{
		ROS_ERROR("Failed to call service acquire_tabletop");
		return 1;
	}

	return 0;

}
