#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <ramp_msgs/RampTrajectory.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

int main() {
	rosbag::Bag bag;
	bag.open("/home/kai/data/test/test_read.bag", rosbag::bagmode::Read);

	std::vector<std::string> topics;
	topics.push_back(std::string("/bestTrajec"));

	rosbag::View view(bag, rosbag::TopicQuery(topics));

	foreach(rosbag::MessageInstance const m, view)
	{
		ramp_msgs::RampTrajectory::ConstPtr s = m.instantiate<ramp_msgs::RampTrajectory>();
		if (s != NULL)
		    std::cout << s->trajectory << std::endl;
	}

	bag.close();

	return 0;
}
