#include "ros/ros.h"
#include "std_msgs/String.h"
#include "swri_profiler_msgs/ProfileIndexArray.h"
#include "swri_profiler_msgs/ProfileDataArray.h"
#include <boost/algorithm/string.hpp>
#include <fstream>
#include <ros/package.h>
#include <sys/stat.h> 
#include <sys/types.h> 

std::map<std::string, std::vector<std::string>> names;
std::map<std::string, bool> headersNeeded;
std::map<std::string, long> starts;
std::ofstream file;
std::string pack_path;

void indexCallback(const swri_profiler_msgs::ProfileIndexArray::ConstPtr& msg);
void dataCallback(const swri_profiler_msgs::ProfileDataArray::ConstPtr& msg);
void recordValues(std::string pack, int index, double vals[]);
void createFolder(std::string path, std::string name);
void recordHeaders(std::string pack, int index);


int main(int argc, char **argv)
{
    pack_path = ros::package::getPath("ramp_profiler");
    createFolder(pack_path, "/timings");

    ros::init(argc, argv, "csv_profiler");
    ros::NodeHandle n;

    ros::Subscriber indexSub = n.subscribe("/profiler/index", 1000, indexCallback);
    ros::Subscriber dataSub = n.subscribe("/profiler/data", 1000, dataCallback);

    ros::spin();

    return 0;
}

void dataCallback(const swri_profiler_msgs::ProfileDataArray::ConstPtr& msg)
{
    // std::cout << "In Data Callback function" << std::endl;

    if (headersNeeded.count(msg->header.frame_id) != 0 && 
        headersNeeded.at(msg->header.frame_id))
    {
        for (int i = 0; i < names.at(msg->header.frame_id).size(); i++)
        {
            // std::cout << msg->data[i] << std::endl;
            recordHeaders(msg->header.frame_id, i);
        }

        starts.at(msg->header.frame_id) = msg->rostime_stamp.toNSec();
        headersNeeded.at(msg->header.frame_id) = false;
    }
    else
    {
        for (int i = 0; i < names.at(msg->header.frame_id).size(); i++)
        {
            double vals[5];
            vals[0] = (msg->rostime_stamp.toNSec() - starts.at(msg->header.frame_id)) / 1000000.0;
            vals[1] = msg->data.at(i).abs_call_count;
            vals[2] = msg->data.at(i).abs_total_duration.toNSec() / 1000000.0;
            vals[3] = msg->data.at(i).rel_total_duration.toNSec() / 1000000.0;
            vals[4] = msg->data.at(i).rel_max_duration.toNSec() / 1000000.0;
            recordValues(msg->header.frame_id, i, vals);
        }
    }
}

void indexCallback(const swri_profiler_msgs::ProfileIndexArray::ConstPtr& msg)
{
    // std::cout << "In Index Callback function" << std::endl;

    createFolder(pack_path + "/timings", msg->header.frame_id);

    if (headersNeeded.count(msg->header.frame_id) != 0)
    {
        headersNeeded.at(msg->header.frame_id) = true;
    }
    else
    {
        headersNeeded.insert({msg->header.frame_id, true});
        starts.insert({msg->header.frame_id, 0});
    }
    
    if (names.count(msg->header.frame_id) != 0)
    {
        names.at(msg->header.frame_id).clear();
    }
    else
    {
        std::vector<std::string> vec;
        names.insert({msg->header.frame_id, vec});
    }

    for (int i = 0; i < msg->data.size(); i++)
    {
        std::vector<std::string> result;
        std::vector<std::string> final;
        boost::split(result, msg->data[i].label, boost::is_any_of("/"));
        boost::split(final, result.at(result.size()-1), boost::is_any_of("-"));
        names.at(msg->header.frame_id).push_back(final.at(1) + "-" + final.at(2) + ".csv");
    }
}

void recordValues(std::string pack, int index, double vals[])
{
    file.open(pack_path + "/timings" + pack + "/" + names.at(pack).at(index), std::ios::app);
    
    for (int i = 0; i < 5; i++)
    {
        file << vals[i];

        if (i < 4)
        {
            file << ",";
        }
    }

    file << "\n";
    file.close();
}

void createFolder(std::string path, std::string name)
{
    struct stat buffer;

    if (stat((path + name).c_str(), &buffer) != -1) 
    {
        if (S_ISDIR(buffer.st_mode)) 
        {
            std::cout << "The \"" + name + "\" folder is already present." << std::endl;
        }
    }
    else
    {
        int error = mkdir((path + name).c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

        if (error == -1)
        {
            ROS_ERROR("A folder could not be created.");
        }
        else
        {
            std::cout << "The \"" + name + "\" folder was created successfully." << std::endl;
        }
    }
}

void recordHeaders(std::string pack, int index)
{
    std::cout << "Creating file -> " << names.at(pack).at(index) << std::endl;
    file.open(pack_path + "/timings" + pack + "/" + names.at(pack).at(index));
    file << "Time Recorded,";
    file << "Absolute Call Count,";
    file << "Absolute Total Duration,";
    file << "Relative Total Duration,";
    file << "Relative Max Duration";
    file << "\n";
    file.close();
}