#ifndef BLOB_DETECTOR_H
#define BLOB_DETECTOR_H
#include "opencv2/features2d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <nav_msgs/OccupancyGrid.h>
#include "GridMap2D.h"
#include <ros/console.h>
using namespace cv;


typedef void (*MomentsInTileFunc)(const Mat& img, double* moments);


struct Center
{
  cv::Point2d location;
    double radius;
    double confidence;
};

class BlobDetector : public SimpleBlobDetector
{
public:
  BlobDetector(nav_msgs::OccupancyGridConstPtr g);
  ~BlobDetector();

  nav_msgs::OccupancyGrid grid;
  cv::Mat src;

  void detect(std::vector<cv::KeyPoint>& keypoints);
  void findBlobs(std::vector<Center>& centers);
  Moments buildMoment(Mat _src, bool binary);

  void convertOGtoMat(nav_msgs::OccupancyGridConstPtr g);

  Params params;
};

#endif
