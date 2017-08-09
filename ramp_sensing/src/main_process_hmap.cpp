#include<iostream>
#include<ros/ros.h>
#include<nav_msgs/OccupancyGrid.h>
#include "utility.h"
#include "GridMap2D.h"
#include <vector>

using namespace std;
using namespace cv;

Mat hmap_mat;
Mat hmap_edges;

void CannyThreshold(int, void*)
{
  /// Reduce noise with a kernel 3x3
  blur( hmap_mat, hmap_edges, Size(3,3) );
  
  imshow("after blur src", hmap_edges);
  waitKey(0);


  // Somehow, lowThreshold is being converted to unsigned int before this point
  // its value is 32767 (-1 for unsigned 4-byte int)
  // Set the value back to 0 for edge detection to work
  int lowThreshold = 0;
  int ratio = 3;
  int kernel_size = 3;

  /// Canny detector
  Canny( hmap_edges, hmap_edges, lowThreshold, lowThreshold*ratio, kernel_size );

  imshow( "hmap_edges", hmap_edges);
  waitKey(0);
}



void getHist()
{
  /*
   * Get histogram
   */
  int histSize = 100;
  float range[]  = {0, 100};
  const float* histRange = {range};
  
  bool uniform = true;
  bool accumulate = false;

  Mat hist;
  calcHist(&hmap_mat, 1, 0, Mat(), hist, 1, &histSize, &histRange, uniform, accumulate);


  /*
   * Draw and display histogram
   */
  int hist_w=512; int hist_h=400;
  int bin_w = cvRound( (double)hist_w/histSize );
  
  Mat histImage(hist_h, hist_w, CV_8UC3, Scalar(0,0,0));

  /// Normalize the result to [ 0, histImage.rows ]
  normalize(hist, hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );

    /// Draw for each channel
  for( int i = 1; i < histSize; i++ )
  {
    line( histImage, cv::Point( bin_w*(i-1), hist_h - cvRound(hist.at<float>(i-1)) ), cv::Point( bin_w*(i), hist_h - cvRound(hist.at<float>(i)) ), Scalar( 255, 0, 0), 2, 8, 0  );
  }

  imshow("Histogram", histImage);
  waitKey(0);
}





vector<cv::Point> bhContoursCenter(const vector<vector< cv::Point> >& contours,bool centerOfMass,int contourIdx=-1)
{
  ROS_INFO("In bhContoursCenter");
  ROS_INFO("centerOfMass: %s contourIdx: %i", centerOfMass ? "True" : "False", contourIdx);
  vector<cv::Point> result;
  if (contourIdx > -1)
  {
      if (centerOfMass)
      {
          Moments m = moments(contours[contourIdx],true);
          result.push_back( cv::Point(m.m10/m.m00, m.m01/m.m00));
      }
      else 
      {
          Rect rct = boundingRect(contours[contourIdx]);
          result.push_back( cv::Point(rct.x + rct.width / 2 ,rct.y + rct.height / 2));
      }
  }
  else 
  {
      if (centerOfMass)
      {
        for (int i=0; i < contours.size();i++)
        {
          if(contours[i].size() > 2)
          {
            for(int j=0;j<contours[i].size();j++)
            {
              ROS_INFO("Contour point[%i][%i]: (%i,%i)", i, j, contours[i][j].x, contours[i][j].y);
            }
            Moments m = moments(contours[i],true);
            ROS_INFO("m.m00: %f m.m10: %f m.m01: %f", m.m00, m.m10, m.m01);
            result.push_back( cv::Point(m.m10/m.m00, m.m01/m.m00));
          }
        }
      }
      else 
      {
        for (int i=0; i < contours.size(); i++)
        {
          Rect rct = boundingRect(contours[i]);
          result.push_back(cv::Point(rct.x + rct.width / 2 ,rct.y + rct.height / 2));
        }
      }
  }

  return result;
}

void contoursTwo(Mat& src)
{
  std::vector<std::vector<cv::Point>> contours;
  std::vector<Vec4i> hierarchy;
  findContours(src,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);


  RNG rng(12345);
   /// Draw contours
  Mat drawing = Mat::zeros( src.size(), CV_8UC3 );
  for( int i = 0; i< contours.size(); i++ )
  {
    Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
    drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, cv::Point() );
  }

  vector<cv::Point> maxima = bhContoursCenter(contours, true);
  /// Show in a window
  namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
  imshow( "Contours", drawing );
  
  for(int i=0;i<maxima.size();i++)
  {
    cv::Point pnt = maxima[i];
    ROS_INFO("Local maxima %i: (%i,%i)", i, pnt.x, pnt.y);
    src.at<uchar>(pnt.x, pnt.y) = 255;
  }
  imshow("maxima", src);
  waitKey(0);
}


vector<cv::Point> bhFindLocalMaximum(InputArray _src,int neighbor=2)
{
  // Get image
  Mat src = _src.getMat();

  Mat peak_img = src.clone();

  // Dilate
  dilate(peak_img,peak_img,Mat(),cv::Point(-1,-1),neighbor);
  imshow("dilate", peak_img);
  peak_img = peak_img - src;
  resize(peak_img, peak_img, Size(peak_img.cols*4, peak_img.rows*4));
  imshow("dilate-src", peak_img);

  
  waitKey(0);

  Mat flat_img;
  erode(src,flat_img,Mat(),cv::Point(-1,-1),neighbor);
  imshow("erode", flat_img);
  flat_img = src - flat_img;
  imshow("erode-src", flat_img);

  waitKey(0);


  threshold(peak_img,peak_img,0,255,CV_THRESH_BINARY);
  threshold(flat_img,flat_img,0,255,CV_THRESH_BINARY);
  bitwise_not(flat_img,flat_img);

  imshow("thr_dil", peak_img);
  imshow("thr_ero", flat_img);
  waitKey(0);
  
  printf("\nDone threshold\n");

  // Set 255 to any pixels that flat_img has nonzero value
  peak_img.setTo(cv::Scalar::all(255),flat_img);
  printf("\nDone setTo\n");
  // not operator b/c findContours looks for high-valued regions
  bitwise_not(peak_img,peak_img);
  printf("\nDone bitwise_not\n");

  std::vector<std::vector<cv::Point>> contours;
  findContours(peak_img,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
  
   /*
    *  Draw contours
    */
  std::vector<Vec4i> hierarchy;
  RNG rng(12345);
  Mat drawing = Mat::zeros( peak_img.size(), CV_8UC3 );
  for( int i = 0; i< contours.size(); i++ )
  {
    Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
    drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, cv::Point() );
  }
  // Done drawing contours

  // Get centroids of the contours
  vector<cv::Point> maxima = bhContoursCenter(contours, true);

  /// Show in a window
  namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
  imshow( "Contours", drawing );

  return bhContoursCenter(contours,true);
}


void hmapCb(nav_msgs::OccupancyGridConstPtr hmap)
{
  printf("\nIn hmapCb\n");
  gridmap_2d::GridMap2D hmap_gmap(hmap, false);
  hmap_mat = hmap_gmap.probMap();
  cv::transpose(hmap_mat, hmap_mat);
  cv::flip(hmap_mat, hmap_mat, 0);
  imshow("hmap", hmap_mat);
  //waitKey(0);

  /*double minV = 0;
  double maxV = 100;
  cv::Point min, max;
  minMaxLoc(hmap_mat, &minV, &maxV, &min, &max);
  ROS_INFO("min: (%i,%i)", min.x, min.y);
  ROS_INFO("max: (%i,%i)", max.x, max.y);*/

  /*
   * Dilation method
   * Dilate with 0 in middle of kernel, then compare image pixel > dilated image pixel
   * Needs peaks to be sharp, can't find peak when gradient is too smooth
   */
  // Create kernel
  int kernalSize = 3;
  Mat1b kernelLM(Size(kernalSize, kernalSize), 1);
  kernelLM.at<uchar>(kernalSize/2, kernalSize/2) = 0;
  Mat imageLM;

  // Gaussian blur the image
  //GaussianBlur(hmap_mat, hmap_mat, Size(3,3), 0, 0);
  //imshow("blurred", hmap_mat);

  dilate(hmap_mat, imageLM, kernelLM);
  imshow("dilated",imageLM);
  Mat1b localMaxima = (hmap_mat > imageLM);
  for(int i=25;i<35;i++)
    for(int j=30;j<40;j++)
      ROS_INFO("hmap_mat[%i][%i]: %i", i, j, hmap_mat.at<uchar>(i,j));
  imshow("localMax", localMaxima);
  waitKey(0);

  std::vector<cv::Point> locations;   // output, locations of non-zero pixels
  cv::findNonZero(localMaxima, locations);
  for(int i=0;i<locations.size();i++)
  {
    cv::Point pnt = locations[i];
    ROS_INFO("Local maxima %i: (%i,%i)", i, pnt.x, pnt.y);
    hmap_mat.at<uchar>(pnt.x, pnt.y) = 255;
  }
  imshow("maxima", hmap_mat);
  waitKey(0);



  /*
   * Contour method
   * Dilate+Erode and subtract from original to get boundaries, then get contours and use centroid point
   * It finds the region that dilation can't get, but it doesn't get the corner point
   * cannot find boundary of corner point
   */
  /*vector<cv::Point> locations = bhFindLocalMaximum(hmap_mat, 10);
  for(int i=0;i<locations.size();i++)
  {
    cv::Point pnt = locations[i];
    ROS_INFO("Local maxima %i: (%i,%i)", i, pnt.x, pnt.y);
    hmap_mat.at<uchar>(pnt.y, pnt.x) = 255;
  }
  imshow("hmap", hmap_mat);
  waitKey(0);*/



  /*
   * Contour method number 2
   * Apply contours to raw image
   * Doesn't work at all - contour goes around whole image
   */
  //contoursTwo(hmap_mat);
  


  //hmap_mat.at<uchar>(min.y,min.x) = 255;
  //hmap_mat.at<uchar>(max.y,max.x) = 255;
  //imshow("Local Maxima", localMaxima);
  //waitKey(0);
  //getHist();
  //CannyThreshold(0,0);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "process_hmap");
  ros::NodeHandle handle;

  ros::Subscriber sub_hmap = handle.subscribe("/hilbert_map", 10, &hmapCb);
  
  printf("\nSpinning\n");
  ros::spin();

  printf("\nExiting normally\n");
}
