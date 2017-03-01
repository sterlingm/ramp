#include "blob_detector.h"


BlobDetector::BlobDetector(nav_msgs::OccupancyGridConstPtr g) 
{
  grid = *g;
  convertOGtoMat(g);
}
BlobDetector::~BlobDetector() {}

void BlobDetector::convertOGtoMat(nav_msgs::OccupancyGridConstPtr g)
{
  ROS_INFO("In BlobDetector::convertOGtoMat");
  
  // Use the GridMap2D library to convert from nav_msgs::OccupancyGrid to cv::Mat
  gridmap_2d::GridMap2D gmap(g, false);

  // Create a window
  //cv::namedWindow("testing", CV_WINDOW_AUTOSIZE);

  src = gmap.binaryMap();

  // Show the image
  //cv::imshow("testing", src);

  // PRESS ESC TO BEFORE CLOSING WINDOW, OTHERWISE THE PROGRAM WILL HANG
  //cv::waitKey(0);
}

void BlobDetector::detect(std::vector<KeyPoint>& keypoints)
{

  keypoints.clear();
  Mat grayscaleImage;
  if (src.channels() == 3)
      cvtColor(src, grayscaleImage, COLOR_BGR2GRAY);
  else
      grayscaleImage = src;

  if (grayscaleImage.type() != CV_8UC1) 
  {
      CV_Error(Error::StsUnsupportedFormat, "Blob detector only supports 8-bit images!");
  }

 
  std::vector < std::vector<Center> > centers;
  for (double thresh = params.minThreshold; thresh < params.maxThreshold; thresh += params.thresholdStep)
  {
    Mat binarizedImage;
    threshold(grayscaleImage, binarizedImage, thresh, 255, THRESH_BINARY);

    std::vector<Center> curCenters;
    findBlobs(curCenters);
  }
  
}


Moments BlobDetector::buildMoment_contour(Mat contour)
{
  Moments m;
  int lpt = contour.checkVector(2);
  int is_float = contour.depth() == CV_32F;
  const Point* ptsi = contour.ptr<Point>();
  const Point2f* ptsf = contour.ptr<Point2f>();

  CV_Assert( contour.depth() == CV_32S || contour.depth() == CV_32F );

  if( lpt == 0 )
      return m;

  double a00 = 0, a10 = 0, a01 = 0, a20 = 0, a11 = 0, a02 = 0, a30 = 0, a21 = 0, a12 = 0, a03 = 0;
  double xi, yi, xi2, yi2, xi_1, yi_1, xi_12, yi_12, dxy, xii_1, yii_1;

  if( !is_float )
  {
      xi_1 = ptsi[lpt-1].x;
      yi_1 = ptsi[lpt-1].y;
  }
  else
  {
      xi_1 = ptsf[lpt-1].x;
      yi_1 = ptsf[lpt-1].y;
  }

  xi_12 = xi_1 * xi_1;
  yi_12 = yi_1 * yi_1;

  for( int i = 0; i < lpt; i++ )
  {
    ROS_INFO("i: %i lpt: %i", i, lpt);
    ROS_INFO("ptsi[%i].x: %i ptsi[%i].y: %i", i, ptsi[i].x, i, ptsi[i].y);
      if( !is_float )
      {
          xi = ptsi[i].x;
          yi = ptsi[i].y;
      }
      else
      {
          xi = ptsf[i].x;
          yi = ptsf[i].y;
      }

      xi2 = xi * xi;
      yi2 = yi * yi;
      dxy = xi_1 * yi - xi * yi_1;
      xii_1 = xi_1 + xi;
      yii_1 = yi_1 + yi;

      a00 += dxy;
      a10 += dxy * xii_1;
      a01 += dxy * yii_1;
      a20 += dxy * (xi_1 * xii_1 + xi2);
      a11 += dxy * (xi_1 * (yii_1 + yi_1) + xi * (yii_1 + yi));
      a02 += dxy * (yi_1 * yii_1 + yi2);
      a30 += dxy * xii_1 * (xi_12 + xi2);
      a03 += dxy * yii_1 * (yi_12 + yi2);
      a21 += dxy * (xi_12 * (3 * yi_1 + yi) + 2 * xi * xi_1 * yii_1 +
                 xi2 * (yi_1 + 3 * yi));
      a12 += dxy * (yi_12 * (3 * xi_1 + xi) + 2 * yi * yi_1 * xii_1 +
                 yi2 * (xi_1 + 3 * xi));
      xi_1 = xi;
      yi_1 = yi;
      xi_12 = xi2;
      yi_12 = yi2;
  }

  if( fabs(a00) > FLT_EPSILON )
  {
      double db1_2, db1_6, db1_12, db1_24, db1_20, db1_60;

      if( a00 > 0 )
      {
          db1_2 = 0.5;
          db1_6 = 0.16666666666666666666666666666667;
          db1_12 = 0.083333333333333333333333333333333;
          db1_24 = 0.041666666666666666666666666666667;
          db1_20 = 0.05;
          db1_60 = 0.016666666666666666666666666666667;
      }
      else
      {
          db1_2 = -0.5;
          db1_6 = -0.16666666666666666666666666666667;
          db1_12 = -0.083333333333333333333333333333333;
          db1_24 = -0.041666666666666666666666666666667;
          db1_20 = -0.05;
          db1_60 = -0.016666666666666666666666666666667;
      }


      ROS_INFO("a00: %f a10: %f a01: %f", a00, a10, a01);
      ROS_INFO("db1_2: %f db1_6: %f", db1_2, db1_6);

      // spatial moments
      m.m00 = a00 * db1_2;
      m.m10 = a10 * db1_6;
      m.m01 = a01 * db1_6;
      m.m20 = a20 * db1_12;
      m.m11 = a11 * db1_24;
      m.m02 = a02 * db1_12;
      m.m30 = a30 * db1_20;
      m.m21 = a21 * db1_60;
      m.m12 = a12 * db1_60;
      m.m03 = a03 * db1_20;

  }
  return m;
}




void BlobDetector::findBlobs(std::vector<Center>& centers)
{
  ROS_INFO("In findBlobs");

  centers.clear();

  // Find the contours, aka edge points
  std::vector< std::vector<Point> > contours;
  Mat tempBinaryImage = src.clone();
  findContours(tempBinaryImage, contours, RETR_LIST, CHAIN_APPROX_NONE);

  // For each contour
  for(size_t contourIdx = 0; contourIdx < contours.size(); contourIdx++)
  {
    Center center;
    center.confidence = 1;
    Mat c = Mat(contours[contourIdx]);

    // Build the moment array for each set of contour points (aka each polygon)
    Moments moms = moments(Mat(contours[contourIdx]));
    ROS_INFO("Opencv moms: moms.m00: %f moms.m01: %f moms.m10: %f", moms.m00, moms.m01, moms.m10);
    moms = buildMoment_contour(Mat(contours[contourIdx]));
    ROS_INFO("My moms: moms.m00: %f moms.m01: %f moms.m10: %f", moms.m00, moms.m01, moms.m10);

    ROS_INFO("moms.m00: %f", moms.m00);

    // After computing moment, compute the center
    if(moms.m00 == 0.0)
        continue;
    center.location = Point2d(moms.m10 / moms.m00, moms.m01 / moms.m00);    


    // Compute radius
    std::vector<double> dists;
    for (size_t pointIdx = 0; pointIdx < contours[contourIdx].size(); pointIdx++)
    {
        Point2d pt = contours[contourIdx][pointIdx];
        dists.push_back(norm(center.location - pt));
    }
    std::sort(dists.begin(), dists.end());
    center.radius = (dists[(dists.size() - 1) / 2] + dists[dists.size() / 2]) / 2.;

    ROS_INFO("Center found: (%f, %f), radius: %f", center.location.x, center.location.y, center.radius);

    centers.push_back(center);
  } // end for each contour
}
