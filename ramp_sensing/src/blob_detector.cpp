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


Moments BlobDetector::buildMoment(Mat _src, bool binary)
{
  ROS_INFO("In buildMoment");

  // Get type of image
  int type = _src.type(), depth = CV_MAT_DEPTH(type), cn = CV_MAT_CN(type);  
  Size size = _src.size();
  const int TILE_SIZE = 32;

  MomentsInTileFunc func = 0;  
  uchar nzbuf[TILE_SIZE*TILE_SIZE];
  Moments m;  

  Mat src0(_src);

  // Set...something?
  /*if( binary || depth == CV_8U )
    func = momentsInTile<uchar, int, int>;
  else if( depth == CV_16U )
    func = momentsInTile<ushort, int, int64>;
  else if( depth == CV_16S )
    func = momentsInTile<short, int, int64>;
  else if( depth == CV_32F )
    func = momentsInTile<float, double, double>;
  else if( depth == CV_64F )
    func = momentsInTile<double, double, double>;
  else
    CV_Error( CV_StsUnsupportedFormat, "" );*/

  /*
   * Go through each pixel in the polygon
   */
  for(int y=0;y<size.height;y+=TILE_SIZE)
  {
    Size tileSize;
    tileSize.height = std::min(TILE_SIZE, size.height - y);
    
    for(int x=0;x<size.width;x+=TILE_SIZE)
    {
      tileSize.width = std::min(TILE_SIZE, size.width-x);
      Mat src(src0, Rect(x, y, tileSize.width, tileSize.height));

       if( binary )
       {
         cv::Mat tmp(tileSize, CV_8U, nzbuf);
         cv::compare( src, 0, tmp, CV_CMP_NE );
         src = tmp;
       } 


      // Need to have func defined
      double mom[10];
      func( src, mom );

      if(binary)
      {
        double s = 1./255;
        for( int k = 0; k < 10; k++ )
        {
          mom[k] *= s;
        }
      }

      /*
       * Start computing values in the moment array!
       */

      double xm = x * mom[0], ym = y * mom[0];
      ROS_INFO("xm: %f ym: %f", xm, ym);
      
      // Accumulate the moments computed in each tile      
      m.m10 += mom[1] + xm;
      m.m01 += mom[2] + ym;


      ROS_INFO("mom[1]: %f mom[2]: %f", mom[1], mom[2]);

    } // end inner for
  } // end outter for
  
  ROS_INFO("Exiting buildMoment");
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

    // Build the moment array for each set of contour points (aka each polygon)
    Moments moms = moments(Mat(contours[contourIdx]));

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

    centers.push_back(center);
  } // end for each contour
}
