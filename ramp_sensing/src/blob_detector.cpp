#include "blob_detector.h"


BlobDetector::BlobDetector() {}
BlobDetector::~BlobDetector() {}


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
    findBlobs(binarizedImage, curCenters);
  }
  
}


Moments BlobDetector::buildMoment(Mat _src, bool binary)
{
  int type = _src.type(), depth = CV_MAT_DEPTH(type), cn = CV_MAT_CN(type);  
  Size size = _src.size();
  const int TILE_SIZE = 32;

  MomentsInTileFunc func = 0;  
  uchar nzbuf[TILE_SIZE*TILE_SIZE];
  Moments m;  

  Mat src0(_src);

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

      double xm = x * mom[0], ym = y * mom[0];

      // Accumulate the moments computed in each tile      


    } // end inner for
  } // end outter for
}


void BlobDetector::findBlobs(Mat binaryImage, std::vector<Center>& centers)
{
  centers.clear();

  std::vector< std::vector<Point> > contours;
  Mat tempBinaryImage = binaryImage.clone();
  findContours(tempBinaryImage, contours, RETR_LIST, CHAIN_APPROX_NONE);

  for(size_t contourIdx = 0; contourIdx < contours.size(); contourIdx++)
  {
    Center center;
    center.confidence = 1;

    Moments moms = moments(Mat(contours[contourIdx]));

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
  }
}
