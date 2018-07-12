#ifndef CIRCLE_PACKER
#define CIRCLE_PACKER
#include "utility.h"
#include "GridMap2D.h"
#include <nav_msgs/OccupancyGrid.h>
#include <queue>
#include <visualization_msgs/Marker.h>
#include "polypartition.h"

struct Attachment
{
  std::vector<int> cirs;
};


class CirclePacker 
{
  public:
    CirclePacker(nav_msgs::OccupancyGridConstPtr);
    CirclePacker(cv::Mat grid, const nav_msgs::OccupancyGrid& g);
    ~CirclePacker();

    void setNewGrid(nav_msgs::OccupancyGridConstPtr);
    void setNewGrid(nav_msgs::OccupancyGrid);

    void setStaticMap(nav_msgs::OccupancyGridConstPtr);

    void convertOGtoMat(nav_msgs::OccupancyGridConstPtr, cv::Mat& result);

    void CannyThreshold(int, void*);

    Normal computeNormal(Edge);

    void detectAttachedCircles(const std::vector<CircleOb*>& cir_obs, std::vector<Attachment>& result) const;
    void combineTwoCircles(const Circle a, const Circle b, Circle& result) const;
    void combineOverlappingCircles(std::vector<Circle> cs, std::vector<Circle>& result) const;


    Point findCenterOfPixels(const std::vector<cv::Point> pixels) const;
    std::vector<double> getWeights(const std::vector<cv::Point> pixels, const Point center) const;
    
    std::vector<Circle> getCirclesFromEdgeSets(const std::vector< std::vector<Edge> > edge_sets);
    std::vector<Circle> getCirclesFromEdges(const std::vector<Edge> edges, const cv::Point robot_cen);
    

    std::vector<Triangle> triangulatePolygon(const Polygon&);

    Circle getCircleFromKeypoint(const cv::KeyPoint k) const;
    std::vector<Circle> go();
    std::vector<Circle> goCorners();
    std::vector<Circle> goHough();
    std::vector<Circle> goMinEncCir();
    std::vector<Circle> goMyBlobs(bool hmap=false);
    std::vector< std::vector<Circle> > goCirclePacking(double min_r=0);
    std::vector<cv::RotatedRect> goEllipse();

    Polygon getPolygonFromContours(const std::vector<cv::Point> contours);
    std::vector<Polygon> getPolygonsFromContours(std::vector< std::vector<cv::Point> > contours);
    std::vector<Cell> getCellsInPolygon(const Polygon& p); 
    bool cellInPoly(Polygon, Point);
    bool cellInPolyConcave(Polygon, Point);
    double getMinDistToPoly(const Polygon&, const Cell&);
    double getMinDistToCirs(const std::vector<Circle>&, const Cell&);
    void deleteCellsInCir(const std::vector<Cell>&, const Circle, std::vector<Cell>&);
    Circle                    fitCirOverContours(const std::vector<cv::Point> contours);
    std::vector<Circle>       packCirsIntoPoly(const Polygon p, const double min_r);


    CircleGroup               getGroupForContours(std::vector<cv::Point> contours, std::vector<CircleGroup>& largeObs, const double gridOriginX, const double gridOriginY, const double gridResolution, bool usingHMap=false);


    std::vector<CircleGroup>  getGroups(std::vector<CircleGroup>& staticObs, const double gridOriginX, const double gridOriginY, const double gridResolution, bool usingHMap=false);
    
    std::vector<CircleGroup>  getGroups(std::vector<CircleGroup>& staticObs, bool usingHMap=false);
    
    std::vector<CircleGroup>  getGroupsForStaticMap();
    std::vector<Cell> getCellsInPolygonStaticMap(const Polygon& p, const std::vector<cv::Point> contours); 
    
    visualization_msgs::Marker polygonMarker_;
    std::vector<visualization_msgs::Marker> pMarkers_;

    std::vector<visualization_msgs::Marker> cMarkers_; 

    nav_msgs::OccupancyGrid staticMap_;

  private:

    void drawContourPoints(std::vector< std::vector< cv::Point> > contours, std::vector<cv::Vec4i> hierarchy);
    visualization_msgs::Marker drawLines(const std::vector<Point>& points, const int id=50000);
    visualization_msgs::Marker drawPolygon(const Polygon& poly, const int id=50000);

    std::vector<visualization_msgs::Marker> drawCells(std::vector<Cell> cells);


    void LineLineEndPoints (const Point& l1_p1, Point& l1_p2, const Point& l2_p1, Point& l2_p2, std::vector<Point>& points_of_collision) const;


    Utility utility_;

    cv::Mat src, srcStaticMap;
    cv::Mat dst, detected_edges;

    nav_msgs::OccupancyGrid grid_;

    std::vector<visualization_msgs::Marker> markers_;

    int edgeThresh = 1;
    int lowThreshold;
    int const max_lowThreshold = 100;
    int ratio = 3;
    int kernel_size = 3;
    std::string window_name = "Edge Map";

    double obSizeThreshold = 3.0; // res = 5cm
};

#endif
