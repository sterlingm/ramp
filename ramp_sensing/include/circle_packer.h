#ifndef CIRCLE_PACKER
#define CIRCLE_PACKER
#include "utility.h"
#include "GridMap2D.h"
#include <nav_msgs/OccupancyGrid.h>
#include <queue>
#include <visualization_msgs/Marker.h>


class CirclePacker 
{
  public:
    CirclePacker(nav_msgs::OccupancyGridConstPtr);
    CirclePacker(nav_msgs::OccupancyGrid);
    CirclePacker(cv::Mat grid);
    ~CirclePacker();

    void convertOGtoMat(nav_msgs::OccupancyGridConstPtr);

    void CannyThreshold(int, void*);
    double getMinDistToPoly(const Polygon&, const Cell&);
    double getMinDistToCirs(const std::vector<Circle>&, const Cell&);
    void deleteCellsInCir(const std::vector<Cell>&, const Circle, std::vector<Cell>&);

    Normal computeNormal(Edge);
    bool cellInPoly(Polygon, cv::Point);

    void combineTwoCircles(const Circle a, const Circle b, Circle& result) const;
    void combineOverlappingCircles(std::vector<Circle> cs, std::vector<Circle>& result) const;

    std::vector<Polygon> getPolygonsFromContours(std::vector< std::vector<cv::Point> > contour_points) const;

    Point findCenterOfPixels(const std::vector<cv::Point> pixels) const;
    std::vector<double> getWeights(const std::vector<cv::Point> pixels, const Point center) const;
    
    std::vector<Circle> getCirclesFromPoly(Polygon);
    std::vector<Circle> getCirclesFromEdgeSets(const std::vector< std::vector<Edge> > edge_sets);
    std::vector<Circle> getCirclesFromEdges(const std::vector<Edge> edges, const cv::Point robot_cen);

    
    std::vector<Triangle> triangulatePolygon(const Polygon&);

    Circle getCircleFromKeypoint(const cv::KeyPoint k) const;
    std::vector<Circle> go();
    std::vector<Circle> goCorners();
    std::vector<Circle> goHough();
    std::vector<Circle> goMinEncCir();
    std::vector<Circle> goMyBlobs(bool hmap=false);
    std::vector<cv::RotatedRect> goEllipse();
  private:

    Utility utility_;

    cv::Mat src;
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
