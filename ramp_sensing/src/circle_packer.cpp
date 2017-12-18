#include "circle_packer.h"



CirclePacker::CirclePacker(nav_msgs::OccupancyGridConstPtr g)
{
  grid_ = *g;
  convertOGtoMat(g);
}

CirclePacker::CirclePacker(cv::Mat grid)
{
  grid.copyTo(src);
}

CirclePacker::~CirclePacker() 
{
  src.release();
  detected_edges.release();
  dst.release();
}






void CirclePacker::setNewGrid(nav_msgs::OccupancyGridConstPtr g)
{
  src.release();
  dst.release();
  detected_edges.release();

  grid_ = *g;
  convertOGtoMat(g);
}




void CirclePacker::convertOGtoMat(nav_msgs::OccupancyGridConstPtr g)
{
  ////ROS_INFO("In CirclePacker::convertOGtoMat");

  
  // Use the GridMap2D library to convert from nav_msgs::OccupancyGrid to cv::Mat
  gridmap_2d::GridMap2D gmap(g, false);
  ////ROS_INFO("Done with making GridMap2D");

  // Set src
  src = gmap.binaryMap();
  ////ROS_INFO("Done calling gmap.binaryMap()");
  
  ////ROS_INFO("Exiting CirclePacker::convertOGtoMat");
}

void CirclePacker::CannyThreshold(int, void*)
{
  /// Reduce noise with a kernel 3x3
  blur( src, detected_edges, cv::Size(3,3) );
  
  // Somehow, lowThreshold is being converted to unsigned int before this point
  // its value is 32767 (-1 for unsigned 4-byte int)
  // Set the value back to 0 for edge detection to work
  lowThreshold = 0;

  /// Canny detector
  cv::Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );

  /// Using Canny's output as a mask, we display our result
  dst = cv::Scalar::all(0);

  //std::cout<<"\nDetected Edges: "<<detected_edges;

  src.copyTo( dst, detected_edges);
  //cv::imshow("detected_edges", dst);
  //cv::waitKey(0);
  imshow( window_name, dst );
}





Normal CirclePacker::computeNormal(Edge e)
{
  //std::cout<<"\ne.start: "<<e.start.x<<" "<<e.start.y;
  //std::cout<<"\ne.end: "<<e.end.x<<" "<<e.end.y;
  Normal result;
  result.a = e.end.y - e.start.y;
  result.b = -(e.end.x - e.start.x);

  result.c = -((result.a*e.start.x) + (result.b*e.start.y));

  //std::cout<<"\na: "<<result.a<<" b: "<<result.b<<" c: "<<result.c;
  return result;
}

Polygon CirclePacker::tpplToPoly(TPPLPoly p)
{
  // Convert to Polygon object
  Polygon result;
  
  for(int j=0;j<p.GetNumPoints();j++)
  {
    Edge e;
    e.start.x = p.GetPoint(j).x;
    e.start.y = p.GetPoint(j).y;
    
    if(j == p.GetNumPoints()-1)
    {
      e.end.x = p.GetPoint(0).x;
      e.end.y = p.GetPoint(0).y;
    }
    else
    {
      e.end.x = p.GetPoint(j+1).x;
      e.end.y = p.GetPoint(j+1).y;
    }

    Normal n = computeNormal(e);
    result.edges.push_back(e);
    result.normals.push_back(n);
  }

  return result;
}

TPPLPoly CirclePacker::polyToTPPL(Polygon p)
{
  TPPLPoly result;
  
  result.Init(p.edges.size());
  result.SetHole(false);
  result.SetOrientation(TPPL_CCW);

  for(int i=0;i<p.edges.size();i++)
  {
    result.GetPoints()[i].x = p.edges[i].start.x;
    result.GetPoints()[i].y = p.edges[i].start.y;
  }

  return result;
}


std::vector<TPPLPoly> CirclePacker::triPolyPart(const Polygon& p)
{
  ROS_INFO("In CirclePacker::triPolyPart");

  // Create the polygon object
  TPPLPoly tppoly;
  tppoly.Init(p.edges.size());

  // Push on the points
  for(int i=0;i<p.edges.size();i++)
  {
    tppoly.GetPoints()[i].x = p.edges[i].start.x;
    tppoly.GetPoints()[i].y = p.edges[i].start.y;
  }
  
  // Polygons won't have holes
  tppoly.SetHole(false);

  // Opencv generates contour points in ccw order
  tppoly.SetOrientation(TPPL_CCW);

  std::list<TPPLPoly> tris;

  TPPLPartition part;
  if(part.Triangulate_OPT(&tppoly, &tris) == 0)
  {
    ROS_INFO("TRIANGULATION FAILED");
  }
  ROS_INFO("TRI LIST SIZE: %i", (int)tris.size());

  std::vector<TPPLPoly> triVec;
  while(tris.size() > 0)
  {
    triVec.push_back(tris.front());
    tris.pop_front();
  }

  for(int i=0;i<triVec.size();i++)
  {
    ROS_INFO("Tri %i", i);
    for(int j=0;j<triVec[i].GetNumPoints();j++)
    {
      ROS_INFO("  (%f,%f)", triVec[i].GetPoint(j).x, triVec[i].GetPoint(j).y);
    }
  }
  
  ROS_INFO("Exiting CirclePacker::triPolyPart");
  return triVec;
}


std::vector<Triangle> CirclePacker::triangulatePolygon(const Polygon& poly)
{
  std::vector<Triangle> result;

  // Get all vertices of polygon
  std::vector<Point> vertices;
  for(int i=0;i<poly.edges.size();i++)
  {
    vertices.push_back(poly.edges[i].start);
  }
  
  std::vector<int> i_reflex;

  // For each vertex, get its two neighbors
  // Check if line connecting them is in polygon
  for(int i=0;i<vertices.size();i++)
  {
    std::cout<<"\nVertex "<<i<<": ("<<vertices[i].x<<", "<<vertices[i].y<<")";

    // Get neighbors
    Point v0 = i == 0 ? vertices[vertices.size()-2] : vertices[i-1];
    Point v1 = vertices[i];
    Point v2 = i == vertices.size() - 1 ? vertices[0] : vertices[i+1];

    std::cout<<"\nNeighbors: ("<<v0.x<<", "<<v0.y<<") ("<<v2.x<<", "<<v2.y<<")";
   
    // Get direction angle of each segment
    // y component negated because grid y increases as it goes down
    double ax = v0.x - v1.x;
    double bx = v2.x - v1.x;
    double ay = -( v0.y - v1.y );
    double by = -( v2.y - v1.y );
    std::cout<<"\nax: "<<ax<<" ay: "<<ay<<" bx: "<<bx<<" by: "<<by;

    double ta = atan2(ay , ax);
    double tb = atan2(by , bx);
    std::cout<<"\nta: "<<ta<<" tb: "<<tb;
   
    double t_final = fmodf(ta - tb, 6.28);
    std::cout<<"\nt_final: "<<t_final;

    if(t_final > 3.14159)
    {
      i_reflex.push_back(i);
    }
  } // end for each vertex


  return result;
}


void CirclePacker::combineTwoCircles(const Circle a, const Circle b, Circle& result) const
{
  // Get the midpoint between the arcles
  std::vector<double> vec, midpoint;
  vec.push_back( b.center.x - a.center.x );
  vec.push_back( b.center.y - a.center.y );

  ////////ROS_INFO("vec: [%f, %f]", vec[0], vec[1]);

  midpoint.push_back( a.center.x + (0.5*vec[0]) ); 
  midpoint.push_back( a.center.y + (0.5*vec[1]) ); 

  ////////ROS_INFO("midpoint: (%f, %f)", midpoint[0], midpoint[1]);

  // Get the distance from the midpoint to each center
  double d_mid_i = utility_.positionDistance(midpoint[0], midpoint[1], a.center.x, a.center.y);
  double d_mid_ii = utility_.positionDistance(midpoint[0], midpoint[1], b.center.x, b.center.y);

  double R = d_mid_i > d_mid_ii ? d_mid_i+a.radius : d_mid_ii+b.radius;

  ////////ROS_INFO("d_mid_i: %f d_mid_ii: %f R: %f", d_mid_i, d_mid_ii, R);

  result.center.x = midpoint[0];
  result.center.y = midpoint[1];
  result.radius = R;
}

/*
 * Make sure attachments don't override each other
 */
void CirclePacker::detectAttachedCircles(const std::vector<CircleOb*>& cir_obs, std::vector<Attachment>& result) const
{
  //ROS_INFO("In detectAttachedCircles");

  double threshold  = 0;
  double R          = 0;
  double d          = 0;
  double scale      = 0.1;
  int i=0;
  while(i<cir_obs.size()-1)
  {
    Circle ci = cir_obs[i]->cir;

    int j = i+1;

    // Go through remaining circles
    while(j<cir_obs.size())
    {
      Circle cj = cir_obs[j]->cir;
      
      //ROS_INFO("Trying i: %i j: %i", i, j);
      //ROS_INFO("Centers i: (%f,%f) j: (%f,%f)", ci.center.x, ci.center.y, cj.center.x, cj.center.y);
      
      // Get R, distance threshold, and distance between circle centers
      R = ci.radius + cj.radius;
      threshold = ci.radius < cj.radius ? R-(scale*ci.radius) : R-(scale*cj.radius);
      threshold = R - 0.1;

      d = utility_.positionDistance(ci.center.x, ci.center.y, cj.center.x, cj.center.y);

      //ROS_INFO("ci.r: %f cj.r: %f R: %f threshold: %f d: %f", ci.radius, cj.radius, R, threshold, d);

      // If distance is below threshold, then attach the two circles
      if(d < threshold)
      {
        //ROS_INFO("Creating attachment %i and %i", i, j);
        Attachment temp;
        temp.cirs.push_back(i);
        temp.cirs.push_back(j);
        result.push_back(temp);
      }

      j++;
    } // end inner while

    i++;
  } // end outer while

  //ROS_INFO("Exiting detectAttachedCircles"); 
}

// result is a final list of circles: contains both the combined ones and the ones that were not combined
// This gets called before converting to global coordinates 1 = 5cm = 0.05m
void CirclePacker::combineOverlappingCircles(std::vector<Circle> cs, std::vector<Circle>& result) const
{
  //ROS_INFO("In combineOverlappingCircles");
  int pairs, i=0, j=0;

  // Initialize variables
  double scale = 0.75;
  double threshold = 0.;

  while(i<cs.size()-1)
  {
    //////////ROS_INFO("i: %i", i);
    Circle ci = cs[i];
    //////////ROS_INFO("ci - Center: (%f, %f) Radius: %f", ci.center.x, ci.center.y, ci.radius);
    
    j = i+1;


    while(j<cs.size())
    {
      //////////ROS_INFO("j: %i", j);

      // Check if they overlap
      Circle cj = cs[j];
      //////////ROS_INFO("cj - Center: (%f, %f) Radius: %f", cj.center.x, cj.center.y, cj.radius);

      double R = ci.radius + cj.radius;
      threshold = ci.radius < cj.radius ? R-(scale*ci.radius) : R-(scale*cj.radius);

      double d = utility_.positionDistance(ci.center.x, ci.center.y, cj.center.x, cj.center.y);

      if(d < threshold)
      {
        //ROS_INFO("Combining circles d: %f threshold: %f", d, threshold);

        // Combine them
        Circle temp;
        combineTwoCircles(ci, cj, temp);
        //////////ROS_INFO("Result - Center: (%f, %f) Radius: %f", temp.center.x, temp.center.y, temp.radius);

        // If combined, replace both circles with overlapping circle
        // Replace i by setting it to temp, erase the circle at j
        result.push_back(temp);
        cs[i] = temp;
        ci = temp;
        cs.erase(cs.begin()+j, cs.begin()+j+1);

        // Then, decrement j to get next circle for comparison
        j--;
      }

      j++;
    } // end inner while

    i++;
  } // end outter while

  result = cs;
} // End combineOverlappingCircles




Point CirclePacker::findCenterOfPixels(const std::vector<cv::Point> pixels) const
{
  Point result;
  if(pixels.size() > 0)
  {
    int x_min = pixels[0].x, y_min = pixels[0].y, x_max = x_min, y_max = y_min;

    for(int i=1;i<pixels.size();i++)
    {
      if(pixels[i].x < x_min)
      {
        x_min = pixels[i].x;
      }
      if(pixels[i].x > x_max)
      {
        x_max = pixels[i].x;
      }
      if(pixels[i].y < y_min)
      {
        y_min = pixels[i].y;
      }
      if(pixels[i].y > y_max)
      {
        y_max = pixels[i].y;
      }
    }
  
    result.x = (x_min + x_max) / 2.f;
    result.y = (y_min + y_max) / 2.f;
  }

  return result;
}

std::vector<double> CirclePacker::getWeights(const std::vector<cv::Point> pixels, const Point center) const
{
  std::vector<double> result;

  int weight = 2;

  for(int i=0;i<pixels.size();i++)
  {
    double dist = utility_.positionDistance(pixels[i].x, pixels[i].y, center.x, center.y);

    result.push_back(weight * dist); 
  }

  return result;
}

std::vector<Circle> CirclePacker::getCirclesFromEdgeSets(const std::vector< std::vector<Edge> > edge_sets)
{
  std::vector<Circle> result;
  //////////ROS_INFO("In CirclePacker::getCirclesFromEdgeSets");

 
  std::vector<Point> means;

  // For each edge set (i.e. polygon)
  for(int i=0;i<edge_sets.size();i++)
  {
    ////////ROS_INFO("Edge set %i", i);
    Point temp_center;
  
    // For each set of edges, find the minimum and maximum values for x and y
    // Find the mean of x and y
    int x_min = edge_sets[i][0].start.x, 
    y_min = edge_sets[i][0].start.y, 
    x_max = x_min, 
    y_max = y_min; 
    int x_mean = edge_sets[i][0].start.x;
    int y_mean = edge_sets[i][0].start.y;
    for(int j=1;j<edge_sets[i].size();j++)
    {
      x_mean += edge_sets[i][j].start.x;
      y_mean += edge_sets[i][j].start.y;

      ////////ROS_INFO("\tEdge %i - start: (%i,%i) end: (%i,%i)", j, edge_sets[i][j].start.y, edge_sets[i][j].start.x, edge_sets[i][j].end.y, edge_sets[i][j].end.x);

      // Get the minimum and maximum x and y values to compute the circle's radius
      if( edge_sets[i][j].start.x < x_min )
      {
        x_min = edge_sets[i][j].start.x;
      } 
      if( edge_sets[i][j].start.x > x_max )
      {
        x_max = edge_sets[i][j].start.x;
      } 
      if( edge_sets[i][j].start.y < y_min )
      {
        y_min = edge_sets[i][j].start.y;
      } 
      if( edge_sets[i][j].start.y > y_max )
      {
        y_max = edge_sets[i][j].start.y;
      }
    } // end inner for

    x_mean /= edge_sets[i].size();
    y_mean /= edge_sets[i].size();

    // Swap x and y
    temp_center.x = y_mean;
    temp_center.y = x_mean;
    means.push_back(temp_center);
    
    // For each edge set, build a Data object
    /*Data d(edge_sets[i].size(), X.data(), Y.data());
    CircleFit cf = CircleFitFitByTaubin(d);
    cf.print();

    CircleFit out;
    CircleFitByLevenbergMarquardtFull(d, cf, 0.1, out);
    out.print();*/
    

    //////////ROS_INFO("\tx_min: %i x_max: %i y_min: %i y_max: %i", x_min, x_max, y_min, y_max);

    // Get difference between min+max for both x and y
    double x_diff = fabs(x_max - x_min);
    double y_diff = fabs(y_max - y_min);

    //////////ROS_INFO("\tx_diff: %f y_diff: %f", x_diff, y_diff);

    // Set radius to half of the largest difference (half because difference would be diameter)
    double r = x_diff > y_diff ? x_diff/2.f : y_diff/2.f;

    /*  
     * Find approximate center by taking half of min and max in both directions
     */
    //****************************************************************************
    // FLIP/SWAP X AND Y!!
    // This is necessary because when Gridmap converts between
    // a costmap and a Mat, it flips the costmap values to conform 
    // to the OpenCV image coordinate system 
    // which starts in the top left corner, x points right, y points down
    //****************************************************************************

    Circle temp;

    // Translate the center by 0.075cm in both directions
    // Inflate radius by 20cm
    temp.radius = r+4;
    temp.center = temp_center;
    temp.center.x += 1.5;
    temp.center.y += 1.5;
    
    ////////ROS_INFO("\tCenter: (%f,%f) Radius: %f", temp.center.x, temp.center.y, temp.radius);

    result.push_back(temp);
  } // end outter for
  
  return result;
}

std::vector<Circle> CirclePacker::getCirclesFromEdges(const std::vector<Edge> edges, const cv::Point robot_cen)
{
  std::vector<Circle> result;

  for(int i=0;i<edges.size();i++)
  {
    Circle temp;

    //////////ROS_INFO("Edge endpoints: (%i,%i) (%i,%i)", edges[i].start.x, edges[i].start.y, edges[i].end.x, edges[i].end.y);

    // Get length of edge to use as diameter of circle
    double dist = sqrt( pow(edges[i].end.x - edges[i].start.x, 2) + pow(edges[i].end.y - edges[i].start.y, 2) );
    temp.radius = dist/2.f;
    
    // Get the midpoint of the edge
    double x_mid = (edges[i].end.x + edges[i].start.x) / 2.f;
    double y_mid = (edges[i].end.y + edges[i].start.y) / 2.f;
    
    // Get angle between robot center and edge midpoint
    std::vector<double> rob_cen; 
    rob_cen.push_back(robot_cen.x); 
    rob_cen.push_back(robot_cen.y);
    std::vector<double> edge_mid;
    edge_mid.push_back(x_mid);
    edge_mid.push_back(y_mid);
    
    double theta = utility_.findAngleFromAToB(rob_cen, edge_mid); 
    double phi = utility_.displaceAngle(PI, theta);

    // Get circle center with phi
    double psi = utility_.displaceAngle(phi, PI);
    double delta_x = temp.radius*cos(psi);
    double delta_y = temp.radius*sin(psi);

    double x_cen = x_mid + delta_x;
    double y_cen = y_mid + delta_y;

    //////////ROS_INFO("Edge midpoint: (%f, %f) theta: %f phi: %f psi: %f Circle center: (%f, %f)", x_mid, y_mid, theta, phi, psi, x_cen, y_cen);

    temp.center.x = x_cen;
    temp.center.y = y_cen;

    result.push_back(temp);
  }

  return result;
}


Circle CirclePacker::getCircleFromKeypoint(const cv::KeyPoint k) const
{
  Circle result;

  // Swap x and y because of axis flipping
  result.center.x = k.pt.y;
  result.center.y = k.pt.x;

  result.radius = k.size;

  return result;
}


std::vector<Circle> CirclePacker::go()
{
  ////////ROS_INFO("In CirclePacker::go()");
  std::vector<Circle> result;

  // Create a matrix of the same size and type as src
  dst.create( src.size(), src.type() );
  ////////ROS_INFO("Done with dst.create");

  // Convert to grayscale
  //cvtColor(src, src_gray, CV_BGR2GRAY);

  // Get the edges
  ros::Time t_start_edge_detect = ros::Time::now();
  CannyThreshold(0, 0);
  ros::Duration d_edges_detect(ros::Time::now()-t_start_edge_detect);
  ////////ROS_INFO("Done with CannyThreshold");

  /*
   * Detect blobs
   */

  // Setup params
  cv::SimpleBlobDetector::Params params;
  params.minDistBetweenBlobs = 1.f;
  params.filterByInertia = false;
  params.filterByConvexity = false;
  params.filterByColor = false;
  params.filterByCircularity = false;
  params.filterByArea = true;
  params.minArea = 5.f;
  params.maxArea = 5000.0f;

  // Make object
  // Ptr line works with my work machine, but I get an error about the create(params) method on my laptop
  //cv::Ptr<cv::SimpleBlobDetector> blobs_detector = cv::SimpleBlobDetector::create(params);   
  cv::SimpleBlobDetector blobs_detector(params);   

  // Detect blobs
  std::vector<cv::KeyPoint> keypoints;
  ros::Time t_start = ros::Time::now();
  blobs_detector.detect(src, keypoints);
  ros::Duration d_blobs = ros::Time::now() - t_start;
  ////////ROS_INFO("d_blobs: %f", d_blobs.toSec());

  ////////ROS_INFO("Keypoints size: %i", (int)keypoints.size());

  for(int i=0;i<keypoints.size();i++)
  {
    ////////ROS_INFO("Keypoint %i: pt: (%f, %f) class_id: %i angle: %f size: %f", i, keypoints[i].pt.x, keypoints[i].pt.y, keypoints[i].class_id, keypoints[i].angle, keypoints[i].size);
    result.push_back(getCircleFromKeypoint(keypoints[i]));
  }
  return result;
}


std::vector<Circle> CirclePacker::goCorners()
{
  ////////ROS_INFO("In CirclePacker::go()");
  std::vector<Circle> result;

  // Create a matrix of the same size and type as src
  dst.create( src.size(), src.type() );
  ////////ROS_INFO("Done with dst.create");

  // Convert to grayscale
  //cvtColor(src, src_gray, CV_BGR2GRAY);

  // Get the edges
  ros::Time t_start_edge_detect = ros::Time::now();
  CannyThreshold(0, 0);
  ros::Duration d_edges_detect(ros::Time::now()-t_start_edge_detect);

  /*
   * Detect corners
   */
  int blockSize = 4;
  int apertureSize = 9;
  double k = 0.04;
  ros::Time t_start_corner_detect = ros::Time::now();
  cv::cornerHarris(src, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);
  ros::Duration d_corner_detect = ros::Time::now() - t_start_corner_detect;
  ////////ROS_INFO("d_corner_detect: %f", d_corner_detect.toSec());

  ////////ROS_INFO("dst.rows: %i dst.cols: %i", dst.rows, dst.cols);
  for(int j=0;j<dst.rows;j++)
  {
    for(int i=0;i<dst.cols;i++)
    {
      if( (int) dst.at<float>(j,i) > 200)
      {
        Circle c;
        c.center.x = j;
        c.center.y = i;
        c.radius = 10;
        result.push_back(c);
      }
    }
  }

  return result;
}


std::vector<cv::RotatedRect> CirclePacker::goEllipse()
{
  ////////ROS_INFO("In CirclePacker::go()");
  std::vector<cv::RotatedRect> result;

  // Create a matrix of the same size and type as src
  dst.create( src.size(), src.type() );
  ////////ROS_INFO("Done with dst.create");

  // Convert to grayscale
  //cvtColor(src, src_gray, CV_BGR2GRAY);

  // Get the edges
  ros::Time t_start_edge_detect = ros::Time::now();
  CannyThreshold(0, 0);
  ros::Duration d_edges_detect(ros::Time::now()-t_start_edge_detect);

  // Get contours
  std::vector< std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;
  findContours( src, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );  

  // Get ellipses
  std::vector<cv::RotatedRect> minEllipse( contours.size() );
  for(int i=0;i<contours.size();i++)
  {
    minEllipse[i] = fitEllipse(cv::Mat(contours[i]));
    ////////ROS_INFO("Ellipse %i: (%f, %f)", i, minEllipse[i].center.x, minEllipse[i].center.y);
  }

  return minEllipse;
}


std::vector<Circle> CirclePacker::goHough()
{
  ////////ROS_INFO("In CirclePacker::go()");
  std::vector<Circle> result;

  // Create a matrix of the same size and type as src
  dst.create( src.size(), src.type() );
  ////////ROS_INFO("Done with dst.create");

  // Convert to grayscale
  //cvtColor(src, src_gray, CV_BGR2GRAY);

  std::vector<cv::Vec3f> circles;
  cv::HoughCircles(src, circles, CV_HOUGH_GRADIENT, 0.5, 5, 200, 10, 10, 0);

  for(int i=0;i<circles.size();i++)
  {
    Circle temp;
    temp.center.x = circles[i][1];
    temp.center.y = circles[i][0];
    temp.radius = circles[i][2];

    result.push_back(temp);
  }

  return result;
}


std::vector<Circle> CirclePacker::goMinEncCir()
{
  std::vector<Circle> result;

  dst.create( src.size(), src.type() );
  
  // Get the edges
  ros::Time t_start_edge_detect = ros::Time::now();
  CannyThreshold(0, 0);
  ros::Duration d_edges_detect(ros::Time::now()-t_start_edge_detect);

  // Get contours
  std::vector< std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;
  findContours( src, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );  

  std::vector<cv::Point2f> centers(contours.size());
  std::vector<float> radii(contours.size());
  // For each contour, find the minimum enclosing circle
  for(int i=0;i<contours.size();i++)
  {
    cv::minEnclosingCircle(contours[i], centers[i], radii[i]);

    Circle temp;
    temp.center.x = centers[i].y;
    temp.center.y = centers[i].x;
    temp.radius = radii[i];
    
    result.push_back(temp);    
  }


  return result;
}

Polygon CirclePacker::getPolygonFromContours(const std::vector<cv::Point> contours)
{
  Polygon result;

  double translate = 0.5;

  /*Point midpoint;
  for(int i=0;i<contours.size();i++)
  {
    midpoint.x += contours[i].x;
    midpoint.y += contours[i].y;
  }
  midpoint.x /= contours.size();
  midpoint.y /= contours.size();*/
  //ROS_INFO("Midpoint: (%f,%f)", midpoint.x, midpoint.y);

  // Get edges
  for(int j=0;j<contours.size();j++)
  {
    Edge e;
    
    e.start.x = contours[j].x + translate;
    e.start.y = contours[j].y + translate;
    if(j == contours.size()-1)
    {
      e.end.x = contours[0].x + translate;
      e.end.y = contours[0].y + translate;
    }
    else
    {
      e.end.x   = contours[j+1].x + translate;
      e.end.y   = contours[j+1].y + translate;
    }

    //ROS_INFO("e.start: (%f,%f) e.end: (%f,%f)", e.start.x, e.start.y, e.end.x, e.end.y);
    result.edges.push_back(e);
  }

  // Get normals
  for(int j=0;j<result.edges.size();j++)
  {
    Normal n;
    
    n.a = result.edges[j].end.y - result.edges[j].start.y;
    n.b = result.edges[j].start.x - result.edges[j].end.x;
    n.c = -n.a*result.edges[j].start.x - n.b*result.edges[j].start.y;

    // Dot product with next edge
    int k = j < result.edges.size()-1 ? j : 0;
    double d = n.a * (result.edges[k].end.x - result.edges[k].start.x) + n.b * (result.edges[k].end.y - result.edges[k].start.y);
    if(d > 0)
    {
      n.a = -n.a;
      n.b = -n.b;
      n.c = -n.c;
    }

    result.normals.push_back(n);
  }

  return result;
}

/*
 * Returns a vector of Polygons - one for each set of contour points
 */ 
std::vector<Polygon> CirclePacker::getPolygonsFromContours(std::vector< std::vector<cv::Point> > contours)
{
  //ROS_INFO("In getPolygonFromContours");
  std::vector<Polygon> result;

  // Set the edges for each polygon
  for(int i=0;i<contours.size();i++)
  {
    result.push_back(getPolygonFromContours(contours[i]));
  }
  //ROS_INFO("result.size(): %i", (int)result.size());

  return result;
}


/*
 * Returns the minimum distance between a cell and any edge on a Polygon
 */ 
double CirclePacker::getMinDistToPoly(const Polygon& poly, const Cell& cell)
{
  //ROS_INFO("In getMinDistToPoly");
  //ROS_INFO("Cell: (%f,%f)", cell.p.x, cell.p.y);
  
  double result = 100000;

  for(int n=0;n<poly.normals.size();n++)
  {
    //ROS_INFO("poly.edges[%i].start: (%f,%f) end: (%f,%f)", n, poly.edges[n].start.x, poly.edges[n].start.y, poly.edges[n].end.x, poly.edges[n].end.y);  
    //ROS_INFO("Poly.normals[%i]: <%f,%f> c=%f", n, poly.normals[n].a, poly.normals[n].b, poly.normals[n].c);
    
    // Get unit normal
    double l = sqrt( pow(poly.normals[n].a,2) + pow(poly.normals[n].b,2) );
    Normal e_hat;
    e_hat.a = poly.normals[n].a / l;
    e_hat.b = poly.normals[n].b / l;
    
    // Vector from cell to start of edge
    std::vector<double> sc;
    sc.push_back(cell.p.x - poly.edges[n].start.x);
    sc.push_back(cell.p.y - poly.edges[n].start.y);
    
    // Get length of vectors SC and AD (dot product of SC and normal)
    double scLen = sqrt( pow(sc[0],2) + pow(sc[1],2) );
    double adLen = sc[0]*e_hat.a + sc[1]*e_hat.b;

    double d = sqrt( pow(scLen,2) + pow(adLen,2) );
    
    //ROS_INFO("l: %f ehat: <%f,%f> sc: <%f,%f> d: %f", l, e_hat.a, e_hat.b, sc[0], sc[1], d);
    if(d < result)
    {
      result = d;
    }
  }

  return result;
}



/*
 * Returns the minimum distance between a cell and a vector of circles
 */ 
double CirclePacker::getMinDistToCirs(const std::vector<Circle>& cirs, const Cell& cell)
{
  if(cirs.size() == 0)
  {
    return -1;
  }

  double result=10000;

  for(int i=0;i<cirs.size();i++)
  {
    double dist = sqrt( pow( cell.p.x - cirs[i].center.x, 2) + pow( cell.p.y - cirs[i].center.y, 2) );
    
    // Then, subtract the radius to get the dist to the outside of the circle
    dist -= cirs[i].radius;

    if( dist < result)
    {
      result = dist;
    }
  }

  return result;
}


/*
 * Returns true if the center of the cell is inside the Polygon
 */ 
bool CirclePacker::cellInPoly(Polygon poly, Point cell)
{
  //ROS_INFO("In cellInPoly, cell: (%f,%f)", cell.x, cell.y);
  for(int i=0;i<poly.normals.size();i++)
  {
    //std::cout<<"\nnormal a: "<<poly.normals[i].a<<" b: "<<poly.normals[i].b<<" c: "<<poly.normals[i].c;
    double d = poly.normals[i].a*cell.x + poly.normals[i].b*cell.y + poly.normals[i].c;
    //ROS_INFO("a: %f b: %f c: %f dist: %f", poly.normals[i].a, poly.normals[i].b, poly.normals[i].c, d);
    //std::cout<<"\ncell center: "<<cell.x<<", "<<cell.y<<" d: "<<d;
    if(d > 1.5)
    {
      //std::cout<<"\nNot in polygon";
      return false;
    }
  }

  return true;
}
    
void CirclePacker::LineLineEndPoints (const Point& l1_p1, Point& l1_p2, const Point& l2_p1, Point& l2_p2, std::vector<Point>& points_of_collision) const
{
  //ROS_INFO("In CirclePacker::LineLineEndPoints");
  //ROS_INFO("Segment 1: (%f,%f) - (%f,%f) Segment 2: (%f,%f) - (%f,%f)", l1_p1.x, l1_p1.y, l1_p2.x, l1_p2.y, l2_p1.x, l2_p1.y, l2_p2.x, l2_p2.y);

  // Get line equation coefficients
  double l1_slope = (l1_p2.y - l1_p1.y) / (l1_p2.x - l1_p1.x);
  double l1_b = l1_p2.y - (l1_slope*l1_p2.x);
  
  double l2_slope = (l2_p2.y - l2_p1.y) / (l2_p2.x - l2_p1.x);
  double l2_b = l2_p2.y - (l2_slope*l2_p2.x);

  //ROS_INFO("l1_slope: %f l1_b: %f l2_slope: %f l2_b: %f", l1_slope, l1_b, l2_slope, l2_b);

  // Flag to toggle if are using x or y
  double intersectValue;


  // Parallel lines
  // Check that they are not the same line!
  if( fabs(l2_slope - l1_slope) < 0.01 )
  {
    //ROS_INFO("Lines are parallel");
    
    // Same line
    if( fabs(l1_slope - l2_slope) < 0.01 && fabs(l1_b - l2_b) < 0.01 )
    {
      //ROS_INFO("Same Line");
      Point temp;
      temp.x = 9999;
      temp.y = 9999;
      points_of_collision.push_back(temp);
    }
  }
  else if(std::isinf(l1_slope))
  {
    intersectValue = l1_p1.x;
  }
  else if(std::isinf(l2_slope))
  {
    intersectValue = l2_p1.x;
  }
  else
  {
    // Find the x-coordinate of intersection
    intersectValue = (-(l1_b - l2_b)) / (l1_slope - l2_slope);
  }

  /*
   *  Compute the y-value at intersection and set ranges for x and y
   */
  double yIn = (l1_slope * intersectValue) + l1_b;
  double l1_x_min = l1_p1.x;
  double l1_x_max = l1_p2.x;
  double l1_y_min = l1_p1.y;
  double l1_y_max = l1_p2.y;
  double l2_x_min = l2_p1.x;
  double l2_x_max = l2_p2.x;
  double l2_y_min = l2_p1.y;
  double l2_y_max = l2_p2.y;

  // Check if we need to swap values
  if(l1_x_min > l1_x_max)
  {
    double temp = l1_x_min;
    l1_x_min = l1_x_max;
    l1_x_max = temp;
  }
  if(l1_y_min > l1_y_max)
  {
    double temp = l1_y_min;
    l1_y_min = l1_y_max;
    l1_y_max = temp;
  }

  if(l2_x_min > l2_x_max)
  {
    double temp = l2_x_min;
    l2_x_min = l2_x_max;
    l2_x_max = temp;
  }
  if(l2_y_min > l2_y_max)
  {
    double temp = l2_y_min;
    l2_y_min = l2_y_max;
    l2_y_max = temp;
  }
    

  //ROS_INFO("intersectValue: %f yIn: %f l1 (xmin,xmax): (%f,%f) (ymin,ymax): (%f,%f) l2 (xmin,xmax): (%f,%f) (ymin,ymax): (%f,%f)", intersectValue, yIn, l1_x_min, l1_x_max, l1_y_min, l1_y_max, l2_x_min, l2_x_max, l2_y_min, l2_y_max);
 
  // If the x and y values at the intersection are within the line ranges (i.e., on the line segment), then add the point
  if( (intersectValue >= l1_x_min && intersectValue <= l1_x_max) && 
      (intersectValue >= l2_x_min && intersectValue <= l2_x_max) && 
      (yIn >= l1_y_min && yIn <= l1_y_max) && 
      (yIn >= l2_y_min && yIn <= l2_y_max)) 

  {
    //ROS_INFO("In if lines intersect");
    Point p_intersect;
    double x_at_inter = intersectValue;
    double y_at_inter = l1_slope*x_at_inter + l1_b;
    p_intersect.x = x_at_inter;
    p_intersect.y = y_at_inter;
    
    points_of_collision.push_back(p_intersect);
    //ROS_INFO("x_at_inter: %f y_at_inter: %f", x_at_inter, y_at_inter);
  }
}

bool CirclePacker::cellInPolyConcave(Polygon poly, Point cell)
{
  // Extend a ray to the right of the cell
  Point endRay;
  endRay.x = 70; // 3.5 (x_max) / 0.05 (resolution)
  endRay.y = cell.y; 
  
  //ROS_INFO("In cellInPoly, cell: (%f,%f)", cell.x, cell.y);

  std::vector<Point> collPoints;
  for(int i=0;i<poly.edges.size();i++)
  {
    LineLineEndPoints(cell, endRay, poly.edges[i].start, poly.edges[i].end, collPoints);
  }

  //ROS_INFO("collPoints.size(): %i", (int)collPoints.size());
  return collPoints.size() % 2 == 1;

}


/*
 * Returns a vector of Cell objects that overlap with the Polygon.
 * The center of the cells may not be in the Polygon (this is detected with cellInPoly method)
 */ 
std::vector<Cell> CirclePacker::getCellsInPolygon(const Polygon& poly)
{
  std::vector<Cell> result;
  
  /*
   *  Get all vertices of polygon
   */
  std::vector<Point> vertices;
  for(int i=0;i<poly.edges.size();i++)
  {
    //ROS_INFO("Poly edge %i: (%f,%f)", i, poly.edges[i].start.x, poly.edges[i].start.y);
    vertices.push_back(poly.edges[i].start);
  }
  
  
  /*
   *  Find minimum and maximum x and y
   */
  double MAX_LENGTH= vertices[0].y;
  double MAX_WIDTH = vertices[0].x;
  double MIN_LENGTH= vertices[0].y;
  double MIN_WIDTH = vertices[0].x;
  for(int i=0;i<vertices.size();i++)
  {
    if(vertices[i].y > MAX_LENGTH)
    {
      MAX_LENGTH = vertices[i].y;
    }
    if(vertices[i].y < MIN_LENGTH)
    {
      MIN_LENGTH = vertices[i].y;
    }
    
    if(vertices[i].x > MAX_WIDTH)
    {
      MAX_WIDTH = vertices[i].x;
    }
    if(vertices[i].x < MIN_WIDTH)
    {
      MIN_WIDTH = vertices[i].x;
    }
  }
  //ROS_INFO("MIN W(x),L(y): %f, %f MAX W(x),L(y): %f, %f", MIN_WIDTH, MIN_LENGTH, MAX_WIDTH, MAX_LENGTH);

  double round = 1;

  // Find number of cells in both directions
  int width_count = (MAX_WIDTH - MIN_WIDTH) / round;
  int length_count = (MAX_LENGTH - MIN_LENGTH) / round;

  // Start from the center
  // Add round/2 because taht makes the cell center be the center of squares formed by grid lines
  double start_x = MIN_WIDTH + round/2.f;
  double start_y = MIN_LENGTH + round/2.f;
  //double start_x = MIN_WIDTH;
  //double start_y = MIN_LENGTH;

  //ROS_INFO("width_count: %i length_count: %i start_x: %f start_y: %f", width_count, length_count, start_x, start_y);

  /*
   * Check each cell in bounds
   */ 
  for(int i=0;i<width_count;i++)
  {
    for(int j=0;j<length_count;j++)
    {
      double x = start_x + (round * (i)); 
      double y = start_y + (round * (j));
      //ROS_INFO("i: %i j: %i x: %f y: %f", i, j, x, y);
      Cell temp;
      temp.p.x = x;
      temp.p.y = y;
    
      //std::cout<<"\n("<<temp.p.x<<", "<<temp.p.y<<")";

      if(cellInPolyConcave(poly, temp.p))
      {
        //ROS_INFO("Cell in poly");
        result.push_back(temp);
      }
      else
      {
        //ROS_INFO("Cell not in poly");
      }
    }
  }

  return result;
}



/*
 * Put any element in cells arg that does not overlap with cir into result
 * deleteCells is not the best name because we are inserting cells into the result
 * rename this at some point...
 */ 
void CirclePacker::deleteCellsInCir(const std::vector<Cell>& cells, const Circle cir, std::vector<Cell>& result)
{
  //std::cout<<"\nIn deleteCellsInCir\n";
  for(int i=0;i<cells.size();i++)
  {
    //std::cout<<"\nTesting cell "<<cells[i].p.x<<", "<<cells[i].p.y;
    //std::cout<<"\nDist: "<<(sqrt( pow( cir.center.x - cells[i].p.x, 2) + pow( cir.center.y - cells[i].p.y, 2) ));
    // Get distance between center of circle and the cell, check if > its radius
    if( sqrt( pow( cir.center.x - cells[i].p.x, 2) + pow( cir.center.y - cells[i].p.y, 2) ) > cir.radius )
    {
      result.push_back(cells[i]);
    }
  }
  //std::cout<<"\nExiting deleteCellsInCir\n";
}


/*
 * Returns a list of Circles that can be packed into a polygon.
 * min_r specifies a minimum radius (algorithm stops when circles are below min_r)
 */
std::vector<Circle> CirclePacker::packCirsIntoPoly(Polygon poly, double min_r)
{
  //ROS_INFO("In packCirsIntoPoly");
  std::vector<Circle> result;

  // Print polygon information
  /*ROS_INFO("Polygon");
  for(int j=0;j<poly.edges.size();j++)
  {
    ROS_INFO("  Edge %i - Start: (%i,%i) End: (%i,%i)", j, poly.edges[j].start.x, poly.edges[j].start.y, poly.edges[j].end.x, poly.edges[j].end.y);
  }*/

  /*
   * Create cells inside the polygon
   */
  std::vector<Cell> cells = getCellsInPolygon(poly);
  //cMarkers_ = drawCells(cells);
  //ROS_INFO("cells.size(): %i", (int)cells.size());
  

  // Initialize reduced_cells to contain all cells
  std::vector<Cell> reduced_cells = cells;


  /*
   * Main algorithm loop
   */
  while(cells.size() > 0)
  {
    //std::cout<<"\nIn while cells.size(): "<<cells.size()<<" result.size(): "<<result.size();
    
    // Set cells to the reduced_cells list from last iteration and create a priority queue
    cells = reduced_cells;
    std::priority_queue<Cell, std::vector<Cell>, CompareDist> updated_pq;


    /*
     *  Delete all cells whose centers lie in the largest circle
     */
    if(result.size() > 0)
    {
      //ROS_INFO("Last cell was size :%f", result[ result.size()-1 ].radius);
      // Exit if we are at minimum radius threshold
      if(result[ result.size()-1 ].radius < min_r)
      {
        break;
      }

      // Otherwise, remove cells that overlap with most recently added circle
      reduced_cells.clear();
      deleteCellsInCir(cells, result[result.size()-1], reduced_cells);
    }
    //ROS_INFO("reduced_cells.size(): %i", (int)reduced_cells.size());

    /*
     *  Recalculate the distance, include existing circles!
     */
    for(int i=0;i<reduced_cells.size();i++)
    {
      Cell& cell = reduced_cells[i];

      //ROS_INFO("Cell %i: (%f,%f)", i, cell.p.x, cell.p.y);

      // Get min distance to polygon edges and set of circles already created
      double min_d=getMinDistToPoly(poly, cell);
      double min_cir=getMinDistToCirs(result, cell);

      //ROS_INFO("min_d: %f min_cir: %f", min_d, min_cir);

      // Set new distance value
      if(min_d < min_cir || min_cir < 0)
      {
        cell.dist = min_d;
      }
      else
      {
        cell.dist = min_cir;
      }

      updated_pq.push(cell);
    } // end for each cell

    /*
     * If cells remain in priority queue
     * Create a new circle from that cell
     */
    if(!updated_pq.empty())
    {
      Cell c = updated_pq.top();

      Circle temp;

      temp.center.x = c.p.x;
      temp.center.y = c.p.y;
      temp.radius = c.dist;

      result.push_back(temp);
    }
  } // end while
  
  //ROS_INFO("result.size(): %i", (int)result.size());
  return result;
}


/*
 * Returns a vector of Circle objects that are packed into each obstacle
 */
std::vector< std::vector<Circle> > CirclePacker::goCirclePacking(double min_r)
{
  //ROS_INFO("In CirclePacker::goCirclePacking()");
  std::vector< std::vector<Circle> > result;

  // Create a matrix of the same size and type as src
  dst.create( src.size(), src.type() );
  
  // Transpose the matrix to get the coordinates right
  cv::Mat srcTrans;
  cv::transpose(src, srcTrans);


  /*
   * Detect contours
   */
  std::vector< std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;

  // ***** findContours modifies input matrix *****
  findContours( srcTrans, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );  


  /*
   * Get convex hull for each contour
   */
  std::vector< std::vector<cv::Point> > hull(contours.size());
  for(int i=0;i<contours.size();i++)
  {
    cv::convexHull(cv::Mat(contours[i]), hull[i], false);
  }


  /*
   * Get list of Polygon objects that represent each convex hull
   */
  std::vector<Polygon> ps = getPolygonsFromContours(hull);
  for(int i=0;i<ps.size();i++)
  {
    // Get circles inside polygon
    result.push_back(packCirsIntoPoly(ps[i], min_r));
  }

  //drawContourPoints(contours, hierarchy);
  //drawContourPoints(hull, hierarchy);

  return result;
}


/*
 * Creates a new Mat object and colors the contours pixels, then calls imshow and waitKey
 */ 
void CirclePacker::drawContourPoints(std::vector< std::vector<cv::Point> > contours, std::vector<cv::Vec4i> hierarchy)
{
  /*
   * Draw contours and hulls
   */
  cv::Mat drawing = cv::Mat::zeros( src.size(), CV_8UC3 );
  for( int j = 0; j< contours.size(); j++ )
  {
    cv::Scalar color = cv::Scalar( 0, 0, 255 );
    cv::Scalar colorHull = cv::Scalar( 0, 255, 0 );
    drawContours( drawing, contours, j, color, 2, 8, hierarchy, 0, cv::Point() );
  }

  /// Show in a window
  imshow( "Contours", drawing );
  cv::waitKey(0);
}

visualization_msgs::Marker CirclePacker::drawLines(const std::vector<Point>& points, const int id)
{
  //ROS_INFO("CirclePacker::drawLines");
  visualization_msgs::Marker result;

  result.id = id;
  result.header.stamp = ros::Time::now();
  result.header.frame_id = "/map";
  result.ns = "basic_shapes";

  result.type = visualization_msgs::Marker::LINE_STRIP;
  result.action = visualization_msgs::Marker::ADD;

  result.color.r = 1;
  result.color.g = 0;
  result.color.b = 1;
  result.color.a = 1;

  // Push on all the trajectory points
  for(int i=0;i<points.size();i++)
  {
    //////ROS_INFO("Point %i: (%f,%f,%f)", i, trajec.msg_.trajectory.points[i].positions[0], trajec.msg_.trajectory.points[i].positions[1], trajec.msg_.trajectory.points[i].positions[2]);
    geometry_msgs::Point p;
    p.x = (points[i].x * 0.05);
    p.y = (points[i].y * 0.05);
    p.z = 0;

    result.points.push_back(p);
  }
  // Planning cycles are usually 20Hz, but put a little padding on there so rviz looks smoother and doesn't start blinking if there are any delays
  result.lifetime = ros::Duration(0.2);

  // Width of the lines
  result.scale.x = 0.01;

  
  //ROS_INFO("drawLines polygonMarker_.size(): %i", (int)result.points.size());

  return result;
}

std::vector<visualization_msgs::Marker> CirclePacker::drawCells(std::vector<Cell> cells)
{
  std::vector<visualization_msgs::Marker> result;

  double offset = 0.5;
  
  for(int i=0;i<cells.size();i++)
  {
    // Make a polygon out of the cell so we can use drawPolygon
    Polygon p; 

    Point nw;
    nw.x = (cells[i].p.x - offset);
    nw.y = (cells[i].p.y + offset);

    Point ne;
    ne.x = (cells[i].p.x + offset);
    ne.y = (cells[i].p.y + offset);

    Point se;
    se.x = (cells[i].p.x + offset);
    se.y = (cells[i].p.y - offset);
    
    Point sw;
    sw.x = (cells[i].p.x - offset);
    sw.y = (cells[i].p.y - offset);

    Edge e;
    e.start = se;
    e.end = ne;

    Edge ee;
    ee.start = ne;
    ee.end = nw;

    Edge eee;
    eee.start = nw;
    eee.end = sw;

    Edge eeee;
    eeee.start = sw;
    eeee.end = se;

    p.edges.push_back(e);
    p.edges.push_back(ee);
    p.edges.push_back(eee); 
    p.edges.push_back(eeee); 

    result.push_back(drawPolygon(p, 60000+i));
  }

  return result;
}

std::vector<visualization_msgs::Marker> CirclePacker::drawPolygons(std::vector<TPPLPoly> polys)
{
  std::vector<visualization_msgs::Marker> result;
  for(int i=0;i<polys.size();i++)
  {
    // Convert to Polygon object
    Polygon p;
    
    for(int j=0;j<polys[i].GetNumPoints();j++)
    {
      Edge e;
      e.start.x = polys[i].GetPoint(j).x;
      e.start.y = polys[i].GetPoint(j).y;
      
      if(j == polys[i].GetNumPoints()-1)
      {
        e.end.x = polys[i].GetPoint(0).x;
        e.end.y = polys[i].GetPoint(0).y;
      }
      else
      {
        e.end.x = polys[i].GetPoint(j+1).x;
        e.end.y = polys[i].GetPoint(j+1).y;
      }

      p.edges.push_back(e);
    }

    result.push_back(drawPolygon(p, 50000+i)); 
  } // end outer for

  return result;
}
 
visualization_msgs::Marker CirclePacker::drawPolygon(const Polygon& poly, const int id)
{
  //ROS_INFO("In CirclePacker::drawPolygon");
  //ROS_INFO("poly.edges.size(): %i", (int)poly.edges.size());
  std::vector<Point> points;
  for(int i=0;i<poly.edges.size();i++)
  {
    points.push_back(poly.edges[i].start); 
    if(i == poly.edges.size()-1)
    {
      points.push_back(poly.edges[i].end); 
    }
  }
  
  return drawLines(points, id);
}



/*
 * Returns a single circle to be placed over a set of contour points
 * Center = average of obstacle pixels in the regions defined by contour points
 * Radius = max dist between any obstacle pixel in region and the computed center
 */ 
Circle CirclePacker::fitCirOverContours(const std::vector<cv::Point> contours)
{
  //ROS_INFO("contours[%i].size(): %i", i, (int)contours[i].size());
  Circle result;
  std::vector<cv::Point2f> obs_points;


  /*
   *  Get all the points within the contour region that are obstacle pixels
   */

  // Get min and max values
  int x_min = contours[0].x, x_max=x_min, y_min = contours[0].y, y_max=y_min;
  for(int j=0;j<contours.size();j++)
  {
    cv::Point2f p = contours[j];

    if(p.x < x_min)
    {
      x_min = p.x;
    }
    if(p.x > x_max)
    {
      x_max = p.x;
    }
    if(p.y < y_min)
    {
      y_min = p.y;
    }
    if(p.y > y_max)
    {
      y_max = p.y;
    }

    //obs_points.push_back(p);
  } // end for each contour point

  /*
   * Go through the region and collect all obstacle pixels
   */
  int num_ob=0, num_free=0;
  for(int x=x_min;x<=x_max;x++)
  {
    for(int y=y_min;y<=y_max;y++)
    {
      int pixel = src.at<uchar>(y, x);
      //ROS_INFO("Point (%i,%i) pixel value: %i", y, x, pixel);

      // If the value is less than some threshold for obstacle pixels
      if(pixel < 100)
      {
        cv::Point2f p;
        p.x = x;
        p.y = y;
        obs_points.push_back(p);
        num_ob++;
      }
      else num_free++;
    } // end inner for
  } // end for each pixel in region

  //ROS_INFO("obs_points.size(): %i", (int)obs_points.size());

  /*
   * Find obstacle center
   */
  float x=0, y=0;
  for(int j=0;j<obs_points.size();j++)
  {
    x+=obs_points[j].x;
    y+=obs_points[j].y; 
  }
  x /= obs_points.size();
  y /= obs_points.size();
  //ROS_INFO("Average center: (%f,%f)", x, y);

  // Set the center value
  // If not operating on hilbert map, flip coordinates
  result.center.x = y;
  result.center.y = x;
  
  /*
   * Get the dist of each pixel to the center
   */
  std::vector<double> dists;
  double max_dist = -1;
  for (size_t pointIdx = 0; pointIdx<obs_points.size(); pointIdx++)
  {
    cv::Point2f pt = obs_points[pointIdx];
    double d = utility_.positionDistance(result.center.x, result.center.y, pt.y, pt.x);
    //ROS_INFO("Point %i, d: %f", pointIdx, d);
    dists.push_back(d);

    if(d > max_dist)
    {
      max_dist = d;
    }
  }
  
  result.radius = max_dist;

  obs_points.clear();
  dists.clear();

  ////////ROS_INFO("c.radius: %f obSizeThreshold: %f", c.radius, obSizeThreshold);

  // Only push on circles that are above a size threshold
  /*if(result.radius > obSizeThreshold)
  {
    result.push_back(c);
  }*/


  return result;
}



/*
 * Returns a vector of Circles where each element is a Circle fit over an obstacle region in the src Mat
 */
std::vector<Circle> CirclePacker::goMyBlobs(bool hmap)
{
  ////ROS_INFO("In CirclePacker::goMyBlobs()");
  std::vector<Circle> result;

  // Create a matrix of the same size and type as src
  dst.create( src.size(), src.type() );
  
  // Make a copy of src because findContours modifies the input matrix
  cv::Mat srcCopy;
  src.copyTo(srcCopy);

  /*
   * Detect blobs
   */
  // Get contours
  std::vector< std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;

  // ***** findContours modifies src! *****
  findContours( srcCopy, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );  
  ////////ROS_INFO("contours.size(): %i", (int)contours.size());
  
  // Go through each set of contour points
  for(int i=0;i<contours.size();i++)
  {
    //ROS_INFO("contours[%i].size(): %i", i, (int)contours[i].size());
    
    // Find the circle that fits over the contour points
    Circle c = fitCirOverContours(contours[i]);
    
    ////////ROS_INFO("c.radius: %f obSizeThreshold: %f", c.radius, obSizeThreshold);

    // Only push on circles that are above a size threshold
    if(c.radius > obSizeThreshold)
    {
      result.push_back(c);
    }
  } // end for each set of contour points

  ////ROS_INFO("Exiting CirclePacker::goMyBlobs()");
  return result;
}


CircleGroup CirclePacker::getGroupForContours(std::vector<cv::Point> contours)
{
  Circle c = fitCirOverContours(contours);


  // Manually transpose set of points
  for(int i=0;i<contours.size();i++)
  {
    int swap = contours[i].x;
    contours[i].x = contours[i].y;
    contours[i].y = swap;
  }
  std::vector<cv::Point> hull;
  cv::convexHull(cv::Mat(contours), hull);
  
  //Polygon p = getPolygonFromContours(hull);
  Polygon p = getPolygonFromContours(contours);

  /*std::vector<TPPLPoly> tris = triPolyPart(p);
  ROS_INFO("tris.size(): %i", (int)tris.size());
  std::vector<Polygon> ps;
  for(int i=0;i<tris.size();i++)
  {
    ps.push_back(tpplToPoly(tris[i]));
  }*/

  std::vector<Polygon> ps;
  ps.push_back(p);
  std::vector<Circle> cs;
  for(int i=0;i<ps.size();i++)
  {
    std::vector<Circle> temp = packCirsIntoPoly(ps[i], 0);
    cs.insert(std::end(cs), std::begin(temp), std::end(temp)); 
  }
  //ROS_INFO("cs.size(): %i", (int)cs.size());

  // Draw polygons
  //pMarkers_ = drawPolygons(tris);

  CircleGroup result;
  result.fitCir = c;
  result.packedCirs = cs;

  return result;
}


/*
 * Returns a vector of CircleGroup objects
 * One for each obstacle region in the src matrix
 */
std::vector<CircleGroup> CirclePacker::getGroups()
{
  std::vector<CircleGroup> result;

  // Create a matrix of the same size and type as src
  dst.create( src.size(), src.type() );
  
  cv::Mat srcCopy, srcTrans;
  src.copyTo(srcCopy);
  cv::transpose(src, srcTrans);

  
  /*
   * Detect blobs
   */
  // Get contours
  std::vector< std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;

  // ***** findContours modifies src! *****
  findContours( srcCopy, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );  
  //ROS_INFO("contours.size(): %i", (int)contours.size());
  
  // Go through each set of contour points
  for(int i=0;i<contours.size();i++)
  {
    // Check size
    if(contours[i].size() < 10)
    {
      continue;
    }

    CircleGroup cg = getGroupForContours(contours[i]);
    result.push_back(cg);
    //ROS_INFO("cg %i, packedCirs.size(): %i fitCir: (%f,%f)", i, (int)cg.packedCirs.size(), cg.fitCir.center.x, cg.fitCir.center.y);
  }

  return result;
}
