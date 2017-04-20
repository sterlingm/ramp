#include "circle_packer.h"
//#include "Utilties.cpp"



CirclePacker::CirclePacker(nav_msgs::OccupancyGridConstPtr g)
{
  //ROS_INFO("In CirclePacker::CirclePacker()");
  grid_ = *g;
  convertOGtoMat(g);
}

CirclePacker::~CirclePacker() 
{
  src.release();
  detected_edges.release();
  dst.release();
}










void CirclePacker::convertOGtoMat(nav_msgs::OccupancyGridConstPtr g)
{
  //ROS_INFO("In CirclePacker::convertOGtoMat");

  
  // Use the GridMap2D library to convert from nav_msgs::OccupancyGrid to cv::Mat
  gridmap_2d::GridMap2D gmap(g, false);

  // Set src
  src = gmap.binaryMap();
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
  //imshow( window_name, dst );
}

double CirclePacker::getMinDistToPoly(const Polygon& poly, const Cell& cell)
{
  double result = 100000;

  for(int n=0;n<poly.normals.size();n++)
  {
    //std::cout<<"\npoly.normals[n]: "<<poly.normals[n].a<<", "<<poly.normals[n].b;
    // Get unit normal
    double l = sqrt( pow(poly.normals[n].a,2) + pow(poly.normals[n].b,2) );
    Normal v_hat;
    v_hat.a = poly.normals[n].a / l;
    v_hat.b = poly.normals[n].b / l;
    
    std::vector<double> r;
    r.push_back(poly.edges[n].start.x - cell.p.x);
    r.push_back(poly.edges[n].start.y - cell.p.y);

    double d = fabs((v_hat.a*r[0]) + (v_hat.b*r[1]));
    //std::cout<<"\nl: "<<l<<" v_hat: <"<<v_hat.a<<","<<v_hat.b<<"> r: <"<<r[0]<<","<<r[1]<<">";
    //std::cout<<"\nd: "<<d;
    if(d < result)
    {
      result = d;
    }
  }

  return result;
}

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


void CirclePacker::deleteCellsInCir(const std::vector<Cell>& cells, const Circle cir, std::vector<Cell>& result)
{
  //std::cout<<"\nIn deleteCellsInCir\n";
  for(int i=0;i<cells.size();i++)
  {
    //std::cout<<"\nTesting cell "<<cells[i].p.x<<", "<<cells[i].p.y;
    //std::cout<<"\nDist: "<<(sqrt( pow( cir.center.x - cells[i].p.x, 2) + pow( cir.center.y - cells[i].p.y, 2) ));
    // Get distance between circle of cell and new cell, check if > its radius
    if( sqrt( pow( cir.center.x - cells[i].p.x, 2) + pow( cir.center.y - cells[i].p.y, 2) ) > cir.radius )
    {
      result.push_back(cells[i]);
    }
  }
  //std::cout<<"\nExiting deleteCellsInCir\n";
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


bool CirclePacker::cellInPoly(Polygon poly, cv::Point cell)
{
  for(int i=0;i<poly.normals.size();i++)
  {
    //std::cout<<"\nnormal a: "<<poly.normals[i].a<<" b: "<<poly.normals[i].b<<" c: "<<poly.normals[i].c;
    double d = poly.normals[i].a*cell.x + poly.normals[i].b*cell.y + poly.normals[i].c;
    //std::cout<<"\ncell center: "<<cell.x<<", "<<cell.y<<" d: "<<d;
    if(d > -0.000001)
    {
      //std::cout<<"\nNot in polygon";
      return false;
    }
  }

  return true;
}


std::vector<Circle> CirclePacker::getCirclesFromPoly(Polygon poly)
{
  //std::cout<<"\n# of edges: "<<poly.edges.size();
  std::vector<Circle> result;
  std::vector<cv::Point> vertices;
  
  // Push on all vertices
  for(int i=0;i<poly.edges.size();i++)
  {
    vertices.push_back(poly.edges[i].start);
    vertices.push_back(poly.edges[i].end);
  }
  
  // Find minimum and maximum x and y
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

  double round = 1;

  // Find number of cells in both directions
  int width_count = (MAX_WIDTH - MIN_WIDTH) / round;
  int length_count = (MAX_LENGTH - MIN_LENGTH) / round;

  // Start from the center
  double start_x = MIN_WIDTH + round/2.f;
  double start_y = MIN_LENGTH + round/2.f;

  //std::cout<<"\nMAX_WIDTH: "<<MAX_WIDTH<<" MAX_LENGTH: "<<MAX_LENGTH<<" width_count: "<<width_count<<" length_count: "<<length_count;

  std::vector<Cell> cells;
 
  for(int i=0;i<width_count;i++)
  {
    for(int j=0;j<length_count;j++)
    {
      //std::cout<<"\ni: "<<i<<" j: "<<j<<" round: "<<round;
      double x = start_x + (round * (i)); 
      double y = start_y + (round * (j));
      Cell temp;
      temp.p.x = x;
      temp.p.y = y;
    
      //std::cout<<"\n("<<temp.p.x<<", "<<temp.p.y<<")";

      if(cellInPoly(poly, temp.p))
      {
        cells.push_back(temp);
      }
    }
  }
  

  std::vector<Cell> reduced_cells = cells;


  while(cells.size() > 0)
  {
    //std::cout<<"\nIn while cells.size(): "<<cells.size()<<" result.size(): "<<result.size();
    cells = reduced_cells;

    std::priority_queue<Cell, std::vector<Cell>, CompareDist> updated_pq;

    // Delete all cells whose centers lie in the largest circle
    if(result.size() > 0)
    {
      reduced_cells.clear();
      deleteCellsInCir(cells, result[result.size()-1], reduced_cells);
    }

    // Recalculate the distance, include existing circles!
    // For each cell, compute distance to the closest polygon edge
    for(int i=0;i<reduced_cells.size();i++)
    {
      Cell& cell = reduced_cells[i];

      double min_d=getMinDistToPoly(poly, cell);
      double min_cir=getMinDistToCirs(result,cell);

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

    if(!updated_pq.empty())
    {
      Cell c = updated_pq.top();
      Circle temp;

      temp.center.x = c.p.x;
      temp.center.y = c.p.y;
      temp.radius = c.dist;

      result.push_back(temp);
    }
  }
  
  /*std::cout<<"\nFinal number of circles: "<<result.size();
  for(int i=0;i<result.size();i++)
  {
    std::cout<<"\nCircle "<<i<<" ("<<result[i].center.x<<", "<<result[i].center.y<<") radius: "<<result[i].radius;
  }*/

  return result;
}


std::vector<Triangle> CirclePacker::triangulatePolygon(const Polygon& poly)
{
  std::vector<Triangle> result;

  // Get all vertices of polygon
  std::vector<cv::Point> vertices;
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
    cv::Point v0 = i == 0 ? vertices[vertices.size()-2] : vertices[i-1];
    cv::Point v1 = vertices[i];
    cv::Point v2 = i == vertices.size() - 1 ? vertices[0] : vertices[i+1];

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

  ROS_INFO("vec: [%f, %f]", vec[0], vec[1]);

  midpoint.push_back( a.center.x + (0.5*vec[0]) ); 
  midpoint.push_back( a.center.y + (0.5*vec[1]) ); 

  ROS_INFO("midpoint: (%f, %f)", midpoint[0], midpoint[1]);

  // Get the distance from the midpoint to each center
  double d_mid_i = utility_.positionDistance(midpoint[0], midpoint[1], a.center.x, a.center.y);
  double d_mid_ii = utility_.positionDistance(midpoint[0], midpoint[1], b.center.x, b.center.y);

  double R = d_mid_i > d_mid_ii ? d_mid_i+a.radius : d_mid_ii+b.radius;

  ROS_INFO("d_mid_i: %f d_mid_ii: %f R: %f", d_mid_i, d_mid_ii, R);

  result.center.x = midpoint[0];
  result.center.y = midpoint[1];
  result.radius = R;
}


// result is a final list of circles: contains both the combined ones and the ones that were not combined
void CirclePacker::combineOverlappingCircles(std::vector<Circle> cs, std::vector<Circle>& result) const
{
  ROS_INFO("In combineOverlappingCircles");
  for(int i=0;i<cs.size();i++)
  {
    ROS_INFO("Circle %i - Center: (%f, %f) Radius: %f", i, cs[i].center.x, cs[i].center.y, cs[i].radius);
  }
  int pairs=0;

  //result = cs;

  int i=0,j=0;

  while(i<cs.size()-1)
  {
    ROS_INFO("i: %i", i);
    ROS_INFO("cs.size(): %i", (int)cs.size());
    Circle ci = cs[i];
    ROS_INFO("ci - Center: (%f, %f) Radius: %f", ci.center.x, ci.center.y, ci.radius);
    j = i+1;
    double inflate = 20.0;
    double threshold = 0.;

    while(j<cs.size())
    {
      ROS_INFO("j: %i", j);

      // Check if they overlap
      Circle cj = cs[j];
      ROS_INFO("cj - Center: (%f, %f) Radius: %f", cj.center.x, cj.center.y, cj.radius);
     
      double max_r = ci.radius > cj.radius ? ci.radius : cj.radius;
      threshold = max_r + inflate;

      double d = utility_.positionDistance(ci.center.x, ci.center.y, cj.center.x, cj.center.y);
      ROS_INFO("d: %f threshold: %f", d, threshold);

      if(d < threshold)
      {
        ROS_INFO("Combining the Circles!");

        // Combine them
        Circle temp;
        combineTwoCircles(ci, cj, temp);
        ROS_INFO("Result - Center: (%f, %f) Radius: %f", temp.center.x, temp.center.y, temp.radius);

        // If combined, replace both circles with overlapping circle
        // Replace i by setting it to temp, erase the circle at j
        result.push_back(temp);
        cs[i] = temp;
        ci = temp;
        cs.erase(cs.begin()+j, cs.begin()+j+1);
        //result.erase(cs.begin()+j, cs.begin()+j+1);

        // Then, decrement j to get next circle for comparison
        j--;
      }

      j++;
    } // end inner while

    i++;
  } // end outter while

  result = cs;
  ROS_INFO("After Overlapping:");
  /*for(int i=0;i<cs.size();i++)
  {
    ROS_INFO("Circle %i - Center: (%f, %f) Radius: %f", i, cs[i].center.x, cs[i].center.y, cs[i].radius);
    result.push_back(cs[i]);
  }*/
} // End combineOverlappingCircles




double CirclePacker::getMedian(const std::vector<double> points) const
{
  // Sort points
  

  // Take the middle element

}


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
  //ROS_INFO("In CirclePacker::getCirclesFromEdgeSets");

 
  std::vector<Point> means;

  // For each edge set (i.e. polygon)
  for(int i=0;i<edge_sets.size();i++)
  {
    ROS_INFO("Edge set %i", i);
      
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

      ROS_INFO("\tEdge %i - start: (%i,%i) end: (%i,%i)", j, edge_sets[i][j].start.y, edge_sets[i][j].start.x, edge_sets[i][j].end.y, edge_sets[i][j].end.x);

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
    

    //ROS_INFO("\tx_min: %i x_max: %i y_min: %i y_max: %i", x_min, x_max, y_min, y_max);

    // Get difference between min+max for both x and y
    double x_diff = fabs(x_max - x_min);
    double y_diff = fabs(y_max - y_min);

    //ROS_INFO("\tx_diff: %f y_diff: %f", x_diff, y_diff);

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
    
    ROS_INFO("\tCenter: (%f,%f) Radius: %f", temp.center.x, temp.center.y, temp.radius);

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

    //ROS_INFO("Edge endpoints: (%i,%i) (%i,%i)", edges[i].start.x, edges[i].start.y, edges[i].end.x, edges[i].end.y);

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

    //ROS_INFO("Edge midpoint: (%f, %f) theta: %f phi: %f psi: %f Circle center: (%f, %f)", x_mid, y_mid, theta, phi, psi, x_cen, y_cen);

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
  ROS_INFO("In CirclePacker::go()");
  std::vector<Circle> result;

  // Create a matrix of the same size and type as src
  dst.create( src.size(), src.type() );
  ROS_INFO("Done with dst.create");

  // Convert to grayscale
  //cvtColor(src, src_gray, CV_BGR2GRAY);

  // Get the edges
  ros::Time t_start_edge_detect = ros::Time::now();
  CannyThreshold(0, 0);
  ros::Duration d_edges_detect(ros::Time::now()-t_start_edge_detect);
  ROS_INFO("Done with CannyThreshold");

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
  cv::Ptr<cv::SimpleBlobDetector> blobs_detector = cv::SimpleBlobDetector::create(params);   

  // Detect blobs
  std::vector<cv::KeyPoint> keypoints;
  ros::Time t_start = ros::Time::now();
  blobs_detector->detect(src, keypoints);
  ros::Duration d_blobs = ros::Time::now() - t_start;
  ROS_INFO("d_blobs: %f", d_blobs.toSec());

  ROS_INFO("Keypoints size: %i", (int)keypoints.size());

  for(int i=0;i<keypoints.size();i++)
  {
    ROS_INFO("Keypoint %i: pt: (%f, %f) class_id: %i angle: %f size: %f", i, keypoints[i].pt.x, keypoints[i].pt.y, keypoints[i].class_id, keypoints[i].angle, keypoints[i].size);
    result.push_back(getCircleFromKeypoint(keypoints[i]));
  }
  return result;
}


std::vector<Circle> CirclePacker::goCorners()
{
  ROS_INFO("In CirclePacker::go()");
  std::vector<Circle> result;

  // Create a matrix of the same size and type as src
  dst.create( src.size(), src.type() );
  ROS_INFO("Done with dst.create");

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
  ROS_INFO("d_corner_detect: %f", d_corner_detect.toSec());

  ROS_INFO("dst.rows: %i dst.cols: %i", dst.rows, dst.cols);
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
  ROS_INFO("In CirclePacker::go()");
  std::vector<cv::RotatedRect> result;

  // Create a matrix of the same size and type as src
  dst.create( src.size(), src.type() );
  ROS_INFO("Done with dst.create");

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
    ROS_INFO("Ellipse %i: (%f, %f)", i, minEllipse[i].center.x, minEllipse[i].center.y);
  }

  return minEllipse;
}


std::vector<Circle> CirclePacker::goHough()
{
  ROS_INFO("In CirclePacker::go()");
  std::vector<Circle> result;

  // Create a matrix of the same size and type as src
  dst.create( src.size(), src.type() );
  ROS_INFO("Done with dst.create");

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


std::vector<Circle> CirclePacker::goMyBlobs()
{
  ROS_INFO("In CirclePacker::go()");
  std::vector<Circle> result;

  // Create a matrix of the same size and type as src
  dst.create( src.size(), src.type() );
  ROS_INFO("Done with dst.create");

  // Convert to grayscale
  //cvtColor(src, src_gray, CV_BGR2GRAY);

  // Get the edges
  ros::Time t_start_edge_detect = ros::Time::now();
  CannyThreshold(0, 0);
  ros::Duration d_edges_detect(ros::Time::now()-t_start_edge_detect);
  ROS_INFO("Done with CannyThreshold");

  /*
   * Detect blobs
   */
  // Get contours
  std::vector< std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;
  findContours( src, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );  

  // Go through each set of contour points
  for(int i=0;i<contours.size();i++)
  {
    Circle c;
    std::vector<cv::Point2f> obs_points;

    // Check that there are at least a min number of contour points
    // This is because we usually get massive circles (radius>1000) when there
    // are only a few points
    if(contours[i].size() < 10)
    {
      continue;
    }

    /*
     *  Get all the points within the contour region that are obstacle pixels
     */

    // Get min and max values
    ROS_INFO("contours[%i].size(): %i", i, (int)contours[i].size());
    int x_min = contours[i][0].x, x_max=x_min, y_min = contours[i][0].y, y_max=y_min;
    for(int j=0;j<contours[i].size();j++)
    {
      cv::Point2f p = contours[i][j];

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
    }

    ROS_INFO("x_min: %i x_max: %i y_min: %i y_max: %i", x_min, x_max, y_min, y_max);
    int num_ob=0, num_free=0;
    for(int x=x_min;x<x_max;x++)
    {
      for(int y=y_min;y<y_max;y++)
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
      }
    }

    ROS_INFO("num_ob: %i num_free: %i", num_ob, num_free);
    ROS_INFO("obs_points.size(): %i", (int)obs_points.size());
    cv::Mat m(obs_points);
    // After getting points, build a moments array
    //cv::Moments moms = moments(obs_points);
    //cv::Moments moms = moments(m);
    //ROS_INFO("moms: m00: %f m10: %f m01: %f m10/m00: %f m01/m00: %f", moms.m00, moms.m01, moms.m10, moms.m10/moms.m00, moms.m01/moms.m00);

    //c.center.x = moms.m01 / moms.m00;
    //c.center.y = moms.m10 / moms.m00;

    int x=0, y=0;
    for(int j=0;j<obs_points.size();j++)
    {
      x+=obs_points[j].x;
      y+=obs_points[j].y; 
    }
    x /= obs_points.size();
    y /= obs_points.size();
    ROS_INFO("Average center: (%i,%i)", x, y);

    c.center.x = y;
    c.center.y = x;

    
    
    
    std::vector<double> dists;
    for (size_t pointIdx = 0; pointIdx<obs_points.size(); pointIdx++)
    {
        cv::Point2f pt = obs_points[pointIdx];
        double d = utility_.positionDistance(c.center.x, c.center.y, pt.y, pt.x);
        //ROS_INFO("d: %f", d);
        dists.push_back(d);
    }
    std::sort(dists.begin(), dists.end());
    //c.radius = (dists[(dists.size() - 1) / 2] + dists[dists.size() / 2]) / 2.;
    c.radius = dists[dists.size()-1];


    obs_points.clear();
    dists.clear();

    // Only push on circles that are above a size threshold
    if(c.radius > 20.0)
    {
      result.push_back(c);
    }
  }




  return result;
}
