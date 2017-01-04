#ifndef SR_NODE_COSTMAT_H
#define SR_NODE_COSTMAT_H

// ROS includes.
#include <ros/ros.h>
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <costmap_2d/costmap_2d.h>
#include <vector>
#include <utility>
#include <queue>
#include <fstream>


#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include <sensor_msgs/image_encodings.h>
#include <opencv2/features2d/features2d.hpp>



using namespace std;
using namespace cv;

class Costmat{
  public:
    // Constructor
    Costmat();

    // Destructor
    ~Costmat();

    // Callback function for map subscriber
    Point getCellIndex(int l);
    int getArrayIndex(Point p);
    void updateCells(vector<int8_t> &occupancy_grid_array);
    void updateRewards();
    void displayCells();
    Point getCellIndex(int l);//TODO check these
    int getArrayIndex( Point p);

    float aStarDist(Point sLoc, Point gLoc);
    vector<Point> aStarPath(Point sLoc, Point gLoc);

    float getEuclidianDistance(Point a, Point b);
    Mat euclidianDistance;

    Mat displayPlot;
    void buildCellsPlot();
    void showCostmapPlot(int index); // show nice display plot and number it
    void addAgentToPlot(Scalar color, vector<Point> myPath, Point cLoc);
    Vec3b cObsFree, cInfFree, cObsWall, cInfWall, cUnknown, cError;

    int map_width;
    int map_height;

    bool init_flag;
    Mat cells, rewards;

    int ros_unknown, ros_wall, ros_free;

    int obsFree, infFree, domFree;
    int unknown;
    int obsWall, infWall, inflatedWall;
};
/*
/// Begin Yawei's old stuff



    // Get centroids of frontiers
    void getCentroids(costmap_2d::Costmap2D *costmap_);

    // Frontier extraction
    void frontierExtract(costmap_2d::Costmap2D *costmap_);

    // Classify the frontier
    bool isFrontier(int point);

    // get frontiers region
    void getRegion(int cell, std::vector<int> & region);

    // Adjacents fuction
    int getAdjacents(int point, int direction);

    // Check if two cells are nbors
    bool isAdjacent(int cell1, int cell2);

    // Check the size of unknown area near the boundary point
    bool isSufficient(int point);

    // Frontier publisher
    void pubFrontierMap(const nav_msgs::OccupancyGrid& costmap);

    // Centroid map publisher
    void pubFrontierCentroid(const nav_msgs::OccupancyGrid & costmap);

    // Centoids index publiser
    void pubCentroid(const nav_msgs::OccupancyGrid & costmap);



    int minRegion;

    nav_msgs::OccupancyGrid frontierMap;
    nav_msgs::OccupancyGrid frontierCentroid;
    std_msgs::Float32MultiArray centroid_msg;
    ros::Publisher pub_frontierMap;
    ros::Publisher pub_frontierCentroidMap;
    ros::Publisher pub_centroid;
    std::vector<int8_t> occupancy_grid_array;
    std::vector<int> frontiers;
    std::vector<int> centroid_cells;
    std::vector<float> centroid_coord;
    std::vector<std::pair<uint, uint> > centroids;
};

*/
 
#endif 
// SR_NODE_COSTMAT_H
