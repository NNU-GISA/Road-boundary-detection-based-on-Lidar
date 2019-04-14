#ifndef GRID_MAP_H
#define GRID_MAP_H
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl/PointIndices.h>
#include<pcl/segmentation/sac_segmentation.h>
#include<pcl/ModelCoefficients.h>
#include<vector>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<eigen3/Eigen/Eigen>
#include<opencv2/opencv.hpp>

using namespace std;
using namespace Eigen;

typedef pcl::PointCloud<pcl::PointXYZI> PointCloudXYZI;
typedef pcl::PointCloud<pcl::PointXY> PointCloudXY;
typedef pcl::PointXYZI PointT;

class point2D
{
public:

    float r;
    float z;
    point2D(){}
    point2D(pcl::PointXYZI point)
    {
        r=sqrt(pow(point.x,2)+pow(point.y,2));
        z=point.z;
    }
};
class grid_map
{
public:
    grid_map(PointCloudXYZI::Ptr incloud,double mapRadius,int segmetNum,int binNum);
    grid_map(PointCloudXYZI::Ptr incloud);
    void generatePolarGridMap();///[input_grid_length] is only choosed from array [g_grid_length_type]
    void generateCartesianGrid(vector<vector<pcl::PointXYZI> > &grid_map_vec_carte);
    void computeLowestPoints();
    void ransac_curve(vector<point2D> incloud, double residualThreshold, Vector3d &model);
    void ransac_line(PointCloudXYZI::Ptr incloud,double residualThreshold, Vector3d& model);
    void computeSegmentCurve();
    void distanceFilterByPolarGrid(PointCloudXYZI::Ptr outcloud);
    void distanceFilterByCartesianGrid(pcl::PointCloud<PointT>::Ptr outcloud,bool left);
    void getGroundPoints(PointCloudXYZI::Ptr outcloud);

private:
    PointCloudXYZI::Ptr _origin_cloud_ptr;
    vector< vector< vector<pcl::PointXYZI> > > _grid_map_vec;
    vector< vector<pcl::PointXYZI> > _grid_map_vec_carte;
    vector< vector<point2D> > _grid_lowest_points;
    vector<PointCloudXYZI::Ptr> _grid_lowest_cloud;
    vector<Vector3d,aligned_allocator<Vector3d> > _segment_curve_model;
    double _grid_map_radius;
    int _segment_num;
    int _bin_num;
    float _ground_threshold;
    float _diffZ;

};
#endif // GRID_MAP_H
