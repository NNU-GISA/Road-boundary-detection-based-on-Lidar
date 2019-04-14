#ifndef ROAD_SEGMENTATION_H
#define ROAD_SEGMENTATION_H

#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<vector>

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<pcl::PointXYZI> PointCloudXYZI;
using namespace std;
class road_segmentation
{
public:
    road_segmentation(PointCloudXYZI::Ptr incloud);
    void generatePolarGrid();
    void computeDistanceVec();
    void computeSegmentAngle();
    void process(PointCloudXYZI::Ptr incloud,vector<PointCloudXYZI::Ptr> outcloud);
    vector<pcl::PointXYZ> _nearest_points;
    vector<double> _distance_vec_filtered;
    vector<double> _distance_vec_;
private:
    PointCloudXYZI::Ptr _completeCloud;

    vector< vector<pcl::PointXYZI> > _grid_map_vec;

    vector<pair<double,int> > _distance_vec_front;
    vector<pair<double,int> > _distance_vec;
    //    vector<pair<double,int> > _distance_vec_;
    vector<pair<double,int> > _distance_vec_rear;

    vector<int> _segmentAngle;

};

#endif // ROAD_SEGMENTATION_H
