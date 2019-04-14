#ifndef CURB_REFINE_RANSAC_H
#define CURB_REFINE_RANSAC_H

#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl/filters/extract_indices.h>
#include<pcl/PointIndices.h>
#include<pcl/ModelCoefficients.h>
#include<eigen3/Eigen/Core>
#include<opencv2/opencv.hpp>
#include<pcl/segmentation/sac_segmentation.h>
#include"cloud_mapper.h"
#include<pcl/filters/statistical_outlier_removal.h>
#include<pcl/filters/project_inliers.h>
#include<pcl/io/pcd_io.h>
#include"ground_segment.h"
#include"grid_map.h"
#include"gaussian_process.h"
#include"road_segmentation.h"
using namespace std;
using namespace Eigen;
using namespace cv;

typedef pcl::PointXYZI PointT;
typedef vector<pcl::PointCloud<PointT>::Ptr > CloudPtrList;

class curb_refine_ransac
{
public:
    curb_refine_ransac(pcl::PointCloud<PointT> &incloud,double varthreshold=5,double fthreshold=1.5);
    void extractPointCloud(pcl::PointCloud<PointT>& incloud, pcl::PointIndicesPtr indices, pcl::PointCloud<PointT>& outcloud);
    void lineFitRansac(pcl::PointCloud<PointT>& incloud, pcl::PointIndices& indices);
    void statisticalFilter_indces(pcl::PointCloud<PointT> incloud, int meanK, pcl::PointIndicesPtr pointindices, double stdThreshold);
    void pointcloud_projection(pcl::PointCloud<PointT>::Ptr incloud,pcl::PointCloud<PointT>& outcloud);
    void ransac_curve(pcl::PointCloud<pcl::PointXYZI>::Ptr incloud,double residualThreshold,
                                          pcl::PointCloud<pcl::PointXYZI>::Ptr outcloud);
    void generateGrid(pcl::PointCloud<PointT>::Ptr incloud,vector<pcl::PointCloud<pcl::PointXYZI> >& outcloud);
   void distanceFilterByGrid(pcl::PointCloud<PointT>::Ptr incloud, pcl::PointCloud<PointT>::Ptr outcloud, bool left);
   void distanceFilterByLaserLeft(pcl::PointCloud<PointT>::Ptr incloud, pcl::PointCloud<PointT>::Ptr outcloud);
   void distanceFilterByLaserRight(pcl::PointCloud<PointT>::Ptr incloud, pcl::PointCloud<PointT>::Ptr outcloud);
    void process(pcl::PointCloud<PointT>::Ptr obstacleCloud, vector<pcl::PointCloud<PointT>::Ptr> clusterCloud,
                 vector<pcl::PointXYZ> &nearestpoints, vector<double> &distance_vec, vector<double> &distance_vec_filtered);
private:
    pcl::PointCloud<PointT> _cloud;
    pcl::PointIndices _indicesLeft;
    pcl::PointIndices _indicesRight;

    double _dbR;//密度直达点的搜索半径
    double _neighborRate;
    int _min_pets;//邻域内最少点个数
    double _varThreshold;
    double _fThreshold;
};

#endif // CURB_REFINE_RANSAC_H
