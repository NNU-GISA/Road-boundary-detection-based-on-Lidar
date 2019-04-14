#ifndef CURB_REFINE_NEW_H
#define CURB_REFINE_NEW_H

#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/segmentation/extract_clusters.h>
#include <string>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/segmentation/min_cut_segmentation.h>
#include<boost/make_shared.hpp>
#include<map>
#include"curb_refine.h"
using namespace std;
using namespace Eigen;
typedef pcl::PointXYZI PointT;
typedef pair<int,float> MyPairType;

class curb_refine_new
{
public:
    curb_refine_new(pcl::PointCloud<PointT>::Ptr incloud, double tolerance=1);

    void process(vector<pcl::PointCloud<pcl::PointXYZI>::Ptr > clusterCloud);
    void pointcloud_projection(pcl::PointCloud<PointT>::Ptr incloud,pcl::PointCloud<PointT>& outcloud);
    void statisticalFilter(pcl::PointCloud<PointT> incloud, pcl::PointCloud<PointT>& outcloud);
    void statisticalFilter_indces(pcl::PointCloud<PointT> incloud,int meanK, vector<int> *indices, double stdThreshold);
    void extractPointCloud(pcl::PointCloud<PointT>& incloud, boost::shared_ptr<vector<int> > indices,pcl::PointCloud<PointT>& outcloud);
private:

    pcl::PointCloud<PointT> _completeCloud;
//    pcl::PointCloud<PointT> _completeCloud2D;
    pcl::PointCloud<PointT> _completeCloudFiltered;
    vector<int> _indices;
    double _tolerance;
};

#endif // CURB_REFINE_NEW_H
