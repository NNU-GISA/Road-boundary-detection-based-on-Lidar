#ifndef CURB_REFINE_H
#define CURB_REFINE_H


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
using namespace std;
using namespace Eigen;
typedef pcl::PointXYZI PointT;
typedef pair<int,float> MyPairType;


/**
 * @brief The curb_refine class
 */

class curbRefine
{
public:
    curbRefine(pcl::PointCloud<PointT>::Ptr incloud, double tolerance=3.5);

    //    pcl::PointCloud<PointT>::Ptr filterNANS(pcl::PointCloud<PointT>::Ptr cloud);
    //    pcl::PointCloud<PointT>::Ptr  extractInliers(pcl::PointCloud<PointT>::Ptr cloud_filtered,pcl::PointIndices::Ptr inliers,bool flag);
    //    pcl::PointCloud<PointT>::Ptr cylinderSegmentation(pcl::PointCloud<PointT>::Ptr cloud);
    //    pcl::PointCloud<PointT>::Ptr voxelGridFiltering(pcl::PointCloud<PointT>::Ptr cloud);
    //bool customCondition(const PointT& seedPoint, const PointT& candidatePoint, float squaredDistance);
//    bool CompareSecondMin(const MyPairType& left, const MyPairType& right);
//    bool CompareSecondMax(const MyPairType& left, const MyPairType& right);
    int getMin(map<int, float> mymap);
    int getMax(map<int, float> mymap);
    void generateClusters();
    void conditionalEucludieanClustering();
    void LabeledEucludieanClustering();
    void eucludieanClustering ();

    void regionBasedClustering(pcl::PointCloud<PointT>::Ptr cloud,int searchRadius,float smoothThreshold,float curveThreshold);

    int getNumberofClusters();
    void lineFit(pcl::PointCloud<PointT>::Ptr incloud,pcl::PointIndices& indices,pcl::ModelCoefficients& model);
    void process(pcl::PointCloud<PointT>::Ptr clusterCloud);
    void pointcloud_projectionXY(pcl::PointCloud<PointT>::Ptr incloud,pcl::PointCloud<PointT>& outcloud);
     void pointcloud_projectionYZ(pcl::PointCloud<PointT>::Ptr incloud,pcl::PointCloud<PointT>& outcloud);
    static bool customCondition(const PointT& seedPoint, const PointT& candidatePoint,
                         float squaredDistance);
    void conditionalFilter(pcl::PointCloud<PointT> incloud, pcl::PointCloud<PointT> &outcloud);

    void statisticalFilter(pcl::PointCloud<PointT> incloud, pcl::PointCloud<PointT>& outcloud);
    void mycluster(pcl::PointCloud<PointT>& incloud);
private:
    vector<pcl::PointIndices> _clusters;
    vector<pcl::PointIndices> _LinePointsIndicesVector;
    vector<pcl::ModelCoefficients> _LineModelCoefVector;
    vector<pcl::PointCloud<PointT> > _cloudVector;
    vector<pcl::PointCloud<PointT> > _cloudVectorFiltered;
    vector<pcl::PointCloud<PointT> > _cloudVector2D;
    pcl::PointCloud<PointT> _completeCloud;
    pcl::PointCloud<PointT> _completeCloud2DXY;
    pcl::PointCloud<PointT> _completeCloud2DYZ;
    pcl::PointCloud<PointT> _completeCloud2DFiltered;
    map<int,float> _ClusterIntercept;
//    map<int,float> _rightClusterIntercept;
    pcl::PointIndices _pointIndices;
    vector<int> _indices;
    //vector<pcl::PointCloud<pcl::PointXYZI> > _cloudVector3D;
//    float _lineThreshold_1;
//    float _lineThreshold_2;
    double _tolerance;
    int _neighborK;


//    vector<int> _clusterLabel;
};

#endif // CURB_REFINE_H
