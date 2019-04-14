#ifndef MAX_DENSITY_CLUSTER_H
#define MAX_DENSITY_CLUSTER_H


#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include <pcl/octree/octree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include<boost/make_shared.hpp>
#include<time.h>

using namespace std;

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT>::Ptr PointCloud;

class max_density_cluster
{
public:
    max_density_cluster(PointCloud incloud,double neighborRateLow);
    void getdc(PointCloud incloud);
    void getLocalDensity(PointCloud incloud);
    void getDistanceToHigherDensity(PointCloud incloud);
    void getDistanceToHigherDensityByOrder(PointCloud incloud);
    void findClusterCentersTwo(PointCloud incloud);
    void findClusterCentersThree(PointCloud incloud);
    void classifyFeatures2Centers();
    void findClusterCenters();
    void process(PointCloud incloud,vector<PointCloud> clusters);

    /* 主要用来标记是否使用了距离矩阵来存储任意两个点之间的距离 */
    bool isUseDistMatrix;
    /* 该参数主要用来存储每个扫描点周边有多少个点是在DC距离内 */
    vector<double> _densityLocal;
    /* 该参数主要用来存储每个扫描点周边的密度信息和该点在features中的编号信息 <密度(个数),编号> */
    vector<pair<int, int>> _densityLocal_pair;
    /* 该参数主要是用来存储每个点到更高一级的密度点中的最短的距离 */
    vector<double> _minDist2Higher;
    /* 该参数主要是用来存储每个点到更高密度点中的最近距离的index */
    vector<int> _nearestNeighbor;
    /* 该参数主要是用来存储该算法找到的聚类中心的位置 */
    vector<int> _centers;
    /* 该参数主要是用来存储算法对每个特征点的分类结果 */
    vector<int> _classType;
    double _dc;
   double _neighborRateLow;
   double _neighborRateHigh;
};

#endif // MAX_DENSITY_CLUSTER_H
