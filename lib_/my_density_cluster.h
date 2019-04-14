#ifndef MY_DENSITY_CLUSTER_H
#define MY_DENSITY_CLUSTER_H
#include<vector>
#include<iostream>
#include<fstream>
#include<omp.h>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include <pcl/octree/octree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include<boost/make_shared.hpp>
#include<time.h>
#include<stack>


using namespace std;
typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT>::Ptr PointCloud;

class pointPty
{
public:
        int visited;
        int pointtype;//1噪声，2边界点，3核心点
        int cluster;
        int pointIndex;
        vector<int> corepts;//存储邻域内点的索引
        vector<int> NOcorepts; //存储非核心点索引
        pointPty() {}
        pointPty(int index)
        {
            pointIndex=index;
        }
};


class my_density_cluster
{
public:
    my_density_cluster(PointCloud incloud, double neighborRate=0.02, int min_pets=1, double dbr=3);
    void getdc(PointCloud incloud);
    void getLocalDensity(PointCloud incloud);
    void getDistanceToHigherDensityByOrder(PointCloud incloud);
    void findClusterCenters(PointCloud incloud);
    void process(PointCloud incloud, vector<PointCloud> clusters);
    void processbyY(PointCloud incloud, vector<PointCloud> clusters);

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
//    vector<int> _classType;
    double _dc;
    double _dbR;
    int _searchK;
    double _neighborRate;
    PointCloud _CoreCloud;
    vector<pointPty> _corecloudIndex;//构建核心点集
    vector<pointPty> _allcloudIndex;
//    float _eps;//邻域距离
    int _min_pets;//邻域内最少点
    int _clusterNum;

};

#endif // MY_DENSITY_CLUSTER_H
