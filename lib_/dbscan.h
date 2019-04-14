#ifndef DBSCAN_H
#define DBSCAN_H

#include<vector>
#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include<boost/make_shared.hpp>
#include<omp.h>
using namespace std;
class point
{
public:
        float x;
        float y;
        float z;
        float intensity;
        int visited = 0;
        int pointtype = 1;//1噪声，2边界点，3核心点
        int cluster = 0;
        vector<int> corepts;//存储邻域内点的索引
        point() {}
        point(float a, float b, float c,float inten)
        {
                x = a;
                y = b;
                z = c;
                intensity=inten;
        }
};

class dbscan
{
public:
    dbscan(pcl::PointCloud<pcl::PointXYZI>::Ptr incloud, float radius, int pointsNum);
    float distance(point a,point b);
    int getNumofCluster();
    void process();
    void getClusters(vector<pcl::PointCloud<pcl::PointXYZI>::Ptr > clusters);
private:
    pcl::PointCloud<pcl::PointXYZI>::Ptr _completeCloud;
    vector<point> _corecloud;//构建核心点集
    vector<point> _allcloud;
    float _eps;//邻域距离
    int _min_pets;//邻域内最少点
    int _clusterNum;
};

#endif // DBSCAN_H
