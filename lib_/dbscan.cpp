#include "dbscan.h"



dbscan::dbscan(pcl::PointCloud<pcl::PointXYZI>::Ptr incloud,float radius,int pointsNum)
    :_eps(radius),_min_pets(pointsNum)
{
    _completeCloud=incloud;
}
float dbscan::distance(point a, point b)
{
    return sqrt((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y) + (a.z - b.z)*(a.z - b.z));
}

void dbscan::process()
{

    //    float resolution=0.5;//最低一级octree的最小体素的尺寸
    double reso=0.05;
    reso=0.05;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> octree(reso);//初始化octree
    octree.setInputCloud(_completeCloud);
    octree.addPointsFromInputCloud();

    size_t len = _completeCloud->points.size();
    //    vector<point> pointlist(8);
    point  pointtemp;
    for (size_t i = 0; i < len; i++)
    {
        pointtemp= point(_completeCloud->points[i].x,_completeCloud->points[i].y,
                         _completeCloud->points[i].z,_completeCloud->points[i].intensity);
        _allcloud.push_back(pointtemp);
    }
    //将核心点放在corecloud中,改变allcloud中的pointtype的值

    for (size_t i = 0; i < len; i++)
    {
        vector<int> radiussearch;//存放点的索引
        vector<float> radiusdistance;//存放点的距离平方
        octree.radiusSearch(_completeCloud->points[i], _eps, radiussearch, radiusdistance);//八叉树的邻域搜索
        if (radiussearch.size() > _min_pets)
        {
            _allcloud[i].pointtype = 3;
            _corecloud.push_back(_allcloud[i]);
        }
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr corecloud1(new pcl::PointCloud<pcl::PointXYZI>);
    corecloud1->points.resize(_corecloud.size());

    for (int i = 0; i < _corecloud.size(); i++)
    {
        corecloud1->points[i].x = _corecloud[i].x;
        corecloud1->points[i].y = _corecloud[i].y;
        corecloud1->points[i].z = _corecloud[i].z;
        corecloud1->points[i].intensity = _corecloud[i].intensity;
    }
    //    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    //    kdtree.setInputCloud(corecloud1);
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> octree1(reso);//初始化octree
    octree1.setInputCloud(corecloud1);
    octree1.addPointsFromInputCloud();

    for (int i = 0; i<_corecloud.size(); i++)
    {
        vector<int> pointIdxNKNSearch;//存放点的索引
        vector<float> pointRadiusSquaredDistance;//存放点的距离平方
        octree1.radiusSearch(corecloud1->points[i], _eps, pointIdxNKNSearch, pointRadiusSquaredDistance);//八叉树的邻域搜索
        for (int j = 0; j < pointIdxNKNSearch.size(); j++)
        {
            _corecloud[i].corepts.push_back(pointIdxNKNSearch[j]);
        }
    }
    //将所有核心点根据是否密度可达归类，改变核心点cluster的值
    _clusterNum = 0;
    for (int i = 0; i<_corecloud.size(); i++)
    {
        stack<point*> ps;
        if (_corecloud[i].visited == 1) continue;
        _clusterNum++;
        _corecloud[i].cluster = _clusterNum;
        ps.push(&_corecloud[i]);
        point *v=&_corecloud[i];
        //将密度可达的核心点归为一类
        while (!ps.empty())
        {
            v = ps.top();
            v->visited = 1;
            ps.pop();
            for (int j = 0; j<v->corepts.size(); j++)
            {
                if (_corecloud[v->corepts[j]].visited == 1) continue;
                _corecloud[v->corepts[j]].cluster = _corecloud[i].cluster;
                _corecloud[v->corepts[j]].visited = 1;
                ps.push(&_corecloud[v->corepts[j]]);
            }
        }
    }
    //找出所有的边界点，噪声点，对边界点分类，更改其cluster

    for (int i = 0; i<len; i++)
    {
        //        if (_allcloud[i].pointtype == 3) continue;
        for (int j = 0; j<_corecloud.size(); j++)
        {
            if (distance(_allcloud[i], _corecloud[j])<_eps)
            {
                _allcloud[i].pointtype = 2;
                _allcloud[i].cluster = _corecloud[j].cluster;
                break;
            }
        }
    }
    //#pragma omp parallel for
    for (int i = 0; i < len; i++)
    {
        if (_allcloud[i].pointtype == 1)
            _allcloud[i].cluster = 0;
    }
}
int dbscan::getNumofCluster()
{
    return _clusterNum+1;
}
void dbscan::getClusters(vector<pcl::PointCloud<pcl::PointXYZI>::Ptr > clusters)
{

    for (int i = 0;  i<_allcloud.size(); ++i)
    {
        pcl::PointXYZI pointTemp;
        pointTemp.x=_allcloud[i].x;
        pointTemp.y=_allcloud[i].y;
        pointTemp.z=_allcloud[i].z;
        pointTemp.intensity=_allcloud[i].intensity;
        clusters[_allcloud[i].cluster]->points.push_back(pointTemp);
    }
}


