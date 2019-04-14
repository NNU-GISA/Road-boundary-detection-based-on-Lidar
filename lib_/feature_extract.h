#ifndef FEATURE_EXTRACT_H
#define FEATURE_EXTRACT_H

#include<rectify.h>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<vector>

using namespace std;

class feature_extract
{
public:
    feature_extract(vector<pcl::PointCloud<pcl::PointXYZI> >& cloudVector,int slopeRegion,
    float normalRegionMax,
    float normalRegionMin,
    float slopeThreshold,
    float normalThreshold,int heightRegion=8);

    float max_z_j(int j,const pcl::PointCloud<pcl::PointXYZI>& scanPoint);

    float min_z_j(int j,const pcl::PointCloud<pcl::PointXYZI>& scanPoint);

    float slope_angle(int i,const pcl::PointCloud<pcl::PointXYZI>& scanPoint);

    float normal_diff(int i,const pcl::PointCloud<pcl::Normal>& normal);

    void compute_normal(int searchRadius, const pcl::PointCloud<pcl::PointXYZI>& scanPoint,
                         pcl::PointCloud<pcl::Normal>& normal);
    vector<int> vectors_intersection(vector<int> v1, vector<int> v2);

    void extractPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr incloud,
                       pcl::PointCloud<pcl::PointXYZI>::Ptr outCloud,
                       boost::shared_ptr<vector<int> > indices,bool setNeg=false);

    void extractFeatures(pcl::PointCloud<pcl::PointXYZI>::Ptr feature_points);

private:

    scanIndices _scanindices;
    vector<pcl::PointCloud<pcl::PointXYZI> > _cloudVector;
    vector<pcl::PointCloud<pcl::Normal> > _normalVectorMax;
    vector<pcl::PointCloud<pcl::Normal> > _normalVectorMin;
//    pcl::PointCloud<pcl::PointXYZI> _incloud;
//    pcl::PointCloud<pcl::Normal> _cloud_normal;
    int  _slopeRegion;
    int _heightRegion;
    float _normalRegionMax;
    float _normalRegionMin;
    float _slopeThreshold;
    float _normalThreshold;
    vector<vector<int> > _slopePointsIndex;
    vector<vector<int> > _heightPointsIndex;
    vector<vector<int> > _normalPointsIndex;
    vector<vector<int> > _Index;

};

#endif // FEATURE_EXTRACT_H
