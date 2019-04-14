#include "feature_extract.h"
#include"math_utils.h"
#include<pcl/features/normal_3d.h>
#include<pcl/filters/extract_indices.h>

#include<boost/make_shared.hpp>
#include<eigen3/Eigen/Dense>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Geometry>
#include<algorithm>
#include<pcl/search/kdtree.h>
#include<pcl/visualization/pcl_visualizer.h>

#define EIGEN_USE_MKL_ALL

using namespace std;
using namespace Eigen;
feature_extract::feature_extract(vector<pcl::PointCloud<pcl::PointXYZI> >& cloudVector, int slopeRegion,
                                 float normalRegionMax,
                                 float normalRegionMin,
                                 float slopeThreshold,
                                 float normalThreshold,int heightRegion)
    ://scanPoint(incloud),
      //_scanindices(scanindices),
      _cloudVector(cloudVector),
      _heightRegion(heightRegion),
      _slopeRegion(slopeRegion),
      _normalRegionMax(normalRegionMax),
      _normalRegionMin(normalRegionMin),
      _slopeThreshold(slopeThreshold),
      _normalThreshold(normalThreshold)
{
    _normalVectorMax.resize(_cloudVector.size());
    _normalVectorMin.resize(_cloudVector.size());
    _slopePointsIndex.resize(_cloudVector.size());
    _heightPointsIndex.resize(_cloudVector.size());
    _normalPointsIndex.resize(_cloudVector.size());
    _Index.resize(_cloudVector.size());
    for(int i=0;i<_cloudVector.size();++i)
    {
        compute_normal(_normalRegionMin,_cloudVector[i],_normalVectorMin[i]);
        compute_normal(_normalRegionMax,_cloudVector[i],_normalVectorMax[i]);
    }
     cout<<"normalThreshold is "<<_normalThreshold<<endl;
     cout<<"normalRegionMin is "<<_normalRegionMin<<endl;
     cout<<"normalRegionMax is "<<_normalRegionMax<<endl;
}


float feature_extract::max_z_j(int j,const pcl::PointCloud<pcl::PointXYZI>& scanPoint)
{
    float max_z =scanPoint[j].z;
    for (int k = j - _heightRegion; k <= j+_heightRegion; ++k)
    {
        if (max_z < scanPoint[k].z)
        {
            max_z =scanPoint[k].z;
        }
    }
    return max_z;
}

float feature_extract::min_z_j(int j,const pcl::PointCloud<pcl::PointXYZI>& scanPoint)
{
    float min_z = scanPoint[j].z;
    for (int k = j -_heightRegion; k <= j +_heightRegion; ++k)
    {
        if (min_z > scanPoint[k].z)
        {
            min_z = scanPoint[k].z;
        }
    }
    return min_z;
}

float feature_extract::slope_angle(int i,const pcl::PointCloud<pcl::PointXYZI>& scanPoint)
{
    float a = scanPoint[i].x;
    float b = scanPoint[i].y;
    float c = scanPoint[i].z;
    float d = scanPoint[i].intensity;
    float angle_1 = atan((scanPoint[i + _slopeRegion].z - scanPoint[i - _slopeRegion].z)/ math::calcPlaneDistance(scanPoint[i+_slopeRegion], scanPoint[i - _slopeRegion]))*180/M_PI;

    float angle_2 = atan((scanPoint[i].z-scanPoint[i-_slopeRegion].z)/math::calcPlaneDistance(scanPoint[i], scanPoint[i - _slopeRegion])) * 180 / M_PI;
    return angle_1 - angle_2;
}

float feature_extract::normal_diff(int i,const pcl::PointCloud<pcl::Normal>& normal)
{

}

void feature_extract::compute_normal(int searchRadius, const pcl::PointCloud<pcl::PointXYZI>& scanPoint,pcl::PointCloud<pcl::Normal>& normal)
{
    if(scanPoint.size()>=500)
    {
        pcl::NormalEstimation<pcl::PointXYZI,pcl::Normal> ne;
        ne.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZI> >(scanPoint));
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
        ne.setSearchMethod(tree);
        //ne.setRadiusSearch(searchRadius);
        ne.setKSearch(searchRadius);
        ne.compute(normal);
    }
}
vector<int> feature_extract::vectors_intersection(std::vector<int> v1, std::vector<int> v2)
{
    vector<int> v;
    //v.resize(v1.size()+v2.size());
    //std::vector<int>::iterator endpos;
    sort(v1.begin(), v1.end());
    sort(v2.begin(), v2.end());
    set_intersection(v1.begin(), v1.end(), v2.begin(), v2.end(), back_inserter(v));//�󽻼�
    //v.resize(endpos - v.begin());
    return v;
}

void feature_extract::extractPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr incloud,
    pcl::PointCloud<pcl::PointXYZI>::Ptr outCloud,
    boost::shared_ptr<vector<int> > indices,
    bool setNeg)
{
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setNegative(setNeg);
    extract.setInputCloud(incloud);
    extract.setIndices(indices);
    extract.filter(*outCloud);
}
void feature_extract::extractFeatures(pcl::PointCloud<pcl::PointXYZI>::Ptr feature_points)
{
    float z_diffMax=0.3;
    float z_diffMin=0.05;
  for(int i=0;i<_cloudVector.size();++i)
  {
      //提取高度差特征点
      if(_cloudVector[i].size()>=500)
      {
          for(int j=_heightRegion;j<_cloudVector[i].size()-_heightRegion;++j)
          {

              float heightDiff=abs(max_z_j(j,_cloudVector[i])-min_z_j(j,_cloudVector[i]));
              if(heightDiff>=z_diffMin&&heightDiff<=z_diffMax)
              {
                  _heightPointsIndex[i].push_back(j);
              }
          }
          //提取坡度差特征点
          for(int j=_slopeRegion;j<_cloudVector[i].size()-_slopeRegion;++j)
          {
              float slopeDiff=slope_angle(j,_cloudVector[i]);
              if(slopeDiff>_slopeThreshold)
              {
                  _slopePointsIndex[i].push_back(j);
              }
          }
          //提取法向量点
          for(int j=0;j<_cloudVector[i].size();++j)
          {
              Vector3f normalMin( _normalVectorMin[i].points[j].normal[0],_normalVectorMin[i].points[j].normal[1], _normalVectorMin[i].points[j].normal[2]);
              Vector3f normalMax(_normalVectorMax[i].points[j].normal[0],_normalVectorMax[i].points[j].normal[1],_normalVectorMax[i].points[j].normal[2]);
              //float norm=normalMax.squaredNorm();
//              normalMin.normalize();
//              normalMax.normalize();
             Vector3f normal_y(0,1,0);

              //float normalDiff=((normalMax-normalMin)/2).squaredNorm();
              float normalDiff=abs(normal_y.dot(normalMin));
              if(normalDiff>=_normalThreshold)
              {
                  _normalPointsIndex[i].push_back(j);
              }
          }
           //求特征点索引交集
    //      boost::shared_ptr<vector<int> > Index_temp(new vector<int>);
          _Index[i]=vectors_intersection(_heightPointsIndex[i],_slopePointsIndex[i]);
    //      _Index[i]=vectors_intersection(*Index_temp,_normalPointsIndex[i]);
          //cout<<"scan "<< i <<" normal points is "<<_normalPointsIndex[i].size()<<endl;
          //_Index[i]=_slopePointsIndex[i];
          pcl::PointCloud<pcl::PointXYZI>::Ptr points_temp(new pcl::PointCloud<pcl::PointXYZI>);
          extractPoints(boost::make_shared<pcl::PointCloud<pcl::PointXYZI> >(_cloudVector[i]),points_temp,boost::make_shared<vector<int> >(_Index[i]));
          (*feature_points)+=(*points_temp);
      }
  }

}
