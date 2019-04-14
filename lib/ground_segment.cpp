#include "ground_segment.h"


ground_segment::ground_segment(pcl::PointCloud<pcl::PointXYZI>::Ptr incloud)
{
  _cloudptrlist.resize(6);
  #pragma omp parallel for  schedule(runtime)
  for (int i = 0; i < _cloudptrlist.size(); ++i)
  {
      _cloudptrlist[i].reset(new pcl::PointCloud<pcl::PointXYZI>);
  }
  //分段存储

  for (int i = 0; i < incloud->points.size(); ++i)
  {
      if(incloud->points[i].x<=15&&incloud->points[i].x>=0&&abs(incloud->points[i].y)<30)
      {
          _cloudptrlist[0]->points.push_back(incloud->points[i]);
      }
      if(incloud->points[i].x>=-15&&incloud->points[i].x<0&&abs(incloud->points[i].y)<30)
      {
          _cloudptrlist[1]->points.push_back(incloud->points[i]);
      }
      if(incloud->points[i].x>15&&incloud->points[i].x<=30&&abs(incloud->points[i].y)<30)
      {
          _cloudptrlist[2]->points.push_back(incloud->points[i]);
      }
      if(incloud->points[i].x>=-30&&incloud->points[i].x<-15&&abs(incloud->points[i].y)<30)
      {
          _cloudptrlist[3]->points.push_back(incloud->points[i]);
      }
      if(incloud->points[i].x<=40&&incloud->points[i].x>=30&&abs(incloud->points[i].y)<30)
      {
          _cloudptrlist[4]->points.push_back(incloud->points[i]);
      }
      if(incloud->points[i].x<=-30&&incloud->points[i].x>=-40&&abs(incloud->points[i].y)<30)
      {
          _cloudptrlist[5]->points.push_back(incloud->points[i]);
      }
  }
}

void ground_segment::extractGround(pcl::PointCloud<pcl::PointXYZI>::Ptr outCloud,
                                   pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud,
                                   pcl::PointIndices::Ptr Indices,
                                   bool setNeg)
{
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setNegative(setNeg);
    extract.setInputCloud(inputCloud);
    extract.setIndices(Indices);
    extract.filter(*outCloud);
}
void ground_segment::planeSeg(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                              pcl::ModelCoefficients::Ptr coefficients,
                              pcl::PointIndices::Ptr planeIndices,float groundThreshold)
{
    pcl::SACSegmentation<pcl::PointXYZI> segmentation;
    segmentation.setInputCloud(cloud);
    segmentation.setModelType(pcl::SACMODEL_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setDistanceThreshold(groundThreshold);
    segmentation.setMaxIterations(1000);
    segmentation.setOptimizeCoefficients(true);
    segmentation.segment(*planeIndices, *coefficients);
}

void ground_segment::groundfilter(pcl::PointCloud<pcl::PointXYZI>::Ptr groundpoints,pcl::PointCloud<pcl::PointXYZI>::Ptr groundnopoints,double threshold)
{

    for (int i = 0; i < _cloudptrlist.size()-2; ++i)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr ground_i(new  pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr ground_no_i(new  pcl::PointCloud<pcl::PointXYZI>);
        pcl::ModelCoefficients::Ptr modelCoe1(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr indices1(new pcl::PointIndices);
        planeSeg(_cloudptrlist[i],modelCoe1,indices1,threshold);
        extractGround(ground_i,_cloudptrlist[i],indices1,false);
        extractGround(ground_no_i,_cloudptrlist[i],indices1,true);
        *groundpoints+=*ground_i;
        *groundnopoints+=*ground_no_i;
    }
}
//void ground_segment::process(pcl::PointCloud<pcl::PointXYZI>::Ptr outcloud)//圆弧特征提取地面点效果差
//{
//    size_t nScans = _scanIndices.size();

//    for (size_t i = 0; i < nScans; i++)
//    {
//        size_t scanStartIdx = _scanIndices[i].first;
//        size_t scanEndIdx = _scanIndices[i].second;
//        vector<pcl::PointCloud<pcl::PointXYZI> > scanPoints(180);
//        float verti = std::atan(_incloud[scanStartIdx].z /sqrt(_incloud[scanStartIdx].x *_incloud[scanStartIdx].x +
//                                                               _incloud[scanStartIdx].y * _incloud[scanStartIdx].y));
//        for (int j = scanStartIdx; j <=scanEndIdx ; ++j)
//        {
//            float ori = std::atan2(_incloud[j].y, _incloud[j].x)*180/M_PI;

//            if(ori<0)
//            {
//                ori+=360;
//            }
//            scanPoints[(int)(ori/2)].push_back(_incloud[j]);
//        }
//        for (int k = 0; k < scanPoints.size(); ++k)
//        {
//            if(scanPoints[k].size()<3)
//            {
//                continue;
//            }
//            float meanZ=0;
//            for (int m = 0; m < scanPoints[k].size(); ++m)
//            {
//                meanZ+=scanPoints[k][m].z;
//            }
//            meanZ=meanZ/scanPoints[k].size();
//            float segmentRadius=meanZ/tan(verti);
//            for (int m = 0; m < scanPoints[k].size(); ++m)
//            {
//                float pointR=sqrt(pow(scanPoints[k][m].x,2)+pow(scanPoints[k][m].y,2));
//                float score=pow(pointR-segmentRadius,2)*2+pow(scanPoints[k][m].z-meanZ,2);
//                if(score<_threshold)
//                {
//                    outcloud->points.push_back(scanPoints[k][m]);
//                }
//            }
//        }

//    }
//}
