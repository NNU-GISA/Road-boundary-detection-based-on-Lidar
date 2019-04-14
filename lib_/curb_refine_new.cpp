
#include<pcl/search/kdtree.h>
#include<pcl/features/normal_3d_omp.h>
#include <pcl/segmentation/region_growing.h>
#include<vector>
#include<boost/make_shared.hpp>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<pcl/sample_consensus/ransac.h>
#include<pcl/sample_consensus/sac_model_line.h>
#include <pcl/filters/project_inliers.h>
#include<pcl/segmentation/extract_labeled_clusters.h>
#include<pcl/filters/conditional_removal.h>
#include<pcl/filters/statistical_outlier_removal.h>
#include"curb_refine_new.h"
#include"cloud_mapper.h"


void curb_refine_new::extractPointCloud(pcl::PointCloud<PointT>& incloud, boost::shared_ptr<vector<int> > indices,pcl::PointCloud<PointT>& outcloud)
{

    //    boost::shared_ptr<vector<int> > indice_=boost::make_shared<vector<int> >(indices);
    pcl::PointCloud<PointT>::Ptr cloudOut(new pcl::PointCloud<PointT>);
//    pcl::PointCloud<PointT>::Ptr cloudIn(new pcl::PointCloud<PointT>);
//    *cloudIn=_completeCloud;
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setNegative(false);
    extract.setInputCloud(boost::make_shared<pcl::PointCloud<PointT> >(incloud));
    extract.setIndices(indices);
    extract.filter(*cloudOut);
    outcloud=*cloudOut;

}
void curb_refine_new::statisticalFilter(pcl::PointCloud<PointT> incloud,pcl::PointCloud<PointT>& outcloud)
{
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud (boost::make_shared<pcl::PointCloud<PointT> >(incloud));
    sor.setMeanK (30);
    sor.setStddevMulThresh (_tolerance);
    sor.setKeepOrganized(false);
    //boost::make_shared<pcl::PointIndices>(_pointIndices)= sor.getRemovedIndices();
    sor.filter (outcloud);
    //sor.filter(_indices);

}
void curb_refine_new::statisticalFilter_indces(pcl::PointCloud<PointT> incloud,int meanK,vector<int>* indices,double stdThreshold)
{
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud (boost::make_shared<pcl::PointCloud<PointT> >(incloud));
    sor.setMeanK (meanK);
    sor.setStddevMulThresh (stdThreshold);
    sor.setKeepOrganized(false);
    //boost::make_shared<pcl::PointIndices>(_pointIndices)= sor.getRemovedIndices();
    sor.filter (*indices);
    //sor.filter(_indices);

}

curb_refine_new::curb_refine_new(pcl::PointCloud<PointT>::Ptr incloud, double tolerance):
    _completeCloud(*incloud),_tolerance(tolerance)
{
    pcl::PointCloud<PointT>::Ptr completeCloudLeft(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr completeCloudRight(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr completeCloudLeft2D(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr completeCloudRight2D(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr completeCloudLeftFiltered(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr completeCloudRightFiltered(new pcl::PointCloud<PointT>);
    vector<int> leftIndices;
    vector<int> rightIndices;

    for(int i=0;i<_completeCloud.size();++i)
    {
        if(_completeCloud[i].y>0&&_completeCloud[i].x>=-8)
        {
            completeCloudLeft->points.push_back(_completeCloud[i]);
        }
        if(_completeCloud[i].y<0&&_completeCloud[i].x>=-8)
        {
            completeCloudRight->points.push_back(_completeCloud[i]);
        }
    }

    pointcloud_projection(completeCloudLeft,*completeCloudLeft2D);
    statisticalFilter_indces(*completeCloudLeft2D,30,&leftIndices,4);
    extractPointCloud( *completeCloudLeft,boost::make_shared<vector<int> >(leftIndices),*completeCloudLeftFiltered);
    pointcloud_projection(completeCloudRight,*completeCloudRight2D);
    statisticalFilter_indces(*completeCloudRight2D,30,&rightIndices,4);
    extractPointCloud( *completeCloudRight,boost::make_shared<vector<int> >(rightIndices),*completeCloudRightFiltered);
    _completeCloudFiltered+=*completeCloudLeftFiltered;
    _completeCloudFiltered+=*completeCloudRightFiltered;

}
void curb_refine_new::pointcloud_projection(pcl::PointCloud<PointT>::Ptr incloud,pcl::PointCloud<PointT>& outcloud)
{
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    coefficients->values.resize(4);
    coefficients->values[0] = 1;
    coefficients->values[1] = 0;
    coefficients->values[2] = 0;
    coefficients->values[3] = 0;
    pcl::ProjectInliers<pcl::PointXYZI> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(incloud);
    proj.setModelCoefficients(coefficients);
    proj.filter(outcloud);
}

void curb_refine_new::process(vector<pcl::PointCloud<pcl::PointXYZI>::Ptr > clusterCloud)
{
    cloud_mapper mapper;
    scanIndices scanIDindices;

    pcl::PointCloud<pcl::PointXYZI>::Ptr feature_points_mapper(new pcl::PointCloud<pcl::PointXYZI>);
//    mapper.process(boost::make_shared<pcl::PointCloud<PointT> >(_completeCloudFiltered),feature_points_mapper,scanIDindices);

    pcl::PointCloud<PointT>::Ptr leftcloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr Rightcloud(new pcl::PointCloud<PointT>);

    // 从每个scan中提取curb点
    size_t nScans = scanIDindices.size();
    //    *clusterCloud[0]=_completeCloudFiltered;
    for (size_t i = 0; i < nScans; i++)
    {
        size_t scanStartIdx = scanIDindices[i].first;
        size_t scanEndIdx = scanIDindices[i].second;


        vector<pcl::PointXYZI> LeftFront;
        vector<pcl::PointXYZI> LeftRear;
        vector<pcl::PointXYZI> RightFront;
        vector<pcl::PointXYZI> RightRear;

        for (size_t i = scanStartIdx; i <= scanEndIdx; ++i)
        {
            if (feature_points_mapper->points[i].x >= 0&&feature_points_mapper->points[i].y>=0)
            {
                LeftFront.push_back(feature_points_mapper->points[i]);
            }
            else if(feature_points_mapper->points[i].x >= 0&&feature_points_mapper->points[i].y<0)
            {
                RightFront.push_back(feature_points_mapper->points[i]);
            }
            else if(feature_points_mapper->points[i].x<0&&feature_points_mapper->points[i].y>=0)
            {
                LeftRear.push_back(feature_points_mapper->points[i]);
            }
            else
            {
                RightRear.push_back(feature_points_mapper->points[i]);
            }
        }


        //针对每个scan每个象限分别提取y值绝对值最小的点
        //左前方
        if (LeftFront.size() > 0)
        {
            int min_y_LeftFront = 0;
            for (size_t i = 0; i < LeftFront.size(); ++i)
            {

                if(abs(LeftFront[min_y_LeftFront].y)> abs(LeftFront[i].y))
                {
                    min_y_LeftFront = i;
                }
            }
            leftcloud->points.push_back(LeftFront[min_y_LeftFront]);
        }
        //左后方
        if (LeftRear.size() > 0)
        {
            int min_y_LeftRear = 0;
            for (size_t i = 0; i < LeftRear.size(); ++i)
            {

                if(abs(LeftRear[min_y_LeftRear].y)> abs(LeftRear[i].y))
                {
                    min_y_LeftRear = i;
                }
            }
            leftcloud->points.push_back(LeftRear[min_y_LeftRear]);
        }
        //右前方
        if (RightFront.size() > 0)
        {
            int min_y_RightFront = 0;
            for (size_t i = 0; i < RightFront.size(); ++i)
            {

                if(abs(RightFront[min_y_RightFront].y)> abs(RightFront[i].y))
                {
                    min_y_RightFront = i;
                }
            }
            Rightcloud->points.push_back(RightFront[min_y_RightFront]);
        }
        //右后方
        if (RightRear.size() > 0)
        {
            int min_y_RightRear = 0;
            for (size_t i = 0; i < RightRear.size(); ++i)
            {

                if(abs(RightRear[min_y_RightRear].y)> abs(RightRear[i].y))
                {
                    min_y_RightRear = i;
                }
            }
            Rightcloud->points.push_back(RightRear[min_y_RightRear]);
        }
    }
    pcl::PointCloud<PointT>::Ptr leftCloudRefineTemp(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr RightCloudRefineTemp(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr leftCloudRefineTemp2D(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr RightCloudRefineTemp2D(new pcl::PointCloud<PointT>);
    vector<int> leftIndices;
    vector<int> rightIndices;
  curbRefine leftRefine(leftcloud,_tolerance);
  leftRefine.process(leftCloudRefineTemp);
  curbRefine rightRefine(Rightcloud,_tolerance);
  rightRefine.process(RightCloudRefineTemp);
//  *clusterCloud[0]=*leftCloudRefineTemp;
//  *clusterCloud[1]=*RightCloudRefineTemp;
if(leftCloudRefineTemp->points.size()>=2)
{
    pointcloud_projection(leftCloudRefineTemp,*leftCloudRefineTemp2D);
    statisticalFilter_indces(*leftCloudRefineTemp2D,leftCloudRefineTemp2D->points.size()-1,&leftIndices,0.2);
    extractPointCloud(*leftCloudRefineTemp,boost::make_shared<vector<int> >(leftIndices),*clusterCloud[0]);
}
if(RightCloudRefineTemp->points.size()>=2)
{
    pointcloud_projection(RightCloudRefineTemp,*RightCloudRefineTemp2D);
    statisticalFilter_indces(*RightCloudRefineTemp2D,RightCloudRefineTemp2D->points.size()-1,&rightIndices,0.2);
//    extractPointCloud(*RightCloudRefineTemp,boost::make_shared<vector<int> >(rightIndices),*clusterCloud[1]);
}
*clusterCloud[0]=*leftcloud;
       *clusterCloud[1]=*Rightcloud;
cout<<" after refine left points number is "<<clusterCloud[0]->points.size()<<endl;
cout<<" after refine right points number is "<<clusterCloud[1]->points.size()<<endl;
}

