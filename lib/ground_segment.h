#ifndef GROUND_SEGMENT_H
#define GROUND_SEGMENT_H

#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl/PointIndices.h>
#include<pcl/ModelCoefficients.h>
#include<pcl/segmentation/sac_segmentation.h>
#include<pcl/filters/extract_indices.h>
#include<vector>
using namespace std;
//using namespace pcl;
typedef std::pair<size_t, size_t> IndexRange;
typedef std::vector<IndexRange> scanIndices;




class ground_segment
{
public:
    ground_segment(pcl::PointCloud<pcl::PointXYZI>::Ptr incloud);
    void extractGround(pcl::PointCloud<pcl::PointXYZI>::Ptr outCloud,
            pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud,
            pcl::PointIndices::Ptr Indices,
            bool setNeg=false);
    void planeSeg(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
            pcl::ModelCoefficients::Ptr coefficients,
            pcl::PointIndices::Ptr planeIndices,float groundThreshold);
    void groundfilter(pcl::PointCloud<pcl::PointXYZI>::Ptr groundpoints,
                      pcl::PointCloud<pcl::PointXYZI>::Ptr groundnopoints,double threshold);
    void process(pcl::PointCloud<pcl::PointXYZI>::Ptr outcloud);

private:
    float _threshold;
    vector<pcl::PointCloud<pcl::PointXYZI>::Ptr > _cloudptrlist;

};

#endif // GROUND_SEGMENT_H
