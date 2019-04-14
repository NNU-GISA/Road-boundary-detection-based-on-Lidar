#ifndef CLOUD_MAPPER_H
#define CLOUD_MAPPER_H


#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<vector>
#include<cmath>


using namespace std;


typedef pair<size_t, size_t> IndexRange;
typedef vector<IndexRange> scanIndices;
class cloud_mapper
{
public:
    cloud_mapper(const float& lowerBound = -24.8f,
                 const float& upperBound = 2,
                 const int& nScanRings = 64);

    const float& getLowerBound() { return _lowerBound; }
    const float& getUpperBound() { return _upperBound; }
    const int& getNumberOfScanRings() { return _nScanRings; }
    int getRingForAngle(const float& angle);
    void processByIntensity(pcl::PointCloud<pcl::PointXYZI>::Ptr incloud, pcl::PointCloud<pcl::PointXYZI>::Ptr outcloud,
                      scanIndices& scanindices);
    void processByOri(pcl::PointCloud<pcl::PointXYZI>::Ptr incloud,pcl::PointCloud<pcl::PointXYZI>::Ptr outcloud);
    void processByVer(pcl::PointCloud<pcl::PointXYZI>::Ptr incloud,pcl::PointCloud<pcl::PointXYZI>::Ptr outcloud,
                               scanIndices& scanindices);
    float _lowerBound;      ///< the vertical angle of the first scan ring
    float _upperBound;      ///< the vertical angle of the last scan ring
    int _nScanRings;   ///< number of scan rings
    float _factor;          ///< linear interpolation factor

};

#endif // CLOUD_MAPPER_H
