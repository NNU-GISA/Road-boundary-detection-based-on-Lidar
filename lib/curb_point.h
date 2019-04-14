#ifndef CURB_POINT_H
#define CURB_POINT_H

#include<vector>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl/ModelCoefficients.h>
#include"math_utils.h"
#include<pcl/features/normal_3d.h>
#include<pcl/features/normal_3d_omp.h>
#include<pcl/filters/extract_indices.h>
#include<boost/make_shared.hpp>
#include<eigen3/Eigen/Dense>
#include<eigen3/Eigen/Core>
#include<algorithm>
#include<pcl/search/kdtree.h>
#include<iterator>
#include <pcl/features/don.h>
#include <pcl/filters/conditional_removal.h>
#include<pcl/io/pcd_io.h>
#include<pcl/filters/project_inliers.h>

using namespace std;
using namespace Eigen;

typedef std::pair<size_t, size_t> IndexRange;
typedef std::vector<IndexRange> scanIndices;


class curb_point
{
public:
    // -sr 2 -hr 5 -st 1 -nrmin 0.25 -nt 0.25
    curb_point(pcl::PointCloud<pcl::PointXYZI>::Ptr incloud, vector<IndexRange> scanIndices,
               float normalRegionMin=0.25,
               float normalThreshold=0,
               int heightRegion=5, float heightSigmaThreshold=0.01, int curvatureRegion=5, float curvatureThreshold=0.005, float distanceHorizonThreshold=1.5,
               float distanceVerticalThreshold=1);
    float max_z_j(int j);
    float min_z_j(int j);
    float slope_angle(int i);
    void compute_normal_omp(double searchRadius, pcl::PointCloud<pcl::Normal>& normal);
    vector<int> vectors_intersection(vector<int> v1, vector<int> v2);
    void extractPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr incloud,
                       pcl::PointCloud<pcl::PointXYZI>::Ptr outCloud,
                       boost::shared_ptr<vector<int> > indices,bool setNeg=false);
    void extractFeatures(pcl::PointCloud<pcl::PointXYZI>::Ptr feature_points);
    float calcPointDistance(const pcl::PointXYZI& p);
    float computeHorizonDiff(int index,int region);
    void normal_diff_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr incloud, pcl::IndicesConstPtr &outIndex);

private:

    pcl::PointCloud<pcl::PointXYZI> _cloud;
    scanIndices _scanindices;
    pcl::PointCloud<pcl::Normal> _normalMax;
    pcl::PointCloud<pcl::Normal> _normalMin;
    int _heightRegion;
    float _heightSigmaThreshold;


    double _normalRegionMin;
    double _normalDiffThreshold;
    float _normalThreshold;

    float _curvatureThreshold;
    int _curvatureRegion;

    float _distanceHorizonThreshold;
    double _distanceVerticalThreshold;

    vector<int> _heightPointsIndex;

    vector<int> _curvaturePointsIndex;

    vector<int> _distanceVerticlePointsIndex;

    vector<int> _distanceHorizonPointsIndex;

    vector<int> _Index;
};

#endif // CURB_POINT_H
