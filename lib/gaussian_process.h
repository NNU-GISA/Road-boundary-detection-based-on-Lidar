#ifndef GAUSSIAN_PROCESS_H
#define GAUSSIAN_PROCESS_H

#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<vector>
#include <boost/filesystem.hpp>
#include <limbo/kernel/exp.hpp>
#include <limbo/kernel/squared_exp_ard.hpp>
#include <limbo/mean/data.hpp>
#include <limbo/model/gp.hpp>
#include <limbo/model/gp/kernel_lf_opt.hpp>
#include <limbo/tools.hpp>
#include <limbo/tools/macros.hpp>
#include<Eigen/Eigen>
#include <limbo/serialize/text_archive.hpp>

using namespace std;
using namespace limbo;
using namespace Eigen;

typedef pcl::PointXYZI PointT;

class gaussian_process
{
public:
    gaussian_process(pcl::PointCloud<pcl::PointXYZI>::Ptr candidatePoints,
                     pcl::PointCloud<pcl::PointXYZI>::Ptr leftInitialPoints,
                     double varthreshold, double fthreshold);
    vector<int> vectors_difference(vector<int> v1, vector<int> v2);
    void process(pcl::PointCloud<PointT>::Ptr clusterCloud);
    void initialTrainData();

private:
    pcl::PointCloud<pcl::PointXYZI> _candidatePoints;
    pcl::PointCloud<pcl::PointXYZI> _leftInitialPoints;
//    pcl::PointCloud<pcl::PointXYZI> _rightInitialPoints;
    vector<VectorXd> _xLeft;//存储内点数据
    vector<VectorXd> _yLeft;

//    vector<VectorXd>_xRight;
//    vector<VectorXd> _yRight;

    vector<VectorXd> _remainX;//存储未处理点数据
    vector<VectorXd>_remainY;
    vector<int> _leftIndex;
//    vector<int> _rightIndex;
    vector<int> _remainIndex;
    vector<int> _candidateIndex;

    double _fThreshold;
    double _varThreshold;

    struct Params {
        struct kernel_exp
        {
            BO_PARAM(double, sigma_sq, 38.1416);
            BO_PARAM(double, l, 16.1003);
//            BO_PARAM(double, noise,0.0039);
        };
        struct kernel : public defaults::kernel
        {
        };
        struct kernel_squared_exp_ard : public defaults::kernel_squared_exp_ard
        {
        };
        struct opt_rprop : public defaults::opt_rprop
        {
        };
    };

};

#endif // GAUSSIAN_PROCESS_H
