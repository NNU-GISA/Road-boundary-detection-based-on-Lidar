#include"gaussian_process.h"

gaussian_process::gaussian_process(pcl::PointCloud<pcl::PointXYZI>::Ptr candidatePoints,
                                   pcl::PointCloud<pcl::PointXYZI>::Ptr leftInitialPoints,
                                   double varthreshold,double fthreshold)
    :_varThreshold(varthreshold),_fThreshold(fthreshold)
{
    _candidatePoints=*candidatePoints;
    _leftInitialPoints=*leftInitialPoints;
    //    _rightInitialPoints=*rightInitialPoints;
    _candidateIndex.resize(_candidatePoints.size());
    _leftIndex.resize(_leftInitialPoints.size());
 #pragma omp parallel for  schedule(runtime)
    for (int i = 0; i < _candidatePoints.size(); ++i)
    {
        _candidateIndex[i]=(int)_candidatePoints[i].intensity;
    }

 #pragma omp parallel for  schedule(runtime)
    for (int i = 0; i < _leftInitialPoints.size(); ++i)
    {
        _leftIndex[i]=(int)_leftInitialPoints[i].intensity;
    }
    _remainIndex=vectors_difference(_candidateIndex,_leftIndex);
}
vector<int> gaussian_process::vectors_difference(vector<int> v1, vector<int> v2)
{
    vector<int> v;
    sort(v1.begin(), v1.end());
    sort(v2.begin(), v2.end());
    set_difference(v1.begin(), v1.end(), v2.begin(), v2.end(), back_inserter(v));
    return v;
}
void gaussian_process::initialTrainData()
{
    _xLeft.resize(_leftIndex.size());
    _yLeft.resize(_leftIndex.size());
     #pragma omp parallel for  schedule(runtime)
    for (int i = 0; i < _leftIndex.size(); ++i)
    {
        _xLeft[i]=tools::make_vector(_candidatePoints[_leftIndex[i]].x);
        _yLeft[i]=tools::make_vector(_candidatePoints[_leftIndex[i]].y);
    }

//    _xRight.resize(_rightIndex.size());
//    _yRight.resize(_rightIndex.size());
//    for (int i = 0; i < _rightIndex.size(); ++i)
//    {
//        //        _xRight[i]=_initialPoints[1][i].x;
//        //        _yRight[i]=_initialPoints[1][i].y;
//        _xRight[i]=tools::make_vector(_candidatePoints[_rightIndex[i]].x);
//        _yRight[i]=tools::make_vector(_candidatePoints[_rightIndex[i]].y);
//    }
    _remainX.resize(_remainIndex.size());
    _remainY.resize(_remainIndex.size());
     #pragma omp parallel for  schedule(runtime)
    for (int i = 0; i < _remainIndex.size(); ++i)
    {
        _remainX[i]=tools::make_vector(_candidatePoints[_remainIndex[i]].x);
        _remainY[i]=tools::make_vector(_candidatePoints[_remainIndex[i]].y);
    }

}
void gaussian_process::process(pcl::PointCloud<PointT>::Ptr clusterCloud)
{
    this->initialTrainData();
    //    double hyperparam[3]={16.1003,38.1416,0.0039};
    int newNumber=_leftIndex.size();
    using Kernel_t = kernel::Exp<Params>;
    using Mean_t = mean::Data<Params>;
    using GP_t = model::GP<Params, Kernel_t, Mean_t>;
    //    cout<<"高斯噪声为 "<<basekernel->noise()<<endl;
    // 1-D inputs, 1-D outputs
    GP_t gp(1, 1);
    while (newNumber>0)
    {
        // compute the GP
        gp.compute(_xLeft,_yLeft);
        newNumber=0;
        for (int i = 0; i < _remainIndex.size(); ++i)
        {
            Eigen::VectorXd mu;
            double sigma;
            std::tie(mu, sigma) = gp.query(_remainX[i]);
            double value1=sigma;
            double value2=abs(mu[0]-_remainY[i][0])/sqrt(value1);
            if(value1<_varThreshold&&value2<_fThreshold)
            {
                _leftIndex.push_back(_remainIndex[i]);
                _xLeft.push_back(_remainX[i]);
                _yLeft.push_back(_remainY[i]);
                newNumber+=1;
            }
        }
        _remainIndex=vectors_difference(_candidateIndex,_leftIndex);
//        _remainIndex=vectors_difference(_remainIndex,_rightIndex);
        _remainX.resize(_remainIndex.size());
        _remainY.resize(_remainIndex.size());
         #pragma omp parallel for  schedule(runtime)
        for (int i = 0; i < _remainIndex.size(); ++i)
        {
            _remainX[i]=tools::make_vector(_candidatePoints[_remainIndex[i]].x);
            _remainY[i]=tools::make_vector(_candidatePoints[_remainIndex[i]].y);
        }
    }
//    newNumber=_rightIndex.size();
//    while (newNumber>0)
//    {
//        // compute the GP
//        gp.compute(_xRight,_yRight);
//        newNumber=0;
//        for (int i = 0; i < _remainIndex.size(); ++i)
//        {
//            Eigen::VectorXd mu;
//            double sigma;
//            std::tie(mu, sigma) = gp.query(_remainX[i]);
//            double value1=sigma;
//            double value2=abs(mu[0]-_remainY[i][0])/sqrt(value1);
//            if(value1<_varThreshold&&value2<_fThreshold)
//            {
//                _rightIndex.push_back(_remainIndex[i]);
//                _xRight.push_back(_remainX[i]);
//                _yRight.push_back(_remainY[i]);
//                newNumber+=1;
//            }
//        }
//        _remainIndex=vectors_difference(_candidateIndex,_leftIndex);
//        _remainIndex=vectors_difference(_remainIndex,_rightIndex);
//        _remainX.resize(_remainIndex.size());
//        _remainY.resize(_remainIndex.size());
//        for (int i = 0; i < _remainIndex.size(); ++i)
//        {
//            _remainX[i]=tools::make_vector(_candidatePoints[_remainIndex[i]].x);
//            _remainY[i]=tools::make_vector(_candidatePoints[_remainIndex[i]].y);
//        }
//    }


    for (int i = 0; i < _leftIndex.size(); ++i)
    {
        clusterCloud->points.push_back(_candidatePoints[_leftIndex[i]]);
    }


//    for (int i = 0; i < _rightIndex.size(); ++i)
//    {
//        clusterCloud[1]->points.push_back(_candidatePoints[_rightIndex[i]]);
//    }

}
