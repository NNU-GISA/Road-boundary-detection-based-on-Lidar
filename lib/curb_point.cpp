#include "curb_point.h"

curb_point::curb_point(pcl::PointCloud<pcl::PointXYZI>::Ptr incloud, vector<IndexRange> scanIndices,
                       float normalRegionMin,
                       float normalThreshold,
                       int heightRegion, float heightSigmaThreshold, int curvatureRegion,
                       float curvatureThreshold, float distanceHorizonThreshold,float distanceVerticalThreshold)
    :_scanindices(scanIndices),
      _cloud(*incloud),
      _heightRegion(heightRegion),
      _heightSigmaThreshold(heightSigmaThreshold),
      _distanceVerticalThreshold(distanceVerticalThreshold),
      _normalRegionMin(normalRegionMin),
      _normalThreshold(normalThreshold),
      _curvatureRegion(curvatureRegion),
      _curvatureThreshold(curvatureThreshold),
      _distanceHorizonThreshold(distanceHorizonThreshold)

{
    //    compute_normal_omp(_normalRegionMin,_normalMin);//计算法向量
}
float curb_point::computeHorizonDiff(int index,int region)
{
 float ori_1 = std::atan2(_cloud[index-region].y, _cloud[index-region].x)*180/M_PI;
 float ori_2 = std::atan2(_cloud[index+region].y, _cloud[index+region].x)*180/M_PI;
 if(ori_1<0)
 {
     ori_1+=360;
 }
 if(ori_2<0)
 {
     ori_2+=360;
 }
 return abs(ori_1-ori_2);
}
float curb_point:: max_z_j(int j)
{
    float max_z =_cloud[j].z;
    for (int k = j - _heightRegion; k <= j+_heightRegion; ++k)
    {
        //float test = _laserCloud[k].z;
        if (max_z < _cloud[k].z)
        {
            max_z =_cloud[k].z;
        }
    }
    return max_z;
}

float curb_point:: min_z_j(int j)
{
    float min_z = _cloud[j].z;
    for (int k = j -_heightRegion; k <= j +_heightRegion; ++k)
    {
        if (min_z > _cloud[k].z)
        {
            min_z = _cloud[k].z;
        }
    }
    return min_z;
}

float curb_point::calcPointDistance(const pcl::PointXYZI& p)
{
    return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

vector<int> curb_point::vectors_intersection(vector<int> v1, vector<int> v2)
{
    vector<int> v;
    sort(v1.begin(), v1.end());
    sort(v2.begin(), v2.end());
    set_intersection(v1.begin(), v1.end(), v2.begin(), v2.end(), back_inserter(v));//�󽻼�
    return v;
}

void curb_point::extractPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr incloud,
                               pcl::PointCloud<pcl::PointXYZI>::Ptr outCloud,
                               boost::shared_ptr<vector<int> > indices,bool setNeg)
{
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setNegative(setNeg);
    extract.setInputCloud(incloud);
    extract.setIndices(indices);
    extract.filter(*outCloud);
}

void curb_point::extractFeatures(pcl::PointCloud<pcl::PointXYZI>::Ptr feature_points)
{
    size_t nScans = _scanindices.size();

    float angleRegionThreshold=0.3;

    for (size_t i = 0; i < nScans; i++)
    {
        size_t scanStartIdx = _scanindices[i].first;
        size_t scanEndIdx = _scanindices[i].second;

        // 跳过点数小于100的scan
        if (scanEndIdx <= scanStartIdx +100)
        {
            continue;
        }

        //提取高度特征点
        for (int k = scanStartIdx +_heightRegion; k <= scanEndIdx-_heightRegion; ++k)
        {
            if (computeHorizonDiff(k,_heightRegion)>(_heightRegion*2)*angleRegionThreshold)
            {
                continue;
            }
            //绝对高度点
            float angleVertical_1 = std::atan(_cloud[k].z / std::sqrt(_cloud[k].x *_cloud[k].x +
                                                                    _cloud[k].y *_cloud[k].y));
            float Threshold_1=abs(sin(angleVertical_1))*sqrt(_cloud[k].x *_cloud[k].x +
                                                                        _cloud[k].y *_cloud[k].y)*M_PI*0.18/180;
//            float heightMax = Threshold_1*2*_heightRegion*5;
//            float heightMin = Threshold_1*2*_heightRegion*0.5;
            float heightMax = 0.5;
            float heightMin = 0.03;
            float heightDiff=max_z_j(k)-min_z_j(k);

            //高度方差点
            float meanZ=_cloud[k].z;
            for (int j = 1; j <= _heightRegion; j++)
            {
                meanZ+= _cloud[k + j].z + _cloud[k - j].z;
            }
            meanZ=meanZ/(1+2*_heightRegion);
            float sigma_height=(_cloud[k].z-meanZ)*(_cloud[k].z-meanZ);
            for (int j = 1; j <= _heightRegion; j++)
            {
                sigma_height+=(_cloud[k + j].z-meanZ)*(_cloud[k + j].z-meanZ);
                sigma_height+=(_cloud[k - j].z-meanZ)*(_cloud[k - j].z-meanZ);
            }
            sigma_height=sqrt(sigma_height/(1+2*_heightRegion));

            //方差和绝对高度都满足条件视为候选curb点
            if(heightDiff>=heightMin&&heightDiff<=heightMax&&sigma_height>_heightSigmaThreshold)
            {
                _heightPointsIndex.push_back(k);
            }
        }
        //提取斜坡特征点
        //        for (int k = scanStartIdx + _slopeRegion; k <= scanEndIdx - _slopeRegion; ++k)
        //        {

        //            if (abs(slope_angle(k))>_slopeThreshold)
        //            {
        //                _slopePointsIndex.push_back(k);

        //            }
        //        }

        //提取法向量特征点
        //        for(int k=scanStartIdx;k<=scanEndIdx;++k)
        //        {
        //            Vector3f normalMin(_normalMin[k].normal_x,_normalMin[k].normal_y,_normalMin[k].normal_z);
        //            Vector3f normal_z(0,0,1);
        //            float normalDiff_z=abs(normal_z.dot(normalMin));
        //            normalMin[0]=0;
        //            //Vector3f normalMax(_normalMax[k].normal_x,_normalMax[k].normal_y,_normalMax[k].normal_z);
        //            //float norm=normalMax.squaredNorm();
        //            //              normalMin.normalize();
        //            //              normalMax.normalize();
        //            Vector3f normal_y(0,1,0);

        //            //Vector3f ground(groundModel->values[0],groundModel->values[1],groundModel->values[2]);
        //            //            cout<<"ground model is "<<groundModel->values[0]<<" "<<groundModel->values[1]<<" "
        //            //               <<groundModel->values[2]<<" "<<groundModel->values[3]<<endl;
        //            //float normalDiff=((normalMax-normalMin)/2).squaredNorm();
        //            float normalDiff=abs(normal_y.dot(normalMin));

        //            if(normalDiff>=_normalThreshold)
        //            {
        //                _normalPointsIndex.push_back(k);
        //            }
        //        }
        //        pcl::PointCloud<pcl::PointXYZI>::Ptr scanCloud(new pcl::PointCloud<pcl::PointXYZI>);
        //        pcl::IndicesConstPtr points_removed(new vector<int>);
        //        for(int k=scanStartIdx;k<=scanEndIdx;++k)
        //        {
        //           scanCloud->points.push_back(_cloud[k]);
        //        }
        //        normal_diff_filter(scanCloud,points_removed);
        //        for(int k=scanStartIdx;k<=scanEndIdx;++k)
        //        {
        //                if(find((*points_removed).begin(),(*points_removed).end(),k-scanStartIdx)==(*points_removed).end())
        //                {
        //                    _normalPointsIndex.push_back(k);
        //                }
        //        }
        //提取平滑特征点
        float pointWeight = -2 * _curvatureRegion;

        for (size_t k = scanStartIdx+_curvatureRegion; k <= scanEndIdx-_curvatureRegion; k++)
        {
            if (computeHorizonDiff(k,_curvatureRegion)>(_curvatureRegion*2)*angleRegionThreshold)
            {
                continue;
            }
            float diffX = pointWeight * _cloud[k].x;
            float diffY = pointWeight * _cloud[k].y;
            float diffZ = pointWeight * _cloud[k].z;

            for (int j = 1; j <= _curvatureRegion; j++)
            {
                diffX += _cloud[k + j].x + _cloud[k - j].x;
                diffY += _cloud[k + j].y + _cloud[k - j].y;
                diffZ += _cloud[k + j].z + _cloud[k - j].z;
            }

            float curvatureValue=sqrt(diffX*diffX+diffY*diffY+diffZ*diffZ)/
                    (_curvatureRegion*calcPointDistance(_cloud[k]));
            if(curvatureValue>_curvatureThreshold)
            {
                _curvaturePointsIndex.push_back(k);
            }

        }


     //平面距离特征点
        for (int k = scanStartIdx+1; k <= scanEndIdx; ++k)
        {
            if (computeHorizonDiff(k,1)>(1*2)*angleRegionThreshold)
            {
                continue;
            }
            float distanceHorizonThreshold=sqrt(_cloud[k].x *_cloud[k].x +_cloud[k].y *_cloud[k].y)*M_PI*0.18/180;
            float distancePre=sqrt((_cloud[k].x-_cloud[k-1].x)*(_cloud[k].x-_cloud[k-1].x) +
                    (_cloud[k].y-_cloud[k-1].y) *(_cloud[k].y-_cloud[k-1].y));
            if(distancePre>distanceHorizonThreshold*_distanceHorizonThreshold)
            {
                _distanceHorizonPointsIndex.push_back(k);
            }
        }
        //垂直距离特征
        for (int k = scanStartIdx+1; k <= scanEndIdx; ++k)
        {
            if (computeHorizonDiff(k,1)>(1*2)*angleRegionThreshold)
            {
                continue;
            }
            float angleVertical = std::atan(_cloud[k].z / std::sqrt(_cloud[k].x *_cloud[k].x +
                                                                    _cloud[k].y *_cloud[k].y));
            float distanceHorizonThreshold=abs(sin(angleVertical))*sqrt(_cloud[k].x *_cloud[k].x +_cloud[k].y *_cloud[k].y)*M_PI*0.18/180;
            float distancePre=abs(_cloud[k].z-_cloud[k-1].z);
            if(distancePre>distanceHorizonThreshold*_distanceVerticalThreshold)
            {
                _distanceVerticlePointsIndex.push_back(k);
            }
        }

    }
        cout<<" height Index is "<<_heightPointsIndex.size()<<endl;
        cout<<" curvature Index is "<<_curvaturePointsIndex.size()<<endl;
        cout<<" distance horizontal Index is "<<_distanceHorizonPointsIndex.size()<<endl;
    vector<int> Index_temp;
    Index_temp=vectors_intersection(_heightPointsIndex,_curvaturePointsIndex);
    _Index=vectors_intersection(Index_temp,_distanceHorizonPointsIndex);
//            _Index=_heightPointsIndex;
    _Index=vectors_intersection(_Index,_distanceVerticlePointsIndex);

    //cout<<"scan "<< i <<" normal points is "<<_normalPointsIndex[i].size()<<endl;
    extractPoints(boost::make_shared<pcl::PointCloud<pcl::PointXYZI> >(_cloud),feature_points,boost::make_shared<vector<int> >(_Index));
}

//float curb_point::slope_angle(int i)
//{

//    float angle_1 = atan((_cloud[i + _slopeRegion].z - _cloud[i - _slopeRegion].z)/ math::calcPlaneDistance(_cloud[i+_slopeRegion], _cloud[i - _slopeRegion]))*180/M_PI;

//    float angle_2 = atan((_cloud[i+_slopeRegion].z-_cloud[i].z)/math::calcPlaneDistance(_cloud[i], _cloud[i + _slopeRegion])) * 180 / M_PI;
//    return angle_1- angle_2;
//}

//void curb_point::compute_normal_omp(double searchRadius,pcl::PointCloud<pcl::Normal>& normal)
//{
//    pcl::NormalEstimationOMP<pcl::PointXYZI,pcl::Normal> ne_omp;
//    ne_omp.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZI> >(_cloud));
//    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
//    tree->setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZI> >(_cloud));
//    ne_omp.setSearchMethod(tree);
//    ne_omp.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());
//    ne_omp.setRadiusSearch(searchRadius);
////        ne_omp.setKSearch(searchRadius);
//    ne_omp.compute(normal);
//}
//void curb_point::normal_diff_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr incloud,pcl::IndicesConstPtr&outIndex)
//{
//    pcl::PointCloud<pcl::PointXYZI>::Ptr outcloud(new pcl::PointCloud<pcl::PointXYZI>);
//    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
//    coefficients->values.resize(4);
//    coefficients->values[0] = 1;
//    coefficients->values[1] = 0;
//    coefficients->values[2] = 0;
//    coefficients->values[3] = 0;
//    pcl::ProjectInliers<pcl::PointXYZI> proj;
//    proj.setModelType(pcl::SACMODEL_PLANE);
//    proj.setInputCloud(incloud);
//    proj.setModelCoefficients(coefficients);
//    proj.filter(*outcloud);

//    pcl::PointCloud<pcl::PointNormal>::Ptr normals_small_scale (new pcl::PointCloud<pcl::PointNormal>);
//    pcl::PointCloud<pcl::PointNormal>::Ptr normals_large_scale (new pcl::PointCloud<pcl::PointNormal>);
//    pcl::NormalEstimationOMP<pcl::PointXYZI,pcl::PointNormal> ne_omp;
//    ne_omp.setInputCloud(outcloud);
//    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
//    tree->setInputCloud(outcloud);
//    ne_omp.setSearchMethod(tree);
//    ne_omp.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());

//    ne_omp.setRadiusSearch(_normalRegionMin);
//    ne_omp.compute(*normals_small_scale);
//    ne_omp.setRadiusSearch(_normalRegionMax);
//    ne_omp.compute(*normals_large_scale);

//    pcl::PointCloud<pcl::PointNormal>::Ptr doncloud (new pcl::PointCloud<pcl::PointNormal>);
//    pcl::copyPointCloud<pcl::PointXYZI, pcl::PointNormal>(*outcloud, *doncloud);

//    pcl::DifferenceOfNormalsEstimation<pcl::PointXYZI, pcl::PointNormal, pcl::PointNormal> don;
//    don.setInputCloud (outcloud);
//    don.setNormalScaleLarge (normals_large_scale);
//    don.setNormalScaleSmall (normals_small_scale);
//    don.computeFeature (*doncloud);

//    pcl::ConditionOr<pcl::PointNormal>::Ptr range_cond (new pcl::ConditionOr<pcl::PointNormal> ());
//    range_cond->addComparison (pcl::FieldComparison<pcl::PointNormal>::ConstPtr (
//                                   new pcl::FieldComparison<pcl::PointNormal> ("curvature", pcl::ComparisonOps::GT, _normalThreshold)));
//    pcl::ConditionalRemoval<pcl::PointNormal> condrem(true);
//    condrem.setCondition(range_cond);
//    condrem.setInputCloud (doncloud);
//    pcl::PointCloud<pcl::PointNormal>::Ptr doncloud_filtered (new pcl::PointCloud<pcl::PointNormal>);
//    condrem.filter (*doncloud_filtered);
//    outIndex=condrem.getRemovedIndices();
//}
