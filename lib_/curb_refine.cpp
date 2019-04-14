#include "curb_refine.h"
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

struct CompareSecondMin
{
    bool operator()(const MyPairType& left, const MyPairType& right) const
    {
        return left.second < right.second;
    }
};
struct CompareSecondMax
{
    bool operator()(const MyPairType& left, const MyPairType& right) const
    {
        return left.second < right.second;
    }
};

curbRefine::curbRefine(pcl::PointCloud<PointT>::Ptr incloud, double tolerance):
    _completeCloud(*incloud),_tolerance(tolerance),_pointIndices()
{
    pointcloud_projectionYZ(boost::make_shared<pcl::PointCloud<PointT> >(_completeCloud),
                            _completeCloud2DYZ);
    pointcloud_projectionXY(boost::make_shared<pcl::PointCloud<PointT> >(_completeCloud),
                            _completeCloud2DXY);
    //    conditionalEucludieanClustering();
    //LabeledEucludieanClustering();
    //    statisticalFilter(_completeCloud2D,_completeCloud2DFiltered);
    eucludieanClustering();
}
void curbRefine::statisticalFilter(pcl::PointCloud<PointT> incloud,pcl::PointCloud<PointT>& outcloud)
{
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud (boost::make_shared<pcl::PointCloud<PointT> >(incloud));
    sor.setMeanK (30);
    sor.setStddevMulThresh (1.0);
    sor.setKeepOrganized(false);
    //boost::make_shared<pcl::PointIndices>(_pointIndices)= sor.getRemovedIndices();
    sor.filter (outcloud);
    //sor.filter(_indices);

}
void curbRefine::conditionalFilter(pcl::PointCloud<PointT> incloud,pcl::PointCloud<PointT>& outcloud)
{
    Vector4d centroidPoint;
    pcl::compute3DCentroid(incloud,centroidPoint);
    pcl::ConditionAnd<PointT>::Ptr range_cond (new pcl::ConditionAnd<PointT> ());
    if(centroidPoint(1)>0)
    {

        range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new
                                                                           pcl::FieldComparison<PointT> ("y", pcl::ComparisonOps::LT, centroidPoint(1)+0.2)));
        range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new
                                                                           pcl::FieldComparison<PointT> ("y", pcl::ComparisonOps::GT, centroidPoint(1)-0.45)));
    }
    else
    {
        range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new
                                                                           pcl::FieldComparison<PointT> ("y", pcl::ComparisonOps::GT, centroidPoint(1)-0.2)));
        range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new
                                                                           pcl::FieldComparison<PointT> ("y", pcl::ComparisonOps::LT, centroidPoint(1)+0.45)));
    }
    pcl::ConditionalRemoval<PointT> condrem;
    condrem.setCondition (range_cond);
    condrem.setInputCloud (boost::make_shared<pcl::PointCloud<PointT> >(incloud));
    condrem.setKeepOrganized(false);
    // apply filter
    condrem.filter (outcloud);


}
int curbRefine::getMin(map<int, float> mymap)
{
    MyPairType min = *min_element(mymap.begin(), mymap.end(), CompareSecondMin());
    return min.first;
}
int curbRefine::getMax(map<int, float> mymap)
{
    MyPairType max = *max_element(mymap.begin(), mymap.end(), CompareSecondMax());
    return max.first;
}


bool curbRefine::customCondition(const PointT& seedPoint, const PointT& candidatePoint,
                                  float squaredDistance)
{
    // Do whatever you want here.返回true肯定是一个类别，返回false不一定
    float xDistance=fabs(seedPoint.x-candidatePoint.x);
    if(fabs(seedPoint.y-candidatePoint.y)<0.2)
    {
        return true;
    }
    if (xDistance>4&&xDistance<=7 &&fabs(seedPoint.y-candidatePoint.y) <xDistance*0.07)
    {
        return true;
    }
    if(xDistance>7&&xDistance<=20&&fabs(seedPoint.y-candidatePoint.y)<xDistance*0.08)
    {
        return true;
    }
    if(candidatePoint.y*seedPoint.y>0)
    {
        return true;
    }

    return false;
}

void curbRefine::pointcloud_projectionXY(pcl::PointCloud<PointT>::Ptr incloud,pcl::PointCloud<PointT>& outcloud)
{
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    coefficients->values.resize(4);
    coefficients->values[0] = 0;
    coefficients->values[1] = 0;
    coefficients->values[2] = 1;
    coefficients->values[3] = 0;
    pcl::ProjectInliers<pcl::PointXYZI> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(incloud);
    proj.setModelCoefficients(coefficients);
    proj.filter(outcloud);
}
void curbRefine::pointcloud_projectionYZ(pcl::PointCloud<PointT>::Ptr incloud,pcl::PointCloud<PointT>& outcloud)
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

void curbRefine::lineFit(pcl::PointCloud<PointT>::Ptr incloud,pcl::PointIndices& indices,pcl::ModelCoefficients& model)
{
        Vector3f axis(1,0,0);
    //        pcl::ModelCoefficients model
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    //    seg.setModelType(pcl::SACMODEL_LINE);
    seg.setModelType(pcl::SACMODEL_LINE);
//    seg.setModelType(pcl::SACMODEL_PARALLEL_LINE);
//    seg.setAxis(axis);
//    seg.setEpsAngle(0.05);
    //    seg.setAxis(axis);
    //    seg.setEpsAngle(0.05);
    seg.setMethodType(pcl::SAC_MSAC);
    seg.setDistanceThreshold(0.1);
    seg.setInputCloud(incloud);
    seg.segment(indices,model);
}
int curbRefine::getNumberofClusters()
{
    return _clusters.size();
}

void curbRefine::generateClusters()
{
    // For every cluster...
    // int currentClusterNum = 1;.
    _cloudVector.resize(_clusters.size());
    //    _cloudVectorFiltered.resize(_clusters.size());
    _cloudVector2D.resize(_clusters.size());
    for (int i=0;i<_clusters.size();++i)
    {
        // ...add all its points to a new cloud...
        //pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
        for (std::vector<int>::const_iterator point = _clusters[i].indices.begin(); point !=_clusters[i].indices.end(); point++)
        {
            _cloudVector[i].points.push_back(_completeCloud.points[*point]);
            _cloudVector2D[i].points.push_back(_completeCloud2DXY.points[*point]);
        }
        _cloudVector[i].width = _cloudVector[i].points.size();
        _cloudVector[i].height = 1;
        _cloudVector[i].is_dense = true;
        //        std::cout << "Cluster " << i << " has " <<_cloudVector[i].points.size() << " points." << std::endl;
        //std::string fileName = clusterType+"_cluster" + boost::to_string(currentClusterNum) + ".pcd";
        //currentClusterNum++;
    }
}

void curbRefine::conditionalEucludieanClustering()
{
    // Conditional Euclidean clustering object.
    //    pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);
    pcl::ConditionalEuclideanClustering<PointT> clustering;
    clustering.setClusterTolerance(_tolerance);
    clustering.setMinClusterSize(50);
    clustering.setMaxClusterSize(5000);
    clustering.setInputCloud(boost::make_shared<pcl::PointCloud<PointT> >(_completeCloud2DYZ));
    // Set the function that will be called for every pair of points to check.
    clustering.setConditionFunction(&curbRefine::customCondition);
    clustering.segment(_clusters);
}
void curbRefine::eucludieanClustering ()
{
    pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);

    //kdtree->setInputCloud(cloud);

    // Euclidean clustering object.
    pcl::EuclideanClusterExtraction<PointT> clustering;
    // Set cluster tolerance to 2cm (small values may cause objects to be divided
    // in several clusters, whereas big values may join objects in a same cluster).
    clustering.setClusterTolerance(_tolerance);
    // Set the minimum and maximum number of points that a cluster can have.
    clustering.setMinClusterSize(8);
    clustering.setMaxClusterSize(500);
    clustering.setSearchMethod(kdtree);
    clustering.setInputCloud(boost::make_shared<pcl::PointCloud<PointT> >(_completeCloud2DYZ));
    //std::vector<pcl::PointIndices> clusters;
    clustering.extract(_clusters);
    //generateClusters(clusters,cloud,"eucludieanClustering");
}
void curbRefine::process(pcl::PointCloud<pcl::PointXYZI>::Ptr clusterCloud)
{

    if(_clusters.size()==0)
    {
        cout<<" One side has no feature points"<<endl;
        return;
    }
    generateClusters();
    _LineModelCoefVector.resize(_clusters.size());
    _LinePointsIndicesVector.resize(_clusters.size());
    //    _clusterLabel.resize(_clusters.size(),-1);
    //计算每个cluster的直线模型系数
    for(int i=0;i<_cloudVector.size();++i)
    {
        //        _cloudVectorFiltered.resize(_clusters.size());
        lineFit(boost::make_shared<pcl::PointCloud<PointT> >(_cloudVector[i]),
                _LinePointsIndicesVector[i],_LineModelCoefVector[i]);

        //根据在y轴截距符号将所有cluster分为左右两类
        Vector3f pointOnLine=Vector3f::Map(&_LineModelCoefVector[i].values[0],3);
        Vector3f LineCof=Vector3f::Map(&_LineModelCoefVector[i].values[3],3);
        //        float b=abs(pointOnLine(1)-LineCof(1)/LineCof(0)*pointOnLine(0));
        float b=abs((0-pointOnLine[0])/LineCof[0]*LineCof[1]+pointOnLine[1]);
        _ClusterIntercept.insert(map<int,float>::value_type(i,b));


        //            _rightClusterIntercept.insert(map<int,float>::value_type(i,b));
        //            //                    cout<<"right ids is "<<i<<endl;
        //        }
        //        cout<<"model cof size is "<<_LineModelCoefVector[i].values.size()<<endl;
        //        cout<<" model "<<i<<" coef is "<<_LineModelCoefVector[i].values[0]<<" "<<_LineModelCoefVector[i].values[1]<<" "<<
        //              _LineModelCoefVector[i].values[2]<<" "<<_LineModelCoefVector[i].values[3]<<" "<<_LineModelCoefVector[i].values[4]<<
        //              " "<<_LineModelCoefVector[i].values[5]<<endl;

        //                (*clusterCloud[i])=_cloudVector[i];
    }
    //    (*cloud_filtered)=_completeCloud2DFiltered;

    int ClusterID=getMin(_ClusterIntercept);
    *clusterCloud=_cloudVector[ClusterID];
    //    int RightClusterID=getMax(_rightClusterIntercept);
    //    cout<<" last left id is "<<LeftClusterID<<endl;
    //    cout<<"last right id is "<<RightClusterID<<endl;
    //    *clusterCloud[0]=_cloudVector[LeftClusterID];
    //    *clusterCloud[1]=_cloudVector[RightClusterID];
    //    conditionalFilter(_cloudVector[LeftClusterID],*clusterCloud[0]);
    //    conditionalFilter(_cloudVector[RightClusterID],*clusterCloud[1]);
}

//void curb_refine::regionBasedClustering(pcl::PointCloud<PointT>::Ptr cloud,int searchRadius,float smoothThreshold,float curveThreshold)
//{
//    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
//    // Estimate the normals.
//    pcl::NormalEstimationOMP<PointT,pcl::Normal> ne_omp;
//    ne_omp.setInputCloud(cloud);
//    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
//    //tree->setInputCloud(cloud);
//    ne_omp.setSearchMethod(tree);
//    //ne_omp.setRadiusSearch(searchRadius);
//    ne_omp.setKSearch(3);
//    ne_omp.compute(*normals);

//    // Region growing clustering object.
//    pcl::RegionGrowing<PointT, pcl::Normal> clustering;
//    clustering.setMinClusterSize(5);
//    clustering.setMaxClusterSize(1000);
//    clustering.setSearchMethod(tree);
//    clustering.setNumberOfNeighbours(searchRadius);
//    clustering.setInputCloud(cloud);
//    clustering.setInputNormals(normals);
//    // Set the angle in radians that will be the smoothness threshold
//    // (the maximum allowable deviation of the normals).
//    clustering.setSmoothnessThreshold(smoothThreshold / 180.0 * M_PI); // 7 degrees.
//    // Set the curvature threshold. The disparity between curvatures will be
//    // tested after the normal deviation check has passed.
//    clustering.setCurvatureThreshold(curveThreshold);
//    clustering.extract(_clusters);
//    //generateClusters(clusters,cloud,"regionBasedClustering");
//}

//void curb_refine::LabeledEucludieanClustering()
//{
//    pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);

//    //kdtree->setInputCloud(cloud);

//    // Euclidean clustering object.
//    pcl::LabeledEuclideanClusterExtraction <PointT> clustering;
//    // Set cluster tolerance to 2cm (small values may cause objects to be divided
//    // in several clusters, whereas big values may join objects in a same cluster).
//    clustering.setClusterTolerance(_tolerance);
//    // Set the minimum and maximum number of points that a cluster can have.
//    clustering.setMinClusterSize(10);
//    clustering.setMaxClusterSize(1000);
//    clustering.setSearchMethod(kdtree);
//    clustering.setInputCloud(boost::make_shared<pcl::PointCloud<PointT> >(_completeCloud));
//    clustering.setMaxLabels(2);
//    //std::vector<pcl::PointIndices> clusters;
//   // clustering.extract(_clusters);
//    //generateClusters(clusters,cloud,"eucludieanClusterin
//}
//    //根据模型系数判断是否共线，如果两个cluster共线则标记为同一个cluster
//    for(int i=0;i<_cloudVector.size();++i)
//    {
//        //        if(_clusterLabel[i]==-1)
//        //        {
//        //            _clusterLabel[i]=i;
//        //        }
//        //        for(int j=i+1;j<_cloudVector.size();++j)
//        //        {
//        //            if(_clusterLabel[j]==-1)
//        //            {

//        //                Vector2f pointOnLine_2=Vector2f::Map(&_LineModelCoefVector[j].values[0],2);
//        //                Vector2f LineCof_2=Vector2f::Map(&_LineModelCoefVector[j].values[3],2);
//        //Vector2f pointDiff=(pointOnLine_1-pointOnLine_2);
//        //pointDiff=pointDiff/(pointDiff.squaredNorm());
//        //                float k=LineCof(1)/LineCof(0);
//        //计算每个类别拟合的直线
//
//        //                float k2=LineCof_2(1)/LineCof_2(0);
//        //        /*        float b2=pointOnLine_2(1)-LineCof_2(1)/LineCof_2(0)*pointOnLine_2(0);
//        //                if(fabs(k1-k2)/(fabs(k1)+fabs(k2))<=_lineThreshold_1&&
//        //                        fabs(b1-b2)/(fabs(b1)+fabs(b2))<=_lineThreshold_2)
//        //                {
//        //                    _clusterLabel[j]=_clusterLabel[i];
//        //                }*/
//


//        //

//        cout<<" cluster "<<i<<" laber is "<<_clusterLabel[i]<<endl;
//    }
//将标记为同一个cluster的点云存入同一点云
//    for(int i=0;i<_cloudVector.size();++i)
//    {
//        for(int j=0;j<_clusterLabel.size();++j)
//        {
//            if(_clusterLabel[j]==i)
//            {
//             (*(clusterCloud[i]))+=_cloudVector[j];
//            }
//        }
//    }
//    cout<<" last feature cluster is "<<clusterCloud.size()<<endl;



//pcl::PointCloud<PointT>::Ptr curb_refine::filterNANS(pcl::PointCloud<PointT>::Ptr cloud)
//{
//  // Read in the cloud data
// // reader.read ("table_scene_mug_stereo_textured.pcd", *cloud);
//  std::cerr << "PointCloud has: " << cloud->points.size () << " data points." << std::endl;
//  pcl::PassThrough<PointT> pass;
//  pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
//  // Build a passthrough filter to remove spurious NaNs
//  pass.setInputCloud (cloud);
//  pass.setFilterFieldName ("z");
//  pass.setFilterLimits (0, 1.5);
//  pass.filter (*cloud_filtered);
//  std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;
//  return cloud_filtered;
//}
//pcl::PointCloud<PointT>::Ptr curb_refine::extractInliers(pcl::PointCloud<PointT>::Ptr cloud_filtered,pcl::PointIndices::Ptr inliers,bool flag){
//  pcl::ExtractIndices<PointT> extract;
//  extract.setNegative(flag);//true for other than segmented
//                            //false for only segmented
//  pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
//  extract.setInputCloud (cloud_filtered);
//  extract.setIndices (inliers);
//  extract.filter (*cloud_plane);
//  return cloud_plane;
//}
// //In this function, will learn  segment arbitrary plane models from a given point cloud dataset.
//pcl::PointCloud<PointT>::Ptr curb_refine::cylinderSegmentation(pcl::PointCloud<PointT>::Ptr cloud){
//  pcl::PointCloud<PointT>::Ptr inlierPoints(new pcl::PointCloud<PointT>);

//  // Object for storing the plane model coefficients.
//  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
//  // Create the segmentation object.
//  pcl::SACSegmentation<PointT> segmentation;
//  segmentation.setInputCloud(cloud);
//  // Configure the object to look for a plane.
//  segmentation.setModelType(pcl::SACMODEL_CYLINDER);
//  // Use RANSAC method.
//  segmentation.setMethodType(pcl::SAC_RANSAC);
//  // Set the maximum allowed distance to the model.
//  segmentation.setDistanceThreshold(0.01);
//  // Enable model coefficient refinement (optional).
//  segmentation.setOptimizeCoefficients(true);
//  // Set minimum and maximum radii of the cylinder.
//  segmentation.setRadiusLimits(0, 0.1);
//  pcl::PointIndices inlierIndices;
//  segmentation.segment(inlierIndices, *coefficients);
//  if (inlierIndices.indices.size() == 0)
//    std::cout << "Could not find any points that fitted the cylinder model." << std::endl;
//  // Copy all inliers of the model to another cloud.
//  else pcl::copyPointCloud<PointT>(*cloud, inlierIndices, *inlierPoints);
//    return cloud;
//}

//pcl::PointCloud<PointT>::Ptr curb_refine::voxelGridFiltering(pcl::PointCloud<PointT>::Ptr cloud){
//  pcl::VoxelGrid<PointT> sor;
//  pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
//  sor.setInputCloud (cloud);
//  sor.setLeafSize (0.01f, 0.01f, 0.01f);
//  sor.filter (*cloud_filtered);
//  return cloud_filtered;
//}


////In this function, will learn  segment arbitrary plane models from a given point cloud dataset.
//pcl::PointCloud<PointT>::Ptr curb_refine::planarSegmentation(pcl::PointCloud<PointT>::Ptr cloud){
//    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
//    pcl::ExtractIndices<PointT> extract;
//    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
//    // Create the segmentation object
//    pcl::SACSegmentation<PointT> seg;
//    // Optional
//    seg.setOptimizeCoefficients (true);
//    // Mandatory
//    seg.setModelType (pcl::SACMODEL_PLANE);
//    seg.setMethodType (pcl::SAC_RANSAC);
//    seg.setDistanceThreshold (0.01);

//    seg.setInputCloud (cloud);
//    seg.segment (*inliers, *coefficients);

//    pcl::PointCloud <PointT>::Ptr cloud1(new pcl::PointCloud<PointT>);

//    //cloud1->points.resize (inliers->indices.size ());
//    cloud1=extractInliers(cloud,inliers,false);
//    return cloud1;
//}

//void curb_refine::colorBasedClustering(pcl::PointCloud<PointT>::Ptr cloud){
//// kd-tree object for searches.
//  pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);
//  kdtree->setInputCloud(cloud);

//  // Color-based region growing clustering object.
//  pcl::RegionGrowingRGB<PointT> clustering;
//  clustering.setInputCloud(cloud);
//  clustering.setSearchMethod(kdtree);
//  // Here, the minimum cluster size affects also the postprocessing step:
//  // clusters smaller than this will be merged with their neighbors.
//  clustering.setMinClusterSize(100);
//  // Set the distance threshold, to know which points will be considered neighbors.
//  clustering.setDistanceThreshold(10);
//  // Color threshold for comparing the RGB color of two points.
//  clustering.setPointColorThreshold(6);
//  // Region color threshold for the postprocessing step: clusters with colors
//  // within the threshold will be merged in one.
//  clustering.setRegionColorThreshold(5);

//  std::vector <pcl::PointIndices> clusters;
//  clustering.extract(clusters);
//  //generateClusters(clusters,cloud,"colorBasedClustering");
// }
//void curb_refine::minCutBasedClustering(pcl::PointCloud<PointT>::Ptr cloud)
//{
//  // Min-cut clustering object.
//  pcl::MinCutSegmentation<PointT> clustering;
//  clustering.setInputCloud(cloud);
//  // Create a cloud that lists all the points that we know belong to the object
//  // (foreground points). We should set here the object's center.
//  pcl::PointCloud<PointT>::Ptr foregroundPoints(new pcl::PointCloud<PointT>());
//  PointT point;
//  point.x = 100.0;
//  point.y = 100.0;
//  point.z = 100.0;
//  foregroundPoints->points.push_back(point);
//  clustering.setForegroundPoints(foregroundPoints);
//  // Set sigma, which affects the smooth cost calculation. It should be
//  // set depending on the spacing between points in the cloud (resolution).
//  clustering.setSigma(0.05);
//  // Set the radius of the object we are looking for.
//  clustering.setRadius(0.20);
//  // Set the number of neighbors to look for. Increasing this also increases
//  // the number of edges the graph will have.
//  clustering.setNumberOfNeighbours(20);
//  // Set the foreground penalty. It is the weight of the edges
//  // that connect clouds points with the source vertex.
//  clustering.setSourceWeight(0.6);

//  std::vector <pcl::PointIndices> clusters;
//  clustering.extract(clusters);

// // std::cout << "Maximum flow is " << clustering.getMaxFlow() << "." << std::endl;
//  //generateClusters(clusters,cloud,"minCutBasedClustering");
//}
