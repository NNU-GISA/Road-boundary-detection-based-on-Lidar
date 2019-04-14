#include "curb_refine_ransac.h"

curb_refine_ransac::curb_refine_ransac(pcl::PointCloud<PointT>& incloud, double varthreshold, double fthreshold)
    :_cloud(incloud),_varThreshold(varthreshold),_fThreshold(fthreshold)
{

}
void curb_refine_ransac::distanceFilterByLaserLeft(pcl::PointCloud<PointT>::Ptr incloud, pcl::PointCloud<PointT>::Ptr outcloud)
{
    pcl::PointCloud<PointT>::Ptr completeCloudLeftMapped(new pcl::PointCloud<PointT>);
    scanIndices scanIDindicesLeft;
    cloud_mapper mapperLeft;
    mapperLeft.processByIntensity(incloud,completeCloudLeftMapped,scanIDindicesLeft);
    pcl::PointCloud<PointT>::Ptr Leftcloud(new pcl::PointCloud<PointT>);
    // 从每个scan中提取curb点
    size_t nScansLeft = scanIDindicesLeft.size();
    //    *clusterCloud[0]=_completeCloudFiltered;
    for (size_t i = 0; i < nScansLeft; i++)
    {
        size_t scanStartIdx = scanIDindicesLeft[i].first;
        size_t scanEndIdx = scanIDindicesLeft[i].second;

        vector<pcl::PointXYZI> LeftFront;
        vector<pcl::PointXYZI> LeftRear;

        for (size_t j= scanStartIdx; j <= scanEndIdx; ++j)
        {
            if (completeCloudLeftMapped->points[j].x >= 0)
            {
                LeftFront.push_back(completeCloudLeftMapped->points[j]);
            }
            else
            {
                LeftRear.push_back(completeCloudLeftMapped->points[j]);
            }
        }
        //针对每个scan每个象限分别提取y值绝对值最小的点
        //左前方
        if (LeftFront.size() > 0)
        {
            vector<pcl::PointXYZI> yNeg;
            //            vector<pcl::PointXYZI> yPos;
            for (size_t k = 0; k < LeftFront.size(); ++k)
            {

                if(LeftFront[k].y<0)
                {
                    yNeg.push_back(LeftFront[k]);
                }
            }

            if(yNeg.size()>0)
            {
                int max_y_LeftFront = 0;
                for (size_t k = 0; k < yNeg.size(); ++k)
                {

                    if(abs(yNeg[max_y_LeftFront].y)< abs(yNeg[k].y))
                    {
                        max_y_LeftFront = k;
                    }
                }
                Leftcloud->points.push_back(yNeg[max_y_LeftFront]);
            }
            else
            {

                int min_y_LeftFront = 0;
                for (size_t k = 0; k < LeftFront.size(); ++k)
                {

                    if(abs(LeftFront[min_y_LeftFront].y)> abs(LeftFront[k].y))
                    {
                        min_y_LeftFront = k;
                    }
                }
                Leftcloud->points.push_back(LeftFront[min_y_LeftFront]);
            }
        }
        //左后方
        if (LeftRear.size() > 0)
        {
            vector<pcl::PointXYZI> yNeg;
            for (size_t k = 0; k < LeftRear.size(); ++k)
            {

                if(LeftRear[k].y<0)
                {
                    yNeg.push_back(LeftRear[k]);
                }
            }
            if(yNeg.size()>0)
            {
                int min_y_LeftRear = 0;
                for (size_t k = 0; k < yNeg.size(); ++k)
                {

                    if(abs(yNeg[min_y_LeftRear].y)< abs(yNeg[k].y))
                    {
                        min_y_LeftRear = k;
                    }
                }
                Leftcloud->points.push_back(yNeg[min_y_LeftRear]);
            }
            else
            {
                int min_y_LeftRear = 0;
                for (size_t k = 0; k < LeftRear.size(); ++k)
                {

                    if(abs(LeftRear[min_y_LeftRear].y)> abs(LeftRear[k].y))
                    {
                        min_y_LeftRear = k;
                    }
                }
                Leftcloud->points.push_back(LeftRear[min_y_LeftRear]);
            }

        }

    }
    *outcloud=*Leftcloud;
}
void curb_refine_ransac::distanceFilterByLaserRight(pcl::PointCloud<PointT>::Ptr incloud, pcl::PointCloud<PointT>::Ptr outcloud)
{
    pcl::PointCloud<PointT>::Ptr completeCloudLeftMapped(new pcl::PointCloud<PointT>);
    scanIndices scanIDindicesLeft;
    cloud_mapper mapperLeft;
    mapperLeft.processByIntensity(incloud,completeCloudLeftMapped,scanIDindicesLeft);
    pcl::PointCloud<PointT>::Ptr Leftcloud(new pcl::PointCloud<PointT>);
    // 从每个scan中提取curb点
    size_t nScansLeft = scanIDindicesLeft.size();
    //    *clusterCloud[0]=_completeCloudFiltered;
    for (size_t i = 0; i < nScansLeft; i++)
    {
        size_t scanStartIdx = scanIDindicesLeft[i].first;
        size_t scanEndIdx = scanIDindicesLeft[i].second;

        vector<pcl::PointXYZI> LeftFront;
        vector<pcl::PointXYZI> LeftRear;

        for (size_t j= scanStartIdx; j <= scanEndIdx; ++j)
        {
            if (completeCloudLeftMapped->points[j].x >= 0)
            {
                LeftFront.push_back(completeCloudLeftMapped->points[j]);
            }
            else
            {
                LeftRear.push_back(completeCloudLeftMapped->points[j]);
            }
        }
        //针对每个scan每个象限分别提取y值绝对值最小的点
        //左前方
        if (LeftFront.size() > 0)
        {
            vector<pcl::PointXYZI> yPos;
            for (size_t k = 0; k < LeftFront.size(); ++k)
            {

                if(LeftFront[k].y>0)
                {
                    yPos.push_back(LeftFront[k]);
                }
            }

            if(yPos.size()>0)
            {
                int max_y_LeftFront = 0;
                for (size_t k = 0; k < yPos.size(); ++k)
                {

                    if(abs(yPos[max_y_LeftFront].y)< abs(yPos[k].y))
                    {
                        max_y_LeftFront = k;
                    }
                }
                Leftcloud->points.push_back(yPos[max_y_LeftFront]);
            }
            else
            {

                int min_y_LeftFront = 0;
                for (size_t k = 0; k < LeftFront.size(); ++k)
                {

                    if(abs(LeftFront[min_y_LeftFront].y)> abs(LeftFront[k].y))
                    {
                        min_y_LeftFront = k;
                    }
                }
                Leftcloud->points.push_back(LeftFront[min_y_LeftFront]);
            }
        }
        //左后方
        if (LeftRear.size() > 0)
        {
            vector<pcl::PointXYZI> yPos;
            for (size_t k = 0; k < LeftRear.size(); ++k)
            {

                if(LeftRear[k].y>0)
                {
                    yPos.push_back(LeftRear[k]);
                }
            }
            if(yPos.size()>0)
            {
                int min_y_LeftRear = 0;
                for (size_t k = 0; k < yPos.size(); ++k)
                {

                    if(abs(yPos[min_y_LeftRear].y)< abs(yPos[k].y))
                    {
                        min_y_LeftRear = k;
                    }
                }
                Leftcloud->points.push_back(yPos[min_y_LeftRear]);
            }
            else
            {
                int min_y_LeftRear = 0;
                for (size_t k = 0; k < LeftRear.size(); ++k)
                {

                    if(abs(LeftRear[min_y_LeftRear].y)> abs(LeftRear[k].y))
                    {
                        min_y_LeftRear = k;
                    }
                }
                Leftcloud->points.push_back(LeftRear[min_y_LeftRear]);
            }

        }

    }
    *outcloud=*Leftcloud;
}

void curb_refine_ransac::extractPointCloud(pcl::PointCloud<PointT>& incloud, pcl::PointIndicesPtr indices,pcl::PointCloud<PointT>& outcloud)
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

void curb_refine_ransac::lineFitRansac(pcl::PointCloud<PointT>& incloud, pcl::PointIndices& indices)
{
    pcl::ModelCoefficients _model;
    pcl::PointIndices _indices;
    Vector3f axis(1,0,0);
    //    pcl::ModelCoefficients model;
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PARALLEL_LINE);
    seg.setAxis(axis);
    seg.setEpsAngle(0.2);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.1);
    seg.setInputCloud(boost::make_shared<pcl::PointCloud<PointT> >(incloud));
    seg.segment(_indices,_model);
    indices=_indices;
}

void curb_refine_ransac::statisticalFilter_indces(pcl::PointCloud<PointT> incloud,int meanK,pcl::PointIndicesPtr pointindices,double stdThreshold)
{
    vector<int> indices;
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud (boost::make_shared<pcl::PointCloud<PointT> >(incloud));
    sor.setMeanK (meanK);
    sor.setStddevMulThresh (stdThreshold);
    sor.setKeepOrganized(false);
    //boost::make_shared<pcl::PointIndices>(_pointIndices)= sor.getRemovedIndices();
    sor.filter (indices);
    //sor.filter(_indices);
    for (int i = 0; i <indices.size(); ++i)
    {
        pointindices->indices.push_back(indices[i]);
    }
}

void curb_refine_ransac::pointcloud_projection(pcl::PointCloud<PointT>::Ptr incloud,pcl::PointCloud<PointT>& outcloud)
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
void curb_refine_ransac::ransac_curve(pcl::PointCloud<pcl::PointXYZI>::Ptr incloud,double residualThreshold, pcl::PointCloud<pcl::PointXYZI>::Ptr outcloud)
{
    srand(time(NULL)) ; //time(NULL)获取当前时间,返回一个长整形数值

    //-------------------------------------------------------------- make sample data

    vector<double> x(incloud->points.size());
    vector<double> y(incloud->points.size()) ;
    int nData=x.size();
    for( int i=0 ; i<x.size() ; i++ )
    {
        x[i] = incloud->points[i].x;
        y[i] = incloud->points[i].y;
    }
    //-------------------------------------------------------------- build matrix
    cv::Mat A(nData, 3, CV_64FC1) ;
    cv::Mat B(nData,1, CV_64FC1) ;

    for( int i=0 ; i<nData ; i++ )
    {
        A.at<double>(i,0) = x[i] * x[i] ;
    }
    for( int i=0 ; i<nData ; i++ )
    {
        A.at<double>(i,1) = x[i] ;
    }
    for( int i=0 ; i<nData ; i++ )
    {
        A.at<double>(i,2) = 1.0 ;
    }

    for( int i=0 ; i<nData ; i++ )
    {
        B.at<double>(i,0) = y[i] ;
    }

    //-------------------------------------------------------------- RANSAC fitting
    //    int n_data = 100 ;
    int N = 300;	//iterations
    double T = residualThreshold;   // residual threshold，拟合残差阈值

    int n_sample = 3;
    int max_cnt = 0;
    cv::Mat best_model(3,1,CV_64FC1) ;

    for( int i=0 ; i<N ; i++ )
    {
        //random sampling - 3 point
        int k[3] = {-1, } ;
        k[0] = floor((rand()%nData+1))+1;

        do
        {
            k[1] = floor((rand()%nData+1))+1;
        }while(k[1]==k[0] || k[1]<0) ;

        do
        {
            k[2] = floor((rand()%nData+1))+1;
        }while(k[2]==k[0] || k[2]==k[1] || k[2]<0) ;

        //        printf("random sample : %d %d %d\n", k[0], k[1], k[2]) ;

        //model estimation
        cv::Mat AA(3,3,CV_64FC1) ;
        cv::Mat BB(3,1, CV_64FC1) ;
        for( int j=0 ; j<3 ; j++ )
        {
            AA.at<double>(j,0) = x[k[j]] * x[k[j]] ;
            AA.at<double>(j,1) = x[k[j]] ;
            AA.at<double>(j,2) = 1.0 ;

            BB.at<double>(j,0) = y[k[j]] ;
        }

        cv::Mat AA_pinv(3,3,CV_64FC1) ;
        invert(AA, AA_pinv, cv::DECOMP_SVD);//求AA逆矩阵

        cv::Mat X = AA_pinv * BB ;

        //evaluation
        if (X.at<double>(0)<0.1)
        {
            cv::Mat residual(nData,1,CV_64FC1) ;
            residual = cv::abs(B-A*X) ;
            int cnt = 0 ;//内点计数
            for( int j=0 ; j<nData ; j++ )
            {
                double data = residual.at<double>(j,0) ;

                if( data < T )
                {
                    cnt++ ;
                }
            }

            if( cnt > max_cnt)  //如果内点数量大于0,则作为best model
            {
                best_model = X ;
                max_cnt = cnt ;
            }
        }

    }

    //------------------------------------------------------------------- optional LS fitting
    cv::Mat residual = cv::abs(A*best_model - B) ;
    std::vector<int> vec_index ;  //存储模型内点索引
    for( int i=0 ; i<nData ; i++ )
    {
        double data = residual.at<double>(i, 0) ;
        if( data < T )   //如果残差小于阈值则作为内点
        {
            vec_index.push_back(i) ;
        }
    }

    cv::Mat A2(vec_index.size(),3, CV_64FC1) ;//存储所有内点
    cv::Mat B2(vec_index.size(),1, CV_64FC1) ;

    for( int i=0 ; i<vec_index.size() ; i++ )
    {
        A2.at<double>(i,0) = x[vec_index[i]] * x[vec_index[i]]  ;
        A2.at<double>(i,1) = x[vec_index[i]] ;
        A2.at<double>(i,2) = 1.0 ;

        B2.at<double>(i,0) = y[vec_index[i]] ;
    }

    cv::Mat A2_pinv(3,vec_index.size(),CV_64FC1) ;
    invert(A2, A2_pinv, cv::DECOMP_SVD);

    cv::Mat X = A2_pinv * B2 ;//利用所有内点再次进行拟合得到优化后的模型系数

    cv::Mat residual_opt = cv::abs(A*X - B) ;
    std::vector<int> vec_index_opt ;  //存储模型内点索引
    for( int i=0 ; i<nData ; i++ )
    {
        double data = residual_opt.at<double>(i, 0) ;
        if( data < T )   //如果残差小于阈值则作为内点
        {
            outcloud->points.push_back(incloud->points[i]);
        }
    }
    cout<<"curb line model coefficient is "<<X.at<double>(0)<<" "<<X.at<double>(1)<<" "<<X.at<double>(2)<<endl;
}
void curb_refine_ransac::process(pcl::PointCloud<PointT>::Ptr obstacleCloud,vector<pcl::PointCloud<PointT>::Ptr> clusterCloud,
                                 vector<pcl::PointXYZ>& nearestpoints,vector<double>& distance_vec,
                                 vector<double>& distance_vec_filtered)
{


    pcl::PointCloud<PointT>::Ptr cloud2D(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloudFilted(new pcl::PointCloud<PointT>);
    pcl::PointIndices::Ptr cloudIndices(new pcl::PointIndices);
    pcl::PointCloud<PointT>::Ptr completeCloudLeft2D(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr completeCloudRight2D(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr completeCloudLeftFiltered(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr completeCloudRightFiltered(new pcl::PointCloud<PointT>);
    pcl::PointIndices::Ptr leftIndices(new pcl::PointIndices);
    pcl::PointIndices::Ptr rightIndices(new pcl::PointIndices);

    //    pointcloud_projection(boost::make_shared<pcl::PointCloud<PointT> >(_cloud),*cloud2D);
    //    statisticalFilter_indces(_cloud,100,cloudIndices,200);
    //    extractPointCloud( _cloud,cloudIndices,*cloudFilted);
    //    cout<<"滤波后特征点为 "<<cloudFilted->points.size()<<endl;
    //密度最大值聚类
    //            CloudPtrList clusterPtrLR(2);
    //            for(int i=0;i<clusterPtrLR.size();++i)
    //            {

    //                clusterPtrLR[i]=boost::make_shared<pcl::PointCloud<PointT> >();
    //            }
    //            max_density_cluster my_cluster(cloudFilted,_neighborRate);
    //            my_cluster.process(cloudFilted,clusterPtrLR);

    //密度最大聚类改进
    //    my_density_cluster my_cluster(cloudFilted,_neighborRate,_min_pets,_dbR);
    //    CloudPtrList clusterPtrLR(2);
    //    for(int i=0;i<clusterPtrLR.size();++i)
    //    {

    //        clusterPtrLR[i]=boost::make_shared<pcl::PointCloud<PointT> >();
    //    }
    //    my_cluster.processbyY(cloudFilted,clusterPtrLR);
    //左右点分类
    CloudPtrList clusterPtrLR(2);
    for(int i=0;i<clusterPtrLR.size();++i)
    {

        clusterPtrLR[i]=boost::make_shared<pcl::PointCloud<PointT> >();
    }
//    for (int i = 0; i < _cloud.size(); ++i)
//    {
//        if(_cloud[i].y>0)
//        {
//            clusterPtrLR[0]->points.push_back(_cloud[i]);
//        }
//        else
//        {
//            clusterPtrLR[1]->points.push_back(_cloud[i]);
//        }
//    }
    //移除车顶干扰点
    pcl::PointCloud<PointT>::Ptr obstacleCloudFiltered(new pcl::PointCloud<PointT>);
    for (int i = 0; i < obstacleCloud->points.size(); ++i)
    {
        if ((pow(obstacleCloud->points[i].x,2)+pow(obstacleCloud->points[i].y,2))>=7)
        {
            obstacleCloudFiltered->points.push_back(obstacleCloud->points[i]);
        }
    }
    road_segmentation mycluster(obstacleCloudFiltered);
    mycluster.process(_cloud.makeShared(),clusterPtrLR);
    nearestpoints=mycluster._nearest_points;
    distance_vec=mycluster._distance_vec_;
    distance_vec_filtered=mycluster._distance_vec_filtered;
    cout<<"分类后左边点为 "<<clusterPtrLR[0]->points.size()<<endl;
    cout<<"分类后右边点为 "<<clusterPtrLR[1]->points.size()<<endl;

//        pointcloud_projection(clusterPtrLR[0],*completeCloudLeft2D);
//        statisticalFilter_indces(*completeCloudLeft2D,200,leftIndices,3);
//        extractPointCloud( *clusterPtrLR[0],leftIndices,*completeCloudLeftFiltered);
//        *clusterPtrLR[0]=*completeCloudLeftFiltered;
//        pointcloud_projection(clusterPtrLR[1],*completeCloudRight2D);
//        statisticalFilter_indces(*completeCloudRight2D,100,rightIndices,3);
//        extractPointCloud( *clusterPtrLR[1],rightIndices,*completeCloudRightFiltered);
//        *clusterPtrLR[1]=*completeCloudRightFiltered;
    pcl::PointCloud<PointT>::Ptr pointcloud_distancefiltered(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr pointcloud_distanceleftfiltered(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr pointcloud_distancerightfiltered(new pcl::PointCloud<PointT>);
    grid_map my_mapL(clusterPtrLR[0]);
    my_mapL.distanceFilterByCartesianGrid(pointcloud_distanceleftfiltered,true);

    for (int i = 0; i < pointcloud_distanceleftfiltered->points.size(); ++i)
    {
        pointcloud_distanceleftfiltered->points[i].intensity=i;
    }
    grid_map my_mapR(clusterPtrLR[1]);
    my_mapR.distanceFilterByCartesianGrid(pointcloud_distancerightfiltered,false);
    for (int i = 0; i < pointcloud_distancerightfiltered->points.size(); ++i)
    {
        pointcloud_distancerightfiltered->points[i].intensity=i;
    }
    *pointcloud_distancefiltered+=*pointcloud_distanceleftfiltered;
    *pointcloud_distancefiltered+=*pointcloud_distancerightfiltered;
    //为242滤除路内障碍点
    //     pcl::PointCloud<PointT>::Ptr temp(new pcl::PointCloud<PointT>);
    //    for (int i = 0; i < clusterPtrLR[0]->points.size(); ++i)
    //    {
    //        if(clusterPtrLR[0]->points[i].y>3)
    //        {
    //            temp->points.push_back(clusterPtrLR[0]->points[i]);
    //        }
    //    }
    //    *clusterPtrLR[0]=*temp;


    //    pcl::PointCloud<PointT>::Ptr Leftcloud(new pcl::PointCloud<PointT>);
    //    pcl::PointCloud<PointT>::Ptr Rightcloud(new pcl::PointCloud<PointT>);
//    pcl::PointCloud<PointT>::Ptr Leftcloud_initial(new pcl::PointCloud<PointT>);
//    pcl::PointCloud<PointT>::Ptr Rightcloud_initial(new pcl::PointCloud<PointT>);
    //    distanceFilterByLaserLeft(clusterPtrLR[0],Leftcloud);
    //    distanceFilterByLaserRight(clusterPtrLR[1],Rightcloud);
//    for (int i = 0; i < pointcloud_distancefiltered->points.size(); ++i)
//    {
//        pcl::PointXYZI point(pointcloud_distancefiltered->points[i]);
//        point.intensity=i;
//        if(point.y>0&&abs(point.x)<6)
//        {
//            Leftcloud_initial->points.push_back(point);
//        }
//        if(point.y<=0&&abs(point.x)<6)
//        {
//            Rightcloud_initial->points.push_back(point);
//        }
//    }



    //    cout<<"左边最近边界点 "<<Leftcloud->points.size()<<endl;
    //    cout<<"右边最近边界点 "<<Rightcloud->points.size()<<endl;
    //    for (int i = 0; i < Leftcloud->points.size(); ++i)
    //    {
    //        if(abs(Leftcloud->points[i].x)<1)
    //        {
    //            Leftcloud->points[i]=Leftcloud->points[15];
    //        }
    //    }

    //242反例Zhang
    //    sort(Rightcloud->points.begin(),Rightcloud->points.end(),[](pcl::PointXYZI& left,pcl::PointXYZI& right)
    //    {
    //        return left.x<right.x;
    //    });
    //    pcl::PointXYZI point=Rightcloud->points[20];
    //    for (int i = 0; i < 9; ++i)
    //    {
    //        Leftcloud->points.push_back(Rightcloud->points[i]);
    //        Leftcloud->points.push_back(Rightcloud->points[Rightcloud->points.size()-1-i]);
    //        Rightcloud->points[i]=point;
    //        Rightcloud->points[Rightcloud->points.size()-1-i]=point;
    //    }



    //将道路最近边界点分成前后两段用于拟合

        pcl::PointCloud<PointT>::Ptr LeftFrontcloud(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr LeftRearcloud(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr RightFrontcloud(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr RightRearcloud(new pcl::PointCloud<PointT>);

        for (int i = 0; i < pointcloud_distanceleftfiltered->points.size(); ++i)
        {
            if(pointcloud_distanceleftfiltered->points[i].x>=0)
            {
                LeftFrontcloud->points.push_back(pointcloud_distanceleftfiltered->points[i]);
            }
            else
            {
                LeftRearcloud->points.push_back(pointcloud_distanceleftfiltered->points[i]);
            }
        }
        for (int i = 0; i < pointcloud_distancerightfiltered->points.size(); ++i)
        {
            if(pointcloud_distancerightfiltered->points[i].x>=0)
            {
                RightFrontcloud->points.push_back(pointcloud_distancerightfiltered->points[i]);
            }
            else
            {
                RightRearcloud->points.push_back(pointcloud_distancerightfiltered->points[i]);
            }
        }

        pcl::PointCloud<PointT>::Ptr LeftFront_filtered(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr LeftRear_filtered(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr RightFront_filtered(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr RightRear_filtered(new pcl::PointCloud<PointT>);

//        ransac_curve(LeftFrontcloud,0.35,LeftFront_filtered);
//        ransac_curve(LeftRearcloud,0.35,LeftRear_filtered);
//        ransac_curve(RightFrontcloud,0.35,RightFront_filtered);
//        ransac_curve(RightRearcloud,0.35,RightRear_filtered);

//            *clusterCloud[0]=*LeftRear_filtered;
//            *clusterCloud[0]+=*LeftFront_filtered;
//            *clusterCloud[1]=*RightRear_filtered;
//            *clusterCloud[1]+=*RightFront_filtered;

    pcl::PointCloud<PointT>::Ptr Leftcloud_initial(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr Rightcloud_initial(new pcl::PointCloud<PointT>);
//    lineFitRansac(*Leftcloud_initial,_indicesLeft);
//    extractPointCloud(*Leftcloud_initial,boost::make_shared<pcl::PointIndices>(_indicesLeft),*Leftcloud_initial_new);
//    lineFitRansac(*Rightcloud_initial,_indicesRight);
//    extractPointCloud(*Rightcloud_initial,boost::make_shared<pcl::PointIndices>(_indicesRight),*Rightcloud_initial_new);


    ransac_curve(pointcloud_distanceleftfiltered,0.25,Leftcloud_initial);
    ransac_curve(pointcloud_distancerightfiltered,0.25,Rightcloud_initial);
    gaussian_process my_gauLeft(pointcloud_distanceleftfiltered,Leftcloud_initial,
                            _varThreshold,_fThreshold);
    my_gauLeft.process(clusterCloud[0]);
    gaussian_process my_gauRight(pointcloud_distancerightfiltered,Rightcloud_initial,
                            _varThreshold,_fThreshold);
    my_gauRight.process(clusterCloud[1]);


//    *clusterCloud[0]=*Leftcloud_initial;
//    *clusterCloud[1]=*Rightcloud_initial;
//*clusterCloud[0]=*clusterPtrLR[0];
//    *clusterCloud[1]=*clusterPtrLR[1];
    cout<<"before gaussian left point is "<<Leftcloud_initial->points.size()<<endl;
    cout<<"after gaussian left point is "<<clusterCloud[0]->points.size()<<endl;
    cout<<"before gaussian right point is "<<Rightcloud_initial->points.size()<<endl;
    cout<<"after gaussian right point is "<<clusterCloud[1]->points.size()<<endl;
    //    *clusterCloud[0]=_cloud;
//        *clusterCloud[0]=*pointcloud_distanceleftfiltered;
//        *clusterCloud[1]=*pointcloud_distancerightfiltered;
    //    pcl::PointCloud<PointT>::Ptr distance_filtered(new pcl::PointCloud<PointT>);
    //    *distance_filtered+=*Leftcloud;
    //    *distance_filtered+=*Rightcloud;
//           for (int i = 0; i < pointcloud_distancefiltered->points.size(); ++i)
//           {
//               if(pointcloud_distancefiltered->points[i].y>0)
//               {
//                   clusterCloud[0]->points.push_back(pointcloud_distancefiltered->points[i]);
//               }
//               else
//               {
//                   clusterCloud[1]->points.push_back(pointcloud_distancefiltered->points[i]);
//               }

//           }
//    *clusterCloud[0]=*pointcloud_distancefiltered;
    //                *clusterCloud[1]=*Rightcloud;
    //    *clusterCloud[0]=*cloudFilted;
//    cout<<"左边最终边界点 "<<clusterCloud[0]->points.size()<<endl;
//    cout<<"右边最终边界点 "<<clusterCloud[1]->points.size()<<endl;

}
