#include "grid_map.h"

grid_map::grid_map(PointCloudXYZI::Ptr incloud, double mapRadius, int segmetNum, int binNum)
    :_grid_map_radius(mapRadius),_segment_num(segmetNum),_bin_num(binNum)
{
    _origin_cloud_ptr.reset(new PointCloudXYZI);
    *_origin_cloud_ptr=*incloud;
    _grid_map_vec.resize(_segment_num);
    _grid_lowest_points.resize(_segment_num);
    _grid_lowest_cloud.resize(_segment_num);
    _segment_curve_model.resize(_segment_num);

    for (int i = 0; i < _grid_map_vec.size(); ++i)
    {
        _grid_map_vec[i].resize(_bin_num);
        _grid_lowest_cloud[i].reset(new PointCloudXYZI);
    }
}
grid_map::grid_map(PointCloudXYZI::Ptr incloud)
{
    _origin_cloud_ptr.reset(new PointCloudXYZI);
    *_origin_cloud_ptr=*incloud;
}
void grid_map::generatePolarGridMap()
{
    for (int i = 0; i < _origin_cloud_ptr->points.size(); ++i)
    {
        //        pcl::PointXYZI point(_origin_cloud_ptr->points[i]);
        if (!pcl_isfinite(_origin_cloud_ptr->points[i].x) ||
                !pcl_isfinite(_origin_cloud_ptr->points[i].y) ||
                !pcl_isfinite(_origin_cloud_ptr->points[i].z))
        {
            continue;
        }

        // skip zero valued points
        if (_origin_cloud_ptr->points[i].x * _origin_cloud_ptr->points[i].x +_origin_cloud_ptr->points[i].y *_origin_cloud_ptr->points[i].y
                +_origin_cloud_ptr->points[i].z * _origin_cloud_ptr->points[i].z < 0.0001)
        {
            continue;
        }
        float ori = std::atan2(_origin_cloud_ptr->points[i].y,
                               _origin_cloud_ptr->points[i].x)*180/M_PI;
        if(ori<0)
        {
            ori+=360;
        }
        float resolution=(float)360/(_segment_num);
        int segment_index=(int)(ori/resolution);
        int bin_index=int(sqrt(pow(_origin_cloud_ptr->points[i].x,2)+pow(_origin_cloud_ptr->points[i].y,2))/
                          (_grid_map_radius/_bin_num));
        if(segment_index<_segment_num&&bin_index<_bin_num)
        {
            _grid_map_vec[segment_index][bin_index].push_back(_origin_cloud_ptr->points[i]);
        }
    }
    //    for (int i = 0; i < _grid_map_vec.size(); ++i)
    //    {
    //        for (int j = 0; j < _grid_map_vec[i].size(); ++j)
    //        {
    //            sort(_grid_map_vec[i][j].begin(), _grid_map_vec[i][j].end(), [](const pcl::PointXYZI &left, const pcl::PointXYZI &right)
    //            {
    //                return left.z < right.z;
    //            });
    //        }
    //    }
}
void grid_map::generateCartesianGrid( vector<vector<pcl::PointXYZI> >& grid_map_vec_carte)
{
    grid_map_vec_carte.resize(80);
    for(int i=0;i<_origin_cloud_ptr->points.size();++i)
    {

        int row=int(_origin_cloud_ptr->points[i].x/1+40);
        if(row>=0&&row<grid_map_vec_carte.size())
        {
            grid_map_vec_carte[row].push_back(_origin_cloud_ptr->points[i]);
        }
    }
}
void grid_map::distanceFilterByPolarGrid(PointCloudXYZI::Ptr outcloud)
{
    this->generatePolarGridMap();
    for (int i = 0; i < _grid_map_vec.size(); ++i)
    {
        for (int j = 0; j < _grid_map_vec[i].size(); ++j)
        {
            if(_grid_map_vec[i][j].size()>0)
            {
                for (int k = 0; k < _grid_map_vec[i][j].size(); ++k)
                {
                    outcloud->points.push_back(_grid_map_vec[i][j][k]);
                }
                break;
            }
        }
    }
}

void grid_map::distanceFilterByCartesianGrid(pcl::PointCloud<PointT>::Ptr outcloud,bool left)
{
    vector<vector<pcl::PointXYZI> > gridLeft;
    this->generateCartesianGrid(gridLeft);

    pcl::PointCloud<PointT>::Ptr Leftcloud(new pcl::PointCloud<PointT>);
    for(int i=0;i<gridLeft.size();++i)
    {

        if (gridLeft[i].size() > 0)
        {
            int min_y_Left = 0;

            for (size_t k = 0; k < gridLeft[i].size(); ++k)
            {
                if(left)
                {
                    if(gridLeft[i][min_y_Left].y> gridLeft[i][k].y)
                    {
                        min_y_Left = k;
                    }
                }
                else
                {
                    if(gridLeft[i][min_y_Left].y< gridLeft[i][k].y)
                    {
                        min_y_Left = k;
                    }
                }

            }
            outcloud->points.push_back(gridLeft[i][min_y_Left]);
        }
    }
}
void grid_map::computeLowestPoints()
{
    for (int i = 0; i < _segment_num; ++i)
    {
        for (int j = 0; j < _bin_num; ++j)
        {
            if(_grid_map_vec[i][j].size()<2)
            {
                continue;
            }

            if(abs(_grid_map_vec[i][j][1].z-_grid_map_vec[i][j][0].z)>0.2)
            {
                continue;
            }
            int lowest_temp=0;
            //            for (int k = 0; k < _grid_map_vec[i][j].size(); ++k)
            //            {
            //                if(_grid_map_vec[i][j][k].z<_grid_map_vec[i][j][lowest_temp].z)
            //                {
            //                    lowest_temp=k;
            //                }
            //            }

            if(j>0&&_grid_map_vec[i][j-1].size()>0&&abs(_grid_map_vec[i][j][0].z-_grid_map_vec[i][j-1][0].z)>0.2)
            {
                continue;
            }
            if(i>0&&_grid_map_vec[i-1][j].size()>0&&abs(_grid_map_vec[i][j][0].z-_grid_map_vec[i-1][j][0].z)>0.2)
            {
                continue;
            }
            if(_grid_lowest_cloud[i]->points.size()>=1)
            {
                point2D pointTemp(_grid_map_vec[i][j][lowest_temp]);
                pcl::PointXYZI point1;
                pcl::PointXYZI point2;
                point1.x=pointTemp.r;
                point1.z=pointTemp.z;
                point1.y=1;
                point2=_grid_lowest_cloud[i]->back();
                float slope=atan((point1.z-point2.z)/(point1.x-point2.x))*180/M_PI;
                if(abs(slope)<10)
                {
                    _grid_lowest_points[i].push_back(pointTemp);
                    _grid_lowest_cloud[i]->points.push_back(point1);
                }
            }
            else
            {
                point2D pointTemp(_grid_map_vec[i][j][lowest_temp]);
                pcl::PointXYZI point;
                point.x=pointTemp.r;
                point.z=pointTemp.z;
                point.y=0;
                point.intensity=1;
                _grid_lowest_cloud[i]->points.push_back(point);
                _grid_lowest_points[i].push_back(pointTemp);
            }


        }
    }
}


void grid_map::computeSegmentCurve()
{
    for (int i = 0; i < _segment_num; ++i)
    {
        //       this->ransac_curve(_grid_lowest_points[i],0.2,_segment_curve_model[i]);
        //        if(_grid_lowest_cloud[i]->points.size()>2)
        //        {
        this->ransac_line(_grid_lowest_cloud[i],0.2,_segment_curve_model[i]);
        //        }
        cout<<"segment "<<i<<" model coefficient is "<<_segment_curve_model[i]<<endl;

    }
    //     cout<<"segment 0 model coefficient is "<<_segment_curve_model[0]<<endl;
}
void grid_map::ransac_curve(vector<point2D> incloud,double residualThreshold, Vector3d& model)
{
    srand(time(NULL)) ; //time(NULL)获取当前时间,返回一个长整形数值

    //-------------------------------------------------------------- make sample data

    vector<double> x(incloud.size());
    vector<double> y(incloud.size()) ;
    int nData=x.size();
    for( int i=0 ; i<x.size() ; i++ )
    {
        x[i] = incloud[i].r;
        y[i] = incloud[i].z;
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
    int N = 80;	//iterations
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

        if( cnt > max_cnt )  //如果内点数量大于0,则作为best model
        {
            best_model = X ;
            max_cnt = cnt ;
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
    model(0)=best_model.at<double>(0);
    model(1)=best_model.at<double>(1);
    model(2)=best_model.at<double>(2);
    //    cout<<"curb line model coefficient is "<<X.at<double>(0)<<" "<<X.at<double>(1)<<" "<<X.at<double>(2)<<endl;
}
void grid_map::ransac_line(PointCloudXYZI::Ptr incloud,double residualThreshold, Vector3d& model)
{
    pcl::PointIndices indices;
    pcl::ModelCoefficients modelCoe;
    //    PointCloudXYZI::Ptr cloud(new PointCloudXYZI);
    //    *cloud=*incloud;
    Vector3f axis(1,0,0);
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    seg.setOptimizeCoefficients(false);
    seg.setAxis(axis);
    seg.setEpsAngle(0.1);
    seg.setMaxIterations(50);
    seg.setModelType(pcl::SACMODEL_LINE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(residualThreshold);
    seg.setInputCloud(incloud);
    seg.segment(indices,modelCoe);
    model(0)=0;
    model(1)=modelCoe.values[5]/modelCoe.values[3];
    model(2)=modelCoe.values[2]-modelCoe.values[5]/modelCoe.values[3]*modelCoe.values[0];
}
void grid_map::getGroundPoints(PointCloudXYZI::Ptr outcloud)
{
    generatePolarGridMap();
    computeLowestPoints();
    computeSegmentCurve();
    for (int i = 0; i < _segment_num; ++i)
    {
        for (int j = 0; j < _bin_num; ++j)
        {
            int lowest_temp=0;
            //            float bin_range=i+0.5;
            //            float bin_height=pow(bin_range,2)*_segment_curve_model[i](0)+bin_range*_segment_curve_model[i](1)+
            //                    _segment_curve_model[i](2);

            for (int k = 0; k < _grid_map_vec[i][j].size(); ++k)
            {
                float point_range=sqrt(pow(_grid_map_vec[i][j][k].x,2)+pow(_grid_map_vec[i][j][k].y,2));
                float point_height=pow(point_range,2)*_segment_curve_model[i](0)+point_range*_segment_curve_model[i](1)+
                        _segment_curve_model[i](2);

                if(abs(_grid_map_vec[i][j][k].z-point_height)<_ground_threshold)
                {
                    outcloud->points.push_back(_grid_map_vec[i][j][k]);
                }
            }
        }
    }
}
