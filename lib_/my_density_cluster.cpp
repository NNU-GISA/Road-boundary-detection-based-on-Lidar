#include "my_density_cluster.h"

my_density_cluster::my_density_cluster(PointCloud incloud, double neighborRate, int min_pets, double dbr)
    :_neighborRate(neighborRate),_min_pets(min_pets),_dbR(dbr)
{
    //    _CompleteCloud=boost::make_shared<pcl::PointCloud<PointT> >();
    _CoreCloud=boost::make_shared<pcl::PointCloud<PointT> >();
    //    _CompleteCloud=incloud;
    _allcloudIndex.resize(incloud->points.size());
#pragma omp parallel for  schedule(runtime)
    for(int i=0;i<_allcloudIndex.size();++i)
    {
        _allcloudIndex[i].pointIndex=i;
        _allcloudIndex[i].pointtype=1;
        _allcloudIndex[i].visited=0;
        _allcloudIndex[i].cluster=0;
    }
    this->getdc(incloud);
        _dc=2.1875;
    this->getLocalDensity(incloud);
    this->getDistanceToHigherDensityByOrder(incloud);
    this->findClusterCenters(incloud);
}
double getDistance(PointT &pt1, PointT &pt2)
{
    double tmp = 0*pow(pt1.x - pt2.x, 2) + pow(pt1.y - pt2.y, 2);
    return pow(tmp, 0.5);
}
double getDistanceY(PointT &pt1, PointT &pt2)
{
    double tmp = pow(pt1.y - pt2.y, 2);
    return pow(tmp, 0.5);
}

void my_density_cluster::getdc(PointCloud incloud)//截断距离
{
    double t1,t2,t3,t4;
    t1 = omp_get_wtime();
    int row = incloud->points.size();
    //    double avgNeighbourNum = row * _neighborRateLow;
    int disNum=row*(row-1)/2;
    _dc = 0.0;

    vector<double> dis;
    int coreNum = omp_get_num_procs();//获得处理器个数
    vector<vector<double> > dis_procs(coreNum);
#pragma omp parallel
    {
#pragma omp for schedule(runtime)
        for(int i=0;i<row;++i)
        {
            for(int j=i+1;j<row;++j)
            {
                int k=omp_get_thread_num();
                double d = 0.0;
                d=getDistance(incloud->points[i], incloud->points[j]);
                dis_procs[k].push_back(d);
                //                            dis.push_back(d);
            }
        }
    }
    t2=omp_get_wtime();
    dis=dis_procs[0];
    for(int i=1;i<coreNum;++i)
    {
        dis.insert(dis.end(),dis_procs[i].begin(),dis_procs[i].end());
    }
    int pos=int(disNum*_neighborRate);
    nth_element(dis.begin(),dis.begin()+pos-1,dis.end());
    //    sort(dis.begin(), dis.end());
    //    _dc = dis[int(avgNeighbourNum * row)];
    _dc=dis[pos-1];
    t3=omp_get_wtime();
    cout<<"截断距离为 "<<_dc<<endl;
    cout << "compute for loop took me " << (t2-t1) << " seconds" << endl;
    cout << "compute sort took me " << (t3-t2) << " seconds" << endl;
}

void my_density_cluster::getLocalDensity(PointCloud incloud)  //计算局部密度
{
    double t1,t2;
    t1=omp_get_wtime();
    double resolution=0.05;
    resolution=0.05;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> octree(resolution);//初始化octree
    octree.setInputCloud(incloud);
    octree.addPointsFromInputCloud();
    int nSamples = incloud->points.size();

    _densityLocal.resize(nSamples, 0);

    int coreNum = omp_get_num_procs();
    vector<vector<pointPty> > corecloudIndexMultiCore(coreNum);
    vector<pcl::PointCloud<pcl::PointXYZI> > CoreCloudMutiCore(coreNum);
    #pragma omp parallel for  schedule(runtime)
    for(int i = 0; i < nSamples - 1; i++)
    {
        vector<int> radiussearch;//存放点的索引
        vector<float> radiusdistance;//存放点的距离平方
        vector<int> radiussearch1;//存放点的索引
        vector<float> radiusdistance1;//存放点的距离平方
        octree.radiusSearch(incloud->points[i], _dc, radiussearch, radiusdistance);//八叉树的邻域搜索
        octree.radiusSearch(incloud->points[i], _dbR, radiussearch1, radiusdistance1);
        //                for(int j=i+1;j<nSamples;++j)
        //                {
        //                    double dis=getDistance(incloud->points[i],incloud->points[j]);
        //                    _densityLocal[i]+=exp(-pow(dis/_dc,2));
        //                    _densityLocal[j]+=exp(-pow(dis/_dc,2));
        //                }

        //         _allcloud[i].NOcorepts=radiussearch;
        int k=omp_get_thread_num();
        if (radiussearch.size() > _min_pets)
        {

            _allcloudIndex[i].NOcorepts=radiussearch1;
            _allcloudIndex[i].pointtype = 3;
            //                        _allcloudIndex[i].pointIndex=i;
            //                        _corecloudIndex.push_back(_allcloudIndex[i]);
            //                        _CoreCloud->points.push_back(incloud->points[i]);
            corecloudIndexMultiCore[k].push_back(_allcloudIndex[i]);
            CoreCloudMutiCore[k].push_back(incloud->points[i]);
        }
        _densityLocal[i]=radiussearch.size();

    }
    for(int i=0;i<coreNum;++i)
    {
        _corecloudIndex.insert(_corecloudIndex.end(),corecloudIndexMultiCore[i].begin(),corecloudIndexMultiCore[i].end());
        *_CoreCloud+=CoreCloudMutiCore[i];
    }

    t2=omp_get_wtime();
    cout << "compute local density took me " << t2-t1<< " seconds" << endl;

    //计算core点的直接密度可达核心点
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> octree1(resolution);//初始化octree
    octree1.setInputCloud(_CoreCloud);
    octree1.addPointsFromInputCloud();
#pragma omp parallel for  schedule(runtime)
    for (int i = 0; i<_CoreCloud->points.size(); i++)
    {
        vector<int> pointIdxNKNSearch;//存放点的索引
        vector<float> pointRadiusSquaredDistance;//存放点的距离平方
        octree1.radiusSearch(_CoreCloud->points[i], _dbR, pointIdxNKNSearch, pointRadiusSquaredDistance);//八叉树的邻域搜索
        _corecloudIndex[i].corepts=pointIdxNKNSearch;
    }

}


void my_density_cluster::getDistanceToHigherDensityByOrder(PointCloud incloud)//计算高密距离
{
    double t1,t2,t3,t4;
    t1=omp_get_wtime();
    int nSamples = incloud->points.size();
    _minDist2Higher.resize(nSamples, 0.0);
    _nearestNeighbor.resize(nSamples, -1);

    _densityLocal_pair.resize(nSamples, make_pair(0,0));
#pragma omp parallel for  schedule(runtime)
    for(int i=0;i<_densityLocal_pair.size();i++)
    {
        _densityLocal_pair[i] = make_pair(_densityLocal[i], i);
    }
    t3=omp_get_wtime();
    sort(_densityLocal_pair.begin(), _densityLocal_pair.end(), [](const pair<int, int> &left, const pair<int, int> &right)
    {
        return left.first > right.first;
    });
    t4=omp_get_wtime();

    double globalMax=0;

#pragma omp parallel for  schedule(runtime)
    for(int i = 0; i < nSamples; i++)
    {
        double min=getDistance(incloud->points[_densityLocal_pair[i].second],
                incloud->points[_densityLocal_pair[0].second]);
        double buf=0;
        _nearestNeighbor[_densityLocal_pair[i].second]=_densityLocal_pair[0].second;
        for(int j = 0; j < i; j++)
        {
            buf=getDistance(incloud->points[_densityLocal_pair[i].second],
                    incloud->points[_densityLocal_pair[j].second]);
            if(buf>globalMax)
            {
                globalMax=buf;
            }
            if(buf<min)
            {
                min=buf;
                _nearestNeighbor[_densityLocal_pair[i].second]=_densityLocal_pair[j].second;
            }
        }
        _minDist2Higher[_densityLocal_pair[i].second] = min;
        _minDist2Higher[_densityLocal_pair[0].second]=globalMax;
    }
    t2 = omp_get_wtime();
    cout << "compute distance to high took me " << t2-t4<< " seconds" << endl;
    cout << "compute sort to high took me " << t4-t3<< " seconds" << endl;
}

void my_density_cluster::findClusterCenters(PointCloud incloud)
{
    double t1,t2;
    t1=omp_get_wtime();
    int total_len = _minDist2Higher.size();
    double density_min,density_max;
    double dis_min,dis_max;
    density_min=*min_element(_densityLocal.begin(),_densityLocal.end());
    density_max=*max_element(_densityLocal.begin(),_densityLocal.end());
    dis_min=*min_element(_minDist2Higher.begin(),_minDist2Higher.end());
    dis_max=*max_element(_minDist2Higher.begin(),_minDist2Higher.end());
    double density_range=density_max-density_min;
    double dis_range=dis_max-dis_min;
    vector<pair<int,double>> tmp;
    vector<pair<int,double>> tmpLeft;
    vector<pair<int,double>> tmpRight;
    for(int i=0;i<total_len;++i)
    {
        double decision_value=(_minDist2Higher[i]-dis_min)*(_densityLocal[i]-density_min)/(density_range*dis_range);
        //        tmp.push_back(make_pair(i, decision_value));
        if(incloud->points[i].y>0&&abs(incloud->points[i].x)<10)
        {
            tmpLeft.push_back(make_pair(i, decision_value));
        }
        if(incloud->points[i].y<0&&abs(incloud->points[i].x)<10)
        {
            tmpRight.push_back(make_pair(i, decision_value));
        }

        //        cout << m_density[i] << "," << m_minDist2Higher[i] << endl;
    }
    //    sort(tmp.begin(), tmp.end(), [](pair<int, double>&left, pair<int, double>& right)
    //    {
    //        return left.second > right.second;
    //    });
    nth_element(tmpLeft.begin(),tmpLeft.begin(),tmpLeft.end(),[](pair<int, double>&left, pair<int, double>& right)
    {
        return left.second > right.second;
    });
    //    sort(tmpLeft.begin(), tmpLeft.end(), [](pair<int, double>&left, pair<int, double>& right)
    //    {
    //        return left.second > right.second;
    //    });
    //    sort(tmpRight.begin(), tmpRight.end(), [](pair<int, double>&left, pair<int, double>& right)
    //    {
    //        return left.second > right.second;
    //    });
    nth_element(tmpRight.begin(),tmpRight.begin(),tmpRight.end(),[](pair<int, double>&left, pair<int, double>& right)
    {
        return left.second > right.second;
    });

    //    int selectInd =1;
    //    for(int i=0; i<total_len; ++i)
    //    {
    //        if(i <= selectInd && tmp[i].second)
    //        {
    //            _centers.push_back(tmp[i].first);
    //        }
    //    }
    _centers.push_back(tmpLeft[0].first);
    _centers.push_back(tmpRight[0].first);
    for(int i=0;i<_centers.size();++i)
    {
        cout<<"center "<<i<<" is "<<incloud->points[_centers[i]].x<<" "<<incloud->points[_centers[i]].y<<endl;
    }

    t2 = omp_get_wtime();
    cout << "compute cluster centers took me " << t2-t1 << " seconds" << endl;
}
void my_density_cluster::processbyY(PointCloud incloud, vector<PointCloud> clusters)
{
    for (int i = 0; i <incloud->points.size() ; ++i)
    {
      if(incloud->points[i].y>0)
      {
          clusters[0]->points.push_back(incloud->points[i]);
      }
      else
      {
          clusters[1]->points.push_back(incloud->points[i]);
      }
    }
}
void my_density_cluster::process(PointCloud incloud, vector<PointCloud> clusters)
{

    double t1,t2;
    t1=omp_get_wtime();
    _clusterNum = 0;

    for (int k = 0; k<_centers.size(); k++)
    {
        for (int i = 0; i < _corecloudIndex.size(); ++i)
        {
            if(_corecloudIndex[i].pointIndex==_centers[k])
            {
                stack<pointPty*> ps;
                if (_corecloudIndex[i].visited == 1) continue;
                _clusterNum++;
                _corecloudIndex[i].cluster = _clusterNum;
                ps.push(&(_corecloudIndex[i]));
                pointPty *v=&(_corecloudIndex[i]);
                //将密度可达的核心点归为一类
                while (!ps.empty())
                {
                    v = ps.top();
                    v->visited = 1;
                    ps.pop();
                    for (int j = 0; j<v->corepts.size(); j++)
                    {
                        if (_corecloudIndex[v->corepts[j]].visited == 1) continue;
                        _corecloudIndex[v->corepts[j]].cluster = _corecloudIndex[i].cluster;
                        _corecloudIndex[v->corepts[j]].visited = 1;
                        ps.push(&_corecloudIndex[v->corepts[j]]);
                    }
                }
            }
        }
    }

    cout<<"核心点分类完毕"<<endl;
    //找出所有的边界点，噪声点，对边界点分类，更改其cluster
    //    _classType.resize(incloud->points.size(),-1);
#pragma omp parallel for  schedule(runtime)
    for (int i = 0; i<_corecloudIndex.size(); i++)
    {
        _allcloudIndex[_corecloudIndex[i].pointIndex].cluster=_corecloudIndex[i].cluster;
        //         _allcloudIndex[_corecloudIndex[i].pointIndex].cluster=_corecloudIndex[i].cluster;
        for (int j = 0; j<_corecloudIndex[i].NOcorepts.size(); j++)
        {
            _allcloudIndex[_corecloudIndex[i].NOcorepts[j]].pointtype = 2;
            _allcloudIndex[_corecloudIndex[i].NOcorepts[j]].cluster = _corecloudIndex[i].cluster;
        }
    }
    cout<<"边界点分类完毕"<<endl;
    //#pragma omp parallel for
    //对离散点进行分类
    PointCloud cloud_in(new pcl::PointCloud<PointT>);   //已分类点
    vector<pointPty> cloud_in_Index;
    PointCloud cloud_out(new pcl::PointCloud<PointT>);  //未分类点
    vector<pointPty> cloud_out_Index;
    for (int i = 0; i < incloud->points.size(); i++)
    {
        if (_allcloudIndex[i].cluster !=0)
        {
            cloud_in->points.push_back(incloud->points[i]);
            cloud_in_Index.push_back(_allcloudIndex[i]);
        }
        else
        {
            cloud_out->points.push_back(incloud->points[i]);
            cloud_out_Index.push_back(_allcloudIndex[i]);
        }
    }
    double resolution=0.01;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> octree2(resolution);//初始化octree

    for (int i = 0; i < cloud_out->points.size(); ++i)
    {
        octree2.setInputCloud(cloud_in);
        octree2.addPointsFromInputCloud();
        vector<int> pointIdxNKNSearch;//存放点的索引
        vector<float> pointRadiusSquaredDistance;//存放点的距离平方
        octree2.nearestKSearch(cloud_out->points[i],1,pointIdxNKNSearch, pointRadiusSquaredDistance);//八叉树的邻域搜索
        int cluster1Count=0;
        int cluster2Count=0;
        for (int j = 0; j < pointIdxNKNSearch.size(); ++j)
        {
            if(cloud_in_Index[pointIdxNKNSearch[j]].cluster==1)
            {
                ++cluster1Count;
            }
            else
            {
                ++cluster2Count;
            }
        }
        //计算与两个聚类中心的距离
                float d1=getDistance(cloud_out->points[i],incloud->points[_centers[0]]);
                float d2=getDistance(cloud_out->points[i],incloud->points[_centers[1]]);
        //        float d1Y=getDistanceY(cloud_out->points[i],incloud->points[_centers[0]]);
        //        float d2Y=getDistanceY(cloud_out->points[i],incloud->points[_centers[1]]);
        if(d1<d2)
        {
            _allcloudIndex[cloud_out_Index[i].pointIndex].cluster=1;
        }
        else
        {
            _allcloudIndex[cloud_out_Index[i].pointIndex].cluster=2;
        }
//                else if(d2>d1)
//                {
//                    _allcloudIndex[cloud_out_Index[i].pointIndex].cluster=1;
//                }
//                else
//                {
//                     _allcloudIndex[cloud_out_Index[i].pointIndex].cluster=2;
//                }
        cloud_in->points.push_back(incloud->points[cloud_out_Index[i].pointIndex]);
        cloud_in_Index.push_back(_allcloudIndex[cloud_out_Index[i].pointIndex]);
    }

    cout<<"离散点分类完毕"<<endl;
    int clusterSize=0;
//        #pragma omp parallel for  schedule(runtime)
    for(int i=0;i<incloud->points.size();++i)
    {
        if(_allcloudIndex[i].cluster!=0)
        {
            clusters[_allcloudIndex[i].cluster-1]->points.push_back(incloud->points[i]);
        }

    }
    for (int i = 0; i < clusters.size(); ++i)
    {
        clusterSize+=clusters[i]->points.size();
    }
    t2=omp_get_wtime();
    cout << "compute process took me " << t2-t1 << " seconds" << endl;
    cout<<"分类后点云数为 "<<clusterSize<<endl;
}
