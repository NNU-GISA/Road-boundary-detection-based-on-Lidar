#include "max_density_cluster.h"
#include<omp.h>

max_density_cluster::max_density_cluster(PointCloud incloud,double neighborRateLow):
    _neighborRateLow(neighborRateLow)
{
    this->getdc(incloud);
    //    _dc=2.1875;
    this->getLocalDensity(incloud);
    this->getDistanceToHigherDensityByOrder(incloud);
    this->findClusterCentersThree(incloud);
}

double getDistance(PointT &pt1, PointT &pt2)
{
    double tmp = pow(pt1.x - pt2.x, 2) + pow(pt1.y - pt2.y, 2) + pow(pt1.z - pt2.z, 2);
    return pow(tmp, 0.5);
}

void max_density_cluster::getdc(PointCloud incloud)//截断距离
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
    int pos=int(disNum*_neighborRateLow);
    nth_element(dis.begin(),dis.begin()+pos-1,dis.end());
    //    sort(dis.begin(), dis.end());
    //    _dc = dis[int(avgNeighbourNum * row)];
    _dc=dis[pos-1];
    t3=omp_get_wtime();
    cout<<"截断距离为 "<<_dc<<endl;
    cout << "compute for loop took me " << (t2-t1) << " seconds" << endl;
    cout << "compute sort took me " << (t3-t2) << " seconds" << endl;
}

void max_density_cluster::getLocalDensity(PointCloud incloud)  //计算局部密度
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
#pragma omp parallel for  schedule(runtime)
    for(int i = 0; i < nSamples - 1; i++)
    {
        vector<int> radiussearch;//存放点的索引
        vector<float> radiusdistance;//存放点的距离平方
        octree.radiusSearch(incloud->points[i], _dc, radiussearch, radiusdistance);//八叉树的邻域搜索
        //                for(int j=i+1;j<nSamples;++j)
        //                {
        //                    double dis=getDistance(incloud->points[i],incloud->points[j]);
        //                    _densityLocal[i]+=exp(-pow(dis/_dc,2));
        //                    _densityLocal[j]+=exp(-pow(dis/_dc,2));
        //                }
        _densityLocal[i]=radiussearch.size();

    }
    t2=omp_get_wtime();
    cout << "compute local density took me " << t2-t1<< " seconds" << endl;

}

void max_density_cluster::getDistanceToHigherDensity(PointCloud incloud)//计算高密距离
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
#pragma omp parallel for  schedule(runtime)
    for(int i = 0; i < nSamples; i++)
    {
        double dist = 0.0;
        int index_neighbor=-1;
        bool flag = false;
        for(int j = 0; j < nSamples; j++)
        {
            if(i == j) continue;
            if(_densityLocal[j] > _densityLocal[i])
            {
                double tmp = getDistance(incloud->points[i], incloud->points[j]);
                if(!flag)
                {
                    dist = tmp;
                    index_neighbor=j;
                    flag = true;
                }
                if(tmp < dist)
                {
                    dist = tmp;
                    index_neighbor=j;
                }
            }
        }
        if(!flag)
        {
            for(int j = 0; j < nSamples; j++)
            {
                double tmp = getDistance(incloud->points[i], incloud->points[j]);
                dist = tmp > dist ? tmp : dist;
            }
        }
        _minDist2Higher[i] = dist;
        _nearestNeighbor[i]=index_neighbor;
        //cout<<"getting delta. Processing point No."<<i<<endl;
    }
    t2 = omp_get_wtime();
    cout << "compute distance to high took me " << t2-t4<< " seconds" << endl;
    cout << "compute sort to high took me " << t4-t3<< " seconds" << endl;
}

void max_density_cluster::getDistanceToHigherDensityByOrder(PointCloud incloud)//计算高密距离
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


void max_density_cluster::findClusterCenters()
{
    int total_len = _minDist2Higher.size();
    vector<pair<int,double>> tmp;
    for(int i=0;i<total_len;++i)
    {
        tmp.push_back(make_pair(i, _minDist2Higher[i] * _densityLocal[i]));
    }

    sort(tmp.begin(), tmp.end(), [](pair<int, double>&left, pair<int, double>& right)
    {
        return left.second > right.second;
    });

    vector<int> diff1;
    int split_index = 0;
    diff1.resize(total_len, 0);
    double total_sum = 0.0;
    for(int i=0;i<total_len-1;++i)
    {
        diff1[i] = (int)(tmp[i].second - tmp[i+1].second);
        total_sum += diff1[i];
    }
    double prefix_sum = diff1[0];
    for(int i=1;i<diff1.size();++i)
    {
        prefix_sum += diff1[i];
        if(prefix_sum / total_sum >= 0.95)
        {
            split_index = i;
            break;
        }
    }
    cout << "Find Center are :" << endl;
    for(int i=0;i<=split_index;++i)
    {
        _centers.push_back(tmp[i].first);
        //        cout << tmp[i].first << endl;
    }
    cout << "_minDist2Higher,m_density,PointInd,Poduct" << endl;
    //    for(int i=0;i<tmp.;++i)
    //    {
    //        cout << _minDist2Higher[i] << "," << _densityLocal[i] << "," << tmp[i].first << "," << tmp[i].second << endl;
    //    }
}
void max_density_cluster::findClusterCentersTwo(PointCloud incloud)
{
    double t1,t2;
    t1=omp_get_wtime();
    int total_len = _minDist2Higher.size();
    vector<pair<int,double>> tmpLeft;
    vector<pair<int,double>> tmpRight;
    vector<pair<int,double>> tmp;
    for(int i=0;i<total_len;++i)
    {
        tmp.push_back(make_pair(i, _minDist2Higher[i]*_densityLocal[i]));
        if(incloud->points[i].y>0)
        {
            tmpLeft.push_back(make_pair(i, _minDist2Higher[i] * _densityLocal[i]));
        }
        if(incloud->points[i].y<0)
        {
            tmpRight.push_back(make_pair(i, _minDist2Higher[i] * _densityLocal[i]));
        }

    }
    sort(tmp.begin(), tmp.end(), [](pair<int, double>&left, pair<int, double>& right)
    {
        return left.second > right.second;
    });
    sort(tmpLeft.begin(), tmpLeft.end(), [](pair<int, double>&left, pair<int, double>& right)
    {
        return left.second > right.second;
    });
    sort(tmpRight.begin(), tmpRight.end(), [](pair<int, double>&left, pair<int, double>& right)
    {
        return left.second > right.second;
    });

    //    int selectInd =2;
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
        //        cout<<"center 2 is "<<incloud->points[_centers[1]].x<<" "<<incloud->points[_centers[1]].y<<endl;
        //        cout<<"center 2 is "<<incloud->points[_centers[1]].x<<" "<<incloud->points[_centers[1]].y<<endl;
    }

    t2 = omp_get_wtime();
    cout << "compute cluster centers took me " << t2-t1 << " seconds" << endl;
}
void max_density_cluster::findClusterCentersThree(PointCloud incloud)
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
    vector<pair<int,double>> tmpRight;
    vector<pair<int,double>> tmpLeft;
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
        sort(tmpLeft.begin(), tmpLeft.end(), [](pair<int, double>&left, pair<int, double>& right)
        {
            return left.second > right.second;
        });
        sort(tmpRight.begin(), tmpRight.end(), [](pair<int, double>&left, pair<int, double>& right)
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
void max_density_cluster::classifyFeatures2Centers()
{
    double t1,t2;
    t1=omp_get_wtime();
    _classType.resize(_densityLocal.size(), -1);
    for(int i=0;i<_centers.size();++i)
    {
        _classType[_centers[i]] = i;
    }
    for(int i=0;i<_densityLocal_pair.size();++i)//按照m_density_pair经过排序（从大到小），按照局部密度从大到小
    {
        int ind = _densityLocal_pair[i].second;
        if(_classType[ind] == -1 && _classType[_nearestNeighbor[ind]] != -1)
        {
            _classType[ind] = _classType[_nearestNeighbor[ind]];
        }
    }
    t2 =omp_get_wtime();
    cout << "compute classify points took me " << t2-t1 << " seconds" << endl;
}
void max_density_cluster::process(PointCloud incloud, vector<PointCloud> clusters)
{

    this->classifyFeatures2Centers();

    for(int i=0;i<incloud->points.size();++i)
    {

        if(_classType[i]!=-1)
        {
            clusters[_classType[i]]->points.push_back(incloud->points[i]);
        }

    }
}
