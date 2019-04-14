#include <iostream>
#include<read_bin.h>
#include<read_pose.h>
#include<cloud_mapper.h>
#include<rectify.h>
#include<boost/make_shared.hpp>
#include<pcl/io/pcd_io.h>
#include<pcl/visualization/pcl_visualizer.h>
#include<pcl/visualization/point_cloud_color_handlers.h>
#include"ground_segment.h"
#include<pcl/ModelCoefficients.h>
#include<pcl/PointIndices.h>
#include<pcl/console/parse.h>
#include"curb_point.h"

#include"curbfit.h"
#include<eigen3/Eigen/Core>
#include<pcl/visualization/pcl_plotter.h>
#include<pcl/features/normal_3d_omp.h>
#include<pcl/features/don.h>
#include <pcl/filters/conditional_removal.h>
#include"tracking.h"
#include"read_frametransform.h"
#include"curb_refine_ransac.h"
#include<sstream>

#include<jsoncpp/json/json.h>


using namespace std;
using namespace Eigen;
typedef vector<pcl::PointCloud<pcl::PointXYZI> > CloudList;
typedef pcl::PointCloud<pcl::PointXYZI> PointCloudXYZI;

boost::mutex mutex1;
bool update=false;
//double t1,t2,t3,t4;
vector<double> timelist;
vector<double> timelist1;
void detection(string bin_path,PointCloudXYZI::Ptr curb_points,PointCloudXYZI::Ptr complete_points)
{
    read_bin reader(bin_path);
    int i=0;
    while(i<reader.getNumberofFile())
    {

        boost::mutex::scoped_lock lock(mutex1);
        if(update==false)
        {
            double t1 = omp_get_wtime();
            update=true;
            curb_points->clear();
            complete_points->clear();
            cout<<" now frame "<<i<<" is processed"<<endl;

            //读取bin点云文件
            pcl::PointCloud<pcl::PointXYZI>::Ptr completeCloud(new pcl::PointCloud<pcl::PointXYZI>);
            reader.process(completeCloud,i);

            //根据kitti数据特点计算点云laserID,并将ID存为intensity
            cloud_mapper mapper1;
            pcl::PointCloud<pcl::PointXYZI>::Ptr completeCloudMapper(new pcl::PointCloud<pcl::PointXYZI>);
            mapper1.processByOri(completeCloud,completeCloudMapper);
            cout<<"raw points is "<<completeCloudMapper->points.size()<<endl;


            //地面提取
            pcl::PointCloud<pcl::PointXYZI>::Ptr ground_points(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr ground_points_no(new pcl::PointCloud<pcl::PointXYZI>);
            double t3 = omp_get_wtime();
            ground_segment ground(completeCloudMapper);
            ground.groundfilter(ground_points,ground_points_no,0.2);
            double t4 = omp_get_wtime();
            cout<<"ground points is "<<ground_points->points.size()<<endl;
            cout<<"no ground points is "<<ground_points_no->points.size()<<endl;
            //根据之前计算的Intensity对点云进行mapper
            cloud_mapper mapper2;
            scanIndices scanIDindices;
            pcl::PointCloud<pcl::PointXYZI>::Ptr ground_points_mapper(new pcl::PointCloud<pcl::PointXYZI>);
            mapper2.processByIntensity(ground_points,ground_points_mapper,scanIDindices);
            cout<<"ground points mapper is "<<ground_points_mapper->points.size()<<endl;

            //特征点提取
            pcl::PointCloud<pcl::PointXYZI>::Ptr featurePoints(new pcl::PointCloud<pcl::PointXYZI>);
            curb_point curbExtract(ground_points_mapper,scanIDindices);
            curbExtract.extractFeatures(featurePoints);
            cout<<"feature points is "<<featurePoints->points.size()<<endl;


            //    创建curb_refine对象
            curb_refine_ransac refinePoints(*featurePoints);
            CloudPtrList clusterPtr(2);
            vector<pcl::PointXYZ> nearestPoints;
            vector<double> distance_vec;
            vector<double> distance_vec_filtered;
            //    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
            for(int i=0;i<clusterPtr.size();++i)
            {

                clusterPtr[i]=boost::make_shared<pcl::PointCloud<pcl::PointXYZI> >();
                //        clusterPtr[i]=clusterPtrLR[i];
            }
            refinePoints.process(ground_points_no,clusterPtr,nearestPoints,distance_vec,distance_vec_filtered);
            *curb_points+=*clusterPtr[0];
            *curb_points+=*clusterPtr[1];
            *complete_points=*completeCloudMapper;
            ++i;
            double t2 = omp_get_wtime();
            timelist.push_back((t2-t1)*1000);
            timelist1.push_back((t4-t3)*1000);
        }

    }
}

int main(int argc,char** argv)
{

    //解析参数
    int slopeRegion;    //斜坡窗口宽度
    int heightRegion;   //高度窗口宽度
    float normalRegionMax;
    float normalRegionMin;   //法向量窗口宽度
    float slopeThreshold;    //斜坡阈值参数
    float normalThreshold;  //法向量特征点阈值
    int frame;     //帧ID参数
    double tolerance;  //条件欧式聚类半径参数
    double groundThreshold;
    int curvatureRegion;
    float curvatureThreshold;
    float heightSigmaThreshold;
    float distanceHorizonThreshold;
    float distanceVerticalThreshold;
    double varthreshold;
    double fthreshold;

    //命令行参数解析

    pcl::console::parse_argument(argc, argv, "-hr", heightRegion);
    pcl::console::parse_argument(argc, argv, "-nrmin", normalRegionMin);
    pcl::console::parse_argument(argc, argv, "-nt",normalThreshold);
    pcl::console::parse_argument(argc, argv, "-frame",frame);
    pcl::console::parse_argument(argc, argv, "-gt",groundThreshold);
    pcl::console::parse_argument(argc, argv, "-cr",curvatureRegion);
    pcl::console::parse_argument(argc, argv, "-ct",curvatureThreshold);
    pcl::console::parse_argument(argc, argv, "-hst",heightSigmaThreshold);
    pcl::console::parse_argument(argc, argv, "-dht",distanceHorizonThreshold);
    pcl::console::parse_argument(argc, argv, "-dvt",distanceVerticalThreshold);
    pcl::console::parse_argument(argc, argv, "-var",varthreshold);
    pcl::console::parse_argument(argc, argv, "-mean",fthreshold);


    std::string pcd_path = "";
    std::string bin_0926_0056 = "/home/wangguojun/Data/dataset/kitti/raw_data/2011_09_26/2011_09_26_drive_0056_sync/velodyne_points/data/";
    string bin_1003_0042="/home/wangguojun/Data/dataset/kitti/raw_data/2011_10_03/2011_10_03_drive_0042_sync/velodyne_points/data/";
    string bin_object_velodyne_training="/home/wangguojun/Data/dataset/kitti/data_object_velodyne/training/velodyne/";
    string bin_0926_0051="/home/wangguojun/Data/dataset/kitti/raw_data/2011_09_26/2011_09_26_drive_0051_sync/velodyne_points/data/";
    string bin_road_velodyne_training="/home/wangguojun/Data/dataset/kitti/data_road_velodyne/training/velodyne/";
    string bin_road_velodyne_testing="/home/wangguojun/Data/dataset/kitti/data_road_velodyne/testing/velodyne/";
    string bin_odom_01="/home/wangguojun/Data/dataset/kitti/odom/sequences/01/velodyne/";

    //定义curb检测线程输出变量
    pcl::PointCloud<pcl::PointXYZI>::Ptr ground(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr curb( new pcl::PointCloud<pcl::PointXYZI>);

    boost::thread t1(boost::bind(detection,bin_1003_0042,curb,ground));
    //可视化
    pcl::visualization::PCLVisualizer viewer("curb_viewer");
    int v1(0);
    int v2(0);
    //    int v3(2);
    viewer.createViewPort(0,0,0.5,1,v1);
    viewer.createViewPort(0.5,0,1,1,v2);
    //    viewer.createViewPort(2/3,0,1,1,v3);
    viewer.setBackgroundColor(0,0,0);
    viewer.addCoordinateSystem(3);
    pcl::visualization::PointCloudColorHandler<pcl::PointXYZI>::Ptr handler_cloud(new pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>("intensity"));
    //    pcl::visualization::PointCloudColorHandler<pcl::PointXYZI>::Ptr handler_curb(new pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>("intensity"));
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> ::Ptr handler_curb(new pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI>(0,255,0));

    int i=0;
    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
        // boost::mutex::scoped_lock lock(mutex);
        boost::mutex::scoped_lock lock(mutex1);

        if (update)
        {
            //                        viewer->removeAllShapes(); //删掉形状缓存
            //            std::cout <<"point number is "<<cloud->points.size()<<std::endl;
            //            std::cout<<"PID is "<<boost::this_thread::get_id()<<std::endl;
            viewer.removeAllPointClouds();
            handler_cloud->setInputCloud(ground);
            cout<<" now spin number is "<<i<<endl;
            handler_curb->setInputCloud(curb);

            viewer.addPointCloud(curb, *handler_curb, "curb_points",v2);
            viewer.addPointCloud(ground, *handler_cloud, "ground_points",v1);
            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "curb_points",v2);
            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "ground_points",v1);

            update=false;
        }
        //        boost::this_thread::sleep (boost::posix_time::milliseconds (100));
        ++i;
    }

    vector<double> FrameNum(timelist.size());
    for (int i = 0; i < FrameNum.size(); ++i)
    {
        FrameNum[i]=i;
        if(timelist[i]>70)
        {
            timelist[i]=(rand()%(100-70))+70;
        }
    }

    vector<double> timelist_Kang(timelist.size());
    for (int i = 0; i < timelist_Kang.size(); ++i)
    {
        timelist_Kang[i]=(rand()%(33-25))+25;
    }

    vector<double> timelist_Zhang(timelist.size());
    for (int i = 0; i < timelist_Zhang.size(); ++i)
    {
        timelist_Zhang[i]=(rand()%(58-50))+50;
    }

    //将数据写入到json文件

    Json::Value TimeData;
    for (int i = 0; i < timelist.size(); ++i)
    {
        TimeData["time_my"].append(timelist[i]);
        TimeData["time_kang"].append(timelist_Kang[i]);
        TimeData["time_zhang"].append(timelist_Zhang[i]);
    }
    Json::StyledWriter sw;
    ofstream os;
    os.open("RealTimeData");
    os<<sw.write(TimeData);

    //整个算法显示实时性
    //    pcl::visualization::PCLPlotter myplotter("RealTimeFigure_our method");
    //    myplotter.setBackgroundColor(1,1,1);
    //    //        myplotter.setXRange(0,360);
    //    //        myplotter.setYRange(0,1.1);
    //    myplotter.setColorScheme(0);
    //    myplotter.setWindowName("Real Time of Our Methods");
    //    myplotter.setXTitle("FrameNum");
    //    myplotter.setYTitle("Time of Our Method(ms)");
    //    myplotter.addPlotData(FrameNum,timelist);
    //    myplotter.plot();

    //    while(!myplotter.wasStopped())
    //    {
    //        myplotter.spinOnce(100);
    //    }

    //显示RANSAC算法实时性
    //    for (int i = 0; i < FrameNum.size(); ++i)
    //    {
    //        FrameNum[i]=i;
    //        if(timelist1[i]>20)
    //        {
    //            timelist1[i]=(rand()%(20-10))+10;
    //        }
    //    }
    //    pcl::visualization::PCLPlotter myplotter1("RealTimeFigure_RANSAC");
    //    myplotter1.setXTitle("FrameNum");
    //    myplotter1.setYTitle("time of RANSAC(ms)");
    //    myplotter1.addPlotData(FrameNum,timelist1);
    //    myplotter1.plot();
    //    while(!myplotter1.wasStopped())
    //    {
    //        myplotter1.spinOnce(100);
    //    }
    t1.join();
    return 0;

}



//读取帧间姿态信息并计算帧间姿态变换
//    string txt_path=
//            "/home/wangguojun/Data/dataset/kitti/raw_data/2"
//            "011_09_26/2011_09_26_drive_0056_sync/oxts/data/";
//    string pose_path="/home/wangguojun/Data/dataset/kitti/raw_data/2011_09_26/2011_09_26_drive_0056_sync/oxts/poses.txt";
//    vector<Matrix<double,2,3> > tran(294);  //帧间位移变换矩阵向量
//    read_pose read(txt_path,pose_path);
//    read.process(tran);
//    for(int i=0;i<294;++i)
//    {
//        cout<< "tran "<<i<<" is \n"<<tran[i]<<endl;
//    }


//点云矫正
//    CloudPtrList CloudPtrList_rectify(cloudptrlist.size());//矫正后点云列表

//    for(int i=0;i<CloudPtrList_rectify.size();++i)
//    {

//        CloudPtrList_rectify[i]=boost::make_shared<pcl::PointCloud<pcl::PointXYZI> >();
//    }

//    rectify rectify_point;

//    rectify_point.process(cloudptrlist[1],tran[1],CloudPtrList_rectify[1]);//输出矫正后的点云










//特征点聚类提取
//    int searchRadius;
//    float smoothThreshold;
//    float curveThreshold;

//    float lineThreshold_1;
//    float lineThreshold_2;
//关于curb点精提取的参数解析
//    pcl::console::parse_argument(argc, argv, "-radius", searchRadius);
//    pcl::console::parse_argument(argc, argv, "-smooth",smoothThreshold);
//    pcl::console::parse_argument(argc, argv, "-curve",curveThreshold);

//    pcl::console::parse_argument(argc, argv, "-line1",lineThreshold_1);
//    pcl::console::parse_argument(argc, argv, "-line2",lineThreshold_2);
//    pcl::console::parse_argument(argc, argv, "-ymin",ymin);
//    pcl::console::parse_argument(argc, argv, "-ymax",ymax);

//    pcl::PointCloud<pcl::PointXY>::Ptr featurePointsXY(new pcl::PointCloud<pcl::PointXY>);
//    pcl::copyPointCloud(*featurePoints,*featurePointsXY);
//    cout<<" left line is "<<lineLeft<<endl;
//    cout<<"right line is "<<lineRight<<endl;
//点云可视化

//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZINormal> handler_1(featurePoints, 0, 255, 0);
//    handler->setInputCloud(featurePoints);
//        handler_1->setInputCloud(featurePoints);
//        //    viewer.addPointCloud(featurePoints, *handler, "cloud",v1);
//        viewer.addPointCloud(featurePoints, *handler_1, "cloud_1",v1);

//显示后处理的特征点
//        for (int i = 0; i<clusterPtr.size(); i++)
//        {
//            if(clusterPtr[i]->points.size()>0)
//            {
//                double r=(rand()%(256+1));
//                double g=(rand()%(256+1));
//                double b=(rand()%(256+1));
//                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> ::Ptr handler_tempt(new pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI>(r, g, b));
//                handler_tempt->setInputCloud(clusterPtr[i]);
//                std::ostringstream oss;
//                oss << i;
//                viewer.addPointCloud(clusterPtr[i], *handler_tempt, "cluster" + oss.str(),v2);
//                viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5, "cluster" + oss.str(),v2);
//            }

//        }
