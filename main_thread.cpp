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
#include"curb_refine.h"
#include"curbfit.h"
#include<eigen3/Eigen/Core>
#include<pcl/visualization/pcl_plotter.h>
#include<pcl/features/normal_3d_omp.h>
#include<pcl/features/don.h>
#include <pcl/filters/conditional_removal.h>
#include"tracking.h"
#include"read_frametransform.h"
#include"curb_refine_new.h"
#include"curb_refine_ransac.h"
#include<sstream>

#define Eigen_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ASSERT

using namespace std;
using namespace Eigen;
typedef vector<pcl::PointCloud<pcl::PointXYZI> > CloudList;


boost::mutex mutex;
bool update=false;

void curb_extract(string bin_path, vector<Matrix4d,aligned_allocator<Matrix4d>> frameTransform, pcl::PointCloud<pcl::PointXYZI>::Ptr curb_points,
                  vector<pcl::PointXYZ>* lineLeft3D,vector<pcl::PointXYZ>* lineRight3D,pcl::PointCloud<pcl::PointXYZI>::Ptr complete_points)
{
    //    Tracking curbTrackerLeft;

    //    Tracking curbTrackerRight;
    read_bin reader(bin_path);

    int i=0;

    while(i<reader.getNumberofFile())
    {
        boost::mutex::scoped_lock lock(mutex);
        if(update==false)
        {
            update=true;
            curb_points->clear();
            complete_points->clear();
            lineLeft3D->clear();
            lineRight3D->clear();
            cout<<" now frame "<<i<<" is processed"<<endl;
            //读取bin点云文件
            pcl::PointCloud<pcl::PointXYZI>::Ptr completeCloud(new pcl::PointCloud<pcl::PointXYZI>);
            reader.process(completeCloud,i);

            //地面提取
            pcl::ModelCoefficients::Ptr modelCoe(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr indices(new pcl::PointIndices);
            pcl::PointCloud<pcl::PointXYZI>::Ptr ground_points(new pcl::PointCloud<pcl::PointXYZI>);
            ground_segment ground;
            ground.planeSeg(completeCloud,modelCoe,indices,0.4);
            ground.extractGround(ground_points,completeCloud,indices);
            cout<<" ground points is "<<ground_points->points.size()<<endl;


            //点云mapper
            cloud_mapper mapper;
            scanIndices scanIDindices;
            // CloudList scanlist_rectify(mapper.getNumberOfScanRings());//mapper后的点云
            pcl::PointCloud<pcl::PointXYZI>::Ptr ground_points_mapper(new pcl::PointCloud<pcl::PointXYZI>);
            mapper.processByOri(ground_points,ground_points_mapper,scanIDindices);
            cout<<"ground points mapper is "<<ground_points_mapper->points.size()<<endl;



            //特征点提取
            pcl::PointCloud<pcl::PointXYZI>::Ptr featurePoints(new pcl::PointCloud<pcl::PointXYZI>);
            curb_point curbExtract(ground_points_mapper,scanIDindices);
            curbExtract.extractFeatures(featurePoints);
            cout<<"feature points is "<<featurePoints->points.size()<<endl;


            //创建curb_refine对象
            //            curb_refine_ransac refinePoints(*featurePoints);
            //            CloudPtrList clusterPtr(2);
            //            //        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
            //            for(int i=0;i<clusterPtr.size();++i)
            //            {

            //                clusterPtr[i]=boost::make_shared<pcl::PointCloud<pcl::PointXYZI> >();
            //            }
            //            refinePoints.process(clusterPtr);
            //             左右curb点拟合
            //                        vector<double> lineLeft(2);
            //                        vector<double> lineRight(2);
            //            if(clusterPtr[0]->points.size()>6)
            //            {
            //                curbFit lineFitLeft(clusterPtr[0]);
            //                //                lineFitLeft.process(*lineLeft3D);
            //                //                lineFitLeft.nurbFit(curve[0]);
            //                lineFitLeft.ransac_curve(clusterPtr[0],0.1,*lineLeft3D);
            //                //左右curb点跟踪（卡尔曼filter）
            //                //                curbTrackerLeft.ProcessMeasurement(*lineLeft3D,frameTransform[i],true);
            //            }
            //            else
            //            {
            //                //                curbTrackerLeft.ProcessMeasurement(*lineLeft3D,frameTransform[i],false);
            //            }
            //            if(clusterPtr[1]->points.size()>6)
            //            {
            //                curbFit lineFitRight(clusterPtr[1]);
            //                //                lineFitRight.process(*lineRight3D);
            //                //                lineFitRight.nurbFit(curve[1]);
            //                lineFitRight.ransac_curve(clusterPtr[1],0.1,*lineRight3D);
            //                //左右curb点跟踪（卡尔曼filter）
            //                //                curbTrackerRight.ProcessMeasurement(*lineRight3D,frameTransform[i],true);
            //            }
            //            else
            //            {
            //                //                curbTrackerRight.ProcessMeasurement(*lineRight3D,frameTransform[i],false);
            //            }

            //            *curb_points+=*clusterPtr[0];
            //            *curb_points+=*clusterPtr[1];
            *complete_points=*completeCloud;
            //            *curb_points=*cloud_list[i];

            ++i;
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
    //string filename;
    double scale1;
    double scale2;
    double threshold;
    //    double segradius;
    //命令行参数解析
    pcl::console::parse_argument(argc, argv, "-sr", slopeRegion);
    pcl::console::parse_argument(argc, argv, "-hr", heightRegion);
    pcl::console::parse_argument(argc, argv, "-nrmax",normalRegionMax);
    pcl::console::parse_argument(argc, argv, "-nrmin", normalRegionMin);
    pcl::console::parse_argument(argc, argv, "-st", slopeThreshold);
    pcl::console::parse_argument(argc, argv, "-nt",normalThreshold);
    pcl::console::parse_argument(argc, argv, "-frame",frame);
    pcl::console::parse_argument(argc, argv, "-tolerance",tolerance);
    pcl::console::parse_argument(argc, argv, "-scale1",scale1);
    pcl::console::parse_argument(argc, argv, "-scale2",scale2);
    pcl::console::parse_argument(argc, argv, "-threshold",threshold);


    std::string pcd_path = "";
    std::string bin_0926_0056 = "/home/wangguojun/Data/dataset/kitti/raw_data/2011_09_26/2011_09_26_drive_0056_sync/velodyne_points/data/";
    string bin_1003_0042="/home/wangguojun/Data/dataset/kitti/raw_data/2011_10_03/2011_10_03_drive_0042_sync/velodyne_points/data/";
    string bin_object_velodyne_training="/home/wangguojun/Data/dataset/kitti/data_object_velodyne/training/velodyne/";
    string bin_0926_0051="/home/wangguojun/Data/dataset/kitti/raw_data/2011_09_26/2011_09_26_drive_0051_sync/velodyne_points/data/";
    string bin_road_velodyne_training="/home/wangguojun/Data/dataset/kitti/data_road_velodyne/training/velodyne/";
    string bin_road_velodyne_testing="/home/wangguojun/Data/dataset/kitti/data_road_velodyne/testing/velodyne/";
    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_points(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr curb_points( new pcl::PointCloud<pcl::PointXYZI>);
    //        pcl::PointCloud<pcl::PointXYZI>::Ptr ground_point_out(new pcl::PointCloud<pcl::PointXYZI>);
    vector<pcl::PointXYZ> lineLeft3D(25);
    vector<pcl::PointXYZ> lineRight3D(25);
    vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> curve_curb(2);
    curve_curb[0]=boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >();
    curve_curb[1]=boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >();

    //读取帧间变换信息
    vector<Matrix4d,aligned_allocator<Matrix4d>> frameTransforms;
    read_frameTransform readTransform("/home/wangguojun/source_code/curb_detection/detection/TrOneFrame.txt");
    readTransform.process(frameTransforms);

    //新建线程处理点云输出拟合直线
    boost::thread t1(boost::bind(curb_extract,bin_road_velodyne_training,frameTransforms,curb_points,&lineLeft3D,&lineRight3D,ground_points));
    //    norDiff(cloudptrlist[frame],scale1,scale2,threshold, curb_points,ground_point_out);
    //    cout<<"curb points is "<<curb_points->points.size()<<endl;

    //计算本车与道路边界距离
    //    double distanceLeft=(0-lineLeft3D[0].x)/(lineLeft3D[1].x-lineLeft3D[0].x)*(lineLeft3D[1].y-lineLeft3D[0].y)+lineLeft3D[0].y;
    //    double distanceRight=(0-lineRight3D[0].x)/(lineRight3D[1].x-lineRight3D[0].x)*(lineRight3D[1].y-lineRight3D[0].y)+lineRight3D[0].y;
    //    stringstream ssLeft;
    //    ssLeft<<" Left curb distance is "<<distanceLeft<<" m";
    //    stringstream ssRight;
    //    ssRight<<" Right curb distance is "<<distanceRight<<" m";

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
    //    viewer.setSize(1920,1080);
    pcl::visualization::PointCloudColorHandler<pcl::PointXYZI>::Ptr handler_cloud(new pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI>(255,255,255));
    pcl::visualization::PointCloudColorHandler<pcl::PointXYZI>::Ptr handler_curb(new pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI>(0,255,0));


    int i=0;
    while (!viewer.wasStopped())
    {
        viewer.spinOnce(1500);
        boost::mutex::scoped_lock lock(mutex);

        if (update)

        {
            cout<<" lineleft start point parameter is "<<lineLeft3D[0].x<<" "<<lineLeft3D[0].y<<" "<<lineLeft3D[0].z<<endl;
            cout<<" lineleft end point parameter is "<<lineLeft3D[1].x<<" "<<lineLeft3D[1].y<<" "<<lineLeft3D[1].z<<endl;
            cout<<" lineright start point parameter is "<<lineRight3D[0].x<<" "<<lineRight3D[0].y<<" "<<lineRight3D[0].z<<endl;
            cout<<" lineright end point parameter is "<<lineRight3D[1].x<<" "<<lineRight3D[1].y<<" "<<lineRight3D[1].z<<endl;
            viewer.removeAllShapes(); //删掉形状缓存
            viewer.removeAllPointClouds();//删除点云缓存

            //添加点云
            handler_cloud->setInputCloud(ground_points);
            handler_curb->setInputCloud(curb_points);
            viewer.addPointCloud(curb_points,*handler_curb,"curb_points",v2);
            viewer.addPointCloud(ground_points,*handler_cloud,"ground_points",v1);
            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "curb_points",v2);

            //添加车道直线

            //            viewer.addLine(lineLeft3D[0],lineLeft3D[1],"lineleft");
            //            viewer.addLine(lineRight3D[0],lineRight3D[1] ,"lineright");
            //            viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,3,"lineleft");
            //            viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,3,"lineright");
            //            viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0,1,0,"lineright");
            //            viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0,1,0,"lineleft");

            //添加车道曲线(B样条曲线）

            //            for (std::size_t i = 0; i < curve_curb[0]->size () - 1; i++)
            //            {
            //                pcl::PointXYZRGB &p1 = curve_curb[0]->at (i);
            //                pcl::PointXYZRGB &p2 = curve_curb[0]->at (i + 1);
            //                std::ostringstream os;
            //                os << "lineLeft_" << i;
            //                viewer.addLine<pcl::PointXYZRGB> (p1, p2, 1, 0, 0, os.str ());
            //                viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,3,os.str());
            //            }
            //            for (std::size_t i = 0; i < curve_curb[1]->size () - 1; i++)
            //            {
            //                pcl::PointXYZRGB &p1 = curve_curb[1]->at (i);
            //                pcl::PointXYZRGB &p2 = curve_curb[1]->at (i + 1);
            //                std::ostringstream os;
            //                os << "lineRight_" << i;
            //                viewer.addLine<pcl::PointXYZRGB> (p1, p2, 1, 0, 0, os.str ());
            //                viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,3,os.str());
            //            }
            //ransac二次曲线

            //            for (std::size_t i = 0; i < lineLeft3D.size () - 1; i++)
            //            {
            //                pcl::PointXYZ &p1 =  lineLeft3D[i];
            //                pcl::PointXYZ &p2 = lineLeft3D[i+1];
            //                std::ostringstream os;
            //                os << "lineLeft_" << i;
            //                viewer.addLine<pcl::PointXYZ> (p1, p2, 1, 0, 0, os.str (),v1);
            //                viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,3,os.str(),v1);
            //            }

            //            for (std::size_t i = 0; i < lineRight3D.size () - 1; i++)
            //            {
            //                pcl::PointXYZ &p1 =  lineRight3D[i];
            //                pcl::PointXYZ &p2 = lineRight3D[i+1];
            //                std::ostringstream os;
            //                os << "lineRight_" << i;
            //                viewer.addLine<pcl::PointXYZ> (p1, p2, 1, 0, 0, os.str (),v1);
            //                viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,3,os.str(),v1);
            //            }




            //            if(!viewer.updateText(ssLeft.str(),50,20,0,255,0))
            //            {
            //                viewer.addText(ssLeft.str(),50,20,0,255,0,"leftTxt");
            //            }
            //            if(!viewer.updateText(ssRight.str(),50,30,0,255,0))
            //            {
            //                viewer.addText(ssRight.str(),50,30,0,255,0,"rightTxt");
            //            }

            update=false;
        }
        ++i;
    }

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
