#include <iostream>
#include<boost/make_shared.hpp>
#include<pcl/io/pcd_io.h>
#include<pcl/io/png_io.h>
#include<pcl/visualization/pcl_visualizer.h>
#include<pcl/visualization/point_cloud_color_handlers.h>
#include<pcl/ModelCoefficients.h>
#include<pcl/PointIndices.h>
#include<pcl/console/parse.h>
#include<eigen3/Eigen/Core>
#include<pcl/visualization/pcl_plotter.h>

#include"read_bin.h"
#include"read_pose.h"
#include"cloud_mapper.h"
#include"rectify.h"
#include"ground_segment.h"
#include"curb_point.h"
#include"curbfit.h"
#include"tracking.h"
#include"curb_refine_ransac.h"

#include<jsoncpp/json/json.h>

using namespace std;
using namespace Eigen;

typedef vector<pcl::PointCloud<pcl::PointXYZI> > CloudList;


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
    //    float radius_dbscan;
    //    int min_points;
    //    double neighborRate;
    //命令行参数解析
    //    pcl::console::parse_argument(argc, argv, "-sr", slopeRegion);
    pcl::console::parse_argument(argc, argv, "-hr", heightRegion);
    //    pcl::console::parse_argument(argc, argv, "-nrmax",normalRegionMax);
    pcl::console::parse_argument(argc, argv, "-nrmin", normalRegionMin);
    //    pcl::console::parse_argument(argc, argv, "-st", slopeThreshold);
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
    //    pcl::console::parse_argument(argc, argv, "-dbR",radius_dbscan);
    //    pcl::console::parse_argument(argc, argv, "-minps",min_points);
    //    pcl::console::parse_argument(argc, argv, "-low",neighborRate);
    std::string pcd_path = "";
    std::string bin_0926_0056 = "/home/wangguojun/Data/dataset/kitti/raw_data/2011_09_26/2011_09_26_drive_0056_sync/velodyne_points/data/";
    string bin_1003_0042="/home/wangguojun/Data/dataset/kitti/raw_data/2011_10_03/2011_10_03_drive_0042_sync/velodyne_points/data/";
    string bin_object_velodyne_training="/home/wangguojun/Data/dataset/kitti/data_object_velodyne/training/velodyne/";
    string bin_0926_0051="/home/wangguojun/Data/dataset/kitti/raw_data/2011_09_26/2011_09_26_drive_0051_sync/velodyne_points/data/";
    string bin_road_velodyne_training="/home/wangguojun/Data/dataset/kitti/data_road_velodyne/training/velodyne/";
    string bin_road_velodyne_testing="/home/wangguojun/Data/dataset/kitti/data_road_velodyne/testing/velodyne/";
    string bin_odom_01="/home/wangguojun/Data/dataset/kitti/odom/sequences/01/velodyne/";
    //读取bin文件。存储进点云向量指针cloudptrlist
    read_bin readfile(bin_0926_0056,pcd_path);

    //    CloudPtrList cloudptrlist1(readfile1.getNumberofFile());
    pcl::PointCloud<pcl::PointXYZI>::Ptr complete_cloud(new pcl::PointCloud<pcl::PointXYZI>);


    readfile.process(complete_cloud,frame);
    //    pcl::PointCloud<pcl::PointXYZI>::Ptr tempt(new pcl::PointCloud<pcl::PointXYZI>);
    //    for (int i = 0; i < cloudptrlist1[frame]->points.size(); ++i)
    //    {
    //     if(abs(cloudptrlist1[frame]->points[i].y)<8)
    //     {
    //         tempt->points.push_back(cloudptrlist1[frame]->points[i]);
    //     }
    //    }
    //    *cloudptrlist1[frame]=*tempt;

    //    read_bin readfile2(bin_odom_01,pcd_path);

    //    CloudPtrList cloudptrlist2(readfile2.getNumberofFile());

    //    for(int i=0;i<cloudptrlist2.size();++i)
    //    {

    //        cloudptrlist2[i]=boost::make_shared<pcl::PointCloud<pcl::PointXYZI> >();
    //    }

    //    readfile2.process(cloudptrlist2[frame],frame);



    //读取帧间姿态信息并计算帧间姿态变换
    //    string txt_path=
    //            "/home/wangguojun/Data/dataset/kitti/raw_data/2"
    //            "011_09_26/2011_09_26_drive_0056_sync/oxts/data/";
    //    string pose_path="/home/wangguojun/Data/dataset/kitti/raw_data/2011_09_26/2011_09_26_drive_0056_sync/oxts/Poses.txt";
    //    vector<Matrix<double,2,3>,aligned_allocator<Matrix<double,2,3>> > tran(294);  //帧间位移变换矩阵向量
    //    read_pose read(txt_path,pose_path);
    //    read.process(tran);
    //    for(int i=0;i<294;++i)
    //    {
    //        cout<< "tran "<<i<<" is \n"<<tran[i]<<endl;
    //    }


    //    //点云矫正
    //    CloudPtrList CloudPtrList_rectify(cloudptrlist.size());//矫正后点云列表

    //    for(int i=0;i<CloudPtrList_rectify.size();++i)
    //    {

    //        CloudPtrList_rectify[i]=boost::make_shared<pcl::PointCloud<pcl::PointXYZI> >();
    //    }
    //    rectify rectify_point;

    //    for (int i = 0;  i < cloudptrlist.size(); ++i)
    //    {
    //        rectify_point.process(cloudptrlist[frame],tran[frame],
    //                              CloudPtrList_rectify[frame]);//输出矫正后的点云
    //   }


    //    vector<pcl::PointCloud<pcl::PointXYZI> > laserCloudScansOut(64);

    //根据kitti数据特点计算点云laserID,并将ID存为intensity
    cloud_mapper mapper1;
    pcl::PointCloud<pcl::PointXYZI>::Ptr completeCloudMapper(new pcl::PointCloud<pcl::PointXYZI>);
    mapper1.processByOri(complete_cloud,completeCloudMapper);
    cout<<"raw points is "<<completeCloudMapper->points.size()<<endl;


    //地面提取
    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_points(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_points_no(new pcl::PointCloud<pcl::PointXYZI>);
    ground_segment ground(completeCloudMapper);
    double t1=omp_get_wtime();
    ground.groundfilter(ground_points,ground_points_no,groundThreshold);
    double t2=omp_get_wtime();
    cout << "compute groundsegment took me " << t2-t1<< " seconds" << endl;
    //    ground.groundfilter(ground_points,ground_points_no,groundThreshold);
    cout<<"ground points is "<<ground_points->points.size()<<endl;
    cout<<"no ground points is "<<ground_points_no->points.size()<<endl;
    //根据之前计算的Intensity对点云进行mapper
    cloud_mapper mapper2;
    scanIndices scanIDindices;
    // CloudList scanlist_rectify(mapper.getNumberOfScanRings());//mapper后的点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_points_mapper(new pcl::PointCloud<pcl::PointXYZI>);
    //    vector<pcl::PointCloud<pcl::PointXYZI> > laserCloudScansOut(64);
    mapper2.processByIntensity(ground_points,ground_points_mapper,scanIDindices);
    cout<<"ground points mapper is "<<ground_points_mapper->points.size()<<endl;

    //特征点提取
    pcl::PointCloud<pcl::PointXYZI>::Ptr featurePoints(new pcl::PointCloud<pcl::PointXYZI>);
    //    curb_point curbExtract(ground_points_mapper,scanIDindices);
    curb_point curbExtract(ground_points_mapper,scanIDindices,normalRegionMin,normalThreshold,heightRegion,
                           heightSigmaThreshold,curvatureRegion,curvatureThreshold,distanceHorizonThreshold,
                           distanceVerticalThreshold);
    curbExtract.extractFeatures(featurePoints);
    cout<<"feature points is "<<featurePoints->points.size()<<endl;


    //    创建curb_refine对象
    curb_refine_ransac refinePoints(*featurePoints,varthreshold,fthreshold);
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
    //            *clusterPtr[0]=*cloudptrlist2[frame];
    //     左右curb点拟合
    pcl::PointXYZ pointInit;
    pointInit.x=pointInit.y=pointInit.z=0;
    vector<pcl::PointXYZ> lineLeft3D(50,pointInit);
    vector<pcl::PointXYZ> lineRight3D(50,pointInit);
    if(clusterPtr[0]->points.size()>6)
    {
        curbFit lineFitLeft(clusterPtr[0]);

        lineFitLeft.ransac_curve(clusterPtr[0],0.02,lineLeft3D,true);
    }
    if(clusterPtr[1]->points.size()>6)
    {
        curbFit lineFitRight(clusterPtr[1]);

        lineFitRight.ransac_curve(clusterPtr[1],0.02,lineRight3D,false);
    }

    //左右curb点拟合
    //    vector<double> lineLeft(2);
    //    vector<double> lineRight(2);
    //    pcl::ModelCoefficients::Ptr lineModelLeft(new pcl::ModelCoefficients);
    //    pcl::ModelCoefficients::Ptr lineModelRight(new pcl::ModelCoefficients);
    //    vector<pcl::PointXYZ> lineLeft3D(2);
    //    vector<pcl::PointXYZ> lineRight3D(2);
    //    curbFit lineFitLeft(clusterPtr[0]);
    //    lineFitLeft.process(lineLeft3D);
    //    curbFit lineFitRight(clusterPtr[1]);
    //    lineFitRight.process(lineRight3D);

    //可视化
    pcl::visualization::PCLVisualizer viewer("curb_viewer");
    int v1(0);
    int v2(0);

    //    viewer.createViewPort(0,0,0.5,1,v1);
    //    viewer.createViewPort(0.5,0,1,1,v2);

    viewer.setBackgroundColor(1,1,1);
    viewer.addCoordinateSystem(2.5);
    pcl::visualization::PointCloudColorHandler<pcl::PointXYZI>::Ptr handler_cloud(new pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI>(0, 0, 0));
    pcl::visualization::PointCloudColorHandler<pcl::PointXYZI>::Ptr handler_cloud1(new pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI>(0, 0, 0));

    //            pcl::visualization::PointCloudColorHandler<pcl::PointXYZI> ::Ptr handler_cloud(new pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>("intensity"));
    //            handler_cloud->setInputCloud(completeCloudMapper);
    //            viewer.addPointCloud(completeCloudMapper,*handler_cloud,"cloud",v1);
    handler_cloud1->setInputCloud(ground_points_no);
    viewer.addPointCloud(ground_points_no,*handler_cloud1,"cloud1");
    //            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5, "cloud",v1);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5, "cloud1",v1);
    //        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1, "cloud",v1);
    //        显示后处理的特征点
    //            *clusterPtr[0]=*ground_points_mapper;
    //            *clusterPtr[1]=*featurePoints;
    //    for (int i = 0; i<clusterPtr.size(); i++)
    //    {
    //        if(clusterPtr[i]->points.size()>0)
    //        {
    //            double r=(rand()%(256+1));
    //            double g=(rand()%(256+1));
    //            double b=(rand()%(256+1));
    //            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> handler(0 ,0, 255);
    //            //            pcl::visualization::PointCloudColorHandler<pcl::PointXYZI> ::Ptr handler(new pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>("intensity"));
    //            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> ::Ptr red_cloud(new pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI>(255,0,0));
    //            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> ::Ptr blue_cloud(new pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI>(	0,0,255));
    //            if(i==0)
    //            {
    //                handler=*red_cloud;
    //            }
    //            else
    //            {
    //                handler=*blue_cloud;
    //            }
    //            //            pcl::visualization::PointCloudColorHandler<pcl::PointXYZI>::Ptr handler_tempt(new pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>("intensity"));
    //            handler.setInputCloud(clusterPtr[i]);
    //            std::ostringstream oss;
    //            oss << i;

    //            viewer.addPointCloud(clusterPtr[i], handler, "cluster" + oss.str(),v2);
    //            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1,"cluster" + oss.str(),v2);
    //            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "cluster" + oss.str(),v2);
    //        }

    //    }

    //显示光束模型
    for (int i = 0; i < nearestPoints.size(); ++i)
    {
        pcl::PointXYZ p1(0,0,0);
        pcl::PointXYZ p2 =nearestPoints[i];
        std::ostringstream os;
        os << "beam_" << i;
        viewer.addLine<pcl::PointXYZ> (p1, p2, 0, 0, 1, os.str (),v1);
        viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,2,os.str(),v1);
    }

    //显示道路便捷拟合结果
    //                for (std::size_t i = 0; i < lineLeft3D.size () - 1; i++)
    //                {
    //                    pcl::PointXYZ &p1 =  lineLeft3D[i];
    //                    pcl::PointXYZ &p2 = lineLeft3D[i+1];
    //                    std::ostringstream os;
    //                    os << "lineLeft_" << i;
    //                    viewer.addLine<pcl::PointXYZ> (p1, p2, 0, 1, 0, os.str (),v1);
    //                    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,8,os.str(),v1);
    //                }

    //                for (std::size_t i = 0; i < lineRight3D.size () - 1; i++)
    //                {
    //                    pcl::PointXYZ &p1 =  lineRight3D[i];
    //                    pcl::PointXYZ &p2 = lineRight3D[i+1];
    //                    std::ostringstream os;
    //                    os << "lineRight_" << i;
    //                    viewer.addLine<pcl::PointXYZ> (p1, p2, 0, 1, 0, os.str (),v1);
    //                    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,6,os.str(),v1);
    //                }

    while(!viewer.wasStopped())
    {
        viewer.spinOnce();
    }

    Json::Value distanceFunction;
    for (int i = 0; i < distance_vec.size(); ++i)
    {
        distanceFunction["before filter"].append(distance_vec[i]);
        distanceFunction["after filter"].append(distance_vec_filtered[i]);
    }

    Json::StyledWriter sw;
    ofstream os;
    os.open("disanceData");
    os<<sw.write(distanceFunction);
    os.close();
    //显示距离函数
    //    vector<double> beamNum(distance_vec.size());
    //    for (int i = 0; i < beamNum.size(); ++i)
    //    {
    //        beamNum[i]=i;
    //    }
    //    pcl::visualization::PCLPlotter myplotter1("RealTimeFigure_our method");
    //    myplotter1.setBackgroundColor(1,1,1);
    //    myplotter1.setXRange(0,360);
    //    myplotter1.setYRange(0,1.1);
    //    myplotter1.setColorScheme(0);
    //    myplotter1.setWindowName("Distance Function");
    //    myplotter1.setXTitle("BeamNum");
    //    myplotter1.setYTitle("Distance");
    //    myplotter1.addPlotData(beamNum,distance_vec);
    //    myplotter1.plot();

    //    while(!myplotter1.wasStopped())
    //    {
    //        myplotter1.spinOnce(100);
    //    }
    //    pcl::visualization::PCLPlotter myplotter2("RealTimeFigure_our method");
    //    myplotter2.setBackgroundColor(1,1,1);
    //    myplotter2.setXRange(0,360);
    //    myplotter2.setYRange(0,1.1);
    //    myplotter2.setColorScheme(0);
    //    myplotter2.setWindowName("Distance Function Filtered");
    //    myplotter2.setXTitle("BeamNum");
    //    myplotter2.setYTitle("DistanceFiltered");
    //    myplotter2.addPlotData(beamNum,distance_vec_filtered);
    //    myplotter2.plot();

    //    while(!myplotter2.wasStopped())
    //    {
    //        myplotter2.spinOnce(100);
    //    }

    return 0;
}

//                viewer.addLine(lineLeft3D[0],lineLeft3D[1],"lineleft",v2);
//                viewer.addLine(lineRight3D[0],lineRight3D[1] ,"lineright",v2);
//                viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,2,"lineleft",v2);
//                viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,2,"lineright",v2);
//                viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0,1,0,"lineright",v2);
//                viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0,1,0,"lineleft",v2);







