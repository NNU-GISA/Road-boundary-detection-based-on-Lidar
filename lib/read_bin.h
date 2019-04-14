#ifndef READ_BIN_H
#define READ_BIN_H

#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<vector>
#include<string>
#include<dirent.h>
#include<fstream>

using namespace std;

typedef vector<pcl::PointCloud<pcl::PointXYZI>::Ptr > CloudPtrList;



class read_bin
{
public:
    read_bin(string bin_path,string pcd_path="./");
    void read_filelists(const std::string& dir_path,std::vector<std::string>& out_filelsits,std::string type);
    void sort_filelists(std::vector<std::string>& filists,std::string type);
    void readKittiPclBinData(std::string &in_file, std::string& out_file,pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    void my_function(CloudPtrList cloud_list);
    void process(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,int i);
    int getNumberofFile();

private:
    vector<string> _file_lists;
    string _bin_path;
    string _pcd_path;
};
#endif // READ_BIN_H
