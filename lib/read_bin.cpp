#include "read_bin.h"


bool computePairNum_bin(std::string pair1,std::string pair2)
{
    return pair1 < pair2;
}


read_bin::read_bin(string bin_path,string pcd_path):_bin_path(bin_path),_pcd_path(pcd_path)
{
   read_filelists( _bin_path, _file_lists, "bin" );
   sort_filelists( _file_lists, "bin" );
}

 int read_bin::getNumberofFile()
 {
     return _file_lists.size();
 }
void read_bin::read_filelists(const std::string& dir_path,std::vector<std::string>& out_filelsits,std::string type)
{
    struct dirent *ptr;
    DIR *dir;
    dir = opendir(dir_path.c_str());
    out_filelsits.clear();
    while ((ptr = readdir(dir)) != NULL)
    {
        std::string tmp_file = ptr->d_name;
        if (tmp_file[0] == '.')
            continue;

        if (type.size() <= 0)
        {
            out_filelsits.push_back(ptr->d_name);
        }
        else
        {
            if (tmp_file.size() < type.size())
                continue;
            std::string tmp_cut_type = tmp_file.substr(tmp_file.size() - type.size(),type.size());
            if (tmp_cut_type == type)
            {
                out_filelsits.push_back(ptr->d_name);
            }
        }
    }
    closedir(dir);
}



void read_bin::sort_filelists(std::vector<std::string>& filists,std::string type)
{
    if (filists.empty())return;
    std::sort(filists.begin(),filists.end(),computePairNum_bin);
}

void read_bin::readKittiPclBinData(std::string &in_file, std::string& out_file,pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{

    std::fstream input(in_file.c_str(), std::ios::in | std::ios::binary);
    if(!input.good()){
        std::cerr << "Could not read file: " << in_file << std::endl;
        exit(EXIT_FAILURE);
    }
    input.seekg(0, std::ios::beg);

    int i;
    for (i=0; input.good() && !input.eof(); i++) {
        pcl::PointXYZI point;
        input.read((char *) &point.x, 3*sizeof(float));
        input.read((char *) &point.intensity, sizeof(float));
        cloud->push_back(point);
    }
    input.close();

}

void read_bin::my_function(CloudPtrList cloud_list)
{


//    sort_filelists( _file_lists, "bin" );
    for (int i = 0; i < _file_lists.size(); ++i)
    {
        std::string bin_file = _bin_path + _file_lists[i];
        std::string tmp_str = _file_lists[i].substr(0, _file_lists[i].length() - 4) + ".pcd";
        std::string pcd_file = _pcd_path + tmp_str;
        readKittiPclBinData( bin_file, pcd_file,cloud_list[i]);
//        cout<<" frame "<<i<<" point cloud "<<endl;
    }
}
void read_bin::process(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,int i)
{
        std::string bin_file = _bin_path + _file_lists[i];
        std::string tmp_str = _file_lists[i].substr(0, _file_lists[i].length() - 4) + ".pcd";
        std::string pcd_file = _pcd_path + tmp_str;
        readKittiPclBinData( bin_file, pcd_file,cloud);
       cout<<" 当前图像为 "<<_file_lists[i]<<endl;

}
