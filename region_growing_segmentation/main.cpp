#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/segmentation/region_growing.h>

#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>


#include <cstdio>
#include <iostream>
#include <memory>
#include <thread>

// 对于 Windows 系统
#ifdef _WIN32
#include <windows.h>
#include <psapi.h>

void PrintMemoryInfo() {
    PROCESS_MEMORY_COUNTERS pmc;
    HANDLE hProcess = GetCurrentProcess();
    if (GetProcessMemoryInfo(hProcess, &pmc, sizeof(pmc))) {
        printf("PageFaultCount: %u\n", pmc.PageFaultCount);
        printf("PeakWorkingSetSize: %u\n", (unsigned int)pmc.PeakWorkingSetSize);
        printf("WorkingSetSize: %u\n", (unsigned int)pmc.WorkingSetSize);
        // 添加更多打印信息...
    }
}

// 对于 macOS 系统
#elif defined(__APPLE__)
#include <mach/mach.h>

void PrintMemoryInfo() {
    struct mach_task_basic_info info;
    mach_msg_type_number_t infoCount = MACH_TASK_BASIC_INFO_COUNT;
    if (task_info(mach_task_self(), MACH_TASK_BASIC_INFO, (task_info_t)&info, &infoCount) == KERN_SUCCESS) {
        printf("Virtual Size: %llu MB\n", info.virtual_size / 1024 / 1024);
        printf("Resident Size: %llu MB\n", info.resident_size / 1024 / 1024);
        // 添加更多打印信息...
    }
}

// 如果不是上述两个系统，则不做任何事
#else
void PrintMemoryInfo() {
    printf("Unsupported platform.\n");
}
#endif


int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        std::cout << ".exe xx.pcd -kn 50 -bc 0 -fc 10.0 -nc 0 -st 30 -ct 0.05" << std::endl;

        return 0;
    } ///如果输入参数小于1个，输出提示
    time_t start, end, diff[5], option;
    start = time(0);
    int KN_normal = 50;                 /// 设置默认输入参数
    bool Bool_Cuting = false;           /// 设置默认输入参数
    float far_cuting = 10, near_cuting = 0, SmoothnessThreshold = 30.0, CurvatureThreshold = 0.05;  /// 设置默认输入参数
    pcl::console::parse_argument(argc, argv, "-kn", KN_normal);
    pcl::console::parse_argument(argc, argv, "-bc", Bool_Cuting);
    pcl::console::parse_argument(argc, argv, "-fc", far_cuting);
    pcl::console::parse_argument(argc, argv, "-nc", near_cuting);
    pcl::console::parse_argument(argc, argv, "-st", SmoothnessThreshold);
    pcl::console::parse_argument(argc, argv, "-ct", CurvatureThreshold);                     /// 设置输入参数方式

    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) == -1)
    {
        std::cout << "cloud reading faild" << std::endl;
        return (-1);
    }   /// 加载点云数据
    end = time(0);
    diff[0] = difftime(end, start);
    PCL_INFO("\Loading pcd file takes(seconds): %d\n", diff[0]);

    /// Normal estimation step(1 parameter)
    pcl::search::Search<pcl::PointXYZ>::Ptr tree = std::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>); /// 创建一个指向kd树搜索对象的共享指针
    pcl::PointCloud<pcl::Normal>::Ptr  normals(new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
    normal_estimation.setSearchMethod(tree);
    normal_estimation.setInputCloud(cloud);
    normal_estimation.setKSearch(50);
    normal_estimation.compute(*normals);

    pcl::IndicesPtr indices(new std::vector<int>);
    pcl::removeNaNFromPointCloud(*cloud, *indices);

    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize(50);
    reg.setMaxClusterSize(1000000);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(30);
    reg.setInputCloud(cloud);
    reg.setIndices(indices);
    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold(3.f / 180.f * M_PI);
    reg.setCurvatureThreshold(1.0f);

    std::vector<pcl::PointIndices> clusters;
    reg.extract(clusters);

    std::cout << "Numbef of clusters is equal to" << clusters.size() << std::endl;
    std::cout << "First cluster has "<< clusters[0].indices.size() << "points." << std::endl;
    std::cout << "These are the indices og the points of the initial" << std::endl
    << "cloud that belong to the first cluster: "<< std::endl;

    std::size_t counter = 0;
    while (counter < clusters[0].indices.size()) {
      std::cout << clusters[0].indices[counter] << ",";
      counter++;
      if (0 == counter % 10) {
        std::cout << std::endl;
      }

    std::cout << std::endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
//    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("3D vIewer"));
//
//    viewer->setBackgroundColor(0, 0, 0);
//    viewer->addPointCloud<pcl::PointXYZRGB>(colored_cloud, "sample cloud");
//    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
//    viewer->addCoordinateSystem (1.0);
//    viewer->initCameraParameters ();
//
//    PrintMemoryInfo();
//    /// 主循环
//    while (!viewer->wasStopped ()) {
//        viewer->spinOnce (100);  // 100毫秒刷新一次
//        std::this_thread::sleep_for(std::chrono::milliseconds(100));
//    }

	pcl::visualization::CloudViewer viewer ("���������ָ��");
	viewer.showCloud(colored_cloud);
	while (!viewer.wasStopped ())
	{
	}//���п��ӻ�

  }


    return 0;
}

