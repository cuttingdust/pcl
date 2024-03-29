
#include<pcl/io/io.h>
#include<pcl/io/pcd_io.h>
#include<pcl/io/ply_io.h>
#include<pcl/visualization/cloud_viewer.h>

#include<iostream>
#include<thread>

int user_data;
 
void viewerOneOff(pcl::visualization::PCLVisualizer& viewer) {
	viewer.setBackgroundColor(1.0, 0.5, 1.0);   //设置背景颜色
}
 
int main() {
char strfilepath[256] = "../rabbit.pcd";

#if 0
  /// # windows 下 随意
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	if (-1 == pcl::io::loadPCDFile(strfilepath, *cloud)) {
		cout << "error input!" << endl;
		return -1;
	}

	cout << cloud->points.size() << endl;
	pcl::visualization::CloudViewer viewer("Cloud Viewer");   //创建viewer对象

	viewer.showCloud(cloud);
	viewer.runOnVisualizationThreadOnce(viewerOneOff);
	system("pause");
#else
    /// mac下仍然可以运行
    // 加载点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> (strfilepath, *cloud) == -1) {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }
    std::cout << "Loaded "
              << cloud->width * cloud->height
              << " data points from test_pcd.pcd with the following fields: "
              << std::endl;

    // 创建PCLVisualizer对象
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

    // 设置背景颜色为黑色（也可以选择其他颜色）
    viewer->setBackgroundColor (0, 0, 0);

    // 添加点云数据
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");

    // 设置点云的渲染属性（这里我们设置点的大小为1）
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

    // 添加坐标轴（可选）
    viewer->addCoordinateSystem (1.0);

    // 设置相机的初始位置
    viewer->initCameraParameters ();

    // 主循环
    while (!viewer->wasStopped ()) {
        viewer->spinOnce (100);  // 100毫秒刷新一次
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
#endif

    return 0;
}