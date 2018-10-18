#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni2_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <stdio.h>
#include <sstream>
#include <stdlib.h>

#ifdef WIN32
#define sleep(x) Sleep((x)*1000)
#endif // WIN32

class SimpleOpenNIViewer
{
public:
	SimpleOpenNIViewer():viewer("Point Cloud Compression Example")
	{

	}
	void cloud_cb_(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud)
	{
		if (!viewer.wasStopped())
		{
			//存储压缩点云的字节流
			std::stringstream compressedData;
			//输出点云
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZRGBA>());
			//压缩点云
			PointCloudEncoder->encodePointCloud(cloud, compressedData);
			//解压缩点云
			PointCloudDecoder->decodePointCloud(compressedData, cloudOut);
			//可视化解压缩点云
			viewer.showCloud(cloudOut);
		}
	}
	void run()
	{
		bool showStatistics = true;
		pcl::io::compression_Profiles_e compressionProfile = pcl::io::MED_RES_ONLINE_COMPRESSION_WITHOUT_COLOR;
		//初始化压缩与解压缩对象
		PointCloudEncoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>(compressionProfile, showStatistics);
		PointCloudDecoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>();
		//创建从openNI获取点云的对象
		pcl::Grabber *interface = new pcl::io::OpenNI2Grabber;
		//建立回调函数
		boost::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &)> f = boost::bind(&SimpleOpenNIViewer::cloud_cb_, this, _1);
		//建立回调函数与回调信号之间的联系
		boost::signals2::connection c = interface->registerCallback(f);
		//开始接受点云数据流
		interface->start();
		while (!viewer.wasStopped())
		{
			sleep(1);
		}
		interface->stop();
		//删除点云压缩与解压缩对象实例
		delete (PointCloudEncoder);
		delete (PointCloudDecoder);
	}
	pcl::visualization::CloudViewer viewer;
	pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA> *PointCloudEncoder;
	pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA> *PointCloudDecoder;
};

int main(int argc, char** argv)
{
	SimpleOpenNIViewer v;
	v.run();
	return 0;
}