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
			//�洢ѹ�����Ƶ��ֽ���
			std::stringstream compressedData;
			//�������
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZRGBA>());
			//ѹ������
			PointCloudEncoder->encodePointCloud(cloud, compressedData);
			//��ѹ������
			PointCloudDecoder->decodePointCloud(compressedData, cloudOut);
			//���ӻ���ѹ������
			viewer.showCloud(cloudOut);
		}
	}
	void run()
	{
		bool showStatistics = true;
		pcl::io::compression_Profiles_e compressionProfile = pcl::io::MED_RES_ONLINE_COMPRESSION_WITHOUT_COLOR;
		//��ʼ��ѹ�����ѹ������
		PointCloudEncoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>(compressionProfile, showStatistics);
		PointCloudDecoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>();
		//������openNI��ȡ���ƵĶ���
		pcl::Grabber *interface = new pcl::io::OpenNI2Grabber;
		//�����ص�����
		boost::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &)> f = boost::bind(&SimpleOpenNIViewer::cloud_cb_, this, _1);
		//�����ص�������ص��ź�֮�����ϵ
		boost::signals2::connection c = interface->registerCallback(f);
		//��ʼ���ܵ���������
		interface->start();
		while (!viewer.wasStopped())
		{
			sleep(1);
		}
		interface->stop();
		//ɾ������ѹ�����ѹ������ʵ��
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