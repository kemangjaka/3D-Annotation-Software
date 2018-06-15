#include "../include/pointWaveLength.h"
#include "../include/pointDefinition.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
using namespace pcl;
using namespace std;

class VisualizerConfig {
public:

	VisualizerConfig(int w_width, int w_height, PointCloud<PointWaveLength>::Ptr cloud, string window_name)
	{
		vis_.reset(new pcl::visualization::PCLVisualizer(window_name.c_str()));
		window_width = w_width;
		window_height = w_height;
		vis_cloud.reset(new pcl::PointCloud<PointWaveLength>);
		copyPointCloud(*cloud, *vis_cloud);
	}

	

	void vis_setup(int pot_id, int vis_mode)
	{
		vis_->setShowFPS(false);
		vis_->setSize(window_width, window_height);
		pcl::visualization::Camera cam;
		string cam_file = "../camfiles/pot" + to_string(pot_id) + ".cam";
		cout << cam_file << endl;
		vis_->loadCameraParameters(cam_file);
		if (vis_mode == 0)
			visLabel();
		else if (vis_mode == 1)
			visNIR();
		vis_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
		vis_->setBackgroundColor(1.0, 1.0, 1.0);
		vis_->registerKeyboardCallback(&VisualizerConfig::keyboardEventOccurred, *this);
		return;
	}
	boost::shared_ptr<pcl::visualization::PCLVisualizer> getVis()
	{
		return vis_;
	}
private:
	boost::shared_ptr<visualization::PCLVisualizer> vis_;
	PointCloud<PointWaveLength>::Ptr vis_cloud;
	int window_width;
	int window_height;

	void keyboardEventOccurred(const visualization::KeyboardEvent &event, void* param)
	{
		if (event.getKeySym() == "l" && event.keyDown())
		{
			std::cout << "l was pressed => change to label visualization" << std::endl;
			visLabel();
		}
		else if (event.getKeySym() == "c" && event.keyDown())
		{
			std::cout << "c was pressed => change to color visualization" << std::endl;
			visRGB();
		}
		else if (event.getKeySym() == "i" && event.keyDown())
		{
			std::cout << "i was pressed => change to NIR visualization" << std::endl;
			visNIR();
		}
	}

	void visLabel()
	{
		pcl::PointCloud<PointLT>::Ptr cloud_label(new pcl::PointCloud<pcl::PointXYZL>);
		for (int i = 0; i < vis_cloud->size(); i++)
		{
			PointWaveLength p = vis_cloud->points[i];
			PointLT px;
			px.x = p.x;
			px.y = p.y;
			px.z = p.z;
			px.label = p.label;
			cloud_label->push_back(px);
		}
		visualization::PointCloudColorHandlerLabelField<PointLT> label_distribution(cloud_label, "label");
		vis_->removeAllPointClouds();
		vis_->addPointCloud<PointLT>(cloud_label->makeShared(), label_distribution, "cloud");
	}

	void visRGB()
	{
		pcl::PointCloud<PointCT>::Ptr rgb(new pcl::PointCloud<PointCT>);
		for (int i = 0; i < vis_cloud->size(); i++)
		{
			PointWaveLength p = vis_cloud->points[i];
			PointCT px;
			px.x = p.x;
			px.y = p.y;
			px.z = p.z;
			px.r = p.r;
			px.g = p.g;
			px.b = p.b;
			rgb->push_back(px);
		}
		visualization::PointCloudColorHandlerRGBField<PointCT> color_distribution(rgb);
		vis_->removeAllPointClouds();
		vis_->addPointCloud<PointCT>(rgb->makeShared(), color_distribution, "cloud");
	}

	void visNIR()
	{
		pcl::PointCloud<PointIT>::Ptr intensity(new pcl::PointCloud<PointIT>);
		for (int i = 0; i < vis_cloud->size(); i++)
		{
			PointWaveLength p = vis_cloud->points[i];
			PointIT px;
			px.x = p.x;
			px.y = p.y;
			px.z = p.z;
			px.intensity = p.nir;
			intensity->push_back(px);
		}
		visualization::PointCloudColorHandlerGenericField<PointIT> intensity_distribution(intensity, "intensity");
		vis_->removeAllPointClouds();
		vis_->addPointCloud<PointIT>(intensity->makeShared(), intensity_distribution, "cloud");
	}

};
