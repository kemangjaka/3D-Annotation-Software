#include "../include/pointWaveLength.h"
#include "../include/pointDefinition.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
using namespace pcl;
using namespace std;

class VisualizerConfig {
public:

	VisualizerConfig(int w_width, int w_height, PointCloud<PointWaveLength>::Ptr cloud, PointCloud<PointWaveLength>::Ptr leaves, string window_name)
	{
		vis_.reset(new pcl::visualization::PCLVisualizer(window_name.c_str()));
		window_width = w_width;
		window_height = w_height;
		vis_cloud.reset(new pcl::PointCloud<PointWaveLength>);
		vis_leaf.reset(new pcl::PointCloud<PointWaveLength>);
		unlabeled_cloud.reset(new pcl::PointCloud<PointWaveLength>);
		copyPointCloud(*cloud, *vis_cloud);
		copyPointCloud(*leaves, *vis_leaf);
		kdtree.setInputCloud(vis_leaf);
	}

	

	void vis_setup(int pid, int vis_mode)
	{
		pot_id = pid;
		vis_type = vis_mode;
		vis_->setShowFPS(false);
		vis_->setSize(window_width, window_height);
		pcl::visualization::Camera cam;
		string cam_file = "../Annotation/camfiles/pot" + to_string(pot_id) + ".cam";
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

	void setText(int leaf_idx)
	{
		vis_->addText("Leaf Index: " + to_string(leaf_idx), 0, window_height - 10, "text");
	}

	boost::shared_ptr<pcl::visualization::PCLVisualizer> getVis()
	{
		return vis_;
	}

	void prepareRender(boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer)
	{
		viewer->updateText("", 0, 0, "text");
		viewer->removeAllPointClouds();
		if (vis_type == 0)
			RenderLabel(viewer);
		else if (vis_type == 1)
			RenderNIR(viewer);
		else if (vis_type == 2)
			RenderRGB(viewer);

	}

	PointCloud<PointWaveLength>::Ptr getReLabeledCloud()
	{
		return unlabeled_cloud;
	}

private:
	boost::shared_ptr<visualization::PCLVisualizer> vis_;
	PointCloud<PointWaveLength>::Ptr vis_cloud;
	PointCloud<PointWaveLength>::Ptr unlabeled_cloud;
	PointCloud<PointWaveLength>::Ptr vis_leaf;
	pcl::KdTreeFLANN<PointWaveLength> kdtree;
	int window_width;
	int window_height;
	int vis_type;
	int pot_id;
	void keyboardEventOccurred(const visualization::KeyboardEvent &event, void* param)
	{
		if (event.getKeySym() == "L" && event.keyDown())
		{
			std::cout << "l was pressed => change to label visualization" << std::endl;
			visLabel();
			vis_type = 0;
		}
		else if (event.getKeySym() == "C" && event.keyDown())
		{
			std::cout << "c was pressed => change to color visualization" << std::endl;
			visRGB();
			vis_type = 2;
		}
		else if (event.getKeySym() == "I" && event.keyDown())
		{
			std::cout << "i was pressed => change to NIR visualization" << std::endl;
			visNIR();
			vis_type = 1;
		}
	}

	bool isLeafExist(PointWaveLength px)
	{
		std::vector<int> pointIdxNKNSearch(1);
		std::vector<float> pointNKNSquaredDistance(1);
		kdtree.nearestKSearchT(px, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
		if (pointNKNSquaredDistance[0] < 1e-5)
			return true;
		else
			return false;
	}

	void visLabel()
	{
		vis_->removeAllPointClouds();
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
		pcl::PointCloud<PointLT>::Ptr leaf_label(new pcl::PointCloud<pcl::PointXYZL>);
		for (int i = 0; i < vis_leaf->size(); i++)
		{
			PointWaveLength p = vis_leaf->points[i];
			PointLT px;
			px.x = p.x;
			px.y = p.y;
			px.z = p.z;
			px.label = p.label;
			leaf_label->push_back(px);
		}

		visualization::PointCloudColorHandlerLabelField<PointLT> label_distribution(cloud_label, "label");
		vis_->addPointCloud<PointLT>(cloud_label->makeShared(), label_distribution, "cloud");

		visualization::PointCloudColorHandlerLabelField<PointLT> leaf_distribution(leaf_label, "label1");
		vis_->addPointCloud<PointLT>(leaf_label->makeShared(), leaf_distribution, "leaf");
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

	void RenderLabel(boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer)
	{
		pcl::PointCloud<PointLT>::Ptr cloud_label(new pcl::PointCloud<pcl::PointXYZL>);
		for (int i = 0; i < vis_cloud->size(); i++)
		{
			PointWaveLength p = vis_cloud->points[i];
			if (isLeafExist(p))
				continue;
			//if (p.label != pot_id)
			//	continue;
			PointLT px;
			px.x = p.x;
			px.y = p.y;
			px.z = p.z;
			px.label = p.label;
			cloud_label->push_back(px);
			unlabeled_cloud->push_back(p);
		}
		visualization::PointCloudColorHandlerLabelField<PointLT> label_distribution(cloud_label, "label");
		viewer->addPointCloud<PointLT>(cloud_label->makeShared(), label_distribution, "cloud");
	}

	void RenderRGB(boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer)
	{
		pcl::PointCloud<PointCT>::Ptr rgb(new pcl::PointCloud<PointCT>);
		for (int i = 0; i < vis_cloud->size(); i++)
		{
			PointWaveLength p = vis_cloud->points[i];
			if (isLeafExist(p))
				continue;
			//if (p.label != pot_id)
			//	continue;
			PointCT px;
			px.x = p.x;
			px.y = p.y;
			px.z = p.z;
			px.r = p.r;
			px.g = p.g;
			px.b = p.b;
			rgb->push_back(px);
			unlabeled_cloud->push_back(p);
		}
		visualization::PointCloudColorHandlerRGBField<PointCT> color_distribution(rgb);
		viewer->addPointCloud<PointCT>(rgb->makeShared(), color_distribution, "cloud");
	}

	void RenderNIR(boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer)
	{
		pcl::PointCloud<PointIT>::Ptr intensity(new pcl::PointCloud<PointIT>);
		for (int i = 0; i < vis_cloud->size(); i++)
		{
			PointWaveLength p = vis_cloud->points[i];
			if (isLeafExist(p))
				continue;
			//if (p.label != pot_id)
			//	continue;
			PointIT px;
			px.x = p.x;
			px.y = p.y;
			px.z = p.z;
			px.intensity = p.nir;
			intensity->push_back(px);
			unlabeled_cloud->push_back(p);
		}
		visualization::PointCloudColorHandlerGenericField<PointIT> intensity_distribution(intensity, "intensity");
		viewer->addPointCloud<PointIT>(intensity->makeShared(), intensity_distribution, "cloud");
	}

};
