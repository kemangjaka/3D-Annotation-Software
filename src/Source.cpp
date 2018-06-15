#include "../include/Header.h"
#include "../src/DrawPolygon.hpp"
#include "../src/visualizerConfig.hpp"
#include "../include/pointWaveLength.h"
#include "../include/picojson.h"
#include "../include/pointDefinition.h"


#define WINDOW_WIDTH 960
#define WINDOW_HEIGHT 540

bool fexists(const std::string& filename) {
	std::ifstream ifile(filename.c_str());
	return (bool)ifile;
}

int loadConfig(string json_file, string& dataset_dir, int& pot_id, bool& resume, bool& create_directory)
{

	ifstream ifs(json_file, ios::in);
	if (ifs.fail())
	{
		cerr << "failed to read " << json_file << endl;
		return -1;
	}
	const string json((istreambuf_iterator<char>(ifs)), istreambuf_iterator<char>());
	ifs.close();

	picojson::value v;
	string err = picojson::parse(v, json);
	if (err.empty() == false) {
		std::cerr << err << std::endl;
		return -1;
	}
	picojson::object& obj = v.get<picojson::object>();
	cout << "---load Config file---" << endl;
	cout << "Dataset dir : " << obj["dataset_dir"].get<string>() << endl;
	cout << "Annotate pot ID : " << obj["pot_id"].get<double>() << endl;
	cout << "resume or not : " << obj["resume"].get<bool>() << endl;
	cout << "---config information---" << endl;

	dataset_dir = obj["dataset_dir"].get<string>();
	pot_id = int(obj["pot_id"].get<double>());
	resume = obj["resume"].get<bool>();
	create_directory = obj["create_directory"].get<bool>();
	return 1;
}

void createDirectories(string root_dir, vector<string> files, int pot_idx)
{
	string dir_name = root_dir + "pot" + to_string(pot_idx);
	cout << "make directory " << dir_name << endl;
	_mkdir(dir_name.c_str());

	for (int f = 0; f < files.size(); f++)
	{
		string subdir_name = root_dir + "pot" + to_string(pot_idx) + "/" + files[f];
		_mkdir(subdir_name.c_str());
		cout << "create directory " << subdir_name << endl;
	}

}

int renderImage(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, cv::Mat& output)
{
	vtkSmartPointer<vtkRenderWindow> renderWindow = viewer->getRenderWindow();
	vtkSmartPointer<vtkWindowToImageFilter> windowToImageFilter = vtkSmartPointer<vtkWindowToImageFilter>::New();
	windowToImageFilter->SetInput(renderWindow);

	//get rgb-image
	windowToImageFilter->SetInputBufferTypeToRGB();
	windowToImageFilter->Update();
	vtkImageData* vtkRGBimage = windowToImageFilter->GetOutput();
	int dimsRGBImage[3];
	vtkRGBimage->GetDimensions(dimsRGBImage);
	cv::Mat cvImageRGB(dimsRGBImage[1], dimsRGBImage[0], CV_8UC3, vtkRGBimage->GetScalarPointer());
	cv::cvtColor(cvImageRGB, cvImageRGB, CV_RGB2BGR);
	cv::flip(cvImageRGB, cvImageRGB, 0);
	//cv::imshow("", cvImageRGB);
	cvImageRGB.copyTo(output);

	return 1;
}

int renderCloud(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, PointCloud<PointT>::Ptr& cloud_out)
{
	vtkSmartPointer<vtkRenderWindow> win_ = viewer->getRenderWindow();

	vtkSmartPointer<vtkRendererCollection> rens_ = viewer->getRendererCollection();
	win_->SetSize(WINDOW_WIDTH, WINDOW_HEIGHT);
	win_->Render();
	float dwidth = 2.0 / float(WINDOW_WIDTH);
	float dheight = 2.0 / float(WINDOW_HEIGHT);
	cloud_out->points.resize(WINDOW_WIDTH * WINDOW_HEIGHT);
	cloud_out->width = WINDOW_WIDTH;
	cloud_out->height = WINDOW_HEIGHT;
	float *depth = new float[WINDOW_WIDTH * WINDOW_HEIGHT];
	win_->GetZbufferData(0, 0, WINDOW_WIDTH - 1, WINDOW_HEIGHT - 1, &(depth[0]));
	vtkRenderer *ren = rens_->GetFirstRenderer();
	vtkCamera *camera = ren->GetActiveCamera();
	vtkSmartPointer<vtkMatrix4x4> composite_projection_transform = camera->GetCompositeProjectionTransformMatrix(ren->GetTiledAspectRatio(), 0, 1);

	Eigen::Matrix4f mat1, mat2;
	for (int i = 0; i < 4; ++i)
		for (int j = 0; j < 4; ++j)
			mat1(i, j) = static_cast<float> (composite_projection_transform->Element[i][j]);

	mat1 = mat1.inverse().eval();

	int ptr = 0;
	for (int y = 0; y < WINDOW_HEIGHT; ++y)
	{
		for (int x = 0; x < WINDOW_WIDTH; ++x, ++ptr)
		{
			pcl::PointXYZ &pt = (*cloud_out)[ptr];

			if (depth[ptr] == 1.0)
			{
				pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN();
				continue;
			}

			Eigen::Vector4f world_coords(dwidth  * float(x) - 1.0f,
				dheight * float(y) - 1.0f,
				depth[ptr],
				1.0f);
			world_coords = mat1 * world_coords;

			float w3 = 1.0f / world_coords[3];
			world_coords[0] *= w3;
			world_coords[1] *= w3;
			world_coords[2] *= w3;

			pt.x = static_cast<float> (world_coords[0]);
			pt.y = static_cast<float> (world_coords[1]);
			pt.z = static_cast<float> (world_coords[2]);
		}
	}
	return 1;
}

int generateWaveLengthCloud(PointCloud<PointWaveLength>::Ptr labeled, PointCloud<PointT>::Ptr rendered_cloud, vector<cv::Point> maskedData, int leaf_idx, PointCloud<PointWaveLength>::Ptr& cloud_segmented)
{

	pcl::KdTreeFLANN<PointWaveLength> kdtree;
	kdtree.setInputCloud(labeled);

	for (int i = 0; i < maskedData.size(); i++)
	{
		int x = maskedData[i].x;
		int y = WINDOW_HEIGHT - maskedData[i].y;
		PointT p = rendered_cloud->at(x, y);
		if (isnan(p.x) || isnan(p.y) || isnan(p.z))
			continue;
		std::vector<int> pointIdxNKNSearch(1);
		std::vector<float> pointNKNSquaredDistance(1);
		kdtree.nearestKSearchT(p, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
		PointWaveLength px = labeled->points[pointIdxNKNSearch[0]];
		px.label = leaf_idx;
		cloud_segmented->push_back(px);
	}
	return 1;
}

void loadPotCloud(string pros_dir, string file, int pot_id, PointCloud<PointWaveLength>::Ptr& labeled)
{
	PointCloud<PointWaveLength>::Ptr raw_cloud(new PointCloud<PointWaveLength>);
	io::loadPCDFile(pros_dir + file + ".pcd", *raw_cloud);
	for (int p = 0; p < raw_cloud->size(); p++)
		if (raw_cloud->points[p].label == pot_id)
			labeled->push_back(raw_cloud->points[p]);
}

void loadPrevPotCloud(string annotation_dir, string pros_dir, string file, int pot_id, int leaf_id, PointCloud<PointWaveLength>::Ptr& labeled)
{
	if (!fexists(pros_dir + file + ".pcd"))
	{
		cout << "file " << pros_dir + file + ".pcd" << " not found!" << endl;
		return;
	}
	PointCloud<PointWaveLength>::Ptr raw_cloud(new PointCloud<PointWaveLength>);
	io::loadPCDFile(pros_dir + file + ".pcd", *raw_cloud);
	for (int p = 0; p < raw_cloud->size(); p++)
		if (raw_cloud->points[p].label == pot_id)
		{
			raw_cloud->points[p].label = 0;
			labeled->push_back(raw_cloud->points[p]);
		}
	PointCloud<PointWaveLength>::Ptr leaf_cloud(new PointCloud<PointWaveLength>);
	string leaf_dir = annotation_dir + "pot" + to_string(pot_id) + "/" + file + "/" + to_string(leaf_id) + ".pcd";
	if (!fexists(leaf_dir))
		return;
	io::loadPCDFile(leaf_dir, *leaf_cloud);
	for (int p = 0; p < leaf_cloud->size(); p++)
	{
		leaf_cloud->points[p].label = 1;
		labeled->push_back(leaf_cloud->points[p]);
	}


}


int main(int argc, char **argv)
{
	string config_path = "../config.json";
	string root_dir;
	int pot_idx;
	bool resume;
	bool create_directory;
	auto ret = loadConfig(config_path, root_dir, pot_idx, resume, create_directory);
	if (ret == -1)
		return -1;


	string pros_dir = root_dir + "preprocessed/";
	string annotation_dir = root_dir + "labeled/";
	vector<string> files;
	ifstream ifs(root_dir + "file_list.txt");
	string str;
	while (getline(ifs, str))
	{
		files.push_back(str);
	}
	cout << "Annotate to " << files.size() << " files" << endl;
	if (create_directory)
		createDirectories(annotation_dir, files, pot_idx);
	int leaf_idx = 0;
	for (auto idx = 0; idx < files.size(); idx++)
	{
		if (!fexists(pros_dir + files[idx] + ".pcd"))
		{
			cout << "file " << pros_dir + files[idx] << ".pcd" << " not found!" << endl;
			continue;
		}
		cout << "read... " << files[idx] << endl;
		PointCloud<PointWaveLength>::Ptr labeled(new PointCloud<PointWaveLength>);
		loadPotCloud(pros_dir, files[idx], pot_idx, labeled);
		VisualizerConfig vis(WINDOW_WIDTH, WINDOW_HEIGHT, labeled, "Current 3D Viewer");
		vis.vis_setup(pot_idx, 1);


		PointCloud<PointWaveLength>::Ptr prev(new PointCloud<PointWaveLength>);
		if(idx - 1 >= 0)
			loadPrevPotCloud(annotation_dir, pros_dir, files[idx - 1], pot_idx, leaf_idx, prev);
		VisualizerConfig prev_vis(WINDOW_WIDTH, WINDOW_HEIGHT, prev, "Previous 3D Viewer");
		if (prev->size() != 0)
			prev_vis.vis_setup(pot_idx, 0);

		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
		boost::shared_ptr<pcl::visualization::PCLVisualizer> prev_viewer;
		viewer = vis.getVis();
		viewer->setPosition(0, 0);
		prev_viewer = prev_vis.getVis();
		prev_viewer->setPosition(0, WINDOW_HEIGHT);
		while (!viewer->wasStopped())
		{
			viewer->spinOnce(1);
			prev_viewer->spinOnce(1);
		}
		pcl::PointCloud<PointT>::Ptr cloud_out(new pcl::PointCloud<PointT>());
		renderCloud(viewer, cloud_out);
		cv::Mat cvImageRGB;
		renderImage(viewer, cvImageRGB);

		PolygonDrawer pd("Polygon Viewer", cvImageRGB, WINDOW_WIDTH, WINDOW_HEIGHT);
		pd.run();
		vector<cv::Point> maskedData = pd.getMaskData();
		PointCloud<PointWaveLength>::Ptr cloud_segmented(new PointCloud<PointWaveLength>);
		generateWaveLengthCloud(labeled, cloud_out, maskedData, leaf_idx, cloud_segmented);

		string output_dir = annotation_dir + "pot" + to_string(pot_idx) + "/" + files[idx];
		string output_file = output_dir + "/" + to_string(leaf_idx) + ".pcd";
		
		if (GetFileAttributes(output_dir.c_str()) == INVALID_FILE_ATTRIBUTES)
		{
			cerr << output_file << " does not exist!" << endl;
			return -1;
		}
		cout << "save to... " << output_file << endl;
		pcl::io::savePCDFileBinaryCompressed(output_file, *cloud_segmented);

		viewer->close();
		prev_viewer->close();
		cv::destroyAllWindows();
	}


	return 1;
}

