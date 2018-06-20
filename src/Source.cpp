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

static vector<string> read(string folder) {
	vector<string> fileList;
	HANDLE hFind;
	WIN32_FIND_DATA fd;

	stringstream ss;
	ss << folder;
	string::iterator itr = folder.end();
	itr--;
	if (*itr != '\\') ss << '\\';
	ss << "*.*";

	hFind = FindFirstFile(ss.str().c_str(), &fd);
	if (hFind == INVALID_HANDLE_VALUE) {
		return fileList;
	}

	do {
		char *file = fd.cFileName;
		string str = file;
		if (str.size() > 2)
			fileList.push_back(str);
	} while (FindNextFile(hFind, &fd));
	FindClose(hFind);

	return fileList;
}

void setConsole()
{
	char TitleBuffer[512];
	HWND ConsoleWindow;
	RECT WindowRect;
	GetConsoleTitle(TitleBuffer, sizeof(TitleBuffer));
	ConsoleWindow = FindWindow(NULL, TitleBuffer);
	GetWindowRect(ConsoleWindow, &WindowRect);
	MoveWindow(ConsoleWindow, WINDOW_WIDTH, WINDOW_HEIGHT, WINDOW_WIDTH, WINDOW_HEIGHT, TRUE);

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

	Eigen::Matrix4f mat1;
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

int CloudReprojection(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, PointCloud<PointWaveLength>::Ptr labeled, cv::Mat maskImage, PointCloud<PointWaveLength>::Ptr& cloud_out, cv::Mat rgbRendered)
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

	Eigen::Matrix4f mat1;
	for (int i = 0; i < 4; ++i)
		for (int j = 0; j < 4; ++j)
			mat1(i, j) = static_cast<float> (composite_projection_transform->Element[i][j]);

	for (int p = 0; p < labeled->size(); p++)
	{
		Eigen::Vector4f world_coords(labeled->points[p].x,
			labeled->points[p].y,
			labeled->points[p].z,
			1.0);
		world_coords = mat1 * world_coords;
		int x = int((world_coords[0] / world_coords[3] + 1.0) / dwidth);
		int y = int((world_coords[1] / world_coords[3] + 1.0) / dheight);
		if (x >= 0 && x < WINDOW_WIDTH && y > 0 && y < WINDOW_HEIGHT)
		{
			rgbRendered.at<cv::Vec3b>(WINDOW_HEIGHT - y, x)[0] = (unsigned char)(labeled->points[p].r);
			rgbRendered.at<cv::Vec3b>(WINDOW_HEIGHT - y, x)[1] = (unsigned char)(labeled->points[p].g);
			rgbRendered.at<cv::Vec3b>(WINDOW_HEIGHT - y, x)[2] = (unsigned char)(labeled->points[p].b);
			if (maskImage.at<cv::Vec3b>(WINDOW_HEIGHT - y, x)[0] == 0 && maskImage.at<cv::Vec3b>(WINDOW_HEIGHT - y, x)[1] == 0 && maskImage.at<cv::Vec3b>(WINDOW_HEIGHT - y, x)[2] == 0)
				cloud_out->push_back(labeled->points[p]);
		}
	}

	return 1;
}

void loadCurrPotCloud(string pros_dir, string file, int pot_id, PointCloud<PointWaveLength>::Ptr& labeled)
{
	PointCloud<PointWaveLength>::Ptr raw_cloud(new PointCloud<PointWaveLength>);
	io::loadPCDFile(pros_dir + file + ".pcd", *raw_cloud);
	for (int p = 0; p < raw_cloud->size(); p++)
		if (raw_cloud->points[p].label == pot_id)
			labeled->push_back(raw_cloud->points[p]);
}

PointCloud<PointWaveLength>::Ptr loadCurrPotCloudAsync(string pros_dir, string file, int pot_id)
{
	cout << "Load current pot" << endl;
	PointCloud<PointWaveLength>::Ptr raw_cloud(new PointCloud<PointWaveLength>);
	PointCloud<PointWaveLength>::Ptr ret_cloud(new PointCloud<PointWaveLength>);
	io::loadPCDFile(pros_dir + file + ".pcd", *raw_cloud);
	for (int p = 0; p < raw_cloud->size(); p++)
		if (raw_cloud->points[p].label == pot_id)
			ret_cloud->push_back(raw_cloud->points[p]);

	return ret_cloud;
}

void loadLeaves(string annotation_dir, int pot_idx, string file, int leaf_idx, PointCloud<PointWaveLength>::Ptr& labeled)
{
	cout << "Load Leaves " << endl;
	pcl::KdTreeFLANN<PointWaveLength> kdtree;
	kdtree.setInputCloud(labeled);
	vector<int> random_label;
	for (int i = 0; i < leaf_idx; i++)
		random_label.push_back(rand() % 100 + 1000);

	for (int idx = leaf_idx - 1; idx > 0; idx--)
	{

		PointCloud<PointWaveLength>::Ptr leaf(new PointCloud<PointWaveLength>);
		string file_path = annotation_dir + "pot" + to_string(pot_idx) + "/" + file + "/" + to_string(idx) + ".pcd";
		io::loadPCDFile(file_path, *leaf);
		int r_label = random_label[idx];
		for (int p = 0; p < leaf->size(); p++)
		{
			PointWaveLength px = leaf->points[p];
			std::vector<int> pointIdxNKNSearch(1);
			std::vector<float> pointNKNSquaredDistance(1);
			kdtree.nearestKSearchT(px, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
			labeled->points[pointIdxNKNSearch[0]].label = r_label;
		}
	}
}

PointCloud<PointWaveLength>::Ptr loadLeavesAsync(string annotation_dir, int pot_idx, string file, int leaf_idx)
{
	cout << "Load Leaves " << endl;
	PointCloud<PointWaveLength>::Ptr ret_cloud(new PointCloud<PointWaveLength>);
	if (pot_idx == -1)
		return ret_cloud;
	
	vector<int> random_label;
	for (int i = 0; i < leaf_idx; i++)
		random_label.push_back(rand() % 100 + 1000);

	for (int idx = leaf_idx - 1; idx > 0; idx--)
	{

		PointCloud<PointWaveLength>::Ptr leaf(new PointCloud<PointWaveLength>);
		string file_path = annotation_dir + "pot" + to_string(pot_idx) + "/" + file + "/" + to_string(idx) + ".pcd";
		io::loadPCDFile(file_path, *leaf);
		int r_label = random_label[idx];
		for (int p = 0; p < leaf->size(); p++)
		{
			PointWaveLength px = leaf->points[p];
			px.label = r_label;
			ret_cloud->push_back(px);
		}
	}

	return ret_cloud;
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
		leaf_cloud->points[p].label = leaf_id;
		labeled->push_back(leaf_cloud->points[p]);
	}
}

PointCloud<PointWaveLength>::Ptr loadPrevPotCloudAsync(string annotation_dir, string pros_dir, string file, int pot_id, int leaf_id)
{
	cout << "load Previous pot" << endl;
	PointCloud<PointWaveLength>::Ptr ret_cloud(new PointCloud<PointWaveLength>);
	if (pot_id == -1)
		return ret_cloud;
	if (!fexists(pros_dir + file + ".pcd"))
	{
		cout << "file " << pros_dir + file + ".pcd" << " not found!" << endl;
		return ret_cloud;
	}

	PointCloud<PointWaveLength>::Ptr raw_cloud(new PointCloud<PointWaveLength>);
	io::loadPCDFile(pros_dir + file + ".pcd", *raw_cloud);
	for (int p = 0; p < raw_cloud->size(); p++)
		if (raw_cloud->points[p].label == pot_id)
		{
			raw_cloud->points[p].label = 0;
			ret_cloud->push_back(raw_cloud->points[p]);
		}

	return ret_cloud;
}

void loadPreviousLeaf(string annotation_dir, string pros_dir, string file, int pot_id, int leaf_id, PointCloud<PointWaveLength>::Ptr& labeled)
{
	cout << "loadPreviousLeaf" << endl;
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

//すでにAnnotation済ｎ
void replaceIntoLeaveLabels(PointCloud<PointWaveLength>::Ptr& labeled, PointCloud<PointWaveLength>::Ptr leaves)
{
	cout << "replaceIntoLeaveLabels" << endl;
	pcl::KdTreeFLANN<PointWaveLength> kdtree;
	kdtree.setInputCloud(labeled);

	for (int pdx = 0; pdx < leaves->size(); pdx++)
	{
		PointWaveLength px = leaves->points[pdx];
		std::vector<int> pointIdxNKNSearch(1);
		std::vector<float> pointNKNSquaredDistance(1);
		kdtree.nearestKSearchT(px, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
		labeled->points[pointIdxNKNSearch[0]].label = leaves->points[pdx].label;
	}
}

PointCloud<PointWaveLength>::Ptr replaceIntoLeaveLabelsAsync(PointCloud<PointWaveLength>::Ptr labeled, PointCloud<PointWaveLength>::Ptr leaves)
{
	cout << "replaceIntoLeaveLabels" << endl;
	pcl::KdTreeFLANN<PointWaveLength> kdtree;
	kdtree.setInputCloud(labeled);
	PointCloud<PointWaveLength>::Ptr ret_cloud(new PointCloud<PointWaveLength>);
	for (int pdx = 0; pdx < leaves->size(); pdx++)
	{
		PointWaveLength px = leaves->points[pdx];
		std::vector<int> pointIdxNKNSearch(1);
		std::vector<float> pointNKNSquaredDistance(1);
		kdtree.nearestKSearchT(px, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
		labeled->points[pointIdxNKNSearch[0]].label = leaves->points[pdx].label;
	}
	copyPointCloud(*labeled, *ret_cloud);
	return ret_cloud;
}

//datasetの各フォルダを探索していって、どこで再開すべきかを返す
void FindContinuePoint(string label_dir, vector<string> files, int pot_id, int& start_leaf_idx, int& start_idx)
{
	cout << "FindContinuePoint" << endl;
	char full_path[1024];
	string relative_path = label_dir + "pot" + to_string(pot_id) + "/" + files[0];
	_fullpath(full_path, relative_path.c_str(), 1024);
	vector<string> leaves = read(full_path);
	if (leaves.size() == 0)
	{
		cout << "start from first file" << endl;
		start_leaf_idx = 1;
		start_idx = 0;
		return;
	}
	int prev_count = leaves.size();
	for (auto fdx = 1; fdx < files.size(); fdx++)
	{
		char full_path[1024];
		string relative_path = label_dir + "pot" + to_string(pot_id) + "/" + files[fdx];
		_fullpath(full_path, relative_path.c_str(), 1024);
		vector<string> leaves = read(full_path);
		if (prev_count == leaves.size())
			continue;
		else
		{
			start_leaf_idx = prev_count;
			start_idx = fdx;
			return;
		}
	}
	start_leaf_idx = prev_count + 1;
	start_idx = 0;
}

void ActivateAnnotation()
{

}

int main(int argc, char **argv)
{
	setConsole();
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

	int leaf_idx = 1;
	int file_start_idx = 0;
	bool annotation_finish = true;
	FindContinuePoint(annotation_dir, files, pot_idx, leaf_idx, file_start_idx);
	cout << "start from ..." << file_start_idx << endl;

	while (annotation_finish)
	{
		PointCloud<PointWaveLength>::Ptr labeled(new PointCloud<PointWaveLength>);
		PointCloud<PointWaveLength>::Ptr currLeaves(new PointCloud<PointWaveLength>);
		PointCloud<PointWaveLength>::Ptr prev(new PointCloud<PointWaveLength>);

		future<PointCloud<PointWaveLength>::Ptr> f1 = async(loadCurrPotCloudAsync, pros_dir, files[file_start_idx], pot_idx);
		future<PointCloud<PointWaveLength>::Ptr> f2 = async(loadLeavesAsync, annotation_dir, pot_idx, files[file_start_idx], leaf_idx);
		future<PointCloud<PointWaveLength>::Ptr> f3;
		if (file_start_idx - 1 >= 0)
			f3 = async(loadPrevPotCloudAsync, annotation_dir, pros_dir, files[file_start_idx - 1], pot_idx, leaf_idx);
		else
			f3 = async(loadPrevPotCloudAsync, annotation_dir, pros_dir, files[0], -1, leaf_idx);

		for (auto idx = file_start_idx; idx < files.size(); idx++)
		{
			if (!fexists(pros_dir + files[idx] + ".pcd"))
			{
				cout << "file " << pros_dir + files[idx] << ".pcd" << " not found!" << endl;
				continue;
			}
			cout << "read... " << files[idx] << endl;

			labeled->clear();
			copyPointCloud(*f1.get(), *labeled);
			if(idx + 1 >= files.size())
				f1 = async(loadCurrPotCloudAsync, pros_dir, files[0], pot_idx);
			else
				f1 = async(loadCurrPotCloudAsync, pros_dir, files[idx + 1], pot_idx);

			currLeaves->clear();
			copyPointCloud(*f2.get(), *currLeaves);
			if(idx + 1 >= files.size())
				f2 = async(loadLeavesAsync, annotation_dir, pot_idx, files[0], leaf_idx);
			else
				f2 = async(loadLeavesAsync, annotation_dir, pot_idx, files[idx + 1], leaf_idx);

			prev->clear();
			copyPointCloud(*f3.get(), *prev);
			f3 = async(loadPrevPotCloudAsync, annotation_dir, pros_dir, files[idx], pot_idx, leaf_idx);

			PointCloud<PointWaveLength>::Ptr prev_leaf(new PointCloud<PointWaveLength>);
			if (idx - 1 >= 0)
				loadPreviousLeaf(annotation_dir, pros_dir, files[idx - 1], pot_idx, leaf_idx, prev_leaf);

			VisualizerConfig vis(WINDOW_WIDTH, WINDOW_HEIGHT, labeled, currLeaves, "Current 3D Viewer");
			if (leaf_idx == 0)
				vis.vis_setup(pot_idx, 1);
			else
				vis.vis_setup(pot_idx, 0);

			VisualizerConfig prev_vis(WINDOW_WIDTH, WINDOW_HEIGHT, prev, prev_leaf, "Previous 3D Viewer");
			if (prev->size() != 0)
				prev_vis.vis_setup(pot_idx, 0);

			PolygonDrawer pd("Polygon Viewer", WINDOW_WIDTH, WINDOW_HEIGHT);
			boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
			boost::shared_ptr<pcl::visualization::PCLVisualizer> prev_viewer;

			viewer = vis.getVis();
			viewer->setPosition(0, 0);
			viewer->addText("Leaf Index: " + to_string(leaf_idx), 0, 0, 50, 255, 0, 0, "text");
			prev_viewer = prev_vis.getVis();
			prev_viewer->setPosition(0, WINDOW_HEIGHT);
			while (!viewer->wasStopped())
			{
				viewer->spinOnce(1);
				prev_viewer->spinOnce(1);
			}



			vis.prepareRender(viewer);


			cv::Mat cvImageRGB;
			renderImage(viewer, cvImageRGB);
			pd.loadImage(cvImageRGB);
			annotation_finish = pd.run();
			if (!annotation_finish)
				break;


			PointCloud<PointWaveLength>::Ptr cloud_unlabeled(new PointCloud<PointWaveLength>);
			copyPointCloud(*vis.getReLabeledCloud(), *cloud_unlabeled);

			vector<cv::Point> maskedData = pd.getMaskData();
			cv::Mat maskImage = pd.getMaskImage();


			PointCloud<PointWaveLength>::Ptr cloud_segmented(new PointCloud<PointWaveLength>);
			cv::Mat rgbRendered = cv::Mat::ones(maskImage.size(), CV_8UC3);
			CloudReprojection(viewer, cloud_unlabeled, maskImage, cloud_segmented, rgbRendered);

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
		
		leaf_idx++;
		file_start_idx = 0;
	}
	return 1;
}

