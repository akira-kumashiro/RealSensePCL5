#include "PointCloud2Mesh.h"



PointCloud2Mesh::PointCloud2Mesh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::string filename, gpParameters &gpp, bool smoothing) :
	_cloud(new pcl::PointCloud<pcl::PointXYZ>),
	cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>)
{
	pcl::copyPointCloud(*cloud, *_cloud);

	cloudConverter(filename, gpp, smoothing);
}


PointCloud2Mesh::PointCloud2Mesh(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const std::string filename, gpParameters &gpp, bool smoothing) :
	_cloud(new pcl::PointCloud<pcl::PointXYZ>),
	cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>)
{
	pcl::copyPointCloud(*cloud, *_cloud);

	cloudConverter(filename, gpp, smoothing);
}

PointCloud2Mesh::PointCloud2Mesh(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, const std::string filename, gpParameters &gpp, bool smoothing) :
	_cloud(new pcl::PointCloud<pcl::PointXYZ>),
	cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>)
{
	pcl::copyPointCloud(*cloud, *_cloud);

	cloudConverter(filename, gpp, smoothing);
}

PointCloud2Mesh::~PointCloud2Mesh()
{
}


pcl::PointCloud<pcl::PointNormal>::Ptr PointCloud2Mesh::findSmoothedNormals(void)
{
	// Create a KD-Tree
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

	// Output has the PointNormal type in order to store the normals calculated by MLS
	pcl::PointCloud<pcl::PointNormal>::Ptr mls_points(new pcl::PointCloud<pcl::PointNormal>);

	// Init object (second point type is for the normals, even if unused)
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

	mls.setComputeNormals(true);

	// Set parameters
	mls.setInputCloud(_cloud);
	mls.setPolynomialFit(true);
	mls.setSearchMethod(tree);
	mls.setSearchRadius(0.01);//0.03

	// Reconstruct
	mls.process(*mls_points);

	return(mls_points);
}

pcl::PointCloud<pcl::PointNormal>::Ptr PointCloud2Mesh::findNormals(void)
{
	// Normal estimation*
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(_cloud);
	n.setInputCloud(_cloud);
	n.setSearchMethod(tree);
	n.setKSearch(20);
	n.compute(*normals);
	//* normals should not contain the point normals + surface curvatures

	// Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*_cloud, *normals, *cloud_with_normals);
	//cloud_with_normals = *_cloud + normals;

	return(cloud_with_normals);
}

pcl::PolygonMesh PointCloud2Mesh::createMesh(gpParameters &param)
{
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>);
	tree->setInputCloud(cloud_with_normals);

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::PolygonMesh triangles;

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius(param.searchRadius);

	// Set typical values for the parameters
	gp3.setMu(param.mu);
	gp3.setMaximumNearestNeighbors(param.maximumNearestNeighbors);
	gp3.setMaximumSurfaceAngle(param.maximumSurfaceAngle); // 45 degrees
	gp3.setMinimumAngle(param.minimumAngle); // 10 degrees
	gp3.setMaximumAngle(param.maximumAngle); // 120 degrees
	gp3.setNormalConsistency(param.normalConsistency);

	// Get result
	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree);
	gp3.reconstruct(triangles);

	return(triangles);
}

int PointCloud2Mesh::cloudConverter(const std::string filename, gpParameters &param, bool smoothing)
{
	if (smoothing)
		cloud_with_normals = findSmoothedNormals();
	else
		cloud_with_normals = findNormals();
	pcl::PolygonMesh triangles;
	triangles = createMesh(param);

	return(pcl::io::saveOBJFile(filename, triangles));
}
