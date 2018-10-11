#pragma once

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/mls.h>
#include <pcl/io/obj_io.h>

class PointCloud2Mesh
{
public:
	class gpParameters
	{
	public:
		double searchRadius;//The maximum distance between connected points(maximum edge length) (default:0.025)
		double mu;//maximum distance between neighbors(default:2.5)
		int maximumNearestNeighbors;//(default:100)
		double maximumSurfaceAngle;// 45 degrees(default:M_PI/4)
		double minimumAngle; // 10 degrees(default:M_PI/18)
		double maximumAngle; // 120 degrees(default:2*M_PI/3)
		bool normalConsistency;//(default:false)

		gpParameters(void)
		{
			searchRadius = 0.025;
			mu = 2.5;
			maximumNearestNeighbors = 100;
			maximumSurfaceAngle = M_PI / 4;
			minimumAngle = M_PI / 18;
			maximumAngle = 2 * M_PI / 3;
			normalConsistency = false;
		}
	};

	PointCloud2Mesh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::string filename, gpParameters &gpp, bool smoothing);
	PointCloud2Mesh(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const std::string filename, gpParameters &gpp, bool smoothing);
	PointCloud2Mesh(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, const std::string filename, gpParameters &gpp, bool smoothing);
	~PointCloud2Mesh();

private:
	pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud;
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals;
	//pcl::PCLPointCloud2 cloud_blob;
	//std::string filename;

	pcl::PointCloud<pcl::PointNormal>::Ptr findSmoothedNormals(void);
	pcl::PointCloud<pcl::PointNormal>::Ptr findNormals(void);
	pcl::PolygonMesh createMesh(gpParameters &param);

	int cloudConverter(const std::string filename, gpParameters &param, bool smoothing);
};
