/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include <random>
#include <cstdlib>
#include <cmath>

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}
struct Line{
	double a;
	double b;
	double c;
	Line(double x1, double x2, double y1, double y2) : a(y1 - y2), b(x2 - x1), c(x1*y2 - x2*y1){}
};

struct Plane {
	std::vector<double> v1;
	std::vector<double> v2;
	std::vector<double> orthogonal_v;
	double x1, x2, x3, y1, y2, y3, z1, z2, z3;
	double A, B, C, D;

	Plane(double x1, double y1, double z1, double x2, double y2, double z2, double x3, double y3, double z3) :
	x1(x1), x2(x2), x3(x3),
	y1(y1), y2(y2), y3(y3),
	z1(z1), z2(z2), z3(z3)
	 {
		double v1x = x2 - x1;
		double v1y = y2 - y1;
		double v1z = z2 - z1;
		double v2x = x3 - x1;
		double v2y = y3 - y1;
		double v2z = z3 - z1;

		A = v1y * v2z - v1z * v2y;
		B = v1z * v2x - v1x * v2z;
		C = v1x * v2y - v1y * v2x;
		D = -(A * x1 + B * y1 + C * z1);

	}
	double distance(pcl::PointXYZ point) {
		return fabs(A * point.x + B * point.y + C * point.z + D) / sqrt(std::pow(A,2) + std::pow(B,2) + std::pow(C,2));
	}
};

std::pair<pcl::PointXYZ, pcl::PointXYZ> getRandomPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
	int min = 0;
	int max = cloud->points.size();
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_int_distribution<int> dis(min, max);
	std::pair<pcl::PointXYZ, pcl::PointXYZ> line(cloud->points[dis(gen)],cloud->points[dis(gen)]);
	return line;
}

std::tuple<pcl::PointXYZ, pcl::PointXYZ, pcl::PointXYZ> getRandomPoints3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
	int min = 0;
	int max = cloud->points.size();
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_int_distribution<int> dis(min, max);
	std::tuple<pcl::PointXYZ, pcl::PointXYZ, pcl::PointXYZ> points(cloud->points[dis(gen)],cloud->points[dis(gen)], cloud->points[dis(gen)]);
	return points;
}

double distance(Line line, pcl::PointXYZ point){
	return fabs(line.a * point.x + line.b * point.y + line.c) / sqrt(line.a * line.a + line.b * line.b);
}


std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 
	int iteration = 0;
	while(iteration < maxIterations){
		auto points = getRandomPoints(cloud);
		std::cout <<"random points: "<<points.first.x << " " << points.second.x << " " << points.first.y << " " << points.second.y << std::endl;
		Line line{points.first.x, points.second.x, points.first.y, points.second.y};
		std::cout << "line: " << line.a << " " << line.b << " " << line.c << std::endl; 
		std::unordered_set<int> inliers;
		for(size_t i = 0; i < cloud->points.size(); i++){
			std::cout << i << std::endl;
			auto dist = distance(line, cloud->points[i]);
			std::cout << "distance: " << dist << std::endl;
			if(distance(line, cloud->points[i]) <= distanceTol){
				std::cout << "inserting" << std::endl;
				inliers.insert(i);		
			}
		}
		if(inliers.size() > inliersResult.size())
			inliersResult = inliers;
		iteration++;
	}

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
	std::cout << "set size " << inliersResult.size() << std::endl;
	
	return inliersResult;
}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 
	int iteration = 0;
	while(iteration < maxIterations){
		auto points = getRandomPoints3D(cloud);
		
		std::cout <<"random points: "<<std::get<0>(points).x << " " << std::get<0>(points).y << " " <<std::get<0>(points).z << "\n" \
		<< std::get<1>(points).x << " " << std::get<1>(points).y << " " <<std::get<1>(points).z << "\n" \
		<< std::get<2>(points).x << " " << std::get<2>(points).y << " " <<std::get<2>(points).z << "\n";
		
		Plane plane{
			std::get<0>(points).x, std::get<0>(points).y, std::get<0>(points).z,
			std::get<1>(points).x, std::get<1>(points).y, std::get<1>(points).z,
			std::get<2>(points).x, std::get<2>(points).y, std::get<2>(points).z
			};

		std::cout << "plane: " << plane.A << " " << plane.B << " " << plane.C << " " << plane.D <<std::endl; 
		std::unordered_set<int> inliers;
		for(size_t i = 0; i < cloud->points.size(); i++){
			std::cout << i << std::endl;
			auto dist = plane.distance(cloud->points[i]);
			std::cout << "distance: " << dist << std::endl;
			if(plane.distance(cloud->points[i]) <= distanceTol){
				std::cout << "inserting" << std::endl;
				inliers.insert(i);
			}
		}
		if(inliers.size() > inliersResult.size())
			inliersResult = inliers;
		iteration++;
	}

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
	std::cout << "set size " << inliersResult.size() << std::endl;
	
	return inliersResult;
}


int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	//std::unordered_set<int> inliers = Ransac(cloud, 10, 1.0);
	std::unordered_set<int> inliers = RansacPlane(cloud, 50, 0.2);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
