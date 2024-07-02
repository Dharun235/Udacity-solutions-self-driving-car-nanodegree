#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <math.h>
#include <vector>
#include <Eigen/Geometry>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

using namespace std;

const double pi = M_PI;

struct Point{
	float x, y, z;

	Point()
		: x(0), y(0), z(0){}

	Point(float setX, float setY, float setZ)
		: x(setX), y(setY), z(setZ){}

	void Print(){
		cout << "x: " << x << " y: " << y << " z: " << z << endl;
	}
};

struct Rotate{
	float yaw, pitch, roll;

	Rotate()
		: yaw(0), pitch(0), roll(0){}

	Rotate(float setYaw, float setPitch, float setRoll)
		: yaw(setYaw), pitch(setPitch), roll(setRoll){}

	void Print(){
		cout << "yaw: " << yaw << " pitch: " << pitch << " roll: " << roll << endl;
	}
};

struct Pose{

	Point position;
	Rotate rotation;

	Pose()
		: position(Point(0, 0, 0)), rotation(Rotate(0, 0, 0)){}

	Pose(Point setPos, Rotate setRotation)
		: position(setPos), rotation(setRotation) {}

	Pose operator-(const Pose& p)
    {
        Pose result(Point(position.x-p.position.x, position.y-p.position.y, position.z-p.position.z), Rotate(rotation.yaw-p.rotation.yaw, rotation.pitch-p.rotation.pitch, rotation.roll-p.rotation.roll) );
        return result;
    }

	Eigen::Matrix4f getTransformationMatrix() const {
        Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();
        Eigen::Translation3f translation(position.x, position.y, position.z);
        Eigen::AngleAxisf rotation_yaw(rotation.yaw, Eigen::Vector3f::UnitZ());
        Eigen::AngleAxisf rotation_pitch(rotation.pitch, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf rotation_roll(rotation.roll, Eigen::Vector3f::UnitX());

        transformation_matrix = (translation * rotation_yaw * rotation_pitch * rotation_roll).matrix();
        return transformation_matrix;
    }
};

struct ControlState{

	float t;
	float s;
	float b;

	ControlState(float setT, float setS, float setB)
		: t(setT), s(setS), b(setB) {}
};

struct Vect2{

	float mag;
	float theta;

	Vect2(float setMag, float setTheta)
		: mag(setMag), theta(setTheta) {}
};

struct Color
{
    float r, g, b;

    Color(float setR, float setG, float setB)
            : r(setR), g(setG), b(setB)
    {}
};

struct BoxQ
{
    Eigen::Vector3f bboxTransform;
    Eigen::Quaternionf bboxQuaternion;
    float cube_length;
    float cube_width;
    float cube_height;
};

Eigen::Matrix4f transform2D(float theta, float xt, float yt);
Eigen::Matrix4f transform3D(float yaw, float pitch, float roll, float xt, float yt, float zt);
Pose getPose(Eigen::Matrix4f matrix);
float getDistance(Point p1, Point p2);
float minDistance(Point p1, std::vector<Point> points);
void print4x4Matrix (const Eigen::Matrix4f & matrix);
void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr& viewer, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::string name, Color color, int renderSize = 4);
void renderRay(pcl::visualization::PCLVisualizer::Ptr& viewer, Point p1, Point p2, std::string name, Color color);
void renderPath(pcl::visualization::PCLVisualizer::Ptr& viewer, const PointCloudT::Ptr& cloud, std::string name, Color color);
Eigen::Quaternionf getQuaternion(float theta);
void renderBox(pcl::visualization::PCLVisualizer::Ptr& viewer, BoxQ box, int id, Color color, float opacity);

struct LineSegment{
	// slope of y component
	float my;
	// slope of x component
	float mx;
	//  (y{mx=1}/x{mx=0}) intercept
	float b;
	// (x{mx=1}/y{mx=0}) interval
	float min;
	float max;

	LineSegment(float setMy, float setMx, float setB, float setMin, float setMax)
                : my(setMy), mx(setMx), b(setB), min(setMin), max(setMax){
                if (setMy == 0 and setMx == 0){
                	my = 0;
                	mx = 1;
                }
    }
    LineSegment()
    			: my(1), mx(1), b(1), min(-1), max(0){}
    //			p1
    //			o
    //		p2		  p3
   	// 		o---------o

    bool Contains(float p1, float p2, float p3){
    	return  (p2 <= p1 ) && (p1 <= p3) ;
    }

    bool Intersect(LineSegment line, Point& intersection){
    	if ( (my/mx) == (line.my/line.mx) ){
    		// lines dont intersect or are on top of each other
    		return false;
    	}   
    	if(mx == 0){
    		intersection.x = b;
    		intersection.y = (line.my/line.mx) * intersection.x + line.b;
    		return Contains(intersection.y, min, max) && Contains(intersection.x, line.min, line.max);
    	}
    	else if(line.mx == 0){
    		intersection.x = line.b;
    		intersection.y = (my/mx) * intersection.x + b;
    		return Contains(intersection.y, line.min, line.max) && Contains(intersection.x, min, max);
    	}
    	// not dealing with any vertical or horizontal lines and lines are not the same slope
    	else{

    		intersection.x = (b - line.b)/( (line.my/line.mx) - (my/mx) ) ;
    		intersection.y = (my/mx) * intersection.x + b;
    		return Contains(intersection.x, min, max) && Contains(intersection.x, line.min, line.max);
    	}
    }

    void Print(){
    	cout << "my: " << my << " mx: " << mx << " b: " << b << " min: " << min << " max: " << max << endl;
    }

};

struct Lidar{

	float x;
	float y;
	float theta;
	float range;
	int res;

	Lidar(float setX, float setY, float setTheta, float setRange, int setRes)
		: x(setX), y(setY), theta(setTheta), range(setRange), res(setRes){}

	pcl::PointCloud<pcl::PointXYZ>::Ptr scan(vector<LineSegment> walls){

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new PointCloudT); 
		float deltaTheta = (2*pi)/float(res);
		float residue = .1*deltaTheta;
		for(float angle = theta; angle < theta + 2*pi - residue ; angle += deltaTheta ){

			LineSegment ray;

			//cout << "angle " << angle << endl;

			// check if angle is vertical
			if( ceil(tan(angle-pi/2)*1000)/1000 == 0){

				//cout << "vertical" << endl;

				float yb = sin(angle) * range;
				float minb = min(y,yb);
				float maxb = max(y,yb);

				ray = LineSegment(1, 0, x, minb, maxb);

			}
			else{

				float m = ceil(tan(angle)*1000)/1000;
				float b = y - x*m;

				float xb = cos(angle) * range;
				float minb = min(x,xb);
				float maxb = max(x,xb);

				ray = LineSegment(m, 1, b, minb, maxb);
			}

			float closetDist = range;
			Point cPoint;
			for(LineSegment wall : walls){
				Point point;
				if( ray.Intersect(wall, point) ){
					//cout << "collision" << endl;
					//ray.Print();
					//wall.Print();
					float distance = sqrt( (x - point.x)*(x - point.x) + (y - point.y)*(y - point.y) );
					//cout << "dis " << distance << endl;
					//point.Print();
					if( distance < closetDist){
						closetDist = distance; 
						cPoint = point;
					}
					
				}
			}
			if( closetDist < range ){
				// transform to lidars local coordinates
				//cout << "angle " << angle << endl;
				//cout << closetDist << endl;
				float pointX = closetDist * cos(angle-theta);
				float pointY = closetDist * sin(angle-theta);
				cloud->points.push_back(pcl::PointXYZ(pointX, pointY, 0));
			}
		}
		return cloud;
	}

	void Move(float step, float rotate){
		theta += rotate;
		x += step * cos(theta);
		y += step * sin(theta);
	}
};

