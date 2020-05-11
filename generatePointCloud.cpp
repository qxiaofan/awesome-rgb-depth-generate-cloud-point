//欢迎关注公众号：3D视觉工坊
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;
// 定义点云类型
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// 相机内参
const double camera_factor = 1000;
const double camera_cx = 649.402466;
const double camera_cy = 649.402466;
const double camera_fx = 640.685730;
const double camera_fy = 359.206085;
// 主函数

int main(int argc, char** argv)
{
    // 读取./data/rgb.png和./data/depth.png，并转化为点云

    // 图像矩阵
    cv::Mat rgb, depth;
    // 使用cv::imread()来读取图像
    // API: http://docs.opencv.org/modules/highgui/doc/reading_and_writing_images_and_video.html?highlight=imread#cv2.imread
    rgb = cv::imread("../depthImg/Color_00046044.png");
    cout << "read rgb"<<endl;
    // rgb 图像是8UC3的彩色图像
    // depth 是16UC1的单通道图像，注意flags设置-1,表示读取原始数据不做任何修改
    depth = cv::imread("../depthImg/Depth_00046044.png");
    cout << "read depth" << endl;
    // 点云变量
    // 使用智能指针，创建一个空点云。这种指针用完会自动释放。
    PointCloud::Ptr cloud(new PointCloud);
    // 遍历深度图
    for (int m = 0; m < depth.rows; m++)
        for (int n = 0; n < depth.cols; n++)
        {
            // 获取深度图中(m,n)处的值
            ushort d = depth.ptr<ushort>(m)[n];
            // d 可能没有值，若如此，跳过此点
            if (d == 0)
                continue;
            // d 存在值，则向点云增加一个点
            PointT p;

            // 计算这个点的空间坐标
            p.z = double(d) / camera_factor;
            p.x = (n - camera_cx) * p.z / camera_fx;
            p.y = (m - camera_cy) * p.z / camera_fy;

            // 从rgb图像中获取它的颜色
            // rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
            p.b = rgb.ptr<uchar>(m)[n * 3];
            p.g = rgb.ptr<uchar>(m)[n * 3 + 1];
            p.r = rgb.ptr<uchar>(m)[n * 3 + 2];

            // 把p加入到点云中
            cloud->points.push_back(p);
            //cout << cloud->points.size() << endl;
        }
    // 设置并保存点云
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cout << "point cloud size = " << cloud->points.size() << endl;
    cloud->is_dense = false;
    try{
        //保存点云图
        pcl::io::savePCDFile("../cloud_pnt.pcd", *cloud);


    }
    catch (pcl::IOException &e){
        cout << e.what()<< endl;
    }
    //显示点云图
    pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");//直接创造一个显示窗口
    viewer.showCloud(cloud);//再这个窗口显示点云
    while (!viewer.wasStopped())
    {
    }

    pcl::io::savePCDFileASCII("../point_cloud.pcd", *cloud);
    // 清除数据并退出
    cloud->points.clear();
    cout << "Point cloud saved." << endl;
    return 0;
}
