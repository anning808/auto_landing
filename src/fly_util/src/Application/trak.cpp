#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <stdlib.h>
#include <stdbool.h>
#include <assert.h>
#include <Queue.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>
#include <tf/tf.h>
using namespace std;
using namespace cv;

class findsquares_4
{
public:
  int num_edg;
  Point center;
  Point center_result;
  int num_sam_center;
  vector<Point> approx;
};

double relative_yaw, relative_x, relative_y; // 降落板与无人机的相对偏航角 单位:rad
int test_relative = 2;                       // 是否输出旋转测试数据
double true_x, true_y, true_z;
Queueclass Point_queue;
findsquares_4 sqtemp;
vector<findsquares_4> sq4, sqall;
Point center, result_center;
Mat dimg;
double dimg_depth; // 输出深度图对于center的深度
int thresh = 50, N = 5;
int r_num;
ros::Publisher visual_pub;
geometry_msgs::Pose visual_pose;
const char *wndname = "Square Detection Demo";
static double angle(Point pt1, Point pt2, Point pt0);
static void drawSquares(Mat &image, const vector<vector<Point>> &squares);
static void drawsqall(const Mat &image, vector<findsquares_4> &squares);
static void findSquares(const Mat &image, vector<vector<Point>> &squares);
static void findSquares_first(const Mat &image, vector<vector<Point>> &squares);
static void findTarget(const Mat &image, vector<vector<Point>> &squares);
static void out_center_dele(Point center, vector<findsquares_4> &nums);

void getEulerYPR(const geometry_msgs::Quaternion &q, double &yaw, double &pitch, double &roll)
{
  double sqw;
  double sqx;
  double sqy;
  double sqz;

  sqx = q.x * q.x;
  sqy = q.y * q.y;
  sqz = q.z * q.z;
  sqw = q.w * q.w;

  // Cases derived from https://orbitalstation.wordpress.com/tag/quaternion/
  double sarg = -2 * (q.x * q.z - q.w * q.y) / (sqx + sqy + sqz + sqw); /* normalization added from urdfom_headers */
  if (sarg <= -0.99999)
  {
    pitch = -0.5 * M_PI;
    roll = 0;
    yaw = -2 * atan2(q.y, q.x);
  }
  else if (sarg >= 0.99999)
  {
    pitch = 0.5 * M_PI;
    roll = 0;
    yaw = 2 * atan2(q.y, q.x);
  }
  else
  {
    pitch = asin(sarg);
    roll = atan2(2 * (q.y * q.z + q.w * q.x), sqw - sqx - sqy + sqz);
    yaw = atan2(2 * (q.x * q.y + q.w * q.z), sqw + sqx - sqy - sqz);
  }
};

void GPS_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  // GPS_ori_position = msg->pose.position;

  getEulerYPR(msg->pose.orientation, relative_yaw, relative_x, relative_y); // q,y,p,r
  relative_yaw = relative_yaw * 180 / 3.14;
  relative_x = relative_x * 180 / 3.14; // 这里x是无人机的Pitch，在更换控制律时坐标系会随之改变，请注意  向前是正
  relative_y = relative_y * 180 / 3.14; // roll  向右是正角度

  if (test_relative == 1)
  {
    cout << "relative_yaw:" << relative_yaw << " relative_x:" << relative_x << "relative_y:" << relative_y << endl;
    cout << "relative_yaw:" << relative_yaw - 90 << endl;
  }
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  // cout << "\033[1;32m go! \033[0m" << endl;
  try
  {
    vector<vector<Point>> squares;
    Mat imageinput = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
    // Mat image_test = imread("/home/b/下载/tes.png", 1);
    findTarget(imageinput, squares);
    // 用于将ROS图像消息转化为Opencv支持的图像格式（采用BGR8编码方式）
    cv::imshow("view_callback", imageinput);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void image2Callback(const sensor_msgs::ImageConstPtr &msg)
{
  // cout << "\033[1;32m go! \033[0m" << endl;
  try
  {
    vector<vector<Point>> squares;
    Mat imageinput = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
    // Mat image_test = imread("/home/b/下载/tes.png", 1);
    findTarget(imageinput, squares);
    // 用于将ROS图像消息转化为Opencv支持的图像格式（采用BGR8编码方式）
    cv::imshow("view_callback", imageinput);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void depthimageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  // cout << "\033[1;31m hw_opencv3! \033[0m" << endl;
  try
  {

    ///////dimg = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1)->image.clone();

    // Mat dst = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1)->image.clone();
    // {
    //     double min;
    //     double max;
    //     int maxIDX;
    //     int minIDX;
    //     cv::minMaxIdx(dst, &min, &max, &minIDX, &maxIDX);
    //     cv::Mat adjMap;
    //     float scale = 255 / (max - min);
    //     dst.convertTo(adjMap, CV_8UC1, scale, -min * scale);
    //     cv::Mat falseColorsMap;
    //     cv::applyColorMap(adjMap, falseColorsMap, cv::COLORMAP_JET);
    //     // cv::imshow("dimg", falseColorsMap);
    // }
    //////dimg_depth = dimg.at<uint16_t>(result_center.x, result_center.y);
    // cout << "center.x:::" << result_center.x << "     center.y:::" << result_center.y << "dimg_depth::" << dimg_depth << endl;

    /////true_z = cos(relative_y) * cos(relative_x) * dimg_depth;
    // dimg_depth = dimg.at<uint16_t>(640, 360);
    // cout << "dimg_depth:::" << dimg_depth << endl;
    // dimg_depth = dimg.at<uint16_t>(dimg.cols / 2, dimg.rows / 2);

    // Mat image_test = imread("/home/b/下载/tes.png", 1);
    // 用于将ROS图像消息转化为Opencv支持的图像格式（采用BGR8编码方式）
    // cv::imshow("dimg", dst);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle node;
  // Queueclass *Point_queue = new (Queueclass);
  Point_queue.QueueInti(&Point_queue.pq);
  cout << "\033[1;31m hw_opencv! \033[0m" << endl;

  cv::startWindowThread();
  // ros::Subscriber sub_img0 = n.subscribe(IMAGE0_TOPIC, 100, img0_callback);
  // ros::Subscriber sub_img1;
  // if (STEREO)
  // {
  //     sub_img1 = n.subscribe(IMAGE1_TOPIC, 100, img1_callback);
  // }

  image_transport::ImageTransport transport(node);                                                    /// iris/usb_cam/image_raw    /camera/color/image_raw  /iris_fpv_cam/usb_cam/image_raw
                                                                                                      //  image_transport::Subscriber sub = transport.subscribe("/camera/color/image_raw", 1, imageCallback); //  /camera/color/image_raw ros::spin();
  image_transport::Subscriber sub = transport.subscribe("/iris/usb_cam/image_raw", 1, imageCallback); //  /camera/color/image_raw ros::spin();  /camera/depth/image_rect_raw
  image_transport::Subscriber sub2 = transport.subscribe("/camera/color/image_raw", 1, image2Callback);
  image_transport::ImageTransport transports(node);
  image_transport::Subscriber subs = transports.subscribe("/camera/aligned_depth_to_color/image_raw", 1, depthimageCallback); //  /camera/color/image_raw ros::spin();  /camera/depth/image_rect_raw
  visual_pub = node.advertise<geometry_msgs::Pose>("/vision/vision_relative_position", 30);
  // 【订阅】GPS位置
  //  来自视觉节点 方向定义：[机体系下：前方x为正，右方y为正，下方z为正]
  ros::Subscriber GPS_pos_sub = node.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, GPS_cb);

  while (ros::ok)
  {
    ros::spinOnce();

    // namedWindow(wndname, 1);
  }

  // cv::destroyWindow("view");
}

// 输入图像，输出阈值，很简单吧

int MIN_CENTER_DIS = 128;

// 辅助功能:
// 求出向量夹角的余弦
// 从pt0->pt1到pt0->pt2
static double angle(Point pt1, Point pt2, Point pt0)
{
  double dx1 = pt1.x - pt0.x;
  double dy1 = pt1.y - pt0.y;
  double dx2 = pt2.x - pt0.x;
  double dy2 = pt2.y - pt0.y;
  return (dx1 * dx2 + dy1 * dy2) / sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10);
}

void comput_center(Point *p, int n, Point &center)
{
  center.x = 0;
  center.y = 0;
  for (size_t i = 0; i < n; i++)
  {
    center.x += (p + i)->x;
    center.y += (p + i)->y;
  }
  center.x = (int)center.x / 4;
  center.y = (int)center.y / 4;
}

// 返回在图像上检测到的正方形序列。
// 序列存储在指定的内存中
int comput_rnum(vector<findsquares_4> sqall)
{
  int i = 0;
  int max = 1;
  int count = 1, tolerant = 5;
  for (i = 0; i < sqall.size(); i++)
  {
    for (size_t j = i; j < sqall.size(); j++)
    {
      // cout << "num:" << sqall[i].num_edg << "  " << sqall[j].num_edg << endl;
      if (sqall[i].num_edg < sqall[j].num_edg - tolerant || sqall[i].num_edg > sqall[j].num_edg + tolerant)
      {
        count++;
      }
    }
    // cout << "count:" << count << "   max:" << max << endl;

    if (count > max)
    {
      max = count;
    }
    count = 1;
  }
  return max;
}
int Major(int A[], int n) // Major函数，用于找寻数组A[]中的主元素
{
  const int tole = 10;
  int maybe = A[0]; // 初始化可能主元素为A[0]
  int count = 1;    // 计数器count，初始为1
  int i;            // 工作指针，初始为1
  for (i = 1; i < n; i++)
  {
    if (A[i] > maybe - tole && A[i] < maybe + tole) // 相同元素，计数器加1
    {
      count++;
    }
    else // 不是相同元素
    {
      if (!(--count)) // 计数器减为0时更新maybe为下一个元素
      {
        if (i < n - 1) // 减为0时不是最后一个元素，取下一个元素为新的maybe
        {
          maybe = A[i + 1];
        }
        else
        {
          break;
        }
      }
    }
  }
  for (count = 0, i = 0; i < n; i++) // 计数器清零，检查是否是真正的主元素
  {
    if (A[i] > maybe - 10 && A[i] < maybe + 10)
    {
      count++;
    }
  }
  if (count > (n / 2))
  {
    return maybe;
  }
  else
  {
    return -1;
  }
}

Point Major_queue(Queueclass p_que) // Major函数，用于找寻数组Queueclass中的主元素
{

  int n = p_que.QueueSize(&p_que.pq);
  int A[n];
  Point result;
  // 数组填入坐标信息
  for (size_t i = 0; i < n; i++)
  {
    A[i] = p_que.pq.tail->val.x;
  }
  result.x = Major(A, n);

  for (size_t i = 0; i < n; i++)
  {
    A[i] = p_que.pq.tail->val.y;
  }
  result.y = Major(A, n);
  return result;
}

Point Major(vector<findsquares_4> nums) // Major函数，用于找寻数组A[]中的主元素
{
  int n = nums.size();
  int A[n];
  Point result;
  // 数组填入坐标信息
  for (size_t i = 0; i < n; i++)
  {
    A[i] = nums.at(i).center.x;
  }
  result.x = Major(A, n);

  for (size_t i = 0; i < n; i++)
  {
    A[i] = nums.at(i).center.y;
  }
  result.y = Major(A, n);
  return result;
}

static void out_center_dele(Point center, vector<findsquares_4> &sqall) // Major函数，用于找寻数组A[]中的主元素
{
  const int tole = 10;
  for (size_t i = 0; i < sqall.size(); i++)
  {
    if (sqall.at(i).center.x >= center.x - tole && sqall.at(i).center.x <= center.x + tole && sqall.at(i).center.y >= center.y - tole && sqall.at(i).center.y <= center.y + tole)
    {
      // sq4.emplace_back(sqall.at(i));
    }
    else
    {
      sqall.erase(sqall.end() - sqall.size() + i);
      i -= 1;
    }
  }
}

static void findTarget(const Mat &image, vector<vector<Point>> &squares)
{

  if (image.empty())
  {
    cout << "Couldn't load " << endl;
  }
  sqall.clear();
  sq4.clear();
  findSquares_first(image, squares);
  findSquares(image, squares);
  // cout << "comput_rnum1:" << comput_rnum(sqall) << endl;
  // imshow(" view ", image);
  Point_queue.QueuePush(&Point_queue.pq, center);
  if (Point_queue.QueueSize(&Point_queue.pq) >= 20)
  {
    result_center = Major_queue(Point_queue);
    Point_queue.QueuePop(&Point_queue.pq);
  }
  else
  {
    result_center = center;
  }
  // TODO：：后续改成滤波更合适
  out_center_dele(result_center, sqall);

  drawsqall(image, sqall);

  // cout << "size of sqall:" << sqall.size() << endl;
  // cout << "center_x:" << result_center.x << "    center_y:" << result_center.y << endl;
  // cout << "comput_rnum:" << comput_rnum(sqall) << endl;

  // imshow(" view ", image);
  //  imwrite("out.jpg", image);

  // 输出位置 机体坐标
  if (comput_rnum(sqall) >= 2)
  {

    int sim = 3;
    if (sim == 1)
    {
      // cout << "dimg_depth:::" << dimg_depth << "true_z:::" << true_z << endl;

      cout << "comput x y:" << 0.1 / double(sqall.back().num_edg) * (image.cols / 2 - center.x)
           << "y:" << 0.1 / double(sqall.back().num_edg) * (center.y - image.rows / 2) << " z:"
           << 0.035 * (image.cols) / double(sqall.back().num_edg) << endl;
      cout << "comput x y:" << image.cols
           << "y:" << image.rows << endl;
      cout << "center x y:" << center.x
           << "y:" << center.y << endl;
      visual_pose.position.y = 0.1 / double(sqall.back().num_edg) * (image.cols / 2 - center.x);
      visual_pose.position.y = -visual_pose.position.x * 5; // 降半增加精度
      visual_pose.position.x = 0.1 / double(sqall.back().num_edg) * (center.y - image.rows / 2);
      visual_pose.position.x = -visual_pose.position.y * 5; // 降半增加精度
      visual_pose.position.z = 0.035 * (image.cols) / double(sqall.back().num_edg);
      visual_pose.orientation.w = 1;
    }
    else if (sim == 2)
    {
      cout << "relative_x:" << relative_x << "relative_xy:" << relative_y << endl;

      visual_pose.position.x = ((double)(image.rows / 2 - center.y) / image.rows - relative_y / 65);
      visual_pose.position.y = ((double)(image.cols / 2 - center.x) / image.cols + relative_x / 90);
      visual_pose.position.z = 0.035 * (image.cols) / double(sqall.back().num_edg);
      visual_pose.orientation.w = 1;
    }
    else
    {
      // cout << "relative_x:" << relative_x << "relative_xy:" << relative_y << endl;
      double temp[4];
      temp[0] = 2.45 * (double)(image.rows / 2 - center.y) / image.rows;
      temp[1] = 2.45 * (double)(image.cols / 2 - center.x) / image.cols;
      temp[2] = 3 * ((double)(image.rows / 2 - center.y) / image.rows + relative_y / 130);
      temp[3] = 3 * ((double)(image.cols / 2 - center.x) / image.cols - relative_x / 180);
      visual_pose.position.x = temp[2];
      visual_pose.position.y = temp[3];
      visual_pose.position.z = 0.035 * (image.cols) / double(sqall.back().num_edg);
      visual_pose.orientation.w = 1;
      // cout << "temp[0]:" << (int)(temp[0] / 0.0001) << "  temp[1]:" << (int)(temp[1] / 0.0001) << "                 temp[2]:" << (int)(temp[2] / 0.0001) << "  temp[3]:" << (int)(temp[3] / 0.0001) << endl;
    }

    if (comput_rnum(sqall) >= 5)
    {
      visual_pose.orientation.w = 2;
    }
    circle(image, center, 1, Scalar(0, 255, 0), -1); // 画半径为1的圆(画点）
    visual_pub.publish(visual_pose);
  }
  else
  {
    visual_pose.orientation.w = 0;
  }
  visual_pub.publish(visual_pose);
}
static void findSquares(const Mat &image, vector<vector<Point>> &squares)
{

  // 按比例放大图像，滤除噪声
  // 中值滤波将增强边缘检测
  Mat timg(image);
  medianBlur(image, timg, 1);                                        // 测试效果
  Mat kernel = (Mat_<float>(3, 3) << 0, -1, 0, -1, 5, -1, 0, -1, 0); // 定义锐化卷积核

  filter2D(timg, timg, -1, kernel);
  medianBlur(timg, timg, 3);
  // imshow("ruihua", timg);
  // waitKey(1);

  Mat gray0(timg.size(), CV_8U), gray;
  vector<vector<Point>> contours;
  // find squares in every color plane of the image
  // 在图像的每个颜色平面上找到正方形
  for (int c = 0; c < 3; c++) // 单通道足够
  {
    int ch[] = {c, 0};
    mixChannels(&timg, 1, &gray0, 1, ch, 1); // 将输入数组的指定通道复制到输出数组的指定通道

    // try several threshold levels
    // 尝试几个阈值级别
    // for (int l = 0; l < N; l++)
    // {
    //     // hack: use Canny instead of zero threshold level.
    //     // Canny helps to catch squares with gradient shading
    //     // Canny帮助捕捉带有渐变阴影的正方形
    //     if (l == 0)
    //     {
    // apply Canny. Take the upper threshold from slider
    // and set the lower to 0 (which forces edges merging)
    cv::threshold(gray0, gray0, 60, 255, THRESH_OTSU);
    // imshow("threshold thresh=60 by Otsu adaptive 60 ", gray0);
    // waitKey(2);
    // imshow("test1", gray0);
    // waitKey(1);

    cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(4, 4));
    Mat ele = (Mat_<int>(5, 5)
                   << 1,
               1, 1, 1, 1,
               1, 1, 1, 1, 1,
               1, 1, 1, 1, 1,
               1, 1, 1, 1, 1,
               1, 1, 1, 1, 1);

    cv::morphologyEx(gray0, gray0, 1, ele);
    // imshow("test2", gray0);
    // waitKey(1);
    // dilate(gray0, gray0, element, Point(-1, -1));

    Canny(gray0, gray, 5, thresh, 5);

    // dilate canny output to remove potential
    // holes between edge segments
    // imshow("test3", gray);
    // waitKey(1);
    dilate(gray, gray, ele, Point(-1, -1));
    // imshow("test4", gray);
    // waitKey(1);
    // }
    // else
    // {
    //     // apply threshold if l!=0:
    //     // tgray(x,y) = gray(x,y) < (l+1)*255/N ? 255 : 0
    //     gray = gray0 >= (l + 1) * 255 / N;
    // }
    /*
    --tab -e 'bash -c "sleep 5; source devel/setup.sh;rosrun fly_util trak; exec bash"' \*/
    // find contours and store them all as a list
    findContours(gray, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);
    vector<Point> approx;
    CvPoint corner[4];
    std::vector<cv::Point> corner_v;
    std::vector<cv::Point> tempapp;
    Point temp;
    // test each contour
    for (size_t i = 0; i < contours.size(); i++)
    {
      // 近似轮廓与精度成正比
      // 到轮廓周长
      approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true) * 0.02, true);

      if (approx.size() >= 4 && fabs(contourArea(Mat(approx))) > 1000 && isContourConvex(Mat(approx))) // 凸性检测 检测一个曲线是不是凸的&& fabs(contourArea(Mat(approx))) > 1000 && isContourConvex(Mat(approx))
      {
        double maxCosine = 0;
        for (int j = 2; j < 5; j++)
        {
          // find the maximum cosine of the angle between joint edges
          double cosine = fabs(angle(approx[j % 4], approx[j - 2], approx[j - 1]));
          maxCosine = MAX(maxCosine, cosine);
        }
        // 计算轮廓的直径宽高minAreaRect
        //  Rect aRect = boundingRect(contours[i]);
        RotatedRect aRect = minAreaRect(contours[i]);
        if (maxCosine < 0.3 && double(aRect.boundingRect().width) / double(aRect.boundingRect().height) > 0.85 && double(aRect.boundingRect().width) / double(aRect.boundingRect().height) < 1.15)
        {
          squares.push_back(approx);
        }
      }
    }
    for (size_t i = 0; i < squares.size(); i++)
    {
      Point *p = &squares[i][0];
      int n = (int)squares[i].size();
      // dont detect the border
      if (p->x > 3 && p->y > 3)
      {

        // cout << "center:" << center.x << endl;
        comput_center(p, n, center);

        // 对矩形和对应的序号存储
        sqtemp.num_edg = abs(p->x - center.x);
        sqtemp.center = center;
        sqtemp.approx = squares.at(i);
        sqall.push_back(sqtemp);
      }

      // cout << "center:" << center.x << endl;
    }
    // 在第一个色域找到若干目标后可不继续。
    if (sqall.size() > 4)
    {
      break;
    }
  }
  center = Major(sqall);
  out_center_dele(center, sqall);
  // cout << "center_x:" << center.x << "    center_y:" << center.y << endl;
}

static void findSquares_first(const Mat &image, vector<vector<Point>> &squares) // 原先没有去除绳子的
{
  squares.clear();
  // 按比例放大图像，滤除噪声
  // 中值滤波将增强边缘检测
  Mat timg1(image);
  medianBlur(image, timg1, 1);                                       // 测试效果
  Mat kernel = (Mat_<float>(3, 3) << 0, -1, 0, -1, 5, -1, 0, -1, 0); // 定义锐化卷积核

  filter2D(timg1, timg1, -1, kernel);
  medianBlur(timg1, timg1, 3);
  // imshow("ruihua", timg1);
  // waitKey(1);
  Mat gray0(timg1.size(), CV_8U), gray;
  vector<vector<Point>> contours;
  // find squares in every color plane of the image
  // 在图像的每个颜色平面上找到正方形
  for (int c = 0; c < 3; c++) // 单通道足够
  {
    int ch[] = {c, 0};
    mixChannels(&timg1, 1, &gray0, 1, ch, 1); // 将输入数组的指定通道复制到输出数组的指定通道
    cv::threshold(gray0, gray0, 60, 255, THRESH_OTSU);
    Mat erzhihua = gray0.clone();
    // imshow("erzhihua", erzhihua);
    waitKey(1);
    Canny(gray0, gray, 5, thresh, 5);
    dilate(gray, gray, Mat(), Point(-1, -1));
    Mat xianshi = gray.clone();
    // imshow("gray", xianshi);
    waitKey(1);
    // find contours and store them all as a list
    findContours(gray, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);
    vector<Point> approx;
    CvPoint corner[4];
    std::vector<cv::Point> corner_v;
    std::vector<cv::Point> tempapp;
    Point temp;
    // test each contour
    for (size_t i = 0; i < contours.size(); i++)
    {
      // 近似轮廓与精度成正比
      // 到轮廓周长
      approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true) * 0.02, true);

      if (approx.size() >= 4 && fabs(contourArea(Mat(approx))) > 1000 && isContourConvex(Mat(approx))) // 凸性检测 检测一个曲线是不是凸的&& fabs(contourArea(Mat(approx))) > 1000 && isContourConvex(Mat(approx))
      {
        double maxCosine = 0;
        for (int j = 2; j < 5; j++)
        {
          // find the maximum cosine of the angle between joint edges
          double cosine = fabs(angle(approx[j % 4], approx[j - 2], approx[j - 1]));
          maxCosine = MAX(maxCosine, cosine);
        }
        // 计算轮廓的直径宽高minAreaRect
        //  Rect aRect = boundingRect(contours[i]);
        RotatedRect aRect = minAreaRect(contours[i]);
        if (maxCosine < 0.3 && double(aRect.boundingRect().width) / double(aRect.boundingRect().height) > 0.85 && double(aRect.boundingRect().width) / double(aRect.boundingRect().height) < 1.15)
        {
          squares.push_back(approx);
        }
      }
    }
    for (size_t i = 0; i < squares.size(); i++)
    {
      Point *p = &squares[i][0];
      int n = (int)squares[i].size();
      // dont detect the border
      if (p->x > 3 && p->y > 3)
      {
        comput_center(p, n, center);

        // 对矩形和对应的序号存储
        sqtemp.num_edg = abs(p->x - center.x);
        sqtemp.center = center;
        sqtemp.approx = squares.at(i);
        sqall.push_back(sqtemp);
      }
    }
    // 在第一个色域找到若干目标后可不继续。
    if (sqall.size() > 4)
    {
      break;
    }
  }
}
// the function draws all the squares in the image
static void drawSquares(Mat &image, const vector<vector<Point>> &squares)
{
  for (size_t i = 0; i < squares.size(); i++)
  {
    const Point *p = &squares[i][0];
    int n = (int)squares[i].size();
    // dont detect the border
    if (p->x > 3 && p->y > 3)
      polylines(image, &p, &n, 1, true, Scalar(0, 0, 255), 3, LINE_AA);
    imshow("test2", image);
    // waitKey(99);
  }
  // imshow("test2", image);
  waitKey(9);
}

static void drawsqall(const Mat &image, vector<findsquares_4> &sqall)
{
  for (size_t i = 0; i < sqall.size(); i++)
  {
    const Point *p = &sqall.at(i).approx[0];
    int n = (int)sqall.at(i).approx.size();
    // dont detect the border
    if (p->x > 3 && p->y > 3)
      polylines(image, &p, &n, 1, true, Scalar(0, 0, 255), 3, LINE_AA);
  }
}
