/***************************************************************************************************************************
 * autonomous_landing.cpp
 *
 * Author: Qyp
 *
 * Update Time: 2019.4.17
 *
 * 说明: 基于单目摄像头的自主降落程序
 *      1. 订阅目标位置(来自视觉的ros节点)
 *      2. 追踪算法及降落策略
 *      3. 发布上层控制指令
 ***************************************************************************************************************************/
// ROS 头文件
#include <ros/ros.h>

// ros头文件
#include <ros/ros.h>
#include <tf/tf.h>

// topic 头文件
#include <iostream>
#include <command_to_mavros.h>
#include <fly_util/command.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>

using namespace std;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
enum Command
{
  Move_ENU,
  Move_Body,
  Hold,
  Land,
  Disarm,
  Failsafe_land,
  Idle,
  Takeoff
};

int Num_StateMachine = 0;      // 状态机编号
int Num_StateMachine_Last = 0; // 上一时刻 状态机编号
int comid = 1;                 // Command_now的id号
//-----------------------------------------视觉相关----------------------------------------------------
float flag_vision = 0;                                       // 视觉FLAG 是否识别到目标 1代表能识别到目标，0代表不能
geometry_msgs::Point vision_relative_position;               // 机体固连坐标系下 降落板的位置
geometry_msgs::Point GPS_position;                           // 机体固连坐标系下 GPS的位置
geometry_msgs::Point Last_position;                          // 机体固连坐标系下 GPS的位置
geometry_msgs::Point Pos_error, Pos_error_last, Pos_error_i; // 机体固连坐标系下 GPS的位置
geometry_msgs::Point Pos_error_error;                        // 机体固连坐标系下 GPS的位置
geometry_msgs::Point GPS_ori_position;                       // 机体固连坐标系下 GPS第一帧的位置
geometry_msgs::Point GPS_offset;                             // 机体固连坐标系下 GPS第一帧的位置
geometry_msgs::Point GPS_relation;                           // 机体固连坐标系下 GPS与第一帧偏差的位置
geometry_msgs::Point GPS_first;                              // 机体固连坐标系下 GPS与第一帧偏差的位置
geometry_msgs::Point Chuanc[10];                             // 机体固连坐标系下 GPS与第一帧偏差的位置
double relative_yaw, relative_x, relative_y;                 // 降落板与无人机的相对偏航角 单位:rad

int test_relative = 1;   // 是否输出旋转测试数据
int orb_or_huizi = 1;    // orb 0 huizi 1
float relative_yaw_last; // 最终偏转角度
float height_on_pad;     // 降落板高度
//-----------------------------------------降落控制相关----------------------------------------------------
float kpx_land, kpy_land, kpz_land;                      // 控制参数 - 比例参数
float kdx_land, kdy_land, kdz_land;                      // 控制参数 - 比例参数
float Max_velx_land, Max_vely_land, Max_velz_land;       // 控制参数 - 最大值限幅
float Thres_velx_land, Thres_vely_land, Thres_velz_land; // 控制参数 - 死区限幅

float distance_pad;                          // 无人机与降落板中心的距离
float Thres_distance_land, Thres_count_land; // 控制参数 - 阈值（追踪程序）
int flag_track_yaw = 0;
int num_count = 0;
float fly_min_z;
int Flag_reach_pad_center = 0;
int Flag_z_below_30cm = 0;
float land_max_z;

int num_count_lost = 0;
float Thres_vision_lost = 30;
//---------------------------------------Output---------------------------------------------
fly_util::command Command_now; // 发送给position_control.cpp的命令
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>函数声明<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
float satfunc(float data, float Max, float Thres); // 限幅函数
void track_land();
void land_end();
void printf_land();
geometry_msgs::Point Point_offset(geometry_msgs::Point a, geometry_msgs::Point b);
geometry_msgs::Point coord_trans(geometry_msgs::Point a, double theta);
void generate_com(int sub_mode, float state_desired[4]);
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回调函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void vision_relative_position_cb(const geometry_msgs::Pose::ConstPtr &msg)
{
  if (orb_or_huizi == 0) // orb  vision_relative_position.x的目标是前x，左y
  {
    vision_relative_position.x = msg->position.y;
    vision_relative_position.y = -msg->position.x;
    vision_relative_position.z = msg->position.z;
    flag_vision = msg->orientation.w;
  }
  else
  {
    vision_relative_position.x = msg->position.x;
    vision_relative_position.y = msg->position.y;
    vision_relative_position.z = msg->position.z;
    flag_vision = msg->orientation.w;
  }
}

void relative_yaw_cb(const geometry_msgs::Pose::ConstPtr &msg)
{
  relative_yaw = msg->orientation.w;
}
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
  relative_x = relative_x * 180 / 3.14; // 这里x是无人机的Pitch，在更换控制律时坐标系会随之改变，请注意
  relative_y = relative_y * 180 / 3.14; // roll

  GPS_ori_position = coord_trans(msg->pose.position, (relative_yaw) / 180 * 3.14);
  if (test_relative == 1)
  {
    cout << "relative_yaw:" << relative_yaw << " relative_x:" << relative_x << "relative_y:" << relative_y << endl;
    cout << "GPS_position:x:" << GPS_ori_position.x << "  y:" << GPS_ori_position.y << "z:" << GPS_ori_position.z << " relative_yaw_last:" << relative_yaw_last << endl;
    cout << "relative_yaw:" << relative_yaw - 90 << endl;
  }
}

void vision_flag(const geometry_msgs::Pose::ConstPtr &msg)
{
  flag_vision = msg->orientation.w;
}

mavros_msgs::State current_state; // 无人机当前状态[包含上锁状态 模式] (从飞控中读取)
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回调函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
  current_state = *msg;
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主函数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
  ros::init(argc, argv, "autonomous_landing");
  ros::NodeHandle nh("~");

  // 节点运行频率： 20hz 【视觉端解算频率大概为20HZ】
  ros::Rate rate(20.0);

  // 【订阅】降落板与无人机的相对位置 单位：米
  //  来自视觉节点 方向定义：[机体系下：前方x为正，右方y为正，下方z为正]
  ros::Subscriber relative_pos_sub = nh.subscribe<geometry_msgs::Pose>("/vision/vision_relative_position", 10, vision_relative_position_cb);
  // 【订阅】GPS位置
  //  来自视觉节点 方向定义：[机体系下：前方x为正，右方y为正，下方z为正]
  ros::Subscriber GPS_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, GPS_cb);

  // 【订阅】无人机当前状态 - 来自飞控
  //  本话题来自飞控(通过/plugins/sys_status.cpp)
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);

  // 【订阅】降落板与无人机的相对偏航角 单位：弧度
  //  利用orientation.w 传递
  ros::Subscriber relative_yaw_sub = nh.subscribe<geometry_msgs::Pose>("/vison/relative_yaw", 10, relative_yaw_cb);

  // 【订阅】视觉flag 来自视觉节点
  //  orientation.w ： 0 for目标丢失，1 for 正常识别
  ros::Subscriber vision_flag_sub = nh.subscribe<geometry_msgs::Pose>("/vision/vision_flag", 10, vision_flag);

  // 【发布】发送给position_control.cpp的命令
  ros::Publisher command_pub = nh.advertise<fly_util::command>("/px4/command", 10);

  //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>参数读取<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  // 降落追踪控制算法 的比例参数
  nh.param<float>("kpx_land", kpx_land, 0.5);
  nh.param<float>("kpy_land", kpy_land, 0.5);
  nh.param<float>("kpz_land", kpz_land, 0.2);

  nh.param<float>("kdx_land", kdx_land, 0.3);
  nh.param<float>("kdy_land", kdy_land, 0.3);
  nh.param<float>("kdz_land", kdz_land, 0.2);

  // 降落追踪控制算法的最大速度
  nh.param<float>("Max_velx_land", Max_velx_land, 0.6);
  nh.param<float>("Max_vely_land", Max_vely_land, 0.6);
  nh.param<float>("Max_velz_land", Max_velz_land, 0.3);

  // 降落追踪控制算法的速度死区
  nh.param<float>("Thres_velx_land", Thres_velx_land, 0.001);
  nh.param<float>("Thres_vely_land", Thres_vely_land, 0.001);
  nh.param<float>("Thres_velz_land", Thres_velz_land, 0.00);

  // 允许降落最大距离阈值
  nh.param<float>("Thres_distance_land", Thres_distance_land, 0.1);

  // 允许降落计数阈值
  nh.param<float>("Thres_count_land", Thres_count_land, 15);

  // 允许降落最大高度阈值
  nh.param<float>("land_max_z", land_max_z, 0.3);

  // 允许飞行最低高度[这个高度是指降落板上方的相对高度]
  nh.param<float>("fly_min_z", fly_min_z, -1);

  // 视觉丢失计数阈值
  nh.param<float>("Thres_vision_lost", Thres_vision_lost, 30);

  // 降落板高度
  nh.param<float>("height_on_pad", height_on_pad, -0.4);

  //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>模式选择<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  int check_flag, takeoff_flag = 0;
  // 输入1,继续，其他，退出程序
  //  cout << "Please check the parameter and setting，1 for go on， else for quit: "<<endl;
  //  cin >> check_flag;

  // if(check_flag != 1)
  // {
  //     return -1;
  // }

  Num_StateMachine = 0;

  int flag_command;       // 机体系FLAG
  float state_desired[4]; // cin的目标位置点

  while (ros::ok())
  {
    // 回调
    ros::spinOnce();

    printf_land();
    if (GPS_ori_position.z >= 3)
    {
      takeoff_flag = 1;
    }

    switch (Num_StateMachine)
    {
    // input
    case 0:
      Num_StateMachine_Last = Num_StateMachine;

      // cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>--------<<<<<<<<<<<<<<<<<<<<<<<<<<< "<< endl;
      cout << "Please input command [0 for move[ned],1 for move[body], 2 for land, 777 for autonomous landing]:    there please input 777 for auto landing: " << endl;
      // cin >> flag_command;
      flag_command = 777;
      // 777 track_land
      if (flag_command == 777)
      {
        ros::spinOnce();
        relative_yaw_last = relative_yaw;
        // ros::spinOnce();
        GPS_first = GPS_ori_position;
        GPS_first.z = -1;
        Num_StateMachine = 4;
        break;
      }
      // 999  land
      else if (flag_command == 2)
      {
        Num_StateMachine = 3;
        break;
      }
      else if (flag_command == 3)
      {
        Num_StateMachine = 3;
        break;
      }

      cout << "Please input next setpoint [x y z yaw]: " << endl;

      cout << "setpoint_t[0] --- x [m] : " << endl;
      cin >> state_desired[0];
      cout << "setpoint_t[1] --- y [m] : " << endl;
      cin >> state_desired[1];
      cout << "setpoint_t[2] --- z [m] : " << endl;
      cin >> state_desired[2];
      cout << "setpoint_t[3] --- yaw [du] : " << endl;
      cout << "500 for input again" << endl;
      cin >> state_desired[3];

      // 500  重新输入各数值
      if (state_desired[3] == 500)
      {
        Num_StateMachine = 0;
      } // 如果是机体系移动
      else if (flag_command == 1)
      {
        Num_StateMachine = 2;
      } // 惯性系移动
      else if (flag_command == 0)
      {
        Num_StateMachine = 1;
      }
      else
      {
        Num_StateMachine = 0;
      }

      break;

    // 惯性系移动
    case 1:
      Command_now.command = Move_ENU;
      generate_com(0, state_desired);
      command_pub.publish(Command_now);

      Num_StateMachine_Last = Num_StateMachine;
      Num_StateMachine = 0;
      break;

    // 机体系移动
    case 2:
    {
      Command_now.command = Move_Body;
      generate_com(3, state_desired);
      int temp = 1;
      while (temp == 1)
      {
        command_pub.publish(Command_now);
        cout << "end for do not 1" << endl;
        cin >> temp;
        ros::spinOnce();
      }

      command_pub.publish(Command_now);

      Num_StateMachine_Last = Num_StateMachine;
      Num_StateMachine = 0;
      break;
    }
    // Land
    case 3:
    {
      int temp = 1;
      while (temp == 1)
      {
        Command_now.command = Hold;
        Command_now.sub_mode = 1; // xy pos z vel
        Command_now.comid = comid;
        Command_now.pos_sp[0] = 0;
        Command_now.pos_sp[1] = 0;
        Command_now.vel_sp[2] = 0;
        Command_now.yaw_sp = 0;
        comid++;

        Num_StateMachine_Last = Num_StateMachine;
        command_pub.publish(Command_now);
        command_pub.publish(Command_now);
        cout << "end for do not 1" << endl;
        cin >> temp;
        ros::spinOnce();
      }
      while (temp == 2)
      {
        Command_now.command = Land;
        command_pub.publish(Command_now);
        cout << "end for do not 2" << endl;
        cin >> temp;
      }
      Command_now.command = Land;
      command_pub.publish(Command_now);
      Num_StateMachine_Last = Num_StateMachine;
      break;
    }
    // 追踪降落
    case 4:

      // 自主降落追踪算法
      track_land();
      // 测试用 后续修改

      // flag_vision=1;
      // 计算与降落板之间的距离
      // distance_pad = sqrt(vision_relative_position.x * vision_relative_position.x + vision_relative_position.y * vision_relative_position.y);
      distance_pad = sqrt(GPS_position.x * GPS_position.x + GPS_position.y * GPS_position.y);
      cout << "Distance_pad: " << distance_pad << endl;
      cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>Land State<<<<<<<<<<<<<<<<<<<<<<<<<<<" << endl;
      // 降落的2个条件：
      // 1：水平距离小于阈值，即无人机抵达降落板中心位置附近
      // 2：相对高度小于阈值，

      // 如果 相对距离 小于 阈值，计数增加
      if (distance_pad < Thres_distance_land) // && flag_vision == 1
      {
        num_count++;
        cout << "Distance_pad: " << distance_pad << endl;
        cout << "Distance_land_count: " << num_count << endl;
      }
      else
      {
        num_count = 0;
        cout << "Distance_pad: " << distance_pad << endl;
        cout << "Distance_land_count: " << num_count << endl;
      }

      // 如果计数增加超过阈值，则满足降落的第一个条件（水平距离）
      if (distance_pad < Thres_distance_land && num_count > Thres_count_land)
      {
        cout << "Flag_reach_pad_center: "
             << "true" << endl;
        Flag_reach_pad_center = 1;
      }
      else
      {
        cout << "Flag_reach_pad_center: "
             << "flase" << endl;
        Flag_reach_pad_center = 0;
      }

      // 如果 相对高度 小于 阈值，则满足降落的第二个条件（高度条件）
      if (vision_relative_position.z <= 0.25 && flag_vision >= 1 && GPS_ori_position.z < 1) //||GPS_ori_position.z <= land_max_z
      {
        cout << "Flag_z_below_30cm: "
             << "true" << endl;
        Flag_z_below_30cm += 1;
      }
      else
      {
        cout << "Flag_z_below_30cm: "
             << "flase" << endl;
        Flag_z_below_30cm = 0;
      }
      cout << "Flag_reach_pad_center:" << Flag_reach_pad_center << endl;
      cout << "Flag_z_below_30cm:" << Flag_z_below_30cm << endl;
      cout << "takeoff_flag:" << takeoff_flag << endl;
      cout << "flag_vision:" << flag_vision << endl;
      // 如果降落的两个条件都满足，则切换状态机（上锁）
      if (Flag_reach_pad_center == 1 && Flag_z_below_30cm >= 30 && takeoff_flag == 1 && flag_vision == 2)
      {
        // arm!
        takeoff_flag = 0;
        Num_StateMachine = 5;
      }
      else
      {
        // keep track
        Num_StateMachine = 4;
      }

      // 如果视觉丢失了降落板目标，计数增加
      if (flag_vision == 0)
      {
        num_count_lost++;
        // cout<< "vision_lost_num: " << num_count_lost <<endl;
      }
      else
      {
        num_count_lost = 0;
        // cout<< "vision_lost_num: " << num_count_lost <<endl;
      }

      // 如果丢失计数超过阈值，则切换状态机（爬升）
      //  if(num_count_lost > Thres_vision_lost)
      //  {
      //      num_count_lost = 0;
      //      Num_StateMachine = 6;
      //  }

      // 发布控制量
      command_pub.publish(Command_now);
      Num_StateMachine_Last = Num_StateMachine;

      break;

    case 5:

      // land_end();
      Command_now.command = Land;
      command_pub.publish(Command_now);
      Num_StateMachine_Last = Num_StateMachine;
      cout << "current_state" << current_state.armed << endl;
      if (current_state.armed == 0)
      {
        Num_StateMachine = 4;
        GPS_first = Point_offset(GPS_ori_position, GPS_first);
      }

      break;

    case 6:

      Command_now.command = Hold;
      Command_now.sub_mode = 1; // xy pos z vel
      Command_now.comid = comid;
      Command_now.pos_sp[0] = 0;
      Command_now.pos_sp[1] = 0;
      Command_now.vel_sp[2] = 0;
      Command_now.yaw_sp = 0;
      comid++;

      Num_StateMachine_Last = Num_StateMachine;
      command_pub.publish(Command_now);

      cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>Search State<<<<<<<<<<<<<<<<<<<<<<<<<<<" << endl;
      // 重新获得视觉信息，计数
      if (flag_vision >= 1)
      {
        num_count_lost++;
        cout << "vision_regain_num: " << num_count_lost << endl;
      }
      else
      {
        num_count_lost = 0;
        cout << "vision_regain_num: " << num_count_lost << endl;
      }

      // 如果重新获得视觉计数超过阈值，则切换状态机（追踪降落）
      if (num_count_lost > Thres_vision_lost)
      {
        num_count_lost = 0;
        Num_StateMachine = 4;
      }

      break;
    }

    rate.sleep();
  }

  return 0;
}

// 自主降落追踪算法
void track_land()
{
  // 位置求解
  // TODO
  // yaw offset do not change;!!!!!!!!!!!!!!!!!!!!!!!
  GPS_relation = Point_offset(GPS_ori_position, GPS_first);
  if (flag_vision >= 1)
  {
    GPS_offset = Point_offset(GPS_relation, vision_relative_position);
  }
  GPS_position = Point_offset(GPS_relation, GPS_offset);

  Command_now.command = Move_Body;
  Command_now.comid = comid;
  comid++;

  // xyz速度控制模式
  Command_now.sub_mode = 3; // xy velocity z velocity
  // Command_now.vel_sp[0] =  -kpx_land * vision_relative_position.x;
  // Command_now.vel_sp[1] =  - kpy_land * vision_relative_position.y;
  // Command_now.vel_sp[2] =  - kpz_land * (vision_relative_position.z - fly_min_z);
  Pos_error = Point_offset(GPS_position, Last_position);
  Last_position = GPS_position;
  Pos_error_i.x += (double)Pos_error.x / 10;
  Pos_error_i.y += (double)Pos_error.y / 10;
  Pos_error_i.x = satfunc(Pos_error_i.x, 1, 0);
  Pos_error_i.y = satfunc(Pos_error_i.y, 1, 0);
  Pos_error_error = Point_offset(Pos_error, Pos_error_last);
  if (flag_vision >= 1)
  {
    double temp1, temp0;
    temp1 = kpy_land * GPS_position.y + Pos_error_i.y + kdy_land * Pos_error.y; // gpsy  Command_now原始机头为0  飞机左侧为y
    temp0 = kpx_land * GPS_position.x + Pos_error_i.x + kdx_land * Pos_error.x; //  -zheng  机头
                                                                                // 饱和函数

    Command_now.vel_sp[0] = satfunc(0.25 * (vision_relative_position.z + 1) * temp0, 1.4, Thres_velx_land);
    Command_now.vel_sp[1] = satfunc(0.25 * (vision_relative_position.z + 1) * temp1, 1.4, Thres_vely_land);
    Command_now.vel_sp[2] = 0.5 * (fabs(GPS_position.x) > fabs(GPS_position.y) ? fabs(Command_now.vel_sp[0]) - Max_velx_land
                                                                               : fabs(Command_now.vel_sp[1]) - Max_velx_land); // kpz_land * (GPS_ori_position.z - fly_min_z);
  }
  else
  {
    Command_now.vel_sp[1] = -kpx_land * GPS_position.y; // gpsy  Command_now原始机头为0  飞机左侧为y
    Command_now.vel_sp[0] = -kpy_land * GPS_position.x; //  -zheng  机头
                                                        // 饱和函数
    Command_now.vel_sp[0] = satfunc(Command_now.vel_sp[0], Max_velx_land, Thres_velx_land);
    Command_now.vel_sp[1] = satfunc(Command_now.vel_sp[1], Max_vely_land, Thres_vely_land);
    Command_now.vel_sp[2] = 0.5 * (fabs(GPS_position.x) > fabs(GPS_position.y) ? fabs(Command_now.vel_sp[0]) - Max_velx_land
                                                                               : fabs(Command_now.vel_sp[1]) - Max_velx_land); // kpz_land * (GPS_ori_position.z - fly_min_z);
  }
  Chuanc[3].x = Command_now.vel_sp[0];
  Chuanc[3].y = Command_now.vel_sp[1];
  double temp3[2];
  for (size_t i = 0; i < 3; i++)
  {
    Chuanc[i] = Chuanc[i + 1];
    temp3[0] += Chuanc[i].x;
    temp3[1] += Chuanc[i].y;
  }

  // 不追踪偏航角，则锁死偏航角为0
  Command_now.yaw_sp = relative_yaw_last - 90;
  // cout<<">>>>>>>>>>>>>>>>>>>>>>>>>>>>test in track_land<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<"<<endl;
  // cout << "target: " << GPS_first.x << " [m] "<< GPS_first.y << " [m] "<< GPS_first.z << " [m] "<<endl;
  // cout<<"now :  x:"<< GPS_ori_position.x <<" y:"<<GPS_ori_position.y<<"  z:"<< GPS_ori_position.z <<endl;
  // cout<<"GPS_trans :  x:"<<GPS_position.x <<" y:"<<GPS_position.y<<"  z:"<< GPS_position.z <<endl;

  // 饱和函数
  Command_now.vel_sp[0] = satfunc(temp3[0] / 3, Max_velx_land, Thres_velx_land);
  Command_now.vel_sp[1] = satfunc(temp3[1] / 3, Max_vely_land, Thres_vely_land);
  if (distance_pad <= 0.20 && flag_vision == 0)
  {
    Command_now.vel_sp[2] = satfunc(Command_now.vel_sp[2], Max_velz_land, Thres_velz_land);
  }
  else if (distance_pad <= vision_relative_position.z && flag_vision >= 1 && vision_relative_position.z >= 1)
  {
    Command_now.vel_sp[2] = satfunc(Command_now.vel_sp[2], Max_velz_land, Thres_velz_land);
  }
  else if (distance_pad <= vision_relative_position.z && flag_vision >= 1 && vision_relative_position.z < 1 && vision_relative_position.z >= 0.4)
  {
    Command_now.vel_sp[2] = satfunc(Command_now.vel_sp[2], Max_velz_land, Thres_velz_land);
  }
  else if (distance_pad <= vision_relative_position.z && flag_vision >= 1 && vision_relative_position.z < 0.4)
  {
    Command_now.vel_sp[2] = satfunc(Command_now.vel_sp[2], Max_velz_land, Thres_velz_land);
  }
  else
  {
    Command_now.vel_sp[2] = 0;
  }
  cout << "Command :  x:" << Command_now.vel_sp[0] << " y:" << Command_now.vel_sp[1] << "  z:" << Command_now.vel_sp[2] << endl;
}
void land_end()
{
  Command_now.command = Move_Body;
  Command_now.comid = comid;
  comid++;

  // xyz速度控制模式
  //  Command_now.sub_mode = 3; // xy velocity z velocity
  //  //如果要去追踪一个动态的降落板，则需要反馈其速度
  //  Command_now.vel_sp[0] =  0;
  //  Command_now.vel_sp[1] =  0;

  Command_now.sub_mode = 1; // xy pos z vel
  Command_now.pos_sp[0] = 0;
  Command_now.pos_sp[1] = 0;
  Command_now.vel_sp[2] = -0.7;
  // 不追踪偏航角，则锁死偏航角为0
  Command_now.yaw_sp = relative_yaw_last - 90;
  // 饱和函数
  Command_now.vel_sp[0] = satfunc(Command_now.vel_sp[0], Max_velx_land, Thres_velx_land);
  Command_now.vel_sp[1] = satfunc(Command_now.vel_sp[1], Max_vely_land, Thres_vely_land);
  Command_now.vel_sp[2] = satfunc(Command_now.vel_sp[2], Max_velz_land, Thres_velz_land);
}

// 饱和函数
float satfunc(float data, float Max, float Thres)
{
  if (abs(data) < Thres)
  {
    return 0;
  }
  else if (abs(data) > Max)
  {
    return (data > 0) ? Max : -Max;
  }
  else
  {
    return data;
  }
}
// float32[3] pos_sp
// float32[3] vel_sp
// float32 yaw_sp
void generate_com(int sub_mode, float state_desired[4])
{
  static int comid = 1;
  Command_now.sub_mode = sub_mode;

  // # sub_mode 2-bit value:
  // # 0 for position, 1 for vel, 1st for xy, 2nd for z.
  // #                   xy position     xy velocity
  // # z position       	0b00(0)       0b10(2)
  // # z velocity		      0b01(1)       0b11(3)

  if ((sub_mode & 0b10) == 0) // xy channel
  {
    Command_now.pos_sp[0] = state_desired[0];
    Command_now.pos_sp[1] = state_desired[1];
    cout << "pos_sp xy" << endl;
  }
  else
  {
    Command_now.vel_sp[0] = state_desired[0];
    Command_now.vel_sp[1] = state_desired[1];
    cout << "sudo xy" << endl;
  }

  if ((sub_mode & 0b01) == 0) // z channel
  {
    Command_now.pos_sp[2] = state_desired[2];
    cout << "pos z" << endl;
  }
  else
  {
    Command_now.vel_sp[2] = state_desired[2];
    cout << "sudo z" << endl;
  }

  Command_now.yaw_sp = state_desired[3];
  Command_now.comid = comid;
  comid++;
}

void printf_land()
{
  cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>Autonomous Landing<<<<<<<<<<<<<<<<<<<<<<<<<<" << endl;
  cout << "Num_StateMachine :     " << Num_StateMachine << endl;

  cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>local position State<<<<<<<<<<<<<<<<<<<<<<<<<<" << endl;
  cout << "GPS local" << endl; // GPS_offset=Point_offset(GPS_relation,vision_relative_position);
  cout << "GPS 原始数据     : " << GPS_ori_position.x << " [m] " << GPS_ori_position.y << " [m] " << GPS_ori_position.z << " [m] " << endl;
  cout << "GPS 最终使用数据   : " << GPS_position.x << " [m] " << GPS_position.y << " [m] " << GPS_position.z << " [m] " << endl;
  cout << "GPS 无视觉的目标数据 : " << GPS_relation.x << " [m] " << GPS_relation.y << " [m] " << GPS_relation.z << " [m] " << endl;
  cout << "GPS 与第一帧偏差的位置 : " << GPS_offset.x << " [m] " << GPS_offset.y << " [m] " << GPS_offset.z << " [m] " << endl;
  cout << "GPS 初始数据     : " << GPS_first.x << " [m] " << GPS_first.y << " [m] " << GPS_first.z << " [m] " << endl;
  cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>command state<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << endl;
  cout << "qian x zuo y  Command :  x:" << Command_now.vel_sp[0] << " y:" << Command_now.vel_sp[1] << " z:" << Command_now.vel_sp[2] << endl;

  cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>Vision State<<<<<<<<<<<<<<<<<<<<<<<<<<" << endl;
  cout << "flag_vision: " << flag_vision << endl;
  cout << "relative position: " << vision_relative_position.x << " [m] " << vision_relative_position.y << " [m] " << vision_relative_position.z << " [m] " << endl;
  cout << "relative_yaw_last: " << relative_yaw_last << " [du] " << endl;
}

geometry_msgs::Point Point_offset(geometry_msgs::Point a, geometry_msgs::Point b)
{
  geometry_msgs::Point result;
  result.x = a.x - b.x;
  result.y = a.y - b.y;
  // result.z=a.z-b.z;
  return result;
}

geometry_msgs::Point coord_trans(geometry_msgs::Point a, double theta)
{
  geometry_msgs::Point b;
  b.x = a.x * cos(theta) + a.y * sin(theta);
  b.y = -a.x * sin(theta) + a.y * cos(theta);
  // cout<<"test sin:"<<sin(theta)<<"test cos:"<<cos(theta)<<endl;
  // cout<<"coord_trans:"<<b.x<<"coordy:"<<b.y<<endl;
  b.z = a.z;
  return b;
}
