#include <iostream>
#include <queue>
#include <ros/ros.h>
#include <serial/serial.h>
#include <smartcar_msgs/DiffSonic.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt8MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

double A = 1.00595663;
double B = 149.61707504;

double Q_Covariance_left;
double R_Covariance_left;
double pre_P_Covariance_left, cur_P_Covariance_left;
double filtered_dis_left, pre_dis_left, pre_pre_dis_left;
double K_left;

double Q_Covariance_right;
double R_Covariance_right;
double pre_P_Covariance_right, cur_P_Covariance_right;
double filtered_dis_right, pre_dis_right, pre_pre_dis_right;
double K_right;

double calibrated(int origin)
{
    return A * origin + B;
}

void linearKalmanFilter_left(const double& measure_value_left, const double& pre_value_left, const double& pre_P_Covariance_left,
    double& cur_value_left, double& cur_P_Covariance_left)
{
    /**
     * pre_speed是上一时刻的预测值  
     * pre_P_Covariance是上一时刻的预测值误差的协方差矩阵 
     * tmp_P_Covariance是这一时刻测量值误差的协方差矩阵的中间值 P_t|t-1
     * cur_P_Covariance是这一时刻测量值误差的协方差矩阵
     * Q_Covariance 是预测值的高斯噪声的协方差矩阵
     * K=卡尔曼增益
     * 本实例H=1 F=1
     * 符号定义参考：https://zhuanlan.zhihu.com/p/36745755
    */

    double tmp_P_Covariance = pre_P_Covariance_left + Q_Covariance_left;
    K_left = tmp_P_Covariance / (tmp_P_Covariance + R_Covariance_left);
    cur_value_left = pre_value_left + K_left * (measure_value_left - pre_value_left);
    cur_P_Covariance_left = tmp_P_Covariance - K_left * tmp_P_Covariance;
}

void linearKalmanFilter_right(const double& measure_value_right, const double& pre_value_right, const double& pre_P_Covariance_right,
    double& cur_value_right, double& cur_P_Covariance_right)
{
    /**
     * pre_speed是上一时刻的预测值  
     * pre_P_Covariance是上一时刻的预测值误差的协方差矩阵 
     * tmp_P_Covariance是这一时刻测量值误差的协方差矩阵的中间值 P_t|t-1
     * cur_P_Covariance是这一时刻测量值误差的协方差矩阵
     * Q_Covariance 是预测值的高斯噪声的协方差矩阵
     * K=卡尔曼增益
     * 本实例H=1 F=1
     * 符号定义参考：https://zhuanlan.zhihu.com/p/36745755
    */

    double tmp_P_Covariance = pre_P_Covariance_right + Q_Covariance_right;
    K_right = tmp_P_Covariance / (tmp_P_Covariance + R_Covariance_right);
    cur_value_right = pre_value_right + K_right * (measure_value_right - pre_value_right);
    cur_P_Covariance_right = tmp_P_Covariance - K_right * tmp_P_Covariance;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_serial_port");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::queue<double> q_left, q_right;
    double s_left, s_right;

    // ros::Publisher pub_left = nh.advertise<std_msgs::Float32>("/sonic_left", 1);
    // ros::Publisher pub_right = nh.advertise<std_msgs::Float32>("/sonic_right", 1);
    // std_msgs::Float32 sonic_left, sonic_right;

    std::string port_left, port_right;
    pnh.param<std::string>("port_left", port_left, "/dev/ttyUSB0");
    pnh.param<std::string>("port_right", port_right, "/dev/ttyUSB1");

    pnh.param("Q_Covariance_left", Q_Covariance_left, double(1.5));
    pnh.param("R_Covariance_left", R_Covariance_left, double(1.0));
    pre_P_Covariance_left = Q_Covariance_left / 10.;
    pre_dis_left = 10;
    pre_pre_dis_left = 10;

    pnh.param("Q_Covariance_right", Q_Covariance_right, double(1.5));
    pnh.param("R_Covariance_right", R_Covariance_right, double(1.0));
    pre_P_Covariance_right = Q_Covariance_right / 10.;
    pre_dis_right = 10;
    pre_pre_dis_right = 10;

    ros::Publisher pub_sonic_raw_ = nh.advertise<smartcar_msgs::DiffSonic>("/sonic_raw", 1);
    ros::Publisher pub_sonic_filtered_ = nh.advertise<smartcar_msgs::DiffSonic>("/sonic_filtered", 1);
    smartcar_msgs::DiffSonic msg_sonic;
    smartcar_msgs::DiffSonic msg_sonic_filtered;

    serial::Serial sp_left;
    serial::Timeout to = serial::Timeout::simpleTimeout(20);
    sp_left.setPort(port_left);
    sp_left.setBaudrate(115200);
    sp_left.setTimeout(to);

    serial::Serial sp_right;
    sp_right.setPort(port_right);
    sp_right.setBaudrate(115200);
    sp_right.setTimeout(to);

    try {
        sp_left.open();
        sp_right.open();
    } catch (serial::IOException& e) {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }

    ros::Rate loop_rate(50);

    int count_left = 0;
    int count_right = 0;

    while (ros::ok()) {
        size_t n_left = sp_left.available();
        size_t n_right = sp_right.available();
        if (n_left == 0 || n_right == 0) {
            ROS_WARN_STREAM("Error Happened at sonic driver");
            if (n_left == 0)
                ROS_WARN_STREAM("sonic left receives no data");
            if (n_right == 0)
                ROS_WARN_STREAM("sonic right receives no data");
            loop_rate.sleep();
            continue;
        }

        uint8_t buffer_left[1024];
        uint8_t buffer_right[1024];

        n_left = sp_left.read(buffer_left, n_left);
        n_right = sp_right.read(buffer_right, n_right);

        if (n_left < 3 || n_right < 3) {
            ROS_WARN_STREAM("Sonic read length error");
            loop_rate.sleep();
            continue;
        }
        if ((buffer_left[0] & 0xff) != 165 || (buffer_right[0] & 0xff) != 165) {
            ROS_WARN_STREAM("Sonic read data error");
            loop_rate.sleep();
            continue;
        }

        int left_origin = (buffer_left[1] & 0xff) * 256 + (buffer_left[2] & 0xff);
        int right_origin = (buffer_right[1] & 0xff) * 256 + (buffer_right[2] & 0xff);

        double raw_dis_left = calibrated(left_origin);
        double raw_dis_right = calibrated(right_origin);

        linearKalmanFilter_left(raw_dis_left, pre_dis_left, pre_P_Covariance_left, filtered_dis_left, cur_P_Covariance_left);
        linearKalmanFilter_right(raw_dis_right, pre_dis_right, pre_P_Covariance_right, filtered_dis_right, cur_P_Covariance_right);
        msg_sonic.type = "sonic_good";
        msg_sonic.left = filtered_dis_left;
        msg_sonic.right = filtered_dis_right;
        pub_sonic_raw_.publish(msg_sonic);

        loop_rate.sleep();

        // if (n_left != 0) {
        //     uint8_t buffer[1024];
        //     n_left = sp_left.read(buffer, n_left);
        //     // for (int i = 0; i < n_left; i++) {
        //     //     //16进制的方式打印到屏幕
        //     //     std::cout << std::hex << (buffer[i] & 0xff) << " ";
        //     // }
        //     if (n_left < 3 || (buffer[0] & 0xff) != 165) {
        //         count_left++;
        //         continue;
        //     } else {
        //         int a = buffer[1] & 0xff;
        //         int b = buffer[2] & 0xff;
        //         int left_origin = a * 256 + b;
        //         // sonic_left.data = calibrated(left_origin);
        //         // pub_left.publish(sonic_left);
        //         msg_sonic.left = calibrated(left_origin);
        //         count_left = 0;
        //         if (q_left.size() > 5) {
        //             double out = q_left.front();
        //             q_left.pop();
        //             q_left.push(msg_sonic.left);
        //             s_left -= out;
        //             s_left += msg_sonic.left;
        //             msg_sonic_filtered.left = s_left / 5.0;
        //         } else {
        //             q_left.push(msg_sonic.left);
        //             s_left += msg_sonic.left;
        //         }
        //     }
        //     // std::cout << " left: " << msg_sonic.left << std::endl;
        // } else {
        //     count_left++;
        // }

        // if (n_right != 0) {
        //     uint8_t buffer[1024];
        //     n_right = sp_right.read(buffer, n_right);
        //     if (n_right < 3 || (buffer[0] & 0xff) != 165) {
        //         count_right++;
        //         continue;
        //     } else {
        //         int a = buffer[1] & 0xff;
        //         int b = buffer[2] & 0xff;
        //         int right_origin = a * 256 + b;
        //         // sonic_right.data = calibrated(right_origin);
        //         // pub_right.publish(sonic_right);
        //         msg_sonic.right = calibrated(right_origin);
        //         count_right = 0;
        //         if (q_right.size() > 5) {
        //             double out = q_right.front();
        //             q_right.pop();
        //             q_right.push(msg_sonic.right);
        //             s_right -= out;
        //             s_right += msg_sonic.right;
        //             msg_sonic_filtered.right = s_left / 5.0;
        //         } else {
        //             q_right.push(msg_sonic.left);
        //             s_right += msg_sonic.left;
        //         }
        //     }
        //     // std::cout << "right: " << msg_sonic.right << std::endl;
        // } else {
        //     count_right++;
        // }
        // if (count_left > 10 || count_right > 10) {
        //     if (count_left > 10) {
        //         msg_sonic.type = "sonic_left_fail";
        //         ROS_WARN_STREAM("sonic_left_fail");
        //     } else {
        //         msg_sonic.type = "sonic_right_fail";
        //         ROS_WARN_STREAM("sonic_right_fail");
        //     }
        //     msg_sonic.left = 0.0;
        //     msg_sonic.right = 0.0;
        //     pub_sonic_raw_.publish(msg_sonic);
        // } else {
        //     msg_sonic.type = "sonic_good";
        //     pub_sonic_raw_.publish(msg_sonic);

        //     msg_sonic_filtered.type = "sonic_good";
        //     pub_sonic_filtered_.publish(msg_sonic_filtered);
        // }
        // loop_rate.sleep();
    }

    //关闭串口
    sp_left.close();
    sp_right.close();

    return 0;
}
