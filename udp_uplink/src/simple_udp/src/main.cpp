#include <stdio.h>
#include <cstdlib>
#include <pthread.h>
#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include "simple_udp.h"
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <cstring>
// 测试时这些变量用宏写死，根据情况修改为可配置参数
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define MY_RATE 10 // 10 Hz

// 目标 UDP 服务器配置，发送数据
#define MY_DEST_ADDR "127.0.0.1" // 设置成目标 IP
#define MY_DEST_PORT 4096        // 根据协议修改

// 本地 UDP 服务器配置，接收数据
#define MY_SERV_ADDR "0.0.0.0" // 不用修改，"0.0.0.0" 表示监听所有网卡
#define MY_SERV_PORT 4096      // 根据协议修改
void *cli;

struct my_arg_struct
{
	int *running_flag;
	ros::Publisher *pub;
};

//创建一个新的数据结构，用于表示包含 x、y 和 yaw 的信息
struct UplinkData {
    int32_t xCoordinate;
    int32_t yCoordinate;
    int32_t yawAngle;
    uint8_t checksum;
};

// 结构体序列化（弃用） 
//std::string serializeUplinkData(const UplinkData& data) {
//    std::string result;
//    result.resize(sizeof(UplinkData));
//    memcpy(&result[0], &data, sizeof(UplinkData));
//    for (char c : result)
//    {
//     std::cout<<static_cast<int>(c) <<" ";
//    }
//    std::cout<<std::endl;
//    return result;
//}
// 序列化函数
void serializeUplinkData(const UplinkData& uplinkData) {
    // 将数据结构序列化为一个 std::vector<uint8_t>
    std::vector<uint8_t> serializedData;
    serializedData.resize(ros::serialization::serializationLength(uplinkData));
    ros::serialization::OStream stream(serializedData.data(), serializedData.size());
    ros::serialization::serialize(stream, uplinkData);

   
}
// 将 std::vector<uint8_t> 转换为 std::string
std::string vectorToString(const std::vector<uint8_t>& data) {
    return std::string(data.begin(), data.end());
}
// 计算 BCC 校验和
uint8_t calculateBCC(const void* data, size_t size) {
    const uint8_t* begin = reinterpret_cast<const uint8_t*>(data);
    const uint8_t* end = begin + size;
    uint8_t bcc = 0;

    while (begin < end) {
        bcc ^= *begin;
        ++begin;
    }

    return bcc;
}

// 收到 UDP 时回调，data 即收到的内容，我们把它通过 pub 发出去
void my_recv_handler(const std::string& data, void *arg_ptr)
{
	struct my_arg_struct *arg = (struct my_arg_struct*)arg_ptr;
	// 本例中遍历 data
	// 按照协议 data 如果不出意外每次应该只有一个字节
	for (auto i : data) {
		std_msgs::Int8 msg;
		msg.data = i;
		arg->pub->publish(msg);
	}
}

// UDP 服务器单独一个线程
void *my_udp_srv_thread(void *arg_ptr)
{
	struct my_arg_struct *arg = (struct my_arg_struct*)arg_ptr;
	my_udp_srv(MY_SERV_ADDR, MY_SERV_PORT,
		my_recv_handler, arg_ptr, SIMPLE_UDP_RECV_BUFSIZ, arg->running_flag);
	ROS_WARN("simple_udp server exited");
	return NULL;
}

//创建一个回调函数，用于处理订阅的里程计数据，并更新 x、y 和 yaw。
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // 更新 x、y 和 yaw
    int32_t x = msg->pose.pose.position.x * 100;  // 将单位转换为厘米
    int32_t y = msg->pose.pose.position.y * 100;  // 将单位转换为厘米
    int32_t yaw = static_cast<int32_t>(msg->pose.pose.orientation.z * 180 / M_PI);  // 将弧度转换为角度
	ros::Rate loop_rate(MY_RATE);
    // 创建包含 x、y 和 yaw 信息的结构体
    printf("data transmit start!\n");
    UplinkData uplinkData;
    uplinkData.xCoordinate = x;
    uplinkData.yCoordinate = y;
    uplinkData.yawAngle = yaw;
    //uplinkData.checksum = calculateChecksum(uplinkData);
    printf("data form odom got!\n");
    printf("x coordinate is %d\n",uplinkData.xCoordinate);
    // 使用序列化函数将结构体转为vector 
    serializeUplinkData(uplinkData);
    
    // 计算并附加 BCC 校验和
    uint8_t checksum = calculateBCC(uplinkData.data(), uplinkData.size());
    uplinkData.push_back(checksum);
    printf("checksum is %d\n",checksum);
    //printf("%s\n",serializedData[0]);
    printf("%s\n",uplinkData.data());
	// 将 std::vector<uint8_t> 转换为 std::string
	std::string serializedString = vectorToString(uplinkData);
    if (!cli) {
		ROS_ERROR("create udp client failed");
		exit(EXIT_FAILURE);
	}
	// 发送数据
	my_udp_send(cli, serializedString);
	//loop_rate.sleep();
	
	
    printf("data transmit over!\n");
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "simple_udp");
	ros::NodeHandle nh("~");
	ros::Rate loop_rate(MY_RATE);
	cli = my_udp_cli(MY_DEST_ADDR, MY_DEST_PORT);
	ros::Publisher pub = nh.advertise<std_msgs::Int8>("simple_int8", 1);
	// 订阅坐标和位姿信息，然后回调函数将消息发出去 
	ros::Subscriber odomSub = nh.subscribe("/imu_odometry/odom", 50, odomCallback);  // 替换为实际的里程计话题
	// 启动服务线程
	int running_flag = ~0; // running_flag 非0，让服务器持续运行
	struct my_arg_struct srv_arg = {
		.running_flag = &running_flag,
		.pub = &pub
	};
	pthread_t srv_pid;
	pthread_create(&srv_pid, NULL, my_udp_srv_thread, &srv_arg);
	pthread_detach(srv_pid);
	
	my_udp_cli_close(cli);
	
	
	
	ros::spin();
	return 0;
}
