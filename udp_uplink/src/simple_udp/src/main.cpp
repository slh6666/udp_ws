#include <stdio.h>
#include <cstdlib>
#include <pthread.h>
#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include "simple_udp.h"
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <cstring>
// ����ʱ��Щ�����ú�д������������޸�Ϊ�����ò���
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define MY_RATE 10 // 10 Hz

// Ŀ�� UDP ���������ã���������
#define MY_DEST_ADDR "127.0.0.1" // ���ó�Ŀ�� IP
#define MY_DEST_PORT 4096        // ����Э���޸�

// ���� UDP ���������ã���������
#define MY_SERV_ADDR "0.0.0.0" // �����޸ģ�"0.0.0.0" ��ʾ������������
#define MY_SERV_PORT 4096      // ����Э���޸�
void *cli;

struct my_arg_struct
{
	int *running_flag;
	ros::Publisher *pub;
};

//����һ���µ����ݽṹ�����ڱ�ʾ���� x��y �� yaw ����Ϣ
struct UplinkData {
    int32_t xCoordinate;
    int32_t yCoordinate;
    int32_t yawAngle;
    uint8_t checksum;
};

// �ṹ�����л������ã� 
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
// ���л�����
void serializeUplinkData(const UplinkData& uplinkData) {
    // �����ݽṹ���л�Ϊһ�� std::vector<uint8_t>
    std::vector<uint8_t> serializedData;
    serializedData.resize(ros::serialization::serializationLength(uplinkData));
    ros::serialization::OStream stream(serializedData.data(), serializedData.size());
    ros::serialization::serialize(stream, uplinkData);

   
}
// �� std::vector<uint8_t> ת��Ϊ std::string
std::string vectorToString(const std::vector<uint8_t>& data) {
    return std::string(data.begin(), data.end());
}
// ���� BCC У���
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

// �յ� UDP ʱ�ص���data ���յ������ݣ����ǰ���ͨ�� pub ����ȥ
void my_recv_handler(const std::string& data, void *arg_ptr)
{
	struct my_arg_struct *arg = (struct my_arg_struct*)arg_ptr;
	// �����б��� data
	// ����Э�� data �����������ÿ��Ӧ��ֻ��һ���ֽ�
	for (auto i : data) {
		std_msgs::Int8 msg;
		msg.data = i;
		arg->pub->publish(msg);
	}
}

// UDP ����������һ���߳�
void *my_udp_srv_thread(void *arg_ptr)
{
	struct my_arg_struct *arg = (struct my_arg_struct*)arg_ptr;
	my_udp_srv(MY_SERV_ADDR, MY_SERV_PORT,
		my_recv_handler, arg_ptr, SIMPLE_UDP_RECV_BUFSIZ, arg->running_flag);
	ROS_WARN("simple_udp server exited");
	return NULL;
}

//����һ���ص����������ڴ����ĵ���̼����ݣ������� x��y �� yaw��
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // ���� x��y �� yaw
    int32_t x = msg->pose.pose.position.x * 100;  // ����λת��Ϊ����
    int32_t y = msg->pose.pose.position.y * 100;  // ����λת��Ϊ����
    int32_t yaw = static_cast<int32_t>(msg->pose.pose.orientation.z * 180 / M_PI);  // ������ת��Ϊ�Ƕ�
	ros::Rate loop_rate(MY_RATE);
    // �������� x��y �� yaw ��Ϣ�Ľṹ��
    printf("data transmit start!\n");
    UplinkData uplinkData;
    uplinkData.xCoordinate = x;
    uplinkData.yCoordinate = y;
    uplinkData.yawAngle = yaw;
    //uplinkData.checksum = calculateChecksum(uplinkData);
    printf("data form odom got!\n");
    printf("x coordinate is %d\n",uplinkData.xCoordinate);
    // ʹ�����л��������ṹ��תΪvector 
    serializeUplinkData(uplinkData);
    
    // ���㲢���� BCC У���
    uint8_t checksum = calculateBCC(uplinkData.data(), uplinkData.size());
    uplinkData.push_back(checksum);
    printf("checksum is %d\n",checksum);
    //printf("%s\n",serializedData[0]);
    printf("%s\n",uplinkData.data());
	// �� std::vector<uint8_t> ת��Ϊ std::string
	std::string serializedString = vectorToString(uplinkData);
    if (!cli) {
		ROS_ERROR("create udp client failed");
		exit(EXIT_FAILURE);
	}
	// ��������
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
	// ���������λ����Ϣ��Ȼ��ص���������Ϣ����ȥ 
	ros::Subscriber odomSub = nh.subscribe("/imu_odometry/odom", 50, odomCallback);  // �滻Ϊʵ�ʵ���̼ƻ���
	// ���������߳�
	int running_flag = ~0; // running_flag ��0���÷�������������
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
