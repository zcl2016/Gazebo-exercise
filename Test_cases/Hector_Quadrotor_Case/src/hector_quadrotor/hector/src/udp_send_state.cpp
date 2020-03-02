/*******************************
 *
 *  This UDP Client is finished by YAN JIE. .
 * 
 **************************/
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "rosplane_msgs/Udp_Send_State.h"

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

//串口相关的头文件
#include <stdio.h>  /*标准输入输出定义*/
#include <stdlib.h> /*标准函数库定义*/
#include <unistd.h> /*Unix 标准函数定义*/
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>   /*文件控制定义*/
#include <termios.h> /*PPSIX 终端控制定义*/
#include <errno.h>   /*错误号定义*/
#include <string.h>
#include <math.h>
#include <boost/thread/thread.hpp>

//宏定义
#define FALSE -1
#define TRUE 0
#define TAIL 0xFFFA55AF
#define REC_BUFFER_SIZE 200
#define DOWN_CHECK_LEN 24
#define UP_CHECK_LEN 68

//#define BUFSIZ 200

// typedef struct
// {
// 	double plane_info_double[6];   //存放经度、纬度、高度、滚转角、俯仰角、偏航角
// 	int plane_info_int;			   //存放无人机编号
// 	float plane_info_float[9];     //航迹速度（目前没有用）
// 	unsigned char sum_check[4];	   //累加校验码（目前没有用）
// 	unsigned char header[4];	   //（目前没有用）
// } Scom_up_protocal;

typedef struct
{
	double plane_info_double[2];  //存放无人机经度、纬度、高度、滚转角、俯仰角、偏航角
	double camera_info_double[2]; //存放相机经度、纬度
	float plane_info_float[7];	//存放无人机高度、滚转角、俯仰角、偏航角、北东地三方向速度
	float camera_info_float[4];   //存放相机高度、滚转角、俯仰角、偏航角
	unsigned int ID;			  //存放无人机编号
	char name[20];
} Scom_up_protocal;

// struct scom_protocal
// {
// 	int plane_info_int[3];  //存放经度、纬度、高度
// 	float plane_info_float; //航迹速度
// };

namespace hector
{

using std::string;

/** @brief async write buffer */
class Udp_commu_send
{
  public:
	Udp_commu_send()
	{
		std::string client_ip;
		int client_port = 0;
		ros::param::get("~client_ip", client_ip);
		ros::param::get("~client_port", client_port);
		if(client_ip.empty())
		{
			client_ip = "192.168.1.15";
		}
		if(client_port == 0)
		{
			client_port = 8000;
		}
		ROS_ERROR("port is: %d\n", client_port);
		memset(&remote_addr, 0, sizeof(remote_addr));			 //数据初始化--清零
		remote_addr.sin_family = AF_INET;						 //设置为IP通信
		remote_addr.sin_addr.s_addr = inet_addr(client_ip.c_str()); //服务器IP地址"172.18.0.3"
		remote_addr.sin_port = htons(client_port);						 //服务器端口号
        // remote_addr.sin_port = htons(8000);		

		/*创建客户端套接字--IPv4协议，面向无连接通信，UDP协议*/
		if ((client_sockfd = socket(PF_INET, SOCK_DGRAM, 0)) < 0)
		{
			perror("socket error");
		}
		sin_size = sizeof(struct sockaddr_in);

		task_send_data = (Scom_up_protocal *)malloc(sizeof(Scom_up_protocal));
		// sub = n.subscribe("/fixedwing_0/udp_send_state", 1, &Udp_commu_send::upsendCallback, this);
        sub = n.subscribe("udp_send_state", 10, &Udp_commu_send::upsendCallback, this);
		//write_thread_ = new boost::thread(boost::bind(&Serial_commu_send::write_handle, this));
		//write_thread_ -> detach();
	};
	~Udp_commu_send()
	{
		free(task_send_data);
		delete write_thread_;
	};

	void write_handle()
	{
		unsigned char temp = num;
		sleep(0.1);
		if ((num == temp) && flag == 1)
			tcflush(fd, TCOFLUSH);
	};

	void upsendCallback(const rosplane_msgs::Udp_Send_State::ConstPtr &msg)
	{
		// printf("*************************The data to send is:\n");
		// ROS_INFO("I heard: msg.status_word is: %d\n", msg->status_word);
		// ROS_INFO("I heard: msg.longitude is %f\n", msg->longitude);
		// ROS_INFO("I heard: msg.latitude is %f\n", msg->latitude);
		// ROS_INFO("I heard: msg.altitude is: %f\n", msg->altitude);
		// ROS_INFO("I heard: msg.roll_angle is: %f\n", msg->roll_angle);
		double i = 50;

		//***************串口发送数据 *************
		//      unsigned char scom_protocal::serial_num_increase = 0;

		// task_send_data->plane_info_int[0] = msg->longitude * (int)(pow(10, 7));
		// task_send_data->plane_info_int[1] = msg->latitude * (int)(pow(10, 7));
		// task_send_data->plane_info_int[2] = msg->altitude * (int)(pow(10, 2));

		// task_send_data->plane_info_int = msg->status_word;
		// task_send_data->plane_info_int = 0;

		// task_send_data->plane_info_double[0] = msg->longitude;
		// task_send_data->plane_info_double[1] = msg->latitude;
		// task_send_data->plane_info_double[2] = msg->altitude;
		// task_send_data->plane_info_double[3] = msg->roll_angle;
		// task_send_data->plane_info_double[4] = msg->pitch_angle;
		// task_send_data->plane_info_double[5] = msg->yaw_angle;
		// task_send_data->plane_info_float[0] = msg->angular_rate_x;
		// task_send_data->plane_info_float[1] = msg->angular_rate_y;
		// task_send_data->plane_info_float[2] = msg->angular_rate_z;
		// task_send_data->plane_info_float[3] = msg->north_direction_speed;
		// task_send_data->plane_info_float[4] = msg->east_direction_speed;
		// task_send_data->plane_info_float[5] = msg->ground_direction_speed;
		// task_send_data->plane_info_float[6] = msg->acceleration_x;
		// task_send_data->plane_info_float[7] = msg->acceleration_y;
		// task_send_data->plane_info_float[8] = msg->acceleration_z;

		task_send_data->ID = 0;
		task_send_data->plane_info_double[0] = msg->longitude;
		task_send_data->plane_info_double[1] = msg->latitude;
		task_send_data->plane_info_float[0] = msg->altitude;
		task_send_data->plane_info_float[1] = msg->roll_angle * 180.0 / M_PI;
		task_send_data->plane_info_float[2] = msg->pitch_angle * 180.0 / M_PI;
		task_send_data->plane_info_float[3] = msg->yaw_angle * 180.0 / M_PI;
		task_send_data->plane_info_float[4] = msg->north_direction_speed;
		task_send_data->plane_info_float[5] = msg->east_direction_speed;
		task_send_data->plane_info_float[6] = msg->ground_direction_speed;
		task_send_data->camera_info_double[0] = msg->longitude_camera;
		task_send_data->camera_info_double[1] = msg->latitude_camera;
		task_send_data->camera_info_float[0] = msg->altitude_camera;
		task_send_data->camera_info_float[1] = msg->roll_angle_camera * 180.0 / M_PI;
		task_send_data->camera_info_float[2] = msg->pitch_angle_camera * 180.0 / M_PI;
		task_send_data->camera_info_float[3] = msg->yaw_angle_camera * 180.0 / M_PI;
		// task_send_data->name = msg->name;
		// strcpy(task_send_data->name, msg->name);
		// strcpy(task_send_data->name, "fixedwing_0");
		// 向char[20]赋值
		int j;
		for (j = 0; j < msg->name.length(); j++)
		{
			task_send_data->name[j] = msg->name[j];
		}
		task_send_data->name[j] = '\0';
		// printf("%s\n", p);
		// cout << p;

		// printf("UDP_send..........................................................\n");
		/*向服务器发送数据包*/
		int len = sendto(client_sockfd, (char *)(&task_send_data->plane_info_double[0]), sizeof(Scom_up_protocal), 0, (struct sockaddr *)&remote_addr, sizeof(struct sockaddr));
		sleep(0.001);

		//	int len = UART0_Send((char *)(&task_send_data->header[0]), sizeof(scom_protocal));
		// printf("UDP_send over..........................................................\n");

		// if (len > 0)
		// 	printf(" %f ms time send %d byte data send successfully\n", i, len);
		// else
		// 	printf("send data failed!\n");
	};

	int get_fd()
	{
		return fd;
	};
	void udp_Close()
	{
		close(client_sockfd);
	}

  private:
	Scom_up_protocal *task_send_data;
	int fd;
	unsigned char num;
	int flag;

	boost::thread *write_thread_;

	ros::NodeHandle n;
	ros::Subscriber sub;

	int client_sockfd;

	struct sockaddr_in remote_addr; //服务器端网络地址结构体
	int sin_size;
	char buf[BUFSIZ]; //数据传送的缓冲区
};

} // namespace hector

int main(int argc, char **argv)
{

	ros::init(argc, argv, "udp_send_0");

	// int err;

	hector::Udp_commu_send send;

	ros::spin();
	/*关闭套接字*/
	send.udp_Close();
	return 0;
}
