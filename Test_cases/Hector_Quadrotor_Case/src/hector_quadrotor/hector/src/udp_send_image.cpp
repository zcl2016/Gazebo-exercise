/************************************************************************	
 *																		*
 * 	Author:	ake															*
 * 	Date:	2018-10-23													*
 * 	Description:	send image of the camera to server with udp			*
 *																		*
 ************************************************************************/
#include "udp_send_image.h"

namespace hector
{

udp_send_image::udp_send_image():
  nh_(ros::NodeHandle()),
  it(nh_)
{
   std::string server_ip, sub_topic;
   int server_port;
   ros::param::get("~server_ip",server_ip);
   ros::param::get("~server_port",server_port);
   ros::param::get("~sub_topic",sub_topic);
   std::cout<<"server_ip: "<<server_ip<<std::endl;
   std::cout<<"sub_topic: "<<sub_topic<<std::endl;
   if(server_ip.empty())
   {
	   server_ip = "192.168.1.5";
   }
   if(sub_topic.empty())
   {
	   sub_topic = "/fixedwing_0/downward_cam/camera/image";
   }
   if(server_port == 0)
   {
	   server_port = 8888;
   }
   image_sub = it.subscribe(sub_topic ,10, &udp_send_image::imageCallback,this);
   memset(&remote_addr, 0, sizeof(remote_addr));
   remote_addr.sin_family = AF_INET;
   remote_addr.sin_addr.s_addr = inet_addr(server_ip.c_str());
   remote_addr.sin_port = htons(server_port);
   
   if((client_sockfd = socket(PF_INET, SOCK_DGRAM, 0)) < 0)
   {
   	perror("socket error");
   }
   struct timeval tv_out;
   tv_out.tv_sec = 3;
   tv_out.tv_usec = 0;
   setsockopt(client_sockfd, SOL_SOCKET, SO_RCVTIMEO, &tv_out,sizeof(tv_out));
   int sockbufsize = 1024*1024*10;
   setsockopt(client_sockfd, SOL_SOCKET, SO_RCVBUF, &sockbufsize, sizeof(sockbufsize));


}
udp_send_image::~udp_send_image()
{
   
}

void udp_send_image::udp_close()
{
	close(client_sockfd);
}



void udp_send_image::imageCallback(const sensor_msgs::ImageConstPtr& tem_msg)
{
//     try
//     {
// //	cv::imshow("test subscribe", cv_bridge::toCvShare(tem_msg,"mono8")->image);
// 	cv::imshow("test subscribe", cv_bridge::toCvCopy(tem_msg, sensor_msgs::image_encodings::BGR8)->image);
// 	cv::waitKey(30);
//     }	
//     catch(cv_bridge::Exception& e)
//     {
// 	ROS_ERROR("Could not convert from '%s' to 'BGR8'", tem_msg->encoding.c_str());
//     }
	struct RecvStatus recvS;
	unsigned int dstLength;
	int currentPacketIndex = 0;
	int nLength = 0;
	int dataLength = 0;
	int packetNum = 0;
	int lastPaketSize = 0;
	cv::Mat img_send = cv_bridge::toCvCopy(tem_msg, sensor_msgs::image_encodings::BGR8)->image;
	//compress the img
	std::vector<uchar> buf;
	std::vector<int> params;
	std::string comprType = ".jpg";
	int quality = 100;
	int compr = cv::IMWRITE_JPEG_QUALITY;
	params.push_back(compr);
	params.push_back(quality);
	cv::imencode(comprType.c_str(), img_send,buf,params);
	// std::cout<<"buf.size() = "<<buf.size()<<std::endl;
	// cv::Mat compressed;
	// cv::imdecode(cv::Mat(buf), cv::IMREAD_COLOR,&compressed);
	// cv::imshow("test compressed", compressed);
    // cv::waitKey(30);
	unsigned char* tmpBuf = new unsigned char[buf.size()];
	for(int i = 0; i < buf.size(); i++)
	{
		tmpBuf[i] = buf[i];
	}	

	dataLength = buf.size();
	packetNum = dataLength/UDP_MAX_SIZE;
	lastPaketSize = dataLength % UDP_MAX_SIZE;
	if(lastPaketSize != 0)
	{
		packetNum = packetNum+1;
	}
	packageHead.uTransPackageHdrSize = sizeof(packageHead);
	packageHead.uDataSize = dataLength;
	packageHead.uDataPackageNum = packetNum;
	bzero(frameBuffer,BUF_SIZE);
	while(currentPacketIndex < packetNum)
	{
		if(currentPacketIndex < packetNum - 1)
		{
			packageHead.uTransPackageSize = sizeof(PackageHeader) + UDP_MAX_SIZE;
			packageHead.uDataPackageCurrIndex = currentPacketIndex + 1;
			packageHead.uDataPackageOffset = currentPacketIndex * UDP_MAX_SIZE;
			memcpy(frameBuffer, &packageHead, sizeof(PackageHeader));
			memcpy(frameBuffer+sizeof(PackageHeader), tmpBuf+packageHead.uDataPackageOffset, UDP_MAX_SIZE);

			nLength = sendto(client_sockfd, (const char*)frameBuffer, packageHead.uTransPackageSize, 0, (struct sockaddr *)&remote_addr, sizeof(struct sockaddr));
			// std::cout<<"nLength = "<<nLength<<std::endl;
			usleep(50);
			// if(nLength != packageHead.uTransPackageSize)
			// {
			// 	printf("Send image failed, try again.");
			// 	nLength = sendto(client_sockfd, (const char*)frameBuffer, packageHead.uTransPackageSize, 0, (struct sockaddr *)&remote_addr, sizeof(struct sockaddr));
			// 	usleep(200);
			// 	if(nLength != packageHead.uTransPackageSize)
			// 	{
			// 		perror("Send image failed!");
			// 	}
			// }
			// nLength = recvfrom(client_sockfd, (char*)&recvS,sizeof(recvS), 0,  (struct sockaddr *)&remote_addr, &dstLength);
			// if(recvS.index == currentPacketIndex++)
			// {
			// 	if(recvS.flag == 1)
			// 	{
			// 		currentPacketIndex++;
			// 	}
			// 	else if(recvS.flag == -1)
			// 	{
			// 		currentPacketIndex = 0;
			// 	}

			// }
			
		}
		else
		{
			packageHead.uTransPackageSize = sizeof(PackageHeader) + (dataLength - currentPacketIndex*UDP_MAX_SIZE);
			packageHead.uDataPackageCurrIndex = currentPacketIndex + 1;
			packageHead.uDataPackageOffset = currentPacketIndex * UDP_MAX_SIZE;
			memcpy(frameBuffer, &packageHead, sizeof(PackageHeader));
			memcpy(frameBuffer+sizeof(PackageHeader),tmpBuf+packageHead.uDataPackageOffset, dataLength - currentPacketIndex*UDP_MAX_SIZE);
			// std::cout<<"packageHead.uTransPackageSize = " <<packageHead.uTransPackageSize<<std::endl;
			int nLength = sendto(client_sockfd, (const char*)frameBuffer, packageHead.uTransPackageSize, 0, (struct sockaddr *)&remote_addr, sizeof(struct sockaddr));
			// std::cout<<"nLength = "<<nLength<<std::endl;
			usleep(50);
			// if(nLength != packageHead.uTransPackageSize)
			// {
			// 	printf("Send image failed, try again.");
			// 	nLength = sendto(client_sockfd, (const char*)frameBuffer, packageHead.uTransPackageSize, 0, (struct sockaddr *)&remote_addr, sizeof(struct sockaddr));
			// 	usleep(200);
			// 	if(nLength != packageHead.uTransPackageSize)
			// 	{
			// 		perror("Send image failed!");
			// 	}
			// }

		}
		// currentPacketIndex++;
		nLength = recvfrom(client_sockfd, (char*)&recvS,sizeof(recvS), 0,  (struct sockaddr *)&remote_addr, &dstLength);
		// std::cout<<"currentPacketIndex = "<<currentPacketIndex<<std::endl;
		if(nLength <= 0)
		{
			break;
		}
		if(recvS.index == currentPacketIndex+1)
		{
			if(recvS.flag == 1)
			{
				currentPacketIndex++;
			}
			else if(recvS.flag == -1)
			{
				currentPacketIndex = 0;
				break;
			}

		}		

	}
	delete []tmpBuf;
	// int dataLength = img_send.dataend - img_send.datastart;
	// // std::cout<<"cols = "<<img_send.cols<<"\trows = "<<img_send.rows<<"\tchannels = "<<img_send.channels()<<std::endl;
	// unsigned char *dataBuffer = (unsigned char*)img_send.data;
	// int packetNum = 0;
	// int lastPaketSize = 0;
	// int nLength;
	// packetNum = dataLength/UDP_MAX_SIZE;
	// lastPaketSize = dataLength % UDP_MAX_SIZE;
	// int currentPacketIndex = 0;
    // if(lastPaketSize != 0)
	// {
	// 	packageHead.uLastPaketSize = lastPaketSize;
	// 	packetNum = packetNum + 1;
	// }
	// else
	// {
	// 	packageHead.uLastPaketSize = UDP_MAX_SIZE;
	// }
	// packageHead.uTransPackageHdrSize = sizeof(packageHead);
	// packageHead.uDataSize = dataLength;
	// packageHead.uDataPackageNum = packetNum;
	// packageHead.cols = img_send.cols;
	// packageHead.rows = img_send.rows;
	// packageHead.channels = img_send.channels();
	// memset(frameBuffer, 0, BUFSIZE);
	// while(currentPacketIndex < packetNum)
	// {
	// 	if(currentPacketIndex < packetNum - 1)
	// 	{
	// 		packageHead.uTransPackageSize = sizeof(PackageHeader) + UDP_MAX_SIZE;
	// 		packageHead.uDataPackageCurrIndex = currentPacketIndex + 1;
	// 		packageHead.uDataPackageOffset = currentPacketIndex * UDP_MAX_SIZE;
	// 		memcpy(frameBuffer, &packageHead, sizeof(PackageHeader));
	// 		memcpy(frameBuffer+sizeof(PackageHeader), dataBuffer+packageHead.uDataPackageOffset, UDP_MAX_SIZE);

	// 		nLength = sendto(client_sockfd, (const char*)frameBuffer, packageHead.uTransPackageSize, 0, (struct sockaddr *)&remote_addr, sizeof(struct sockaddr));
	// 		usleep(50);
	// 		// if(nLength != packageHead.uTransPackageSize)
	// 		// {
	// 		// 	printf("Send image failed, try again.");
	// 		// 	nLength = sendto(client_sockfd, (const char*)frameBuffer, packageHead.uTransPackageSize, 0, (struct sockaddr *)&remote_addr, sizeof(struct sockaddr));
	// 		// 	usleep(200);
	// 		// 	if(nLength != packageHead.uTransPackageSize)
	// 		// 	{
	// 		// 		perror("Send image failed!");
	// 		// 	}
	// 		// }
	// 		// nLength = recvfrom(client_sockfd, (char*)&recvS,sizeof(recvS), 0,  (struct sockaddr *)&remote_addr, &dstLength);
	// 		// if(recvS.index == currentPacketIndex++)
	// 		// {
	// 		// 	if(recvS.flag == 1)
	// 		// 	{
	// 		// 		currentPacketIndex++;
	// 		// 	}
	// 		// 	else if(recvS.flag == -1)
	// 		// 	{
	// 		// 		currentPacketIndex = 0;
	// 		// 	}

	// 		// }
			
	// 	}
	// 	else
	// 	{
	// 		packageHead.uTransPackageSize = sizeof(PackageHeader) + (dataLength - currentPacketIndex*UDP_MAX_SIZE);
	// 		packageHead.uDataPackageCurrIndex = currentPacketIndex + 1;
	// 		packageHead.uDataPackageOffset = currentPacketIndex * UDP_MAX_SIZE;
	// 		memcpy(frameBuffer, &packageHead, sizeof(PackageHeader));
	// 		memcpy(frameBuffer+sizeof(PackageHeader),dataBuffer+packageHead.uDataPackageOffset, dataLength - currentPacketIndex*UDP_MAX_SIZE);

	// 		int nLength = sendto(client_sockfd, (const char*)frameBuffer, packageHead.uTransPackageSize, 0, (struct sockaddr *)&remote_addr, sizeof(struct sockaddr));
	// 		usleep(50);
	// 		// if(nLength != packageHead.uTransPackageSize)
	// 		// {
	// 		// 	printf("Send image failed, try again.");
	// 		// 	nLength = sendto(client_sockfd, (const char*)frameBuffer, packageHead.uTransPackageSize, 0, (struct sockaddr *)&remote_addr, sizeof(struct sockaddr));
	// 		// 	usleep(200);
	// 		// 	if(nLength != packageHead.uTransPackageSize)
	// 		// 	{
	// 		// 		perror("Send image failed!");
	// 		// 	}
	// 		// }

	// 	}
	// 	nLength = recvfrom(client_sockfd, (char*)&recvS,sizeof(recvS), 0,  (struct sockaddr *)&remote_addr, &dstLength);
	// 	if(recvS.index == currentPacketIndex+1)
	// 	{
	// 		if(recvS.flag == 1)
	// 		{
	// 			currentPacketIndex++;
	// 		}
	// 		else if(recvS.flag == -1)
	// 		{
	// 			currentPacketIndex = 0;
	// 			break;
	// 		}

	// 	}

	// }
	// std::cout<<"packetNum = " << packetNum <<std::endl;
	// std::cout<<"currentPacketIndex="<<currentPacketIndex<<std::endl;

}

} //end namespace
// void Stop(int signo)
// {
// 	exit(1);
// }
int main(int argc,char **argv)
{
	// signal(SIGINT, Stop);
    ros::init(argc,argv,"udp_send_image");
    hector::udp_send_image image_send;

    ros::spin();
	/*关闭套接字*/
	image_send.udp_close();
    return 0;
}

