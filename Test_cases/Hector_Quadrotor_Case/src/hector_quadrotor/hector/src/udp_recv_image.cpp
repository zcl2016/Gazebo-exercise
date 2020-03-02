/************************************************************************	
 *  																	*
 *   	Author:	ake														*
 *    	Date:	2018-10-23												*
 *      Description:	recv image of the camera to server with udp		*
 *      																*
 *************************************************************************/
#include "udp_recv_image.h"

namespace hector
{

udp_recv_image::udp_recv_image():nh_(ros::NodeHandle()),it(nh_)
{
    image_pub = it.advertise("udp_image",1);

    memset(&my_addr, 0, sizeof(my_addr));
    my_addr.sin_family = AF_INET;
    my_addr.sin_addr.s_addr = INADDR_ANY;

    my_addr.sin_port = htons(8888);
    sin_size = sizeof(struct sockaddr_in);

    if((server_sockfd = socket(PF_INET, SOCK_DGRAM, 0)) < 0)
    {
    	perror("socket error");
	return;
    }

    if(bind(server_sockfd, (struct sockaddr *)&my_addr, sizeof(struct sockaddr)) < 0)
    {
    	perror("bind error");
	return;
    }
    struct timeval tv_out;
    tv_out.tv_sec = 3;
    tv_out.tv_usec = 0;
    setsockopt(server_sockfd, SOL_SOCKET, SO_RCVTIMEO, &tv_out,sizeof(tv_out));
    int sockbufsize = 1024*1024*10;
    setsockopt(server_sockfd, SOL_SOCKET, SO_RCVBUF, &sockbufsize, sizeof(sockbufsize));

}

udp_recv_image::~udp_recv_image()
{

}

void udp_recv_image::udp_close()
{
    close(server_sockfd);
}

void udp_recv_image::recv_image()
{
    sensor_msgs::ImagePtr msg;
    struct RecvStatus recvS;
    timespec t1,t2;
    int nCount = 0;
    bool bFlag = false;
    char bufS[20];
    // int num = 1;
    int count = 0;
    unsigned int size = 0;
    char tmpBuf[1024*1024];
    cv::Mat image_recv;
    std::vector<uchar> buf;
    std::vector<int> params;
    std::string comprType = ".jpg";
	int quality = 100;
	int compr = cv::IMWRITE_JPEG_QUALITY;
	params.push_back(compr);
	params.push_back(quality);
    clock_gettime(CLOCK_MONOTONIC, &t1);
    while(ros::ok())
    {
        // std::cout<<"-----------------------------------------pos 1------------------------------------------\n";
        memset(frameBuffer,0,BUF_SIZE);
        int len = recvfrom(server_sockfd, frameBuffer, sizeof(PackageHeader)+UDP_MAX_SIZE, 0, (struct sockaddr *)&remote_addr, &sin_size);
        // std::cout<<"-----------------------------------------pos 2------------------------------------------\n";
        // std::cout<<"len = "<<len<<std::endl;
        if(len > 0)
        {
            // std::cout<<"-----------------------------------------pos 3------------------------------------------\n";
            bFlag = true;
            packageHead = (PackageHeader*)frameBuffer;
            if(packageHead->uDataPackageCurrIndex == 1)
            {
                count = 0;
                size = 0;
                bzero(tmpBuf, packageHead->uDataSize);
                
            }
            count++;
            size += packageHead->uTransPackageSize-packageHead->uTransPackageHdrSize;
            // std::cout<<"1111111111111111111111111111111111111111111111111111\n";
            memcpy(tmpBuf+packageHead->uDataPackageOffset, frameBuffer + packageHead->uTransPackageHdrSize,packageHead->uTransPackageSize-packageHead->uTransPackageHdrSize);
            // std::cout<<"2222222222222222222222222222222222222222222222222222\n";
            if(count == packageHead->uDataPackageNum)
            {
                if(size == packageHead->uDataSize)
                {
                    buf.resize(packageHead->uDataSize);
                    buf.assign(tmpBuf, tmpBuf+packageHead->uDataSize);
                    cv::imdecode(cv::Mat(buf), cv::IMREAD_COLOR, &image_recv);
                    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_recv).toImageMsg();
                    image_pub.publish(msg);
                    nCount++;
                    // usleep(50);
                    // cv::imshow("test subscribe", cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image);
                    // cv::waitKey(30);
                    clock_gettime(CLOCK_MONOTONIC, &t2);
                    std::cout<<"time: "<<t2.tv_sec - t1.tv_sec<<"\t count = "<<nCount<<std::endl;
                }
            }
            recvS.index = packageHead->uDataPackageCurrIndex;
            recvS.flag = 1;
            bzero(bufS, 20);
            memcpy(bufS, (char*)&recvS,sizeof(recvS));
            sendto(server_sockfd,bufS, sizeof(recvS), 0, (struct sockaddr *)&remote_addr, sin_size);

        }
        else
        {
            // std::cout<<"3333333333333333333333333333333333333333333333333\n";
            // std::cout<<"-----------------------------------------pos 4------------------------------------------\n";
            if(bFlag)
            {
                recvS.index = packageHead->uDataPackageCurrIndex;
                recvS.flag = -1;
                bzero(bufS, 20);
                memcpy(bufS, (char*)&recvS,sizeof(recvS));
                sendto(server_sockfd,bufS, sizeof(recvS), 0, (struct sockaddr *)&remote_addr, sin_size);
            }
            
        }
//         if(len > 0)
//         {
// //            memcpy(&packageHead, frameBuffer, sizeof(struct PackageHeader));
//             packageHead = (PackageHeader*)frameBuffer;
//             // int *flag = new int[packageHead->uDataPackageNum];
//             // for(int j = 0; j < packageHead->uDataPackageNum; j++)
//             // {
//             //     flag[j] = 0;
//             // }
//             // std::cout<<"-------------1 start-----------------------\n";
//             // std::cout<<"packageHead.uDataPackageCurrIndex = "<<packageHead->uDataPackageCurrIndex<<std::endl;
//             // std::cout<<"-------------1 end-------------------------\n";
//             if(packageHead->uDataPackageCurrIndex == num)
//             {
//                 // std::cout<<"-------------2 start-----------------------\n";
//                 // std::cout<<"packageHead.uDataPackageCurrIndex = "<<packageHead->uDataPackageCurrIndex<<std::endl;
//                 // std::cout<<"-------------2 end-------------------------\n";
//                 // std::cout<<"-------------3 start-----------------------\n";
//                 // std::cout<<"packageHead.uDataPackageNum = "<<packageHead->uDataPackageNum<<std::endl;
//                 // std::cout<<"-------------3 end-------------------------\n";
//                 // cv::Mat image_recv;
//                 // switch(packageHead->channels)
//                 // {
//                 // case 1:
//                 //     image_recv = cv::Mat(packageHead->rows,packageHead->cols,CV_8UC1);
//                 //     break;
//                 // case 2:
//                 //     image_recv = cv::Mat(packageHead->rows, packageHead->cols,CV_8UC2);
//                 //     break;
//                 // case 3:
//                 //     image_recv = cv::Mat(packageHead->rows, packageHead->cols,CV_8UC3);
//                 //     break;
//                 // case 4:
//                 //     image_recv = cv::Mat(packageHead->rows, packageHead->cols,CV_8UC4);
//                 //     break;
//                 // }
//                 while(num <= packageHead->uDataPackageNum)
//                 {
//                     if(num == packageHead->uDataPackageCurrIndex)
//                     {
//                         size += packageHead->uTransPackageSize-packageHead->uTransPackageHdrSize;
//                         if(size > packageHead->uDataSize)
//                         {
//                             perror("error image data, too big.");
//                          size = 0;
//                             num = 1;
//                          break;
//                         }
//                         if(packageHead->uDataPackageOffset > packageHead->uDataSize)
//                         {
//                          perror("error image data, format error.");
//                          size = 0;
//                         num = 1;
//                         break;
//                           }
//                     // std::cout<<"size = "<<size<<"\tpackageHead.uDataSize= "<<packageHead->uDataSize<<std::endl;
//                     // std::cout<<"***************start*******************\n";
//                         std::cout<<"packageHead.uDataPackageCurrIndex = "<<packageHead->uDataPackageCurrIndex<<std::endl;
//                         memcpy(image_recv.data+packageHead->uDataPackageOffset, frameBuffer+packageHead->uTransPackageHdrSize,packageHead->uTransPackageSize-packageHead->uTransPackageHdrSize);
//                         // std::cout<<"***************end*******************\n";
//                     // std::cout<<"num ="<<num<<"\tpackageHead->uDataPackageNum="<<packageHead->uDataPackageNum<<"\tpackageHead->uDataPackageCurrIndex="<<packageHead->uDataPackageCurrIndex<<std::endl;
//                         recvS.index = num;
//                         recvS.flag = 1;
//                         sendto(server_sockfd,(char*)&recvS, sizeof(recvS), 0, (struct sockaddr *)&remote_addr, sin_size);
//                         if((packageHead->uDataPackageNum == packageHead->uDataPackageCurrIndex))
//                          {
//                         // std::cout<<"packageHead.uDataPackageNum= "<<packageHead->uDataPackageNum<<std::endl;
//                          if((size == packageHead->uDataSize))
//                          {
//                             msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_recv).toImageMsg();
//                             image_pub.publish(msg);
//                             usleep(50);
//                             cv::imshow("test subscribe", cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image);
//                             cv::waitKey(30);
//                          }
//                             num = 1;
//                             size = 0;
//                             break;
                        
//                         }
//                         num++;

//                     }
                    
//                     memset(frameBuffer,0,BUFSIZE);
                    
//                     len = recvfrom(server_sockfd, frameBuffer, sizeof(PackageHeader)+UDP_MAX_SIZE, 0, (struct sockaddr *)&remote_addr, &sin_size);
//                     if(len < 0 )
//                     {
                        
//                         recvS.index = num;
//                         recvS.flag = -1;
//                         sendto(server_sockfd,(char*)&recvS, sizeof(recvS), 0, (struct sockaddr *)&remote_addr, sin_size);
//                         num = 1;
//                             size = 0;
//                             break;
//                     }
                    
//                 }
//                 num = 1;
//                 size = 0;
//             }
//         }
    }
}

}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"udp_recv_image");
    hector::udp_recv_image image_recv;

    image_recv.recv_image();

    image_recv.udp_close();
    return 0;
}