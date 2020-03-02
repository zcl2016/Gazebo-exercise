/************************************************************************	
 *									                                    *
 * 	Author:	ake							                                *
 * 	Date:	2018-10-23						                            *
 * 	Description:	send image of the camera to server with udp	        *
 *									                                    *
 ************************************************************************/
#ifndef _UDP_SEND_IMAGE_H_
#define _UDP_SEND_IMAGE_H_

#include "packageHeader.h"



namespace hector
{

class udp_send_image
{
public:
    udp_send_image();
    ~udp_send_image();
    void imageCallback(const sensor_msgs::ImageConstPtr& tem_msg);
    void udp_close();

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub;

    struct PackageHeader packageHead;
    
    int client_sockfd;

    bool bConnected;
    struct sockaddr_in remote_addr;
    unsigned char frameBuffer[BUF_SIZE];
};

}

#endif
