/************************************************************************	
 *									                                    *
 * 	Author:	ake							                                *
 * 	Date:	2018-10-23						                            *
 * 	Description:	recv image of the camera to server with udp	        *
 *									                                    *
 ************************************************************************/
#ifndef _UDP_RECV_IMAGE_H_
#define _UDP_RECV_IMAGE_H_

#include "packageHeader.h"



namespace rosplane
{

class udp_recv_image
{
public:
    udp_recv_image();
    ~udp_recv_image();
    void recv_image();
    void udp_close();

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it;
    image_transport::Publisher image_pub;

    struct PackageHeader* packageHead;
    
    int server_sockfd;
    struct sockaddr_in my_addr; 
    unsigned int sin_size;
    struct sockaddr_in remote_addr;
    unsigned char frameBuffer[BUF_SIZE];
};

}

#endif
