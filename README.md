简介：本项目（Gazebo_exercise）在官方Gazebo7.14版本仿真器的基础上，采用OpenMP、线程池等技术进行了优化。
日期：2019-04-01
作者：zhangshuai，lihao，zenglei

---------------------------------------- 

本项目包含三个部分，分别是Gazebo源码，第三方依赖包以及测试案例，其中：
	gazebo7_7.14.0_exercise文件夹为优化后的Gazebo源码，里面实现了多线程优化等相关内容；
	Third-party Packages为第三方依赖包，里面包含sdformat标签功能包；
	Test Cases为测试案例文件夹，里面包含Hector四旋翼无人机进行简单飞行动作的测试案例。
	

使用说明
------------
请按照如下顺序编译安装相关软件包。

一、编译与安装sdformat
	1、cd sdformat
	2、mkdir build
	3、cd build
	4、cmake ../
	5、sudo make -jX（X为编译时启用的线程数，其根据CPU核心数确定，不要超过CPU核心数）
	6、sudo make install

二、编译与安装lievent
	1、./autogen.sh
	2、./configure
	3、make -jX
	4、sudo make install
注意：若安装libevent过程中运行./autogen.sh时出现
	./autogen.sh: 18: ./autogen.sh: aclocal: not found
      错误，则运行sudo apt-get install automake解决该错误

三、编译与安装Gazebo
	1、cd gazebo
	2、mkdir build
	3、cd build
	4、cmake ../
	5、sudo make -jX（X为编译时启用的线程数，其根据CPU核心数确定，不要超过CPU核心数）
	6、sudo make install
	
注意：利用cmake生成makefile期间，可能会出现依赖库缺失的情况，需要利用sudo apt install或源码编译安装方式进行依赖库的安装

四、编译运行Hector_Quadrotor_Case四旋翼无人机仿真案例
	1、cd Hector_Quadrotor_Case
	2、catkin_make
	3、source devel/setup.bash
	4、roslaunch hector_quadrotor_gazebo  hector_quadrotor_one_node_30.launch
	
