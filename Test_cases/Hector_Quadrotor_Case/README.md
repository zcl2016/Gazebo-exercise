简介：Hector_Quadrotor_Case - Hector四旋翼无人机gazebo仿真案例。
时间：2019-04-01
作者：airc_exercise@163.com
---------------------------------------- 

本案例是ROS+Gazebo仿真框架下的Hector四旋翼无人机简单飞行动作的仿真，案例在Hector_quadrotor和Ros_flight包的基础上进行一定修改得到。
---------------------------------------- 

安装说明：
和一般的ROS包安装是相同的，即首先进入案例目录，然后执行catkin_make即可。
1、cd Hector_Quadrotor_Case
2、source /opt/ros/kinect/setup.bash
3、catkin_make
---------------------------------------- 

使用说明:
安装完成后，在案例目录下执行source操作，然后即可运行案例。
    1、source devel/setup.bash
    2、roslaunch hector_quadrotor_gazebo  hector_quadrotor_one_node_30.launch
---------------------------------------- 

特别强调：
本案例对仿真过程中的worldUpdateBegin模块进行了多线程优化，可以采用多种多线程技术对该模块进行加速，具体的设置路径位于“Hector_Quadrotor_Case/src/hector_quadrotor/hector_quadrotor_gazebo/worlds/kunming_airport.world”文件中：
    <parallel method=1 numbers_of_thread=20 size_of_block=0 type=0 />
其中，method参数表示是否开启多线程：
    0为不开启（即串行执行），此时后面的参数自动忽略；
    1～5表示开启OpenMP多线程，不同的数字表示不同的多线程任务划分方法，一般选择1即可，numbers_of_thread表示线程数量；
    6表示开启线程池，numbers_of_thread表示线程数量；
    7表示使用C++11多线程技术，numbers_of_thread表示线程数量；
    8表示使用TBB多线程技术，此时size_of_block表示TBB分块参数，type表示分类器（0：auto_partitioner 1: affinity_partitioner 2: simple_partitioner）。
