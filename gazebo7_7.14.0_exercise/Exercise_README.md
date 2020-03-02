# Gazebo-exercise安装&使用说明文档
2019-05-23 by airc_exercise@163.com
```
______________________________________________________________
|                 This is Gazebo-exercis-0.2                 |
|                         by AIRC-01                         |
|                                                            |
|---------------------- WorldUpdateBegin --------------------|
|   Multithreading event_signal: ON                          |
|   Multithreading event_signal: OFF                         |
|   Using OpenMP with n threads                              |
|---------------------- UpdateCollision ---------------------|
|   Using  dSweepAndPruneSpace                               |
|   Using  dHashSpace                                        |
|   Multithreading dHashSpace: ON                            |
|   Using OpenMP with n threads                              |
|   Multithreading dHashSpace: OFF                           |
|---------------------- UpdatePhysics -----------------------|
|   Multithreading dxprocessislands: ON                      |
|   Using OpenMP with n threads                              |
|   Multithreading dxprocessislands: OFF                     |
|   This code is running with NO OPTIMIZATION !              |
|____________________________________________________________|
```

## 安装第三方依赖库
由于Gazebo-exercise是基于官方版本Gazebo7_7.14.0进行开发，因此，安装过程与Gazebo7_7.14.0类似，需要首先安装一些第三方依赖库。关于依赖库的具体安装过程不在这里赘述，需要注意的是这些库的版本需要跟Gazebo的版本相匹配。
- 安装ign-math_2.8.0
- 安装protobuf-2.6.1
- 安装sdformat_4.4.0
```
	mkdir build
	cd build
	cmake ../
	make -jX
	sudo make install
```
- 安装libevent
```
	./autogen.sh
	./configure
	make -jX
	sudo make install
```
注意：若安装libevent过程中运行./autogen.sh时出现
	./autogen.sh: 18: ./autogen.sh: aclocal: not found
      错误，则运行sudo apt-get install automake解决该错误

## 安装Gazebo-exercise
```
	mkdir build
	cd build
	cmake ../
	make -jX（X为编译时启用的线程数，其根据CPU核心数确定，不要超过CPU核心数）
	sudo make install
```
## 使用Gazebo-exercise中的优化
当前版本针对Gazebo仿真流程中的预处理（WorldUpdateBegin）、碰撞更新（UpdateCollision）以及物理更新（UpdatePhysics）等模块分别进行了优化，主要采用了基于OpenMP的多线程优化，也有小部分其他优化措施。为了使用这些优化，需要在仿真的主世界文件（一般以.world结尾）中，添加如下所示的优化标签：
```
<world>
...
    <use_asyn_event>     <!--Added by zenglei for Asyn Event-->
      <use_sim_time>1</use_sim_time>
      <frequency>100</frequency>
    </use_asyn_event>
    <use_asyn_processmessages>  <!--Added by zenglei for Asyn Process Messages -->
      <flag>1</flag>
      <frequency>1000</frequency>
    </use_asyn_processmessages>
    <exercise_opt>
      <event_signal parallel_type=1 threads=2 />
	  <collide_space>1</collide_space>
      <dxhashspace_collide threads=2 />
      <updatephysics_dxprocessislands parallel_type=1 threads=2 />
    </exercise_opt>
	...
</world>
```

标签释义如下：
- <exercise_opt>标签：优化总开关。
- <event_signal>标签：用于设置模型预处理模块并行优化参数，parallel_type为1~5表示OpenMp优化（一般设置为1即可，其他是采用不同的任务并发方式），threads为开启的线程数；parallel_type为其他数字时表示不开启优化。
- <collide_space>标签：用于设置碰撞更新模块中碰撞空间的类型，值为1时表示SAP空间类型，其他数字表示默认的Hash空间类型。
- <dxhashspace_collide>标签：用于设置碰撞更新模块中dxHashSpace::collide模块并行参数，当<collide_space>标签设置为1时，该参数没有意义；其他情况下，threads大于1，表示采用OpenMP并行优化，threads为开启的线程数，否则不开启优化。
- <updatephysics_dxprocessislands>标签：用于设置物理更新模块中dxProcessIslands模块并行参数；parallel_type为1~5表示OpenMp优化（一般设置为1即可，其他是采用不同的任务并发方式），threads为开启的线程数；parallel_type为其他数字时表示不开启优化。
- <use_asyn_event>标签：用于设置异步事件机制，其元素标签示意如下。
	- <use_sim_time>： 值为1时则开启异步事件机制；
	- <frequency>： 用于控制异步事件机制频率
- <use_asyn_processmessages>： 用于设置异步化ProcessMessages模块，其元素标签示意如下。
	- <flag>： 值为1则开启异步化ProcessMessages模块，将模型~/pose/info、~/pose/local/info、~/model/info话题发布异步化，进而从仿真回路中解耦
	- <frequency>： 用来设置异步话题发布频率	

