## Gazebo 7.14.0

## Gazebo-exercise-0.3 (2019-10-21)

1. 增加异步事件机制，并向外提供异步事件机制接口event::EventAsyns::ConnectWorldUpdateBegin与event::EventAsyns::DisconnectWorldUpdateBegin
    * [SDF控制标签] use_asyn_event
2. 异步化ProcessMessages模块，将将模型~/pose/info、~/pose/local/info、~/model/info话题发布从仿真回路中解耦
    * [SDF控制标签] use_asyn_processmessages

## Gazebo-exercise-0.2 (2019-05-23)

1. 添加优化统一SDF标签exercise-opt，原标签parallel失效

1. 添加dxProcessIslands模块OpenMP并行优化
    * [SDF控制标签] updatephysics_dxprocessislands

1. 添加dxSpace构建类型选择
    * [SDF控制标签] collide_space

1. 添加默认dxSpace类型（dxHashSpace）碰撞处理模块OpenMp并行优化
    * [SDF控制标签] dxhashspace_collide

1. 去掉事件处理模块(worldUpdateBegin)中的线程池等并行优化措施，只保留OpenMP 
    * [相关文件]/gazebo/common/Event.hh
    * [SDF控制标签] event_signal

## Gazebo-exercise-0.1 (2019-04-01)

1. 添加事件处理模块（worldUpdateBegin）中OpenMp、Tbb、线程池、C++11Thread等并行优化措施
    * [相关文件]/gazebo/common/Event.hh

1. 添加并行处理优化控制SDF标签parallel
