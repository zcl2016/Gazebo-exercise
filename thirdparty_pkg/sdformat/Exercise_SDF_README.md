# Gazebo_exercise优化控制SDF标签使用说明
2019-05-20 by tjpu_zenglei@sina.com
## 优化控制SDF标签设计总体思路
为方便用户使用，集中管理自主实现的优化SDF标签，统一设计并实现了基于SDFormat包的优化控制标签。

针对目前Gazebo_exercise已经优化的模型预处理模块、UpdatePhysics模块、dxHashSpace::collide模块等分别添加标签<event_signal>（用于设置模型预处理模块并行参数）、<updatephysics_dxprocessislands>（用于设置UpdatePhysics模块并行参数）、<dxhashspace_collide>（用于设置collide模块并行参数）、<collide_space>（用于选择碰撞空间数据结构类型）。

## SDF标签使用说明
所有优化控制标签均为标签<exercise_opt>子标签，<exercise_opt>为标签<world>子标签，如下所示。
```
<world>
...
    <exercise_opt>
      <event_signal parallel_type=1 threads=7 />
	  <collide_space>1</collide_space>
      <dxhashspace_collide threads=7 />
      <updatephysics_dxprocessislands parallel_type=1 threads=4 />
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
