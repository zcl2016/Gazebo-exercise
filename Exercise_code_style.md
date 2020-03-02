简介：此文件描述了Gazebo_7.14_exercise版本开发过程中所遵循的代码添加规范
时间：2019-04-01
作者：airc_exercise@163.com

主要包括三种格式：
1.新添加文件
在文件头加上如下说明，其中Copyright部分格式固定，[Author][Date]根据具体情况填写，为必填项，[Description]为选填项。
/*
 * Copyright (C) 2019 AIRC 
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * [Author] zenglei
 * [Date] 2018-12-04
 * [Description] Head file for profiler
*/

2.在原始代码中添加一行代码
在代码后添加4个空格，然后添加注释，注释内容为“// Added by XXX@xxx-xx-xx”，其中XXX表示作者，xxx-xx-xx表示修改日期，如下面代码所示：
#include "gazebo/countTime.hh"          // Added by zenglei@2018-12-04

3.在原始代码中添加代码块（一行以上的代码）
在代码块开始和结束位置分别添加注释，注释内容分别为：
  //////////////////// AIRC begin ////////////////////
  // Added by XXX@xxx-xx-xx, Modified by YYY@yyy-yy-yy
  // [Description] zzzzzzzzzzzzzzzzzzzz
和
  //////////////////// AIRC end ////////////////////
其中[Description]根据具体情况选填。
