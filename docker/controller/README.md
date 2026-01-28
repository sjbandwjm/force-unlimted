## Controller dockerfile
### 基础镜像
- ros:jazzy-perception-noble
  - ros2
  - ubuntu 24.04
- 下载方法
~~~bash
# docker hub 源 
## 基础开发环境
docker pull ros:jazzy-ros-base-noble
## 带图像/点云感知库
docker pull ros:jazzy-perception-noble

# 国内开源代理
docker pull docker.1ms.run/ros:jazzy-ros-base-noble
docker pull docker.1ms.run/ros:jazzy-perception-noble

~~~
- 国内可用代理源
~~~bash
docker.lms.run
docker.domys.cc
docker.imgdb.de
docker-0.unsee.tech
docker.hlmirror.com
cije.eu.org
docker.m.daocloud.io
hub.rat.dev
docker.1panel.live
docker.rainbond.cc
~~~