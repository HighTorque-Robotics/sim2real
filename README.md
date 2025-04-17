# README.md

# Sim2real

## 简述
机器人sim2real部署代码
## 编译流程

### 1.下载sim2real\_master

```bash 
cd ~
git clone https://gitee.com/hightorque-opensource/hightorque_rl.git
mv hightorque_rl sim2real_master

```


### 2.下载sim2real到sim2real\_master并编译。&#x20;

- 假设sim2real\_master在‘\~/’ 目录中

```bash 
cd ~/
cd sim2real_master/src
rm -r sim2real
git clone https://github.com/HighTorque-Robotics/sim2real.git
cd ..
catkin build sim2real
```


### 3.运行

- cakin build之后

```bash 
source devel/setup.bash
roslaunch sim2real sim2real.launch
```
