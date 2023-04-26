# 代码说明

## get_positions

获得运行的全局导航地图

左键右键点击获得gps坐标
右键点击的点会被记录下来
中键点击撤销

## get_real_data

通过读取message，获取、处理并保存位姿、速度、加速度、角速度、点云、GPS等信息

可以在线读取，也可以回放bag数据读取

最新的代码可以通过ekf得到imu和GPS融合后的结果，并保存

## pm

通过位姿信息（pos.txt或者ekf.txt）得到未来位姿在当前相机图像的投影，包含了异常数据检测和滤波等功能

## ipm

通过pm和激光雷达数据得到势场图

## gene_nav

单独通过gps数据生成导航图，减少`get_real_data`的负担，增加代码实时性，生成导航图的时候需要map和navigation两张图片

## ekf_pos

imu和GPS数据融合的代码主体，使用x,y,theta,v为状态量，w(z轴角速度),a(x轴加速度)为控制量，被控对象为四轮差分的小车。融合后效果显著提升于gps。

## filters

一些对位置、角度量平滑化的滤波器尝试，主要是简单的二维kf，用来对每一个数据做平滑处理。效果一般。

## play

播放指定文件夹下的图片，并生成视频。