# nagi-stm32f1-foc

[![License](https://img.shields.io/badge/License-GPL3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0.en.html)

[English](README.md) | [中文](README_CN.md)

# 介绍
本项目基于STM32F103C8T6，实现了BLDC电机的FOC控制。引脚定义如下：

![cube_mx](images/cube_mx.png)

电机驱动采用MS8313/DRV8313，该芯片内部有3个半桥驱动以及保护电路，可以减少对驱动设计的要求。

电流采样采用INA199A，运算放大器放大倍数是50倍，采样电阻0.02Ω。

编码器采用MT6701磁编码器，使用SPI读取角度。

PWM频率为28kHz中心对齐，电流采样在PWM中心对齐时每7个周期采样一次，ADC1和ADC2进行同步采样，即4kHz。
电机控制代码在ADC采样终端中执行。

## 开发环境

使用STM32CubeMX打开`nagi_stm32f1_foc.ioc`，导出CMake工程。再使用VSCode开发（需要安装STM32的扩展）。