%这个程序，介绍了如何把连续的控制器离散化的例子
clear all
close all
clc

s = zpk('s') ;

%设置你的控制器
ks = (s+3)/(s*(0.1*s+1)*(0.5*s+1))

%设置控制器周期Ts
Ts = 0.01;

%使用c2d进行双线性变换；
kz = c2d(ks,Ts,'tustin')  

%使用tf转变为离散的传递函数
kz_tf = tf(kz) 


