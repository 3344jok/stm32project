% 这是一个获取主控板发送过来的扫频数据，画图，并记录数据到workspace的脚本
% 【note】这个脚本每次运行都会把之前的workspace中的数据情况，如果有数据是需要保存的请使用save函数进行保存
clear all
close all
clc

scoms = instrfindall;
if isempty(scoms) == 0
stopasync(scoms);
fclose(scoms);
delete(scoms);
clc;
end

%定义一些全局变量，这样回调函数callback_mcsdk.m就可以直接修改
global time_buf;
global data_buf;
global figure_handle;
global RxFrameNum;
global RxFrameState;
global RxFrameBuf;
global RxFrameBufMax; %一个包最多字节数
global RxFrameErrCnt; %错误的包的个数
global RxFrameRgtCnt; %正确的包的个数
global input;  %记录扫频输入的数组
global output; %记录扫频输出的数组

time_buf = [];
data_buf = [];
RxFrameNum = 0;
RxFrameState = 0;  
RxFrameBufMax = 250;
RxFrameErrCnt = 0;
RxFrameRgtCnt = 0;
RxFrameBuf = uint8(zeros(1,RxFrameBufMax)); %最大buffer数量
input = [];
output = [];

figure_handle = figure(1);
plot(time_buf,data_buf,'EraseMode','background','MarkerSize',5);
axis([0 20 -1500 1500]);
xlabel('time/second');
grid on;


%%配置串口信息
try
    s=serial('com9'); %这里需要修改为主控板连接的对应的串口号
catch
    error('cant serial');
end
set(s,'BaudRate', 115200,'DataBits',8,'StopBits',1,'Parity','none','FlowControl','none');
s.BytesAvailableFcnMode = 'byte';
s.BytesAvailableFcnCount = 160;
s.InputBufferSize = 151200;
s.BytesAvailableFcn = {@callback_mcsdk}; %设置回调函数

%%打开串口信息
fopen(s);

%%显示这句话的时候，就可以去运行主控板上的扫频辨识程序了
disp('wait for data ...')

%挂起，可以使用ctrl+c中断程序
pause; %挂起，等待通信数据带来，调用回调函数callcback


%关闭通信
scoms = instrfindall;
if isempty(scoms) == 0
stopasync(scoms);
fclose(scoms);
delete(scoms);
clc;
end









