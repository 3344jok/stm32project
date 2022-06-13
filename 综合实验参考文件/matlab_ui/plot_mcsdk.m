% ����һ����ȡ���ذ巢�͹�����ɨƵ���ݣ���ͼ������¼���ݵ�workspace�Ľű�
% ��note������ű�ÿ�����ж����֮ǰ��workspace�е�����������������������Ҫ�������ʹ��save�������б���
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

%����һЩȫ�ֱ����������ص�����callback_mcsdk.m�Ϳ���ֱ���޸�
global time_buf;
global data_buf;
global figure_handle;
global RxFrameNum;
global RxFrameState;
global RxFrameBuf;
global RxFrameBufMax; %һ��������ֽ���
global RxFrameErrCnt; %����İ��ĸ���
global RxFrameRgtCnt; %��ȷ�İ��ĸ���
global input;  %��¼ɨƵ���������
global output; %��¼ɨƵ���������

time_buf = [];
data_buf = [];
RxFrameNum = 0;
RxFrameState = 0;  
RxFrameBufMax = 250;
RxFrameErrCnt = 0;
RxFrameRgtCnt = 0;
RxFrameBuf = uint8(zeros(1,RxFrameBufMax)); %���buffer����
input = [];
output = [];

figure_handle = figure(1);
plot(time_buf,data_buf,'EraseMode','background','MarkerSize',5);
axis([0 20 -1500 1500]);
xlabel('time/second');
grid on;


%%���ô�����Ϣ
try
    s=serial('com9'); %������Ҫ�޸�Ϊ���ذ����ӵĶ�Ӧ�Ĵ��ں�
catch
    error('cant serial');
end
set(s,'BaudRate', 115200,'DataBits',8,'StopBits',1,'Parity','none','FlowControl','none');
s.BytesAvailableFcnMode = 'byte';
s.BytesAvailableFcnCount = 160;
s.InputBufferSize = 151200;
s.BytesAvailableFcn = {@callback_mcsdk}; %���ûص�����

%%�򿪴�����Ϣ
fopen(s);

%%��ʾ��仰��ʱ�򣬾Ϳ���ȥ�������ذ��ϵ�ɨƵ��ʶ������
disp('wait for data ...')

%���𣬿���ʹ��ctrl+c�жϳ���
pause; %���𣬵ȴ�ͨ�����ݴ��������ûص�����callcback


%�ر�ͨ��
scoms = instrfindall;
if isempty(scoms) == 0
stopasync(scoms);
fclose(scoms);
delete(scoms);
clc;
end









