%����matlab�Ļص�����
function callback_mcsdk(s, BytesAvailable)
   
    global time_buf;
    global data_buf;
    global figure_handle;
    
    global RxFrameNum;
    global RxFrameState;
    global RxFrameSize;
    global RxHeadCRC;
    global RxFrameBuf;
    global RxFrameBufMax;
    global RxFrameErrCnt;
    global RxFrameRgtCnt;
    global input;
    global output;
    
    
    %���λص���ȡ��Frame����
    RxFrameCnt = 0;
    %���λص���ȡ������
    dataRecBuf = [];
   
    disp('����ʣ��δ�����ֽ�');
    s.BytesAvailable
    
    while (s.BytesAvailable && RxFrameCnt < 100)

      rByte =  uint8(fread(s,1,'uchar'));

      if ((RxFrameState == 0) && (rByte == 170))  %��ʼ��־λ
          RxFrameState = 1;
          RxFrameNum = 1;
          RxFrameBuf(RxFrameNum) = rByte;
          continue;
      end
      
      if RxFrameState == 1  %
          RxFrameNum = RxFrameNum + 1;
          
          if RxFrameNum == 2  %�ڶ����ַ���ʾ��������Frame�ĳ���
              RxFrameSize = rByte;
              RxFrameBuf(RxFrameNum) = rByte;
              
          elseif RxFrameNum == 3  %�ڶ����ַ���ʾ��������Frame header��CRC
              headerCRC = get_uint8_crc(RxFrameBuf,RxFrameNum-1);
              if (headerCRC ~= rByte || RxFrameSize > RxFrameBufMax) % ���CRCУ�鲻���� or ��������̫����
                  RxFrameState = 0;
                  RxFrameNum = 0;
              else
                  RxFrameBuf(RxFrameNum) = rByte;
              end
      
          elseif RxFrameNum < RxFrameSize
              RxFrameBuf(RxFrameNum) = rByte;
              
          else
             frameCRC = get_uint8_crc(RxFrameBuf,RxFrameNum-1);
             if (frameCRC ~= rByte)
                  %format short
                  %[RxFrameBuf(1) RxFrameBuf(2) RxFrameBuf(3) RxFrameBuf(4) RxFrameBuf(5) RxFrameBuf(6) RxFrameBuf(7) RxFrameBuf(8) RxFrameBuf(9) RxFrameBuf(10) RxFrameBuf(11) RxFrameBuf(12) RxFrameBuf(13) RxFrameBuf(14) RxFrameBuf(15) rByte]
                  RxFrameState = 0;
                  RxFrameNum = 0;
                  RxFrameErrCnt = RxFrameErrCnt + 1;
                  
             else
                  RxFrameCnt = RxFrameCnt + 1;
                  RxFrameRgtCnt = RxFrameRgtCnt + 1;
                  dataRecBuf(RxFrameCnt,1) = typecast(RxFrameBuf(4:7),'single');
                  dataRecBuf(RxFrameCnt,2) = typecast(RxFrameBuf(8:11),'single');
                  dataRecBuf(RxFrameCnt,3) = typecast(RxFrameBuf(12:15),'single');
                  RxFrameState = 0;
             end
                 
          end
          
      end   
        
    end 
    

    %���λص��������ȡ����Ч�����ݣ���������ݴ���
    now_time = 0;
    for i = 1:RxFrameCnt
        now_time = dataRecBuf(i,1);
        time_buf = [time_buf;now_time]; 
        data_buf = [data_buf;dataRecBuf(i,2:3)];
        %% ���ݺ�������
         input  = [input; dataRecBuf(i,2)];
         output = [output; dataRecBuf(i,3)];
    end

    if mod(RxFrameRgtCnt,50) == 1  || RxFrameCnt > 30
        plot(time_buf,data_buf);
        grid on
        axis([0 now_time+20 -1500 1500]);
        xlabel('time/second');
        legend('input','output');  %�趨�ڶ������ݺ͵��������ݵĺ���
        drawnow
        recv_buf_len = s.BytesAvailable;
        [now_time recv_buf_len RxFrameRgtCnt RxFrameErrCnt]
    end

end
