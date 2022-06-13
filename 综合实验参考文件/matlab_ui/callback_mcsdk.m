%真是matlab的回调函数
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
    
    
    %本次回调获取的Frame数量
    RxFrameCnt = 0;
    %本次回调获取的数据
    dataRecBuf = [];
   
    disp('串口剩余未处理字节');
    s.BytesAvailable
    
    while (s.BytesAvailable && RxFrameCnt < 100)

      rByte =  uint8(fread(s,1,'uchar'));

      if ((RxFrameState == 0) && (rByte == 170))  %开始标志位
          RxFrameState = 1;
          RxFrameNum = 1;
          RxFrameBuf(RxFrameNum) = rByte;
          continue;
      end
      
      if RxFrameState == 1  %
          RxFrameNum = RxFrameNum + 1;
          
          if RxFrameNum == 2  %第二个字符表示的是整个Frame的长度
              RxFrameSize = rByte;
              RxFrameBuf(RxFrameNum) = rByte;
              
          elseif RxFrameNum == 3  %第二个字符表示的是整个Frame header的CRC
              headerCRC = get_uint8_crc(RxFrameBuf,RxFrameNum-1);
              if (headerCRC ~= rByte || RxFrameSize > RxFrameBufMax) % 如果CRC校验不满足 or 发送数据太长了
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
    

    %本次回调，如果获取了有效的数据，则进行数据处理
    now_time = 0;
    for i = 1:RxFrameCnt
        now_time = dataRecBuf(i,1);
        time_buf = [time_buf;now_time]; 
        data_buf = [data_buf;dataRecBuf(i,2:3)];
        %% 传递函数方法
         input  = [input; dataRecBuf(i,2)];
         output = [output; dataRecBuf(i,3)];
    end

    if mod(RxFrameRgtCnt,50) == 1  || RxFrameCnt > 30
        plot(time_buf,data_buf);
        grid on
        axis([0 now_time+20 -1500 1500]);
        xlabel('time/second');
        legend('input','output');  %设定第二个数据和第三个数据的含义
        drawnow
        recv_buf_len = s.BytesAvailable;
        [now_time recv_buf_len RxFrameRgtCnt RxFrameErrCnt]
    end

end
