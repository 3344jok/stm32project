function crc = get_uint8_crc(uchar_buf, len)
    crc = uint8(0);
    for i = 1:len
        data1 = int16(crc);
        data2 = int16(uchar_buf(i));
        temp = data1 + data2;
        if temp >= 255
            temp = temp - 256;
        end
        crc = uint8(temp);
    end
end