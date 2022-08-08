#include <iostream>
#include <string>
#include <serial/serial.h>
#include <time.h>
#include <unistd.h>
int main()
{
    serial::Serial serial_;

    try
    {
        ///尝试初始化与开启串口
        serial_.setPort("/dev/ttyUSB0");                          //选择要开启的串口号
        serial_.setBaudrate(115200);                              //设置波特率
        serial_.open();                                           //开启串口
    }
    catch (serial::IOException &e)
    {
        printf("originbot can not open serial port,Please check the serial port cable! "); //如果开启串口失败，打印错误信息
    }

    if (serial_.isOpen())
    {
        printf("originbot serial port opened"); //串口开启成功提示
    }

    uint8_t data = 0;

    while (true)
    {
        serial_.read(&data, 1);
        //serial_.write(&data, sizeof(data)); 
        printf("%02X ", data);
        usleep(1000);
    }
    
    serial_.close();
    
    return true;

}