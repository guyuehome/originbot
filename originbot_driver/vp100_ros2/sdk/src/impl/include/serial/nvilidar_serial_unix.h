#if defined(__linux__)

#ifndef _NVILIDAR_SERIAL_UNIX
#define _NVILIDAR_SERIAL_UNIX

#include <string>
#include <termios.h> /* POSIX terminal control definitions */

//串口信息宏定义 
//校验 
#define ParityNone  0  ///< No Parity 
#define ParityOdd   1   ///< Odd Parity 
#define ParityEven  2  ///< Even Parity
#define ParityMark  3  ///< Mark Parity 
#define ParitySpace  4 ///< Space Parity 

//数据位
#define DataBits5  5 ///< 5 data bits 
#define DataBits6  6 ///< 6 data bits 
#define DataBits7  7 ///< 7 data bits 
#define DataBits8  8  ///< 8 data bits 

//停止位
#define	StopOne  0        ///< 1 stop bit 
#define	StopOneAndHalf 1 ///< 1.5 stop bit  - This is only for the Windows platform
#define	StopTwo   2         ///< 2 stop bit 

//流控
#define FlowNone  0    ///< No flow control 
#define FlowHardware  1 ///< Hardware(RTS / CTS) flow control 
#define FlowSoftware  2  ///< Software(XON / XOFF) flow control 

namespace vp100_serial
{
    class Nvilidar_Serial
    {
    public:
        Nvilidar_Serial();
        ~Nvilidar_Serial();
        void serialInit(std::string portName,
                      int baudRate = 512000,
                      int parity = ParityNone,
                      int dataBits = DataBits8,
                      int stopbits = StopOne,
                      int flowControl = FlowNone,
                      unsigned int readBufferSize = 512);

        bool serialOpen();
        void serialClose();
        bool isSerialOpen();        //is serialport open? 
        int  serialReadAvaliable(); //byte to read  
        int  serialReadData(const uint8_t *data,int len);
        int  serialWriteData(const uint8_t *data,int len);        //write data to serialport 
        void serialFlush();         //flush serialport data  
    private:
        bool setTermios(int fd,const termios *tio);
        bool serialSetpara(int fd,
                 int baudRate = 512000,
                 int parity = ParityNone,
                 int dataBits = DataBits8,
                 int stopbits = StopOne,
                 int flowControl = FlowNone);
        int rate2UnixBaud(int baudrate); 
        bool setBaudRate(int baudRate);
        void SetCommonProps(termios *tio);
        bool setStandardBaudRate(int fd,int baud_unix);       //set standard baudrate 
        bool setCustomBaudRate(int fd,int baud);         //set custom baudrate 
        void setDataBits(int fd,struct termios *tio,int databits);              //set serialport databits 
        void setParity(int fd,struct termios *tio,int parity);  //set parity 
        void setStopBits(int fd,struct termios *tio,int stopBits);  //set stopbits 
        void setFlowControl(int fd,struct termios *tio,int flowcontrol);

        int fd = -1; /* File descriptor for the port */  
        bool serial_open_flag = false;   

        std::string m_portName;
        int m_baudRate;
        int m_parity;
        int m_dataBits;
        int m_stopbits;
        int m_flowControl;

    };
}

#endif

#endif