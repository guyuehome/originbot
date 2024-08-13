#if defined(__linux__)

#include "serial/nvilidar_serial_unix.h"
#include <errno.h>   /* Error number definitions */
#include <fcntl.h>   /* File control definitions */
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>
#include <err.h>
#include <linux/serial.h>
#include <sys/ioctl.h> //ioctl

namespace vp100_serial
{
    // linux/include/uapi/asm-generic/termbits.h
    struct termios2 {
        tcflag_t c_iflag;       /* input mode flags */
        tcflag_t c_oflag;       /* output mode flags */
        tcflag_t c_cflag;       /* control mode flags */
        tcflag_t c_lflag;       /* local mode flags */
        cc_t c_line;            /* line discipline */
        cc_t c_cc[19];          /* control characters */
        speed_t c_ispeed;       /* input speed */
        speed_t c_ospeed;       /* output speed */
    };

    #ifndef TCGETS2
    #define TCGETS2     _IOR('T', 0x2A, struct termios2)
    #endif

    #ifndef TCSETS2
    #define TCSETS2     _IOW('T', 0x2B, struct termios2)
    #endif

    #ifndef BOTHER
    #define BOTHER      0010000
    #endif


    Nvilidar_Serial::Nvilidar_Serial()
    {

    }

    Nvilidar_Serial::~Nvilidar_Serial()
    {
        serialClose();
    }

    //初始化 
    void Nvilidar_Serial::serialInit(std::string portName,int baudRate,int parity,
                                   int dataBits,int stopbits,
                                    int flowControl,unsigned int readBufferSize)
    {
        m_portName = portName; // portName;//串口 /dev/ttySn, USB /dev/ttyUSBn
        m_baudRate = baudRate;
        m_parity = parity;
        m_dataBits = dataBits;
        m_stopbits = stopbits;
        m_flowControl = flowControl;
    }

    //设置串口参数  
    bool Nvilidar_Serial::serialSetpara(int fd,int baudRate,int parity,int dataBits,
                 int stopbits,int flowControl)
    {
        struct termios tio;

        ::memset(&tio, 0, sizeof(termios));
        if (::tcgetattr(fd, &tio) == -1) {
            return false;
        }

        SetCommonProps(&tio);
        setDataBits(fd,&tio,dataBits);
        setParity(fd,&tio,parity);
        setStopBits(fd,&tio,stopbits);
        setFlowControl(fd,&tio,flowControl);

        if (::tcsetattr(fd, TCSANOW, &tio) == -1) {
            return false;
        } 

        //set baud 
        if (!setBaudRate(baudRate)){
            return false;
        }

        return true;
    }

    //打开串口 
    bool Nvilidar_Serial::serialOpen()
    {
        bool bRet = false;

        fd = open(m_portName.c_str(), O_RDWR | O_NOCTTY | O_NDELAY); //非阻塞

        if (fd != -1)
        {
            // if(fcntl(fd,F_SETFL,FNDELAY) >= 0)//非阻塞，覆盖前面open的属性
            if (fcntl(fd, F_SETFL, 0) >= 0) // 阻塞，即使前面在open串口设备时设置的是非阻塞的，这里设为阻塞后，以此为准
            {
                // set param
                if (!serialSetpara(fd, m_baudRate, m_parity, m_dataBits, m_stopbits, m_flowControl))
                {
                    fprintf(stderr, "uart set failed!\n");
                    // exit(EXIT_FAILURE);
                    bRet = false;  
                }
            }
            else
            {
                bRet = true;
            }
        }
        else 
        {
            // Could not open the port
            char str[256];
            snprintf(str, sizeof(str), "open port error: Unable to open %s", m_portName.c_str());
            perror(str);

            bRet = false;
        }

        if (true == bRet)
        {
            serialClose();
        }     

        return bRet;
    }

    //关闭串口
    void Nvilidar_Serial::serialClose()
    {
        if (isSerialOpen())
        {
            close(fd);
            fd = -1;
        }
    }

    //查看串口打开状态  
    bool Nvilidar_Serial::isSerialOpen()
    {
        return fd != -1;
    }

    //读有效字节长度  
    int Nvilidar_Serial::serialReadAvaliable()
    {
        int iRet = -1;
        if (isSerialOpen())
        {
            // read前获取可读的字节数,不区分阻塞和非阻塞
            ioctl(fd, FIONREAD, &iRet);
        }
        else
        {
            iRet = -1;
        }
        return iRet;
    }

    //读数据  
    int  Nvilidar_Serial::serialReadData(const uint8_t *data,int len)
    {
        int iRet = -1;
        if (isSerialOpen())
        {
            iRet = read(fd, (char *)data, len);
        }
        else
        {
            iRet = -1;
        }
        return iRet;
    }

    //写数据  
    int Nvilidar_Serial::serialWriteData(const uint8_t *data, int len)
    {
        int iRet = -1;

        if (isSerialOpen())
        {
            // Write N bytes of BUF to FD.  Return the number written, or -1
            iRet = write(fd, data, len);
        }
        else
        {
            iRet = -1;
        }

        return iRet;
    }

    //serialport flush 
    void Nvilidar_Serial::serialFlush()
    {
        if (isSerialOpen())
        {
            tcflush(fd,TCIOFLUSH);
        }
    }

    //set termios 
    bool Nvilidar_Serial::setTermios(int fd,const termios *tio)
    {
        if (::tcsetattr(fd, TCSANOW, tio) == -1) {
            return false;
        }
        return true;
    }

    //set standard baudrate 
    bool Nvilidar_Serial::setStandardBaudRate(int fd,int baud_unix){

        // try to clear custom baud rate, using termios v2
        struct termios2 tio2;
        if (::ioctl(fd, TCGETS2, &tio2) != -1) {
            if (tio2.c_cflag & BOTHER) {
                tio2.c_cflag &= ~BOTHER;
                tio2.c_cflag |= CBAUD;
                ::ioctl(fd, TCSETS2, &tio2);
            }
        }

        // try to clear custom baud rate, using serial_struct (old way)
        struct serial_struct serial;
        ::memset(&serial, 0, sizeof(serial));
        if (::ioctl(fd, TIOCGSERIAL, &serial) != -1) {
            if (serial.flags & ASYNC_SPD_CUST) {
                serial.flags &= ~ASYNC_SPD_CUST;
                serial.custom_divisor = 0;
                // we don't check on errors because a driver can has not this feature
                ::ioctl(fd, TIOCSSERIAL, &serial);
            }
        }

        struct termios tio;
        ::memset(&tio, 0, sizeof(termios));
        if (::tcgetattr(fd, &tio) == -1) {
            return false;
        }
        //set standard baudrate 
        if(cfsetispeed(&tio, baud_unix) < 0){
            return false;
        }
        if(cfsetospeed(&tio, baud_unix) < 0){
            return false;
        }

        return setTermios(fd,&tio);
    }

    //set  custom baudrate 
    bool Nvilidar_Serial::setCustomBaudRate(int fd,int baud){
        struct termios2 tio2;

        if (::ioctl(fd, TCGETS2, &tio2) != -1) {
            tio2.c_cflag &= ~CBAUD;
            tio2.c_cflag |= BOTHER;

            tio2.c_ispeed = baud;
            tio2.c_ospeed = baud;

            if (::ioctl(fd, TCSETS2, &tio2) != -1
                    && ::ioctl(fd, TCGETS2, &tio2) != -1) {
                return true;
            }
        }

        struct serial_struct serial;
        if (::ioctl(fd, TIOCGSERIAL, &serial) == -1) {
            return false;
        }

        serial.flags &= ~ASYNC_SPD_MASK;
        serial.flags |= (ASYNC_SPD_CUST /* | ASYNC_LOW_LATENCY*/);
        serial.custom_divisor = serial.baud_base / baud;

        if (serial.custom_divisor == 0) {
            return false;
        }

        if (serial.custom_divisor * baud != serial.baud_base) {
            return false;
        }

        if (::ioctl(fd, TIOCSSERIAL, &serial) == -1) {
            return false;
        }

        return setStandardBaudRate(fd,B38400);
    }

    //set baudrate 
    bool Nvilidar_Serial::setBaudRate(int baudRate){
        //set baudrate 
        int baudRateConstant = 0;
        baudRateConstant = rate2UnixBaud(baudRate);
        if (0 != baudRateConstant)
        {
            if(false == setStandardBaudRate(fd,baudRateConstant)){
                return false;
            }
        }
        else
        {
            if(false == setCustomBaudRate(fd,baudRate)){
                return false;
            }
        }
        return true;
    }

    //set databits 
    void Nvilidar_Serial::setDataBits(int fd,struct termios *tio,int databits){

        tio->c_cflag &= ~CSIZE;

        switch (databits) {
        case DataBits5:
            tio->c_cflag |= CS5;
            break;
        case DataBits6:
            tio->c_cflag |= CS6;
            break;
        case DataBits7:
            tio->c_cflag |= CS7;
            break;
        case DataBits8:
            tio->c_cflag |= CS8;
            break;
        default:
            tio->c_cflag |= CS8;
            break;
        }

        setTermios(fd,tio);         //set termios 
    }

    //set parity 
    void Nvilidar_Serial::setParity(int fd,struct termios *tio,int parity)
    {
        tio->c_iflag &= ~(PARMRK | INPCK);
        tio->c_iflag |= IGNPAR;

        switch (parity) {
            #ifdef CMSPAR
                // Here Installation parity only for GNU/Linux where the macro CMSPAR.
                case ParitySpace:{
                    tio->c_cflag &= ~PARODD;
                    tio->c_cflag |= PARENB | CMSPAR;
                    break;
                }
                case ParityMark:{
                    tio->c_cflag |= PARENB | CMSPAR | PARODD;
                    break;
                }
            #endif
            case ParityNone:{
                tio->c_cflag &= ~PARENB;
                break;
            }
            case ParityEven:{
                tio->c_cflag &= ~PARODD;
                tio->c_cflag |= PARENB;
                break;
            }
            case ParityOdd:{
                tio->c_cflag |= PARENB | PARODD;
                break;
            }
            default:{
                tio->c_cflag |= PARENB;
                tio->c_iflag |= PARMRK | INPCK;
                tio->c_iflag &= ~IGNPAR;
                break;
            }
        }

        setTermios(fd,tio);         //set termios 
    }

    //set stopbits 
    void Nvilidar_Serial::setStopBits(int fd,struct termios *tio,int stopBits){
        switch (stopBits) {
        case StopOne:
            tio->c_cflag &= ~CSTOPB;
            break;
        case StopTwo:
            tio->c_cflag |= CSTOPB;
            break;
        default:
            tio->c_cflag &= ~CSTOPB;
            break;
        }

        setTermios(fd,tio);         //set termios 
    }

    //set flowcontrol 
    void Nvilidar_Serial::setFlowControl(int fd,struct termios *tio,int flowcontrol){
        switch (flowcontrol) {
            case FlowNone:
                tio->c_cflag &= ~CRTSCTS;
                tio->c_iflag &= ~(IXON | IXOFF | IXANY);
                break;
            case FlowHardware:
                tio->c_cflag |= CRTSCTS;
                tio->c_iflag &= ~(IXON | IXOFF | IXANY);
                break;
            case FlowSoftware:
                tio->c_cflag &= ~CRTSCTS;
                tio->c_iflag |= IXON | IXOFF | IXANY;
                break;
            default:
                tio->c_cflag &= ~CRTSCTS;
                tio->c_iflag &= ~(IXON | IXOFF | IXANY);
                break;
        } 

        setTermios(fd,tio);         //set termios 
    }

    //set CommonProps 
    void Nvilidar_Serial::SetCommonProps(termios *tio){
        ::cfmakeraw(tio);

        tio->c_cflag |= CLOCAL;
        tio->c_cc[VTIME] = 0;
        tio->c_cc[VMIN] = 0;

        //tio->c_cflag |= CREAD;
    }

    //baud change to standard baudrate 
    int Nvilidar_Serial::rate2UnixBaud(int baudrate)
    {
        // https://jim.sh/ftx/files/linux-custom-baudrate.c

        #define B(x) \
            case x:  \
                return B##x

        switch (baudrate)
        {
            #ifdef B50
                    B(50);
            #endif
            #ifdef B75
                    B(75);
            #endif
            #ifdef B110
                    B(110);
            #endif
            #ifdef B134
                    B(134);
            #endif
            #ifdef B150
                    B(150);
            #endif
            #ifdef B200
                    B(200);
            #endif
            #ifdef B300
                    B(300);
            #endif
            #ifdef B600
                    B(600);
            #endif
            #ifdef B1200
                    B(1200);
            #endif
            #ifdef B1800
                    B(1800);
            #endif
            #ifdef B2400
                    B(2400);
            #endif
            #ifdef B4800
                    B(4800);
            #endif
            #ifdef B9600
                    B(9600);
            #endif
            #ifdef B19200
                    B(19200);
            #endif
            #ifdef B38400
                    B(38400);
            #endif
            #ifdef B57600
                    B(57600);
            #endif
            #ifdef B115200
                    B(115200);
            #endif
            #ifdef B230400
                    B(230400);
            #endif
            #ifdef B460800
                    B(460800);
            #endif
            #ifdef B500000
                    B(500000);
            #endif
            #ifdef B576000
                    B(576000);
            #endif
            #ifdef B921600
                    B(921600);
            #endif
            #ifdef B1000000
                    B(1000000);
            #endif
            #ifdef B1152000
                    B(1152000);
            #endif
            #ifdef B1500000
                    B(1500000);
            #endif
            #ifdef B2000000
                    B(2000000);
            #endif
            #ifdef B2500000
                    B(2500000);
            #endif
            #ifdef B3000000
                    B(3000000);
            #endif
            #ifdef B3500000
                    B(3500000);
            #endif
            #ifdef B4000000
                    B(4000000);
            #endif

            default:
                return 0;
        }

        #undef B
    }
}

#endif