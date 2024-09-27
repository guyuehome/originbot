#if defined(_WIN32)

#include "serial/nvilidar_serial_win.h"
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <process.h> //_beginthreadex
#include <WinSock2.h>
#include <windows.h>

static std::wstring stringToWString(const std::string &str)
{
	if (str.empty())
	{
		return std::wstring();
	}

	int size = MultiByteToWideChar(CP_ACP, 0, &str[0], (int)str.size(), NULL, 0);
	std::wstring ret = std::wstring(size, 0);
	MultiByteToWideChar(CP_ACP, 0, &str[0], (int)str.size(), &ret[0], size);

	return ret;
}

namespace vp100_serial
{ 
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
        m_portName = "\\\\.\\" + portName; // support COM10 above \\\\.\\COM10
        m_baudRate = baudRate;
        m_parity = parity;
        m_dataBits = dataBits;
        m_stopbits = stopbits;
        m_flowControl = flowControl;
    }

    //打开串口 
    bool Nvilidar_Serial::serialOpen()
    {
        bool bRet = false;

        TCHAR *tcPortName = NULL;
        #ifdef UNICODE
            std::wstring wstr = stringToWString(m_portName);
            tcPortName = const_cast<TCHAR *>(wstr.c_str());
        #else
            tcPortName = const_cast<TCHAR *>(m_portName.c_str());
        #endif
            unsigned long serialconfigSize = sizeof(COMMCONFIG);
            serialConfig.dwSize = serialconfigSize;

        if(!isSerialOpen())
        {
            // get a handle to the port
            serialHandle = CreateFile(tcPortName,                   // communication port string (COMX)
                                GENERIC_READ | GENERIC_WRITE, // read/write types
                                0,                            // comm devices must be opened with exclusive access
                                NULL,                         // no security attributes
                                OPEN_EXISTING,                // comm devices must use OPEN_EXISTING
                                FILE_ATTRIBUTE_NORMAL,         // Async I/O or sync I/O
                                NULL);

            if (serialHandle != INVALID_HANDLE_VALUE)
            {
				printf("handle ok 1:%d\n", serialHandle);

                // get default parameter
                GetCommConfig(serialHandle, &serialConfig, &serialconfigSize);
                GetCommState(serialHandle, &(serialConfig.dcb));

                // set parameter
                serialConfig.dcb.BaudRate = m_baudRate;
                serialConfig.dcb.ByteSize = m_dataBits;
                serialConfig.dcb.Parity = m_parity;
                serialConfig.dcb.StopBits = m_stopbits;
                // m_comConfigure.dcb.fDtrControl;
                // m_comConfigure.dcb.fRtsControl;

                serialConfig.dcb.fBinary = true;
                serialConfig.dcb.fInX = false;
                serialConfig.dcb.fOutX = false;
                serialConfig.dcb.fAbortOnError = false;
                serialConfig.dcb.fNull = false;

                // setBaudRate(m_baudRate);
                // setDataBits(m_dataBits);
                // setStopBits(m_stopbits);
                // setParity(m_parity);

                serialSetFlowControl(m_flowControl); // @todo

                //            COMMTIMEOUTS m_commTimeouts;
                //            //set read timeout
                //            m_commTimeouts.ReadIntervalTimeout = MAXWORD;
                //            m_commTimeouts.ReadTotalTimeoutMultiplier = 0;
                //            m_commTimeouts.ReadTotalTimeoutConstant = 0;
                //            //set write timeout
                //            m_commTimeouts.WriteTotalTimeoutConstant = 500;
                //            m_commTimeouts.WriteTotalTimeoutMultiplier = 100;
                //            SetCommTimeouts(m_handle,&m_commTimeouts); // @todo for test
                if (SetCommConfig(serialHandle, &serialConfig, serialconfigSize))
                {
                    // @todo
                    // Discards all characters from the output or input buffer of a specified communications resource. It
                    // can also terminate pending read or write operations on the resource.
                    PurgeComm(serialHandle, PURGE_TXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR | PURGE_RXABORT);

					serialConfigTimeout.ReadIntervalTimeout = MAXDWORD;//MAXDWORD;
                    serialConfigTimeout.ReadTotalTimeoutMultiplier = MAXDWORD;
                    serialConfigTimeout.ReadTotalTimeoutConstant = 0;
                    serialConfigTimeout.WriteTotalTimeoutMultiplier = 0;
                    serialConfigTimeout.WriteTotalTimeoutConstant = 0;
                    SetCommTimeouts(serialHandle, &serialConfigTimeout);

                    bRet = true;
                }
                else 
                {
                    bRet = false;
                }
            }
            else 
            {
                bRet = false;
            }
        }
        else 
        {
            bRet = false;
        }

        if(bRet == false)
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
            if (serialHandle != INVALID_HANDLE_VALUE)
            {
                // stop all event
                SetCommMask(serialHandle, 0); // SetCommMask(m_handle,0) stop WaitCommEvent()

                // Discards all characters from the output or input buffer of a specified communications resource. It can
                // also terminate pending read or write operations on the resource.
                PurgeComm(serialHandle, PURGE_TXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR | PURGE_RXABORT);

                CloseHandle(serialHandle);
                serialHandle = INVALID_HANDLE_VALUE;
            }
        }
    }

    //查看串口打开状态  
    bool Nvilidar_Serial::isSerialOpen()
    {
        // Finished
        return serialHandle != INVALID_HANDLE_VALUE;
    }

    //设置流控 
    void Nvilidar_Serial::serialSetFlowControl(int flow)
    {
        m_flowControl = flow;

        if (isSerialOpen())
        {
            switch (m_flowControl)
            {
                case FlowNone: // No flow control
                {
                    serialConfig.dcb.fOutxCtsFlow = FALSE;
                    serialConfig.dcb.fRtsControl = RTS_CONTROL_DISABLE;
                    serialConfig.dcb.fInX = FALSE;
                    serialConfig.dcb.fOutX = FALSE;
                    SetCommConfig(serialHandle, &serialConfig, sizeof(COMMCONFIG));
                    break;
                }
                case FlowSoftware: // Software(XON / XOFF) flow control
                {
                    serialConfig.dcb.fOutxCtsFlow = FALSE;
                    serialConfig.dcb.fRtsControl = RTS_CONTROL_DISABLE;
                    serialConfig.dcb.fInX = TRUE;
                    serialConfig.dcb.fOutX = TRUE;
                    SetCommConfig(serialHandle, &serialConfig, sizeof(COMMCONFIG));
                    break;
                }
                case FlowHardware: // Hardware(RTS / CTS) flow control
                {
                    serialConfig.dcb.fOutxCtsFlow = TRUE;
                    serialConfig.dcb.fRtsControl = RTS_CONTROL_HANDSHAKE;
                    serialConfig.dcb.fInX = FALSE;
                    serialConfig.dcb.fOutX = FALSE;
                    SetCommConfig(serialHandle, &serialConfig, sizeof(COMMCONFIG));
                    break;
                }
            }
        }
    }

    //读有效字节长度  
    int Nvilidar_Serial::serialReadAvaliable()
    {
        int iRet = -1;
        DWORD dwError = 0;
        COMSTAT comstat;
        if (!ClearCommError(serialHandle, &dwError, &comstat))
        {
            return -1;   
        }
        iRet = comstat.cbInQue;

        return iRet;
    }

    //读数据  
    int  Nvilidar_Serial::serialReadData(const uint8_t *data,int len)
    {
        DWORD iRet = -1;
        if (isSerialOpen())
        {
            if (ReadFile(serialHandle, (void *)data, (DWORD)len, &iRet, NULL))
            {
            }
            else
            {
                iRet = (DWORD)-1;
            }
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
        DWORD iRet = -1;

        if (isSerialOpen())
        {
            if (WriteFile(serialHandle, (void *)data, (DWORD)len, &iRet, NULL))
            {
            }
            else
            {
                iRet = (DWORD)-1;
            }
        }
        else
        {
            iRet = -1;
        }

        return iRet;
    }

    //刷新数据  
    void Nvilidar_Serial::serialFlush()
    {
        if (isSerialOpen())
        {
            PurgeComm(serialHandle, PURGE_TXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR | PURGE_RXABORT);
        }
    }
}

#endif