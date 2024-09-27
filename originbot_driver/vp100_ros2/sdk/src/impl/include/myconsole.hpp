
#pragma once 

#include <iostream>
#include <stdarg.h>
#include <stdio.h>
#include <string>
#include <fstream>
#include <string.h>
#include <stdlib.h>

#if defined(_WIN32)
	#include <WinSock2.h>
    #include <windows.h>
    #include <iostream>
    #if defined(_MSC_VER)
        #include <corecrt_io.h>
    #else
        #include <unistd.h>
    #endif
#else 
    #include <unistd.h>
#endif


using namespace std;

namespace vp100_lidar
{

//linux color type info 
#if !defined (_WIN32)
    #define COLOR_NONE "\033[m"
    #define COLOR_RED "\033[1;31m"
    #define COLOR_GREEN "\033[1;32m"
    #define COLOR_YELLOW "\033[1;33m"
    #define COLOR_CYAN "\033[1;36m"
#endif 

//the class 
class Console {
private:
    //Constructor
    Console (){
    }
    //destructor
    ~Console(){ 
    }
public:
    //show info 
    static void show(const char* message_, ...){
        char out[1024] = { 0 };
        va_list args;
        va_start(args, message_);
        vsnprintf(out, sizeof(out), message_, args);
        va_end(args);
    #if defined (_WIN32)
        SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), 0x02);
        printf("%s\r\n", out);
        SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), 0x07);
    #else
        printf(COLOR_GREEN);
        printf("%s\n", out);
        printf(COLOR_NONE);
    #endif
    }

    //message info 
    static void message(const char* message_, ...){
        char out[1024] = { 0 };
        va_list args;
        va_start(args, message_);
        vsnprintf(out, sizeof(out), message_, args);
        va_end(args);
    #if defined (_WIN32)
        SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), 0x02);
        printf("[VP100 Message]: ");
        printf("%s\r\n", out);
        SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), 0x07);
    #else
        printf(COLOR_GREEN);
        printf("[VP100 Message]: ");
        printf("%s\n", out);
        printf(COLOR_NONE);
    #endif
    }

    //warning info 
    static void warning(const char* warning_, ...){
        char out[1024] = { 0 };
        va_list args;
        va_start(args, warning_);
        vsnprintf(out, sizeof(out), warning_, args);
        va_end(args);
    #if defined (_WIN32)
        SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), 0x06);
        printf ("[VP100 Warning]: ");;
        printf ("%s\r\n", out);
        SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), 0x07);
    #else
        printf(COLOR_YELLOW);
        printf("[VP100 Warning]: ");
        printf("%s\n", out);
        printf(COLOR_NONE);
    #endif
    }

    //show error 
    static void error (const char* error_, ...)
    {
        char out[1024] = { 0 };
        va_list args;
        va_start(args, error_);
        vsnprintf (out, sizeof(out), error_, args);
        va_end(args);
    #if defined (_WIN32)
        SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), 0x04);
        printf ("[VP100 Error]: ");
        printf ("%s", out);
        printf ("\r\n");
        SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), 0x07);
    #else
        printf(COLOR_RED);
        printf("[VP100 Error]: ");
        printf("%s\n", out);
        printf(COLOR_NONE);
    #endif
    }
};

}
