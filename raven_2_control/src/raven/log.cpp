
/**
Generic logging function

**/
#include <stdio.h>
#include <stdarg.h>
#include <ros/console.h>
char buf[1024];
int log_msg(const char* fmt,...)
{
    va_list args;
    va_start (args, fmt);
    //Do somethinh
    vsprintf(buf,fmt,args);
    va_end(args);
//    printf("%s",buf);
    ROS_INFO("%s",buf);
    return 0;
}

int log_warn(const char* fmt,...)
{
    va_list args;
    va_start (args, fmt);
    //Do somethinh
    vsprintf(buf,fmt,args);
    va_end(args);
//    printf("%s",buf);
    ROS_WARN("%s",buf);
    return 0;
}

int log_err(const char* fmt,...)
{
    va_list args;
    va_start (args, fmt);
    //Do somethinh
    vsprintf(buf,fmt,args);
    va_end(args);
//    printf("%s",buf);
    ROS_WARN("%s",buf);
    return 0;
}


int err_msg(const char* fmt,...)
{
    va_list args;
    va_start (args, fmt);
    //Do somethinh
    vsprintf(buf,fmt,args);
    va_end(args);
//    printf("%s",buf);
    ROS_ERROR("%s",buf);
    return 0;
}
