#ifndef LOG_H
#define LOG_H

#include <stdio.h>


int log_msg(const char* fmt,...);
int log_warn(const char* fmt,...);
int log_err(const char* fmt,...);

int err_msg(const char* fmt,...);

#endif // LOG_H
