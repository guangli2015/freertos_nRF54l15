
#pragma once

void log_inf(const char *fmt, ...);

#define LOG_INF(...) log_inf(__VA_ARGS__)
//#define LOG_INF(...)
int log_init();


