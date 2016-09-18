#ifndef _LOG_H_
#define _LOG_H_

#ifdef __DEBUG__
#define LOG_D(X,...) printf(X,##__VA_ARGS__)
#define LOG_E(X,...) printf(X,##__VA_ARGS__)
#else
#define LOG_D(X,...)
#define LOG_E(X,...)
#endif

#endif
