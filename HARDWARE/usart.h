#ifndef _USART_H_
#define _USART_H_


#define USE_USART1
//#define USE_USART3

#ifdef 	USE_USART3
#define	usart_Init()			usart3_Init()
#define usart_GetChar()		usart3_GetChar()
#define usart_PutChar(x)	usart3_PutChar(x)
#endif

#ifdef 	USE_USART1
#define	usart_Init()			usart1_Init()
#define usart_GetChar()		usart1_GetChar()
#define usart_PutChar(x)	usart1_PutChar(x)
#endif

extern void usart1_Init(void);
extern void usart3_Init(void);

extern int  usart1_GetChar(void);
extern int  usart1_PutChar(int c);

extern int  usart3_GetChar(void);
extern int  usart3_PutChar(int c);

#endif
