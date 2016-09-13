#include <stdio.h>
#include "usart.h"

#pragma import(__use_no_semihosting_swi)

struct __FILE { int handle; /* Add whatever you need here */ };
FILE __stdout;
FILE __stdin;

int fputc(int c, FILE *f) {
  return (usart_PutChar(c));
}

int fgetc(FILE *f) {
  return (usart_GetChar());
}

int ferror(FILE *f) {
  /* Your implementation of ferror */
  return EOF;
}

void _ttywrch(int c) {
  usart_PutChar(c);
}

void _sys_exit(int x) {
	x = x;
}
