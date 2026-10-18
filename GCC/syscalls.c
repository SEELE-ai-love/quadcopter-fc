/* GCC/newlib runtime bridge. The existing main.c fputc remains the UART backend. */
#include <errno.h>
#include <stddef.h>
#include <stdio.h>
#include <sys/stat.h>
#include "stm32f4xx.h"
extern char __HeapBase, __HeapLimit;
void _init(void) {}
void _fini(void) {}
int _write(int file, char *data, int length)
{
    int i;
    if(file != 1 && file != 2) { errno = EBADF; return -1; }
    for(i = 0; i < length; ++i)
        if(fputc((unsigned char)data[i], stdout) == EOF) return i ? i : -1;
    return length;
}
void *_sbrk(ptrdiff_t increment)
{
    static char *current;
    char *previous;
    if(current == NULL) current = &__HeapBase;
    if(increment > &__HeapLimit-current || increment < -((ptrdiff_t)(current-&__HeapBase)))
    { errno = ENOMEM; return (void *)-1; }
    previous = current;
    current += increment;
    return previous;
}
int _read(int file, char *data, int length)
{ (void)file; (void)data; (void)length; errno = ENOSYS; return -1; }
int _close(int file) { (void)file; errno = EBADF; return -1; }
int _fstat(int file, struct stat *st)
{
    if(file < 0 || file > 2) { errno = EBADF; return -1; }
    st->st_mode = S_IFCHR;
    return 0;
}
int _isatty(int file) { return file >= 0 && file <= 2; }
int _lseek(int file, int offset, int whence)
{ (void)file; (void)offset; (void)whence; errno = ESPIPE; return -1; }
int _getpid(void) { return 1; }
int _kill(int pid, int signal)
{ (void)pid; (void)signal; errno = EINVAL; return -1; }
void _exit(int status)
{ (void)status; __disable_irq(); for(;;) __WFI(); }
