#include "./asprintf.h"

#include <stdarg.h> /* needed for va_list */
#include <stdio.h>  /* needed for vsnprintf */
#include <stdlib.h> /* needed for malloc-free */

int _vscprintf_so(const char *format, va_list pargs) {
  int retval;
  va_list argcopy;
  va_copy(argcopy, pargs);
  retval = vsnprintf(NULL, 0, format, argcopy);
  va_end(argcopy);
  return retval;
}

int vasprintf(char **strp, const char *fmt, va_list ap) {
  int len = _vscprintf_so(fmt, ap);
  if (len == -1)
    return -1;
  char *str = malloc((size_t)len + 1);
  if (!str)
    return -1;
  int r = vsnprintf(str, len + 1, fmt, ap); /* "secure" version of vsprintf */
  if (r == -1)
    return free(str), -1;
  *strp = str;
  return r;
}

int asprintf(char *strp[], const char *fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  int r = vasprintf(strp, fmt, ap);
  va_end(ap);
  return r;
}