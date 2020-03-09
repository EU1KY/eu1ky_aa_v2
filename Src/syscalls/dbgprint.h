#ifndef DBGPRINT_H_
#define DBGPRINT_H_

#include <stdio.h>

#ifdef DEBUG
#define DBGPRINT(...) printf(__VA_ARGS__)
#else
#define DBGPRINT(...)
#endif

#endif
