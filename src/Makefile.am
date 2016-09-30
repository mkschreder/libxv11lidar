@CODE_COVERAGE_RULES@
includedir=$(prefix)/include/xv11/
lib_LTLIBRARIES=libxv11.la
include_HEADERS=xv11.h xv11_tty.h
libxv11_la_SOURCES=xv11.c xv11_tty.c
libxv11_la_CFLAGS=$(CODE_COVERAGE_CFLAGS) -std=gnu99 -Wall -Werror
