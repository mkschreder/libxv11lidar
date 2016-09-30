@CODE_COVERAGE_RULES@
AM_CFLAGS=$(CODE_COVERAGE_CFLAGS) -std=gnu99 -Wall -Werror
AM_LDFLAGS=-L../src/.libs -lxv11
check_PROGRAMS=protocol
protocol_SOURCES=protocol.c
protocol_CFLAGS=$(AM_CFLAGS)
protocol_LDFLAGS=$(AM_LDFLAGS) 
TESTS=$(check_PROGRAMS)
