@CODE_COVERAGE_RULES@
COMMON_CFLAGS:=$(CODE_COVERAGE_CFLAGS) -std=gnu99 -Wall -Werror
COMMON_LDFLAGS:=-L../src/.libs -lxv11
check_PROGRAMS=protocol
protocol_SOURCES=protocol.c
protocol_CFLAGS=$(COMMON_CFLAGS)
protocol_LDFLAGS=$(COMMON_LDFLAGS) 
TESTS=$(check_PROGRAMS)
