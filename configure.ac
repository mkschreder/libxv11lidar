dnl Process this file with autoconf to produce a configure script
AC_INIT(libxv11,[0.1.0],mkschreder.uk@gmail.com)
AC_CONFIG_AUX_DIR(config)
AC_CONFIG_SRCDIR(./src/)
AC_CONFIG_MACRO_DIR([m4])

m4_pattern_allow([AM_DEFAULT_VERBOSITY])

AM_INIT_AUTOMAKE
AX_CODE_COVERAGE

LT_INIT

AC_PROG_CC

AC_ARG_WITH(pkgconfigdir,
	AC_HELP_STRING([--with-pkgconfigdir],
	[Use the specified pkgconfig dir (default is libdir/pkgconfig)]),
	[pkgconfigdir=${withval}],
	[pkgconfigdir='${libdir}/pkgconfig'])
AC_SUBST([pkgconfigdir])
AC_MSG_NOTICE([pkgconfig directory is ${pkgconfigdir}])

AC_CHECK_HEADERS([termios.h], HAVE_TTY_SUPPORT=1, [HAVE_TTY_SUPPORT=0])

AC_CONFIG_FILES([libxv11.pc])

AC_OUTPUT(Makefile src/Makefile test/Makefile)

