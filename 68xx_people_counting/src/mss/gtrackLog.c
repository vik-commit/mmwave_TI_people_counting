#if (defined (GTRACK_2D)) || (defined (GTRACK_3D))
#include <ti/alg/gtrack/gtrack.h>
#else
#include <ti/alg/gtrack/gtrack.h>
#endif

#include <stdio.h>
#include <stdarg.h>

void gtrack_log(GTRACK_VERBOSE_TYPE level, const char *format, ...)
{
	va_list args;
    va_start(args, format);
	vprintf(format, args);
	va_end(args);
}