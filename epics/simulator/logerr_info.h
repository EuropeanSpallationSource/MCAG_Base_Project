#include <string.h> /* strerror */
#include <errno.h>

extern unsigned int debug_print_flags;
extern unsigned int die_on_error_flags;

#define PRINT_STDOUT_BIT0() (debug_print_flags & 1)
#define PRINT_STDOUT_BIT1() (debug_print_flags & (1<<1))
#define PRINT_STDOUT_BIT2() (debug_print_flags & (1<<2))

#define PRINT_STDOUT_BIT6() (debug_print_flags & (1<<6))
#define PRINT_STDOUT_BIT7() (debug_print_flags & (1<<7))

#define DIE_ON_ERROR_BIT0() (die_on_error_flags & 1)
#define DIE_ON_ERROR_BIT1() (die_on_error_flags & (1<<1))




#define LOGINFO(fmt, ...)                        \
{                                                \
  (void)fprintf(stdlog, fmt, ##__VA_ARGS__);     \
}

#define LOGINFO6(fmt, ...)                       \
do {                                             \
  if (PRINT_STDOUT_BIT6()) (void)fprintf(stdlog, fmt, ##__VA_ARGS__);   \
} while (0)

#define LOGINFO7(fmt, ...)                       \
do {                                             \
  if (PRINT_STDOUT_BIT7()) (void)fprintf(stdlog, fmt, ##__VA_ARGS__);   \
} while (0)


#define LOGERR(fmt, ...)                         \
{                                                \
  (void)fprintf(stdlog, fmt, ##__VA_ARGS__);     \
}


#define LOGERR_ERRNO(fmt, ...)                   \
{                                                \
  (void)fprintf(stdlog, "%s/%s:%d errno=%d (%s) ", __FILE__,__FUNCTION__, __LINE__, errno, strerror(errno)); \
  (void)fprintf(stdlog, fmt, ##__VA_ARGS__);     \
}
