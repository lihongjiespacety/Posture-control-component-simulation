#ifndef CLOCK_H
#define CLOCK_H

#ifdef __cplusplus
 extern "C" {
#endif

typedef struct ts
{
    uint32_t tv_usec;
    uint32_t tv_sec;
}timestamp_t;

void clock_get_time(timestamp_t * time);

void clock_set_time(timestamp_t * time);
#ifdef __cplusplus
}
#endif

#endif