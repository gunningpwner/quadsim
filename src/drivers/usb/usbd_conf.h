// In include/usbd_conf.h

#ifndef __USBD_CONF_H__
#define __USBD_CONF_H__

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define DEVICE_FS 0

#define USBD_MAX_NUM_INTERFACES     1U
#define USBD_MAX_NUM_CONFIGURATION  1U
#define USBD_MAX_STR_DESC_SIZ       0x100U
#define USBD_DEBUG_LEVEL            0U
#define USBD_LPM_ENABLED            0U
#define USBD_SELF_POWERED           1U

#define USBD_malloc         (void *)malloc
#define USBD_free           free
#define USBD_memset         memset
#define USBD_memcpy         memcpy

#if (USBD_DEBUG_LEVEL > 0U)
#define USBD_UsrLog(...)    printf(__VA_ARGS__);\
                            fflush(stdout);
#else
#define USBD_UsrLog(...)
#endif

#if (USBD_DEBUG_LEVEL > 1U)
#define USBD_ErrLog(...)    printf("ERROR: ");\
                            printf(__VA_ARGS__);\
                            fflush(stdout);
#else
#define USBD_ErrLog(...)
#endif

#if (USBD_DEBUG_LEVEL > 2U)
#define USBD_DbgLog(...)    printf("DEBUG: ");\
                            printf(__VA_ARGS__);\
                            fflush(stdout);
#else
#define USBD_DbgLog(...)
#endif

#ifdef __cplusplus
}
#endif

#endif /*__USBD_CONF_H__*/