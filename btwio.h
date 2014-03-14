/******************************************************************************
**
**      Filename:   btwio.h
**
**      Purpose:    SBS Windows NT PCI NanoBus/dataBlizzard driver header file.
**
**                  Contains common definitions for driver, user ISR, and DLL.
**
**  $Revision: 1.1 $
**
******************************************************************************/
/******************************************************************************
**
** Copyright (c) 2000 by SBS Technologies, Inc.
** Copyright (c) 1995-1997 by Bit 3 Computer Corporation.
**
** All Rights Reserved.
** License governs use and distribution.
**
******************************************************************************/
#ifndef BTWIO_H
#define BTWIO_H 1

#ifdef _BTDD_H
#include <devioctl.h>
#else /* _BTDD_H */
#include <winioctl.h>
#endif /* _BTDD_H */

/*
** Type definition for compatability with UNIX
*/
typedef PBYTE caddr_t;
typedef UCHAR u_char;
typedef USHORT u_short;
typedef ULONG u_long;

/*
**  Include files
*/
#include "btwdef.h"
#include "btngpci.h"
/*
** Ioctl definitions
*/
/*
** Interrupt sending ioctls
*/
#define BT_DIOC_SENDINTR \
    CTL_CODE(FILE_DEVICE_BTBRIDGE, 0x800, METHOD_BUFFERED, FILE_ANY_ACCESS)
typedef struct {
    bt_error_t error;
} BT_DIOCS_SENDINTR, *PBT_DIOCS_SENDINTR;
#define BT_DIOC_SENDVECTOR \
    CTL_CODE(FILE_DEVICE_BTBRIDGE, 0x801, METHOD_BUFFERED, FILE_ANY_ACCESS)

/*
** Status/initialization ioctls
*/
#define BT_DIOC_SETUP \
    CTL_CODE(FILE_DEVICE_BTBRIDGE, 0x802, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define BT_DIOC_STATUS \
    CTL_CODE(FILE_DEVICE_BTBRIDGE, 0x803, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define BT_DIOC_CLR_STATUS \
    CTL_CODE(FILE_DEVICE_BTBRIDGE, 0x804, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define BT_DIOC_RESET \
    CTL_CODE(FILE_DEVICE_BTBRIDGE, 0x805, METHOD_BUFFERED, FILE_ANY_ACCESS)
typedef struct {
    bt_error_t error;
    bt_status_t hw_status;
} BT_DIOCS_STATUS, *PBT_DIOCS_STATUS;

/*
** Register read/write ioctls
*/
#define BT_DIOC_IOREG \
    CTL_CODE(FILE_DEVICE_BTBRIDGE, 0x806, METHOD_BUFFERED, FILE_ANY_ACCESS)

/*
** Tracing ioctls
*/
#define BT_DIOC_GETTRACE \
    CTL_CODE(FILE_DEVICE_BTBRIDGE, 0x807, METHOD_BUFFERED, FILE_ANY_ACCESS)
typedef struct {
    bt_error_t error;
} BT_DIOCS_GETTRACE, *PBT_DIOCS_GETTRACE;

/*
** TAS/CAS ioctls
*/
#define BT_DIOC_TAS \
    CTL_CODE(FILE_DEVICE_BTBRIDGE, 0x808, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define BT_DIOC_CAS \
    CTL_CODE(FILE_DEVICE_BTBRIDGE, 0x809, METHOD_BUFFERED, FILE_ANY_ACCESS)

/*
** Info paramter ioctls
*/
#define BT_DIOC_SETPARAM \
    CTL_CODE(FILE_DEVICE_BTBRIDGE, 0x80a, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define BT_DIOC_GETPARAM \
    CTL_CODE(FILE_DEVICE_BTBRIDGE, 0x80b, METHOD_BUFFERED, FILE_ANY_ACCESS)
    
/*
** Interrupt thread ioctls
*/
#define BT_DIOC_THREAD_REG \
    CTL_CODE(FILE_DEVICE_BTBRIDGE, 0x80c, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define BT_DIOC_THREAD_UNREG \
    CTL_CODE(FILE_DEVICE_BTBRIDGE, 0x80d, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define BT_DIOC_THREAD_ADD \
    CTL_CODE(FILE_DEVICE_BTBRIDGE, 0x80e, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define BT_DIOC_THREAD_DELETE \
    CTL_CODE(FILE_DEVICE_BTBRIDGE, 0x80f, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define BT_DIOC_THREAD_WAIT \
    CTL_CODE(FILE_DEVICE_BTBRIDGE, 0x810, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define BT_DIOC_THREAD_WAKE \
    CTL_CODE(FILE_DEVICE_BTBRIDGE, 0x811, METHOD_BUFFERED, FILE_ANY_ACCESS)
    
#define BT_DIOC_SEMA_TAKE \
    CTL_CODE(FILE_DEVICE_BTBRIDGE, 0x812, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define BT_DIOC_SEMA_GIVE \
    CTL_CODE(FILE_DEVICE_BTBRIDGE, 0x813, METHOD_BUFFERED, FILE_ANY_ACCESS)
    
/*
** Map/unmap ioctls
*/
#define BT_DIOC_MMAP \
    CTL_CODE(FILE_DEVICE_BTBRIDGE, 0x814, METHOD_BUFFERED, FILE_ANY_ACCESS)
typedef struct {
    bt_error_t error;
    void *mem_p;
    bt_devaddr_t map_off;
    size_t map_len;
    bt_accessflag_t access_flags;
    bt_swap_t swapping;
} BT_DIOCS_MMAP, *PBT_DIOCS_MMAP;
#define BT_DIOC_UNMMAP \
    CTL_CODE(FILE_DEVICE_BTBRIDGE, 0x815, METHOD_BUFFERED, FILE_ANY_ACCESS)
typedef struct {
    bt_error_t error;
    void *mem_p;
} BT_DIOCS_UNMMAP, *PBT_DIOCS_UNMMAP;
/*
** Buffer and hardware bind/unbind ioctls
*/
#define BT_DIOC_BIND \
    CTL_CODE(FILE_DEVICE_BTBRIDGE, 0x816, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define BT_DIOC_UNBIND \
    CTL_CODE(FILE_DEVICE_BTBRIDGE, 0x817, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define BT_DIOC_HW_BIND \
    CTL_CODE(FILE_DEVICE_BTBRIDGE, 0x818, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define BT_DIOC_HW_UNBIND \
    CTL_CODE(FILE_DEVICE_BTBRIDGE, 0x819, METHOD_BUFFERED, FILE_ANY_ACCESS)

/*
** Hardware read/write ioctls
*/
#define BT_DIOC_HW_READ \
    CTL_CODE(FILE_DEVICE_BTBRIDGE, 0x81a, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define BT_DIOC_HW_WRITE \
    CTL_CODE(FILE_DEVICE_BTBRIDGE, 0x81b, METHOD_BUFFERED, FILE_ANY_ACCESS)

/*
** No longer implemented, allways returns BT_SUCCESS
*/
#define BT_DIOC_LOCK \
    CTL_CODE(FILE_DEVICE_BTBRIDGE, 0x81c, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define BT_DIOC_UNLOCK \
    CTL_CODE(FILE_DEVICE_BTBRIDGE, 0x81d, METHOD_BUFFERED, FILE_ANY_ACCESS)

/*
** Hardware read/write ioctls
*/
#define BT_DIOC_LCARD_DIAG \
    CTL_CODE(FILE_DEVICE_BTBRIDGE, 0x81e, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define BT_DIOC_CABLE_DIAG \
    CTL_CODE(FILE_DEVICE_BTBRIDGE, 0x81f, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define BT_DIOC_RCARD_DIAG \
    CTL_CODE(FILE_DEVICE_BTBRIDGE, 0x820, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define BT_DIOC_PAIR_DIAG \
    CTL_CODE(FILE_DEVICE_BTBRIDGE, 0x821, METHOD_BUFFERED, FILE_ANY_ACCESS)

/*
**  driver version ioctl
*/
#define BT_DIOC_DRIVER_VERSION \
    CTL_CODE(FILE_DEVICE_BTBRIDGE, 0x822, METHOD_BUFFERED, FILE_ANY_ACCESS)

/* 
** Always leave BT_DIOC_QUERY as the last IOCTL number 
*/
#define BT_DIOC_QUERY \
    CTL_CODE(FILE_DEVICE_BTBRIDGE, 0x823, METHOD_BUFFERED, FILE_ANY_ACCESS)
typedef struct {
    bt_error_t error;
    int unit_num;
    bt_data32_t bus_num;
    bt_data32_t slot_num;
    bt_data32_t loc_id;
} BT_DIOCS_QUERY, *PBT_DIOCS_QUERY;

#endif /* BTWIO_H */
