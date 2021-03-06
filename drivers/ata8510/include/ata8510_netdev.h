/*
 * Copyright (C) 2016 Antonio Galea <antonio.galea@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_ata8510
 * @{
 *
 * @file
 * @brief       Netdev interface to ATA8510 driver
 *
 * @author      Antonio Galea <antonio.galea@gmail.com>
 */

#ifndef ATA8510_NETDEV_H_
#define ATA8510_NETDEV_H_

#include "net/netdev2.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Reference to the netdev device driver struct
 */
extern const netdev2_driver_t ata8510_driver;

#define ATA8510_SYSTEM  0
#define ATA8510_EVENTS  1
#define ATA8510_POWER   2
#define ATA8510_CONFIG  3

#define ATA8510_SYSTEM_SYS_ERR      0x80
#define ATA8510_SYSTEM_CMD_RDY      0x40
#define ATA8510_SYSTEM_SYS_RDY      0x20
#define ATA8510_SYSTEM_AVCCLOW      0x10
#define ATA8510_SYSTEM_LOWBATT      0x08
#define ATA8510_SYSTEM_SFIFO        0x04
#define ATA8510_SYSTEM_DFIFO_RX     0x02
#define ATA8510_SYSTEM_DFIFO_TX     0x01

#define ATA8510_EVENTS_IDCHKA       0x80
#define ATA8510_EVENTS_WCOKA        0x40
#define ATA8510_EVENTS_SOTA         0x20
#define ATA8510_EVENTS_EOTA         0x10
#define ATA8510_EVENTS_IDCHKB       0x08
#define ATA8510_EVENTS_WCOKB        0x04
#define ATA8510_EVENTS_SOTB         0x02
#define ATA8510_EVENTS_EOTB         0x01

#define ATA8510_CONFIG_SERVICE(x)   ((x) & 0x07)
#define ATA8510_CONFIG_CHANNEL(x)   (((x) & 0x30)>>4)

#ifdef __cplusplus
}
#endif

#endif /* ATA8510_NETDEV_H_ */
/** @} */
