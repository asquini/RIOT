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
 * @brief       Netdev adaption for the ATA8510 driver
 *
 * @author      Antonio Galea <antonio.galea@gmail.com>
 * @author      Roberto Asquini <bobasquins@gmail.com>
 *
 * @}
 */

#include <string.h>
#include <assert.h>
#include <errno.h>

#include "net/eui64.h"
#include "net/ieee802154.h"
#include "net/netdev2.h"
#include "net/netdev2/ieee802154.h"

#include "ata8510.h"
#include "ata8510_netdev.h"
#include "ata8510_internal.h"
#include "xtimer.h"

#define ENABLE_DEBUG (1)
#include "debug.h"

#define _MAX_MHR_OVERHEAD   (25)

static int _send(netdev2_t *netdev, const struct iovec *vector, unsigned count);
static int _recv(netdev2_t *netdev, void *buf, size_t len, void *info);
static int _init(netdev2_t *netdev);
static void _isr(netdev2_t *netdev);
static int _get(netdev2_t *netdev, netopt_t opt, void *val, size_t max_len);
static int _set(netdev2_t *netdev, netopt_t opt, void *val, size_t len);

const netdev2_driver_t ata8510_driver = {
    .send = _send,
    .recv = _recv,
    .init = _init,
    .isr = _isr,
    .get = _get,
    .set = _set,
};


static void _irq_handler(void *arg)
{
    netdev2_t *dev = (netdev2_t *) arg;

    if (dev->event_callback) {
        dev->event_callback(dev, NETDEV2_EVENT_ISR);
    }
}

static int _init(netdev2_t *netdev)
{
    ata8510_t *dev = (ata8510_t *)netdev;

    /* initialise GPIOs */
    gpio_init(dev->params.cs_pin, GPIO_OUT);
    gpio_set(dev->params.cs_pin);
    gpio_init(dev->params.sleep_pin, GPIO_OUT);
    gpio_clear(dev->params.sleep_pin);
    gpio_init(dev->params.reset_pin, GPIO_OUT);
    gpio_set(dev->params.reset_pin);
    gpio_init_int(dev->params.int_pin, GPIO_IN, GPIO_FALLING, _irq_handler, dev);

    ata8510_power_on(dev);
    spi_poweron(dev->params.spi);

    /* test if the SPI is set up correctly and the device is responding */
    if (ata8510_get_device_signature(dev) != ATA8510_PARTNUM){
        DEBUG("[ata8510] error: unable to read correct part number\n");
        return -1;
    }
    ringbuffer_init(&dev->tx_rb, (char *)dev->tx_mem, sizeof(dev->tx_mem));
    ringbuffer_init(&dev->rx_rb, (char *)dev->rx_mem, sizeof(dev->rx_mem));

    DEBUG("[ata8510] init done\n");

#ifdef MODULE_NETSTATS_L2
    memset(&netdev->stats, 0, sizeof(netstats_t));
#endif
    /* reset device to default values and put it into RX state */
    ata8510_reset(dev);

    return 0;
}

static int _send(netdev2_t *netdev, const struct iovec *vector, unsigned count)
{
    ata8510_t *dev = (ata8510_t *)netdev;
    const struct iovec *ptr = vector;
    size_t len = 0;
    uint8_t data[ATA8510_DFIFO_TX_LENGTH];
    int n;

    if (dev->pending_tx) {
        DEBUG("[ata8510] error: pending transmissiong\n");
        return -EOVERFLOW;
    }
    dev->pending_tx = 1;

    ata8510_tx_prepare(dev);

    /* load packet data into buffer */
    for (unsigned i = 0; i < count; i++, ptr++) {
        /* current packet data too long */
        if ((len + ptr->iov_len) > ATA8510_MAX_PKT_LENGTH) {
            DEBUG("[ata8510] error: packet too large (%u byte) to be sent\n",
                  (unsigned)len);
            return -EOVERFLOW;
        }
#ifdef MODULE_NETSTATS_L2
        netdev->stats.tx_bytes += len;
#endif
        len = ata8510_tx_load(dev, ptr->iov_base, ptr->iov_len, len);
    }
DEBUG("_send: packet len=%d\n", len);

    // activate tx mode 
    ata8510_tx_exec(dev);

    // first FIFO load; afterwards moving from buffer to DFIFO_TX is accomplished in _isr()
    n = ata8510_ReadFillLevelTxFIFO(dev); // TODO: DFIFO_TX should be empty
    if (n < ATA8510_DFIFO_TX_LENGTH) { // there is free space in DFIFO_TX
        n = ringbuffer_get(&dev->tx_rb, (char *)data, ATA8510_DFIFO_TX_LENGTH - n);
DEBUG("_send: first batch len=%d\n", n);
        ata8510_WriteTxFifo(dev, n, data);
    }

    /* return the number of bytes that were queued for sending */
    return (int)len;
}

static int _recv(netdev2_t *netdev, void *buf, size_t len, void *info)
{
    size_t pkt_len;
    uint8_t i;
	int rssicalc;

    ata8510_t *dev = (ata8510_t *)netdev;

    // TODO: avoid races when accessing dev->rx_rb
    if(!dev->rx_rb.avail){ return 0; }

    pkt_len = dev->rx_rb.avail;

    /* just return length when buf == NULL */
    if (buf == NULL) {
        return pkt_len;
    }
#ifdef MODULE_NETSTATS_L2
    netdev->stats.rx_count++;
    netdev->stats.rx_bytes += pkt_len;
#endif
    /* not enough space in buf */
    if (pkt_len > len) {
        return -ENOBUFS;
    }

    /* copy payload */
    ringbuffer_get(&dev->rx_rb, (char *)buf, pkt_len);

	DEBUG("_recv pkt_len=%d\n", pkt_len);
	DEBUG("RSSI_len=%d\nFirst 5 Values: ", dev->RSSI_len); //ROB
	for (i=0; i<6; i++) DEBUG("%d ",dev->RSSI[i]); //ROB
	DEBUG("\n"); //ROB

    if (info != NULL) {
        netdev2_ieee802154_rx_info_t *radio_info = info;
		// ROB radio_info contains 2 x 1 byte of info. Not possible to use them for making average RSSI or storing dBm that is always negative.
        radio_info->lqi=0;
        radio_info->rssi=0;
        rssicalc=0;
        if (dev->RSSI_len > 0) {
			// TO BE CHECKED: first value of RSSI is always zero. Why?
            for (i=0; i<dev->RSSI_len; i++) { rssicalc+=dev->RSSI[i]; }
            radio_info->rssi = rssicalc/dev->RSSI_len;
            /* we abuse LQI for storing dBm */  // <- Not possible! dBm always negative better have a function to calulate it only when you need
            DEBUG(" RSSI values = %d RSSI = %d \n", dev->RSSI_len, radio_info->rssi);
        } else {
            DEBUG("no RSSI values read\n");
        }
    }
    return pkt_len;
}

static int _get(netdev2_t *netdev, netopt_t opt, void *val, size_t max_len)
{
    int res = -ENOTSUP;

    if (((res = netdev2_ieee802154_get((netdev2_ieee802154_t *)netdev, opt, val,
                                       max_len)) >= 0) || (res != -ENOTSUP)) {
        return res;
    }

    return res;
}

static int _set(netdev2_t *netdev, netopt_t opt, void *val, size_t len)
{
    int res = -ENOTSUP;
    return res;
}

static void _isr(netdev2_t *netdev){
    uint8_t data[37];
    uint8_t status[4];
    uint8_t dataSFIFO[19];
    uint8_t mystate8510, mynextstate8510;
	int i, n, m;
#if ENABLE_DEBUG
    uint16_t errorcode;
#endif

    ata8510_t *dev = (ata8510_t *)netdev;
	dev->interrupts++;
	ata8510_GetEventBytes(dev, status);

	mystate8510 = ata8510_get_state(dev);
	mynextstate8510 = ata8510_get_state_after_tx(dev);


	if (status[ATA8510_EVENTS] & ATA8510_EVENTS_EOTA) {
//DEBUG("_isr: EOTA, state=%d\n", mystate8510);
	    switch (mystate8510) {
            case TX_ON:
                dev->pending_tx = 0;

                // empty TX buffer
                if (dev->tx_rb.avail>0) {
                    DEBUG(
                        "_isr: TX end, discarding %d stale bytes from buffer\n",
                        dev->tx_rb.avail
                    );
                    ringbuffer_remove(&dev->tx_rb, dev->tx_rb.avail);
                }

                ata8510_SetIdleMode(dev);
                switch (mynextstate8510) {
                    case IDLE:
                        break;
                    case POLLING:
                         ata8510_SetPollingMode(dev);
                         break;
                    default:
                         printf("_isr: Cannot handle state 8510 %d after TX\n", mynextstate8510);
                         break;
                }
                break;
		    case POLLING:
                dev->service = ATA8510_CONFIG_SERVICE(status[ATA8510_CONFIG]);
                dev->channel = ATA8510_CONFIG_CHANNEL(status[ATA8510_CONFIG]);


                n = ata8510_ReadFillLevelRxFIFO(dev);
                ata8510_ReadRxFIFO(dev, n, data);

                m = ata8510_ReadFillLevelRSSIFIFO(dev);
                if (m>0) {
                    ata8510_ReadRSSIFIFO(dev, m, dataSFIFO);
                    for (i=0; i<m; i++) { dev->RSSI[i] = dataSFIFO[i+3]; }
                    dev->RSSI_len = m;
                }

                DEBUG("_isr: EOTA, RSSI len = %d\n", m);
                DEBUG("_isr: EOTA, RSSI len = %d; RSSI data: ", m);
                for (i=0; i<m; i++) DEBUG(".%d", dataSFIFO[i+3]);
                DEBUG("\n");

				DEBUG("_isr: EOTA, n=%d, dev->rx_rb.avail=%d\n", n, dev->rx_rb.avail);

                if (n>0) { // there is data to read

                    i = ringbuffer_add(&dev->rx_rb, (char *)data+3, n);
                    if (i != n){
                        DEBUG("_isr: rx buffer overflow\n");
                    }
                }
                DEBUG("--\n");
                DEBUG("_isr: EOTA, pkt_len = %d\n", dev->rx_rb.avail);


                // TODO: is this really needed?
                ata8510_SetIdleMode(dev);
                ata8510_SetPollingMode(dev);

                netdev->event_callback(netdev, NETDEV2_EVENT_RX_COMPLETE);
                break;

		    default:
                printf("_isr EOTA in state %d; event bytes: %02x %02x %02x %02x\n",
                    mystate8510, status[0], status[1], status[2], status[3]
                );
				dev->unknown_case++;
		        break;
        }
    }

	if (status[ATA8510_SYSTEM] & ATA8510_SYSTEM_SYS_ERR) {
		// SYS_ERR happened
#if ENABLE_DEBUG
		errorcode = ata8510_read_error_code(dev);
		DEBUG("     _isr SYS_ERR! %02x %02x %02x %02x Errorcode = 0x%04x  SysErr=%d  SSM state= %d\n",
				status[0], status[1], status[2], status[3], errorcode, errorcode>>8, errorcode&0xff);
#endif
		dev->sys_errors++;
	}

    // start event
#if 0
	if (status[ATA8510_EVENTS] & ATA8510_EVENTS_SOTA) {
//DEBUG("_isr: SOTA, state=%d\n", mystate8510);
	    switch (mystate8510) {
		    case POLLING:

                n = ata8510_ReadFillLevelRxFIFO(dev);
                if (n>0){
                    DEBUG(
                        "_isr: RX start, discarding %d stale bytes from DFIFO_RX\n",
                        n
                    );
                    ata8510_ReadRxFIFO(dev, n, data);
                }
                if (dev->rx_rb.avail>0) {
                    DEBUG(
                        "_isr: RX start, discarding %d stale bytes from buffer\n",
                        dev->rx_rb.avail
                    );
                    ringbuffer_remove(&dev->rx_rb, dev->rx_rb.avail);
                }
                break;
        }
    }
#endif

    // end event


    // SFIFO fill event
	if (status[ATA8510_SYSTEM] & ATA8510_SYSTEM_SFIFO) {
        // TODO: in TX mode, SFIFO fills up with preamble
		n = ata8510_ReadFillLevelRSSIFIFO(dev);
		if (n>0) {
            ata8510_ReadRSSIFIFO(dev, n, dataSFIFO);
            for (i=0; i<n; i++) { dev->RSSI[i] = dataSFIFO[i+3]; }
            dev->RSSI_len = n;
            DEBUG("_isr: SFIFO, RSSI data: ");
            for (i=0; i<n; i++) DEBUG(".%d", dataSFIFO[i]);
            DEBUG("\n");
        }
		DEBUG("_isr: SFIFO, state=%d, n=%d\n", mystate8510, n);
    }

    // DFIFO_RX fill event
	if (status[ATA8510_SYSTEM] & ATA8510_SYSTEM_DFIFO_RX) {
        n = ata8510_ReadFillLevelRxFIFO(dev);
        if (n>0) { // there is data to read
            ata8510_ReadRxFIFO(dev, n, data);
            i = ringbuffer_add(&dev->rx_rb, (char *)data+3, n);
            if (i != n){
                DEBUG("_isr rx buffer overflow");
            }
        }
		DEBUG("_isr: DFIFO_RX, state=%d, n=%d\n", mystate8510, n);
	}


    // DFIFO_TX fill event
	if (status[ATA8510_SYSTEM] & ATA8510_SYSTEM_DFIFO_TX) {

        n = ata8510_ReadFillLevelTxFIFO(dev);
n=47-32;

DEBUG("_isr: DFIFO_TX, state=%d, n=%d\n", mystate8510, n);
        if (n < ATA8510_DFIFO_TX_LENGTH) { // there is free space in DFIFO_TX
            n = ringbuffer_get(&dev->tx_rb, (char *)data, ATA8510_DFIFO_TX_LENGTH - n);
            if (n>0) { // we have data to send
                 ata8510_WriteTxFifo(dev, n, data);
            }
        }
	}

DEBUG("     _isr Get Event Bytes: %02x %02x %02x %02x \n",
				status[0], status[1], status[2], status[3]);

}