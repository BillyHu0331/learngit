/*
 * AiSpeech spi interface driver for DW SPI Core
 *
 * Copyright (c) 2016, AiSpeech Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 */
#include <linux/version.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/fs.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/miscdevice.h>
#include <linux/hrtimer.h>
#include <asm/uaccess.h>
#include <asm/param.h>     /* HZ */
#include <linux/proc_fs.h>

#include "aispeech-spi-audio.h"

enum {
    LOG_WARNING  = 1U << 0,
    LOG_DEBUG = 1U << 1,
    LOG_VERBOSE = 1U << 2
};

static u32 gDebugMask = LOG_WARNING | LOG_DEBUG;

#define print_vdbg(format, arg...)    \
({  \
    if (gDebugMask & LOG_VERBOSE)  \
        printk(format, ##arg); \
})

#define print_ddbg(format, arg...)    \
({  \
    if (gDebugMask & LOG_DEBUG)  \
        printk(format, ##arg); \
})

#define print_wdbg(format, arg...)    \
({  \
    if (gDebugMask & LOG_WARNING)  \
        printk(format, ##arg); \
})

/*
 * This supports access to SPI devices using normal userspace I/O calls.
 * Note that while traditional UNIX/POSIX I/O semantics are half duplex,
 * and often mask message boundaries, full SPI support requires full duplex
 * transfers.  There are several kinds of internal message boundaries to
 * handle chipselect management and other protocol options.
 *
 * SPI has a character major number assigned.  We allocate minor numbers
 * dynamically using a bitmask.  You must use hotplug tools, such as udev
 * (or mdev with busybox) to create and destroy the /dev/spidevB.C device
 * nodes, since there is no fixed association of minor numbers with any
 * particular SPI bus or device.
 */
#define SPIDEV_MAJOR            153   /* assigned */
#define N_SPI_MINORS            32    /* ... up to 256 */

static DECLARE_BITMAP(minors, N_SPI_MINORS);

/* Bit masks for spi_device.mode management.  Note that incorrect
 * settings for some settings can cause *lots* of trouble for other
 * devices on a shared bus:
 *
 *  - CS_HIGH ... this device will be active when it shouldn't be
 *  - 3WIRE ... when active, it won't behave as it should
 *  - NO_CS ... there will be no explicit message boundaries; this
 *    is completely incompatible with the shared bus model
 *  - READY ... transfers may proceed when they shouldn't.
 *
 * REVISIT should changing those flags be privileged?
 */
#define SPI_MODE_MASK        (SPI_CPHA | SPI_CPOL | SPI_CS_HIGH \
                | SPI_LSB_FIRST | SPI_3WIRE | SPI_LOOP \
                | SPI_NO_CS | SPI_READY)

#define SPI_AUDIO_TAIL_MARGIN_LEN    32

const struct spi_audio_config gSpiAudioConfig = {
    .channels = 8,
    .rate = 16000,
    .periodSize = 320, /* (sample_rate * BUFF_DURATION_MS) / 1000 = 16000 * 20 / 1000 = 320 */
    .periodCount = 10,
    .format = SPI_AUDIO_FORMAT_S16_LE,
};

struct spiaud_ring_buffer {
    u8      *pBufBase;
    u8      *pRead;
    u8      *pWrite;
    int     bufLen;
    int     bufCount;

    struct mutex  bufLock;
};

struct spiaud_data {
    dev_t             devt;
    struct mutex      spiLock;
    struct spi_device   *spi;
    struct list_head    deviceEntry;
    unsigned        users;

    struct spi_audio_header_info  audioHeaderInfo;

    u8            *pTxBuffer;
    u8            *pRxBuffer;
    int           transferBufLen;
    struct mutex  transferBufLock;
    struct spiaud_ring_buffer     ringBuffer;

    u8         *pUserCopyBuf;
    int        userCopyBufLen;
};

static LIST_HEAD(gDeviceList);
static DEFINE_MUTEX(gDeviceListLock);

static atomic_t gTransferEventFlag = ATOMIC_INIT(0);
static DECLARE_WAIT_QUEUE_HEAD(gTransferEventWq);
static DECLARE_WAIT_QUEUE_HEAD(gTransferEventRq);

static int gPeerSequenceNumber = -1;
static int gPeerSequenceNumberBase = -1;
static s64 gTimeUsArray[10];
static unsigned gTimeIndex = 0;

/*-------------------------------------------------------------------------*/

static u32 spiaud_format_to_bits(enum spi_audio_format format)
{
    switch (format)
    {
    case SPI_AUDIO_FORMAT_S32_LE:
        return 32;
    case SPI_AUDIO_FORMAT_S24_LE:
        return 24;
    default:
    case SPI_AUDIO_FORMAT_S16_LE:
        return 16;
    };
}

static u32 spiaud_frames_to_bytes(const struct spi_audio_config *pConfig, unsigned int frames)
{
    return frames * pConfig->channels * (spiaud_format_to_bits(pConfig->format) >> 3);
}

static void spiaud_reset_ring_buffer(struct spiaud_ring_buffer *pRingBuf) {
    mutex_lock(&pRingBuf->bufLock);
    pRingBuf->pRead = pRingBuf->pWrite = pRingBuf->pBufBase;
    pRingBuf->bufCount = 0;
    mutex_unlock(&pRingBuf->bufLock);
}

static int spiaud_write_data_to_ring_buffer(struct spiaud_ring_buffer *pRingBuf, u8 *pBuf, u32 size)
{
    u8 *pEnd = pRingBuf->pBufBase + pRingBuf->bufLen;
    u32 size1, size2;

    if (size == 0) {
        return 0;
    }

    //print_vdbg("Write to ring buffer, pEnd=0x%x, pRingBuf->pWrite=0x%x, size=%d\n", pEnd, pRingBuf->pWrite, size);
    if (size <= (pEnd - pRingBuf->pWrite)) //copy once
    {
        memcpy(pRingBuf->pWrite, pBuf, size);
        pRingBuf->pWrite += size;

        if ((u32)pRingBuf->pWrite >= (u32)pEnd)
        {
            pRingBuf->pWrite = pRingBuf->pBufBase;
        }
    }
    else
    {
        size1 = pEnd - pRingBuf->pWrite;
        size2 = size - size1;
        memcpy(pRingBuf->pWrite, pBuf, size1);
        memcpy(pRingBuf->pBufBase, pBuf + size1, size2);
        pRingBuf->pWrite = pRingBuf->pBufBase + size2;
    }

    mutex_lock(&pRingBuf->bufLock);
    pRingBuf->bufCount = pRingBuf->bufCount + size;
    if (pRingBuf->bufCount > pRingBuf->bufLen)
    {
        print_wdbg("WARNING: Write to ring buffer overrun %d\n", pRingBuf->bufCount - pRingBuf->bufLen);
        pRingBuf->bufCount = 0;
        pRingBuf->pWrite = pRingBuf->pRead;
    }
    mutex_unlock(&pRingBuf->bufLock);

    return size;
}

static int spiaud_read_data_from_ring_buffer(struct spiaud_ring_buffer *pRingBuf, u8 *pBuf, int size)
{
    u8 *pEnd;
    u32 size1, size2;
    int ringBufCount;

    if (pRingBuf->pBufBase == NULL || pRingBuf->bufLen == 0)
    {
        print_wdbg("WARNING: Ring buffer has been release!");
        return 0;
    }

    ringBufCount = pRingBuf->bufCount;
    if (ringBufCount == 0)
    {
        return 0;
    }

    pEnd = pRingBuf->pBufBase + pRingBuf->bufLen;
    //print_vdbg("Read from ring buffer, pEnd=0x%x, pRingBuf->pRead=0x%x, size=%d\n", pEnd, pRingBuf->pRead, size);
    if (ringBufCount < size)
    {
        print_vdbg("ring buffer lack of data %d, need wait \n", size - pRingBuf->bufCount);
        size = ringBufCount;
    }

    if (size <= (pEnd - pRingBuf->pRead)) //copy once
    {
        memcpy(pBuf, pRingBuf->pRead, size);
        pRingBuf->pRead += size;
        if ((u32)pRingBuf->pRead >= (u32)pEnd)
        {
            pRingBuf->pRead = pRingBuf->pBufBase;
        }
    }
    else
    {
        size1 = pEnd - pRingBuf->pRead;
        size2 = size - size1;
        memcpy(pBuf, pRingBuf->pRead, size1);
        memcpy(pBuf + size1, pRingBuf->pBufBase, size2);
        pRingBuf->pRead = pRingBuf->pBufBase + size2;
    }

    mutex_lock(&pRingBuf->bufLock);
    pRingBuf->bufCount = pRingBuf->bufCount - size;
    if (pRingBuf->bufCount < 0) 
    {
        print_wdbg("WARNING: Ring buffer has been overrun, reset. size=%d\n", size);
        pRingBuf->bufCount = 0;
        pRingBuf->pRead = pRingBuf->pWrite;
    }
    mutex_unlock(&pRingBuf->bufLock);

    return size;
}

static int spiaud_free_transfer_buffers(struct spiaud_data *spiaud)
{
    struct spiaud_ring_buffer *pRingBuffer = &spiaud->ringBuffer;

    print_ddbg("spiaud_free_transfer_buffers()\n");
    mutex_lock(&spiaud->transferBufLock);
    if (spiaud->pTxBuffer)
    {
        kfree(spiaud->pTxBuffer);
        spiaud->pTxBuffer = NULL;
    }

    if (spiaud->pRxBuffer)
    {
        kfree(spiaud->pRxBuffer);
        spiaud->pRxBuffer = NULL;
    }
    spiaud->transferBufLen = 0;
    
    if (pRingBuffer->pBufBase)
    {
        kfree(pRingBuffer->pBufBase);
        pRingBuffer->pBufBase = NULL;
        pRingBuffer->pRead = pRingBuffer->pWrite = NULL;
        pRingBuffer->bufLen = 0;
        pRingBuffer->bufCount = 0;
    }

    mutex_unlock(&spiaud->transferBufLock);
    
    return 0;
}

static int spiaud_init_transfer_buffers(struct spiaud_data *spiaud, const struct spi_audio_config *pAudioConfig)
{
    int transferSize = spiaud_frames_to_bytes(pAudioConfig, pAudioConfig->periodSize)
                         + sizeof(struct spi_audio_header_info) + SPI_AUDIO_TAIL_MARGIN_LEN;
    int ringBufferSize = spiaud_frames_to_bytes(pAudioConfig, 
                        pAudioConfig->periodSize * pAudioConfig->periodCount);
    struct spiaud_ring_buffer *pRingBuffer = &spiaud->ringBuffer;

    print_ddbg("Init transfer buffer, transferSize=%d, ringBufferSize=%d\n", transferSize, ringBufferSize);
    spiaud->pTxBuffer = kmalloc(transferSize, GFP_KERNEL);
    spiaud->pRxBuffer = kmalloc(transferSize, GFP_KERNEL);
    pRingBuffer->pBufBase = kmalloc(ringBufferSize, GFP_KERNEL);
    spiaud->transferBufLen = transferSize;
    pRingBuffer->pRead = pRingBuffer->pWrite = pRingBuffer->pBufBase;
    pRingBuffer->bufLen = ringBufferSize;
    pRingBuffer->bufCount = 0;

    if (spiaud->pTxBuffer == NULL || spiaud->pRxBuffer == NULL || pRingBuffer->pBufBase == NULL)
    {
        print_wdbg("WARNING: Not enough memory, transferSize=%d, ringBufferSize=%d\n", transferSize, ringBufferSize);
        spiaud_free_transfer_buffers(spiaud);

        return -1;
    } 

    return 0;
}

static bool spiaud_is_valid_packet(u8 *pBuf)
{
    if (memcmp(pBuf, SPI_AUDIO_MAGIC_NUMBER, SPI_AUDIO_MAGIC_NUMBER_LENGTH) == 0)
    {
        return true;
    } else
    {
        return false;
    }
}

static int spiaud_process_recv_data(struct spiaud_data *spiaud)
{
    int errCode = 0;
    int i, sequenceNumDiff = 0;
    struct spi_audio_header_info* pHeaderInfo = (struct spi_audio_header_info *)spiaud->pRxBuffer;

    if ((pHeaderInfo->headerLength != sizeof(struct spi_audio_header_info))
        || (pHeaderInfo->totalLength < pHeaderInfo->headerLength))
    {
        print_wdbg("WARNING: receive invalid data packet! header len %d, total len %d\n",
            pHeaderInfo->headerLength, pHeaderInfo->totalLength);
        return -1;
    }

    for (i=0; i<SPI_AUDIO_CHANNEL_NUM; i++)
    {
        if ((pHeaderInfo->channelsGainDb[i] != spiaud->audioHeaderInfo.channelsGainDb[i])
            && (spiaud->audioHeaderInfo.channelsGainDb[i] <= SPI_AUDIO_CHANNEL_GAIN_MAX))
        {
            u8* pRecvChDB = pHeaderInfo->channelsGainDb;
            u8* pOrigChDB = spiaud->audioHeaderInfo.channelsGainDb;
            print_ddbg("Need update audio param: recv[%d,%d,%d,%d,%d,%d,%d,%d] vs orig[%d,%d,%d,%d,%d,%d,%d,%d]\n",
                pRecvChDB[0],pRecvChDB[1],pRecvChDB[2],pRecvChDB[3],pRecvChDB[4],pRecvChDB[5],pRecvChDB[6],pRecvChDB[7],
                pOrigChDB[0],pOrigChDB[1],pOrigChDB[2],pOrigChDB[3],pOrigChDB[4],pOrigChDB[5],pOrigChDB[6],pOrigChDB[7]);
            break;
        }
    }

    print_vdbg("spiaud_process_recv_data(), buffer len %d, total len %d, buf[0]=%d\n",
        spiaud->transferBufLen,
        pHeaderInfo->totalLength - pHeaderInfo->headerLength,
        spiaud->pRxBuffer[pHeaderInfo->headerLength]);

    if (gPeerSequenceNumber == -1) {
        gPeerSequenceNumber = pHeaderInfo->sequenceNumber;
        gPeerSequenceNumberBase = pHeaderInfo->sequenceNumber;
        print_ddbg("spiaud_process_recv_data(), context len:%d, base sequence number:%d\n",
            pHeaderInfo->totalLength - pHeaderInfo->headerLength, pHeaderInfo->sequenceNumber);
    } else {
        gPeerSequenceNumber++;
        sequenceNumDiff = pHeaderInfo->sequenceNumber - gPeerSequenceNumber;
        if (sequenceNumDiff != 0) {
            print_wdbg("WARNING: missing some raw data, expect %d but %d (base %d)\n",
                gPeerSequenceNumber, pHeaderInfo->sequenceNumber, gPeerSequenceNumberBase);
            gPeerSequenceNumber = pHeaderInfo->sequenceNumber;
            errCode = -2;
        }

        if (pHeaderInfo->overrunFlag) {
            print_wdbg("WARNING: missing some raw data, sequence number %d\n", pHeaderInfo->sequenceNumber);
            errCode = -2;
        }
    }

    print_vdbg("INFO: sequence number, expect %d recv %d\n",
                gPeerSequenceNumber, pHeaderInfo->sequenceNumber);

    spiaud_write_data_to_ring_buffer(&spiaud->ringBuffer,
        &spiaud->pRxBuffer[pHeaderInfo->headerLength],
        pHeaderInfo->totalLength - pHeaderInfo->headerLength);

    return errCode;
}

/*
 * We can't use the standard synchronous wrappers for file I/O; we
 * need to protect against async removal of the underlying spi_device.
 */
static void spiaud_complete(void *arg)
{
    print_vdbg("spiaud_complete()\n");
    complete(arg);
}

static int spiaud_sync(struct spiaud_data *spiaud, struct spi_message *message)
{
    DECLARE_COMPLETION_ONSTACK(done);
    int status;

    message->complete = spiaud_complete;
    message->context = &done;

    mutex_lock(&spiaud->spiLock);
    if (spiaud->spi == NULL)
        status = -ESHUTDOWN;
    else
        status = spi_async(spiaud->spi, message);
    mutex_unlock(&spiaud->spiLock);
    if (status == 0) {
        wait_for_completion(&done);
        status = message->status;
        if (status == 0)
            status = message->actual_length;
    }

    return status;
}

static int spiaud_sync_transfer(struct spiaud_data *spiaud)
{
    struct spi_transfer    t = {
            .rx_buf        = spiaud->pRxBuffer,
            .tx_buf        = spiaud->pTxBuffer,
            .len        = spiaud->transferBufLen,

            .speed_hz = 10000000,
        };
    struct spi_message    m;
    int bytes;

    spi_message_init(&m);
    spi_message_add_tail(&t, &m);
    bytes = spiaud_sync(spiaud, &m);
    {
        u8* tBuf = spiaud->pRxBuffer;
        struct spi_audio_header_info *Header = (struct spi_audio_header_info *)spiaud->pRxBuffer;
        print_vdbg("spiaud_sync_transfer(), send op %x, recv %02x %02x %02x %02x %02x %02x %02x %02x, op=0x%02x, len=%d\n",
                 spiaud->pTxBuffer[16],
                 tBuf[0], tBuf[1], tBuf[2], tBuf[3], tBuf[4], tBuf[5], tBuf[6], tBuf[7],
                 Header->opCode, Header->totalLength);
    }
    return bytes;
}

static void spiaud_power_saving_request(struct spiaud_data *spiaud, bool bActive)
{
    struct spi_audio_header_info* pHeaderInfo = &spiaud->audioHeaderInfo;

    //This function not ready
    if (1) return;

    mutex_lock(&spiaud->transferBufLock);
    if (bActive) {
        pHeaderInfo->opCode = SPI_AUDIO_SLAVER_OP_POWER_SAVING;
        spiaud_sync_transfer(spiaud);
    } else {
        pHeaderInfo->opCode = SPI_AUDIO_SLAVER_OP_NULL;
    }
    mutex_unlock(&spiaud->transferBufLock);
    print_ddbg("spiaud_power_saving_request(), bActive=%d\n", bActive);
}

static int spiaud_loopTranferFunc(void *pUsrdata)
{
    ktime_t k1,k2;
    s64     timeUs;
    struct spiaud_data *spiaud = (struct spiaud_data *)pUsrdata;
    bool    bValidPacket;
    int     errCode = 0;

    while(1)
    {
        k1 = ktime_get();
        print_vdbg("spiaud_loopTranferFunc begin\n");
        //wait for signal
        if (atomic_read(&gTransferEventFlag) == 0) {
            print_ddbg("spiaud_loopTranferFunc wait\n");
            wait_event_interruptible(gTransferEventWq, (atomic_read(&gTransferEventFlag) != 0));
        }

        mutex_lock(&spiaud->transferBufLock);
        if (spiaud->pTxBuffer == NULL || spiaud->pRxBuffer == NULL)
        {
            print_wdbg("transfer buffer has been released!\n");
            mutex_unlock(&spiaud->transferBufLock);
            break;
        }

        // Set the value 0xcd for debug easily
        memset(spiaud->pTxBuffer, 0xcd, sizeof(spiaud->audioHeaderInfo) + 256);
        memcpy(spiaud->pTxBuffer, &spiaud->audioHeaderInfo, sizeof(spiaud->audioHeaderInfo));
        spiaud->audioHeaderInfo.sequenceNumber++;
        /* For Test { */
        //spiaud->audioHeaderInfo.totalLength = sizeof(spiaud->audioHeaderInfo) + 256;
        //memcpy(&spiaud->pTxBuffer[sizeof(spiaud->audioHeaderInfo)], "Hello World", sizeof("Hello World"));
        /* } For Test */
        spiaud_sync_transfer(spiaud);

        bValidPacket = spiaud_is_valid_packet(spiaud->pRxBuffer);
        if (bValidPacket)
        {
            errCode = spiaud_process_recv_data(spiaud);
            wake_up(&gTransferEventRq);
        }
        mutex_unlock(&spiaud->transferBufLock);

        /*
        if (bValidPacket) {
            // Can sleep a while for optimizing the system perforamce
            msleep(0);
        }
        */
        msleep(0);
        
        k2 = ktime_get();
        k2 = ktime_sub(k2, k1);
        timeUs = ktime_to_us(k2);
        gTimeUsArray[gTimeIndex % sizeof(gTimeUsArray)] = timeUs;
        gTimeIndex++;
        print_vdbg("spiaud_loopTranferFunc loop, cost:%lldus\n", timeUs);
        if (errCode) {
            // Some data miss
            print_wdbg("SPI transfer, cur=%d cost(us)[%lld,%lld,%lld,%lld,%lld,%lld,%lld,%lld,%lld,%lld]\n",
                gTimeIndex-1,
                gTimeUsArray[0], gTimeUsArray[1], gTimeUsArray[2], gTimeUsArray[3], gTimeUsArray[4],
                gTimeUsArray[5], gTimeUsArray[6], gTimeUsArray[7], gTimeUsArray[8], gTimeUsArray[9]);
        }
    }
    return 0;
}

/*-------------------------------------------------------------------------*/

/* Read-only message with current device setup */
static int
spiaud_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
    struct spiaud_data    *spiaud = filp->private_data;
    int            periodSizeToBytes = spiaud_frames_to_bytes(&gSpiAudioConfig, gSpiAudioConfig.periodSize);
    int            readBytes, bytes;
    int            retryCountMax = count/periodSizeToBytes + 1;
    int            retryCount = 0;

    print_vdbg("spiaud_read()\n");
    
    if (atomic_read(&gTransferEventFlag) == 0)
    {
        print_wdbg("ERROR: illegal state!\n");
        return -EFAULT;
    }

    if (count > spiaud->userCopyBufLen)
    {
        if (spiaud->pUserCopyBuf != NULL)
        {
            kfree(spiaud->pUserCopyBuf);
            spiaud->userCopyBufLen = 0;
        }
        
        spiaud->pUserCopyBuf = kmalloc(count, GFP_KERNEL);
        if (spiaud->pUserCopyBuf == NULL)
        {
            print_wdbg("WARNING: Not enough memory, count=%d\n", count);
            return -EMSGSIZE;
        }
        spiaud->userCopyBufLen = count;
    }

    readBytes = 0;
    while ((atomic_read(&gTransferEventFlag) != 0) && (retryCount < retryCountMax))
    {
        bytes = spiaud_read_data_from_ring_buffer(&spiaud->ringBuffer,
                        &spiaud->pUserCopyBuf[readBytes], count - readBytes);
        readBytes = readBytes + bytes;
        if (readBytes >= count)
        {
            break;
        }
        else
        {
            print_vdbg("lack data(%d/%d), waiting... retryCount(%d)\n", readBytes, count, retryCount);
            wait_event_interruptible_timeout(gTransferEventRq, spiaud->ringBuffer.bufCount >= periodSizeToBytes, HZ);
            retryCount++;
        }
    }

    if (readBytes > 0) {
        unsigned long    missing;

        missing = copy_to_user(buf, spiaud->pUserCopyBuf, readBytes);
        if (missing != 0)
        {
            print_wdbg("ERROR: copy to user missing %ld bytes\n", missing);
            readBytes = -EFAULT;
        }
    }

    print_vdbg("spiaud_read(), readBytes=%d, count=%d\n", readBytes, count);

    return readBytes;
}

/* Write-only message with current device setup */
static int
spiaud_write(struct file *filp, const char __user *buf,
        size_t count, loff_t *f_pos)
{
    print_wdbg("spiaud_write(), not implement!\n");
    return 0;
}

static long
spiaud_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    int            err = 0;
    int            retval = 0;
    int         i;
    struct spiaud_data    *spiaud;
    struct spi_device    *spi;
    u32            tmp;
    unsigned        n_ioc;
    struct spi_ioc_transfer    *ioc;
    struct spi_audio_channels_gain  channelsGain;

    /* Check type and command number */
    if (_IOC_TYPE(cmd) != SPI_IOC_MAGIC)
        return -ENOTTY;

    /* Check access direction once here; don't repeat below.
     * IOC_DIR is from the user perspective, while access_ok is
     * from the kernel perspective; so they look reversed.
     */
    if (_IOC_DIR(cmd) & _IOC_READ)
        err = !access_ok(VERIFY_WRITE,
                (void __user *)arg, _IOC_SIZE(cmd));
    if (err == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
        err = !access_ok(VERIFY_READ,
                (void __user *)arg, _IOC_SIZE(cmd));
    if (err)
        return -EFAULT;

    /* guard against device removal before, or while,
     * we issue this ioctl.
     */
    spiaud = filp->private_data;
    mutex_lock(&spiaud->spiLock);
    spi = spi_dev_get(spiaud->spi);
    mutex_unlock(&spiaud->spiLock);

    print_wdbg("spiaud_ioctl(), cmd=%d\n", cmd);

    if (spi == NULL)
        return -ESHUTDOWN;

    switch (cmd) {
    /* read requests */
    case SPI_IOC_RD_MODE:
        retval = __put_user(spi->mode & SPI_MODE_MASK,
                    (__u8 __user *)arg);
        break;
    case SPI_IOC_RD_LSB_FIRST:
        retval = __put_user((spi->mode & SPI_LSB_FIRST) ?  1 : 0,
                    (__u8 __user *)arg);
        break;
    case SPI_IOC_RD_BITS_PER_WORD:
        retval = __put_user(spi->bits_per_word, (__u8 __user *)arg);
        break;
    case SPI_IOC_RD_MAX_SPEED_HZ:
        retval = __put_user(spi->max_speed_hz, (__u32 __user *)arg);
        break;

    /* write requests */
    case SPI_IOC_WR_MODE:
        retval = __get_user(tmp, (u8 __user *)arg);
        if (retval == 0) {
            u8    save = spi->mode;

            if (tmp & ~SPI_MODE_MASK) {
                retval = -EINVAL;
                break;
            }

            tmp |= spi->mode & ~SPI_MODE_MASK;
            spi->mode = (u8)tmp;
            retval = spi_setup(spi);
            if (retval < 0)
                spi->mode = save;
            else
                print_ddbg("spi mode %02x\n", tmp);
        }
        break;
    case SPI_IOC_WR_LSB_FIRST:
        retval = __get_user(tmp, (__u8 __user *)arg);
        if (retval == 0) {
            u8    save = spi->mode;

            if (tmp)
                spi->mode |= SPI_LSB_FIRST;
            else
                spi->mode &= ~SPI_LSB_FIRST;
            retval = spi_setup(spi);
            if (retval < 0)
                spi->mode = save;
            else
                print_ddbg("%csb first\n", tmp ? 'l' : 'm');
        }
        break;
    case SPI_IOC_WR_BITS_PER_WORD:
        retval = __get_user(tmp, (__u8 __user *)arg);
        if (retval == 0) {
            u8    save = spi->bits_per_word;

            spi->bits_per_word = tmp;
            retval = spi_setup(spi);
            if (retval < 0)
                spi->bits_per_word = save;
            else
                print_ddbg("%d bits per word\n", tmp);
        }
        break;
    case SPI_IOC_WR_MAX_SPEED_HZ:
        retval = __get_user(tmp, (__u32 __user *)arg);
        if (retval == 0) {
            u32    save = spi->max_speed_hz;

            spi->max_speed_hz = tmp;
            retval = spi_setup(spi);
            if (retval < 0)
                spi->max_speed_hz = save;
            else
                print_ddbg("%d Hz (max)\n", tmp);
        }
        break;
    case SPI_IOC_WR_INIT:
        break;
    case SPI_IOC_WR_CHANNELS_GAIN:
        if (__copy_from_user(&channelsGain, (void __user *)arg, sizeof(struct spi_audio_channels_gain)))
        {
            retval = -EFAULT;
            break;
        }
        print_ddbg("Set SPI slaver audio codec channels gain: [%d %d %d %d %d %d %d %d]\n",
            channelsGain.gainDb[0], channelsGain.gainDb[1], channelsGain.gainDb[2], channelsGain.gainDb[3],
            channelsGain.gainDb[4], channelsGain.gainDb[5], channelsGain.gainDb[6], channelsGain.gainDb[7]);
        for (i=0; i<SPI_AUDIO_CHANNEL_NUM; i++)
        {
            if (SPI_AUDIO_CHANNEL_GAIN_MAX < channelsGain.gainDb[i])
            {
                retval = -EINVAL;
            }
        }
        if (retval)
        {
            print_wdbg("ERROR: channel gain not allowed over than %d(db)", SPI_AUDIO_CHANNEL_GAIN_MAX);
            break;
        }

        memcpy(spiaud->audioHeaderInfo.channelsGainDb, channelsGain.gainDb,
                 sizeof(struct spi_audio_channels_gain));
        // Will update the param at next transfer period
        print_ddbg("Setting SPI slaver audio codec channels gain Done\n");
        break;
    case SPI_IOC_WR_EM_MODE:
        retval = __get_user(tmp, (__u8 __user *)arg);
        if (retval == 0) {
            spiaud->audioHeaderInfo.testMode = tmp;
            print_ddbg("Setting SPI slaver EM mode, value=%d\n", tmp);
        }
        break;
    default:
        break;
    }

    spi_dev_put(spi);
    return retval;
}

#ifdef CONFIG_COMPAT
static long
spiaud_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    return spiaud_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#else
#define spiaud_compat_ioctl NULL
#endif /* CONFIG_COMPAT */

static int spiaud_open(struct inode *inode, struct file *filp)
{
    struct spiaud_data    *spiaud;
    int            status = -ENXIO;

    mutex_lock(&gDeviceListLock);

    list_for_each_entry(spiaud, &gDeviceList, deviceEntry)
    {
        if (spiaud->devt == inode->i_rdev)
        {
            status = 0;
            break;
        }
    }

    print_ddbg("spiaud_open() status=%d, user=%d\n", status, spiaud->users);
    if (status == 0) {
        spiaud->users++;
        filp->private_data = spiaud;
        nonseekable_open(inode, filp);
    } else
        print_ddbg("spiaud: nothing for minor %d\n", iminor(inode));

    gPeerSequenceNumber = -1;
    spiaud_reset_ring_buffer(&spiaud->ringBuffer);
    spiaud_power_saving_request(spiaud, false);

    atomic_set(&gTransferEventFlag, 1);
    wake_up(&gTransferEventWq);

    mutex_unlock(&gDeviceListLock);

    return status;
}

static int spiaud_release(struct inode *inode, struct file *filp)
{
    struct spiaud_data    *spiaud;
    int status = 0;

    mutex_lock(&gDeviceListLock);
    spiaud = filp->private_data;
    filp->private_data = NULL;

    atomic_set(&gTransferEventFlag, 0);

    print_ddbg("spiaud_release() user=%d\n", spiaud->users);
    print_ddbg("SPI transfer, cur=%d cost(us)[%lld,%lld,%lld,%lld,%lld,%lld,%lld,%lld,%lld,%lld]\n",
                gTimeIndex-1,
                gTimeUsArray[0], gTimeUsArray[1], gTimeUsArray[2], gTimeUsArray[3], gTimeUsArray[4],
                gTimeUsArray[5], gTimeUsArray[6], gTimeUsArray[7], gTimeUsArray[8], gTimeUsArray[9]);
    /* last close? */
    spiaud->users--;
    if (spiaud->users == 0) {
        int        dofree;

        gTimeIndex = 0;
        spiaud_power_saving_request(spiaud, true);

        /* ... after we unbound from the underlying device? */
        mutex_lock(&spiaud->spiLock);
        dofree = (spiaud->spi == NULL);
        mutex_unlock(&spiaud->spiLock);

        if (dofree)
            kfree(spiaud);
    }
    mutex_unlock(&gDeviceListLock);
    
    return status;
}

static int spiaud_proc_version_read(struct file *file, char __user *buffer,
                   size_t count, loff_t * offset) {
    char tmpBuf[50];
    int  len;
    loff_t pos;

    pos = *offset;
    if (pos < 0 || (long) pos != pos || (ssize_t) count < 0) {
        return -EIO;
    }

    if (pos > 0) {
        //Read END
        return 0;
    }

    len = sprintf(tmpBuf, "version:%x\n", SPI_AUDIO_VERSION);

    if (count >= len) {
        unsigned long    missing;

        missing = copy_to_user(buffer, tmpBuf, len);
        if (missing != 0)
        {
            print_wdbg("ERROR: copy to user missing %ld bytes\n", missing);
            len = -EFAULT;
        }
    } else {
        len = -EFAULT;
    }

    if (len > 0)  *offset = pos + len;
    return len;
}

static const struct file_operations spiaud_version_entry_operations =
{
    .owner = THIS_MODULE,
    .read = spiaud_proc_version_read,
};

static int spiaud_proc_version_init(struct proc_dir_entry* proc_root) {
    struct proc_dir_entry *p;

    p = proc_create_data("version", 0444, proc_root, &spiaud_version_entry_operations, NULL);
    if(p == NULL) {
        printk("create proc error!\n");
        return -ENOMEM;;
    }

    return 0;
}

static int spiaud_proc_gain_open(struct inode *inode, struct file *file)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
    file->private_data = PDE_DATA(inode);
#else
    struct proc_dir_entry* entry = PDE(inode);
    file->private_data = entry->data;
#endif

    if (file->private_data == NULL) {
        print_wdbg("ERROR: proc gains without private data!\n");
        return -ENODEV;
    }
 
    return 0;
}

static int spiaud_proc_gain_read(struct file *file, char __user *buffer,
                   size_t count, loff_t * offset) {
    struct spiaud_data *spiaud = (struct spiaud_data *)file->private_data;
    char tmpBuf[100];
    int  len;
    u8* gains;
    loff_t pos;

    pos = *offset;
    if (pos < 0 || (long) pos != pos || (ssize_t) count < 0) {
        return -EIO;
    }

    if (pos > 0) {
        //Read END
        return 0;
    }

    gains = spiaud->audioHeaderInfo.channelsGainDb;
    len = sprintf(tmpBuf, "mic gain[%d,%d,%d,%d,%d,%d,%d,%d]\n", gains[0], gains[1], gains[2], gains[3],
                gains[4], gains[5], gains[6], gains[7]);
    if (count >= len) {
        unsigned long    missing;

        missing = copy_to_user(buffer, tmpBuf, len);
        if (missing != 0)
        {
            print_wdbg("ERROR: copy to user missing %ld bytes\n", missing);
            len = -EFAULT;
        }
    } else {
        len = -EFAULT;
    }

    if (len > 0)  *offset = pos + len;
    return len;
}

static int spiaud_proc_gain_write(struct file *file, const char __user *buffer,
                    size_t count, loff_t * offset) {
    struct spiaud_data *spiaud = (struct spiaud_data *)file->private_data;
    char pathBuf[5];
    int  i, len;
    int  gain = 0;

    len = count;
    if (count > sizeof(pathBuf)) {
        len = sizeof(pathBuf);
    }

    if(copy_from_user(pathBuf, buffer, len)) {
        printk("copy from user fail\n");
        return -EFAULT;
    }

    for (i=0; i<len; i++) {
        char ch = pathBuf[i];
        if(ch>='0' && ch<='9') {
            gain = gain * 10 + (ch-'0');
        }
    }

    if (gain > SPI_AUDIO_CHANNEL_GAIN_MAX) {
        printk("gain %d is illegal!\n", gain);
        return -EFAULT;
    }

    for (i=0; i < SPI_AUDIO_CHANNEL_NUM; i++) {
        spiaud->audioHeaderInfo.channelsGainDb[i] = (u8)gain;
    }

    return len;
}

static const struct file_operations spiaud_gain_entry_operations =
{
    .owner = THIS_MODULE,
    .open = spiaud_proc_gain_open,
    .read = spiaud_proc_gain_read,
    .write = spiaud_proc_gain_write,
};

static int spiaud_proc_gain_init(struct proc_dir_entry* proc_root, struct spiaud_data* private_data) {
    struct proc_dir_entry *p;

    p = proc_create_data("gains", 0664, proc_root, &spiaud_gain_entry_operations, private_data);
    if(p == NULL) {
        printk("create proc error!\n");
        return -ENOMEM;;
    }

    return 0;
}

static int spiaud_proc_debug_read(struct file *file, char __user *buffer,
                   size_t count, loff_t * offset) {
    char tmpBuf[50];
    int  len;
    loff_t pos;

    pos = *offset;
    if (pos < 0 || (long) pos != pos || (ssize_t) count < 0) {
        return -EIO;
    }

    if (pos > 0) {
        //Read END
        return 0;
    }

    len = sprintf(tmpBuf, "debug mask:0x%x\n", gDebugMask);
    if (count >= len) {
        unsigned long    missing;

        missing = copy_to_user(buffer, tmpBuf, len);
        if (missing != 0)
        {
            print_wdbg("ERROR: copy to user missing %ld bytes\n", missing);
            len = -EFAULT;
        }
    } else {
        len = -EFAULT;
    }

    if (len > 0)  *offset = pos + len;
    return len;
}

static int spiaud_proc_debug_write(struct file *file, const char __user *buffer,
                    size_t count, loff_t * offset) {
    char tmpBuf[5];
    int  i, len;
    int  mask = 0;

    len = count;
    if (count > sizeof(tmpBuf)) {
        len = sizeof(tmpBuf);
    }

    if(copy_from_user(tmpBuf, buffer, len)) {
        printk("copy from user fail\n");
        return -EFAULT;
    }

    for (i=0; i<len; i++) {
        char ch = tmpBuf[i];
        if(ch>='0' && ch<='9') {
            mask = mask * 10 + (ch-'0');
        }
    }

    gDebugMask = mask;

    return len;
}

static const struct file_operations spiaud_debug_entry_operations =
{
    .owner = THIS_MODULE,
    .read = spiaud_proc_debug_read,
    .write = spiaud_proc_debug_write,
};

static int spiaud_proc_debug_init(struct proc_dir_entry* proc_root) {
    struct proc_dir_entry *p;

    p = proc_create_data("debug", 0664, proc_root, &spiaud_debug_entry_operations, NULL);
    if(p == NULL) {
        printk("create proc error!\n");
        return -ENOMEM;;
    }

    return 0;
}

static const struct file_operations spiaud_fops = {
    .owner =    THIS_MODULE,
    /* REVISIT switch to aio primitives, so that userspace
     * gets more complete API coverage.  It'll simplify things
     * too, except for the locking.
     */
    .write = spiaud_write,
    .read = spiaud_read,
    .unlocked_ioctl = spiaud_ioctl,
    .compat_ioctl = spiaud_compat_ioctl,
    .open = spiaud_open,
    .release = spiaud_release,
    .llseek = no_llseek,
};

/*-------------------------------------------------------------------------*/

/* The main reason to have this class is to make mdev/udev create the
 * /dev/spidevB.C character device nodes exposing our userspace API.
 * It also simplifies memory management.
 */

static struct class *spiaud_class;

static int spiaud_probe(struct spi_device *spi)
{
    struct spiaud_data *spiaud;
    int status;
    unsigned long minor;
    struct task_struct *pTransferEventThread = NULL;
    struct proc_dir_entry *p;

    if(!spi)    
    return -ENOMEM;

    spi->bits_per_word = 8;
    spi->mode = SPI_MODE_0; /* SCPOL = 0, SCPH = 0 */
    //spi->mode = spi->mode | SPI_LSB_FIRST;

    /* Allocate driver data */
    spiaud = kzalloc(sizeof(*spiaud), GFP_KERNEL);
    if (!spiaud)
        return -ENOMEM;

    /* Initialize the driver data */
    spiaud->spi = spi;
    mutex_init(&spiaud->spiLock);
    mutex_init(&spiaud->transferBufLock);
    mutex_init(&spiaud->ringBuffer.bufLock);

    INIT_LIST_HEAD(&spiaud->deviceEntry);

    /* If we can allocate a minor number, hook up this device.
     * Reusing minors is fine so long as udev or mdev is working.
     */
    mutex_lock(&gDeviceListLock);
    minor = find_first_zero_bit(minors, N_SPI_MINORS);
    if (minor < N_SPI_MINORS) {
        struct device *dev;

        spiaud->devt = MKDEV(SPIDEV_MAJOR, minor);

        dev = device_create(spiaud_class, &spi->dev, spiaud->devt,
                    spiaud, "spiaud");

        status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
    } else {
        print_wdbg( "no minor number available!\n");
        status = -ENODEV;
    }
    if (status == 0) {
        set_bit(minor, minors);
        list_add(&spiaud->deviceEntry, &gDeviceList);
    }
    mutex_unlock(&gDeviceListLock);

    spiaud_init_transfer_buffers(spiaud, &gSpiAudioConfig);
    memcpy(spiaud->audioHeaderInfo.magicNumber, SPI_AUDIO_MAGIC_NUMBER, SPI_AUDIO_MAGIC_NUMBER_LENGTH);
    spiaud->audioHeaderInfo.version = SPI_AUDIO_VERSION;
    spiaud->audioHeaderInfo.headerLength = sizeof(spiaud->audioHeaderInfo);
    spiaud->audioHeaderInfo.totalLength = sizeof(spiaud->audioHeaderInfo);
    spiaud->audioHeaderInfo.sequenceNumber = 0;
    spiaud->audioHeaderInfo.opCode = SPI_AUDIO_SLAVER_OP_NULL;
    memset(spiaud->audioHeaderInfo.channelsGainDb, 0xff, SPI_AUDIO_CHANNEL_NUM);

    init_waitqueue_head(&gTransferEventWq);
    init_waitqueue_head(&gTransferEventRq);
    //start transfer event thread
    pTransferEventThread = kthread_run(spiaud_loopTranferFunc, spiaud, "spiaud_transfer");
    if (IS_ERR(pTransferEventThread)) 
    { 
        status = PTR_ERR(pTransferEventThread);
        print_wdbg( " failed to create kernel thread: %d\n", status);
    }

    if (status == 0) {
        spi_set_drvdata(spi, spiaud);

        p = proc_mkdir("spiaud", NULL);
        if (p == NULL) {
            return -ENOMEM;
        }
        spiaud_proc_version_init(p);
        spiaud_proc_gain_init(p, spiaud);
        spiaud_proc_debug_init(p);
    }
    else {
        kfree(spiaud);
    }
    
    return status;
}

static int spiaud_remove(struct spi_device *spi)
{
    struct spiaud_data    *spiaud = spi_get_drvdata(spi);

    print_ddbg("spiaud_remove()\n");

    /* make sure ops on existing fds can abort cleanly */
    mutex_lock(&spiaud->spiLock);
    spiaud->spi = NULL;
    spi_set_drvdata(spi, NULL);
    mutex_unlock(&spiaud->spiLock);

    /* prevent new opens */
    mutex_lock(&gDeviceListLock);
    list_del(&spiaud->deviceEntry);
    device_destroy(spiaud_class, spiaud->devt);
    clear_bit(MINOR(spiaud->devt), minors);

    spiaud_free_transfer_buffers(spiaud);
    if (spiaud->users == 0)
        kfree(spiaud);
    mutex_unlock(&gDeviceListLock);

    remove_proc_entry("spiaud", NULL);

    return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id spi_audio_dt_match[] = {
    { .compatible = "aispeech,spi_audio", },
    {},
};
MODULE_DEVICE_TABLE(of, spi_audio_dt_match);
#endif

static struct spi_driver spiaud_spi_driver = {
    .driver = {
        .name = "spi_audio",
        .owner = THIS_MODULE,
        #ifdef CONFIG_OF
        .of_match_table = of_match_ptr(spi_audio_dt_match),
        #endif
    },
    .probe =    spiaud_probe,
    .remove =     spiaud_remove,

    /* NOTE:  suspend/resume methods are not necessary here.
     * We don't do anything except pass the requests to/from
     * the underlying controller.  The refrigerator handles
     * most issues; the controller driver handles the rest.
     */
};

/*-------------------------------------------------------------------------*/
static int __init spiaud_init(void)
{
    int status;

    /* Claim our 256 reserved device numbers.  Then register a class
     * that will key udev/mdev to add/remove /dev nodes.  Last, register
     * the driver which manages those device numbers.
     */
    BUILD_BUG_ON(N_SPI_MINORS > 256);
    status = register_chrdev(SPIDEV_MAJOR, "spi", &spiaud_fops);
    if (status < 0)
        return status;

    spiaud_class = class_create(THIS_MODULE, "spi_audio");
    if (IS_ERR(spiaud_class)) {
        unregister_chrdev(SPIDEV_MAJOR, spiaud_spi_driver.driver.name);
        return PTR_ERR(spiaud_class);
    }

    status = spi_register_driver(&spiaud_spi_driver);
    if (status < 0) {
        class_destroy(spiaud_class);
        unregister_chrdev(SPIDEV_MAJOR, spiaud_spi_driver.driver.name);
    }

    return status;
}
module_init(spiaud_init);

static void __exit spiaud_exit(void)
{
    spi_unregister_driver(&spiaud_spi_driver);
    class_destroy(spiaud_class);
    unregister_chrdev(SPIDEV_MAJOR, spiaud_spi_driver.driver.name);
}
module_exit(spiaud_exit);

MODULE_AUTHOR("AISpeech: Loso");
MODULE_LICENSE("GPL");
//MODULE_ALIAS("spi:spidaud");
