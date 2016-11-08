/* drivers/spi/aispeech_spi_audio.h
 *
 * Copyright (C) 2016 AISpeech, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef __DRIVERS_SPI_AUDIO_HEADER_H
#define __DRIVERS_SPI_AUDIO_HEADER_H

#include <linux/types.h>

#define SPI_AUDIO_VERSION	2

#define SPI_AUDIO_SLAVER_OP_NULL     		0
#define SPI_AUDIO_SLAVER_OP_POWER_SAVING 	2
#define SPI_AUDIO_TEST_MODE_FLAG     	'T'

#define SPI_AUDIO_MAGIC_NUMBER_LENGTH 	8
#define SPI_AUDIO_MAGIC_NUMBER           "AISPEECH"
#define SPI_AUDIO_CHANNEL_NUM  		8
#define SPI_AUDIO_CHANNEL_GAIN_MAX 	80

/* Bit formats */
enum spi_audio_format {
    SPI_AUDIO_FORMAT_S16_LE = 0,
    SPI_AUDIO_FORMAT_S32_LE,
    SPI_AUDIO_FORMAT_S8,
    SPI_AUDIO_FORMAT_S24_LE,

    SPI_AUDIO_FORMAT_MAX,
};

/* Configuration for a stream */
struct spi_audio_config {
    unsigned int channels;
    unsigned int rate;
    unsigned int periodSize;
    unsigned int periodCount;
    enum spi_audio_format format;
};

/* Channels gain */
struct spi_audio_channels_gain {
    u8 gainDb[SPI_AUDIO_CHANNEL_NUM];
};

struct spi_audio_header_info {
	u8 magicNumber[SPI_AUDIO_MAGIC_NUMBER_LENGTH];
	u8 version;
	u8 headerLength;
	u16 totalLength;
	u32 sequenceNumber;
	u8 opCode; /* 0 - nothing to do; 1 - modify audio codec gain; 2 - request power saving */
	u8 overrunFlag;
	u8 testMode;
	u8 dummy[5];
	u8 channelsGainDb[SPI_AUDIO_CHANNEL_NUM];
};

/* User space versions of kernel symbols for SPI clocking modes,
 * matching <linux/spi/spi.h>
 */

#define SPI_CPHA		0x01
#define SPI_CPOL		0x02

#define SPI_MODE_0		(0|0)
#define SPI_MODE_1		(0|SPI_CPHA)
#define SPI_MODE_2		(SPI_CPOL|0)
#define SPI_MODE_3		(SPI_CPOL|SPI_CPHA)

#define SPI_CS_HIGH		0x04
#define SPI_LSB_FIRST		0x08
#define SPI_3WIRE		0x10
#define SPI_LOOP		0x20
#define SPI_NO_CS		0x40
#define SPI_READY		0x80

/*---------------------------------------------------------------------------*/

/* IOCTL commands */

#define SPI_IOC_MAGIC			'k'

/**
 * struct spi_ioc_transfer - describes a single SPI transfer
 * @tx_buf: Holds pointer to userspace buffer with transmit data, or null.
 *	If no data is provided, zeroes are shifted out.
 * @rx_buf: Holds pointer to userspace buffer for receive data, or null.
 * @len: Length of tx and rx buffers, in bytes.
 * @speed_hz: Temporary override of the device's bitrate.
 * @bits_per_word: Temporary override of the device's wordsize.
 * @delay_usecs: If nonzero, how long to delay after the last bit transfer
 *	before optionally deselecting the device before the next transfer.
 * @cs_change: True to deselect device before starting the next transfer.
 *
 * This structure is mapped directly to the kernel spi_transfer structure;
 * the fields have the same meanings, except of course that the pointers
 * are in a different address space (and may be of different sizes in some
 * cases, such as 32-bit i386 userspace over a 64-bit x86_64 kernel).
 * Zero-initialize the structure, including currently unused fields, to
 * accommodate potential future updates.
 *
 * SPI_IOC_MESSAGE gives userspace the equivalent of kernel spi_sync().
 * Pass it an array of related transfers, they'll execute together.
 * Each transfer may be half duplex (either direction) or full duplex.
 *
 *	struct spi_ioc_transfer mesg[4];
 *	...
 *	status = ioctl(fd, SPI_IOC_MESSAGE(4), mesg);
 *
 * So for example one transfer might send a nine bit command (right aligned
 * in a 16-bit word), the next could read a block of 8-bit data before
 * terminating that command by temporarily deselecting the chip; the next
 * could send a different nine bit command (re-selecting the chip), and the
 * last transfer might write some register values.
 */
struct spi_ioc_transfer {
	__u64		tx_buf;
	__u64		rx_buf;

	__u32		len;
	__u32		speed_hz;

	__u16		delay_usecs;
	__u8		bits_per_word;
	__u8		cs_change;
	__u32		pad;

	/* If the contents of 'struct spi_ioc_transfer' ever change
	 * incompatibly, then the ioctl number (currently 0) must change;
	 * ioctls with constant size fields get a bit more in the way of
	 * error checking than ones (like this) where that field varies.
	 *
	 * NOTE: struct layout is the same in 64bit and 32bit userspace.
	 */
};

/* not all platforms use <asm-generic/ioctl.h> or _IOC_TYPECHECK() ... */
#define SPI_MSGSIZE(N) \
	((((N)*(sizeof (struct spi_ioc_transfer))) < (1 << _IOC_SIZEBITS)) \
		? ((N)*(sizeof (struct spi_ioc_transfer))) : 0)
#define SPI_IOC_MESSAGE(N) _IOW(SPI_IOC_MAGIC, 0, char[SPI_MSGSIZE(N)])


/* Read / Write of SPI mode (SPI_MODE_0..SPI_MODE_3) */
#define SPI_IOC_RD_MODE			_IOR(SPI_IOC_MAGIC, 1, __u8)
#define SPI_IOC_WR_MODE			_IOW(SPI_IOC_MAGIC, 1, __u8)

/* Read / Write SPI bit justification */
#define SPI_IOC_RD_LSB_FIRST		_IOR(SPI_IOC_MAGIC, 2, __u8)
#define SPI_IOC_WR_LSB_FIRST		_IOW(SPI_IOC_MAGIC, 2, __u8)

/* Read / Write SPI device word length (1..N) */
#define SPI_IOC_RD_BITS_PER_WORD	_IOR(SPI_IOC_MAGIC, 3, __u8)
#define SPI_IOC_WR_BITS_PER_WORD	_IOW(SPI_IOC_MAGIC, 3, __u8)

/* Read / Write SPI device default max speed hz */
#define SPI_IOC_RD_MAX_SPEED_HZ		_IOR(SPI_IOC_MAGIC, 4, __u32)
#define SPI_IOC_WR_MAX_SPEED_HZ		_IOW(SPI_IOC_MAGIC, 4, __u32)

/* Must set the config before getting the audio stream  */
#define SPI_IOC_WR_INIT				_IOW(SPI_IOC_MAGIC, 5, struct spi_audio_config *)

/* Set/Reset the audio codec paramters to the SPI slaver */
#define SPI_IOC_WR_CHANNELS_GAIN	_IOW(SPI_IOC_MAGIC, 6, struct spi_audio_channels_gain *)

/* Set/Reset EM mode to the SPI slaver */
#define SPI_IOC_WR_EM_MODE			_IOW(SPI_IOC_MAGIC, 7, __u8)

#endif /* __DRIVERS_SPI_AUDIO_HEADER_H */
