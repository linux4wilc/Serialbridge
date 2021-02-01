
#ifndef WILC_SBRIDGE_H
#define WILC_SBRIDGE_H

#include <linux/types.h>

enum wilc_chip_type {
	WILC_1000,
	WILC_3000,
};

struct wilc {
	struct device *dev;
	enum wilc_chip_type chip;
	struct mutex hif_cs;
};

struct sdio_cmd52 {
	u32 read_write:		1;
	u32 function:		3;
	u32 raw:		1;
	u32 address:		17;
	u32 data:		8;
};

struct sdio_cmd53 {
	u32 read_write:		1;
	u32 function:		3;
	u32 block_mode:		1;
	u32 increment:		1;
	u32 address:		17;
	u32 count:		9;
	u8 *buffer;
	u32 block_size;
};


static inline bool is_wilc1000(u32 id)
{
	return ((id & 0xfffff000) == 0x100000 ? true : false);
}

static inline bool is_wilc3000(u32 id)
{
	return ((id & 0xfffff000) == 0x300000 ? true : false);
}

#define WILC_SPI_REG_BASE		0xe800
#define WILC_SPI_CTL			WILC_SPI_REG_BASE
#define WILC_SPI_PROTOCOL_CONFIG	(WILC_SPI_REG_BASE + 0x24)

#define WILC_SPI_PROTOCOL_OFFSET	(WILC_SPI_PROTOCOL_CONFIG - \
					 WILC_SPI_REG_BASE)

#define MODALIAS		"WILC_SPI"

#endif /** WILC_SBRIDGE_H */

