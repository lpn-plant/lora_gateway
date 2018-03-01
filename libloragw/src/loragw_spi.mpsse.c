/*
  Copyright LPN Plant 2018

  Maintainer: Kamil Wcislo <kamil.wcislo@lpnplant.io>
*/


/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include <stdint.h>        /* C99 types */
#include <stdio.h>        /* printf fprintf */
#include <string.h>        /* memset */

#include "mpsse.h"

#include "loragw_spi.h"
#include "loragw_hal.h"

/*  PRIVATE MACROS  */

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define TRUE       1

#if DEBUG_SPI == 1
    #define DEBUG_MSG(str)                fprintf(stderr, str)
    #define DEBUG_PRINTF(fmt, args...)    fprintf(stderr,"%s:%d: "fmt, __FUNCTION__, __LINE__, args)
    #define CHECK_NULL(a)                if((a)==NULL){fprintf(stderr,"%s:%d: ERROR: NULL POINTER AS ARGUMENT\n", __FUNCTION__, __LINE__);return LGW_SPI_ERROR;}
#else
    #define DEBUG_MSG(str)
    #define DEBUG_PRINTF(fmt, args...)
    #define CHECK_NULL(a)                if((a)==NULL){return LGW_SPI_ERROR;}
#endif

#define CHECK_ERROR(a)                   if(MPSSE_OK!=(a)){DEBUG_PRINTF("ERROR: SPI FAILURE: %s\n",ErrorString(spi_target));return LGW_SPI_ERROR;}

/*  PRIVATE CONSTANTS  */

#define READ_ACCESS     0x00
#define WRITE_ACCESS    0x80
#define SPI_SPEED       TEN_MHZ

/*  PUBLIC FUNCTIONS DEFINITION  */

/* SPI initialization and configuration */
int lgw_spi_open(void **spi_target_ptr) {
    struct mpsse_context *spi_device = NULL;

    /* check input variables */
    CHECK_NULL(spi_target_ptr); /* cannot be null, must point on a void pointer
                                   (*spi_target_ptr can be null) */

    /* initialize MPSSE lib */
    spi_device = MPSSE(SPI0, SPI_SPEED, MSB);

    if (NULL == spi_device || (!spi_device->open)) {
        DEBUG_MSG("ERROR: failed to open SPI device\n");
        return LGW_SPI_ERROR;
    }

    SetCSIdle(spi_device, HIGH);
    FlushAfterRead(spi_device, TRUE);

    *spi_target_ptr = (void *)spi_device;

    DEBUG_PRINTF("Note: SPI port opened and configured ok:\n\t- %s initialized at %dHZ\n",
        GetDescription(spi_device), GetClock(spi_device));
    return LGW_SPI_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* SPI release */
int lgw_spi_close(void *spi_target) {
    struct mpsse_context *spi_device;

    CHECK_NULL(spi_target);

    spi_device = (struct mpsse_context *)spi_target;
    Close(spi_device);

    DEBUG_MSG("Note: SPI port closed\n");
    return LGW_SPI_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Simple write */
int lgw_spi_w(void *spi_target, uint8_t spi_mux_mode, uint8_t spi_mux_target, uint8_t address, uint8_t data) {
    struct mpsse_context *spi_device;
    char out_buf[3];
    int command_size;

    /* check input variables */
    CHECK_NULL(spi_target);
    if ((address & 0x80) != 0) {
        DEBUG_MSG("WARNING: SPI address > 127\n");
    }

    spi_device = (struct mpsse_context *)spi_target;

    /* prepare frame to be sent */
    if (spi_mux_mode == LGW_SPI_MUX_MODE1) {
        out_buf[0] = spi_mux_target;
        out_buf[1] = WRITE_ACCESS | (address & 0x7F);
        out_buf[2] = data;
        command_size = 3;
    } else {
        out_buf[0] = WRITE_ACCESS | (address & 0x7F);
        out_buf[1] = data;
        command_size = 2;
    }

    /* I/O transaction */
    CHECK_ERROR(Start(spi_device));
    CHECK_ERROR(FastWrite(spi_device, out_buf, command_size));
    CHECK_ERROR(Stop(spi_device));

    DEBUG_PRINTF("Note: SPI write success (0x%02X) < 0x%02X\n", address, data);
    return LGW_SPI_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Simple read */
int lgw_spi_r(void *spi_target, uint8_t spi_mux_mode, uint8_t spi_mux_target, uint8_t address, uint8_t *data) {
    struct mpsse_context *spi_device;
    char out_buf[2];
    int command_size;
    char in_buf[2];

    /* check input variables */
    CHECK_NULL(spi_target);
    if ((address & 0x80) != 0) {
        DEBUG_MSG("WARNING: SPI address > 127\n");
    }
    CHECK_NULL(data);

    spi_device = (struct mpsse_context *)spi_target;

    /* prepare frame to be sent */
    if (spi_mux_mode == LGW_SPI_MUX_MODE1) {
        out_buf[0] = spi_mux_target;
        out_buf[1] = READ_ACCESS | (address & 0x7F);
        command_size = 2;
    } else {
        out_buf[0] = READ_ACCESS | (address & 0x7F);
        command_size = 1;
    }

    /* I/O transaction */
    CHECK_ERROR(Start(spi_device));
    CHECK_ERROR(FastWrite(spi_device, out_buf, command_size));
    CHECK_ERROR(FastRead(spi_device, in_buf, 1));
    CHECK_ERROR(Stop(spi_device));

    *data = in_buf[0];
    DEBUG_PRINTF("Note: SPI read success (0x%02X) > 0x%02X\n", address, *data);
    return LGW_SPI_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Burst (multiple-byte) write */
int lgw_spi_wb(void *spi_target, uint8_t spi_mux_mode, uint8_t spi_mux_target, uint8_t address, uint8_t *data, uint16_t size) {
    struct mpsse_context *spi_device;
    char command[2];
    int command_size;
    char *buf;
    int size_to_do, chunk_size, offset;
    int i;

    /* check input parameters */
    CHECK_NULL(spi_target);
    if ((address & 0x80) != 0) {
        DEBUG_MSG("WARNING: SPI address > 127\n");
    }
    CHECK_NULL(data);
    if (size == 0) {
        DEBUG_MSG("ERROR: BURST OF NULL LENGTH\n");
        return LGW_SPI_ERROR;
    }

    spi_device = (struct mpsse_context *)spi_target;

    /* prepare command byte */
    if (spi_mux_mode == LGW_SPI_MUX_MODE1) {
        command[0] = spi_mux_target;
        command[1] = WRITE_ACCESS | (address & 0x7F);
        command_size = 2;
    } else {
        command[0] = WRITE_ACCESS | (address & 0x7F);
        command_size = 1;
    }
    size_to_do = size;

    /* I/O transaction */
    CHECK_ERROR(Start(spi_device));
    CHECK_ERROR(FastWrite(spi_device, command, command_size));
    for (i=0; size_to_do > 0; ++i) {
        chunk_size = (size_to_do < LGW_BURST_CHUNK) ? size_to_do : LGW_BURST_CHUNK;
        offset = i * LGW_BURST_CHUNK;
        buf = (char *)(data + offset);
        CHECK_ERROR(FastWrite(spi_device, buf, chunk_size));
        DEBUG_PRINTF("BURST WRITE: (0x%02X) < to trans %d # chunk %d (offset %d)\n", address, size_to_do, chunk_size, offset);
        size_to_do -= chunk_size; /* subtract the quantity of data already transferred */
    }
    CHECK_ERROR(Stop(spi_device));

    DEBUG_MSG("Note: SPI burst write success\n");
    return LGW_SPI_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Burst (multiple-byte) read */
int lgw_spi_rb(void *spi_target, uint8_t spi_mux_mode, uint8_t spi_mux_target, uint8_t address, uint8_t *data, uint16_t size) {
    struct mpsse_context *spi_device;
    char command[2];
    int command_size;
    char *buf;
    int size_to_do, chunk_size, offset;
    int i;

    /* check input parameters */
    CHECK_NULL(spi_target);
    if ((address & 0x80) != 0) {
        DEBUG_MSG("WARNING: SPI address > 127\n");
    }
    CHECK_NULL(data);
    if (size == 0) {
        DEBUG_MSG("ERROR: BURST OF NULL LENGTH\n");
        return LGW_SPI_ERROR;
    }

    spi_device = (struct mpsse_context *)spi_target; /* must check that spi_target is not null beforehand */

    /* prepare command byte */
    if (spi_mux_mode == LGW_SPI_MUX_MODE1) {
        command[0] = spi_mux_target;
        command[1] = READ_ACCESS | (address & 0x7F);
        command_size = 2;
    } else {
        command[0] = READ_ACCESS | (address & 0x7F);
        command_size = 1;
    }
    size_to_do = size;

    /* I/O transaction */
    CHECK_ERROR(Start(spi_device));
    CHECK_ERROR(FastWrite(spi_device, command, command_size));
    for (i=0; size_to_do > 0; ++i) {
        chunk_size = (size_to_do < LGW_BURST_CHUNK) ? size_to_do : LGW_BURST_CHUNK;
        offset = i * LGW_BURST_CHUNK;
        buf = (char *)(data + offset);
        CHECK_ERROR(FastRead(spi_device, buf, chunk_size));
        DEBUG_PRINTF("BURST READ: (0x%02X) > to trans %d # chunk %d (offset %d)\n", address, size_to_do, chunk_size, offset);
        size_to_do -= chunk_size; /* subtract the quantity of data already transferred */
    }
    CHECK_ERROR(Stop(spi_device));

    DEBUG_MSG("Note: SPI burst read success\n");
    return LGW_SPI_SUCCESS;
}

/* --- EOF ------------------------------------------------------------------ */
