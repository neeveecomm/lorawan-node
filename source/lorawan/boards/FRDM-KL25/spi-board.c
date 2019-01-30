/*!
 * \file      spi-board.c
 *
 * \brief     Target board SPI driver implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */
#include <stdio.h>
#include "utilities.h"
#include "board.h"
#include "gpio.h"
#include "spi-board.h"
#include "fsl_spi.h"
#include "fsl_gpio.h"
#include "fsl_debug_console.h"

void SpiInit( Spi_t *obj, SpiId_t spiId, PinNames mosi, PinNames miso, PinNames sclk, PinNames nss )
{
	uint16_t reg = 0x4200, val;
	obj->SpiId = spiId;

    GpioInit( &obj->Mosi, mosi, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );
    GpioInit( &obj->Miso, miso, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );
    GpioInit( &obj->Sclk, sclk, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );
    GpioInit( &obj->Nss, nss, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1 );
    //GpioInit( &obj->Nss, nss, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_UP, 1 );

    if( nss == NC )
    {
        SpiFormat( obj, 0, 0, 0, 0 );
    }
    else
    {
        SpiFormat( obj, 0, 0, 0, 1 );
    }

    //val = SpiInOut(obj, reg);
    //PRINTF("Version Register value: %d\n", val);
}

void SpiDeInit( Spi_t *obj )
{
	SPI_Deinit(SPI0);
}

void SpiFormat( Spi_t *obj, int8_t bits, int8_t cpol, int8_t cpha, int8_t slave )
{
	spi_master_config_t masterConfig = {0};
	uint32_t frequency = CLOCK_GetFreq(kCLOCK_BusClk); //10000000;

	CLOCK_EnableClock(kCLOCK_Spi0);

    /* Init SPI master */
    /*
     * masterConfig.enableStopInWaitMode = false;
     * masterConfig.polarity = kSPI_ClockPolarityActiveHigh;
     * masterConfig.phase = kSPI_ClockPhaseFirstEdge;
     * masterConfig.direction = kSPI_MsbFirst;
     * masterConfig.dataMode = kSPI_8BitMode;
     * masterConfig.txWatermark = kSPI_TxFifoOneHalfEmpty;
     * masterConfig.rxWatermark = kSPI_RxFifoOneHalfFull;
     * masterConfig.pinMode = kSPI_PinModeNormal;
     * masterConfig.outputMode = kSPI_SlaveSelectAutomaticOutput;
     * masterConfig.baudRate_Bps = 500000U;
     */
    SPI_MasterGetDefaultConfig(&masterConfig);
	masterConfig.baudRate_Bps = 100000U;
	masterConfig.outputMode = kSPI_SlaveSelectAsGpio;
    //kSPI_SlaveSelectAutomaticOutput
#if 0
    if(cpol == 0)
    	masterConfig.polarity = kSPI_ClockPolarityActiveLow;
    else
    	masterConfig.polarity = kSPI_ClockPolarityActiveHigh;

    /*if(slave == 0)
    	masterConfig.outputMode = kSPI_SlaveSelectAsGpio;
    else*/
    	masterConfig.outputMode = kSPI_SlaveSelectAutomaticOutput;
#endif
    SPI_MasterInit(SPI0, &masterConfig, frequency);
}

void SpiFrequency( Spi_t *obj, uint32_t hz )
{
}

uint16_t SpiInOut( Spi_t *obj, uint16_t outData )
{
	spi_transfer_t xfer = {0};
	uint8_t srcBuff[2];
	uint8_t rxBuff[2];

	srcBuff[0] = outData & 0xFF;
	srcBuff[1] = (outData >> 8) & 0xFF;

	/* SPI master start transfer */
	xfer.txData = srcBuff;
	xfer.rxData = rxBuff;
	xfer.dataSize = 1;
	SPI_MasterTransferBlocking(SPI0, &xfer);

	return rxBuff[0];
}

