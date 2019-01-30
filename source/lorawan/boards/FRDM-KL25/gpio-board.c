/*!
 * \file      gpio-board.c
 *
 * \brief     Target board GPIO driver implementation
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
#include "MKL25Z4.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "utilities.h"
#include "board-config.h"
#include "rtc-board.h"
#include "gpio-board.h"
#include "assert.h"

#if defined( BOARD_IOE_EXT )
#include "gpio-ioe.h"
#endif

static Gpio_t *GpioIrq[16];

void GpioMcuInit( Gpio_t *obj, PinNames pin, PinModes mode, PinConfigs config, PinTypes type, uint32_t value )
{
	gpio_pin_config_t gpioCfg = {0};
	port_pin_config_t pinCfg = {0};

	if(mode < PIN_ALTERNATE_FCT)
		pinCfg.mux = kPORT_MuxAsGpio;
	else
		pinCfg.mux = kPORT_MuxAlt2;

    if( pin < IOE_0 )
    {
        obj->pin = pin;

        if( pin == NC )
        {
            return;
        }

        obj->pinIndex = ( 0x01 << ( obj->pin & 0x0F ) );

        if( ( obj->pin & 0xF00 ) == 0x00 )
        {
            obj->port = PORTA;
            obj->gpio = GPIOA;
            CLOCK_EnableClock(kCLOCK_PortA);                           /* Port E Clock Gate Control: Clock enabled */
        }
        else if( ( obj->pin & 0xF00 ) == 0x100 )
        {
            obj->port = PORTB;
            obj->gpio = GPIOB;
            CLOCK_EnableClock(kCLOCK_PortB);                           /* Port E Clock Gate Control: Clock enabled */
        }
        else if( ( obj->pin & 0xF00 ) == 0x200 )
        {
            obj->port = PORTC;
            obj->gpio = GPIOC;
            CLOCK_EnableClock(kCLOCK_PortC);                           /* Port E Clock Gate Control: Clock enabled */
        }
        else if( ( obj->pin & 0xF00 ) == 0x300 )
        {
            obj->port = PORTD;
            obj->gpio = GPIOD;
            CLOCK_EnableClock(kCLOCK_PortD);                           /* Port E Clock Gate Control: Clock enabled */
        }
        else
        {
            assert( FAIL );
        }

        if(type == PIN_PULL_UP) {
        	pinCfg.pullSelect = obj->pull = kPORT_PullUp;
        }else if(type == PIN_PULL_DOWN) {
        	pinCfg.pullSelect = obj->pull = kPORT_PullDown;
        }

        if( mode == PIN_INPUT )
        {
            gpioCfg.pinDirection = kGPIO_DigitalInput;
        }

        // Sets initial output value
        if( mode == PIN_OUTPUT )
        {
        	gpioCfg.pinDirection = kGPIO_DigitalOutput;
            gpioCfg.outputLogic = value;
        }

        PORT_SetPinConfig(obj->port, (obj->pin & 0xFF), &pinCfg);
        GPIO_PinInit(obj->gpio, (obj->pin & 0xFF), &gpioCfg);
    }
}

void GpioMcuSetContext( Gpio_t *obj, void* context )
{
    obj->Context = context;
}

void GpioMcuSetInterrupt( Gpio_t *obj, IrqModes irqMode, IrqPriorities irqPriority, GpioIrqHandler *irqHandler )
{
    if( obj->pin < IOE_0 )
    {
        uint32_t priority = 0;
        port_pin_config_t pin_config = {0};
        IRQn_Type IRQnb;

        if( obj->pin == NC )
        {
            return;
        }

        if( irqHandler == NULL )
        {
            return;
        }

        if(obj->gpio == GPIOD) {
        	IRQnb = PORTD_IRQn;
        }else{
        	IRQnb = PORTA_IRQn;
        }
        obj->IrqHandler = irqHandler;

    	pin_config.pullSelect = kPORT_PullUp;;
    	pin_config.mux = kPORT_MuxAsGpio;

    	PORT_SetPinConfig(obj->port, (obj->pin & 0xFF), &pin_config);

        if( irqMode == IRQ_RISING_EDGE )
        {
        	PORT_SetPinInterruptConfig(obj->port, (obj->pin & 0xFF), kPORT_InterruptRisingEdge);
        }
        else if( irqMode == IRQ_FALLING_EDGE )
        {
        	PORT_SetPinInterruptConfig(obj->port, (obj->pin & 0xFF), kPORT_InterruptFallingEdge);
        }
        else
        {
        	PORT_SetPinInterruptConfig(obj->port, (obj->pin & 0xFF), kPORT_InterruptEitherEdge);
        }


        switch( irqPriority )
        {
        case IRQ_VERY_LOW_PRIORITY:
        case IRQ_LOW_PRIORITY:
            priority = 3;
            break;
        case IRQ_MEDIUM_PRIORITY:
            priority = 2;
            break;
        case IRQ_HIGH_PRIORITY:
            priority = 1;
            break;
        case IRQ_VERY_HIGH_PRIORITY:
        default:
            priority = 0;
            break;
        }

        GpioIrq[( obj->pin ) & 0x0F] = obj;

        NVIC_SetPriority( IRQnb , priority);
        NVIC_EnableIRQ( IRQnb );
    }
    return;
}

void GpioMcuRemoveInterrupt( Gpio_t *obj )
{
    if( obj->pin < IOE_0 )
    {
    	PORT_SetPinInterruptConfig(obj->port, (obj->pin & 0xFF), kPORT_InterruptOrDMADisabled);
    }
}

void GpioMcuWrite( Gpio_t *obj, uint32_t value )
{
    if( obj->pin < IOE_0 )
    {
        if( obj == NULL )
        {
            assert( FAIL );
        }
        // Check if pin is not connected
        if( obj->pin == NC )
        {
            return;
        }

        if(value == 0)
        	GPIO_ClearPinsOutput(obj->gpio, 1 << (obj->pin & 0xFF));
        else
        	GPIO_SetPinsOutput(obj->gpio, 1 << (obj->pin & 0xFF));
    }
}

void GpioMcuToggle( Gpio_t *obj )
{
    if( obj->pin < IOE_0 )
    {
        if( obj == NULL )
        {
            assert( FAIL );
        }

        // Check if pin is not connected
        if( obj->pin == NC )
        {
            return;
        }
        GPIO_TogglePinsOutput(obj->gpio, 1 << (obj->pin & 0xFF));
    }
}

uint32_t GpioMcuRead( Gpio_t *obj )
{
    if( obj->pin < IOE_0 )
    {
        if( obj == NULL )
        {
            assert( FAIL );
        }
        // Check if pin is not connected
        if( obj->pin == NC )
        {
            return 0;
        }
        return GPIO_ReadPinInput( obj->gpio, (obj->pin & 0xFF));
    }
    else
    {
        return 0;
    }
}

void PORTA_IRQHandler( void )
{
	uint32_t intr = PORT_GetPinsInterruptFlags(PORTA);

	/* DIO 5*/
	if (intr & (1 << 13)) {
		SX1276OnDio5Irq(NULL);
		PORT_ClearPinsInterruptFlags(PORTA, (1 << 13));
	}

	/* DIO1 */
	if (intr & (1 << 12)) {
		SX1276OnDio1Irq(NULL);
		PORT_ClearPinsInterruptFlags(PORTA, (1 << 12));
	}

	/* DIO2 */
	if (intr & (1 << 4)) {
		SX1276OnDio2Irq(NULL);
		PORT_ClearPinsInterruptFlags(PORTA, (1 << 4));
	}
	/* DIO3 */
	if (intr & (1 << 5)) {
		SX1276OnDio3Irq(NULL);
		PORT_ClearPinsInterruptFlags(PORTA, (1 << 5));
	}

}

void PORTD_IRQHandler( void )
{
	SX1276OnDio0Irq(NULL);
	PORT_ClearPinsInterruptFlags(PORTD, (1 << 4));
}

void HAL_GPIO_EXTI_Callback( uint16_t gpioPin )
{
    uint8_t callbackIndex = 0;

    if( gpioPin > 0 )
    {
        while( gpioPin != 0x01 )
        {
            gpioPin = gpioPin >> 1;
            callbackIndex++;
        }
    }

    if( ( GpioIrq[callbackIndex] != NULL ) && ( GpioIrq[callbackIndex]->IrqHandler != NULL ) )
    {
        GpioIrq[callbackIndex]->IrqHandler( GpioIrq[callbackIndex]->Context );
    }
}
