/*!
 * \file      board-config.h
 *
 * \brief     Board configuration
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
 *               ___ _____ _   ___ _  _____ ___  ___  ___ ___
 *              / __|_   _/_\ / __| |/ / __/ _ \| _ \/ __| __|
 *              \__ \ | |/ _ \ (__| ' <| _| (_) |   / (__| _|
 *              |___/ |_/_/ \_\___|_|\_\_| \___/|_|_\\___|___|
 *              embedded.connectivity.solutions===============
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 *
 * \author    Daniel Jaeckle ( STACKFORCE )
 *
 * \author    Johannes Bruder ( STACKFORCE )
 */
#ifndef __BOARD_CONFIG_H__
#define __BOARD_CONFIG_H__

/*!
 * Defines the time required for the TCXO to wakeup [ms].
 */
#define BOARD_TCXO_WAKEUP_TIME                      5

/*!
 * Board MCU pins definitions
 */
#define RADIO_RESET                                 PD_5

#define RADIO_MOSI                                  PD_2
#define RADIO_MISO                                  PD_3
#define RADIO_SCLK                                  PD_1

#define RADIO_NSS                                   PD_0

#define RADIO_DIO_0                                 PD_4
#define RADIO_DIO_1                                 PC_8
#define RADIO_DIO_2                                 PC_9
#define RADIO_DIO_3                                 NC
//#define RADIO_DIO_1                                 PC_8
//#define RADIO_DIO_2                                 PC_9
//#define RADIO_DIO_3                                 NC
#define RADIO_DIO_4                                 NC
#define RADIO_DIO_5                                 PA_13

#define RADIO_DUMMY_DIO1							PC_8
#define RADIO_DUMMY_DIO2							PC_9

#define RADIO_TCXO_POWER                            NC

#define RADIO_ANT_SWITCH_RX                         NC
#define RADIO_ANT_SWITCH_TX_BOOST                   NC
#define RADIO_ANT_SWITCH_TX_RFO                     NC

#define LED_1                                       NC
#define LED_2                                       NC
#define LED_3                                       NC
#define LED_4                                       NC

#define LED_GREEN                                   LED_1
#define LED_RED1                                    LED_2
#define LED_BLUE                                    LED_3
#define LED_RED2                                    LED_4

#define BTN_1                                       NC

#define OSC_LSE_IN                                  NC
#define OSC_LSE_OUT                                 NC

#define OSC_HSE_IN                                  NC
#define OSC_HSE_OUT                                 NC

#define SWCLK                                       NC
#define SWDAT                                       NC

#define I2C_SCL                                     PE_25
#define I2C_SDA                                     PE_24

#define UART_TX                                     PA_1
#define UART_RX                                     PA_2

// Debug pins definition.
#define RADIO_DBG_PIN_TX                            PA_1
#define RADIO_DBG_PIN_RX                            PA_2

#endif // __BOARD_CONFIG_H__
