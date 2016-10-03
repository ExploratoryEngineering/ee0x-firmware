/*******************************************************************************
 * Copyright (c) 2014-2015 IBM Corporation.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *
 * Contributors:
 *    IBM Zurich Research Lab - initial API, implementation and documentation
 *    Semtech Apps Team       - Modified to support the MBED sx1276 driver
 *                              library.
 *                              Possibility to use original or Semtech's MBED
 *                              radio driver. The selection is done by setting
 *                              USE_SMTC_RADIO_DRIVER preprocessing directive
 *                              in lmic.h
 *    Telenor Digital         - Rewrite to use the C-based sx1276 driver.
 *******************************************************************************/

#include "lmic.h"

#include <stdint.h>
#include <stdbool.h>

#include "board.h"
#include "sx1276.h"

/*!
 * Syncword for lora networks
 */
#define LORA_MAC_SYNCWORD                           0x34

/*
 * Callback functions prototypes
 */
/*!
 * @brief Function to be executed on Radio Tx Done event
 */
void OnTxDone( void );

/*!
 * @brief Function to be executed on Radio Rx Done event
 */
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );

/*!
 * @brief Function executed on Radio Tx Timeout event
 */
void OnTxTimeout( void );

/*!
 * @brief Function executed on Radio Rx Timeout event
 */
void OnRxTimeout( void );

/*!
 * @brief Function executed on Radio Rx Error event
 */
void OnRxError( void );

/*!
 * @brief Function executed on Radio Fhss Change Channel event
 */
void OnFhssChangeChannel( uint8_t channelIndex );

/*!
 * @brief Function executed on CAD Done event
 */
void OnCadDone( void );

/*!
 * Radio events function pointer
 */
static RadioEvents_t RadioEvents;

static const u2_t LORA_RXDONE_FIXUP[] = {
    [FSK]  =     us2osticks(0), // (   0 ticks)
    [SF7]  =     us2osticks(0), // (   0 ticks)
    [SF8]  =  us2osticks(1648), // (  54 ticks)
    [SF9]  =  us2osticks(3265), // ( 107 ticks)
    [SF10] =  us2osticks(7049), // ( 231 ticks)
    [SF11] = us2osticks(13641), // ( 447 ticks)
    [SF12] = us2osticks(31189), // (1022 ticks)
};

void OnTxDone( void )
{
    ostime_t now = os_getTime( );
    // save exact tx time
    LMIC.txend = now - us2osticks( RADIO_WAKEUP_TIME ); // TXDONE FIXUP

    // go from stanby to sleep
    SX1276SetSleep( );
    // run os job (use preset func ptr)
    os_setCallback( &LMIC.osjob, LMIC.osjob.func );
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    ostime_t now = os_getTime( );
    // save exact rx time
    if( getBw( LMIC.rps ) == BW125 )
    {
        now -= LORA_RXDONE_FIXUP[getSf( LMIC.rps )];
    }
    LMIC.rxtime = now;
    // read the PDU and inform the MAC that we received something
    LMIC.dataLen = size;
    // now read the FIFO
    memcpy( LMIC.frame, payload, size );
    // read rx quality parameters
    LMIC.snr  = snr; // SNR [dB] * 4
    LMIC.rssi = rssi; // RSSI [dBm] (-196...+63)

    // go from stanby to sleep
    SX1276SetSleep( );
    // run os job (use preset func ptr)
    os_setCallback( &LMIC.osjob, LMIC.osjob.func );
}

void OnTxTimeout( void )
{
    // indicate error
    LMIC.dataLen = 0;

    // go from stanby to sleep
    SX1276SetSleep( );
    // run os job (use preset func ptr)
    os_setCallback( &LMIC.osjob, LMIC.osjob.func );
}

void OnRxTimeout( void )
{
    // indicate timeout
    LMIC.dataLen = 0;

    // go from stanby to sleep
    SX1276SetSleep( );
    // run os job (use preset func ptr)
    os_setCallback( &LMIC.osjob, LMIC.osjob.func );
}

void OnRxError( void )
{

    // indicate error
    LMIC.dataLen = 0;

    // go from stanby to sleep
    SX1276SetSleep( );
    // run os job (use preset func ptr)
    os_setCallback( &LMIC.osjob, LMIC.osjob.func );
}

/*!
 * LMIC API implementation
 */
// RADIO STATE
// (initialized by radio_init( ), used by radio_rand1( ))
static u1_t randbuf[16];

#ifdef EE02
//#define EE02_EXT_ANT 1
#define EE02_INT_ANT 1
const uint32_t ext_ant_pin = SX1276_ANT_HF_CTRL;
const uint32_t int_ant_pin = SX1276_ANT_LF_CTRL;
#endif

// get random seed from wideband noise rssi
void radio_init( void )
{
    hal_disableIRQs( );

    // Set antenna
    #ifdef EE02_EXT_ANT
    GpioWrite(&ext_ant_pin, 1);
    GpioWrite(&int_ant_pin, 1);
    #endif
    #ifdef EE02_CHIP_ANT
    GpioWrite(&ext_ant_pin, 0);
    GpioWrite(&int_ant_pin, 0);
    #endif

    // Initialize Radio driver
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.RxError = OnRxError;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    SX1276Init( &RadioEvents );

    // seed 15-byte randomness via noise rssi
    // Set LoRa modem ON
    SX1276SetModem( MODEM_LORA );
    // Disable LoRa modem interrupts
    SX1276Write( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                  RFLR_IRQFLAGS_RXDONE |
                  RFLR_IRQFLAGS_PAYLOADCRCERROR |
                  RFLR_IRQFLAGS_VALIDHEADER |
                  RFLR_IRQFLAGS_TXDONE |
                  RFLR_IRQFLAGS_CADDONE |
                  RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                  RFLR_IRQFLAGS_CADDETECTED );

    // Set radio in continuous reception
    SX1276SetRx( 0 );

    for( int i = 1; i < 16; i++ )
    {
        for( int j = 0; j < 8; j++ )
        {
            u1_t b; // wait for two non-identical subsequent least-significant bits
            while( ( b = SX1276Read( REG_LR_RSSIWIDEBAND ) & 0x01 ) == ( Radio.Read( REG_LR_RSSIWIDEBAND ) & 0x01 ) );
            randbuf[i] = ( randbuf[i] << 1 ) | b;
        }
    }
    randbuf[0] = 16; // set initial index

    // Change LoRa modem SyncWord
    SX1276Write( REG_LR_SYNCWORD, LORA_MAC_SYNCWORD );

    SX1276SetSleep( );

    hal_enableIRQs( );
}

// return next random byte derived from seed buffer
// (buf[0] holds index of next byte to be returned)
u1_t radio_rand1( void )
{
    u1_t i = randbuf[0];
    LMIC_ASSERT( i != 0 );
    if( i == 16 )
    {
        os_aes( AES_ENC, randbuf, 16 ); // encrypt seed with any key
        i = 0;
    }
    u1_t v = randbuf[i++];
    randbuf[0] = i;
    return v;
}

void os_radio( u1_t mode )
{
    hal_disableIRQs( );
    switch( mode )
    {
    case RADIO_RST:
        // put radio to sleep
        SX1276SetSleep( );
        break;

    case RADIO_TX:
        // transmit frame now
        //LMIC_ASSERT( Radio.GetState( ) == IDLE );

        SX1276SetChannel( LMIC.freq );
        if( getSf( LMIC.rps ) == FSK )
        { // FSK modem
            SX1276SetTxConfig( MODEM_FSK, LMIC.txpow, 25e3, 0, 50e3, 0, 5, false, true, 0, 0, false, 3e6 );
        }
        else
        { // LoRa modem

            SX1276SetTxConfig( MODEM_LORA, LMIC.txpow, 0, getBw( LMIC.rps ), getSf( LMIC.rps ) + 6, getCr( LMIC.rps ) + 1, 8, getIh( LMIC.rps ) ? true : false, ( getNocrc( LMIC.rps ) == 0 ) ? true : false, 0, 0, false, 3e6 );
        }

        //starttx( ); // buf=LMIC.frame, len=LMIC.dataLen
        SX1276Send( LMIC.frame, LMIC.dataLen );
        break;

    case RADIO_RX:
        // receive frame now (exactly at rxtime)
        //LMIC_ASSERT( Radio.GetState( ) == IDLE );

        SX1276SetChannel( LMIC.freq );
        if( getSf( LMIC.rps ) == FSK )
        { // FSK modem
            //Radio.SetRxConfig( MODEM_FSK, 50e3, 50e3, 0, 83.333e3, 5, 0, false, 0, true, 0, 0, false, false );
            SX1276SetRxConfig( MODEM_FSK, 50e3, 50e3, 0, 83.333e3, 5, 0, false, 0, true, 0, 0, false, true );
        }
        else
        { // LoRa modem
            if( ( getSf( LMIC.rps ) <= SF9 ) && ( LMIC.rxsyms < 8 ) )
            {
                SX1276SetRxConfig( MODEM_LORA, getBw( LMIC.rps ), getSf( LMIC.rps ) + 6, getCr( LMIC.rps ) + 1, 0, 8, LMIC.rxsyms + 3, getIh( LMIC.rps ) ? true : false, getIh( LMIC.rps ), ( getNocrc( LMIC.rps ) == 0 ) ? true : false, 0, 0, true, false );
            }
            else
            {
                SX1276SetRxConfig( MODEM_LORA, getBw( LMIC.rps ), getSf( LMIC.rps ) + 6, getCr( LMIC.rps ) + 1, 0, 8, LMIC.rxsyms, getIh( LMIC.rps ) ? true : false, getIh( LMIC.rps ), ( getNocrc( LMIC.rps ) == 0 ) ? true : false, 0, 0, true, false );
            }
        }

        // now instruct the radio to receive
        hal_waitUntil( LMIC.rxtime ); // busy wait until exact rx time

        //startrx( RXMODE_SINGLE ); // buf = LMIC.frame, time = LMIC.rxtime, timeout=LMIC.rxsyms
        if( getSf( LMIC.rps ) == FSK )
        { // FSK modem
            SX1276SetRx( 50e3 ); // Max Rx window 50 ms
        }
        else
        { // LoRa modem
            SX1276SetRx( 3e6 ); // Max Rx window 3 seconds
        }
        break;

    case RADIO_RXON:
        // start scanning for beacon now

        //LMIC_ASSERT( Radio.GetState( ) == IDLE );

        SX1276SetChannel( LMIC.freq );
        if( getSf( LMIC.rps ) == FSK )
        { // FSK modem
            SX1276SetRxConfig( MODEM_FSK, 50e3, 50e3, 0, 83.333e3, 5, 0, false, 0, true, 0, 0, false, true );
        }
        else
        { // LoRa modem
            SX1276SetRxConfig( MODEM_LORA, getBw( LMIC.rps ), getSf( LMIC.rps ) + 6, getCr( LMIC.rps ) + 1, 0, 8, LMIC.rxsyms, getIh( LMIC.rps ) ? true : false, getIh( LMIC.rps ), ( getNocrc( LMIC.rps ) == 0 ) ? true : false, 0, 0, true, true );
        }

        //startrx( RXMODE_SCAN ); // buf = LMIC.frame
        SX1276SetRx( 0 );
        break;
    }
    hal_enableIRQs( );
}
