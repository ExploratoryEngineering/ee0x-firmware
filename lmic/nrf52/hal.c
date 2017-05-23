/*
**   Copyright 2016 Telenor Digital AS
**
**  Licensed under the Apache License, Version 2.0 (the "License");
**  you may not use this file except in compliance with the License.
**  You may obtain a copy of the License at
**
**      http://www.apache.org/licenses/LICENSE-2.0
**
**  Unless required by applicable law or agreed to in writing, software
**  distributed under the License is distributed on an "AS IS" BASIS,
**  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
**  See the License for the specific language governing permissions and
**  limitations under the License.
*/

/**
 * HAL implementation for for nRF52.
 */
#include "nrf_delay.h"
#include "lmic.h"
#include "app_error.h"
#include "nrf_rtc.h"
#include "nrf_drv_rtc.h"
#include "nrf_drv_clock.h"
#include "nrf52_pins.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_spi.h"
#include "app_util_platform.h"
#include "board.h"
#include "battery.h"

/*
 * Enable and disable IRQs when requested by the library. Note that hal_sleep()
 * is called with NMI set so you'd have to modify oslmic.c to enable interrupts
 * around hal_sleep().
 */
#define DO_IRQ 1

#define NO_LATE_SCHEDULES 1

/*
 * This is the official duration of "soon"; the hal_checkTime()
 * call checks if the time is somewhere near this.
 */
#define NEAR_TIME_TICKS 5 // 5 = 5/32768s = 153us, 253 = approx 7.7 ms, 0.0077s * 32768Hz

/*
 * Error code to trigger when hal_failed() is called
 */
#define THERMONUCLEAR_ERROR_OMG_WTF_BBQ 99999

/*
 * RTC clock instance
 */
static const nrf_drv_rtc_t rtc = NRF_DRV_RTC_INSTANCE(0);

/*
 * SPI master configuration
 */
static const nrf_drv_spi_t spi_master_0 = NRF_DRV_SPI_INSTANCE(1);

// LF clock rolls over on 24 bits; increment this when there's a rollover.
static int64_t clock_mask = 0;
#define CLOCK_MASK_INCREMENT 0x1000000
#define LF_CLOCK_ROLLOVER 0xFFFFFF

static uint32_t to_ticks(const int64_t ticks) {
    return (ticks & LF_CLOCK_ROLLOVER);
}

/**
 * Interrupt level. Counts up and down when enable/disable is called.
 */
static unsigned char irq_level = 0;

/**
 * sx1276 HAL - max number of timers.
 */
#define MAX_TIMERS 3

/**
 * sx1276 HAL - number of active timers.
 */
static uint8_t timer_count = 0;

/**
 * sx1276 HAL - max number of timers.
 */
static TimerEventHandler_t timer_irq_handlers[MAX_TIMERS] = { NULL };

/*
 * Configure LF clock
 */
static void lfclk_config(void)
{
    APP_ERROR_CHECK(nrf_drv_clock_init());
    nrf_drv_clock_lfclk_request(NULL);

    while (!nrf_drv_clock_lfclk_is_running()) {
        nrf_delay_ms(10);
    }
}

/**
 * Callback for RTC clock. Not used but required by the API.
 */
static void rtc_handler(nrf_drv_rtc_int_type_t int_type)
{
    switch (int_type) {
        case NRF_DRV_RTC_INT_COMPARE0:
            // Wakes up processing
            break;

        case NRF_DRV_RTC_INT_COMPARE1:
            if (timer_irq_handlers[0]) {
                timer_irq_handlers[0]();
            }
            break;

        case NRF_DRV_RTC_INT_COMPARE2:
            if (timer_irq_handlers[1]) {
                timer_irq_handlers[1]();
            }
            break;

        case NRF_DRV_RTC_INT_COMPARE3:
            if (timer_irq_handlers[2]) {
                timer_irq_handlers[2]();
            }
            break;

        case NRF_DRV_RTC_INT_OVERFLOW:
            clock_mask += CLOCK_MASK_INCREMENT;
            break;

        case NRF_DRV_RTC_INT_TICK:
            break;

        default:
            break;
    }
}

/**
 * Configure (and start) the RTC clock. Frequency is set in the config file.
 */
static void rtc_config(void)
{
    //Initialize RTC instance
    nrf_drv_rtc_config_t config = {
        .prescaler = 0,
        .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
        .reliable = RTC0_CONFIG_RELIABLE,
        .tick_latency =  RTC_US_TO_TICKS(NRF_MAXIMUM_LATENCY_US, RTC0_CONFIG_FREQUENCY)
    };
    APP_ERROR_CHECK(nrf_drv_rtc_init(&rtc, &config, rtc_handler));

    nrf_drv_rtc_tick_disable(&rtc);
    nrf_drv_rtc_int_enable(&rtc,
        NRF_RTC_INT_COMPARE0_MASK
        |NRF_RTC_INT_COMPARE1_MASK
        |NRF_RTC_INT_COMPARE2_MASK
        |NRF_RTC_INT_COMPARE3_MASK
        |NRF_RTC_INT_OVERFLOW_MASK
        );
    //Enable tick event & interrupt
    nrf_drv_rtc_tick_enable(&rtc, true);

    //Power on RTC instance
    nrf_drv_rtc_enable(&rtc);
}

/* callbacks inside the driver */
extern void SX1276OnDio0Irq(void);
extern void SX1276OnDio1Irq(void);
extern void SX1276OnDio2Irq(void);
extern void SX1276OnDio3Irq(void);
extern void SX1276OnDio4Irq(void);
/**
 * Event handler for GPIO pins.
 */
static void pin_event_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
    switch (pin) {
        case SX1276_DIO0_PIN:
            SX1276OnDio0Irq();
            break;
        case SX1276_DIO1_PIN:
            SX1276OnDio1Irq();
            break;
        case SX1276_DIO2_PIN:
            SX1276OnDio2Irq();
            break;
        case SX1276_DIO3_PIN:
            SX1276OnDio3Irq();
            break;
        case SX1276_DIO4_A_PIN:
            SX1276OnDio4Irq();
            break;
        default:
            printf("Don't know what pin %d does\n", (unsigned int) pin);
            break;
    }
}

/**
 * Configure and set up GPIO pins (NSS, RXTX and RST out, DIO0..DIO3 in).
 * DIO4 and DIO5 isn't used.
 */
static void gpio_config(void) {

    APP_ERROR_CHECK(nrf_drv_gpiote_init());

    nrf_drv_gpiote_out_config_t outconfig = GPIOTE_CONFIG_OUT_SIMPLE(true);
    APP_ERROR_CHECK(nrf_drv_gpiote_out_init(SX1276_NSS_PIN, &outconfig));
    APP_ERROR_CHECK(nrf_drv_gpiote_out_init(SX1276_RXTX_PIN, &outconfig));
    APP_ERROR_CHECK(nrf_drv_gpiote_out_init(SX1276_RST_PIN, &outconfig));
    APP_ERROR_CHECK(nrf_drv_gpiote_out_init(SX1276_RST_PIN, &outconfig));
    #ifdef EE02
    APP_ERROR_CHECK(nrf_drv_gpiote_out_init(SX1276_ANT_HF_CTRL, &outconfig));
    #endif
    nrf_drv_gpiote_in_config_t inconfig = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
    inconfig.pull = NRF_GPIO_PIN_NOPULL;

    APP_ERROR_CHECK(nrf_drv_gpiote_in_init(SX1276_DIO0_PIN, &inconfig, pin_event_handler));
    APP_ERROR_CHECK(nrf_drv_gpiote_in_init(SX1276_DIO1_PIN, &inconfig, pin_event_handler));
    APP_ERROR_CHECK(nrf_drv_gpiote_in_init(SX1276_DIO2_PIN, &inconfig, pin_event_handler));
    APP_ERROR_CHECK(nrf_drv_gpiote_in_init(SX1276_DIO3_PIN, &inconfig, pin_event_handler));
    APP_ERROR_CHECK(nrf_drv_gpiote_in_init(SX1276_DIO4_A_PIN, &inconfig, pin_event_handler));
    APP_ERROR_CHECK(nrf_drv_gpiote_in_init(BNO055_BL_IND, &inconfig, pin_event_handler));

    nrf_drv_gpiote_in_event_enable(SX1276_DIO0_PIN, true);
    nrf_drv_gpiote_in_event_enable(SX1276_DIO1_PIN, true);
    nrf_drv_gpiote_in_event_enable(SX1276_DIO2_PIN, true);
    nrf_drv_gpiote_in_event_enable(SX1276_DIO3_PIN, true);
    nrf_drv_gpiote_in_event_enable(SX1276_DIO4_A_PIN, true);
}

/**
 * Configure and start SPI driver.
 */
static void spi_config(void) {
    uint32_t err_code;
    nrf_drv_spi_config_t config =
    {
        .orc          = 0xCC,
        .frequency    = NRF_DRV_SPI_FREQ_4M,
        .mode         = NRF_DRV_SPI_MODE_0,
        .bit_order    = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST,
    };
    // cs/ss pin is set by library
    config.ss_pin = NRF_DRV_SPI_PIN_NOT_USED;
    config.sck_pin = PLAT_SPI_SCK;
    config.mosi_pin = PLAT_SPI_MOSI;
    config.miso_pin = PLAT_SPI_MISO;
    err_code = nrf_drv_spi_init(&spi_master_0, &config, NULL);
    APP_ERROR_CHECK(err_code);
}

/**
 * Init HAL library
 */
void hal_init (void) {
    lfclk_config();
    rtc_config();
    gpio_config();
    spi_config();
}

/**
 * Set and clear the RXTX pin on the SX1276 chip
 */
void hal_pin_rxtx (u1_t val) {
    switch (val) {
        case 0:
            nrf_drv_gpiote_out_clear(SX1276_RXTX_PIN);
            break;
        case 1:
            nrf_drv_gpiote_out_set(SX1276_RXTX_PIN);
            break;
    }
}

/**
 * Set, clear and float (ie release) the RST pin
 */
void hal_pin_rst (u1_t val) {
    const nrf_drv_gpiote_out_config_t inconfig = GPIOTE_CONFIG_OUT_SIMPLE(true);
    switch (val) {
        case 0:
            nrf_drv_gpiote_out_clear(SX1276_RST_PIN);
            break;
        case 1:
            nrf_drv_gpiote_out_set(SX1276_RST_PIN);
            break;
        case 2:
            nrf_drv_gpiote_out_uninit(SX1276_RST_PIN);
            APP_ERROR_CHECK(nrf_drv_gpiote_out_init(SX1276_RST_PIN, &inconfig));
            break;
    }
}

/**
 * Read (a single byte) from the SPI interface.
 */
u1_t hal_spi (u1_t outval) {
    // This is a synchronous transfer that blocks.
    u1_t retval = 0;
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi_master_0, &outval, sizeof(outval), &retval, sizeof(retval)));
    return retval;
}

/**
 * Disable interrupts
 */
void hal_disableIRQs (void) {
#if DO_IRQ
   __disable_irq();
#endif
   irq_level++;
}

/**
 * Enable interrupts
 */
void hal_enableIRQs (void) {
    // TODO: Enable when everything is working.
    if (--irq_level <= 0) {
#if DO_IRQ
        __enable_irq();
#endif
    }
}

/**
 * Go to sleep.
 * TODO: Put all other devices to sleep here? GPS receiver might be active
 * at this time.
 */
void hal_sleep (void) {
    // Wait for event/irq to trigger.
    __WFI();
}

/**
 * Return ticks from the OS. Uses the RTC clock on the nRF52 board. I don't know
 * how LMiC handles overflows (probably don't?) but it must be implemented.
 *
 * The clock will overflow in a little more than 36 hours (with a 32 bit
 * integer as counter). There might be nasty crashing scenarios in that case.
 */
s8_t hal_ticks (void) {
    return clock_mask | nrf_drv_rtc_counter_get(&rtc);
}


/**
 * Do a (semi-busy) wait until it is time to do work.
 */
void hal_waitUntil (s8_t ttime) {
    uint32_t actual = to_ticks(ttime);
    while (hal_ticks() < actual) /* busy wait */ ;
}

#if NO_LATE_SCHEDULES
static int64_t last_scheduled_event = - 1;
#endif

#define CCNUM 0
/**
 * The purpose of this call is to make sure the OS event loop inside the
 * LMiC library runs. If this call returns "a very long time into the
 * future" it will call hal_sleep() and try again. The LMiC documentation
 * is a bit brief on what is "soon" but it should probably match the hal_sleep()
 * delay in some way.
 */
u1_t hal_checkTimer (s8_t targettime) {
    int64_t now = hal_ticks();
    int32_t ticks_to_event = targettime - now;

    if (ticks_to_event < NEAR_TIME_TICKS) {
        // Time is "soon" or have already passed
        return 1;
    }

    uint32_t tick_time = to_ticks(targettime);
    #if NO_LATE_SCHEDULES
    if (last_scheduled_event < 0 || last_scheduled_event < now || last_scheduled_event > targettime) {
        last_scheduled_event = targettime;
        APP_ERROR_CHECK(nrf_drv_rtc_cc_set(&rtc,CCNUM,tick_time,true));
    }
    #else
    APP_ERROR_CHECK(nrf_drv_rtc_cc_set(&rtc,CCNUM,tick_time,true));
    #endif
    return 0;
}

/**
 * Something went terribly wrong
 */
void hal_failed (void) {
    while (irq_level > 0) {
        hal_enableIRQs();
    }

    NRF_LOG("*** HAL has failed. Exploding in 2 seconds...\n");
    nrf_delay_ms(2000);
    // *giggles* - this resets the board, emulating an application error.
    // Probably not a very pretty way of doing it.
    APP_ERROR_CHECK(THERMONUCLEAR_ERROR_OMG_WTF_BBQ);
}

/**
 * Battery level implementation. Used by LMiC (and declared in oslmic.h)
 */
u1_t os_getBattLevel(void) {
    uint16_t milli_volts = 0;
    battery_get_mv(&milli_volts);
    return battery_to_byte(milli_volts);
}

/*
 * ============================================================================
 *
 * HAL for sx1276 driver. These calls emulate (to a certain extent) the mBed
 * classes. Implementation might look a bit weird but it works.
 *
 * ============================================================================
 */
void GpioWrite(const Gpio_t* pin, const uint8_t high) {
    if (high) {
        nrf_drv_gpiote_out_set(*pin);
    }
    else {
        nrf_drv_gpiote_out_clear(*pin);
    }
}

/**
 * Straightforward delay function.
 */
void DelayMs(const uint32_t ms) {
    nrf_delay_ms(ms);
}

/**
 * hal_spi() does the exact same thing.
 */
uint8_t __inline SpiInOut(const Spi_t* spi, uint8_t data) {
    return hal_spi(data);
}

/**
 * Initialize timer. There's three timers in use, all of these use CC1..3 in
 * the NRF52 library.
 */
void TimerInit(TimerEvent_t* event, const TimerEventHandler_t event_handler) {
    if (timer_count >= MAX_TIMERS) {
        NRF_LOG("WARNING: Max timer count (3) exceeded!\n");
        return;
    }
    timer_irq_handlers[timer_count] = event_handler;
    event->timer_id = timer_count + 1;
    event->ticks = 0;
    event->triggered = false;
    timer_count++;
}

/**
 * Cancel/stop timer.
 */
void TimerStop(const TimerEvent_t* event) {
    nrf_drv_rtc_cc_disable(&rtc, event->timer_id);
}

/**
 * Set trigger time for timer
 */
void TimerSetValue(TimerEvent_t* event, const uint32_t ticks) {
    event->ticks = ticks;
}

/**
 * Set the actual timer.
 */
void TimerStart(const TimerEvent_t* event) {
    int64_t now = hal_ticks();
    int32_t ticks_to_event = to_ticks(now + event->ticks);

    APP_ERROR_CHECK(nrf_drv_rtc_cc_set(&rtc, event->timer_id, ticks_to_event, true));
}
