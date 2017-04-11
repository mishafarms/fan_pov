/*
 * timer.cpp
 *
 *  Created on: Feb 19, 2017
 *      Author: micro
 */

#include "sdkconfig.h"
#include <stdio.h>
#include "esp_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "soc/timer_group_struct.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"

#define TIMER_INTR_SEL TIMER_INTR_LEVEL  /*!< Timer level interrupt */
#define TIMER_GROUP    TIMER_GROUP_0     /*!< Test on timer group 0 */
#define TIMER_DIVIDER   8                /*!< Hardware timer clock divider */
#define TIMER_SCALE    (TIMER_BASE_CLK / TIMER_DIVIDER)  /*!< used to calculate counter value */
#define TIMER_FINE_ADJ   (1.4*(TIMER_BASE_CLK / TIMER_DIVIDER)/1000000) /*!< used to compensate alarm value */
#define TIMER_INTERVAL0_SEC   (3.4179)   /*!< test interval for timer 0 */
#define TIMER_INTERVAL1_SEC   (.000549)   /*!< test interval for timer 1 */
#define TEST_WITHOUT_RELOAD   0   /*!< example of auto-reload mode */
#define TEST_WITH_RELOAD   1      /*!< example without auto-reload mode */

typedef struct {
    int type;                  /*!< event type */
    timer_group_t group;       /*!< timer group */
    timer_idx_t idx;           /*!< timer number */
    uint64_t counter_val;      /*!< timer counter value */
    double time_sec;           /*!< calculated time from counter value */
} timer_event_t;

xQueueHandle timer_queue;

/*
 * @brief Print a uint64_t value
 */
static void inline print_u64(uint64_t val)
{
    printf("0x%08x%08x\n", (uint32_t) (val >> 32), (uint32_t) (val));
}

void timer_evt_task(void *arg)
{
    while(1) {
        timer_event_t evt;
        xQueueReceive(timer_queue, &evt, portMAX_DELAY);
        uint64_t timer_val;
        timer_get_counter_value(evt.group, evt.idx, &timer_val);
        double time;
        timer_get_counter_time_sec(evt.group, evt.idx, &time);
        if(evt.type == TEST_WITHOUT_RELOAD) {
            printf("\n\n   example of count-up-timer \n");
        } else if(evt.type == TEST_WITH_RELOAD) {
            printf("\n\n   example of reload-timer \n");

        }
        /*Show timer event from interrupt*/
        printf("-------INTR TIME EVT--------\n");
        printf("TG[%d] timer[%d] alarm evt\n", evt.group, evt.idx);
        printf("reg: ");
        print_u64(evt.counter_val);
        printf("time: %.8f S\n", evt.time_sec);
        /*Read timer value from task*/
        printf("======TASK TIME======\n");
        printf("TG[%d] timer[%d] alarm evt\n", evt.group, evt.idx);
        printf("reg: ");
        print_u64(timer_val);
        printf("time: %.8f S\n", time);
    }
}

int times = 0;
uint32_t alarmTime = 5000;
bool resetAlarmTime = false;

#define HI32(x) ((uint32_t)((x >> 32) & 0xffffffff))
#define LO32(x) ((uint32_t)(x & 0xffffffff))

void IRAM_ATTR cycle_isr(void* arg)
{
	static uint64_t last_time;
	static uint64_t cycle_times[10];
	uint64_t total_times = 0;
	extern void db_pulse(void);

	(void) arg;

	// go get the timer0 from group 0 counter and figure out the difference and keep a 10 entry average

    TIMERG0.hw_timer[TIMER_0].update = 1;

    /* We don't call a API here because they are not declared with IRAM_ATTR.
       If we're okay with the timer irq not being serviced while SPI flash cache is disabled,
       we can alloc this interrupt without the ESP_INTR_FLAG_IRAM flag and use the normal API. */
//    TIMERG0.int_clr_timers.t0 = 1;
    uint64_t timer_val = ((uint64_t) TIMERG0.hw_timer[TIMER_0].cnt_high) << 32
        | TIMERG0.hw_timer[TIMER_0].cnt_low;

    // what is the difference from last time.

    uint64_t delta_time = timer_val - last_time;
    last_time = timer_val;

    // here is where I need to store X entries

    for( int x = 8 ; x >= 0; x-- )
    {
    	// move all the entries by one and lose the oldest

    	cycle_times[x + 1] = cycle_times[x];
    	total_times += cycle_times[x];
    }

    cycle_times[0] = delta_time;

    total_times += delta_time;

/*    if( (++times) % 20 == 0)
    {
    	printf("tval = 0x%08x%08x delta_time = 0x%08x%08x\n", HI32(total_times), LO32(total_times),
    			HI32(delta_time), LO32(delta_time));
    }
*/
    total_times = (total_times / 1000) - 18;   // 10 for the number of samples and 100 times more for the number of lines
    alarmTime = total_times;

/*    if( (times) % 20 == 0)
    {
    	printf("tval = 0x%08x%08x ltime = 0x%08x%08x times = %d\n", total_times);
    }
*/

    if (alarmTime > 4500)
    {
    	// now we have the average cycle timer. If there is DMA going on, we should stop it.????
    	// set the new alarm value to something really small, after the first interrupt we will update it.

    	TIMERG0.hw_timer[TIMER_1].alarm_high = (uint32_t) 0;
    	TIMERG0.hw_timer[TIMER_1].alarm_low = (uint32_t) 50;

    	// tell the interrupt to update the alarm

    	resetAlarmTime = true;

    	// clear the counter
    	TIMERG0.hw_timer[TIMER_1].load_high = (uint32_t) 0;
    	TIMERG0.hw_timer[TIMER_1].load_low = (uint32_t) 0;
    	TIMERG0.hw_timer[TIMER_1].reload = 1;

    	// enable the alarm

    	TIMERG0.hw_timer[TIMER_1].config.alarm_en = 1;

    	// enable the timer

    	TIMERG0.hw_timer[TIMER_1].config.enable = 1;

    	// setup to show the first line
    	{
    		extern void kick_all(void);

    		kick_all();
    		db_pulse();
    	}
    }
}
/*
 * @brief timer group0 ISR handler
 */
void IRAM_ATTR timer_group0_isr(void *para)
{
    int timer_idx = (int) para;
    uint32_t intr_status = TIMERG0.int_st_timers.val;
//    timer_event_t evt;
	extern void dc_pulse(void);

    dc_pulse();

    if((intr_status & BIT(timer_idx)) && timer_idx == TIMER_0) {
#if 0
        /*Timer0 is an example that doesn't reload counter value*/
        TIMERG0.hw_timer[timer_idx].update = 1;

        /* We don't call a API here because they are not declared with IRAM_ATTR.
           If we're okay with the timer irq not being serviced while SPI flash cache is disabled,
           we can alloc this interrupt without the ESP_INTR_FLAG_IRAM flag and use the normal API. */
        TIMERG0.int_clr_timers.t0 = 1;
        uint64_t timer_val = ((uint64_t) TIMERG0.hw_timer[timer_idx].cnt_high) << 32
            | TIMERG0.hw_timer[timer_idx].cnt_low;
        double time = (double) timer_val / (TIMER_BASE_CLK / TIMERG0.hw_timer[timer_idx].config.divider);

        /*Post an event to out example task*/
        evt.type = TEST_WITHOUT_RELOAD;
        evt.group = (timer_group_t) 0;
        evt.idx = (timer_idx_t) timer_idx;
        evt.counter_val = timer_val;
        evt.time_sec = time;
        xQueueSendFromISR(timer_queue, &evt, NULL);

        /*For a timer that will not reload, we need to set the next alarm value each time. */
        timer_val +=
            (uint64_t) (TIMER_INTERVAL0_SEC * (TIMER_BASE_CLK / TIMERG0.hw_timer[timer_idx].config.divider));
        /*Fine adjust*/
        timer_val -= TIMER_FINE_ADJ;
        TIMERG0.hw_timer[timer_idx].alarm_high = (uint32_t) (timer_val >> 32);
        TIMERG0.hw_timer[timer_idx].alarm_low = (uint32_t) timer_val;
        /*After set alarm, we set alarm_en bit if we want to enable alarm again.*/
        TIMERG0.hw_timer[timer_idx].config.alarm_en = 1;
#endif
    } else if((intr_status & BIT(timer_idx)) && timer_idx == TIMER_1) {
        /*Timer1 is an example that will reload counter value*/
//        TIMERG0.hw_timer[timer_idx].update = 1;
        /*We don't call a API here because they are not declared with IRAM_ATTR*/
        TIMERG0.int_clr_timers.t1 = 1;

        if (resetAlarmTime)
        {
        	// set the new alarm value
        	TIMERG0.hw_timer[TIMER_1].alarm_high = (uint32_t) 0;
        	TIMERG0.hw_timer[TIMER_1].alarm_low = alarmTime;

        	// clear the counter
        	TIMERG0.hw_timer[TIMER_1].load_high = (uint32_t) 0;
        	TIMERG0.hw_timer[TIMER_1].load_low = (uint32_t) 0;
        	TIMERG0.hw_timer[TIMER_1].reload = 1;
        }
#if 0
        uint64_t timer_val = ((uint64_t) TIMERG0.hw_timer[timer_idx].cnt_high) << 32
            | TIMERG0.hw_timer[timer_idx].cnt_low;
        double time = (double) timer_val / (TIMER_BASE_CLK / TIMERG0.hw_timer[timer_idx].config.divider);
        /*Post an event to out example task*/
        evt.type = TEST_WITH_RELOAD;
        evt.group = (timer_group_t) 0;
        evt.idx = (timer_idx_t) timer_idx;
        evt.counter_val = timer_val;
        evt.time_sec = time;
        xQueueSendFromISR(timer_queue, &evt, NULL);
#endif

		// I want to start my SPI dma so call a func
		{
        	extern int32_t start_all_spi(void);

        	if (start_all_spi() < 100)
        	{
                /*For a auto-reload timer, we still need to set alarm_en bit if we want to enable alarm again.*/
                TIMERG0.hw_timer[timer_idx].config.alarm_en = 1;
        	}
        	else
        	{
        		// I think I should pause the timer

        		TIMERG0.hw_timer[timer_idx].config.enable = 0;
        	}
		}
    }
}

/*
 * @brief timer group0 hardware timer0 init
 */
void tg0_timer0_init()
{
    timer_group_t timer_group = TIMER_GROUP_0;
    timer_idx_t timer_idx = TIMER_0;
    timer_config_t config;
    config.alarm_en = 0;
    config.auto_reload = 0;
    config.counter_dir = TIMER_COUNT_UP;
    config.divider = TIMER_DIVIDER;
//    config.intr_type = TIMER_INTR_SEL;
    config.counter_en = TIMER_PAUSE;
    /*Configure timer*/
    timer_init(timer_group, timer_idx, &config);
    /*Stop timer counter*/
    timer_pause(timer_group, timer_idx);
    /*Load counter value */
    timer_set_counter_value(timer_group, timer_idx, 0x00000000ULL);
    /*Set alarm value*/
//    timer_set_alarm_value(timer_group, timer_idx, TIMER_INTERVAL0_SEC * TIMER_SCALE - TIMER_FINE_ADJ);
    /*Enable timer interrupt*/
//    timer_enable_intr(timer_group, timer_idx);
    /*Set ISR handler*/
//    timer_isr_register(timer_group, timer_idx, timer_group0_isr, (void*) timer_idx, ESP_INTR_FLAG_IRAM, NULL);
    /*Start timer counter*/
    timer_start(timer_group, timer_idx);
}

/*
 * @brief timer group0 hardware timer1 init
 */
void tg0_timer1_init()
{
    timer_group_t timer_group = TIMER_GROUP_0;
    timer_idx_t timer_idx = TIMER_1;
    timer_config_t config;
    config.alarm_en = 1;
    config.auto_reload = 1;
    config.counter_dir = TIMER_COUNT_UP;
    config.divider = TIMER_DIVIDER;
    config.intr_type = TIMER_INTR_SEL;
    config.counter_en = TIMER_PAUSE;
    /*Configure timer*/
    timer_init(timer_group, timer_idx, &config);
    /*Stop timer counter*/
    timer_pause(timer_group, timer_idx);
    /*Load counter value */
    timer_set_counter_value(timer_group, timer_idx, 0x00000000ULL);
    /*Set alarm value*/
    timer_set_alarm_value(timer_group, timer_idx, TIMER_INTERVAL1_SEC * TIMER_SCALE);
    /*Enable timer interrupt*/
    timer_enable_intr(timer_group, timer_idx);
    /*Set ISR handler*/
    timer_isr_register(timer_group, timer_idx, timer_group0_isr, (void*) timer_idx, ESP_INTR_FLAG_IRAM, NULL);
    /*Start timer counter*/
    timer_start(timer_group, timer_idx);
}

void timer_set_new_alarm(uint32_t alarm_value)
{
	// set the new alarm value
	TIMERG0.hw_timer[TIMER_1].alarm_high = (uint32_t) 0;
	TIMERG0.hw_timer[TIMER_1].alarm_low = (uint32_t) alarm_value;

	// clear the counter
	TIMERG0.hw_timer[TIMER_1].load_high = (uint32_t) 0;
	TIMERG0.hw_timer[TIMER_1].load_low = (uint32_t) 0;
	TIMERG0.hw_timer[TIMER_1].reload = 1;

	// enable the alarm

    TIMERG0.hw_timer[TIMER_1].config.alarm_en = 1;

    // enable the timer

	TIMERG0.hw_timer[TIMER_1].config.enable = 1;
}
/**
 * @brief In this test, we will test hardware timer0 and timer1 of timer group0.
 */
void timer_app_main()
{
//    timer_queue = xQueueCreate(10, sizeof(timer_event_t));
    tg0_timer0_init();
    tg0_timer1_init();
//    xTaskCreate(timer_evt_task, "timer_evt_task", 2048, NULL, configMAX_PRIORITIES - 1, NULL);
}


