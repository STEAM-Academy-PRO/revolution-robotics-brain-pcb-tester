/**
 * \file
 *
 * \brief Timer functionality implementation.
 *
 * Copyright (c) 2014-2018 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 *
 */

#include "hal_timer.h"
#include <utils_assert.h>
#include <utils.h>
#include <hal_atomic.h>
#include <hpl_irq.h>

/**
 * \brief Driver version
 */
#define DRIVER_VERSION 0x00000001u

/**
 * \brief Timer flags
 */
#define TIMER_FLAG_QUEUE_IS_TAKEN 1
#define TIMER_FLAG_INTERRUPT_TRIGERRED 2

static void timer_add_timer_task(struct list_descriptor *list, struct timer_task *const new_task, const uint32_t time);
static void timer_process_counted(struct _timer_device *device);
static void timer_error(struct _timer_device *device);
static void capture_chan0(struct _timer_device *device);
static void capture_chan1(struct _timer_device *device);

/**
 * \brief Initialize timer
 */
int32_t timer_init(struct timer_descriptor *const descr, void *const hw, struct _timer_hpl_interface *const func)
{
	ASSERT(descr && hw && func);
	descr->func = func;
	descr->func->init(&descr->device, hw);
	descr->time                           = 0;
	descr->device.timer_cb.period_expired = timer_process_counted;
	descr->device.timer_cb.error_detect = timer_error;
	descr->device.timer_cb.capture_chan0 = capture_chan0;
	descr->device.timer_cb.capture_chan1 = capture_chan1;


	return ERR_NONE;
}

/**
 * \brief Deinitialize timer
 */
int32_t timer_deinit(struct timer_descriptor *const descr)
{
	ASSERT(descr && descr->func);
	descr->func->deinit(&descr->device);

	return ERR_NONE;
}

/**
 * \brief Start timer
 */
int32_t timer_start(struct timer_descriptor *const descr)
{
	ASSERT(descr && descr->func);
	if (descr->func->is_timer_started(&descr->device)) {
		return ERR_DENIED;
	}
	descr->func->start_timer(&descr->device);

	return ERR_NONE;
}

/**
 * \brief Stop timer
 */
int32_t timer_stop(struct timer_descriptor *const descr)
{
	ASSERT(descr && descr->func);
	if (!descr->func->is_timer_started(&descr->device)) {
		return ERR_DENIED;
	}
	descr->func->stop_timer(&descr->device);

	return ERR_NONE;
}

/**
 * \brief Set amount of clock cycler per timer tick
 */
int32_t timer_set_clock_cycles_per_tick(struct timer_descriptor *const descr, const uint32_t clock_cycles)
{
	ASSERT(descr && descr->func);
	descr->func->set_timer_period(&descr->device, clock_cycles);

	return ERR_NONE;
}

int32_t timer_set_chan_compare_value(struct timer_descriptor *const descr, const uint8_t chan, const uint32_t value)
{
	ASSERT(descr && descr->func);
	descr->func->set_compare_value(&descr->device, value, chan);
	return ERR_NONE;
}
/**
 * \brief Add timer task
 */
int32_t timer_add_task(struct timer_descriptor *const descr, struct timer_task *const task)
{
	ASSERT(descr && task && descr->func);

	descr->flags |= TIMER_FLAG_QUEUE_IS_TAKEN;
	if (is_list_element(&descr->tasks, task)) {
		descr->flags &= ~TIMER_FLAG_QUEUE_IS_TAKEN;
		ASSERT(false);
		return ERR_ALREADY_INITIALIZED;
	}
	task->time_label = descr->time;
	timer_add_timer_task(&descr->tasks, task, descr->time);

	descr->flags &= ~TIMER_FLAG_QUEUE_IS_TAKEN;
	if (descr->flags & TIMER_FLAG_INTERRUPT_TRIGERRED) {
		CRITICAL_SECTION_ENTER()
		descr->flags &= ~TIMER_FLAG_INTERRUPT_TRIGERRED;
		descr->func->set_timer_irq(&descr->device);
		CRITICAL_SECTION_LEAVE()
	}

	return ERR_NONE;
}

/**
 * \brief Remove timer task
 */
int32_t timer_remove_task(struct timer_descriptor *const descr, const struct timer_task *const task)
{
	ASSERT(descr && task && descr->func);

	descr->flags |= TIMER_FLAG_QUEUE_IS_TAKEN;
	if (!is_list_element(&descr->tasks, task)) {
		descr->flags &= ~TIMER_FLAG_QUEUE_IS_TAKEN;
		//ASSERT(false);
		return ERR_NOT_FOUND;
	}
	list_delete_element(&descr->tasks, task);

	descr->flags &= ~TIMER_FLAG_QUEUE_IS_TAKEN;
	if (descr->flags & TIMER_FLAG_INTERRUPT_TRIGERRED) {
		CRITICAL_SECTION_ENTER()
		descr->flags &= ~TIMER_FLAG_INTERRUPT_TRIGERRED;
		descr->func->set_timer_irq(&descr->device);
		CRITICAL_SECTION_LEAVE()
	}

	return ERR_NONE;
}

/**
 * \brief Retrieve the amount of clock cycles in a tick
 */
int32_t timer_get_clock_cycles_in_tick(const struct timer_descriptor *const descr, uint32_t *const cycles)
{
	ASSERT(descr && cycles && descr->func);
	*cycles = descr->func->get_period(&descr->device);
	return ERR_NONE;
}

/**
 * \brief Retrieve the current driver version
 */
uint32_t timer_get_version(void)
{
	return DRIVER_VERSION;
}

/**
 * \internal Insert a timer task into sorted timer's list
 *
 * \param[in] head The pointer to the head of timer task list
 * \param[in] task The pointer to task to add
 * \param[in] time Current timer time
 */
static void timer_add_timer_task(struct list_descriptor *list, struct timer_task *const new_task, const uint32_t time)
{
	struct timer_task *it, *prev = NULL, *head = (struct timer_task *)list_get_head(list);

	if (!head) {
		list_insert_as_head(list, new_task);
		return;
	}

	for (it = head; it; it = (struct timer_task *)list_get_next_element(it)) {
		uint32_t time_left;

		if (it->time_label <= time) {
			time_left = it->interval - (time - it->time_label);
		} else {
			time_left = it->interval - (0xFFFFFFFF - it->time_label) - time;
		}
		if (time_left >= new_task->interval)
			break;
		prev = it;
	}

	if (it == head) {
		list_insert_as_head(list, new_task);
	} else {
		list_insert_after(prev, new_task);
	}
}

/**
 * \internal Process interrupts
 */
static void timer_process_counted(struct _timer_device *device)
{
	struct timer_descriptor *timer = CONTAINER_OF(device, struct timer_descriptor, device);
	struct timer_task *      it    = (struct timer_task *)list_get_head(&timer->tasks);
	uint32_t                 time  = ++timer->time;

	if ((timer->flags & TIMER_FLAG_QUEUE_IS_TAKEN) || (timer->flags & TIMER_FLAG_INTERRUPT_TRIGERRED)) {
		timer->flags |= TIMER_FLAG_INTERRUPT_TRIGERRED;
		return;
	}

	while (it && ((time - it->time_label) >= it->interval)) {
		struct timer_task *tmp = it;

		list_remove_head(&timer->tasks);
		if (TIMER_TASK_REPEAT == tmp->mode) {
			tmp->time_label = time;
			timer_add_timer_task(&timer->tasks, tmp, time);
		}
		it = (struct timer_task *)list_get_head(&timer->tasks);

		tmp->cb(tmp);
	}
}

static int capt0_count = 0;
static int capt1_count = 0;
static int err_count = 0;

static void timer_error(struct _timer_device *device)
{
	err_count++;

 	struct timer_descriptor *timer = CONTAINER_OF(device, struct timer_descriptor, device);
// 
// 	static uint16_t diff_time = 0;
// 	uint16_t curtime = hri_tccount16_read_CC_reg(device->hw, 0);
	if (timer->cbs[TIMER_ERROR].func)
		timer->cbs[TIMER_ERROR].func(0, timer->cbs[TIMER_ERROR].user_data);
}

static void capture_chan0(struct _timer_device *device)
{
	capt0_count++;

	struct timer_descriptor *timer = CONTAINER_OF(device, struct timer_descriptor, device);

	uint32_t curtime = hri_tccount16_read_CC_reg(device->hw, 0);
	bool ovf = hri_tc_get_interrupt_OVF_bit(device->hw);
	if (ovf)
	{
		hri_tc_clear_interrupt_OVF_bit(device->hw);
	}
// 	uint32_t diff_time=0;
// 	if (ovf)
// 		diff_time = 1;
// 	else if (curtime < device->prev_time_cc0)
// 		diff_time = 2;
// 	else if (curtime == device->prev_time_cc0)
// 		diff_time = 3;
// 	else
// 		diff_time = 4;
	if (timer->cbs[TIMER_MC0].func)
		timer->cbs[TIMER_MC0].func(curtime, timer->cbs[TIMER_MC0].user_data);
	device->prev_time_cc0 = curtime;

}

static void capture_chan1(struct _timer_device *device)
{
	capt1_count++;	

	struct timer_descriptor *timer = CONTAINER_OF(device, struct timer_descriptor, device);

	uint32_t curtime = hri_tccount16_read_CC_reg(device->hw, 1);
	bool ovf = hri_tc_get_interrupt_OVF_bit(device->hw);
	if (ovf)
	{
//		curtime += 0x10000
		hri_tc_clear_interrupt_OVF_bit(device->hw);
	}
//	uint32_t diff_time=0;
// 	if (ovf)
// 		diff_time = 1;
// 	else if (curtime < device->prev_time_cc1)
// 		diff_time = 2;
// 	else if (curtime == device->prev_time_cc1)
// 		diff_time = 3;
// 	else
// 		diff_time = 4;
		//diff_time = curtime - device->prev_time_cc1;
	if (timer->cbs[TIMER_MC1].func)
		timer->cbs[TIMER_MC1].func(curtime, timer->cbs[TIMER_MC1].user_data);
	device->prev_time_cc1 = curtime;
}


int32_t timer_register_cb(struct timer_descriptor *const descr, enum TIMER_CB_FUNC_TUPE type, timer_timer_cb_t func, void* user_data)
{
	if (!descr || !func || type>=TIMER_CB_FUNC_MAX)
		return ERR_INVALID_ARG;
	descr->cbs[type].func = func;
	descr->cbs[type].user_data = user_data;

    return 0;
};

int32_t timer_unregister_cb(struct timer_descriptor *const descr, enum TIMER_CB_FUNC_TUPE type)
{
	if (!descr || type>=TIMER_CB_FUNC_MAX)
		return ERR_INVALID_ARG;
	descr->cbs[type].func = NULL;
	descr->cbs[type].user_data = NULL;

    return 0;
};