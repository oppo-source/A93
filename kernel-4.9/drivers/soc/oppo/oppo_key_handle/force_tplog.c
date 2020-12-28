/*
 * Copyright (C), 2019, OPPO Mobile Comm Corp., Ltd.
 * VENDOR_EDIT
 * File: - oppo_key_handle/force_tplog.c
 * Description: Util about dump in the period of debugging.
 * Version: 1.0
 * Date: 2019/11/01
 * Author: Bin.Xu@BSP.Kernel.Stability
 *
 *----------------------Revision History: ---------------------------
 *   <author>        <date>         <version>         <desc>
 *    Bin.Xu       2019/11/01        1.0              created
 *-------------------------------------------------------------------
 */
#include <linux/kernel.h>
#include <linux/bug.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/kthread.h>
#include <uapi/linux/input-event-codes.h>
#include <linux/oppo_key_handle.h>

#define OPPO_TPLG_MASK 0x0079
#define OPPO_TPLG_TEXT "OPPO_FORCE_TPLG"

static struct task_struct *tplg_t = NULL;
static struct input_dev *tplg_dev = NULL;

int tplg_thread(void *dev) {
	char *env[2] = {"TPLG_STATUS=ON", NULL};
	while(!kthread_should_stop()) {
		if (OPPO_KEY_TASKS[OPPO_KEY_TPLG].open == 0) {
			pr_info("tplg_door_open \n");
			OPPO_KEY_TASKS[OPPO_KEY_TPLG].open = 1;
			if(kobject_uevent_env(&(*(struct input_dev **)dev)->dev.kobj, KOBJ_CHANGE, env))
				pr_err("tplg open uevent failed!");
		}
		set_current_state(TASK_INTERRUPTIBLE);
		schedule();
	}
	return 0;
}

void tplg_process(struct input_dev *dev, int key, int val)
{
	if (OPPO_KEY_TASKS[OPPO_KEY_TPLG].open == 0) {
		tplg_dev = dev;
		wake_up_process(tplg_t);
	}
}

void tplg_close_process(void) {
	if (OPPO_KEY_TASKS[OPPO_KEY_TPLG].open == 0) {
		// TO DO if needed
	}
}

void tplg_post_process(struct input_dev *dev, int key, int val)
{
	if (OPPO_KEY_TASKS[OPPO_KEY_TPLG].open) {
		// TO DO if needed
	}
}

static int __init oppo_tplg_init(void)
{
	// init tplg thread
	tplg_t = kthread_create(tplg_thread, &tplg_dev, "tplg_send");
	if(IS_ERR(tplg_t)) {
		pr_err("Creating Tp log thread fails!\n");
		return -1;
	}
	tplg_t->state = TASK_INTERRUPTIBLE;

	// register the Tp log function
	register_oppo_key_task(OPPO_KEY_TPLG,
	                        false,
	                        OPPO_TPLG_MASK,
	                        OPPO_TPLG_TEXT,
	                        tplg_process,
	                        tplg_post_process,
	                        tplg_close_process);
 	return 0;
}

device_initcall(oppo_tplg_init);