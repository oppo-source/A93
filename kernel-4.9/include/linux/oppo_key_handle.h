/*
 * Copyright (C), 2019, OPPO Mobile Comm Corp., Ltd.
 * VENDOR_EDIT
 * File: - linux/oppo_key_handle.h
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
#include <linux/kobject.h>
#include <linux/input.h>

#define DEFAULT_BASE 0xFF00

typedef enum {
	OPPO_KEY_DUMP,
	OPPO_KEY_TPLG,
	OPPO_KEY_NUM
} oppo_key_handle_t;

struct oppo_key_task_struct {
	bool open;
	u16  passwd;
	char *name;
	void (*process)(struct input_dev *dev, int key, int val);
	void (*post_process)(struct input_dev *dev, int key, int val);
	void (*close_process)(void);
};

extern struct oppo_key_task_struct OPPO_KEY_TASKS[OPPO_KEY_NUM];

extern void oppo_key_event(struct input_dev *dev, int type, int key, int val);
extern void get_passwd_base(const char *val);
extern void register_oppo_key_task(oppo_key_handle_t type,
                                    bool open,
                                    u16 mask,
                                    char* name,
                                    void *process,
                                    void *post_process,
                                    void *close_process);
// Added by module

extern void OPPO_DUMP(const char s[24]);