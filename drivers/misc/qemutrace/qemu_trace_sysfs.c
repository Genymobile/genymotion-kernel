/* drivers/misc/qemu_sysfs.c
 *
 * Copyright (C) 2007-2008 Google, Inc.
 * Author: Jack Veenstra
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/list.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/sysdev.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/wait.h>
#include "qemu_trace.h"

MODULE_DESCRIPTION("Qemu Trace Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");

static struct kobject *qemu_trace_kobj;

static ssize_t state_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
    int val = qemu_trace_get_tracing();
    buf[0] = '0' + val;
    buf[1] = '\n';
    return 2;
}

static ssize_t state_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
    if (n <= 0)
	return -EINVAL;
    if (buf[0] == '0')
        qemu_trace_stop();
    else if (buf[0] == '1')
        qemu_trace_start();
    else
	return -EINVAL;
    return n;
}

static ssize_t symbol_show(struct kobject *kobj, struct kobj_attribute *attr, char * buf)
{
    return 0;
}

// We are expecting a string of the form "addr symbol" where 'addr' is a hex address
// (without the leading '0x') and symbol is a newline-terminated string.  This symbol
// with its corresponding address will be added to the trace file.
//
// To remove the mapping for (addr, symbol) in the trace file, write just the
// address.  As before, the address is in hex without the leading '0x'.  It can
// be newline-terminated or zero-terminated.
static ssize_t symbol_store(struct kobject *kobj, struct kobj_attribute *attr, const char * buf, size_t n)
{
    const char *cp;
    unsigned int addr = 0;
    int len;
    char *sym;

    if (n <= 0 || buf == NULL)
	return -EINVAL;
    for (cp = buf; *cp != ' '; ++cp) {
        unsigned int digit;

        if (*cp >= '0' && *cp <= '9')
            digit = *cp - '0';
        else if (*cp >= 'a' && *cp <= 'f')
            digit = *cp - 'a' + 10;
        else if (*cp == 0 || *cp == '\n') {
            qemu_trace_remove_mapping(addr);
            return n;
        } else
            return -EINVAL;
        addr = (addr << 4) + digit;
    }
    // Move past the space
    cp += 1;

    // Copy the string to a new buffer so that we can replace the newline
    // with '\0'.
    len = strlen(cp);
    sym = kzalloc(len + 1, GFP_KERNEL);
    strcpy(sym, cp);
    if (sym[len - 1] == '\n')
        sym[len - 1] = 0;

    qemu_trace_add_mapping(addr, sym);
    kfree(sym);
    return n;
}

static ssize_t process_name_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    return 0;
}

/* This expects a string that is the process name.  If the string contains
 * a trailing newline, that is removed in the emulator tracing code because
 * it is simpler to do it there.
 */
static ssize_t process_name_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t n)
{
    if (n <= 0 || buf == NULL)
	return -EINVAL;

    qemu_trace_process_name(buf);
    return n;
}


#define qemu_trace_attr(_name) \
static struct kobj_attribute _name##_attr = {	\
	.attr	= {				\
		.name = __stringify(_name),	\
		.mode = 0666,			\
	},					\
	.show	= _name##_show,			\
	.store	= _name##_store,		\
}

qemu_trace_attr(state);
qemu_trace_attr(symbol);
qemu_trace_attr(process_name);

static struct attribute * qemu_trace_attrs[] = {
	&state_attr.attr,
	&symbol_attr.attr,
	&process_name_attr.attr,
	NULL,
};

static struct attribute_group qemu_trace_attr_group = {
	.attrs = qemu_trace_attrs,
};

static int __init qemu_trace_init(void)
{
	int ret;

	qemu_trace_kobj = kobject_create_and_add("qemu_trace", NULL);
	if (qemu_trace_kobj == NULL) {
		printk("qemu_trace_init: kobject_create_and_add failed\n");
		ret = -ENOMEM;
		return ret;
	}
	ret = sysfs_create_group(qemu_trace_kobj, &qemu_trace_attr_group);
	if (ret) {
		printk("qemu_trace_init: sysfs_create_group failed\n");
		goto err;
	}

	return 0;

err:
	kobject_del(qemu_trace_kobj);
	qemu_trace_kobj = NULL;
	return ret;
}

static void  __exit qemu_trace_exit(void)
{
	sysfs_remove_group(qemu_trace_kobj, &qemu_trace_attr_group);
	kobject_del(qemu_trace_kobj);
}

core_initcall(qemu_trace_init);
module_exit(qemu_trace_exit);
