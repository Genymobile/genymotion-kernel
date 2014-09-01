/* drivers/misc/qemutrace/qemu_trace.h
 *
 * Copyright (C) 2007-2008 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 *
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

void qemu_trace_start(void);
void qemu_trace_stop(void);
int qemu_trace_get_tracing(void);
void qemu_trace_add_mapping(unsigned int addr, const char *symbol);
void qemu_trace_remove_mapping(unsigned int addr);
void qemu_trace_process_name(const char *name);
