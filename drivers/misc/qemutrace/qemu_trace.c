/* drivers/misc/qemutrace/qemu_trace.c
 *
 * Copyright (C) 2007-2008 Google, Inc.
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/miscdevice.h>
#include <linux/pci.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include "qemu_trace.h"

/* trace device registers */
#define TRACE_DEV_REG_SWITCH            0
#define TRACE_DEV_REG_FORK              1
#define TRACE_DEV_REG_EXECVE_PID        2
#define TRACE_DEV_REG_EXECVE_VMSTART    3
#define TRACE_DEV_REG_EXECVE_VMEND      4
#define TRACE_DEV_REG_EXECVE_OFFSET     5
#define TRACE_DEV_REG_EXECVE_EXEPATH    6
#define TRACE_DEV_REG_EXIT              7
#define TRACE_DEV_REG_CMDLINE           8
#define TRACE_DEV_REG_CMDLINE_LEN       9
#define TRACE_DEV_REG_MMAP_EXEPATH      10
#define TRACE_DEV_REG_INIT_PID          11
#define TRACE_DEV_REG_INIT_NAME         12
#define TRACE_DEV_REG_CLONE             13
#define TRACE_DEV_REG_UNMAP_START       14
#define TRACE_DEV_REG_UNMAP_END         15
#define TRACE_DEV_REG_NAME              16
#define TRACE_DEV_REG_TGID              17
#define TRACE_DEV_REG_DYN_SYM           50
#define TRACE_DEV_REG_DYN_SYM_ADDR      51
#define TRACE_DEV_REG_REMOVE_ADDR       52
#define TRACE_DEV_REG_ENABLE            100

static unsigned char __iomem *qt_base;
static int init_called;
static uint32_t qemu_trace_paddr;

/* PIDs that start before our device registered */
#define MAX_INIT_PIDS   2048
static int tb_next = 0;
static int init_pids[MAX_INIT_PIDS];
static DEFINE_SPINLOCK(qemu_trace_lock);

void qemu_trace_start(void)
{
	unsigned long irq_flags;

	if (qt_base == NULL)
		return;
	spin_lock_irqsave(&qemu_trace_lock, irq_flags);
	writel(1, qt_base + (TRACE_DEV_REG_ENABLE << 2));
	spin_unlock_irqrestore(&qemu_trace_lock, irq_flags);
}

void qemu_trace_stop(void)
{
	unsigned long irq_flags;

	if (qt_base == NULL)
		return;
	spin_lock_irqsave(&qemu_trace_lock, irq_flags);
	writel(0, qt_base + (TRACE_DEV_REG_ENABLE << 2));
	spin_unlock_irqrestore(&qemu_trace_lock, irq_flags);
}

int qemu_trace_get_tracing(void)
{
	int val = 0;
	if (qt_base != NULL)
		val = readl(qt_base + (TRACE_DEV_REG_ENABLE << 2));
	return val;
}

void qemu_trace_add_mapping(unsigned int addr, const char *symbol)
{
	unsigned long irq_flags;

	if (qt_base == NULL)
		return;

	/* Write the address first, then the symbol name. */
	spin_lock_irqsave(&qemu_trace_lock, irq_flags);
	writel(addr, qt_base + (TRACE_DEV_REG_DYN_SYM_ADDR << 2));
	writel(symbol, qt_base + (TRACE_DEV_REG_DYN_SYM << 2));
	spin_unlock_irqrestore(&qemu_trace_lock, irq_flags);
}

void qemu_trace_remove_mapping(unsigned int addr)
{
	unsigned long irq_flags;

	if (qt_base == NULL)
		return;

	spin_lock_irqsave(&qemu_trace_lock, irq_flags);
	writel(addr, qt_base + (TRACE_DEV_REG_REMOVE_ADDR << 2));
	spin_unlock_irqrestore(&qemu_trace_lock, irq_flags);
}

/* trace the context switch */
void qemu_trace_cs(struct task_struct *next)
{
	unsigned long irq_flags;

	if (qt_base == NULL)
		return;

	spin_lock_irqsave(&qemu_trace_lock, irq_flags);
	writel(task_pid_nr(next), qt_base);
	spin_unlock_irqrestore(&qemu_trace_lock, irq_flags);
}
EXPORT_SYMBOL(qemu_trace_cs);

/* trace the execve */
void qemu_trace_execve(int argc, char __user * __user *argv)
{
	unsigned long irq_flags;
	char page[PAGE_SIZE];
	char *ptr = page;
	int remaining = sizeof(page);

	if (qt_base == NULL)
		return;

	while (argc-- > 0 && remaining > 1) {
		char __user *str;
		int len;
		if (get_user(str, argv ++))
			return;
		len = strnlen_user(str, remaining-1);
		if (len == 0)
			break; /* end of argv list */
		if (copy_from_user(ptr, str, len))
			return;
		ptr += len;
		*ptr++ = '\0';
		remaining -= len + 1;
	}

	if (ptr > page) {
		int len = ptr - page;
		spin_lock_irqsave(&qemu_trace_lock, irq_flags);
		writel(len, qt_base + (TRACE_DEV_REG_CMDLINE_LEN << 2));
		writel(page, qt_base + (TRACE_DEV_REG_CMDLINE << 2));
		spin_unlock_irqrestore(&qemu_trace_lock, irq_flags);
	}
}
EXPORT_SYMBOL(qemu_trace_execve);

/* trace the mmap */
void qemu_trace_mmap(struct vm_area_struct *vma)
{
	unsigned long irq_flags;
	char page[PAGE_SIZE];
	char *p;

	if (qt_base == NULL)
		return;

	if (vma->vm_file == NULL)
		return;

	p = d_path(&vma->vm_file->f_path, page, PAGE_SIZE);
	if (IS_ERR(p))
		return;

	spin_lock_irqsave(&qemu_trace_lock, irq_flags);
	writel(vma->vm_start, qt_base + (TRACE_DEV_REG_EXECVE_VMSTART << 2));
	writel(vma->vm_end, qt_base + (TRACE_DEV_REG_EXECVE_VMEND << 2));
	writel(vma->vm_pgoff * PAGE_SIZE, qt_base + (TRACE_DEV_REG_EXECVE_OFFSET << 2));
	writel(p, qt_base + (TRACE_DEV_REG_MMAP_EXEPATH << 2));
	spin_unlock_irqrestore(&qemu_trace_lock, irq_flags);
}
EXPORT_SYMBOL(qemu_trace_mmap);

/* trace the munmap */
void qemu_trace_munmap(unsigned long start, unsigned long end)
{
	unsigned long irq_flags;

	if (qt_base == NULL)
		return;

	spin_lock_irqsave(&qemu_trace_lock, irq_flags);
	writel(start, qt_base + (TRACE_DEV_REG_UNMAP_START << 2));
	writel(end, qt_base + (TRACE_DEV_REG_UNMAP_END << 2));
	spin_unlock_irqrestore(&qemu_trace_lock, irq_flags);
}
EXPORT_SYMBOL(qemu_trace_munmap);

/* trace the fork */
void qemu_trace_fork(struct task_struct *forked, unsigned long clone_flags)
{
	unsigned long irq_flags;

	spin_lock_irqsave(&qemu_trace_lock, irq_flags);
	if (qt_base == NULL) {
		if (tb_next >= MAX_INIT_PIDS) {
			if (!init_called)
				printk(KERN_ERR
				       "QEMU Trace: too many PIDs before "
				       "device registered ignoring %d\n",
				       forked->pid);
		} else {
			init_pids[tb_next] = task_pid_nr(forked);
			tb_next++;
		}
	} else {
		writel(task_tgid_nr(forked), qt_base + (TRACE_DEV_REG_TGID << 2));
		if (clone_flags & CLONE_VM)
			writel(task_pid_nr(forked), qt_base + (TRACE_DEV_REG_CLONE << 2));
		else
			writel(task_pid_nr(forked), qt_base + (TRACE_DEV_REG_FORK << 2));
	}
	spin_unlock_irqrestore(&qemu_trace_lock, irq_flags);
}
EXPORT_SYMBOL(qemu_trace_fork);

/* trace the exit */
void qemu_trace_exit(int code)
{
	unsigned long irq_flags;

	if (qt_base == NULL)
		return;

	spin_lock_irqsave(&qemu_trace_lock, irq_flags);
	writel(code, qt_base + (TRACE_DEV_REG_EXIT << 2));
	spin_unlock_irqrestore(&qemu_trace_lock, irq_flags);
}
EXPORT_SYMBOL(qemu_trace_exit);

/* trace the thread name */
void qemu_trace_thread_name(const char *name)
{
	unsigned long irq_flags;

	if (qt_base == NULL)
		return;

	spin_lock_irqsave(&qemu_trace_lock, irq_flags);
	writel(name, qt_base + (TRACE_DEV_REG_NAME << 2));
	spin_unlock_irqrestore(&qemu_trace_lock, irq_flags);
}
EXPORT_SYMBOL(qemu_trace_thread_name);

/* trace the process name */
void qemu_trace_process_name(const char *name)
{
	unsigned long irq_flags;

	if (qt_base == NULL)
		return;

	spin_lock_irqsave(&qemu_trace_lock, irq_flags);
	writel(name, qt_base + (TRACE_DEV_REG_NAME << 2));
	spin_unlock_irqrestore(&qemu_trace_lock, irq_flags);
}
EXPORT_SYMBOL(qemu_trace_process_name);

static void qemu_trace_pid_exec(struct task_struct *tsk)
{
	unsigned long irq_flags;
	char page[PAGE_SIZE];
	struct mm_struct *mm = get_task_mm(tsk);
	if (mm == NULL)
		return;
	down_read(&mm->mmap_sem);
	{
		struct vm_area_struct *vma = mm->mmap;
		while (vma) {
			if ((vma->vm_flags & VM_EXEC) && vma->vm_file) {
				char *p;
				p = d_path(&vma->vm_file->f_path, page, PAGE_SIZE);
				if (!IS_ERR(p)) {
					spin_lock_irqsave(&qemu_trace_lock, irq_flags);
					writel(vma->vm_start, qt_base + (TRACE_DEV_REG_EXECVE_VMSTART << 2));
					writel(vma->vm_end, qt_base + (TRACE_DEV_REG_EXECVE_VMEND << 2));
					writel(vma->vm_pgoff * PAGE_SIZE, qt_base + (TRACE_DEV_REG_EXECVE_OFFSET << 2));
					writel(p, qt_base + (TRACE_DEV_REG_EXECVE_EXEPATH << 2));
					spin_unlock_irqrestore(&qemu_trace_lock, irq_flags);
				}
			}
			vma = vma->vm_next;
		}
	}
	up_read(&mm->mmap_sem);
	mmput(mm);
}

static void qemu_trace_dump_init_threads(void)
{
	unsigned long irq_flags;
	int i;

	for (i = 0; i < tb_next; i++) {
		struct task_struct *tsk;
		struct pid *pid = find_get_pid(init_pids[i]);
		if (pid == NULL)
			continue;

		if ((tsk = get_pid_task(pid, PIDTYPE_PID)) != NULL) {
			/* first give the pid and name */
			task_lock(tsk);
			spin_lock_irqsave(&qemu_trace_lock, irq_flags);
			writel(task_tgid_nr(tsk), qt_base + (TRACE_DEV_REG_TGID << 2));
			writel(task_pid_nr(tsk), qt_base + (TRACE_DEV_REG_INIT_PID << 2));
			writel(tsk->comm, qt_base + (TRACE_DEV_REG_INIT_NAME << 2));
			spin_unlock_irqrestore(&qemu_trace_lock, irq_flags);
			task_unlock(tsk);
			/* check if the task has execs */
			qemu_trace_pid_exec(tsk);
		}
	}
}

static int qemu_trace_mmap_fop(struct file *file, struct vm_area_struct *vma)
{
	int ret = io_remap_pfn_range(vma, vma->vm_start,
			(qemu_trace_paddr >> PAGE_SHIFT) + 1,
			PAGE_SIZE, vma->vm_page_prot);
	if (ret < 0)
		return ret;
	return 0;
}

static const struct file_operations qemu_trace_fops = {
	.owner = THIS_MODULE,
	.mmap = qemu_trace_mmap_fop,
};

static struct miscdevice qemu_trace_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "qemu_trace",
	.fops = &qemu_trace_fops,
};

static int qemu_trace_probe(struct platform_device *pdev)
{
	int err;
	struct resource *r;

	/* not thread safe, but this should not happen */
	if (qt_base != NULL) {
		printk(KERN_ERR "QEMU TRACE Device: already mapped at %p\n", qt_base);
		return -ENODEV;
	}
	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (r == NULL || r->end - r->start < 2 * PAGE_SIZE - 1)
		return -EINVAL;
	qemu_trace_paddr = r->start;
	qt_base = ioremap(r->start, PAGE_SIZE);
	printk(KERN_INFO "QEMU TRACE Device: The mapped IO base is %p\n", qt_base);

	qemu_trace_dump_init_threads();
	err = misc_register(&qemu_trace_device);
	if (err)
		goto err_misc_register;

	return 0;

err_misc_register:
	iounmap(qt_base);
	qt_base = NULL;
	return err;
}

static int qemu_trace_remove(struct platform_device *pdev)
{
	misc_deregister(&qemu_trace_device);
	iounmap(qt_base);
	qt_base = NULL;
	return 0;
}

static struct platform_driver qemu_trace = {
	.probe = qemu_trace_probe,
	.remove = qemu_trace_remove,
	.driver = {
		.name = "qemu_trace"
	}
};

static int __init qemu_trace_dev_init(void)
{
	int ret;
	ret = platform_driver_register(&qemu_trace);
	init_called = 1;
	return ret;
}

static void qemu_trace_dev_exit(void)
{
	platform_driver_unregister(&qemu_trace);
}


module_init(qemu_trace_dev_init);
module_exit(qemu_trace_dev_exit);

MODULE_AUTHOR("Ye Wen <ywen@google.com>");
MODULE_LICENSE("GPL");
