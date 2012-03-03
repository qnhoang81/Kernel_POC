/*
 * OpenFIMG Kernel Interface
 *
 * Copyright 2010-2011 Tomasz Figa <tomasz.figa at gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/hrtimer.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/pm_runtime.h>
#include <plat/power_clk_gating.h>
#include <plat/s5p6442-dvfs.h>
#include "s3c_g3d.h"

#define DEBUG

/*
 * Various definitions
 */
#define G3D_AUTOSUSPEND_DELAY		(1000)
#define G3D_TIMEOUT			(1*HZ)

/*
 * Registers
 */
#define G3D_FGGB_PIPESTAT_REG		(0x00)
#define G3D_FGGB_PIPESTAT_MSK		(0x0005171f)

#define G3D_FGGB_CACHECTL_REG		(0x04)
#define G3D_FGGB_FLUSH_MSK		(0x00000033)
#define G3D_FGGB_INVAL_MSK		(0x00001300)

#define G3D_FGGB_RESET_REG		(0x08)
#define G3D_FGGB_VERSION		(0x10)
#define G3D_FGGB_INTPENDING_REG		(0x40)
#define G3D_FGGB_INTMASK_REG		(0x44)
#define G3D_FGGB_PIPEMASK_REG		(0x48)
#define G3D_FGGB_PIPETGTSTATE_REG	(0x4c)

/*
 * Private structures
 */
struct g3d_context;

struct g3d_drvdata {
	void __iomem		*base;

	uint32_t		mask;
	struct mutex		lock;
	struct mutex		hw_lock;
	struct g3d_context	*hw_owner;
	struct completion	completion;

	int			irq;
	struct resource 	*mem;
	struct clk		*clock;
	struct device		*dev;
	struct miscdevice	mdev;
};

struct g3d_context {
	struct g3d_drvdata	*data;
	/* More to come */
};

static void __iomem *base_addr;
static struct completion	d_completion;
static struct resource *s3c_g3d_mem;
static struct mutex		d_lock;
static struct mutex		d_hw_lock;
uint32_t d_mask;
static int s3c_g3d_irq;
static struct clk *g3d_clock;
static struct g3d_context *d_hw_owner;

/*
 * Register accessors
 */
static inline void g3d_write(struct g3d_drvdata *d, uint32_t b, uint32_t r)
{
	writel(b, base_addr + r);
}

static inline uint32_t g3d_read(struct g3d_drvdata *d, uint32_t r)
{
	return readl(base_addr + r);
}

/*
 * Hardware operations
 */
static inline void g3d_soft_reset(struct g3d_drvdata *data)
{
	printk("g3d_soft_reset debut\n");
	g3d_write(data, 1, G3D_FGGB_RESET_REG);
	udelay(1);
	g3d_write(data, 0, G3D_FGGB_RESET_REG);
	printk("g3d_soft_reset fin\n");
}

static inline int g3d_flush_pipeline(struct g3d_drvdata *data, unsigned int mask)
{
	int ret = 0;
	printk("g3d_flush_pipeline debut\n");
	if((g3d_read(data, G3D_FGGB_PIPESTAT_REG) & mask) == 0)
	{
		printk("g3d_flush_pipeline fin1\n");
		return 0;
	}
	
	/* Setup the interrupt */
	d_mask = mask;
	init_completion(&d_completion);
	g3d_write(data, 0, G3D_FGGB_PIPEMASK_REG);
	g3d_write(data, 0, G3D_FGGB_PIPETGTSTATE_REG);
	g3d_write(data, mask, G3D_FGGB_PIPEMASK_REG);
	g3d_write(data, 1, G3D_FGGB_INTMASK_REG);

	/* Check if the condition isn't already met */
	if((g3d_read(data, G3D_FGGB_PIPESTAT_REG) & mask) == 0) {
		/* Disable the interrupt */
		g3d_write(data, 0, G3D_FGGB_INTMASK_REG);
		printk("g3d_flush_pipeline fin2\n");
		return 0;
	}

	if(!wait_for_completion_interruptible_timeout(&d_completion,
								G3D_TIMEOUT)) {
		//dev_err(data->dev, "timeout while waiting for interrupt, resetting (stat=%08x)\n",
				//	g3d_read(data, G3D_FGGB_PIPESTAT_REG));
		g3d_soft_reset(data);
		ret = -EFAULT;
	}

	/* Disable the interrupt */
	g3d_write(data, 0, G3D_FGGB_INTMASK_REG);
	printk("g3d_flush_pipeline fin3 ret : %d\n", ret);
	return ret;
}

static inline void g3d_flush_caches(struct g3d_drvdata *data)
{
	int timeout = 1000000000;
	printk("g3d_flush_caches debut\n");
	g3d_write(data, G3D_FGGB_FLUSH_MSK, G3D_FGGB_CACHECTL_REG);

	do {
		if(!g3d_read(data, G3D_FGGB_CACHECTL_REG))
			break;
	} while (--timeout);
	printk("g3d_flush_caches fin\n");
}

static inline void g3d_invalidate_caches(struct g3d_drvdata *data)
{
	int timeout = 1000000000;
	printk("g3d_invalidate_caches debut\n");
	g3d_write(data, G3D_FGGB_INVAL_MSK, G3D_FGGB_CACHECTL_REG);

	do {
		if(!g3d_read(data, G3D_FGGB_CACHECTL_REG))
			break;
	} while (--timeout);
	printk("g3d_invalidate_caches fin\n");
}

/*
 * State processing
 */
static irqreturn_t g3d_handle_irq(int irq, void *dev_id)
{
	struct g3d_drvdata *data = (struct g3d_drvdata *)dev_id;
	uint32_t stat;
	printk("g3d_handle_irq debut\n");
	g3d_write(data, 0, G3D_FGGB_INTPENDING_REG);
	stat = g3d_read(data, G3D_FGGB_PIPESTAT_REG) & d_mask;

	if(!stat)
		complete(&d_completion);
	printk("g3d_handle_irq fin\n");
	return IRQ_HANDLED;
}

static inline int ctx_has_lock(struct g3d_context *ctx)
{
	struct g3d_drvdata *data = ctx->data;
	printk("ctx_has_lock\n");
	return mutex_is_locked(&d_hw_lock) && (d_hw_owner == ctx);
}

/*
 * File operations
 */
static int s3c_g3d_unlock(struct g3d_context *ctx)
{
	struct g3d_drvdata *data = ctx->data;
	int ret = 0;
	printk("s3c_g3d_unlock debut\n");
	mutex_lock(&d_lock);

	if (unlikely(!ctx_has_lock(ctx))) {
		//dev_err(data->dev, "called S3C_G3D_UNLOCK without holding the hardware lock\n");
		ret = -EPERM;
		goto exit;
	}

	mutex_unlock(&d_hw_lock);
	//dev_dbg(data->dev, "hardware lock released by %p\n", ctx);
	
exit:
	mutex_unlock(&d_lock);
	printk("s3c_g3d_unlock fin\n");
	return ret;
}

static int s3c_g3d_lock(struct g3d_context *ctx)
{
	struct g3d_drvdata *data = ctx->data;
	int ret = 0;
	printk("s3c_g3d_lock debut\n");
	mutex_lock(&d_hw_lock);

	//dev_dbg(data->dev, "hardware lock acquired by %p\n", ctx);

	mutex_lock(&d_lock);

	if (unlikely(ret < 0)) {
		//dev_err(data->dev, "runtime resume failed.\n");
		mutex_unlock(&d_hw_lock);
		goto exit;
	}

	if (likely(d_hw_owner == ctx)) {
		mutex_unlock(&d_lock);
		printk("s3c_g3d_unlock fin1\n");
		return 0;
	}

	ret = 1;

	if (d_hw_owner) {
		g3d_flush_pipeline(data, G3D_FGGB_PIPESTAT_MSK);
		ret = 2;
	}

	d_hw_owner = ctx;

exit:
	mutex_unlock(&d_lock);
	printk("s3c_g3d_unlock fin2\n");
	return ret;
}

static int s3c_g3d_flush(struct g3d_context *ctx, u32 mask)
{
	struct g3d_drvdata *data = ctx->data;
	int ret = 0;
	printk("s3c_g3d_flush debut\n");
	mutex_lock(&d_lock);

	if (unlikely(!ctx_has_lock(ctx))) {
		//dev_err(data->dev, "called S3C_G3D_FLUSH without holding the hardware lock\n");
		ret = -EPERM;
		goto exit;
	}

	ret = g3d_flush_pipeline(data, mask & G3D_FGGB_PIPESTAT_MSK);

exit:
	mutex_unlock(&d_lock);
	printk("s3c_g3d_flush fin\n");
	return ret;
}

static long s3c_g3d_ioctl(struct file *file,
					unsigned int cmd, unsigned long arg)
{
	struct g3d_context *ctx = file->private_data;
	int ret = 0;
	printk("s3c_g3d_ioctl debut\n");
	switch(cmd) {
	/* Prepare and lock the hardware */
	case S3C_G3D_LOCK:
		ret = s3c_g3d_lock(ctx);
		break;

	/* Unlock the hardware and start idle timer */
	case S3C_G3D_UNLOCK:
		ret = s3c_g3d_unlock(ctx);
		break;

	/* Wait for the hardware to finish its work */
	case S3C_G3D_FLUSH:
		ret = s3c_g3d_flush(ctx, arg & G3D_FGGB_PIPESTAT_MSK);
		break;

	default:
		ret = -EINVAL;
	}
	printk("s3c_g3d_ioctl fin\n");
	return ret;
}

static int s3c_g3d_open(struct inode *inode, struct file *file)
{
	struct miscdevice *mdev = file->private_data;
	struct g3d_drvdata *data = container_of(mdev, struct g3d_drvdata, mdev);
	struct g3d_context *ctx;
	printk("s3c_g3d_open debut\n");
	ctx = kmalloc(sizeof(struct g3d_context), GFP_KERNEL);
	ctx->data = data;

	file->private_data = ctx;

	//dev_dbg(data->dev, "device opened\n");
	printk("s3c_g3d_open fin\n");
	return 0;
}

static int s3c_g3d_release(struct inode *inode, struct file *file)
{
	struct g3d_context *ctx = file->private_data;
	struct g3d_drvdata *data = ctx->data;
	int unlock = 0;
	printk("s3c_g3d_release debut\n");
	/* Do this atomically */
	mutex_lock(&d_lock);

	unlock = ctx_has_lock(ctx);

	mutex_unlock(&d_lock);

	/* Unlock if we have the lock */
	if(unlock)
		s3c_g3d_ioctl(file, S3C_G3D_UNLOCK, 0);

	kfree(ctx);
	//dev_dbg(data->dev, "device released\n");
	printk("s3c_g3d_release fin\n");
	return 0;
}

int s3c_g3d_mmap(struct file* file, struct vm_area_struct *vma)
{
	struct g3d_context *ctx = file->private_data;
	struct g3d_drvdata *data = ctx->data;
	unsigned long pfn;
	size_t size = vma->vm_end - vma->vm_start;
	printk("s3c_g3d_mmap debut\n");
	
	pfn = __phys_to_pfn(0xD8000000);

	/*if(size > resource_size(data->mem)) {
		dev_err(data->dev, "mmap size bigger than G3D SFR block\n");
		return -EINVAL;
	}*/

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	if ((vma->vm_flags & VM_WRITE) && !(vma->vm_flags & VM_SHARED)) {
		dev_err(data->dev, "mmap of G3D SFR block must be shared\n");
		return -EINVAL;
	}

	if (remap_pfn_range(vma, vma->vm_start, pfn, size, vma->vm_page_prot)) {
		dev_err(data->dev, "remap_pfn range failed\n");
		return -EINVAL;
	}

	//dev_dbg(data->dev, "hardware mapped by %p\n", ctx);
	printk("s3c_g3d_mmap fin\n");
	return 0;
}

static struct file_operations s3c_g3d_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl	= s3c_g3d_ioctl,
	.open		= s3c_g3d_open,
	.release	= s3c_g3d_release,
	.mmap		= s3c_g3d_mmap,
};

/*
 * Platform device operations
 */
static int __init s3c_g3d_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct g3d_drvdata *data;
	int ret;
	uint32_t version;

	if (pdev->id != -1) {
		dev_err(&pdev->dev, "only single instance is allowed.\n");
		return -EINVAL;
	}

	data = kzalloc(sizeof(struct g3d_drvdata), GFP_KERNEL);
	if(data == NULL) {
		dev_err(&pdev->dev, "failed to allocate driver data.\n");
		return -ENOMEM;
	}

	/* initialize the miscdevice struct */
	data->mdev.minor	= MISC_DYNAMIC_MINOR;
	data->mdev.name		= "s3c-g3d";
	data->mdev.fops		= &s3c_g3d_fops;

	s5p6442_pwrgate_config(S5P6442_G3D_ID, S5P6442_ACTIVE_MODE);

	/* get device clock */
	g3d_clock = clk_get(&pdev->dev, "3dse");
	if (g3d_clock == NULL) {
		dev_err(&pdev->dev, "failed to find g3d clock source\n");
		ret = -ENOENT;
		goto err_clock;
        }

	/* get the memory region for the post processor driver */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if(res == NULL) {
		dev_err(&pdev->dev, "failed to get memory region resource.\n");
		ret = -ENOENT;
		goto err_mem;
	}

	/* reserve the memory */
	s3c_g3d_mem = request_mem_region(res->start, (res->end-res->start)+1,
								pdev->name);
	if (s3c_g3d_mem == NULL) {
		dev_err(&pdev->dev, "failed to reserve memory region\n");
		ret = -ENOENT;
		goto err_mem;
	}
	printk("res->start : %x res->end %x taille : %x tailleressource : %x\n", res->start, res->end, (res->end-res->start)+1, resource_size(res));
	//printk("data->mem->start : %x taille : %x\n", data->mem->start, resource_size(data->mem));

	/* map the memory */
	base_addr = ioremap(res->start, (res->end-res->start)+1);
	if (base_addr == NULL) {
		dev_err(&pdev->dev, "ioremap failed\n");
		ret = -ENOENT;
		goto err_ioremap;
	}

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	s3c_g3d_irq = res->start;
	if(res == NULL) {
		dev_err(&pdev->dev,
			"failed to get irq resource (%d).\n", s3c_g3d_irq);
		ret = -ENOENT;
		goto err_irq;
	}
	printk("IRQ  res irq : %x\n", res->start);
	/* request the IRQ */
	ret = request_irq(res->start, g3d_handle_irq, 0, pdev->name, pdev);
	if (ret) {
		dev_err(&pdev->dev, "request_irq failed (%d).\n", ret);
		goto err_irq;
	}

	data->dev = &pdev->dev;
	d_hw_owner = NULL;
	mutex_init(&d_lock);
	mutex_init(&d_hw_lock);
	init_completion(&d_completion);

	platform_set_drvdata(pdev, data);
	

	version = g3d_read(data, G3D_FGGB_VERSION);
	dev_info(&pdev->dev, "detected FIMG-3DSE version %d.%d.%d\n",
		version >> 24, (version >> 16) & 0xff, (version >> 8) & 0xff);

	ret = misc_register(&data->mdev);
	if (ret < 0) {
		dev_err(&pdev->dev, "could not register miscdev (%d)\n", ret);
		goto err_misc_register;
	}
	
	g3d_soft_reset(data); 

	//printk("data->mem->start : %x taille : %x\n", ctx->data->mem->start, resource_size(ctx->data->mem));
	return 0;

err_misc_register:
	free_irq(s3c_g3d_irq, pdev);
err_irq:
	iounmap(base_addr);
err_ioremap:
	release_resource(s3c_g3d_mem);
err_mem:
	clk_put(g3d_clock);
err_clock:
	kfree(data);

	return ret;
}

static int __devexit s3c_g3d_remove(struct platform_device *pdev)
{
	struct g3d_drvdata *data = platform_get_drvdata(pdev);
	printk("s3c_g3d_remove debut\n");
	misc_deregister(&data->mdev);

	free_irq(s3c_g3d_irq, data);
	iounmap(base_addr);
	release_resource(s3c_g3d_mem);
	clk_put(g3d_clock);
	kfree(data);
	printk("s3c_g3d_remove fin\n");
	return 0;
}

static int s3c_g3d_runtime_suspend(struct device *dev)
{
	struct g3d_drvdata *data = dev_get_drvdata(dev);
	printk("s3c_g3d_runtime_suspend debut\n");
	clk_disable(g3d_clock);
	d_hw_owner = NULL;
	printk("s3c_g3d_runtime_suspend fin\n");
	return 0;
}

static int s3c_g3d_runtime_resume(struct device *dev)
{
	struct g3d_drvdata *data = dev_get_drvdata(dev);
	printk("s3c_g3d_runtime_resume debut\n");
	clk_enable(g3d_clock);
	g3d_soft_reset(data);
	printk("s3c_g3d_runtime_resume fin\n");
	return 0;
}

static int s3c_g3d_suspend(struct device *dev)
{
	struct g3d_drvdata *data = dev_get_drvdata(dev);
	printk("s3c_g3d_suspend debut\n");
	if (mutex_is_locked(&d_hw_lock)) {
		dev_err(dev, "suspend requested with locked hardware (broken userspace?)\n");
		return -EAGAIN;
	}

	clk_disable(g3d_clock);
	d_hw_owner = NULL;
	printk("s3c_g3d_suspend fin\n");
	return 0;
}

static int s3c_g3d_resume(struct device *dev)
{
	struct g3d_drvdata *data = dev_get_drvdata(dev);
	printk("s3c_g3d_resume debut\n");
	clk_enable(g3d_clock);
	g3d_soft_reset(data);
	printk("s3c_g3d_resume fin\n");
	return 0;
}

static struct dev_pm_ops s3c_g3d_pm_ops = {
	.suspend		= s3c_g3d_suspend,
	.resume			= s3c_g3d_resume,
	.runtime_suspend	= s3c_g3d_runtime_suspend,
	.runtime_resume		= s3c_g3d_runtime_resume,
};

static struct platform_driver s3c_g3d_driver = {
	.remove	= __devexit_p(s3c_g3d_remove),
	.driver	= {
		.owner	= THIS_MODULE,
		.name	= "s3c-g3d",
		.pm	= &s3c_g3d_pm_ops,
	},
};

/*
 * Module operations
 */
int __init  s3c_g3d_init(void)
{
	printk("s3c_g3d_init debut\n");
	return platform_driver_probe(&s3c_g3d_driver, s3c_g3d_probe);
}

void __exit s3c_g3d_exit(void)
{
	printk("s3c_g3d_exit debut\n");
	platform_driver_unregister(&s3c_g3d_driver);
}

module_init(s3c_g3d_init);
module_exit(s3c_g3d_exit);

MODULE_AUTHOR("Tomasz Figa <tomasz.figa@gmail.com>");
MODULE_DESCRIPTION("OpenFIMG Kernel Interface");
MODULE_LICENSE("GPL");
