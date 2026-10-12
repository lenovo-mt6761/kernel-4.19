#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/utsname.h>

static int factory_value = -1;

extern int msdc_sdcard_intr_gpio_value_show(struct seq_file *m, void *v);

static int sdcard_intr_proc_show(struct seq_file *m, void *v)
{
    msdc_sdcard_intr_gpio_value_show(m,v);
    return 0;
}
static int sdcard_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, sdcard_intr_proc_show, NULL);
}
static const struct file_operations sdcard_proc_fops = {
       .open           = sdcard_proc_open,
       .read           = seq_read,
       .llseek         = seq_lseek,
       .release        = single_release,
};

static int factory_proc_show(struct seq_file *m, void *v)
{
    u32 value = 0x0;
    if  (factory_value == -1){
        return 0;
    }
    if (factory_value == 0)
        value = 0x0;
    if (factory_value == 1)
        value = 0x1;

    seq_printf(m, "0x%x\n",value);
    return 0;
}
static int factory_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, factory_proc_show, NULL);
}
static const struct file_operations factory_proc_fops = {
	.open		= factory_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};
static int __init root_dev_setup(char *line)
{
    if ('\0' == *line)
        return -1;
    else
        factory_value = simple_strtol(line, NULL, 10);
        return 0;
}
__setup("factory_reset=", root_dev_setup);
static int __init proc_factory_init(void)
{
    struct proc_dir_entry *prEntry;

    prEntry = proc_create("factory_state", 0, NULL, &factory_proc_fops);
    if (!prEntry)
               pr_notice("[%s]: failed to create /proc/factory_state\n", __func__);

    prEntry = proc_create("sd_tray_gpio_value", 0, NULL, &sdcard_proc_fops);
    if (!prEntry)
               pr_notice("[%s]: failed to create /proc/sd_tray_gpio_value\n", __func__);

     return 0;

}
fs_initcall(proc_factory_init);
