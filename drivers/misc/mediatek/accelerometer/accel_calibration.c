#include "inc/accel_calibration.h"

static int acc_calibration_open(struct inode *inode, struct file *file)
{
	file->private_data = acc_context_obj;

	if (file->private_data == NULL) {
		ACC_ERR("null pointer!!\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);
}

static int acc_calibration_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

static ssize_t acc_calibration_read_proc(struct file *filp, char __user *user_buff, size_t len, loff_t *off)
{
	int cali[3] = {0, 0, 0}, cal_valid = 0, cnt;
	struct acc_context *cxt = acc_context_obj;
	char buff[32];
		
	cal_valid = cxt->acc_ctl.acc_calibration(0, cali);
	ACC_LOG("%s cal_valid =%d x=%d y=%d z=%d\n", __func__, cal_valid, cali[0], cali[1], cali[2]);

	cnt = sprintf(buff, "%d %d %d %d\n", cal_valid, cali[0], cali[1], cali[2]);
	
	return simple_read_from_buffer(user_buff, len, off, buff, cnt);
}

// called when started-up
static ssize_t acc_calibration_write_proc(struct file *filp, const char *buff, size_t len, loff_t *off)
{
	int cali[3] = {0, 0, 0};
	struct acc_context *cxt = acc_context_obj;
	
	sscanf(buff, "%d %d %d", &cali[0], &cali[1], &cali[2]);

	cxt->cali_sw[ACC_AXIS_X] = cali[0];
	cxt->cali_sw[ACC_AXIS_Y] = cali[1];
	cxt->cali_sw[ACC_AXIS_Z] = cali[2];

	ACC_LOG("%s write acc cali %d %d %d\n", __func__, cali[0], cali[1], cali[2]);

	return len;
}

static const struct file_operations acc_calibration_fops = {
	.owner	= THIS_MODULE,
	.open	= acc_calibration_open,
	.release	= acc_calibration_release,
	.read	= acc_calibration_read_proc,
	.write	= acc_calibration_write_proc,
};

int acc_calibration_init(void)
{
	struct proc_dir_entry *acc_proc_file; 
	
	acc_proc_file = proc_create("driver/acc_calibration", 0646, NULL, &acc_calibration_fops);
	if(acc_proc_file == NULL)
	{
		ACC_ERR("create /driver/acc_calibration/ file failed!\n");
		return -1;
	}
	ACC_LOG("driver/acc_calibration created Success!\n");

	return 0;
}
