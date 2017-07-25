#ifndef HW_INFO_H
#define HW_INFO_H
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#define HW_DEBUG(fmt,arg...) printk("HW_INFO" "[%s]-[%d]\t" fmt,__func__,__LINE__,##arg)
typedef enum {
MAIN_CAM=0,
SUB_CAM,
LCM,
TOUCH_PANEL,
MAX_SENSOR
}hw_sensor_type;

const char* hwidtostr(hw_sensor_type);

struct hw_info{
struct list_head lh;
hw_sensor_type hst;
char name[20];
char vendor[20];
};
static LIST_HEAD(hw_info_list);
void hw_info_add(struct hw_info*);
void hw_info_del(struct hw_info*);
#endif
