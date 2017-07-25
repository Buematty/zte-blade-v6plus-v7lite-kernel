#include "hw_info.h"

const char* hwidtostr(hw_sensor_type hst)
{
	switch (hst)
	{
		case MAIN_CAM:			 return "Main Camera";
		case SUB_CAM:			 return "Sub Camera";
		case LCM:				 return "LCM";
		case TOUCH_PANEL: 		 return "Touch Panel";
		default:					 return "NULL";
	}
}

void hw_info_add(struct hw_info* hwi)
{
	struct list_head* listhead = NULL;
	struct hw_info *temp = NULL;
	if((!hwi) || (hwi->hst >= MAX_SENSOR))
	{
		HW_DEBUG("error:check params\n");
	}
	list_for_each(listhead,&hw_info_list)
	{	
		temp = list_entry(listhead,struct hw_info,lh);
		if((!strcmp(temp->name,hwi->name))  && (temp->hst == hwi->hst)){
			HW_DEBUG("error:%s has added\n",temp->name);
			return;
			}
	}
	
	temp = kzalloc(sizeof(struct hw_info),GFP_KERNEL);  //allocate size of str hw_info  from mem

	sprintf(temp->name,"%s",hwi->name);                   //assignment for different domain of this str
	temp->name[strlen(hwi->name)] = '\0';
	sprintf(temp->vendor,"%s",hwi->vendor);
	temp->vendor[strlen(hwi->vendor)]='\0';
	temp->hst = hwi->hst;
	
	INIT_LIST_HEAD(&(temp->lh));                            //initial list_node and add into list
	list_add_tail(&(temp->lh),&hw_info_list);
}

void hw_info_del(struct hw_info* hwi)
{
	struct list_head* temp=NULL;
	struct hw_info* info = NULL;
	list_for_each(temp,&hw_info_list)                         //Traverse the list and find the certain hw_info by name 
	{	
		info = list_entry(temp,struct hw_info,lh);
		if(!strcmp(info->name,hwi->name))
		{
			list_del(&(info->lh));                 		//if we find,then del it from list and free the mem
			kfree(info);                                 
		}
	}
}
//for sysfs access
static ssize_t hw_info_kobj_show(struct device *dev, 
				struct device_attribute *attr, 
				char *buffer) 
{
	struct list_head * temp = NULL;
	static struct hw_info * info;
	int len=0;
	if(list_empty(&hw_info_list))
	{
		sprintf(buffer,"%s\n","hw_info list is empty");
		return sprintf(buffer,"%s\n","hw_info list is empty");
	}
	list_for_each(temp,&hw_info_list)
	{	
		info = list_entry(temp,struct hw_info,lh);
		HW_DEBUG("Hw_type:%d Name:%s Vendor:%s\n",
					info->hst,info->name,info->vendor);
		len+=snprintf(buffer+len,PAGE_SIZE,
			"Hw_Type:%s\nName:%s\nVendor:%s\n\n",
			hwidtostr(info->hst),info->name,info->vendor);
	}
	return len;
}
//static struct kobj_attribute  hw_info_attr = __ATTR(hw_info_attr,0666,hw_info_kobj_show,NULL);

static DEVICE_ATTR(hw_info, S_IRUGO, hw_info_kobj_show,NULL);


static int hw_info_probe(struct platform_device *pdev)
{
#if 0
kobject_init_and_add(hw_info_kobj, &hw_info_kobj_type, NULL, "hw_info_kobj") ;
#else
device_create_file(&(pdev->dev),&dev_attr_hw_info);
#endif 
return 0;
}
struct platform_device hw_info_dev =
{
.name = "hw_info",
.id = -1,
};
struct platform_driver hw_info_drv =
{
.probe  = hw_info_probe,
//.remove     = hw_info_remove,   
.driver = {
.name = "hw_info",
.owner = THIS_MODULE,
}
};
static int __init hw_info_init(void)
{
//sysfs
HW_DEBUG("sysfs add hw_info\n");
platform_device_register(&hw_info_dev);
platform_driver_register(&hw_info_drv);
#if 0
    //add kobject
kobject_init_and_add(hw_info_kobj, &hw_info_kobj_type, NULL, "hw_info_kobj") ;
#endif
return 0;
}

static void __exit hw_info_exit(void)
{
HW_DEBUG("hw_info remove\n");
//kobject_put(hw_info_kobj);
platform_device_unregister(&hw_info_dev);
platform_driver_unregister(&hw_info_drv);
}

module_init(hw_info_init);
module_exit(hw_info_exit);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("<wang.tao3@zte.com.cn>");
MODULE_DESCRIPTION("SYSFS ADD HW_INFO");


