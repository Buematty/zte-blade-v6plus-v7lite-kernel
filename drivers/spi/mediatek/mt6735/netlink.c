#include <linux/init.h>
#include <linux/module.h>
#include <linux/timer.h>
#include <linux/time.h>
#include <linux/types.h>
#include <net/sock.h>
#include <net/netlink.h>

#define NETLINK_TEST 25
#define MAX_MSGSIZE 4*1024
int stringlength(char *s);
void sendnlmsg(char * message);
int portid;
//static int err;
struct sock *nl_sk = NULL;
//static int flag = 0;


struct gf_uk_channel{
        int channel_id;
        int reserved;
        char buf[3*1024];
        int len;
};


void sendnlmsg(char *message)
{
    struct sk_buff *skb_1;
    struct nlmsghdr *nlh;
    int len = NLMSG_SPACE(MAX_MSGSIZE);
    int slen = 0;
    if(!message || !nl_sk)
    {
	return ;
    }
    skb_1 = alloc_skb(len,GFP_KERNEL);
    if(!skb_1)
    {
	printk(KERN_ERR "my_net_link:alloc_skb_1 error\n");
    }
    slen = strlen(message);
    nlh = nlmsg_put(skb_1,0,0,0,MAX_MSGSIZE,0);

    NETLINK_CB(skb_1).portid = 0;
    NETLINK_CB(skb_1).dst_group = 0;

    message[slen]= '\0';
    memcpy(NLMSG_DATA(nlh),message,slen+1);
    //printk("my_net_link:send message '%s'.\n",(char *)NLMSG_DATA(nlh));

    netlink_unicast(nl_sk,skb_1,portid,MSG_DONTWAIT);

}


void nl_data_ready(struct sk_buff *__skb)
{
    struct sk_buff *skb;
    struct nlmsghdr *nlh;
    char str[100];
   // u8 i = 255;
    //struct completion cmpl;
    skb = skb_get (__skb);
    if(skb->len >= NLMSG_SPACE(0))
    {
	nlh = nlmsg_hdr(skb);

	memcpy(str, NLMSG_DATA(nlh), sizeof(str));
	portid = nlh->nlmsg_pid;
	//printk("Message pid %d received:%s\n",pid, str) ;
	//while(i--)
	//{
	    //init_completion(&cmpl);
	    //wait_for_completion_timeout(&cmpl,3 * HZ);
	    //sendnlmsg("I am from kernel!");
	//}
	//flag = 1;
	kfree_skb(skb);
    }

}

// Initialize netlink

struct timer_list gf_timer;
static void gf_timer_func(unsigned long arg)
{
    sendnlmsg("I am from kernel!");
    mod_timer(&gf_timer, jiffies + 2*HZ);
}


void ninit_timer(void)
{
    init_timer(&gf_timer);
    gf_timer.function = gf_timer_func;
    gf_timer.expires = jiffies + 3*HZ;
    gf_timer.data = 0;
    add_timer(&gf_timer);
}



int netlink_init(void)
{
		struct netlink_kernel_cfg cfg = {
    		.input = nl_data_ready,//该函数原型可参考内核代码，其他参数默认即可，可参考内核中的调用
		};
    nl_sk = netlink_kernel_create(&init_net, NETLINK_TEST,&cfg);

    if(!nl_sk){
	printk(KERN_ERR "my_net_link: create netlink socket error.\n");
	return 1;
    }

    //ninit_timer();
    //printk("my_net_link_3: create netlink socket ok.\n");
    return 0;
}

void netlink_exit(void)
{
    if(nl_sk != NULL){
        sock_release(nl_sk->sk_socket);
    }

    //del_timer_sync(&gf_timer);

    printk("my_net_link: self module exited\n");
}

