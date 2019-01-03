/*===========================================================================
FILE:
   usbnet_2_6_32.c

DESCRIPTION:
   The default "usbnet_start_xmit" function is over-ridden by Sierra to provide the URB_Monitor
   for Linux kernel 2.6.32

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
===========================================================================*/

//---------------------------------------------------------------------------
// Include Files
//---------------------------------------------------------------------------

#include "Structs.h"
#include "QMIDevice.h"
#include "QMI.h"
#include "gobi_usbnet.h"
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/module.h>
#include <net/ip.h>

#include <asm/siginfo.h>   //siginfo
#include <linux/rcupdate.h>   //rcu_read_lock
#include <linux/sched.h>   //find_task_by_pid_type

#if (LINUX_VERSION_CODE == KERNEL_VERSION( 2,6,31 ) ||\
	   LINUX_VERSION_CODE == KERNEL_VERSION( 2,6,32 ))
void (*URB_monitor) (bool,unsigned char);
EXPORT_SYMBOL(URB_monitor);
#if 0
/*
 * Dummy function to test URB back-pressure. In actual implementation,
 * customer will implement this function
 */
// Define PDN interfaced as per specific Sierra module PTS. The below are for MC73xx modules 
#define PDN1_INTERFACE 8 
#define PDN2_INTERFACE 10
#define BACK_PRESSURE_WATERMARK 10
static unsigned short urb_count_pdn1 = 0;
static unsigned short urb_count_pdn2 = 0;
void URB_monitor (bool isUpCount,unsigned char interface)
{
/*	printk(KERN_WARNING "[%s] isUpCount %d, intf %d", \
			__func__, isUpCount, interface);*/
	if (isUpCount)
	{
		if (interface == PDN1_INTERFACE)
		{
			urb_count_pdn1++;
		}
		else if (interface == PDN2_INTERFACE)
		{
			urb_count_pdn2++;
		}
		else
		{
			// unknown interface. ignore or log as needed
		}
	}
	else
	{
		if (interface == PDN1_INTERFACE)
		{
			urb_count_pdn1--;
		}
		else if (interface == PDN2_INTERFACE)
		{
			urb_count_pdn2--;
		}
		else
		{
			// unknown interface. ignore or log as needed
		}	
	}
	if (BACK_PRESSURE_WATERMARK <= urb_count_pdn1)
	{
		// Back pressure on PDN1
		printk(KERN_WARNING "[%s] Backpressure %d on PDN1", \
				__func__, urb_count_pdn1);
	}	
	if (BACK_PRESSURE_WATERMARK <= urb_count_pdn2)
	{
		// Back pressure on PDN2
		printk(KERN_WARNING "[%s] Backpressure %d on PDN2", \
				__func__, urb_count_pdn2);
	}	
}
#endif // 0 End of dummy function

// Get the USB interface from a usbnet pointer. The function return 0 on success, -1 on error
__always_inline static int get_usb_interface_from_device (struct usbnet	*dev, unsigned char *pb_usb_interface)
{
    int iRet = -1; // set to error by default	
	
    if ((NULL!=dev) && (NULL!=pb_usb_interface))
    {
        if ((NULL != dev->intf) &&
            (NULL != dev->intf->cur_altsetting))
        {
            *pb_usb_interface = dev->intf->cur_altsetting->desc.bInterfaceNumber;
            iRet = 0;
        } // (NULL != dev->intf) && (NULL != dev->intf->cur_altsetting)
    } //(NULL != pURB) && (NULL!=pb_usb_interface)
    return iRet;
}

// Get the USB interface from a URB. The function return 0 on success, -1 on error
__always_inline static int get_usb_interface (struct urb * pURB, unsigned char *pb_usb_interface)
{
    int iRet = -1; // set to error by default	
    struct sk_buff  *skb   = NULL;
    struct skb_data	*entry = NULL;

    if ((NULL!=pURB) && (NULL!=pb_usb_interface))
    { 
        skb = (struct sk_buff *) pURB->context;
        if (NULL != skb)
        {
            entry = (struct skb_data *) skb->cb;
            if (NULL != entry)
            {
                iRet = get_usb_interface_from_device (entry->dev, pb_usb_interface);
            }
        } // (NULL != skb)
    } // (NULL != pURB) && (NULL!=pb_usb_interface)
    return iRet;
}

// unlink pending rx/tx; completion handlers do all other cleanup
static int unlink_urbs (struct usbnet *dev, struct sk_buff_head *q)
{
	unsigned long		flags;
	struct sk_buff		*skb, *skbnext;
	int			count = 0;

	spin_lock_irqsave (&q->lock, flags);
	skb_queue_walk_safe(q, skb, skbnext) {
		struct skb_data		*entry;
		struct urb		*urb;
		int			retval;

		entry = (struct skb_data *) skb->cb;
		urb = entry->urb;

		/*
		 * Get reference count of the URB to avoid it to be
		 * freed during usb_unlink_urb, which may trigger
		 * use-after-free problem inside usb_unlink_urb since
		 * usb_unlink_urb is always racing with .complete
		 * handler(include defer_bh).
		 */
		usb_get_urb(urb);
		spin_unlock_irqrestore(&q->lock, flags);
		// during some PM-driven resume scenarios,
		// these (async) unlinks complete immediately
		retval = usb_unlink_urb (urb);
		if (retval != -EINPROGRESS && retval != 0)
			devdbg (dev, "unlink urb err, %d", retval);
		else
			count++;
		usb_put_urb(urb);
		spin_lock_irqsave(&q->lock, flags);
	}
	spin_unlock_irqrestore (&q->lock, flags);
	return count;
}

void gobi_usbnet_tx_timeout_2_6_32 (struct net_device *net)
{
   struct usbnet		*dev = netdev_priv(net);
#ifdef TX_URB_MONITOR
   int count = 0;   
   int iRet = -1;
   unsigned char b_usb_if_num = 0;
   // Get the USB interface
   iRet = get_usb_interface_from_device (dev, &b_usb_if_num);	

   count = unlink_urbs (dev, &dev->txq);
   tasklet_schedule (&dev->bh);

   if ((URB_monitor) && (0==iRet))
   {
       while (count)
	   {
			URB_monitor(false, b_usb_if_num);
			count--;
	   }
   }
#else // TX_URB_MONITOR
   unlink_urbs (dev, &dev->txq);
   tasklet_schedule (&dev->bh);
#endif // TX_URB_MONITOR
   // FIXME: device recovery -- reset?
}

/* some LK 2.4 HCDs oopsed if we freed or resubmitted urbs from
 * completion callbacks.  2.5 should have fixed those bugs...
 */

static void defer_bh(struct usbnet *dev, struct sk_buff *skb, struct sk_buff_head *list)
{
   unsigned long		flags;

   spin_lock_irqsave(&list->lock, flags);
   __skb_unlink(skb, list);
   spin_unlock(&list->lock);
   spin_lock(&dev->done.lock);
   __skb_queue_tail(&dev->done, skb);
   if (dev->done.qlen == 1)
      tasklet_schedule(&dev->bh);
   spin_unlock_irqrestore(&dev->done.lock, flags);
}

static void tx_complete (struct urb *urb)
{
   struct sk_buff		*skb = (struct sk_buff *) urb->context;
   struct skb_data		*entry = (struct skb_data *) skb->cb;
   struct usbnet		*dev = entry->dev;

#ifdef TX_URB_MONITOR
	unsigned char b_usb_if_num = 0;
    int iRet = get_usb_interface(urb, &b_usb_if_num);
#endif //#ifdef TX_URB_MONITOR

   if (urb->status == 0)
   {
      dev->net->stats.tx_packets++;
      dev->net->stats.tx_bytes += entry->length;
   }
   else
   {
      dev->net->stats.tx_errors++;
      switch (urb->status)
      {
         case -EPIPE:
            usbnet_defer_kevent (dev, EVENT_TX_HALT);
            break;
       
         /* software-driven interface shutdown */
         case -ECONNRESET:		// async unlink
         case -ESHUTDOWN:		// hardware gone
            break;
       
         // like rx, tx gets controller i/o faults during khubd delays
         // and so it uses the same throttling mechanism.
         case -EPROTO:
         case -ETIME:
         case -EILSEQ:
            if (!timer_pending (&dev->delay)) {
             mod_timer (&dev->delay,
             jiffies + THROTTLE_JIFFIES);
            if (netif_msg_link (dev))
               devdbg (dev, "tx throttle %d",
                            urb->status);
            }
            netif_stop_queue (dev->net);
            break;
         default:
            if (netif_msg_tx_err (dev))
               devdbg (dev, "tx err %d", entry->urb->status);
            break;
    		}
  	}

   entry->state = tx_done;
   defer_bh(dev, skb, &dev->txq);

#ifdef TX_URB_MONITOR
   if ((URB_monitor) && (0==iRet))
   {
       URB_monitor(false, b_usb_if_num);
   }
#endif //#ifdef TX_URB_MONITOR

}

int gobi_usbnet_start_xmit_2_6_32 (struct sk_buff *skb, struct net_device *net)
{
   struct usbnet		*dev = netdev_priv(net);
   int			length;
   struct urb		*urb = NULL;
   struct skb_data		*entry;
   struct driver_info	*info = dev->driver_info;
   unsigned long		flags;
   int retval;
#ifdef TX_URB_MONITOR   
   unsigned char b_usb_if_num = 0;
   int iRet = -1;
#endif //#ifdef TX_URB_MONITOR
   // some devices want funky USB-level framing, for
   // win32 driver (usually) and/or hardware quirks
   if (info->tx_fixup)
   {
      skb = info->tx_fixup (dev, skb, GFP_ATOMIC);
      if (!skb)
      {
         if (netif_msg_tx_err (dev))
            devdbg (dev, "can't tx_fixup skb");
         goto drop;
      }
   }
   length = skb->len;
  
   if (!(urb = usb_alloc_urb (0, GFP_ATOMIC)))
   {
      if (netif_msg_tx_err (dev))
         devdbg (dev, "no urb");
      goto drop;
   }

   entry = (struct skb_data *) skb->cb;
   entry->urb = urb;
   entry->dev = dev;
   entry->state = tx_start;
   entry->length = length;
  
   usb_fill_bulk_urb (urb, dev->udev, dev->out,
   	                         skb->data, skb->len, tx_complete, skb);
  
   /* don't assume the hardware handles USB_ZERO_PACKET
    * NOTE:  strictly conforming cdc-ether devices should expect
    * the ZLP here, but ignore the one-byte packet.
    */
   if (!(info->flags & FLAG_SEND_ZLP) && (length % dev->maxpacket) == 0)
   {
      urb->transfer_buffer_length++;
      if (skb_tailroom(skb))
      {
         skb->data[skb->len] = 0;
         __skb_put(skb, 1);
      }
   }
   spin_lock_irqsave (&dev->txq.lock, flags);
#ifdef TX_URB_MONITOR
   iRet = get_usb_interface(urb, &b_usb_if_num);
#endif //#ifdef TX_URB_MONITOR  
   switch ((retval = usb_submit_urb (urb, GFP_ATOMIC)))
   {
      case -EPIPE:
         netif_stop_queue (net);
         usbnet_defer_kevent (dev, EVENT_TX_HALT);
         break;
      default:
         if (netif_msg_tx_err (dev))
            devdbg (dev, "tx: submit urb err %d", retval);
         break;
      	case 0:
         net->trans_start = jiffies;
         __skb_queue_tail (&dev->txq, skb);
         if (dev->txq.qlen >= TX_QLEN (dev))
            netif_stop_queue (net);
   }
#ifdef TX_URB_MONITOR
   /*
    * Call URB_monitor() with true as the URB has been successfully 
    * submitted to the txq. 
    */
   if ((URB_monitor) && (0==iRet) && (0==retval))
   {
       URB_monitor(true, b_usb_if_num);
   }
#endif //#ifdef TX_URB_MONITOR
   spin_unlock_irqrestore (&dev->txq.lock, flags);
   if (retval)
   {
      if (netif_msg_tx_err (dev))
       devdbg (dev, "drop, code %d", retval);
      drop:
      retval = NET_XMIT_SUCCESS;
      dev->net->stats.tx_dropped++;
      if (skb)
         dev_kfree_skb_any (skb);
      usb_free_urb (urb);
   }
   else if (netif_msg_tx_queued (dev))
   {
      devdbg (dev, "> tx, len %d, type 0x%x",
                  length, skb->protocol);
   }
   return NETDEV_TX_OK;
}
#endif /* LINUX_VERSION_CODE == KERNEL_VERSION( 2,6,31 ) */

