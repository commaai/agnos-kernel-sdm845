/*===========================================================================
FILE:
   usbnet_3_12_xx.c

DESCRIPTION:
   The default "usbnet_start_xmit" function is over-ridden by Sierra to provide the URB_Monitor
   for Linux kernel 3.12.0 to 3.12.xx

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

#if (LINUX_VERSION_CODE >= KERNEL_VERSION( 3,12,0 ) &&\
     LINUX_VERSION_CODE < KERNEL_VERSION( 3,13,00))
#if 1
void (*URB_monitor) (bool,unsigned char);
EXPORT_SYMBOL(URB_monitor);
#else
/*
 * Dummy function to test URB back-pressure. In actual implementation,
 * customer will implement this function
 */
// Define PDN interfaced as per specific Sierra module PTS. The below are for MC73xx modules 
#define PDN1_INTERFACE 8 
#define PDN2_INTERFACE 10
#define BACK_PRESSURE_WATERMARK 1//10
static unsigned short urb_count_pdn1 = 0;
static unsigned short urb_count_pdn2 = 0;
void URB_monitor (bool isUpCount,unsigned char interface)
{
/*   printk(KERN_WARNING "[%s] isUpCount %d, intf %d", \
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
__always_inline static int get_usb_interface_from_device (struct usbnet   *dev, unsigned char *pb_usb_interface)
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
   struct skb_data   *entry = NULL;

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
   unsigned long      flags;
   struct sk_buff      *skb;
   int         count = 0;

   spin_lock_irqsave (&q->lock, flags);
   while (!skb_queue_empty(q)) {
      struct skb_data      *entry;
      struct urb      *urb;
      int         retval;

      skb_queue_walk(q, skb) {
         entry = (struct skb_data *) skb->cb;
         if (entry->state != unlink_start)
            goto found;
      }
      break;
found:
      entry->state = unlink_start;
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
         netdev_dbg(dev->net, "unlink urb err, %d\n", retval);
      else
         count++;
      usb_put_urb(urb);
      spin_lock_irqsave(&q->lock, flags);
   }
   spin_unlock_irqrestore (&q->lock, flags);
   return count;
}


void gobi_usbnet_tx_timeout_3_12_xx (struct net_device *net)
{
   struct usbnet      *dev = netdev_priv(net);
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

static enum skb_state defer_bh(struct usbnet *dev, struct sk_buff *skb,
      struct sk_buff_head *list, enum skb_state state)
{
   unsigned long      flags;
   enum skb_state       old_state;
   struct skb_data *entry = (struct skb_data *) skb->cb;

   spin_lock_irqsave(&list->lock, flags);
   old_state = entry->state;
   entry->state = state;
   __skb_unlink(skb, list);
   spin_unlock(&list->lock);
   spin_lock(&dev->done.lock);
   __skb_queue_tail(&dev->done, skb);
   if (dev->done.qlen == 1)
      tasklet_schedule(&dev->bh);
   spin_unlock_irqrestore(&dev->done.lock, flags);
   return old_state;
}

static void tx_complete (struct urb *urb)
{
   struct sk_buff      *skb = (struct sk_buff *) urb->context;
   struct skb_data      *entry = (struct skb_data *) skb->cb;
   struct usbnet      *dev = entry->dev;

#ifdef TX_URB_MONITOR
   unsigned char b_usb_if_num = 0;
    int iRet = get_usb_interface(urb, &b_usb_if_num);
#endif //#ifdef TX_URB_MONITOR

   if (urb->status == 0) {
      if (!(dev->driver_info->flags & FLAG_MULTI_PACKET))
         dev->net->stats.tx_packets++;
      dev->net->stats.tx_bytes += entry->length;
   } else {
      dev->net->stats.tx_errors++;

      switch (urb->status) {
      case -EPIPE:
         usbnet_defer_kevent (dev, EVENT_TX_HALT);
         break;

      /* software-driven interface shutdown */
      case -ECONNRESET:      // async unlink
      case -ESHUTDOWN:      // hardware gone
         break;

      // like rx, tx gets controller i/o faults during khubd delays
      // and so it uses the same throttling mechanism.
      case -EPROTO:
      case -ETIME:
      case -EILSEQ:
         usb_mark_last_busy(dev->udev);
         if (!timer_pending (&dev->delay)) {
            mod_timer (&dev->delay,
               jiffies + THROTTLE_JIFFIES);
            netif_dbg(dev, link, dev->net,
                 "tx throttle %d\n", urb->status);
         }
         netif_stop_queue (dev->net);
         break;
      default:
         netif_dbg(dev, tx_err, dev->net,
              "tx err %d\n", entry->urb->status);
         break;
      }
   }

   gobi_usb_autopm_put_interface_async(dev->intf);
   (void) defer_bh(dev, skb, &dev->txq, tx_done);

#ifdef TX_URB_MONITOR
   if ((URB_monitor) && (0==iRet))
   {
      URB_monitor(false, b_usb_if_num);
   }
#endif //#ifdef TX_URB_MONITOR

}

static int build_dma_sg(const struct sk_buff *skb, struct urb *urb)
{
   unsigned num_sgs, total_len = 0;
   int i, s = 0;

   num_sgs = skb_shinfo(skb)->nr_frags + 1;
   if (num_sgs == 1)
      return 0;

   /* reserve one for zero packet */
   urb->sg = kmalloc((num_sgs + 1) * sizeof(struct scatterlist),
                    GFP_ATOMIC);
   if (!urb->sg)
      return -ENOMEM;

   urb->num_sgs = num_sgs;
   sg_init_table(urb->sg, urb->num_sgs);

   sg_set_buf(&urb->sg[s++], skb->data, skb_headlen(skb));
   total_len += skb_headlen(skb);

   for (i = 0; i < skb_shinfo(skb)->nr_frags; i++) {
      struct skb_frag_struct *f = &skb_shinfo(skb)->frags[i];

      total_len += skb_frag_size(f);
      sg_set_page(&urb->sg[i + s], f->page.p, f->size,
                    f->page_offset);
   }
   urb->transfer_buffer_length = total_len;

   return 1;
}


/* The caller must hold list->lock */ 
static void __usbnet_queue_skb(struct sk_buff_head *list,
                                struct sk_buff *newsk, enum skb_state state)
{
   struct skb_data *entry = (struct skb_data *) newsk->cb;

   __skb_queue_tail(list, newsk);
   entry->state = state;
}

netdev_tx_t gobi_usbnet_start_xmit_3_12_xx (struct sk_buff *skb,
                 struct net_device *net)
{
   struct usbnet           *dev = netdev_priv(net);
   int                     length;
   struct urb              *urb = NULL;
   struct skb_data         *entry;
   struct driver_info      *info = dev->driver_info;
   unsigned long           flags;
   int retval;
#ifdef TX_URB_MONITOR
unsigned char        b_usb_if_num = 0;
int iRet = -1;
#endif //#ifdef TX_URB_MONITOR

   if (skb)
      skb_tx_timestamp(skb);

   // some devices want funky USB-level framing, for
   // win32 driver (usually) and/or hardware quirks
   if (info->tx_fixup) {
         skb = info->tx_fixup (dev, skb, GFP_ATOMIC);
         if (!skb) {
                 /* packet collected; minidriver waiting for more */
                 if (info->flags & FLAG_MULTI_PACKET)
                         goto not_drop;
                 netif_dbg(dev, tx_err, dev->net, "can't tx_fixup skb\n");
                 goto drop;
         }
   }

   if (!(urb = usb_alloc_urb (0, GFP_ATOMIC))) {
         netif_dbg(dev, tx_err, dev->net, "no urb\n");
         goto drop;
   }

   entry = (struct skb_data *) skb->cb;
   entry->urb = urb;
   entry->dev = dev;

   usb_fill_bulk_urb (urb, dev->udev, dev->out,
                 skb->data, skb->len, tx_complete, skb);

   if (dev->can_dma_sg) {
         if (build_dma_sg(skb, urb) < 0)
                 goto drop;
   }
   length = urb->transfer_buffer_length;

   /* don't assume the hardware handles USB_ZERO_PACKET
     * NOTE:  strictly conforming cdc-ether devices should expect
     * the ZLP here, but ignore the one-byte packet.
     * NOTE2: CDC NCM specification is different from CDC ECM when
     * handling ZLP/short packets, so cdc_ncm driver will make short
     * packet itself if needed.
     */
   if (length % dev->maxpacket == 0) {
         if (!(info->flags & FLAG_SEND_ZLP)) {
                 if (!(info->flags & FLAG_MULTI_PACKET)) {
                         length++;
                         if (skb_tailroom(skb) && !urb->num_sgs) {
                                 skb->data[skb->len] = 0;
                                 __skb_put(skb, 1);
                         } else if (urb->num_sgs)
                                 sg_set_buf(&urb->sg[urb->num_sgs++],
                                                 dev->padding_pkt, 1);
                 }
         } else
                 urb->transfer_flags |= URB_ZERO_PACKET;
   }
   entry->length = urb->transfer_buffer_length = length;

   spin_lock_irqsave(&dev->txq.lock, flags);
   retval = gobi_usb_autopm_get_interface_async(dev->intf);
   if (retval < 0) {
         spin_unlock_irqrestore(&dev->txq.lock, flags);
         goto drop;
   }

#ifdef CONFIG_PM
   /* if this triggers the device is still a sleep */
   if (test_bit(EVENT_DEV_ASLEEP, &dev->flags)) 
   {
      /* transmission will be done in resume */
      usb_anchor_urb(urb, &dev->deferred);
      /* no use to process more packets */
      netif_stop_queue(net);
      usb_put_urb(urb);
      spin_unlock_irqrestore(&dev->txq.lock, flags);
      netdev_dbg(dev->net, "Delaying transmission for resumption\n");
      goto deferred;
   }
#endif
   #ifdef TX_URB_MONITOR
   iRet = get_usb_interface(urb, &b_usb_if_num);
   #endif //#ifdef TX_URB_MONITOR
   switch ((retval = usb_submit_urb (urb, GFP_ATOMIC))) 
   {
   case -EPIPE:
         netif_stop_queue (net);
         usbnet_defer_kevent (dev, EVENT_TX_HALT);
         gobi_usb_autopm_put_interface_async(dev->intf);
         break;
   default:
         gobi_usb_autopm_put_interface_async(dev->intf);
         netif_dbg(dev, tx_err, dev->net,
                   "tx: submit urb err %d\n", retval);
         break;
   case 0:
         net->trans_start = jiffies;
         __usbnet_queue_skb(&dev->txq, skb, tx_start);
         if (dev->txq.qlen >= TX_QLEN (dev))
                 netif_stop_queue (net);
   }

#ifdef TX_URB_MONITOR
/*
 * This can be called from here or from inside the
 * case 0 in the above switch. There will be one less
 * condition to check
 */
/*
 * Call URB_monitor() with true as the URB has been successfully 
 * submitted to the txq. 
 */
if ((0==iRet) && (0==retval))//(URB_monitor) && 
{
   URB_monitor(true, b_usb_if_num);
}
#endif //#ifdef TX_URB_MONITOR

   spin_unlock_irqrestore (&dev->txq.lock, flags);

   if (retval) {
      netif_dbg(dev, tx_err, dev->net, "drop, code %d\n", retval);
drop:
   dev->net->stats.tx_dropped++;
not_drop:
   if (skb)
      dev_kfree_skb_any (skb);
      if (urb) 
      {
         kfree(urb->sg);
         usb_free_urb(urb);
      }
   } else
      netif_dbg(dev, tx_queued, dev->net,
      "> tx, len %d, type 0x%x\n", length, skb->protocol);
#ifdef CONFIG_PM
deferred:
#endif
   return NETDEV_TX_OK;
}

#endif 
/********* 
#if (LINUX_VERSION_CODE >= KERNEL_VERSION( 3,12,0 ) &&\
               LINUX_VERSION_CODE < KERNEL_VERSION( 3,13,00))
               
***********/




