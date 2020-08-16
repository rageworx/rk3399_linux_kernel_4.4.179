/**
  Raphael K's rk3399 usb peripheral driver (usb-gadget)
  =====================================================================
  (C)2020 rageworx@gmail.com, https://rageworx.info
  All rights reverved for Raphael Kim
**/

// * WARNING *
// rk3399 linux using android APIs.
#ifndef android
    #define android
#endif

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/idr.h>
#include <linux/timer.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/moduleparam.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/types.h>
#include <linux/ctype.h>
#include <linux/cdev.h>
#include <linux/idr.h>
#include <asm/byteorder.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/uaccess.h>
#include <asm/unaligned.h>

#include <linux/usb/ch9.h>
#include <linux/usb/composite.h>
#include <linux/usb/cdc.h>
#include <linux/usb/gadget.h>

#include "rkudev.h"

#define MAX_BULKOUT_REQUESTS    (128)
#define MAX_BULKIN_REQUESTS     (128)
#define RKUDEV_MINORS           4

#define MODE_0                  0
#define MODE_1                  1
#define RKUDEV_NAME_S           "rkudev"
#define RKUDEV_G_S              "rk_usb_gadget"
#define RKUDEV_G_S_R            "RKUSBgadget"
#define RKUDEV_VERSION_S        "0.5.1.31"
#define RKUDEV_IOCTL_UID        0xC8
#define RKUDEVUID               RKUDEV_IOCTL_UID

#define RKUDEV_CTRL_IO_BASE   32

/* 
   -- Raph.Kay --
   USB speed definitions should be refer to here:
   https://docs.huihoo.com/doxygen/linux/kernel/3.7/uapi_2linux_2usb_2ch9_8h.html
*/
static u8   opt_opr_mode    = MODE_1;
static u8   opt_usb_speed   = USB_SPEED_UNKNOWN;
static u8   opt_debug_lvl   = 0;

#define SET_MODE \
    _IOW(RKUDEVUID, RKUDEV_CTRL_IO_BASE+1, unsigned int)

#define GET_MODE \
    _IOR(RKUDEVUID, RKUDEV_CTRL_IO_BASE+2, unsigned int)

#define GET_USBSPEED \
    _IOR(RKUDEVUID, RKUDEV_CTRL_IO_BASE+3, unsigned int)

#define GET_MAX_REQLEN \
    _IOR(RKUDEVUID, RKUDEV_CTRL_IO_BASE+4, unsigned int)

#define GET_MAX_BUFFLEN \
    _IOR(RKUDEVUID, RKUDEV_CTRL_IO_BASE+5, unsigned int)

///////////////////////////////////////////////////////////////////////////

static int      major = 0;
static int      minors = 0;
static struct   class* usb_gadget_class = NULL;

static DEFINE_IDA(rkudev_ida);
static DEFINE_MUTEX(rkudev_ida_lock);

struct ctrl_pkt 
{
    void*               buf;
    int                 len;
    struct list_head    list;
};

struct rkudev_t 
{
    u8                  data_id;
    spinlock_t          lock;
    spinlock_t          Txlock;
    spinlock_t          Rxlock;
    
    struct \
    usb_gadget*         gadget;
    s8                  interface;
    struct \
    usb_ep*             in_ep;
    struct \
    usb_ep*             out_ep;
    struct \
    list_head           rx_reqs;
    struct \
    list_head           rx_reqs_active;
    struct \
    list_head           rx_buffers;
    
    wait_queue_head_t   rx_wait;
    
    struct \
    usb_request*        current_rx_req;
    size_t              current_rx_bytes;
    u8*                 current_rx_buf;
    
    struct \
    list_head           tx_reqs;
    struct \
    list_head           tx_reqs_active;
    
    wait_queue_head_t   tx_wait;
    wait_queue_head_t   tx_flush_wait;
    atomic_t            read_excl;
    atomic_t            write_excl;
    wait_queue_head_t   read_wq;
    wait_queue_head_t   write_wq;
    bool                EnabledDevice;
    u8                  reset_rkudev;
    int                 minor;
    struct \
    cdev                rkudev_cdev;
    u8                  rkudev_cdev_open;
    wait_queue_head_t   wait;
    unsigned            q_len;
    char*               pnp_string;   /// no memory contains in driver.
    struct usb_function function;
};

static void setup_rx_req(struct rkudev_t *dev, struct usb_request *req);

static inline struct rkudev_t *func_to_mudev(struct usb_function* f)
{
    return container_of(f, struct rkudev_t, function);
}

/*-------------------------------------------------------------------------*/

#define USB_DESC_BUFSIZE        (256)
#define USB_BUFSIZE             (8192*4)
#define DEV_NOT_EMPTY(_x_)      likely(!list_empty(_x_))

/* interface descriptor: */

struct usb_interface_descriptor rkudev_data_intf = 
{
    .bLength            = sizeof( rkudev_data_intf ),
    .bDescriptorType    = USB_DT_INTERFACE,
    .bInterfaceNumber   = 0,
    .bAlternateSetting  = 0,
    .bNumEndpoints      = 2,
    .bInterfaceClass    = 0xFF,
    .bInterfaceSubClass = 0,
    .bInterfaceProtocol = 0,
    /* .iInterface = DYNAMIC */
};

/* full speed support: */
struct usb_endpoint_descriptor fs_rkudev_in_desc = 
{
    .bLength            = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType    = USB_DT_ENDPOINT,
    .bEndpointAddress   = USB_DIR_IN,
    .bmAttributes       = USB_ENDPOINT_XFER_BULK,
};

struct usb_endpoint_descriptor fs_rkudev_out_desc =
{
    .bLength            = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType    = USB_DT_ENDPOINT,
    .bEndpointAddress   = USB_DIR_OUT,
    .bmAttributes       = USB_ENDPOINT_XFER_BULK,
};

struct usb_descriptor_header *rkudev_fs_function[] = 
{
    (struct usb_descriptor_header*) &rkudev_data_intf,
    (struct usb_descriptor_header*) &fs_rkudev_in_desc,
    (struct usb_descriptor_header*) &fs_rkudev_out_desc,
    NULL,
};

/* high speed support: */
struct usb_endpoint_descriptor hs_rkudev_in_desc = 
{
    .bLength            = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType    = USB_DT_ENDPOINT,
    .bEndpointAddress   = USB_DIR_IN,
    .bmAttributes       = USB_ENDPOINT_XFER_BULK,
    .wMaxPacketSize     = cpu_to_le16(512),
};

struct usb_endpoint_descriptor hs_rkudev_out_desc = 
{
    .bLength            = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType    = USB_DT_ENDPOINT,
    .bEndpointAddress   = USB_DIR_OUT,
    .bmAttributes       = USB_ENDPOINT_XFER_BULK,
    .wMaxPacketSize     = cpu_to_le16(512),
};

struct usb_descriptor_header* rkudev_hs_function[] = 
{
    (struct usb_descriptor_header*) &rkudev_data_intf,
    (struct usb_descriptor_header*) &hs_rkudev_in_desc,
    (struct usb_descriptor_header*) &hs_rkudev_out_desc,
    NULL,
};

struct usb_endpoint_descriptor ss_rkudev_in_desc = 
{
    .bLength            = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType    = USB_DT_ENDPOINT,
    .bEndpointAddress   = USB_DIR_IN,
    .bmAttributes       = USB_ENDPOINT_XFER_BULK,
    .wMaxPacketSize     = cpu_to_le16(1024),
};

struct usb_endpoint_descriptor ss_rkudev_out_desc = 
{
    .bLength            = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType    = USB_DT_ENDPOINT,
    .bEndpointAddress   = USB_DIR_OUT,
    .bmAttributes       = USB_ENDPOINT_XFER_BULK,
    .wMaxPacketSize     = cpu_to_le16(1024),
};

struct usb_ss_ep_comp_descriptor ss_rkudev_bulk_comp_desc = 
{
    .bLength            = sizeof(ss_rkudev_bulk_comp_desc),
    .bDescriptorType    = USB_DT_SS_ENDPOINT_COMP,
    .bMaxBurst          = 8,

    /* the following 2 values can be tweaked if necessary */
    /* .bMaxBurst =     0, */
    /* .bmAttributes =  0, */
};

struct usb_descriptor_header* rkudev_ss_function[] = 
{
    (struct usb_descriptor_header*) &rkudev_data_intf,
    (struct usb_descriptor_header*) &ss_rkudev_in_desc,
    (struct usb_descriptor_header*) &ss_rkudev_bulk_comp_desc,
    (struct usb_descriptor_header*) &ss_rkudev_out_desc,
    (struct usb_descriptor_header*) &ss_rkudev_bulk_comp_desc,
    NULL,
};

static inline \
struct usb_endpoint_descriptor* ep_desc( struct usb_gadget *gadget,
                                         struct usb_endpoint_descriptor *fs,
                                         struct usb_endpoint_descriptor *hs,
                                         struct usb_endpoint_descriptor *ss )
{
    opt_usb_speed = gadget->speed;

    switch ( gadget->speed )
    {
        case USB_SPEED_SUPER:
            return ss;
            
        case USB_SPEED_HIGH:
            return hs;
            
        default:
            return fs;
    }
}

static inline int rkudev_lock( atomic_t* excl )
{
    if ( atomic_inc_return( excl ) == 1 )
    {
        return 0;
    }
    
    atomic_dec( excl );
    return -EBUSY;
}

static inline void rkudev_unlock( atomic_t *excl )
{
    atomic_dec( excl );
}

/*-------------------------------------------------------------------------*/

static struct \
usb_request* rkudev_req_alloc( struct usb_ep* ep, unsigned len, gfp_t gfp_flags )
{
    struct usb_request* req = usb_ep_alloc_request(ep, gfp_flags);

    if (req != NULL) 
    {
        req->length = len;
        req->buf = kmalloc(len, gfp_flags);
        
        if (req->buf == NULL) 
        {
            usb_ep_free_request(ep, req);
            return NULL;
        }
    }

    return req;
}

static void rkudev_req_free( struct usb_ep* ep, struct usb_request* req )
{
    if ( ( ep != NULL ) && ( req != NULL ) )
    {
        kfree(req->buf);
        usb_ep_free_request(ep, req);
    }
}

/*-------------------------------------------------------------------------*/

static void rx_complete( struct usb_ep* ep, struct usb_request* req )
{
    struct \
    rkudev_t*  dev     = ep->driver_data;
    int             status  = req->status;
    unsigned long flags     = 0;

    if ( opt_debug_lvl > 0 )
    {
        pr_info( "%s: %s\n", RKUDEV_NAME_S , __func__ );
    }

    spin_lock_irqsave( &dev->Rxlock, flags );
    list_del_init( &req->list );
    spin_unlock_irqrestore( &dev->Rxlock, flags );

    switch (status) 
    {
        case 0:
            if ( req->actual > 0 ) 
            {
                if ( opt_opr_mode == MODE_0 )
                {
                    spin_lock_irqsave( &dev->Rxlock, flags );
                    list_add_tail( &req->list, &dev->rx_buffers );
                    spin_unlock_irqrestore( &dev->Rxlock, flags );
                }
                else 
                if ( opt_opr_mode == MODE_1 )
                {
                    spin_lock_irqsave( &dev->Rxlock, flags );
                    setup_rx_req( dev, req );
                    spin_unlock_irqrestore( &dev->Rxlock, flags );
                }
                else
                {
                    spin_lock_irqsave( &dev->Rxlock, flags );
                    list_add( &req->list, &dev->rx_reqs );
                    spin_unlock_irqrestore( &dev->Rxlock, flags );
                }
            }
            else 
            {
                spin_lock_irqsave( &dev->Rxlock, flags );
                list_add( &req->list, &dev->rx_reqs );
                spin_unlock_irqrestore( &dev->Rxlock, flags );
            }
            break;

        /* software-driven interface shutdown */
        case -ECONNRESET:       /* unlink */
        case -ESHUTDOWN:        /* disconnect etc */
            if ( opt_debug_lvl > 1 )
            {
                printk( "%s: rx shutdown, code %d\n", 
                        RKUDEV_NAME_S,
                        status);
            }
            spin_lock_irqsave( &dev->Rxlock, flags );
            list_add( &req->list, &dev->rx_reqs );
            spin_unlock_irqrestore( &dev->Rxlock, flags );
            break;

        /* for hardware automagic (such as pxa) */
        case -ECONNABORTED:     /* endpoint reset */
            if ( opt_debug_lvl > 1 )
            {
                printk( "%s: rx <%s> reset(-ECONNABORTED)\n", 
                        RKUDEV_NAME_S,
                        ep->name );
            }
            spin_lock_irqsave( &dev->Rxlock, flags );
            list_add( &req->list, &dev->rx_reqs );
            spin_unlock_irqrestore( &dev->Rxlock, flags );
            break;

        /* data overrun */
        case -EOVERFLOW:
            if ( opt_debug_lvl > 0 )
            {
                printk( "%s: rx overflow, status %d\n", 
                        RKUDEV_NAME_S,
                        status);
            }
            // -- do not break here -- 
        default:
            if ( opt_debug_lvl > 1 )
            {
                printk( "%s: rx status changed to %d\n", 
                        RKUDEV_NAME_S,
                        status);
            }
            spin_lock_irqsave( &dev->Rxlock, flags );
            list_add( &req->list, &dev->rx_reqs );
            spin_unlock_irqrestore( &dev->Rxlock, flags );
            break;
    } /// of switch()

    wake_up_interruptible( &dev->rx_wait );
}

static void tx_complete( struct usb_ep* ep, struct usb_request* req )
{
    struct rkudev_t* dev    = ep->driver_data;
    unsigned long    flags  = 0;

    if ( opt_debug_lvl > 0 )
    {
        pr_info( "%s: %s\n", RKUDEV_NAME_S, __func__ );
    }

    switch (req->status) 
    {
        case -ECONNRESET:       /* unlink */
        case -ESHUTDOWN:        /* disconnect etc */
            break;
            
        case 0:
            break;
            
        default:
            if ( opt_debug_lvl > 0 )
            {
                printk( "%s: tx_complete, error = %d\n", 
                        RKUDEV_NAME_S,
                        req->status);
            }
            break;
    }

    spin_lock_irqsave( &dev->Txlock, flags );

    list_del_init( &req->list );
    list_add( &req->list, &dev->tx_reqs );

    if ( opt_debug_lvl > 0 )
    {
        printk( "%s: tx_complete req(%p) to ready\n", 
                RKUDEV_NAME_S,
                req);
    }

    spin_unlock_irqrestore( &dev->Txlock, flags );
    wake_up_interruptible( &dev->tx_wait );
}

/*-------------------------------------------------------------------------*/
static int rkudev_open( struct inode* inode, struct file* fd )
{
    struct \
    rkudev_t*  dev     = NULL;
    unsigned long   flags   = 0;
    int             ret     = -EBUSY;

    dev = container_of( inode->i_cdev, struct rkudev_t, rkudev_cdev );

    if ( dev == NULL )
    {
        return -EINVAL;
    }

    fd->private_data = dev;
    spin_lock_irqsave( &dev->lock, flags );

    if (!dev->rkudev_cdev_open)
    {
        dev->rkudev_cdev_open++;
    }
    else
    {
        dev->rkudev_cdev_open++;
    }

    ret = 0;
    spin_unlock_irqrestore( &dev->lock, flags );
    
    return ret;
}

static int rkudev_close( struct inode *inode, struct file *fd )
{
    struct \
    rkudev_t*  dev     = fd->private_data;
    unsigned long   flags   = 0;

    if ( dev == NULL )
    {
        return -EINVAL;
    }

    spin_lock_irqsave( &dev->lock, flags );

    if ( dev->rkudev_cdev_open )
    {
        dev->rkudev_cdev_open--;
    }

    fd->private_data = NULL;
    spin_unlock_irqrestore( &dev->lock, flags );

    return 0;
}

static void setup_rx_req( struct rkudev_t *dev, struct usb_request *req )
{
    int error = -1;
    
    req->length   = USB_BUFSIZE;
    req->complete = rx_complete;
    
    error = usb_ep_queue(dev->out_ep, req, GFP_ATOMIC);
    
    if ( error )
    {
        if ( opt_debug_lvl > 1 )
        {
            printk( "%s: setuo_rx_req,usb_ep_queue() error = %d\n", 
                    RKUDEV_NAME_S,
                    error);
        }
        list_add(&req->list, &dev->rx_reqs);
    }
}

static void setup_rx_reqs( struct rkudev_t* dev )
{
    struct usb_request* req = NULL;

    while ( DEV_NOT_EMPTY(&dev->rx_reqs) ) 
    {
        int error;

        req = container_of(dev->rx_reqs.next, struct usb_request, list);
        list_del_init(&req->list);
        
        req->length = USB_BUFSIZE;
        req->complete = rx_complete;

        error = usb_ep_queue(dev->out_ep, req, GFP_ATOMIC);
        if (error) 
        {
            if ( opt_debug_lvl > 1 )
            {
                printk( "%s: setuo_rx_reqs, usb_ep_queue() error = %d\n", 
                        RKUDEV_NAME_S,
                        error);
            }
            list_add(&req->list, &dev->rx_reqs);
            break;
        }
        else 
        if ( list_empty( &req->list ) )
        {
            list_add(&req->list, &dev->rx_reqs_active);
        }
    }
}

static ssize_t rkudev_read( struct file* fd, char __user* buf, size_t len, loff_t* ptr )
{
    struct \
    rkudev_t*      dev                 = fd->private_data;
    unsigned long       flags               = 0;
    size_t              size                = 0;
    size_t              bytes_copied        = 0;
    struct \
    usb_request*        req                 = NULL;
    struct \
    usb_request*        current_rx_req      = NULL;
    size_t              current_rx_bytes    = 0;
    u8*                 current_rx_buf      = NULL;

    if( opt_opr_mode != MODE_0 )
        return -EINVAL;

    if (len == 0)
        return -EINVAL;

    if (dev->EnabledDevice == false)
        return -EINVAL;

    spin_lock_irqsave(&dev->Rxlock, flags);
    dev->reset_rkudev = 0;
    setup_rx_reqs(dev);

    current_rx_req = dev->current_rx_req;
    current_rx_bytes = dev->current_rx_bytes;
    current_rx_buf = dev->current_rx_buf;
    dev->current_rx_req = NULL;
    dev->current_rx_bytes = 0;
    dev->current_rx_buf = NULL;

    if ( ( current_rx_bytes == 0 ) && DEV_NOT_EMPTY(&dev->rx_buffers) )
     {

        spin_unlock_irqrestore(&dev->Rxlock, flags);

        if ( fd->f_flags & (O_NONBLOCK | O_NDELAY) ) 
        {
            return -EAGAIN;
        }

        wait_event_interruptible( dev->rx_wait, DEV_NOT_EMPTY(&dev->rx_buffers) );
        spin_lock_irqsave( &dev->Rxlock, flags );
    }

    while ( ( current_rx_bytes || DEV_NOT_EMPTY(&dev->rx_buffers) )
            && len )
    {
        if (current_rx_bytes == 0) 
        {
            req = container_of( dev->rx_buffers.next, 
                                struct usb_request, list );
            list_del_init(&req->list);

            if ( req->actual && req->buf ) 
            {
                current_rx_req = req;
                current_rx_bytes = req->actual;
                current_rx_buf = req->buf;
            }
            else 
            {
                list_add(&req->list, &dev->rx_reqs);
                continue;
            }
        }

        spin_unlock_irqrestore( &dev->Rxlock, flags );

        if (len > current_rx_bytes)
        {
            size = current_rx_bytes;
        }
        else
        {
            size = len;
        }

        size -= copy_to_user( buf, current_rx_buf, size );
        bytes_copied += size;
        len -= size;
        buf += size;

        spin_lock_irqsave(&dev->Rxlock, flags);

        /* We've disconnected or reset so return. */
        if ( dev->reset_rkudev ) 
        {
            list_add( &current_rx_req->list, &dev->rx_reqs );
            spin_unlock_irqrestore( &dev->Rxlock, flags );
            return -EAGAIN;
        }

        if (size < current_rx_bytes) 
        {
            current_rx_bytes -= size;
            current_rx_buf += size;
        }
        else 
        {
            list_add(&current_rx_req->list, &dev->rx_reqs);
            current_rx_bytes = 0;
            current_rx_buf = NULL;
            current_rx_req = NULL;
        }
    }

    dev->current_rx_req = current_rx_req;
    dev->current_rx_bytes = current_rx_bytes;
    dev->current_rx_buf = current_rx_buf;

    spin_unlock_irqrestore(&dev->Rxlock, flags);

    if ( bytes_copied )
    {
        return bytes_copied;
    }
    
    return -EAGAIN;
}

static ssize_t rkudev_write( struct file* fd, const char __user* buf, size_t len, loff_t* ptr )
{
    struct rkudev_t*    dev = fd->private_data;
    unsigned long       flags;
    size_t              size;
    size_t              bytes_copied = 0;
    struct usb_request* req = NULL;

    if ( opt_debug_lvl > 0 )
    {
        pr_info( "%s: %s\n", RKUDEV_NAME_S, __func__ );
    }

    if (dev->EnabledDevice == false)
        return -EINVAL;

    if (len == 0)
        return -EINVAL;

    spin_lock_irqsave( &dev->Txlock, flags );
    dev->reset_rkudev = 0;

    if ( DEV_NOT_EMPTY(&dev->tx_reqs) ) 
    {

        if ( fd->f_flags & ( O_NONBLOCK | O_NDELAY ) ) 
        {
            spin_unlock_irqrestore(&dev->Txlock, flags);
            return -EAGAIN;
        }

        spin_unlock_irqrestore( &dev->Txlock, flags );
        wait_event_interruptible( dev->tx_wait, DEV_NOT_EMPTY(&dev->tx_reqs) );
        spin_lock_irqsave( &dev->Txlock, flags );
    }

    while ( DEV_NOT_EMPTY(&dev->tx_reqs) && len) 
    {
        if (dev->EnabledDevice == false)
        {
            spin_unlock_irqrestore(&dev->Txlock, flags);
            return -EINVAL;
        }

        if (len > USB_BUFSIZE)
            size = USB_BUFSIZE;
        else
            size = len;

        req = container_of(dev->tx_reqs.next, struct usb_request,
            list);
        list_del_init(&req->list);

        if ( opt_debug_lvl > 1 )
        {
            printk( "%s: get req(%p) from ready\n", 
                    RKUDEV_NAME_S,
                    req);
        }

        req->complete = tx_complete;
        req->length = size;

        if (len > size)
            req->zero = 0;
        else
            req->zero = ( (len % dev->in_ep->maxpacket) == 0 );

        if ( copy_from_user( req->buf, buf, size ) ) 
        {
            if ( opt_debug_lvl > 0 )
            {
                printk( "%s: copy_from_user error, put req(%p) to ready\n",
                        RKUDEV_NAME_S,
                        req);
            }

            list_add( &req->list, &dev->tx_reqs );
            spin_unlock_irqrestore( &dev->Txlock, flags );
            return bytes_copied;
        }

        bytes_copied += size;
        len -= size;
        buf += size;

        if ( dev->reset_rkudev ) 
        {

            list_add(&req->list, &dev->tx_reqs);
            spin_unlock_irqrestore(&dev->Txlock, flags);
            return -EAGAIN;
        }

        if ( opt_debug_lvl > 1 )
        {
            printk( "%s: put req(%p) to active\n", 
                    RKUDEV_NAME_S,
                    req);
        }

        list_add(&req->list, &dev->tx_reqs_active);

        if ( opt_debug_lvl > 1 )
        {
            printk( "%s: usb_ep_queue call\n", RKUDEV_NAME_S );
        }

        if ( usb_ep_queue( dev->in_ep, req, GFP_ATOMIC ) != 0 ) 
        {
            list_del_init( &req->list );
            list_add( &req->list, &dev->tx_reqs );

            if ( opt_debug_lvl > 0 )
            {
                printk( "%s: usb_ep_queue return error, put req(%p) to ready\n", 
                        RKUDEV_NAME_S,
                        req);
            }
            spin_unlock_irqrestore(&dev->Txlock, flags);
            return -EAGAIN;
        }
    }

    spin_unlock_irqrestore(&dev->Txlock, flags);

    if ( bytes_copied )
    {
        return bytes_copied;
    }

    return -EAGAIN;
}

static long rkudev_ioctl( struct file* fd, unsigned int code, unsigned long arg )
{
    long ret = -EINVAL;
    
    if ( arg == 0 )
        return ret;
    
    switch( code )
    {
        case SET_MODE:
        {
            unsigned int* pmode = (unsigned int*)arg;

            switch( *pmode )
            {
                case MODE_0:
                case MODE_1:
                    opt_opr_mode = *pmode;
                    ret = 0;
                    break;
                
                default:
                    ret = -EINVAL;
                    break;
            }
        }
        break;
        
        case GET_MODE:
        {
            unsigned int* pmode = (unsigned int*)arg;
            *pmode = opt_opr_mode;
            ret = 0;
        }
        break;

        case GET_USBSPEED:
        {
            unsigned int* pusbspd = (unsigned int*)arg;
            *pusbspd = (unsigned int)opt_usb_speed;
            ret = 0;
        }
        break;

        case GET_MAX_REQLEN:
        {
            unsigned int* plen = (unsigned int*)arg;
            *plen =(unsigned int)MAX_BULKIN_REQUESTS;
            ret = 0;
        }
        break;


        case GET_MAX_BUFFLEN:
        {
            unsigned int* plen = (unsigned int*)arg;
            *plen = (unsigned int)USB_BUFSIZE;
            ret = 0;
        }
        break;

    } /// of switch() --

    return ret;
}

static const struct file_operations rkudev_io_operations = 
{
    .owner          = THIS_MODULE,
    .open           = rkudev_open,
    .read           = rkudev_read,
    .write          = rkudev_write,
    .unlocked_ioctl = rkudev_ioctl,
    .release        = rkudev_close,
    .llseek         = noop_llseek,
};

/*-------------------------------------------------------------------------*/

static int set_rkudev_interface( struct rkudev_t* dev )
{
    int result = 0;

    if ( opt_debug_lvl > 0 )
    {
        pr_info( "%s: %s\n", RKUDEV_NAME_S, __func__ );
    }

    dev->in_ep->desc = ep_desc( dev->gadget, 
                                &fs_rkudev_in_desc, 
                                &hs_rkudev_in_desc,
                                &ss_rkudev_in_desc );
    dev->in_ep->driver_data = dev;
    dev->out_ep->desc = ep_desc( dev->gadget, 
                                 &fs_rkudev_out_desc,
                                 &hs_rkudev_out_desc, 
                                 &ss_rkudev_out_desc );
    dev->out_ep->driver_data = dev;

    result = usb_ep_enable(dev->in_ep);
    if (result != 0) 
    {
        DBG(dev, "enable %s --> %d\n", dev->in_ep->name, result);
        goto done;
    }

    result = usb_ep_enable(dev->out_ep);
    if (result != 0) 
    {
        DBG(dev, "enable %s --> %d\n", dev->in_ep->name, result);
        goto done;
    }

done:
    if ( result != 0 ) 
    {
        (void)usb_ep_disable(dev->in_ep);
        (void)usb_ep_disable(dev->out_ep);
        dev->in_ep->desc = NULL;
        dev->out_ep->desc = NULL;
    }

    return result;
}

static void reset_rkudev_interface( struct rkudev_t* dev )
{
    unsigned long   flags;

    if ( opt_debug_lvl > 0 )
    {
        pr_info( "%s: %s\n", RKUDEV_NAME_S, __func__ );
    }

    if (dev->interface < 0)
    {
        if ( opt_debug_lvl > 0 )
        {
            printk( "%s: error, interface is %d.\n", 
                    RKUDEV_NAME_S,
                    dev->interface );
        }
        return;
    }

    if (dev->in_ep->desc)
        usb_ep_disable(dev->in_ep);

    if (dev->out_ep->desc)
        usb_ep_disable(dev->out_ep);

    spin_lock_irqsave(&dev->lock, flags);
    spin_lock_irqsave(&dev->Txlock, flags);
    spin_lock_irqsave(&dev->Rxlock, flags);
    dev->in_ep->desc = NULL;
    dev->out_ep->desc = NULL;
    dev->interface = -1;
    spin_unlock_irqrestore(&dev->Rxlock, flags);
    spin_unlock_irqrestore(&dev->Txlock, flags);
    spin_unlock_irqrestore(&dev->lock, flags);
}

static int set_interface( struct rkudev_t* dev, unsigned number )
{
    int result = 0;

    if ( opt_debug_lvl > 0 )
    {
        pr_info( "%s: %s\n", RKUDEV_NAME_S, __func__ );
    }

    reset_rkudev_interface(dev);

    result = set_rkudev_interface(dev);
    if (result)
        reset_rkudev_interface(dev);
    else
        dev->interface = number;
    if (!result)
        INFO(dev, "Using interface %x\n", number);

    return result;
}

static int rkudev_func_setup( struct usb_function* f,
                                 const struct usb_ctrlrequest* ctrl )
{
    return -EOPNOTSUPP;
}

static int rkudev_func_bind( struct usb_configuration* c,
                                struct usb_function *f )
{
    struct usb_gadget*          gadget = c->cdev->gadget;
    struct rkudev_t*       dev = func_to_mudev(f);
    struct device*              pdev = NULL;
    struct usb_composite_dev*   cdev = c->cdev;
    struct usb_ep*              in_ep = NULL;
    struct usb_ep*              out_ep = NULL;
    struct usb_request*         req = NULL;
    dev_t                       devt;
    int                         id = -1;
    int                         ret = 0;
    u32                         cnt;
    
    printk( "RaphK's USB peripheral driver v%s\n",
            RKUDEV_VERSION_S );

    id = usb_interface_id(c, f);
 
    if ( opt_debug_lvl > 1 )
    {
        printk( "%s: USB interface ID = %d\n", RKUDEV_NAME_S, id );
    }

    if (id < 0)
        return id;

    dev->data_id = id;
    rkudev_data_intf.bInterfaceNumber = id;

    dev->gadget = gadget;
    in_ep = usb_ep_autoconfig( cdev->gadget, &fs_rkudev_in_desc );
    if (!in_ep)
    {
        ERROR(dev, "usb_ep_autoconfig error, in_ep\n");
        return -ENODEV;
    }

    if ( opt_debug_lvl > 1 )
    {
        printk( "%s: usb_ep_autoconfig( in ) OK.\n", RKUDEV_NAME_S );
    }

    out_ep = usb_ep_autoconfig( cdev->gadget, &fs_rkudev_out_desc );
    if (!out_ep)
    {
        ERROR(dev, "usb_ep_autoconfig error, out_ep\n");
        return -ENODEV;
    }

    if ( opt_debug_lvl > 1 )
    {
        printk( "%s: usb_ep_autoconfig( out ) OK.\n", RKUDEV_NAME_S );
    }

    dev->in_ep = in_ep;
    dev->out_ep = out_ep;
    dev->EnabledDevice = false;

    hs_rkudev_in_desc.bEndpointAddress = \
            fs_rkudev_in_desc.bEndpointAddress;
    hs_rkudev_out_desc.bEndpointAddress = \
            fs_rkudev_out_desc.bEndpointAddress;

    ss_rkudev_in_desc.bEndpointAddress = \
            fs_rkudev_in_desc.bEndpointAddress;
    ss_rkudev_out_desc.bEndpointAddress = \
            fs_rkudev_out_desc.bEndpointAddress;

#ifndef android
    ret = usb_assign_descriptors( f, 
                                  rkudev_fs_function, 
                                  rkudev_hs_function,
                                  rkudev_ss_function, 
                                  NULL );
#else
    ret = usb_assign_descriptors( f, 
                                  rkudev_fs_function, 
                                  rkudev_hs_function,
                                  rkudev_ss_function );
#endif  
    if ( ret != 0 )
    {
        ERROR( dev, "usb_assign_descriptor() failed.\n" );
        return ret;
    }

    if ( opt_debug_lvl > 1 )
    {
        printk( "%s: usb_assign_descriptor() OK.\n", RKUDEV_NAME_S );
    }

    ret = -ENOMEM;

    for ( cnt=0; cnt<MAX_BULKIN_REQUESTS; cnt++ ) 
    {
        req = rkudev_req_alloc( dev->in_ep, USB_BUFSIZE, GFP_KERNEL );

        if ( req == 0 )
            goto fail_tx_reqs;
        
        list_add( &req->list, &dev->tx_reqs );
    }

    DBG( dev, "USB requested lists : %d\n", cnt );

    if( list_empty(&dev->tx_reqs) )
    {
        ERROR(dev, "tx_reqs Failed: rkudev\n");
        goto fail_rx_reqs;
    }

    for ( cnt=0; cnt<MAX_BULKOUT_REQUESTS; cnt++ )
    {
        req = rkudev_req_alloc( dev->out_ep, USB_BUFSIZE, GFP_KERNEL );
        
        if ( req == 0 )
            goto fail_rx_reqs;
        
        list_add( &req->list, &dev->rx_reqs );
    }

    if ( opt_debug_lvl > 0 )
    {
        printk( "%s: device creating as %s%d\n", 
                RKUDEV_NAME_S,
                RKUDEV_NAME_S,
                dev->minor );
    }

    devt = MKDEV(major, dev->minor);
    pdev = device_create( usb_gadget_class, 
                          NULL, 
                          devt,
                          NULL, 
                          "%s%d", 
                          RKUDEV_NAME_S,
                          dev->minor );
    if ( IS_ERR(pdev) ) 
    {
        ERROR( dev, 
               "Failed to create device: %s%d\n", 
               RKUDEV_NAME_S,
               dev->minor );
        ret = PTR_ERR(pdev);
        goto fail_rx_reqs;
    }

    cdev_init( &dev->rkudev_cdev, &rkudev_io_operations );
    dev->rkudev_cdev.owner = THIS_MODULE;
    ret = cdev_add( &dev->rkudev_cdev, devt, 1 );
    if ( ret != 0 ) 
    {
        ERROR(dev, "Failed to open char device\n");
        goto fail_cdev_add;
    }

    init_waitqueue_head(&dev->read_wq);
    init_waitqueue_head(&dev->write_wq);

    atomic_set(&dev->read_excl, 0);
    atomic_set(&dev->write_excl, 0);

    if ( opt_debug_lvl > 1 )
    {
        printk("%s: func_bind completed.\n", RKUDEV_NAME_S );
    }

    return 0;

fail_cdev_add:
    device_destroy(usb_gadget_class, devt);

fail_rx_reqs:
    if ( opt_debug_lvl > 1 )
    {
        printk("%s: dev->rx_reqs free!!\n", RKUDEV_NAME_S );
    }
    while ( !list_empty(&dev->rx_reqs) ) 
    {
        req = container_of( dev->rx_reqs.next, 
                            struct usb_request, 
                            list );
        list_del( &req->list );
        rkudev_req_free( dev->out_ep, req );
    }

fail_tx_reqs:
    if ( opt_debug_lvl > 1 )
    {
        printk("%s: dev->tx_reqs free!!\n", RKUDEV_NAME_S );
    }
    while ( !list_empty(&dev->tx_reqs) ) 
    {
        req = container_of( dev->tx_reqs.next, 
                            struct usb_request, 
                            list );
        list_del(&req->list);
        rkudev_req_free( dev->in_ep, req );
    }

    return ret;
}

static int rkudev_func_set_alt( struct usb_function* f,
                                   unsigned intf, 
                                   unsigned alt )
{
    struct rkudev_t *dev = func_to_mudev(f);
    int ret = -EINVAL;

    if ( opt_debug_lvl > 0 )
    {
        pr_info( "%s: %s intf %d alt %d\n", 
                 RKUDEV_NAME_S,
                 __func__, 
                 intf, alt );
    }

    ret = set_interface(dev, intf);

    setup_rx_reqs(dev);
    dev->EnabledDevice = true;
    return ret;
}

static int rkudev_func_get_alt( struct usb_function *f, unsigned intf )
{
    return 0;
}

static void rkudev_func_disable( struct usb_function *f )
{
    struct rkudev_t *dev = func_to_mudev(f);

    if ( opt_debug_lvl > 0 )
    {
        pr_info( "%s: %s\n", RKUDEV_NAME_S, __func__ );
    }

    dev->EnabledDevice = false;

    reset_rkudev_interface(dev);
}

static inline \
struct f_rkudev_opts* to_f_rkudev_opts( struct config_item* item )
{
    if ( opt_debug_lvl > 0 )
    {
        pr_info( "%s: %s\n", RKUDEV_NAME_S, __func__ );
    }
  
    if ( item == NULL )
        return NULL;
    
    return container_of( to_config_group(item), 
                         struct f_rkudev_opts,
                         func_inst.group );
}

static void rkudev_attr_release( struct config_item* item )
{
    struct f_rkudev_opts* opts = to_f_rkudev_opts(item);
    
    if ( opts == NULL )
        return;

    if ( opt_debug_lvl > 0 )
    {
        pr_info( "%s: %s\n", RKUDEV_NAME_S, __func__ );
    }

    usb_put_function_instance( &opts->func_inst );
}

static struct configfs_item_operations rkudev_item_ops = 
{
    .release = rkudev_attr_release,
};

static ssize_t f_rkudev_opts_pnp_string_show( struct config_item* item,
                                              char* page )
{
    struct f_rkudev_opts *opts = to_f_rkudev_opts(item);
    int result = 0;

    if ( item == NULL )
        return result;

    mutex_lock( &opts->lock );

    if ( !opts->pnp_string )
        goto unlock;

    result = strlcpy(page, opts->pnp_string, PAGE_SIZE);
    
    if ( result >= PAGE_SIZE ) 
    {
        result = PAGE_SIZE;
    }
    else 
    if ( ( page[result - 1] != '\n' ) && ( result + 1 < PAGE_SIZE ) )
    {
        page[result++] = '\n';
        page[result] = '\0';
    }

unlock:
    mutex_unlock( &opts->lock );

    return result;
}

static ssize_t f_rkudev_opts_pnp_string_store( struct config_item* item,
                                               const char* page, 
                                               size_t len )
{
    struct f_rkudev_opts* opts = to_f_rkudev_opts( item );
    char* new_pnp = NULL;
    int result = 0;

    if ( opts == NULL )
        return result;

    mutex_lock( &opts->lock );

    new_pnp = kstrndup(page, len, GFP_KERNEL);
    
    if ( !new_pnp ) 
    {
        result = -ENOMEM;
        goto unlock;
    }

    if ( opts->pnp_string_allocated )
        kfree(opts->pnp_string);

    opts->pnp_string_allocated = true;
    opts->pnp_string = new_pnp;
    result = len;
    
unlock:
    mutex_unlock( &opts->lock );

    return result;
}

CONFIGFS_ATTR( f_rkudev_opts_, pnp_string );

static ssize_t f_rkudev_opts_q_len_show( struct config_item* item,
                                         char* page )
{
    struct f_rkudev_opts* opts = to_f_rkudev_opts( item );
    int result = 0;

    if ( opts == NULL )
        return result;

    mutex_lock( &opts->lock );
    result = sprintf( page, "%d\n", opts->q_len );
    mutex_unlock( &opts->lock );

    return result;
}

static ssize_t f_rkudev_opts_q_len_store( struct config_item* item,
                                          const char* page, 
                                          size_t len )
{
    struct f_rkudev_opts* opts = to_f_rkudev_opts( item );
    int ret = 0;
    u16 num = 0;

    if ( opts == NULL )
        return ret;

    mutex_lock(&opts->lock);
    
    if ( opts->refcnt ) 
    {
        ret = -EBUSY;
        goto end;
    }

    ret = kstrtou16( page, 0, &num );

    if (ret)
        goto end;

    opts->q_len = (unsigned)num;
    ret = len;

end:
    mutex_unlock(&opts->lock);
    return ret;
}

CONFIGFS_ATTR( f_rkudev_opts_, q_len );

static struct configfs_attribute *rkudev_attrs[] = 
{
    &f_rkudev_opts_attr_pnp_string,
    &f_rkudev_opts_attr_q_len,
    NULL,
};

static struct config_item_type rkudev_func_type = 
{
    .ct_item_ops    = &rkudev_item_ops,
    .ct_attrs       = rkudev_attrs,
    .ct_owner       = THIS_MODULE,
};

static inline int gdev_get_minor(void)
{
    int ret;

    ret = ida_simple_get( &rkudev_ida, 0, 0, GFP_KERNEL );
    
    if ( ret >= RKUDEV_MINORS ) 
    {
        ida_simple_remove(&rkudev_ida, ret);
        ret = -ENODEV;
    }

    return ret;
}

static inline void gdev_put_minor( int minor )
{
    ida_simple_remove( &rkudev_ida, minor );
}

static int  gdev_setup(int);
static void gdev_cleanup(void);

static void gdev_free_inst( struct usb_function_instance* f )
{
    struct f_rkudev_opts* opts = NULL;

    if ( opt_debug_lvl > 0 )
    {
        pr_info( "%s: %s\n", RKUDEV_NAME_S, __func__ );
    }

    opts = container_of( f, struct f_rkudev_opts, func_inst );

    mutex_lock(&rkudev_ida_lock);

    gdev_put_minor(opts->minor);
    
#ifndef android
    if ( ida_is_empty(&rkudev_ida) )
#else
    if ( idr_is_empty(&rkudev_ida.idr) )
#endif
    {
        gdev_cleanup();
    }

    mutex_unlock(&rkudev_ida_lock);

    if ( opts->pnp_string_allocated )
        kfree( opts->pnp_string );
    
    kfree(opts);
}

static void gdev_free( struct usb_function *f )
{
    struct rkudev_t* dev = func_to_mudev(f);
    struct f_rkudev_opts *opts;

    if ( dev == NULL )
        return;

    if ( opt_debug_lvl > 0 )
    {
        pr_info( "%s: %s\n", RKUDEV_NAME_S, __func__ );
    }

    opts = container_of( f->fi, struct f_rkudev_opts, func_inst );
    kfree(dev);
    mutex_lock( &opts->lock );
    opts->refcnt--;
    mutex_unlock( &opts->lock );
}

static void rkudev_func_unbind( struct usb_configuration* c,
                                   struct usb_function* f )
{
    struct rkudev_t*    dev = NULL;
    struct usb_request* req = NULL;

    if ( opt_debug_lvl > 0 )
    {
        pr_info( "%s: %s\n", RKUDEV_NAME_S, __func__ );
    }

    dev = func_to_mudev(f);
    
    if ( dev == NULL )
    {
        DBG( dev, "dev is NULL ?\n" );
        return;
    }

    device_destroy( usb_gadget_class, MKDEV(major, dev->minor) );

    cdev_del( &dev->rkudev_cdev );

    WARN_ON( !list_empty(&dev->tx_reqs_active) );
    WARN_ON( !list_empty(&dev->rx_reqs_active) );

    while ( !list_empty(&dev->tx_reqs) ) 
    {
        req = container_of( dev->tx_reqs.next, 
                            struct usb_request,
                            list );
        if ( req != NULL )
        {
            list_del( &req->list );
            rkudev_req_free( dev->in_ep, req );
        }
        else
        {
            break;
        }
    }

    if ( dev->current_rx_req != NULL )
    {
        rkudev_req_free( dev->out_ep, dev->current_rx_req );
    }

    while ( !list_empty(&dev->rx_reqs) ) 
    {
        req = container_of( dev->rx_reqs.next,
                            struct usb_request, list );
        if ( req != NULL )
        {
            list_del(&req->list);
            rkudev_req_free(dev->out_ep, req);
        }
        else
        {
            break;
        }
    }

    while ( !list_empty(&dev->rx_buffers) ) 
    {
        req = container_of( dev->rx_buffers.next,
                            struct usb_request, list );
        if ( req != NULL )
        {
            list_del(&req->list);
            rkudev_req_free(dev->out_ep, req);
        }
        else
        {
            break;
        }
    }
    
    usb_free_all_descriptors(f);
}

static struct usb_function_instance* gdev_alloc_inst(void)
{
    struct f_rkudev_opts* opts = NULL;
    struct usb_function_instance* ret = NULL;
    int status = 0;

    if ( opt_debug_lvl > 0 )
    {
        pr_info( "%s: %s\n", RKUDEV_NAME_S, __func__ );
    }

    opts = kzalloc(sizeof(*opts), GFP_KERNEL);
    
    if ( opts == NULL )
        return ERR_PTR(-ENOMEM);

    mutex_init( &opts->lock );
    opts->func_inst.free_func_inst = gdev_free_inst;
    ret = &opts->func_inst;

    mutex_lock(&rkudev_ida_lock);

#ifndef android
    if ( ida_is_empty(&rkudev_ida) ) 
#else
    if ( idr_is_empty(&rkudev_ida.idr) ) 
#endif
    {
        status = gdev_setup( RKUDEV_MINORS );
        
        if (status) 
        {
            ret = ERR_PTR(status);
            kfree(opts);
            goto unlock;
        }
    }

    opts->minor = gdev_get_minor();
    
    if ( opts->minor < 0 ) 
    {
        ret = ERR_PTR(opts->minor);
        kfree(opts);
        
#ifndef android
        if ( ida_is_empty(&rkudev_ida) )
#else
        if ( idr_is_empty(&rkudev_ida.idr) )
#endif
        {
            gdev_cleanup();
        }
        
        goto unlock;
    }
    
    config_group_init_type_name( &opts->func_inst.group, 
                                 "",
                                 &rkudev_func_type );

unlock:
    mutex_unlock( &rkudev_ida_lock );
    
    return ret;
}

static struct usb_function* gdev_alloc( struct usb_function_instance* fi )
{
    struct rkudev_t*    dev = NULL;
    struct f_rkudev_opts* opts = NULL;

    if ( opt_debug_lvl > 0 )
    {
        pr_info( "%s: %s\n", RKUDEV_NAME_S, __func__ );
    }

    opts = container_of(fi, struct f_rkudev_opts, func_inst);
    
    if ( opts == NULL )
    {
        DBG( dev, "gdev_alloc(): container of instance NULL\n" );
        return ERR_PTR(-ENOMEM);
    }

    mutex_lock( &opts->lock );
    
    if ( opts->minor >= minors ) 
    {
        DBG( dev, "gdev_alloc(): opts->minor >= minors -> %d >= %d\n",
             opts->minor, minors );

        mutex_unlock( &opts->lock );
        return ERR_PTR(-ENOENT);
    }

    dev = kzalloc( sizeof(*dev), GFP_KERNEL );
    
    if ( dev == NULL )
    {
        if ( opt_debug_lvl > 0 )
        {
            printk( "%s: *WARNING: dev == NULL, usb device allocating failed\n",
                    RKUDEV_NAME_S );
        }
        mutex_unlock( &opts->lock );
        return ERR_PTR(-ENOMEM);
    }

    opts->refcnt++;
    dev->minor = opts->minor;
    dev->pnp_string = opts->pnp_string;
    dev->q_len = opts->q_len;
    mutex_unlock(&opts->lock);

    dev->function.name      = RKUDEV_NAME_S;
    dev->function.bind      = rkudev_func_bind;
    dev->function.setup     = rkudev_func_setup;
    dev->function.unbind    = rkudev_func_unbind;
    dev->function.set_alt   = rkudev_func_set_alt;
    dev->function.get_alt   = rkudev_func_get_alt;
    dev->function.disable   = rkudev_func_disable;
    dev->function.free_func = gdev_free;

    INIT_LIST_HEAD(&dev->tx_reqs);
    if ( opt_debug_lvl > 1 )
    {
        printk( "%s: dev->tx_reqs initialize!!\n", RKUDEV_NAME_S );
    }

    INIT_LIST_HEAD(&dev->rx_reqs);
    if ( opt_debug_lvl > 1 )
    {        
        printk( "%s: dev->rx_reqs initialize!!\n", RKUDEV_NAME_S );
    }

    INIT_LIST_HEAD(&dev->rx_buffers);
    INIT_LIST_HEAD(&dev->tx_reqs_active);
    INIT_LIST_HEAD(&dev->rx_reqs_active);

    spin_lock_init(&dev->lock);
    spin_lock_init(&dev->Txlock);
    spin_lock_init(&dev->Rxlock);

    init_waitqueue_head(&dev->rx_wait);
    init_waitqueue_head(&dev->tx_wait);
    init_waitqueue_head(&dev->tx_flush_wait);

    dev->interface = -1;
    dev->rkudev_cdev_open = 0;
    dev->current_rx_req = NULL;
    dev->current_rx_bytes = 0;
    dev->current_rx_buf = NULL;
    
    return &dev->function;
}

DECLARE_USB_FUNCTION_INIT( rkudev, gdev_alloc_inst, gdev_alloc );

static int gdev_setup( int count )
{
    int status;
    dev_t devt;

    if ( opt_debug_lvl > 0 )
    {
        pr_info( "%s: %s\n", RKUDEV_NAME_S, __func__ );
    }

    usb_gadget_class = class_create( THIS_MODULE, RKUDEV_G_S );
    
    if ( IS_ERR(usb_gadget_class) ) 
    {
        status = PTR_ERR(usb_gadget_class);
        usb_gadget_class = NULL;
        pr_err( "unable to create usb_gadget class %d\n", status );
        
        return status;
    }

    status = alloc_chrdev_region( &devt, 0, count, RKUDEV_G_S_R );
    
    if ( status != 0 )
    {
        pr_err("alloc_chrdev_region %d\n", status);
        class_destroy( usb_gadget_class );
        usb_gadget_class = NULL;

        return status;
    }

    major = MAJOR(devt);
    minors = count;

    return status;
}

static void gdev_cleanup(void)
{
    if ( opt_debug_lvl > 0 )
    {
        pr_info( "%s: %s\n", RKUDEV_NAME_S, __func__ );
    }

    if ( major > 0 )
    {
        unregister_chrdev_region(MKDEV(major, 0), minors);
        major = minors = 0;
    }
    
    class_destroy(usb_gadget_class);
    usb_gadget_class = NULL;
}

////////////////////////////////////////////////////////////////////////////////

MODULE_AUTHOR( "Raphael Kim(rageworx@gmail.com)" );
MODULE_VERSION( RKUDEV_VERSION_S );
MODULE_DESCRIPTION( "RaphKay's USB perpheral driver" );
MODULE_LICENSE( "GPL v2" );
