#ifndef __RK_USBBLK_H__
#define __RK_USBBLK_H__

#include <linux/usb/composite.h>

struct f_rkudev_opts 
{
    struct \
    usb_function_instance    func_inst;
    int                      minor;
    char*                    pnp_string;
    bool                     pnp_string_allocated;
    unsigned                 q_len;
    struct \
    mutex                    lock;
    int                      refcnt;
};

#endif /* __RK_USBBLK_H__ */
