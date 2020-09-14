#include <linux/types.h>
#include <asm-generic/ioctl.h>         //_IOW(), _IOR()
#ifdef __KERNEL__
#ifdef CONFIG_COMPAT
#include <linux/compat.h> // compat_uptr_t
#endif
#endif
#include <uapi/miniISP/miniISP_ioctl.h>


struct miniISP_cmd_config32{
	uint16_t opcode;
	uint32_t size;
	uint32_t param;
} __attribute__ ((packed));


#define BASE_MINIISP_CONTROL 100

#define IOC_ISP_CTRL_FLOW_MAGIC 'C'

#define IOCTL_ISP_LOAD_FW32 \
	_IOWR(IOC_ISP_CTRL_FLOW_MAGIC, BASE_MINIISP_CONTROL, struct miniISP_cmd_config32)

#define IOCTL_ISP_PURE_BYPASS32 \
	_IOWR(IOC_ISP_CTRL_FLOW_MAGIC, BASE_MINIISP_CONTROL + 1, struct miniISP_cmd_config32)

#define IOCTL_ISP_POWER_OFF32 \
	_IOWR(IOC_ISP_CTRL_FLOW_MAGIC, BASE_MINIISP_CONTROL + 2, struct miniISP_cmd_config32)



