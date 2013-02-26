/*
 * Custom NMI handler for Google Fiber Bruno boxes usually triggered by
 * hardware watchdog
 *
 * - print registers
 * - panic
 */
#include <linux/init.h>
#include <asm/brcmstb/brcmapi.h>

extern void show_registers(struct pt_regs *regs);

static NORET_TYPE void ATTRIB_NORET bruno_nmi_exception_handler(struct pt_regs *regs)
{
	unsigned int cpu = smp_processor_id();
	bust_spinlocks(1);
	console_verbose();
	printk(KERN_EMERG "NMI taken on CPU %d. Probably hardware watchdog timeout.\n",cpu);
	show_registers(regs);
	panic("NMI");
}

static int __init register_handler(void) {
	brcm_set_nmi_handler(bruno_nmi_exception_handler);
	return 0;
}

__initcall(register_handler);
