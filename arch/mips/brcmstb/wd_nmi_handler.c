/*
 * Custom NMI handler for Google Fiber Bruno boxes usually triggered by
 * hardware watchdog
 *
 * - print registers
 * - set mode register to also route this NMI to Thread 1
 * - wait for a short while
 * - restart
 */
#include <linux/init.h>
#include <linux/reboot.h>
#include <asm/brcmstb/brcmapi.h>

extern void show_registers(struct pt_regs *regs);

static int timeout = 1;

static NORET_TYPE void ATTRIB_NORET bruno_nmi_exception_handler(struct pt_regs *regs)
{
	unsigned int cpu = smp_processor_id();
	bust_spinlocks(1);
	console_verbose();
	printk(KERN_EMERG "NMI taken on CPU %d. Probably hardware watchdog timeout.\n",cpu);
	show_registers(regs);
	/* Broadcom Mode Register (CP0 Register 22, Select 1)
	 * 0x1: NMI is routed to Thread 0.
	 * 0x2: NMI is routed to Thread 1.
	 * 0x3: NMI is routed to both threads.
	 * Change NMI exception routing to route NMI to both threads. Since the
	 * NMI is already asserted, this will cause an NMI exception on Thread
	 * 1 immediately.
	 * */
	write_c0_brcm_mode(read_c0_brcm_mode() | 0x3);
	printk(KERN_EMERG "Rebooting in %d seconds..", timeout);
	while (1) {
		mdelay(timeout*1000);
		machine_restart(NULL);
	}
}

static int __init register_handler(void) {
	brcm_set_nmi_handler(bruno_nmi_exception_handler);
	return 0;
}

__initcall(register_handler);
