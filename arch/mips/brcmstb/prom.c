/*
 * Copyright (C) 2009 Broadcom Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include <linux/ctype.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/root_dev.h>
#include <linux/types.h>
#include <linux/smp.h>
#include <linux/bmoca-compat.h>
#include <linux/version.h>
#include <linux/serial_reg.h>
#include <linux/io.h>
#include <linux/string.h>
#include <linux/compiler.h>
#include <linux/mtd/mtd.h>

#include <asm/bootinfo.h>
#include <asm/r4kcache.h>
#include <asm/traps.h>
#include <asm/cacheflush.h>
#include <asm/mipsregs.h>
#include <asm/hazards.h>
#include <asm/brcmstb/brcmstb.h>
#include <asm/fw/cfe/cfe_api.h>
#include <asm/fw/cfe/cfe_error.h>

#include <spaces.h>

int cfe_splashmem_present = 0;
int cfe_lock_console_in = 1;
unsigned long brcm_dram0_size_mb;
unsigned long brcm_dram1_size_mb;
unsigned long brcm_dram1_linux_mb;
unsigned long brcm_dram1_start = MEMC1_START;
unsigned long brcm_min_auth_region_size = 0x1000;

static u8 brcm_eth0_macaddr[6] = { 0x00, 0x00, 0xde, 0xad, 0xbe, 0xef };
static u8 brcm_moca0_macaddr[6] = { 0x00, 0x00, 0xde, 0xad, 0xbe, 0xf0 };

unsigned long brcm_base_baud0 = BRCM_BASE_BAUD_STB;	/* UPG UARTA */
unsigned long brcm_base_baud = BRCM_BASE_BAUD_STB;	/* UPG_UART[BC] */

static void __cpuinit set_board_nmi_handler(void);

unsigned long __initdata cfe_seal;
unsigned long __initdata cfe_entry;
unsigned long __initdata cfe_handle;

#ifdef CONFIG_CMDLINE_BOOL
static char __initdata builtin_cmdline[COMMAND_LINE_SIZE] = CONFIG_CMDLINE;
#endif

static void macaddr_increment(u8 *buf, u8 len, u8 incr)
{
	u8 old;

	if (incr == 0)
		return;

	old = buf[len - 1];
	buf[len - 1] += incr;

	if (buf[len - 1] < old) {
		buf[len - 2] += 1;
		if (buf[len - 2] == 0) {
			buf[len - 3] += 1;
		}
	}
}

/***********************************************************************
 * CFE bootloader queries
 ***********************************************************************/

static int __init hex(char ch)
{
	if (ch >= 'a' && ch <= 'f')
		return ch-'a'+10;
	if (ch >= '0' && ch <= '9')
		return ch-'0';
	if (ch >= 'A' && ch <= 'F')
		return ch-'A'+10;
	return -1;
}

static int __init hex16(const char *b)
{
	int d0, d1;

	d0 = hex(b[0]);
	d1 = hex(b[1]);
	if ((d0 == -1) || (d1 == -1))
		return -1;
	return (d0 << 4) | d1;
}

void __init cfe_die(char *fmt, ...)
{
	char msg[128];
	va_list ap;
	int handle;
	unsigned int count;

	va_start(ap, fmt);
	vsprintf(msg, fmt, ap);
	strcat(msg, "\r\n");

	if (cfe_seal != CFE_EPTSEAL)
		goto no_cfe;

	/* disable XKS01 so that CFE can access the registers */

#if defined(CONFIG_BMIPS4380)
	__write_32bit_c0_register($22, 3,
		__read_32bit_c0_register($22, 3) & ~BIT(12));
#elif defined(CONFIG_BMIPS5000)
	__write_32bit_c0_register($22, 5,
		__read_32bit_c0_register($22, 5) & ~BIT(8));
#endif

	handle = cfe_getstdhandle(CFE_STDHANDLE_CONSOLE);
	if (handle < 0)
		goto no_cfe;

	cfe_write(handle, msg, strlen(msg));

	for (count = 0; count < 0x7fffffff; count++)
		mb();
	cfe_exit(0, 1);
	while (1)
		;

no_cfe:
	/* probably won't print anywhere useful */
	printk(KERN_ERR "%s", msg);
	BUG();

	va_end(ap);
}

static inline int __init parse_eth0_hwaddr(const char *buf, u8 *out)
{
	int i, t;
	u8 addr[6];

	for (i = 0; i < 6; i++) {
		t = hex16(buf);
		if (t == -1)
			return -1;
		addr[i] = t;
		buf += 3;
	}
	memcpy(out, addr, 6);

	return 0;
}

static inline int __init parse_ulong(const char *buf, unsigned long *val)
{
	char *endp;
	unsigned long tmp;

	tmp = simple_strtoul(buf, &endp, 0);
	if (*endp == 0) {
		*val = tmp;
		return 0;
	}
	return -1;
}

static inline int __init parse_hex(const char *buf, unsigned long *val)
{
	char *endp;
	unsigned long tmp;

	tmp = simple_strtoul(buf, &endp, 16);
	if (*endp == 0) {
		*val = tmp;
		return 0;
	}
	return -1;
}

static inline int __init parse_boardname(const char *buf, void *slop)
{
	int __maybe_unused len;

#if defined(CONFIG_BCM7405)
	/* autodetect 97459 DOCSIS boards */
	if (strstarts("BCM9745", buf))
		brcm_docsis_platform = 1;
#endif

#if defined(CONFIG_BCM7420)
	len = strlen(buf);
	if (len > 2) {
		if (buf[len - 2] == 'D' && buf[len - 1] == 'B') {
			/* BCM97420xxDB = satellite board with MidRF MoCA */
			brcm_moca_rf_band = MOCA_BAND_MIDRF;
		} else {
			/* BCM97420xx = cable/DOCSIS board with HighRF MoCA */
			brcm_docsis_platform = 1;
		}
	}
	if (strstarts(buf, "BCM93380SMS") ||
	    strstarts(buf, "BCM93380VMS") ||
	    strstarts(buf, "BCM97420_MOCA_GN")) {
		brcm_enet0_force_ext_mii = 1;
		brcm_enet_no_mdio = 1;
	}
	if (strstarts(buf, "BCM97420_MOCA_GN_SAT"))
		brcm_moca_rf_band = MOCA_BAND_MIDRF;
#elif defined(CONFIG_BCM7425)
	if (strstarts(buf, "BCM97425VMS")) {
		brcm_enet0_force_ext_mii = 1;
		brcm_enet_no_mdio = 1;
	}
#elif defined(CONFIG_BCM7344)
	/* 7344 is normally MidRF, but the 7418 variant might not be */
	if (strstarts(buf, "BCM97418SAT") || strstarts(buf, "BCM97418SFF_RVU"))
		brcm_moca_rf_band = MOCA_BAND_MIDRF;
	else if (strstarts(buf, "BCM97418"))
		brcm_moca_rf_band = MOCA_BAND_HIGHRF;
#elif defined(CONFIG_BCM7408)
	if (strstarts(buf, "BCM97408SAT"))
		brcm_moca_rf_band = MOCA_BAND_MIDRF;
#endif

#if defined(CONFIG_BCM7405)
	/* autodetect 97405-MSG board (special MII configuration) */
	if (strstr(buf, "_MSG") != NULL)
		brcm_enet_no_mdio = 1;
#endif

	strcpy(brcm_cfe_boardname, buf);
	return 0;
}

static inline int __init parse_cmdline(const char *buf, char *dst)
{
	strlcpy(dst, buf, COMMAND_LINE_SIZE);
	return 0;
}

static inline int __init parse_string(const char *buf, char *dst)
{
	strlcpy(dst, buf, CFE_STRING_SIZE);
	return 0;
}

static char __initdata cfe_buf[COMMAND_LINE_SIZE];

static void __init __maybe_unused cfe_read_configuration(void)
{
	int fetched = 0;

	printk(KERN_INFO "Fetching vars from bootloader... ");
	if (cfe_seal != CFE_EPTSEAL) {
		printk(KERN_CONT "none present, using defaults.\n");
		return;
	}

#define DPRINTK(...) do { } while (0)
/* #define DPRINTK(...) printk(__VA_ARGS__) */

#define FETCH(name, fn, arg) do { \
	if (cfe_getenv(name, cfe_buf, COMMAND_LINE_SIZE) == CFE_OK) { \
		DPRINTK("Fetch var '%s' = '%s'\n", name, cfe_buf); \
		fn(cfe_buf, arg); \
		fetched++; \
	} else { \
		DPRINTK("Could not fetch var '%s'\n", name); \
	} \
	} while (0)

	FETCH("ETH0_HWADDR", parse_eth0_hwaddr, brcm_eth0_macaddr);
	memcpy(brcm_moca0_macaddr, brcm_eth0_macaddr, ETH_ALEN);
	macaddr_increment(brcm_moca0_macaddr, ETH_ALEN, 1);
	FETCH("MOCA0_HWADDR", parse_eth0_hwaddr, brcm_moca0_macaddr);
	FETCH("DRAM0_SIZE", parse_ulong, &brcm_dram0_size_mb);
	FETCH("DRAM1_SIZE", parse_ulong, &brcm_dram1_size_mb);
	FETCH("CFE_BOARDNAME", parse_boardname, NULL);
	FETCH("BOOT_FLAGS", parse_cmdline, arcs_cmdline);

	FETCH("LINUX_FFS_STARTAD", parse_hex, &brcm_mtd_rootfs_start);
	FETCH("LINUX_FFS_SIZE", parse_hex, &brcm_mtd_rootfs_len);
	FETCH("LINUX_PART_STARTAD", parse_hex, &brcm_mtd_kernel_start);
	FETCH("LINUX_PART_SIZE", parse_hex, &brcm_mtd_kernel_len);
	FETCH("OCAP_PART_STARTAD", parse_hex, &brcm_mtd_ocap_start);
	FETCH("OCAP_PART_SIZE", parse_hex, &brcm_mtd_ocap_len);
	FETCH("FLASH_SIZE", parse_ulong, &brcm_mtd_flash_size_mb);
	FETCH("FLASH_TYPE", parse_string, brcm_mtd_flash_type);

	printk(KERN_CONT "found %d vars.\n", fetched);
}

/***********************************************************************
 * Early printk
 ***********************************************************************/

static unsigned long brcm_early_uart;

#define UART_REG(x)		(brcm_early_uart + ((x) << 2))
#define BAUD			115200

static void __init init_port(void)
{
	unsigned int divisor;

	BDEV_WR(UART_REG(UART_LCR), 0x3);	/* 8n1 */
	BDEV_WR(UART_REG(UART_IER), 0);		/* no interrupt */
	BDEV_WR(UART_REG(UART_FCR), 0);		/* no fifo */
	BDEV_WR(UART_REG(UART_MCR), 0x3);	/* DTR + RTS */

	BDEV_SET(UART_REG(UART_LCR), UART_LCR_DLAB);
#if defined(CONFIG_BRCM_IKOS)
	/* Reverse-engineer brcm_base_baud0 from the bootloader's setting */

	divisor = (BDEV_RD(UART_REG(UART_DLM)) << 8) |
		BDEV_RD(UART_REG(UART_DLL));
	brcm_base_baud0 = divisor * BAUD;
#endif
	divisor = (brcm_base_baud0 + BAUD/2) / BAUD;
	BDEV_WR(UART_REG(UART_DLL), divisor & 0xff);
	BDEV_WR(UART_REG(UART_DLM), (divisor >> 8) & 0xff);
	BDEV_UNSET(UART_REG(UART_LCR), UART_LCR_DLAB);
}

void prom_putchar(char x)
{
	while (!(BDEV_RD(UART_REG(UART_LSR)) & UART_LSR_THRE))
		;
	BDEV_WR(UART_REG(UART_TX), x);
}

#ifdef CONFIG_BRUNO
int lock_console_in(void) {
	return cfe_lock_console_in;
}
#endif  /* CONFIG_BRUNO */

static void __init brcm_setup_early_printk(void)
{
	char *arg = strstr(arcs_cmdline, "console=");
	int dev = CONFIG_BRCM_CONSOLE_DEVICE;
	const unsigned long base[] = {
		BCHP_UARTA_REG_START, BCHP_UARTB_REG_START,
#ifdef CONFIG_BRCM_HAS_UARTC
		BCHP_UARTC_REG_START,
#endif
		0, 0,
	};

	if (strstr(arcs_cmdline, "login=") || strstr(arcs_cmdline, "debug=")) {
		cfe_lock_console_in = 0;
	}

	/*
	 * quick command line parse to pick the early printk console
	 * valid formats:
	 *   console=ttyS0,115200
	 *   console=0,115200
	 */
	while (arg && *arg != '\0' && *arg != ' ') {
		if ((*arg >= '0') && (*arg <= '3')) {
			dev = *arg - '0';
			if (base[dev] == 0)
				dev = 0;
			break;
		}
		arg++;
	}
	brcm_early_uart = base[dev];
	init_port();
}

#define SPLASH_TOKEN "splashmem="
int parse_splash_mem(const char *arcs_cmdline,
			    unsigned long *start,
			    unsigned long *size)
{
	char *ptr;
	char *end;
	char *endptr;	/* local pointer to end of parsed string */

	ptr = strstr(arcs_cmdline, SPLASH_TOKEN);
	if (!ptr) {
		printk(KERN_INFO "No splashmem present\n");
		return -1;
	}
	end = strstr(ptr, "@memc1");
	if (!end) {
		printk(KERN_WARNING "Invalid mem bank for splashmem\n");
		return -1;
	}

	ptr += sizeof(SPLASH_TOKEN);
	*start = (unsigned long)memparse(ptr, &endptr);

	ptr = strchr(endptr, '/');
	if (!ptr) {
		printk(KERN_WARNING "Invalid splashmem format\n");
		return -1;
	}
	++ptr;
	*size = (unsigned long)memparse(ptr, &endptr);
        cfe_splashmem_present = 1;
	return 0;
}

/***********************************************************************
 * Main entry point
 ***********************************************************************/

void __init prom_init(void)
{
	char *ptr;

	cfe_init(cfe_handle, cfe_entry);

	bchip_check_compat();
	board_pinmux_setup();

	bchip_mips_setup();
	set_board_nmi_handler();

	/* default to SATA (where available) or MTD rootfs */
#ifdef CONFIG_BRCM_HAS_SATA
	ROOT_DEV = Root_SDA1;
#else
	ROOT_DEV = MKDEV(MTD_BLOCK_MAJOR, 0);
#endif
	root_mountflags &= ~MS_RDONLY;

	bchip_set_features();

#if defined(CONFIG_BRCM_IKOS_DEBUG)
	strcpy(arcs_cmdline, "debug initcall_debug");
#elif !defined(CONFIG_BRCM_IKOS)
	cfe_read_configuration();
#endif
	brcm_setup_early_printk();

#ifdef CONFIG_CMDLINE_BOOL
#ifdef CONFIG_CMDLINE_OVERRIDE
	strlcpy(arcs_cmdline, builtin_cmdline, COMMAND_LINE_SIZE);
#else
	if (builtin_cmdline[0]) {
		strlcat(arcs_cmdline, " ", COMMAND_LINE_SIZE);
		strlcat(arcs_cmdline, builtin_cmdline, COMMAND_LINE_SIZE);
	}
#endif
#endif
	/* provide "ubiroot" alias to reduce typing */
	if (strstr(arcs_cmdline, "ubiroot"))
		strcat(arcs_cmdline, " ubi.mtd=rootfs rootfstype=ubifs "
			"root=ubi0:rootfs");

	ptr = strstr(arcs_cmdline, "memc1=");
	if (ptr)
		brcm_dram1_linux_mb = memparse(ptr + 6, &ptr) >> 20;

	printk(KERN_INFO "Options: enet_en=%d enet0_mii=%d enet_no_mdio=%d "
		"enet1_en=%d moca=%d\n",
		brcm_enet_enabled, brcm_enet0_force_ext_mii,
		brcm_enet_no_mdio, brcm_enet1_enabled, brcm_moca_enabled);
	printk(KERN_INFO "         sata=%d docsis=%d pci=%d pcie=%d smp=%d "
		"usb=%d\n",
		brcm_sata_enabled, brcm_docsis_platform, brcm_pci_enabled,
		brcm_pcie_enabled, brcm_smp_enabled, brcm_usb_enabled);

	bchip_early_setup();

	board_get_ram_size(&brcm_dram0_size_mb, &brcm_dram1_size_mb);

	do {
		unsigned long dram0_mb = brcm_dram0_size_mb, mb;

		mb = min(dram0_mb, BRCM_MAX_LOWER_MB);
		dram0_mb -= mb;

		add_memory_region(0, mb << 20, BOOT_MEM_RAM);
		if (!dram0_mb)
			break;

#ifdef CONFIG_BRCM_UPPER_MEMORY
		mb = min(dram0_mb, BRCM_MAX_UPPER_MB);
		dram0_mb -= mb;

		brcm_upper_tlb_setup();
		add_memory_region(UPPERMEM_START, mb << 20, BOOT_MEM_RAM);
		if (!dram0_mb)
			break;
#endif

#if defined(CONFIG_HIGHMEM)
		add_memory_region(HIGHMEM_START, dram0_mb << 20, BOOT_MEM_RAM);
		break;
#endif
		/*
		 * We wound up here because the chip's architecture cannot
		 * make use of all MEMC0 RAM in Linux.  i.e. no suitable
		 * HIGHMEM or upper memory options are supported by the CPU.
		 *
		 * But we can still report the excess memory as a "bonus"
		 * reserved (bmem) region, so the application can manage it.
		 */
		mb = brcm_dram0_size_mb - dram0_mb;	/* Linux memory */
		if (!brcm_dram1_size_mb && mb == 256) {
			printk(KERN_INFO "MEMC0 split: %lu MB -> Linux; "
				"%lu MB -> extra bmem\n", mb, dram0_mb);
			brcm_dram1_size_mb = dram0_mb;
			brcm_dram1_start = UPPERMEM_START;
		}
	} while (0);

#if defined(CONFIG_HIGHMEM) && defined(CONFIG_BRCM_HAS_1GB_MEMC1)
	if (brcm_dram1_linux_mb > brcm_dram1_size_mb) {
		printk(KERN_WARNING "warning: 'memc1=%luM' exceeds "
			"available memory (%lu MB); ignoring\n",
			brcm_dram1_linux_mb, brcm_dram1_size_mb);
		brcm_dram1_linux_mb = 0;
	} else if (brcm_dram1_linux_mb) {
		/* Since the bootloader can only map the first 256M of memc1
		 * when it boots, if we get memc1= request from bootloader, we
		 * should try to pull the memory from the end to avoid crossing
		 * over the memory that is allocated for boot logo image by
		 * bootloader.
		 */
		unsigned long start_mb, start_b, size, splash_bound = 0;
		if (0 == parse_splash_mem(arcs_cmdline, &splash_bound, &size)) {
			splash_bound += size;
		}

		start_mb = brcm_dram1_size_mb - brcm_dram1_linux_mb;
		start_b  = start_mb << 20;
		if (splash_bound > start_b) {
			unsigned long orig_dram1 = brcm_dram1_linux_mb;
			start_mb = (splash_bound + 0x000FFFFF) >> 20;
			start_b = start_mb << 20;
			brcm_dram1_linux_mb = brcm_dram1_size_mb - start_mb;
			printk(KERN_WARNING "warning: 'memc1=%luM' starts "
			       " before splash memory bound (0x%lx);"
			       " adjusting to (memc1=%luM)\n",
			       orig_dram1, splash_bound, brcm_dram1_linux_mb);
		}
		printk(KERN_INFO "memc1: adding %luMB at %luMB "
		       "(0x%08lx@0x%08lx)",
		       brcm_dram1_linux_mb, (MEMC1_START >> 20) + start_mb,
		       brcm_dram1_linux_mb << 20, MEMC1_START + start_b);
		add_memory_region(MEMC1_START + start_b,
				  brcm_dram1_linux_mb << 20,
				  BOOT_MEM_RAM);
	}
#else
	if (brcm_dram1_linux_mb) {
		printk(KERN_WARNING "warning: MEMC1 is not available on this "
			"system; ignoring\n");
		brcm_dram1_linux_mb = 0;
	}
#endif

#ifdef CONFIG_SMP
	register_smp_ops(&brcmstb_smp_ops);
#endif
}

/***********************************************************************
 * Vector relocation for NMI and SMP TP1 boot
 ***********************************************************************/

extern void nmi_exception_handler(struct pt_regs *regs);
void (*brcm_nmi_handler)(struct pt_regs *) = &nmi_exception_handler;

void brcm_set_nmi_handler(void (*fn)(struct pt_regs *))
{
	brcm_nmi_handler = fn;
}
EXPORT_SYMBOL(brcm_set_nmi_handler);

void __cpuinit brcm_wr_vec(unsigned long dst, char *start, char *end)
{
	memcpy((void *)dst, start, end - start);
	dma_cache_wback((unsigned long)start, end - start);
	local_flush_icache_range(dst, dst + (end - start));
	instruction_hazard();
}

static inline void __cpuinit brcm_nmi_handler_setup(void)
{
	brcm_wr_vec(BRCM_NMI_VEC, brcm_reset_nmi_vec, brcm_reset_nmi_vec_end);
	brcm_wr_vec(BRCM_WARM_RESTART_VEC, brcm_tp1_int_vec,
		brcm_tp1_int_vec_end);
	brcm_wr_vec(BRCM_EJTAG_DEBUG_VEC, except_vec_ejtag_debug,
		except_vec_ejtag_debug_end);
}

static void __cpuinit set_board_nmi_handler(void)
{
#if defined(CONFIG_BCM7468) || defined(CONFIG_BCM7550)
	/*
	 * Older BMIPS3300 CPUs do not support
	 * exception vector relocation.
	 * Do not setup an nmi hander as it will
	 * overwrite the tlb handler.
	 */
#else
	board_nmi_handler_setup = &brcm_nmi_handler_setup;
#endif
}

unsigned long brcm_setup_ebase(void)
{
	unsigned long ebase = CAC_BASE;
#if defined(CONFIG_BCM7468) || defined(CONFIG_BCM7550)
	/*
	 * Older BMIPS3300 CPUs do not support
	 * exception vector relocation
	 */
#elif defined(CONFIG_BMIPS3300) || defined(CONFIG_BMIPS4380)
	/*
	 * Exception vector configuration on BMIPS4380:
	 *
	 * 8000_0000 - new reset/NMI vector            (was: bfc0_0000)
	 * 8000_0400 - new !BEV exception base         (was: 8000_0000)
	 *
	 * The reset/NMI vector can only be adjusted in 1MB increments, so
	 * we put it at 8000_0000 and then move the runtime exception vectors
	 * up a little bit.
	 *
	 * The initial reset/NMI vector for TP1 is at a000_0000 because the
	 * BMIPS4380 I$ comes up in an undefined state, but it is almost
	 * immediately moved down to kseg0.
	 *
	 * This is nearly identical on BMIPS3300, except for the fact that
	 * BMIPS3300 only has a single thread.
	 */

	unsigned long cbr = BMIPS_GET_CBR();
	DEV_WR_RB(cbr + BMIPS_RELO_VECTOR_CONTROL_0, 0x80080800);
#ifdef CONFIG_BMIPS4380
	DEV_WR_RB(cbr + BMIPS_RELO_VECTOR_CONTROL_1, 0xa0080800);
#endif
	ebase = 0x80000400;

#elif defined(CONFIG_BMIPS5000)
	/*
	 * BMIPS5000 is similar to BMIPS4380, but it uses different
	 * configuration registers with different semantics:
	 *
	 * 8000_0000 - new reset/NMI vector            (was: bfc0_0000)
	 * 8000_1000 - new !BEV exception base         (was: 8000_0000)
	 *
	 * The initial reset/NMI vector for TP1 is at a000_0000 because
	 * CP0 CONFIG comes up in an undefined state, but it is almost
	 * immediately moved down to kseg0.
	 */
	ebase = 0x80001000;

	write_c0_brcm_bootvec(0xa0088008);
	write_c0_ebase(ebase);

#endif
	return ebase;
}

/***********************************************************************
 * Miscellaneous utility functions
 ***********************************************************************/

static char brcm_system_type[64];

const char *get_system_type(void)
{
	u32 class = BRCM_CHIP_ID();

	if (class >> 16 == 0)
		class >>= 8;
	else
		class >>= 12;

	snprintf(brcm_system_type, 64, "BCM%04x%02X %s platform",
		BRCM_CHIP_ID(), BRCM_CHIP_REV() + 0xa0,
		class == 0x35 ? "DTV" :
		(class == 0x76 ? "DVD" : "STB"));

	return (const char *)brcm_system_type;
}

void __init prom_free_prom_memory(void) {}

int brcm_alloc_macaddr(u8 *buf, u8 intf_id, bool intf_is_moca)
{
	if (intf_is_moca) {
		/*
		 * Only one MoCA interface is supported.
		 * We can ignore intf_id offset.
		 */
		memcpy(buf, brcm_moca0_macaddr, ETH_ALEN);
	} else {
		memcpy(buf, brcm_eth0_macaddr, ETH_ALEN);
		macaddr_increment(buf, ETH_ALEN, intf_id);
	}

	return 0;
}
EXPORT_SYMBOL(brcm_alloc_macaddr);
