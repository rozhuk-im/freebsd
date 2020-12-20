/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2008, 2009 Rui Paulo <rpaulo@FreeBSD.org>
 * Copyright (c) 2009 Norikatsu Shigemura <nork@FreeBSD.org>
 * Copyright (c) 2009-2012 Jung-uk Kim <jkim@FreeBSD.org>
 * Copyright (c) 2013-2020 Rozhuk Ivan <rozhuk.im@gmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Driver for the AMD CPU on-die thermal sensors.
 * Initially based on the k8temp Linux driver.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/bus.h>
#include <sys/resource.h>
#include <sys/rman.h>
#include <sys/sysctl.h>
#include <sys/lock.h>
#include <sys/mutex.h>

#include <machine/bus.h>
#include <machine/resource.h>
#include <machine/md_var.h>
#include <machine/specialreg.h>
#include <machine/cputypes.h>
#include <machine/pci_cfgreg.h>

#include <dev/pci/pcivar.h>
#include <dev/pci/pcireg.h>


typedef struct pci_io_data_s {
	uint32_t		bus; /* 0xffffffff = amdtemp bus. */
	uint32_t		slot;
	uint32_t		func;
	uint32_t		addr; /* Addr to set addr for read/write data. */
	uint32_t		data; /* Addr for read/write data */
	uint32_t		addr_offset; /* Register offset in this addr space. */
} pci_io_t, *pci_io_p;

typedef struct amdtemp_softc {
	device_t		dev;
	struct mtx		lock; /* Read/write lock for some registers. */
	uint32_t		cpu_ncores;
	uint32_t		flags;
	uint32_t		tts_flags; /* Thermaltrip Status flags. */
	int32_t			tts_temp_offset[4];
	int32_t			rtc_temp_offset;
	pci_io_t		pci_io;
	struct sysctl_oid	*sysctl_cpu[MAXCPU]; /* dev.cpu.X.temperature oids. */
	struct intr_config_hook	sc_ich;
} amdtemp_softc_t, *amdtemp_softc_p;
#define	AMDTEMP_F_RTC		1	/* Reported Temperature Control. */
#define	AMDTEMP_F_TTS		2	/* Thermaltrip Status. */
#define	AMDTEMP_F_HTC		4	/* Hardware Thermal Control (HTC). */

#define	AMDTEMP_TTS_F_CS_SWAP	0x01	/* ThermSenseCoreSel is inverted. */
#define	AMDTEMP_TTS_F_CT_10BIT	0x02	/* CurTmp is 10-bit wide. */
#define	AMDTEMP_TTS_F_OFF28	0x04	/* CurTmp starts at -28C. */
#define	AMDTEMP_TTS_F_NO_SENS	0x08	/* No sensors. */


/* D18F3xFC CPUID Family/Model/Stepping */
#define	AMD_REG_CPUID		0xfc
/* DRAM Configuration High Register */
#define	AMD_REG_DRAM_CONF_HIGH	0x94	/* Function 2 */
#define	AMD_REG_DRAM_MODE_DDR3	0x0100

/* D18F3xA4 Reported Temperature Control Register */
#define	AMD_REG_REPTMP_CTRL	0xa4
union reg_amd_rep_tmp_ctrl_desc {
	uint32_t u32;
	struct reg_amd_rep_tmp_ctrl_bits {
		uint32_t PerStepTimeUp:5; /* 4:0 rw per 1/8th step time up. */
		uint32_t TmpMaxDiffUp:2;/* 6:5 rw temperature maximum difference up. */
		uint32_t TmpSlewDnEn:1;	/* 7 rw temperature slew downward enable. */
		uint32_t PerStepTimeDn:5;/* 12:8 rw per 1/8th step time down. */
		uint32_t r0:3;		/* 15:13 Reserved. */
		uint32_t CurTmpTjSel:2;	/* 17:16 rw Current temperature select. */
		uint32_t CurTmpTjSlewSel:1;/* 18 rw  */
		uint32_t CurTmpRangeSel:1;/* 19 rw  */
		uint32_t r1:1;		/* 20 Reserved. */
		uint32_t CurTmp:11;	/* 31:21 ro/rw current temperature. */
	} bits;
};
CTASSERT(sizeof(struct reg_amd_rep_tmp_ctrl_bits) == sizeof(uint32_t));
/* CurTmpTjSel valid family 10h, 15h, 16h processors. */

/* 
 * Thermaltrip Status Register
 * BIOS and Kernel Developer’s Guide for AMD NPT Family 0Fh Processors
 * 32559 Rev. 3.16 November 2009
 */
/* D18F3xE4 Thermtrip Status Register */
#define	AMD_REG_THERMTRIP_STAT	0xe4
union reg_amd_thermtrip_status_desc {
	uint32_t u32;
	struct reg_amd_thermtrip_status_bits {
		uint32_t r0:1;		/* 0 Reserved. */
		uint32_t Thermtp:1;	/* 1 ro The processor has entered the THERMTRIP state. */
		uint32_t ThermSenseCoreSel:1; /* 2 rw  */
		uint32_t ThermtpSense0:1; /* 3 ro  */
		uint32_t ThermtpSense1:1; /* 4 ro  */
		uint32_t ThermtpEn:1;	/* 5 ro The THERMTRIP state is supported by the processor. */
		uint32_t ThermSenseSel:1; /* 6 rw  */
		uint32_t r1:1;		/* 7 Reserved. */
		uint32_t DiodeOffset:6;	/* 13:8 ro Thermal diode offset is used to correct the measurement made by an external temperature sensor. */
		uint32_t CurTmp:10;	/* 23:14 ro This field returns the current value of the internal thermal sensor. */
		uint32_t TjOffset:5;	/* 28:24 ro This field is the offset from CurTmp used to normalize to Tcontrol. */
		uint32_t r2:2;		/* 30:29 Reserved. */
		uint32_t SwThermtp:1;	/* 31 rw  */
	} bits;
};
CTASSERT(sizeof(struct reg_amd_thermtrip_status_bits) == sizeof(uint32_t));

/*
 * The default value of the HTC temperature threshold (Tctl_max) is specified
 * in the AMD Family 14h Processor Power and Thermal Datasheet.
 */
/* D18F3x64 Hardware Thermal Control (HTC) */
#define	AMD_REG_HTC_CTRL	0x64
union reg_amd_htc_desc {
	uint32_t u32;
	struct reg_amd_htc_bits {
		uint32_t HtcEn:1;	/* 0 rw 1=HTC is enabled; the processor is capable of entering the HTC-active state. */
		uint32_t r0:3;		/* 3:1 Reserved. */
		uint32_t HtcAct:1;	/* 4 ro 1=The processor is currently in the HTC-active state. */
		uint32_t HtcActSts:1;	/* 5 ro Read; set-by-hardware; write-1-to-clear. Reset: 0. This bit is set by hardware when the processor enters the HTC-active state. It is cleared by writing a 1 to it. */
		uint32_t PslApicHiEn:1;	/* 6 rw P-state limit higher value change APIC interrupt enable. */
		uint32_t PslApicLoEn:1;	/* 7 rw  P-state limit lower value change APIC interrupt enable. */
		uint32_t r1:8;		/* 15:8 Reserved. */
		uint32_t HtcTmpLmt:7;	/* 22:16 rw HTC temperature limit. */
		uint32_t HtcSlewSel:1;	/* 23 rw HTC slew-controlled temperature select. */
		uint32_t HtcHystLmt:4;	/* 27:24 rw HTC hysteresis. The processor exits the HTC active state when the temperature selected by HtcSlewSel is less than the HTC temperature limit (HtcTmpLmt) minus the HTC hysteresis (HtcHystLmt). */
		uint32_t HtcPstateLimit:3; /* 30:28 rw  HTC P-state limit select. */
		uint32_t HtcLock:1;	/* 31 Read; write-1-only. 1=HtcPstateLimit, HtcHystLmt, HtcTmpLmt, and HtcEn are read-only. */
	} bits;
};
CTASSERT(sizeof(struct reg_amd_htc_bits) == sizeof(uint32_t));

/* For F15h M60h. */
static pci_io_t pci_io_smu_f15 = {
	.bus = 0xffffffff,
	.slot = 0,
	.func = 0,
	.addr = 0xb8,
	.data = 0xbc,
	.addr_offset = 0xd8200c00
};

/* For F17h M00h. */
static pci_io_t pci_io_smn_f17 = {
	.bus = 0xffffffff,
	.slot = 0,
	.func = 0,
	.addr = 0x60,
	.data = 0x64,
	.addr_offset = (0x59800 - AMD_REG_REPTMP_CTRL)
};

#define	AMDTEMP_ZERO_C_TO_K	2731

#if __FreeBSD_version < 1200018 /* Since revision 310051. */
#define AMDTEMP_SYSCTL_ADD_PROC(__ctx, __parent, __nbr,			\
	    __name, __access, __ptr, __arg, __handler, __fmt, __descr)	\
		sysctl_add_oid((__ctx), (__parent), (__nbr), (__name),	\
		    (__access), (__ptr), (__arg), (__handler), (__fmt),	\
		    (__descr))
#else
#define AMDTEMP_SYSCTL_ADD_PROC(__ctx, __parent, __nbr, __name,		\
	    __access, __ptr, __arg, __handler, __fmt, __descr)		\
		sysctl_add_oid((__ctx), (__parent), (__nbr), (__name),	\
		    (__access),	(__ptr), (__arg), (__handler), (__fmt),	\
		        (__descr), NULL)
#endif
#define ARG2_GET_REG(__arg)	((__arg) & 0xffff)
#define ARG2_GET_A1(__arg)	(((__arg) >> 16) & 0xff)
#define ARG2_GET_A2(__arg)	(((__arg) >> 24) & 0xff)
#define MAKE_ARG2(__reg, __a1, __a2)					\
    (((__reg) & 0xff) | (((__a1) & 0xff) << 16) | (((__a2) & 0xff) << 24))

#define	AMDTEMP_LOCK(__sc)	mtx_lock(&(__sc)->lock)
#define	AMDTEMP_UNLOCK(__sc)	mtx_unlock(&(__sc)->lock)


typedef struct amdtemp_sysctl_reg_s {
	uint16_t	reg;
	uint8_t		a1;
	uint8_t		a2;
	uint32_t	flags;
	char		*fmt;
	int 		(*oid_handler)(SYSCTL_HANDLER_ARGS);
	char		*name;
	char		*descr;
} sysctl_reg_t, *sysctl_reg_p;

static void	amdtemp_sysctl_reg_add(amdtemp_softc_p sc,
		    struct sysctl_oid_list *child,
		    sysctl_reg_p regs, const size_t count);
static int	amdtemp_sysctl_reg_bits(SYSCTL_HANDLER_ARGS);

static uint32_t	amdtemp_tts_get_temp(amdtemp_softc_p sc,
		    uint32_t reg, uint8_t core, uint8_t sense);
static int	amdtemp_tts_temp_reg_sysctl(SYSCTL_HANDLER_ARGS);

static int	amdtemp_htc_temp_sysctl(SYSCTL_HANDLER_ARGS);

static int	amdtemp_rtc_temp_sysctl(SYSCTL_HANDLER_ARGS);

static uint32_t	amdtemp_pci_read(amdtemp_softc_p sc, uint32_t addr);
static void	amdtemp_pci_write(amdtemp_softc_p sc, uint32_t addr,
		    uint32_t data);


/* D18F3xE4 Thermtrip Status Register */
static sysctl_reg_t amdtemp_thermtrip_status_reg_bits[] = {
	{
		.reg = AMD_REG_THERMTRIP_STAT,
		.a1 = 24,
		.a2 = 5,
		.flags = (CTLFLAG_RD | CTLTYPE_UINT),
		.fmt = "IU",
		.oid_handler = amdtemp_sysctl_reg_bits,
		.name = "tj_offset",
		.descr = __DESCR("This field is the offset from "
		"CurTmp used to normalize to Tcontrol.")
	}, {
		.reg = AMD_REG_THERMTRIP_STAT,
		.a1 = 8,
		.a2 = 6,
		.flags = (CTLFLAG_RD | CTLTYPE_UINT),
		.fmt = "IU",
		.oid_handler = amdtemp_sysctl_reg_bits,
		.name = "diode_offset",
		.descr = __DESCR("Thermal diode offset is used to "
		"correct the measurement made by an external "
		"temperature sensor.")
	}, {
		.reg = AMD_REG_THERMTRIP_STAT,
		.a1 = 5,
		.a2 = 1,
		.flags = (CTLFLAG_RD | CTLTYPE_UINT),
		.fmt = "IU",
		.oid_handler = amdtemp_sysctl_reg_bits,
		.name = "enable",
		.descr = __DESCR("The THERMTRIP state is supported by "
		"the processor.")
	}, {
		.reg = AMD_REG_THERMTRIP_STAT,
		.a1 = 3,
		.a2 = 1,
		.flags = (CTLFLAG_RD | CTLTYPE_UINT),
		.fmt = "IU",
		.oid_handler = amdtemp_sysctl_reg_bits,
		.name = "sense",
		.descr = __DESCR("The processor temperature exceeded "
		"the THERMTRIP value.")
	}, {
		.reg = AMD_REG_THERMTRIP_STAT,
		.a1 = 1,
		.a2 = 1,
		.flags = (CTLFLAG_RD | CTLTYPE_UINT),
		.fmt = "IU",
		.oid_handler = amdtemp_sysctl_reg_bits,
		.name = "thermtrip",
		.descr = __DESCR("The processor has entered the "
		"THERMTRIP state.")
	},
};

/* D18F3x64 Hardware Thermal Control (HTC) */
static sysctl_reg_t amdtemp_htc_reg_bits[] = {
	{
		.reg = AMD_REG_HTC_CTRL,
		.a1 = 16,
		.a2 = 7,
		.flags = (CTLFLAG_RD | CTLTYPE_INT),
		.fmt = "IK",
		.oid_handler = amdtemp_htc_temp_sysctl,
		.name = "temperature_limit",
		.descr = __DESCR("HTC temperature limit")
	}, {
		.reg = AMD_REG_HTC_CTRL,
		.a1 = 24,
		.a2 = 4,
		.flags = (CTLFLAG_RW | CTLTYPE_INT),
		.fmt = "IK",
		.oid_handler = amdtemp_htc_temp_sysctl,
		.name = "hysteresis_limit",
		.descr = __DESCR("HTC hysteresis. The processor exits "
		"the HTC active state when the temperature selected by "
		"HtcSlewSel is less than the HTC temperature limit "
		"(HtcTmpLmt) minus the HTC hysteresis (HtcHystLmt).")
	}, {
		.reg = AMD_REG_HTC_CTRL,
		.a1 = 0,
		.a2 = 1,
		.flags = (CTLFLAG_RW | CTLTYPE_UINT),
		.fmt = "IU",
		.oid_handler = amdtemp_sysctl_reg_bits,
		.name = "enabled",
		.descr = __DESCR("HTC is enabled; the processor is "
		"capable of entering the HTC-active state.")
	}, {
		.reg = AMD_REG_HTC_CTRL,
		.a1 = 31,
		.a2 = 1,
		.flags = (CTLFLAG_RW | CTLTYPE_UINT),
		.fmt = "IU",
		.oid_handler = amdtemp_sysctl_reg_bits,
		.name = "lock",
		.descr = __DESCR("HtcPstateLimit, HtcHystLmt, "
		"HtcTmpLmt, and HtcEn are read-only.")
	}, {
		.reg = AMD_REG_HTC_CTRL,
		.a1 = 23,
		.a2 = 1,
		.flags = (CTLFLAG_RW | CTLTYPE_UINT),
		.fmt = "IU",
		.oid_handler = amdtemp_sysctl_reg_bits,
		.name = "slew_select",
		.descr = __DESCR("HTC slew-controlled temperature "
		"select.")
	}, {
		.reg = AMD_REG_HTC_CTRL,
		.a1 = 28,
		.a2 = 3,
		.flags = (CTLFLAG_RW | CTLTYPE_UINT),
		.fmt = "IU",
		.oid_handler = amdtemp_sysctl_reg_bits,
		.name = "p-state_limit",
		.descr = __DESCR("HTC P-state limit select.")
	}, {
		.reg = AMD_REG_HTC_CTRL,
		.a1 = 4,
		.a2 = 1,
		.flags = (CTLFLAG_RW | CTLTYPE_UINT),
		.fmt = "IU",
		.oid_handler = amdtemp_sysctl_reg_bits,
		.name = "active",
		.descr = __DESCR("The processor is currently in the "
		"HTC-active state.")
	}, {
		.reg = AMD_REG_HTC_CTRL,
		.a1 = 5,
		.a2 = 1,
		.flags = (CTLFLAG_RW | CTLTYPE_UINT),
		.fmt = "IU",
		.oid_handler = amdtemp_sysctl_reg_bits,
		.name = "active_sts",
		.descr = __DESCR("set-by-hardware; write-1-to-clear. "
		"Reset: 0. This bit is set by hardware when the "
		"processor enters the HTC-active state. It is cleared "
		"by writing a 1 to it.")
	}, {
		.reg = AMD_REG_HTC_CTRL,
		.a1 = 6,
		.a2 = 1,
		.flags = (CTLFLAG_RW | CTLTYPE_UINT),
		.fmt = "IU",
		.oid_handler = amdtemp_sysctl_reg_bits,
		.name = "p-state_apic_higher_enable",
		.descr = __DESCR("P-state limit higher value change "
		"APIC interrupt enable.")
	}, {
		.reg = AMD_REG_HTC_CTRL,
		.a1 = 7,
		.a2 = 1,
		.flags = (CTLFLAG_RW | CTLTYPE_UINT),
		.fmt = "IU",
		.oid_handler = amdtemp_sysctl_reg_bits,
		.name = "p-state_apic_lower_enable",
		.descr = __DESCR("P-state limit lower value change "
		"APIC interrupt enable.")
	}
};

/* D18F3xA4 Reported Temperature Control Register */
static sysctl_reg_t amdtemp_reptmp_reg_bits[] = {
	{
		.reg = AMD_REG_REPTMP_CTRL,
		.a1 = 21,
		.a2 = 11,
		.flags = (CTLFLAG_RD | CTLTYPE_INT),
		.fmt = "IK",
		.oid_handler = amdtemp_rtc_temp_sysctl,
		.name = "current_temperature",
		.descr = __DESCR("Provides the current control "
		"temperature, Tctl, after the slew-rate controls have "
		"been applied.")
	}, {
		.reg = AMD_REG_REPTMP_CTRL,
		.a1 = 16,
		.a2 = 2,
		.flags = (CTLFLAG_RW | CTLTYPE_UINT),
		.fmt = "IU",
		.oid_handler = amdtemp_sysctl_reg_bits,
		.name = "current_temperature_tj_sel",
		.descr = __DESCR("Specifies a value used to create "
		"Tctl.")
	}, {
		.reg = AMD_REG_REPTMP_CTRL,
		.a1 = 7,
		.a2 = 1,
		.flags = (CTLFLAG_RW | CTLTYPE_UINT),
		.fmt = "IU",
		.oid_handler = amdtemp_sysctl_reg_bits,
		.name = "temperature_slew_downward_enable",
		.descr = __DESCR("Temperature slew downward enable.")
	}, {
		.reg = AMD_REG_REPTMP_CTRL,
		.a1 = 5,
		.a2 = 2,
		.flags = (CTLFLAG_RW | CTLTYPE_UINT),
		.fmt = "IU",
		.oid_handler = amdtemp_sysctl_reg_bits,
		.name = "temperature_max_diff_up",
		.descr = __DESCR("Specifies the maximum difference, "
		"(Tctlm - Tctl), when Tctl immediatly updates to "
		"Tctlm.")
	}, {
		.reg = AMD_REG_REPTMP_CTRL,
		.a1 = 8,
		.a2 = 5,
		.flags = (CTLFLAG_RW | CTLTYPE_UINT),
		.fmt = "IU",
		.oid_handler = amdtemp_sysctl_reg_bits,
		.name = "per_step_time_dn",
		.descr = __DESCR("Specifies the time that Tctlm must "
		"remain below Tctl before applying a 0.125 downward "
		"step.")
	}, {
		.reg = AMD_REG_REPTMP_CTRL,
		.a1 = 0,
		.a2 = 5,
		.flags = (CTLFLAG_RW | CTLTYPE_UINT),
		.fmt = "IU",
		.oid_handler = amdtemp_sysctl_reg_bits,
		.name = "per_step_time_up",
		.descr = __DESCR("Specifies the time that Tctlm must "
		"remain above Tctl before applying a 0.125 upward "
		"step.")
	}
};

/*
 * Device methods.
 */
static void	amdtemp_identify(driver_t *driver, device_t parent);
static int	amdtemp_probe(device_t dev);
static int	amdtemp_attach(device_t dev);
static int	amdtemp_detach(device_t dev);
static void	amdtemp_intrhook(void *arg);


static device_method_t amdtemp_methods[] = {
	/* Device interface */
	DEVMETHOD(device_identify,	amdtemp_identify),
	DEVMETHOD(device_probe,		amdtemp_probe),
	DEVMETHOD(device_attach,	amdtemp_attach),
	DEVMETHOD(device_detach,	amdtemp_detach),
	DEVMETHOD_END
};

static driver_t amdtemp_driver = {
	"amdtemp",
	amdtemp_methods,
	sizeof(struct amdtemp_softc),
};
static devclass_t amdtemp_devclass;
DRIVER_MODULE(amdtemp, hostb, amdtemp_driver, amdtemp_devclass, NULL, NULL);
MODULE_VERSION(amdtemp, 1);


static int
amdtemp_dev_check(device_t dev)
{
	uint32_t cpuid;

	if (resource_disabled("amdtemp", 0))
		return (ENXIO);
	/*
	 * Device 18h Function 3 Configuration Registers:
	 * vendor = AMD (0x1022)
	 * class = bridge (0x06000000)
	 * function = 3
	 */
	if (pci_get_vendor(dev) != CPU_VENDOR_AMD ||
	    pci_get_class(dev) != PCIC_BRIDGE ||
	    pci_get_function(dev) != 3)
		return (ENXIO);
	/* Does processor have Temperature sensor / THERMTRIP / HTC ? */
	if ((amd_pminfo & (AMDPM_TS | AMDPM_TTP | AMDPM_TM)) == 0)
		return (ENXIO);
	/* Check minimum cpu family. */
	cpuid = pci_read_config(dev, AMD_REG_CPUID, 4);
	if (cpuid != cpu_id) { /* XXX: Ryzen fix. */
		device_printf(dev, "cpu_id = %x, AMD_REG_CPUID = %x.\n",
		    cpu_id, cpuid);
		cpuid = cpu_id;
	}
	if (CPUID_TO_FAMILY(cpuid) < 0x0f)
		return (ENXIO);

	return (0);
}

static void
amdtemp_identify(driver_t *driver, device_t parent)
{
	device_t child;

	/* Make sure we're not being doubly invoked. */
	if (device_find_child(parent, "amdtemp", -1) != NULL)
		return;
	if (amdtemp_dev_check(parent))
		return;
	child = device_add_child(parent, "amdtemp", -1);
	if (child == NULL)
		device_printf(parent, "add amdtemp child failed.\n");
}

static int
amdtemp_probe(device_t dev)
{

	if (amdtemp_dev_check(dev))
		return (ENXIO);
	device_set_desc(dev, "AMD CPU On-Die Thermal Sensors");

	return (BUS_PROBE_GENERIC);
}

static int
amdtemp_attach(device_t dev)
{
	amdtemp_softc_p sc = device_get_softc(dev);
	uint32_t i, cpuid, model, family, regs[4], bid;
	union reg_amd_thermtrip_status_desc reg_tts;
	union reg_amd_htc_desc reg_htc;
	struct sysctl_ctx_list *ctx;
	struct sysctl_oid_list *child, *list;
	struct sysctl_oid *node, *sub_node;
	char str[32];
	int erratum319 = 0;

	sc->dev = dev;
	/* Find number of cores per package. */
	sc->cpu_ncores = ((amd_feature2 & AMDID2_CMP) ?
	    ((cpu_procinfo2 & AMDID_CMP_CORES) + 1) : 1);
	if (sc->cpu_ncores > MAXCPU)
		return (ENXIO);
	mtx_init(&sc->lock, device_get_nameunit(dev), "amdtemp", MTX_DEF);

	/* Detect supported therm interfaces. */
	do_cpuid(0x80000001, regs);
	cpuid = pci_read_config(dev, AMD_REG_CPUID, 4);
	if (cpuid != cpu_id) /* XXX: Ryzen fix. */
		cpuid = cpu_id;
	model = CPUID_TO_MODEL(cpuid);
	family = CPUID_TO_FAMILY(cpuid);

	/* PCI reading specific. */
	switch (family) {
	case 0x15:
		if (model >= 0x60 && model <= 0x6f) 
			sc->pci_io = pci_io_smu_f15; /* Read via SMU. */
		break;
	case 0x17: /* Read via SMN. */
		sc->pci_io = pci_io_smn_f17;
		break;
	}

	/* Temperature sensor / Reported Temperature Control. */
	if (family > 0x0f && (amd_pminfo & AMDPM_TS) != 0)
		sc->flags |= AMDTEMP_F_RTC;

	/* ThermalTrip. */
	switch (family) {
	case 0x0f:
		/*
		 * Thermaltrip Status Register
		 *
		 * - ThermSenseCoreSel
		 *
		 * Revision F & G:	0 - Core1, 1 - Core0
		 * Other:		0 - Core0, 1 - Core1
		 *
		 * - CurTmp
		 *
		 * Revision G:		bits 23-14
		 * Other:		bits 23-16
		 *
		 * XXX According to the BKDG, CurTmp, ThermSenseSel and
		 * ThermSenseCoreSel bits were introduced in Revision F
		 * but CurTmp seems working fine as early as Revision C.
		 * However, it is not clear whether ThermSenseSel and/or
		 * ThermSenseCoreSel work in undocumented cases as well.
		 * In fact, the Linux driver suggests it may not work but
		 * we just assume it does until we find otherwise.
		 *
		 * XXX According to Linux, CurTmp starts at -28C on
		 * Socket AM2 Revision G processors, which is not
		 * documented anywhere.
		 * XXX check TjOffset and DiodeOffset for -49C / -28C
		 */
		if ((amd_pminfo & AMDPM_TTP) == 0) /* No TTP: THERMTRIP */
			break;
		reg_tts.u32 = pci_read_config(dev, AMD_REG_THERMTRIP_STAT, 4);
		if (reg_tts.bits.ThermtpEn == 0)
			break;
		if ((model == 0x04 && (cpuid & CPUID_STEPPING) == 0) ||
		    (model == 0x05 && (cpuid & CPUID_STEPPING) <= 1))
			break; /* No ThermalTrip. */
		sc->flags |= AMDTEMP_F_TTS;
		if (model >= 0x40)
			sc->tts_flags |= AMDTEMP_TTS_F_CS_SWAP;
		if (model >= 0x60 && model != 0xc1) {
			bid = ((regs[1] >> 9) & 0x1f);
			switch (model) {
			case 0x68: /* Socket S1g1 */
			case 0x6c:
			case 0x7c:
				break;
			case 0x6b: /* Socket AM2 and ASB1 (2 cores) */
				if (bid != 0x0b && bid != 0x0c) {
					sc->tts_flags |= AMDTEMP_TTS_F_OFF28;
				}
				break;
			case 0x6f: /* Socket AM2 and ASB1 (1 core) */
			case 0x7f:
				if (bid != 0x07 && bid != 0x09 &&
				    bid != 0x0c) {
					sc->tts_flags |= AMDTEMP_TTS_F_OFF28;
				}
				break;
			default:
				sc->tts_flags |= AMDTEMP_TTS_F_OFF28;
				break;
			}
			sc->tts_flags |= AMDTEMP_TTS_F_CT_10BIT;
		}
		break;
	default:
		if ((amd_pminfo & AMDPM_TTP) == 0) /* No TTP: THERMTRIP */
			break;
		sc->flags |= AMDTEMP_F_TTS;
		if (amdtemp_tts_get_temp(sc, AMD_REG_THERMTRIP_STAT, 0, 0) == 2241 &&
		    amdtemp_tts_get_temp(sc, AMD_REG_THERMTRIP_STAT, 0, 1) == 2241)
			sc->tts_flags |= AMDTEMP_TTS_F_NO_SENS;
		break;
	}

	/* Hardware Thermal Control (HTC). */
	if (amd_pminfo & AMDPM_TM) {
		reg_htc.u32 = amdtemp_pci_read(sc, AMD_REG_HTC_CTRL);
		if (reg_htc.bits.HtcEn)
			sc->flags |= AMDTEMP_F_HTC;
	}

	/* Init sysctl interface. */
	ctx = device_get_sysctl_ctx(dev);
	child = SYSCTL_CHILDREN(device_get_sysctl_tree(dev));
	if (sc->flags & AMDTEMP_F_RTC) { /* Reported Temperature Control */
		node = SYSCTL_ADD_NODE(ctx, child, OID_AUTO, "rtc",
		    CTLFLAG_RD, NULL, "Reported Temperature Control");
		list = SYSCTL_CHILDREN(node);
		amdtemp_sysctl_reg_add(sc, list, amdtemp_reptmp_reg_bits,
		    nitems(amdtemp_reptmp_reg_bits));
		SYSCTL_ADD_INT(ctx, list, OID_AUTO, "sensor_offset",
		    CTLFLAG_RW, &sc->rtc_temp_offset, 0,
		    "Temperature sensor offset");
	}
	if (sc->flags & AMDTEMP_F_TTS) { /* Thermaltrip Status */
		node = SYSCTL_ADD_NODE(ctx, child, OID_AUTO, "tts",
		    CTLFLAG_RD, NULL, "Thermaltrip Status");
		list = SYSCTL_CHILDREN(node);
		amdtemp_sysctl_reg_add(sc, list,
		    amdtemp_thermtrip_status_reg_bits,
		    nitems(amdtemp_thermtrip_status_reg_bits));
		
		for (i = 0; i < sc->cpu_ncores && i < 2; i ++) {
			if (sc->tts_flags & AMDTEMP_TTS_F_NO_SENS)
				break;
			snprintf(str, sizeof(str), "core%i", i);
			sub_node = SYSCTL_ADD_NODE(ctx, list, OID_AUTO,
			    str, CTLFLAG_RD, NULL, "CPU core sensors");
			SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(sub_node),
			    OID_AUTO, "sensor0",
			    (CTLTYPE_INT | CTLFLAG_RD), sc,
			    MAKE_ARG2(AMD_REG_THERMTRIP_STAT, i, 0),
			    amdtemp_tts_temp_reg_sysctl, "IK",
			    "Sensor 0 temperature");
			SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(sub_node),
			    OID_AUTO, "sensor1",
			    (CTLTYPE_INT | CTLFLAG_RD), sc,
			    MAKE_ARG2(AMD_REG_THERMTRIP_STAT, i, 1),
			    amdtemp_tts_temp_reg_sysctl, "IK",
			    "Sensor 1 temperature");
			SYSCTL_ADD_INT(ctx, SYSCTL_CHILDREN(sub_node),
			    OID_AUTO, "sensor0_offset", CTLFLAG_RW,
			    &sc->tts_temp_offset[((i << 1) | 0)], 0,
			    "Temperature sensor offset");
			SYSCTL_ADD_INT(ctx, SYSCTL_CHILDREN(sub_node),
			    OID_AUTO, "sensor1_offset", CTLFLAG_RW,
			    &sc->tts_temp_offset[((i << 1) | 1)], 0,
			    "Temperature sensor offset");
		}
	}
	if (sc->flags & AMDTEMP_F_HTC) { /* Hardware Thermal Control (HTC) */
		node = SYSCTL_ADD_NODE(ctx, child, OID_AUTO, "htc",
		    CTLFLAG_RD, NULL, "Hardware Thermal Control (HTC)");
		amdtemp_sysctl_reg_add(sc, SYSCTL_CHILDREN(node),
		    amdtemp_htc_reg_bits, nitems(amdtemp_htc_reg_bits));
	}

	/* Verbose staff. */
	if (bootverbose) {
		/* CPUID Fn8000_0007_EDX Advanced Power Management Information
		 * 0 TS: Temperature sensor.
		 * 3 TTP: THERMTRIP. Value: Fuse[ThermTripEn].
		 * 4 TM: hardware thermal control (HTC). Value: ~Fuse[HtcDis].
		 */
		device_printf(dev, "amdtemp_attach: %d:%d:%d\n",
		    pci_get_bus(dev), pci_get_slot(dev),
		    pci_get_function(dev));
		if (amd_pminfo & AMDPM_TS) {
			device_printf(dev, "CPU have TS: Temperature "
			    "sensor.\n");
		}
		if (sc->flags & AMDTEMP_F_RTC) {
			device_printf(dev, "Found: Reported "
			    "Temperature Control (RTC).\n");
		}
		if (sc->flags & AMDTEMP_F_TTS) {
			device_printf(dev, "Found: Thermaltrip Status "
			    "(TTS).\n");
		}
		if (amd_pminfo & AMDPM_TM) {
			device_printf(dev, "Found: Hardware Thermal "
			    "Control (HTC), state: %sabled.\n",
			    ((sc->flags & AMDTEMP_F_HTC) ? "en" : "dis"));
		}
	}
	if (family == 0x10) {
		/*
		 * Erratum 319 Inaccurate Temperature Measurement
		 * http://support.amd.com/us/Processor_TechDocs/41322.pdf
		 */
		switch (((regs[1] >> 28) & 0x0f)) {
		case 0:	/* Socket F */
			erratum319 = 1;
			break;
		case 1:	/* Socket AM2+ or AM3 */
			bid = pci_cfgregread(pci_get_bus(dev),
			    pci_get_slot(dev), 2,
			    AMD_REG_DRAM_CONF_HIGH, 2);
			if ((bid & AMD_REG_DRAM_MODE_DDR3) ||
			    model > 0x04 ||
			    (model == 0x04 && (cpuid & CPUID_STEPPING) >= 3))
				break;
			/* XXX 00100F42h (RB-C2) exists in both formats. */
			erratum319 = 1;
			break;
		}
		if (erratum319) {
			device_printf(dev, "Erratum 319: temperature "
			    "measurement may be inaccurate.\n");
		}
	}

	/*
	 * Try to create dev.cpu sysctl entries and setup intrhook function.
	 * This is needed because the cpu driver may be loaded late on boot,
	 * after us.
	 */
	amdtemp_intrhook(sc);
	if (sc->sysctl_cpu[0] == NULL) {
		sc->sc_ich.ich_func = amdtemp_intrhook;
		sc->sc_ich.ich_arg = sc;
		if (config_intrhook_establish(&sc->sc_ich)) {
			amdtemp_detach(dev);
			device_printf(dev, "config_intrhook_establish "
			    "failed!\n");
			return (ENXIO);
		}
	}

	return (0);
}

int
amdtemp_detach(device_t dev)
{
	amdtemp_softc_p sc = device_get_softc(dev);
	uint32_t i;

	for (i = 0; i < sc->cpu_ncores; i ++) {
		if (sc->sysctl_cpu[i]) {
			sysctl_remove_oid(sc->sysctl_cpu[i], 1, 0);
		}
	}
	/* NewBus removes the dev.amdtemp.N tree by itself. */
	if (sc->sc_ich.ich_arg) {
		sc->sc_ich.ich_arg = NULL;
		config_intrhook_disestablish(&sc->sc_ich);
	}
	mtx_destroy(&sc->lock);

	return (0);
}

void
amdtemp_intrhook(void *arg)
{
	amdtemp_softc_p sc = arg;
	device_t dev = sc->dev, acpi, cpu, nexus;
	int (*sysctl_handler)(SYSCTL_HANDLER_ARGS);
	intptr_t sysctl_arg2;
	uint32_t i, unit_base;

	if (sc->sc_ich.ich_arg) {
		sc->sc_ich.ich_arg = NULL;
		config_intrhook_disestablish(&sc->sc_ich);
	}

	/* dev.cpu.N.temperature. */
	nexus = device_find_child(root_bus, "nexus", 0);
	acpi = device_find_child(nexus, "acpi", 0);
	/* XXX: cpu_ncores not constant for different CPUs... */
	unit_base = (device_get_unit(dev) * sc->cpu_ncores);

	for (i = 0; i < sc->cpu_ncores; i ++) {
		if (sc->sysctl_cpu[i])
			continue;
		cpu = device_find_child(acpi, "cpu", (unit_base + i));
		if (cpu == NULL)
			continue;
		sysctl_handler = NULL;
		if (sc->flags & AMDTEMP_F_RTC) {
			/* Reported Temperature Control */
			sysctl_handler = amdtemp_rtc_temp_sysctl;
			sysctl_arg2 = MAKE_ARG2(AMD_REG_REPTMP_CTRL, 21, 11);
		} else if ((sc->flags & AMDTEMP_F_TTS) &&
		    (sc->tts_flags & AMDTEMP_TTS_F_NO_SENS) == 0) {
			/* Thermaltrip Status */
			sysctl_handler = amdtemp_tts_temp_reg_sysctl;
			sysctl_arg2 = MAKE_ARG2(AMD_REG_THERMTRIP_STAT, i, 0xff);
		}
		if (sysctl_handler == NULL)
			continue;
		sc->sysctl_cpu[i] = SYSCTL_ADD_PROC(
		    device_get_sysctl_ctx(cpu),
		    SYSCTL_CHILDREN(device_get_sysctl_tree(cpu)),
		    OID_AUTO, "temperature", (CTLTYPE_INT | CTLFLAG_RD),
		    sc, sysctl_arg2, sysctl_handler, "IK",
		    "Current temperature");
	}
}


/* Sysctl staff. */
static void
amdtemp_sysctl_reg_add(amdtemp_softc_p sc,
    struct sysctl_oid_list *child, sysctl_reg_p regs, const size_t count)
{
	struct sysctl_ctx_list *ctx = device_get_sysctl_ctx(sc->dev);
	size_t i;

	for (i = 0; i < count; i ++) {
		AMDTEMP_SYSCTL_ADD_PROC(ctx, child, OID_AUTO,
		    regs[i].name, regs[i].flags, sc,
		    MAKE_ARG2(regs[i].reg, regs[i].a1, regs[i].a2),
		    regs[i].oid_handler, regs[i].fmt, regs[i].descr);
	}
}

static int
amdtemp_sysctl_reg_bits(SYSCTL_HANDLER_ARGS)
{
	amdtemp_softc_p sc = arg1;
	uint32_t i, reg_data, reg_num, bits_off, bits_len, bits_mask = 0;
	unsigned val;
	int error;

	reg_num = ARG2_GET_REG(arg2);
	bits_off = ARG2_GET_A1(arg2);
	bits_len = ARG2_GET_A2(arg2);
	reg_data = amdtemp_pci_read(sc, reg_num);

	for (i = 0; i < bits_len; i ++) {
		bits_mask |= (((uint32_t)1) << i);
	}

	val = ((reg_data >> bits_off) & bits_mask);
	error = sysctl_handle_int(oidp, &val, 0, req);
	if (error || req->newptr == NULL || val == reg_data)
		return (error);
	reg_data &= ~(bits_mask << bits_off); /* Clear all bits at offset. */
	reg_data |= ((val & bits_mask) << bits_off); /* Set value bits. */
	amdtemp_pci_write(sc, reg_num, reg_data);

	return (0);
}


/* Thermaltrip Status Register */
static uint32_t
amdtemp_tts_get_temp(amdtemp_softc_p sc, uint32_t reg, uint8_t core,
    uint8_t sense)
{
	union reg_amd_thermtrip_status_desc reg_tts;
	uint32_t val;

	reg_tts.u32 = 0;
	if ((sc->tts_flags & AMDTEMP_TTS_F_CS_SWAP) == 0) {
		reg_tts.bits.ThermSenseCoreSel = (core ? 1 : 0);
	} else { /* Swap. */
		reg_tts.bits.ThermSenseCoreSel = (core ? 0 : 1);
	}
	reg_tts.bits.ThermSenseSel = (sense ? 1 : 0);

	/* XXX: probably lock required. */
	amdtemp_pci_write(sc, reg, reg_tts.u32);
	reg_tts.u32 = amdtemp_pci_read(sc, reg);

	val = reg_tts.bits.CurTmp;
	if ((sc->tts_flags & AMDTEMP_TTS_F_CT_10BIT) == 0) {
		val &= ~0x00000003; /* Clear first 2 bits. */
	}
	val = (AMDTEMP_ZERO_C_TO_K + ((val * 5) / 2) -
	    ((sc->tts_flags & AMDTEMP_TTS_F_OFF28) ? 280 : 490));
	val += (sc->tts_temp_offset[((2 * reg_tts.bits.ThermSenseCoreSel) +
	    reg_tts.bits.ThermSenseSel)] * 10);

	return (val);
}
/* If 0xff == ARG2_GET_A2(arg2) then retun max temp for core. */
static int
amdtemp_tts_temp_reg_sysctl(SYSCTL_HANDLER_ARGS)
{
	amdtemp_softc_p sc = arg1;
	uint32_t reg_num;
	unsigned val;
	int error;

	reg_num = ARG2_GET_REG(arg2);
	if (ARG2_GET_A2(arg2) == 0xff) { /* Core temp max. */
		val = imax(amdtemp_tts_get_temp(sc, reg_num, ARG2_GET_A1(arg2), 0),
		    amdtemp_tts_get_temp(sc, reg_num, ARG2_GET_A1(arg2), 1));
	} else {
		val = amdtemp_tts_get_temp(sc, reg_num, ARG2_GET_A1(arg2),
		    ARG2_GET_A2(arg2));
	}
	error = sysctl_handle_int(oidp, &val, 0, req);
	if (error || req->newptr == NULL)
		return (error);

	return (0);
}


/* x64 Hardware Thermal Control (HTC) */
static int
amdtemp_htc_temp_sysctl(SYSCTL_HANDLER_ARGS)
{
	amdtemp_softc_p sc = arg1;
	union reg_amd_htc_desc reg_htc;
	uint32_t reg_num, bits_off;
	unsigned val;
	int error;

	reg_num = ARG2_GET_REG(arg2);
	bits_off = ARG2_GET_A1(arg2);

	reg_htc.u32 = amdtemp_pci_read(sc, reg_num);
	switch (bits_off) {
	case 16: /* HtcTmpLmt */
		val = (((reg_htc.bits.HtcTmpLmt * 10) / 2) + 520);
		break;
	case 24: /* HtcHystLmt */
		val = ((reg_htc.bits.HtcHystLmt * 10) / 2);
		break;
	}
	val += AMDTEMP_ZERO_C_TO_K;
	error = sysctl_handle_int(oidp, &val, 0, req);
	if (error || req->newptr == NULL)
		return (error);
	/* amdtemp_pci_write(sc, reg_num, reg_htc.u32); */

	return (0);
}


/* xA4 Reported Temperature Control Register */
static int
amdtemp_rtc_temp_sysctl(SYSCTL_HANDLER_ARGS)
{
	amdtemp_softc_p sc = arg1;
	union reg_amd_rep_tmp_ctrl_desc reg_rtc;
	uint32_t reg_num;
	unsigned val;
	int error;

	reg_num = ARG2_GET_REG(arg2);

	reg_rtc.u32 = amdtemp_pci_read(sc, reg_num);
	val = ((((uint32_t)reg_rtc.bits.CurTmp) * 10) / 8);
	val += (AMDTEMP_ZERO_C_TO_K + (sc->rtc_temp_offset * 10));
	if (3 == reg_rtc.bits.CurTmpTjSel ||
	    0 != reg_rtc.bits.CurTmpRangeSel) { /* not RangeUnajusted. */
		val -= 490;
	}
	error = sysctl_handle_int(oidp, &val, 0, req);
	if (error || req->newptr == NULL)
		return (error);

	return (0);
}


static uint32_t
amdtemp_pci_read(amdtemp_softc_p sc, uint32_t addr)
{
	uint32_t ret, bus;
	device_t dev;

	if (sc->pci_io.addr == 0)
		return (pci_read_config(sc->dev, addr, 4));

	bus = sc->pci_io.bus;
	if (bus == 0xffffffff)
		bus = pci_get_bus(sc->dev);
	dev = pci_find_bsf(bus, sc->pci_io.slot, sc->pci_io.func);
	if (dev == NULL) {
		device_printf(sc->dev, "Couldn't find NB PCI device\n");
		return (0);
	}

	AMDTEMP_LOCK(sc);
	pci_write_config(dev, sc->pci_io.addr,
	    (sc->pci_io.addr_offset + addr), 4);
	ret = pci_read_config(dev, sc->pci_io.data, 4);
	AMDTEMP_UNLOCK(sc);

	return (ret);
}

static void
amdtemp_pci_write(amdtemp_softc_p sc, uint32_t addr, uint32_t data)
{
	uint32_t bus;
	device_t dev;

	if (sc->pci_io.addr == 0) {
		pci_write_config(sc->dev, addr, data, 4);
		return;
	}

	bus = sc->pci_io.bus;
	if (bus == 0xffffffff)
		bus = pci_get_bus(sc->dev);
	dev = pci_find_bsf(bus, sc->pci_io.slot, sc->pci_io.func);
	if (dev == NULL) {
		device_printf(sc->dev, "Couldn't find NB PCI device\n");
		return;
	}

	AMDTEMP_LOCK(sc);
	pci_write_config(dev, sc->pci_io.addr,
	    (sc->pci_io.addr_offset + addr), 4);
	pci_write_config(dev, sc->pci_io.data, data, 4);
	AMDTEMP_UNLOCK(sc);
}
