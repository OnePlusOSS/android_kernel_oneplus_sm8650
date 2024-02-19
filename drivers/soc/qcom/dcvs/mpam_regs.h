/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _MPAM_REGS_H
#define _MPAM_REGS_H

/* MPAM register */
#define SYS_MPAM0_EL1			sys_reg(3, 0, 10, 5, 1)
#define SYS_MPAM1_EL1			sys_reg(3, 0, 10, 5, 0)

#define MPAM_MASK(n)			((1UL << n) - 1)

/*
 * MPAMx_ELn:
 * 15:0		PARTID_I
 * 31:16	PARTID_D
 * 39:32	PMG_I
 * 47:40	PMG_D
 * 48		TRAPMPAM1EL1
 * 49		TRAPMPAM0EL1
 * 61:49	Reserved
 * 62		TRAPLOWER
 * 63		MPAMEN
 */
#define PARTID_BITS			(16)
#define PMG_BITS			(8)
#define PARTID_MASK			MPAM_MASK(PARTID_BITS)
#define PMG_MASK			MPAM_MASK(PMG_BITS)

#define PARTID_I_SHIFT			(0)
#define PARTID_D_SHIFT			(PARTID_I_SHIFT + PARTID_BITS)
#define PMG_I_SHIFT			(PARTID_D_SHIFT + PARTID_BITS)
#define PMG_D_SHIFT			(PMG_I_SHIFT + PMG_BITS)

#define PARTID_I_MASK			(PARTID_MASK << PARTID_I_SHIFT)
#define PARTID_D_MASK			(PARTID_MASK << PARTID_D_SHIFT)
#define PARTID_I_CLR(r)			((r) & ~PARTID_I_MASK)
#define PARTID_D_CLR(r)			((r) & ~PARTID_D_MASK)
#define PARTID_CLR(r)			(PARTID_I_CLR(r) & PARTID_D_CLR(r))

#define PARTID_I_SET(r, id)		(PARTID_I_CLR(r) | ((id) << PARTID_I_SHIFT))
#define PARTID_D_SET(r, id)		(PARTID_D_CLR(r) | ((id) << PARTID_D_SHIFT))
#define PARTID_SET(r, id)		(PARTID_CLR(r) | ((id) << PARTID_I_SHIFT) | \
					((id) << PARTID_D_SHIFT))

#define PMG_I_MASK			(PMG_MASK << PMG_I_SHIFT)
#define PMG_D_MASK			(PMG_MASK << PMG_D_SHIFT)
#define PMG_I_CLR(r)			((r) & ~PMG_I_MASK)
#define PMG_D_CLR(r)			((r) & ~PMG_D_MASK)
#define PMG_CLR(r)			(PMG_I_CLR(r) & PMG_D_CLR(r))

#define PMG_I_SET(r, id)		(PMG_I_CLR(r) | ((id) << PMG_I_SHIFT))
#define PMG_D_SET(r, id)		(PMG_D_CLR(r) | ((id) << PMG_D_SHIFT))
#define PMG_SET(r, id)			(PMG_CLR(r) | ((id) << PMG_I_SHIFT) | ((id) << PMG_D_SHIFT))

#define TRAPMPAM1EL1_SHIFT		(PMG_D_SHIFT + PMG_BITS)
#define TRAPMPAM0EL1_SHIFT		(TRAPMPAM1EL1_SHIFT + 1)
#define TRAPLOWER_SHIFT			(TRAPMPAM0EL1_SHIFT + 13)
#define MPAMEN_SHIFT			(TRAPLOWER_SHIFT + 1)

#define MPAMCFG_PART_SEL	0x0100
#define MPAMCFG_CPBM		0x1000

#endif

