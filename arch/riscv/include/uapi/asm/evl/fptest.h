/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note
 *
 * Copyright (C) 2026 Siemens AG
 * Author:       Tobias Schaffner <tobias.schaffner@siemens.com>.
 */
#ifndef _EVL_RISCV_ASM_UAPI_FPTEST_H
#define _EVL_RISCV_ASM_UAPI_FPTEST_H

#include <linux/types.h>

#define evl_riscv_fp  0x1

#define evl_set_fpregs(__features, __val)
#define evl_check_fpregs(__features, __val, __bad) 0

#endif /* !_EVL_RISCV_ASM_UAPI_FPTEST_H */
