/* This file is part of the dynarmic project.
 * Copyright (c) 2016 MerryMage
 * SPDX-License-Identifier: 0BSD
 */

#pragma once

#include <cstddef>

namespace Dynarmic::Backend::X64 {

struct JitStateInfo {
    template<typename JitStateType>
    JitStateInfo(const JitStateType&, size_t bias = 0)  // x86Sim Perf 20: offsets from r15 = state + bias
            : r15_bias(bias)
            , offsetof_guest_MXCSR(offsetof(JitStateType, guest_MXCSR) - bias)
            , offsetof_asimd_MXCSR(offsetof(JitStateType, asimd_MXCSR) - bias)
            , offsetof_rsb_ptr(offsetof(JitStateType, rsb_ptr) - bias)
            , rsb_ptr_mask(JitStateType::RSBPtrMask)
            , offsetof_rsb_location_descriptors(offsetof(JitStateType, rsb_location_descriptors) - bias)
            , offsetof_rsb_codeptrs(offsetof(JitStateType, rsb_codeptrs) - bias)
            , offsetof_cpsr_nzcv(offsetof(JitStateType, cpsr_nzcv) - bias)
            , offsetof_fpsr_exc(offsetof(JitStateType, fpsr_exc) - bias)
            , offsetof_fpsr_qc(offsetof(JitStateType, fpsr_qc) - bias)
            , offsetof_halt_reason(offsetof(JitStateType, halt_reason) - bias) {}

    const size_t r15_bias;
    const size_t offsetof_guest_MXCSR;
    const size_t offsetof_asimd_MXCSR;
    const size_t offsetof_rsb_ptr;
    const size_t rsb_ptr_mask;
    const size_t offsetof_rsb_location_descriptors;
    const size_t offsetof_rsb_codeptrs;
    const size_t offsetof_cpsr_nzcv;
    const size_t offsetof_fpsr_exc;
    const size_t offsetof_fpsr_qc;
    const size_t offsetof_halt_reason;
};

}  // namespace Dynarmic::Backend::X64
