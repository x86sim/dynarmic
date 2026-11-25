/* This file is part of the dynarmic project.
 * Copyright (c) 2018 MerryMage
 * SPDX-License-Identifier: 0BSD
 */

#include "dynarmic/frontend/A64/translate/impl/impl.h"

namespace Dynarmic::A64 {

// x86Sim: whether this conditional branch may be a side exit (TranslationOptions::max_side_exits): not when single
// stepping, within the budget, not running into the stop PC, and not a block grown past 256 instructions.
bool TranslatorVisitor::CanSideExit() {
    if (options.max_side_exits == 0 || side_exits >= options.max_side_exits || ir.current_location->SingleStepping())
        return false;
    if (ir.block.CycleCount() >= 256)
        return false;
    const u64 next = ir.PC() + 4;
    if (options.side_exit_stop_pc != 0 && next == options.side_exit_stop_pc)
        return false;
    side_exits++;
    return true;
}

bool TranslatorVisitor::B_cond(Imm<19> imm19, Cond cond) {
    const s64 offset = concatenate(imm19, Imm<2>{0}).SignExtend<s64>();
    const u64 target = ir.PC() + offset;

    if (cond != Cond::AL && cond != Cond::NV && CanSideExit()) {
        ir.SideExitIf(cond, ir.current_location->SetPC(target));
        return true;
    }

    const auto cond_pass = IR::Term::LinkBlock{ir.current_location->SetPC(target)};
    const auto cond_fail = IR::Term::LinkBlock{ir.current_location->AdvancePC(4)};
    ir.SetTerm(IR::Term::If{cond, cond_pass, cond_fail});
    return false;
}

bool TranslatorVisitor::B_uncond(Imm<26> imm26) {
    const s64 offset = concatenate(imm26, Imm<2>{0}).SignExtend<s64>();
    const u64 target = ir.PC() + offset;

    ir.SetTerm(IR::Term::LinkBlock{ir.current_location->SetPC(target)});
    return false;
}

bool TranslatorVisitor::BL(Imm<26> imm26) {
    const s64 offset = concatenate(imm26, Imm<2>{0}).SignExtend<s64>();

    X(64, Reg::R30, ir.Imm64(ir.PC() + 4));
    ir.PushRSB(ir.current_location->AdvancePC(4));

    const u64 target = ir.PC() + offset;
    ir.SetTerm(IR::Term::LinkBlock{ir.current_location->SetPC(target)});
    return false;
}

bool TranslatorVisitor::BLR(Reg Rn) {
    const auto target = X(64, Rn);

    X(64, Reg::R30, ir.Imm64(ir.PC() + 4));
    ir.PushRSB(ir.current_location->AdvancePC(4));

    ir.SetPC(target);
    ir.SetTerm(IR::Term::FastDispatchHint{});
    return false;
}

bool TranslatorVisitor::BR(Reg Rn) {
    const auto target = X(64, Rn);

    ir.SetPC(target);
    ir.SetTerm(IR::Term::FastDispatchHint{});
    return false;
}

bool TranslatorVisitor::RET(Reg Rn) {
    const auto target = X(64, Rn);

    ir.SetPC(target);
    ir.SetTerm(IR::Term::PopRSBHint{});
    return false;
}

bool TranslatorVisitor::CBZ(bool sf, Imm<19> imm19, Reg Rt) {
    const size_t datasize = sf ? 64 : 32;
    const s64 offset = concatenate(imm19, Imm<2>{0}).SignExtend<s64>();

    const IR::U32U64 operand1 = X(datasize, Rt);

    if (CanSideExit()) {
        ir.SideExitIfBit(ir.IsZero(operand1), true, ir.current_location->SetPC(ir.PC() + offset));
        return true;
    }
    ir.SetCheckBit(ir.IsZero(operand1));

    const u64 target = ir.PC() + offset;
    const auto cond_pass = IR::Term::LinkBlock{ir.current_location->SetPC(target)};
    const auto cond_fail = IR::Term::LinkBlock{ir.current_location->AdvancePC(4)};
    ir.SetTerm(IR::Term::CheckBit{cond_pass, cond_fail});
    return false;
}

bool TranslatorVisitor::CBNZ(bool sf, Imm<19> imm19, Reg Rt) {
    const size_t datasize = sf ? 64 : 32;
    const s64 offset = concatenate(imm19, Imm<2>{0}).SignExtend<s64>();

    const IR::U32U64 operand1 = X(datasize, Rt);

    if (CanSideExit()) {
        ir.SideExitIfBit(ir.IsZero(operand1), false, ir.current_location->SetPC(ir.PC() + offset));
        return true;
    }
    ir.SetCheckBit(ir.IsZero(operand1));

    const u64 target = ir.PC() + offset;
    const auto cond_pass = IR::Term::LinkBlock{ir.current_location->AdvancePC(4)};
    const auto cond_fail = IR::Term::LinkBlock{ir.current_location->SetPC(target)};
    ir.SetTerm(IR::Term::CheckBit{cond_pass, cond_fail});
    return false;
}

bool TranslatorVisitor::TBZ(Imm<1> b5, Imm<5> b40, Imm<14> imm14, Reg Rt) {
    const size_t datasize = b5 == 1 ? 64 : 32;
    const u8 bit_pos = concatenate(b5, b40).ZeroExtend<u8>();
    const s64 offset = concatenate(imm14, Imm<2>{0}).SignExtend<s64>();

    const auto operand = X(datasize, Rt);

    if (CanSideExit()) {
        ir.SideExitIfBit(ir.TestBit(operand, ir.Imm8(bit_pos)), false, ir.current_location->SetPC(ir.PC() + offset));
        return true;
    }
    ir.SetCheckBit(ir.TestBit(operand, ir.Imm8(bit_pos)));

    const u64 target = ir.PC() + offset;
    const auto cond_1 = IR::Term::LinkBlock{ir.current_location->AdvancePC(4)};
    const auto cond_0 = IR::Term::LinkBlock{ir.current_location->SetPC(target)};
    ir.SetTerm(IR::Term::CheckBit{cond_1, cond_0});
    return false;
}

bool TranslatorVisitor::TBNZ(Imm<1> b5, Imm<5> b40, Imm<14> imm14, Reg Rt) {
    const size_t datasize = b5 == 1 ? 64 : 32;
    const u8 bit_pos = concatenate(b5, b40).ZeroExtend<u8>();
    const s64 offset = concatenate(imm14, Imm<2>{0}).SignExtend<s64>();

    const auto operand = X(datasize, Rt);

    if (CanSideExit()) {
        ir.SideExitIfBit(ir.TestBit(operand, ir.Imm8(bit_pos)), true, ir.current_location->SetPC(ir.PC() + offset));
        return true;
    }
    ir.SetCheckBit(ir.TestBit(operand, ir.Imm8(bit_pos)));

    const u64 target = ir.PC() + offset;
    const auto cond_1 = IR::Term::LinkBlock{ir.current_location->SetPC(target)};
    const auto cond_0 = IR::Term::LinkBlock{ir.current_location->AdvancePC(4)};
    ir.SetTerm(IR::Term::CheckBit{cond_1, cond_0});
    return false;
}

}  // namespace Dynarmic::A64
