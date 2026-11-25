/* This file is part of the dynarmic project.
 * Copyright (c) 2016 MerryMage
 * SPDX-License-Identifier: 0BSD
 */

#include "dynarmic/ir/opcodes.h"

#include <array>
#include <vector>

#include <mcl/stdint.hpp>

#include "dynarmic/ir/type.h"

namespace Dynarmic::IR {

// Opcode information

namespace OpcodeInfo {

struct Meta {
    const char* name;
    Type type;
    std::vector<Type> arg_types;
};

constexpr Type Void = Type::Void;
constexpr Type A32Reg = Type::A32Reg;
constexpr Type A32ExtReg = Type::A32ExtReg;
constexpr Type A64Reg = Type::A64Reg;
constexpr Type A64Vec = Type::A64Vec;
constexpr Type Opaque = Type::Opaque;
constexpr Type U1 = Type::U1;
constexpr Type U8 = Type::U8;
constexpr Type U16 = Type::U16;
constexpr Type U32 = Type::U32;
constexpr Type U64 = Type::U64;
constexpr Type U128 = Type::U128;
constexpr Type CoprocInfo = Type::CoprocInfo;
constexpr Type NZCV = Type::NZCVFlags;
constexpr Type Cond = Type::Cond;
constexpr Type Table = Type::Table;
constexpr Type AccType = Type::AccType;

static const std::array opcode_info{
#define OPCODE(name, type, ...) Meta{#name, type, {__VA_ARGS__}},
#define A32OPC(name, type, ...) Meta{#name, type, {__VA_ARGS__}},
#define A64OPC(name, type, ...) Meta{#name, type, {__VA_ARGS__}},
#include "./opcodes.inc"
#undef OPCODE
#undef A32OPC
#undef A64OPC
};

}  // namespace OpcodeInfo

Type GetTypeOf(Opcode op) {
    return OpcodeInfo::opcode_info.at(static_cast<size_t>(op)).type;
}

// x86Sim: a flat byte table (Inst::NumArgs runs for every argument loop while translating)
static const auto num_args_of = [] {
    std::array<u8, OpcodeInfo::opcode_info.size()> n{};
    for (size_t i = 0; i < n.size(); i++)
        n[i] = static_cast<u8>(OpcodeInfo::opcode_info[i].arg_types.size());
    return n;
}();

size_t GetNumArgsOf(Opcode op) {
    return num_args_of[static_cast<size_t>(op)];
}

Type GetArgTypeOf(Opcode op, size_t arg_index) {
    return OpcodeInfo::opcode_info.at(static_cast<size_t>(op)).arg_types.at(arg_index);
}

std::string GetNameOf(Opcode op) {
    return OpcodeInfo::opcode_info.at(static_cast<size_t>(op)).name;
}

}  // namespace Dynarmic::IR
