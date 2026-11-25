/* x86Sim: a small A64 interpreter for cold code (see a64_interp.h). Semantics follow the Arm ARM pseudocode
 * (AddWithCarry, DecodeBitMasks, the bitfield moves); encodings outside the supported set return Unsupported. */

#include "dynarmic/backend/x64/a64_interp.h"

#include <atomic>

#include "dynarmic/backend/x64/a64_jitstate.h"
#include "dynarmic/backend/x64/nzcv_util.h"
#include "dynarmic/interface/A64/config.h"

namespace Dynarmic::Backend::X64 {

namespace {

using u128 = unsigned __int128;
using s128 = __int128;

inline u32 Bits(u32 v, int hi, int lo) {
    return (v >> lo) & ((1u << (hi - lo + 1)) - 1);
}
inline bool Bit(u32 v, int b) {
    return (v >> b) & 1;
}
inline s64 SignExtend(u64 v, int bits) {
    return static_cast<s64>(v << (64 - bits)) >> (64 - bits);
}
inline u64 Mask(bool sf) {
    return sf ? ~0ull : 0xffffffffull;
}
inline u64 Ones(unsigned n) {
    return n >= 64 ? ~0ull : (1ull << n) - 1;
}
inline u64 Ror(u64 v, unsigned r, unsigned size) {
    r %= size;
    v &= Ones(size);
    return r == 0 ? v : ((v >> r) | (v << (size - r))) & Ones(size);
}

struct Interp {
    A64JitState& s;
    A64::UserCallbacks* cb;

    u64 X(u32 n) const { return n == 31 ? 0 : s.reg[n]; }
    u64 XSP(u32 n) const { return n == 31 ? s.sp : s.reg[n]; }
    void SetX(u32 n, u64 v) {
        if (n != 31) s.reg[n] = v;
    }
    void SetXSP(u32 n, u64 v) {
        if (n == 31) s.sp = v; else s.reg[n] = v;
    }
    // A W write zero-extends into the X register
    void SetR(bool sf, u32 n, u64 v) { SetX(n, v & Mask(sf)); }
    void SetRSP(bool sf, u32 n, u64 v) { SetXSP(n, v & Mask(sf)); }

    u32 Nzcv() const { return NZCV::FromX64(s.cpsr_nzcv) >> 28; }
    void SetNzcv(u32 nzcv) { s.cpsr_nzcv = NZCV::ToX64(nzcv << 28); }

    bool Cond(u32 cond) const {
        const u32 f = Nzcv();
        const bool n = f & 8, z = f & 4, c = f & 2, v = f & 1;
        bool r;
        switch (cond >> 1) {
        case 0: r = z; break;
        case 1: r = c; break;
        case 2: r = n; break;
        case 3: r = v; break;
        case 4: r = c && !z; break;
        case 5: r = n == v; break;
        case 6: r = n == v && !z; break;
        default: r = true; break;
        }
        if ((cond & 1) && cond != 15) r = !r;
        return r;
    }

    // AddWithCarry: the result (truncated to the datasize) and NZCV
    static u64 Awc(bool sf, u64 x, u64 y, u32 cin, u32& nzcv) {
        if (sf) {
            const u128 us = static_cast<u128>(x) + y + cin;
            const s128 ss = static_cast<s128>(static_cast<s64>(x)) + static_cast<s64>(y) + cin;
            const u64 r = static_cast<u64>(us);
            nzcv = (static_cast<u32>(r >> 63) << 3) | (u32(r == 0) << 2) | (u32((us >> 64) != 0) << 1) |
                   u32(static_cast<s128>(static_cast<s64>(r)) != ss);
            return r;
        }
        const u64 us = static_cast<u64>(static_cast<u32>(x)) + static_cast<u32>(y) + cin;
        const s64 ss = static_cast<s64>(static_cast<s32>(x)) + static_cast<s32>(y) + cin;
        const u32 r = static_cast<u32>(us);
        nzcv = ((r >> 31) << 3) | (u32(r == 0) << 2) | (u32((us >> 32) != 0) << 1) | u32(static_cast<s64>(static_cast<s32>(r)) != ss);
        return r;
    }
    static u32 LogicFlags(bool sf, u64 r) {
        r &= Mask(sf);
        return ((sf ? (r >> 63) : (r >> 31)) & 1) << 3 | u32(r == 0) << 2;
    }

    // DecodeBitMasks; false: reserved
    static bool BitMasks(bool sf, u32 n, u32 imms, u32 immr, bool immediate, u64& wmask, u64& tmask) {
        const u32 combined = (n << 6) | (~imms & 0x3f);
        if (combined == 0) return false;
        const int len = 31 - __builtin_clz(combined);
        if (len < 1) return false;
        const unsigned datasize = sf ? 64 : 32;
        if ((1u << len) > datasize) return false;
        const u32 levels = (1u << len) - 1;
        if (immediate && (imms & levels) == levels) return false;
        const u32 S = imms & levels, R = immr & levels;
        const u32 d = (S - R) & levels;
        const unsigned esize = 1u << len;
        const u64 welem = Ror(Ones(S + 1), R, esize);
        const u64 telem = Ones(d + 1);
        wmask = tmask = 0;
        for (unsigned i = 0; i < datasize; i += esize) {
            wmask |= welem << i;
            tmask |= telem << i;
        }
        return true;
    }

    static u64 Shift(bool sf, u64 v, u32 type, u32 amount) {
        const unsigned size = sf ? 64 : 32;
        v &= Mask(sf);
        switch (type) {
        case 0: return (v << amount) & Mask(sf);
        case 1: return v >> amount;
        case 2: return static_cast<u64>((sf ? static_cast<s64>(v) : static_cast<s64>(static_cast<s32>(v))) >> amount) & Mask(sf);
        default: return Ror(v, amount, size);
        }
    }

    static u64 Extend(u64 v, u32 option, u32 shift) {
        switch (option) {
        case 0: v = static_cast<u8>(v); break;
        case 1: v = static_cast<u16>(v); break;
        case 2: v = static_cast<u32>(v); break;
        case 3: break;
        case 4: v = static_cast<u64>(static_cast<s64>(static_cast<s8>(v))); break;
        case 5: v = static_cast<u64>(static_cast<s64>(static_cast<s16>(v))); break;
        case 6: v = static_cast<u64>(static_cast<s64>(static_cast<s32>(v))); break;
        default: break;
        }
        return v << shift;
    }

    u64 Read(u64 a, unsigned bytes) {
        switch (bytes) {
        case 1: return cb->MemoryRead8(a);
        case 2: return cb->MemoryRead16(a);
        case 4: return cb->MemoryRead32(a);
        default: return cb->MemoryRead64(a);
        }
    }
    void Write(u64 a, unsigned bytes, u64 v) {
        switch (bytes) {
        case 1: cb->MemoryWrite8(a, static_cast<u8>(v)); break;
        case 2: cb->MemoryWrite16(a, static_cast<u16>(v)); break;
        case 4: cb->MemoryWrite32(a, static_cast<u32>(v)); break;
        default: cb->MemoryWrite64(a, v); break;
        }
    }

    // One register of a load/store: integer (V=0) or FP/SIMD (V=1) of `bytes` bytes. Loads read into lo/hi first.
    struct Val {
        u64 lo = 0, hi = 0;
    };
    Val Load(u64 a, unsigned bytes) {
        Val v;
        if (bytes == 16) {
            const auto q = cb->MemoryRead128(a);
            v.lo = q[0];
            v.hi = q[1];
        } else {
            v.lo = Read(a, bytes);
        }
        return v;
    }
    void Store(u64 a, unsigned bytes, bool vec, u32 t) {
        if (vec) {
            if (bytes == 16) {
                cb->MemoryWrite128(a, A64::Vector{s.vec[2 * t], s.vec[2 * t + 1]});
            } else {
                Write(a, bytes, s.vec[2 * t]);
            }
        } else {
            Write(a, bytes, X(t));
        }
    }
    // the loaded value into Rt: vec (rest of the vector register zeroed), signed or not, to 32 or 64 bits
    void Commit(const Val& v, unsigned bytes, bool vec, u32 t, bool sign, bool to64) {
        if (vec) {
            s.vec[2 * t] = bytes == 16 ? v.lo : (bytes == 8 ? v.lo : v.lo & Ones(bytes * 8));
            s.vec[2 * t + 1] = bytes == 16 ? v.hi : 0;
            return;
        }
        u64 r = v.lo;
        if (sign) r = static_cast<u64>(SignExtend(r, static_cast<int>(bytes * 8)));
        SetR(to64, t, r);
    }

    // Load/store single register: the access shape (bytes, vec, load, sign, to64) from size/V/opc; false: unsupported
    // (prefetch: prfm = true, nothing to do)
    static bool Shape(u32 size, bool V, u32 opc, unsigned& bytes, bool& load, bool& sign, bool& to64, bool& prfm) {
        prfm = false;
        sign = false;
        if (V) {
            const u32 scale = ((opc >> 1) << 2) | size;
            if (scale > 4) return false;
            bytes = 1u << scale;
            load = opc & 1;
            to64 = true;
            return true;
        }
        bytes = 1u << size;
        switch (opc) {
        case 0: load = false; to64 = size == 3; return true;
        case 1: load = true; to64 = size == 3; return true;
        case 2:
            if (size == 3) { prfm = true; return true; }
            load = true; sign = true; to64 = true; return true;
        default:
            if (size >= 2) return false;
            load = true; sign = true; to64 = false; return true;
        }
    }

    // Runs the instruction at pc; next_pc set; returns false if unsupported (nothing changed). branch: it was one.
    bool Step(u32 insn, u64 pc, u64& next_pc, bool& branch) {
        next_pc = pc + 4;
        branch = false;
        const bool sf = Bit(insn, 31);
        const u32 rd = Bits(insn, 4, 0), rn = Bits(insn, 9, 5), rm = Bits(insn, 20, 16);

        // ---- branches
        if ((insn & 0x7C000000) == 0x14000000) {  // B, BL
            if (Bit(insn, 31)) SetX(30, pc + 4);
            next_pc = pc + static_cast<u64>(SignExtend(Bits(insn, 25, 0), 26) * 4);
            branch = true;
            return true;
        }
        if ((insn & 0xFF000010) == 0x54000000) {  // B.cond
            if (Cond(Bits(insn, 3, 0))) next_pc = pc + static_cast<u64>(SignExtend(Bits(insn, 23, 5), 19) * 4);
            branch = true;
            return true;
        }
        if ((insn & 0x7E000000) == 0x34000000) {  // CBZ, CBNZ
            const bool zero = (X(rd) & Mask(sf)) == 0;
            if (zero != Bit(insn, 24)) next_pc = pc + static_cast<u64>(SignExtend(Bits(insn, 23, 5), 19) * 4);
            branch = true;
            return true;
        }
        if ((insn & 0x7E000000) == 0x36000000) {  // TBZ, TBNZ
            const u32 b = (Bits(insn, 31, 31) << 5) | Bits(insn, 23, 19);
            const bool set = (X(rd) >> b) & 1;
            if (set == Bit(insn, 24)) next_pc = pc + static_cast<u64>(SignExtend(Bits(insn, 18, 5), 14) * 4);
            branch = true;
            return true;
        }
        if ((insn & 0xFFFFFC1F) == 0xD61F0000 || (insn & 0xFFFFFC1F) == 0xD63F0000 || (insn & 0xFFFFFC1F) == 0xD65F0000) {
            const u64 target = X(rn);  // BR, BLR, RET (read before BLR writes X30)
            if ((insn & 0xFFFFFC1F) == 0xD63F0000) SetX(30, pc + 4);
            next_pc = target;
            branch = true;
            return true;
        }

        // ---- data processing, immediate
        if ((insn & 0x1F000000) == 0x10000000) {  // ADR, ADRP
            const u64 imm = static_cast<u64>(SignExtend((Bits(insn, 23, 5) << 2) | Bits(insn, 30, 29), 21));
            SetX(rd, Bit(insn, 31) ? (pc & ~0xfffull) + (imm << 12) : pc + imm);
            return true;
        }
        if ((insn & 0x1F800000) == 0x11000000) {  // ADD/ADDS/SUB/SUBS (immediate)
            const bool sub = Bit(insn, 30), setflags = Bit(insn, 29);
            const u64 imm = static_cast<u64>(Bits(insn, 21, 10)) << (Bit(insn, 22) ? 12 : 0);
            u32 nzcv;
            const u64 r = Awc(sf, XSP(rn), sub ? ~imm : imm, sub, nzcv);
            if (setflags) {
                SetNzcv(nzcv);
                SetR(sf, rd, r);
            } else {
                SetRSP(sf, rd, r);
            }
            return true;
        }
        if ((insn & 0x1F800000) == 0x12000000) {  // AND/ORR/EOR/ANDS (immediate)
            const u32 n = Bit(insn, 22);
            if (!sf && n) return false;
            u64 imm, tmask;
            if (!BitMasks(sf, n, Bits(insn, 15, 10), Bits(insn, 21, 16), true, imm, tmask)) return false;
            const u64 a = X(rn);
            u64 r;
            switch (Bits(insn, 30, 29)) {
            case 0: r = a & imm; break;
            case 1: r = a | imm; break;
            case 2: r = a ^ imm; break;
            default:
                r = a & imm;
                SetNzcv(LogicFlags(sf, r));
                SetR(sf, rd, r);
                return true;
            }
            SetRSP(sf, rd, r);
            return true;
        }
        if ((insn & 0x1F800000) == 0x12800000) {  // MOVN, MOVZ, MOVK
            const u32 opc = Bits(insn, 30, 29), hw = Bits(insn, 22, 21);
            if (opc == 1 || (!sf && hw >= 2)) return false;
            const unsigned pos = hw * 16;
            const u64 imm = static_cast<u64>(Bits(insn, 20, 5)) << pos;
            u64 r;
            if (opc == 0) r = ~imm;
            else if (opc == 2) r = imm;
            else r = (X(rd) & ~(0xffffull << pos)) | imm;
            SetR(sf, rd, r);
            return true;
        }
        if ((insn & 0x1F800000) == 0x13000000) {  // SBFM, BFM, UBFM
            const u32 opc = Bits(insn, 30, 29), n = Bit(insn, 22), immr = Bits(insn, 21, 16), imms = Bits(insn, 15, 10);
            if (opc == 3 || n != static_cast<u32>(sf) || (!sf && (immr >= 32 || imms >= 32))) return false;
            u64 wmask, tmask;
            if (!BitMasks(sf, n, imms, immr, false, wmask, tmask)) return false;
            const unsigned size = sf ? 64 : 32;
            const u64 src = X(rn) & Mask(sf);
            const u64 dst = opc == 1 ? X(rd) & Mask(sf) : 0;
            const u64 bot = (dst & ~wmask) | (Ror(src, immr, size) & wmask);
            const u64 top = opc == 0 ? (((src >> imms) & 1) ? Mask(sf) : 0) : dst;
            SetR(sf, rd, (top & ~tmask) | (bot & tmask));
            return true;
        }
        if ((insn & 0x7FA00000) == 0x13800000) {  // EXTR
            if (Bit(insn, 22) != sf) return false;
            const u32 lsb = Bits(insn, 15, 10);
            if (!sf && lsb >= 32) return false;
            const unsigned size = sf ? 64 : 32;
            const u128 concat = (static_cast<u128>(X(rn) & Mask(sf)) << size) | (X(rm) & Mask(sf));
            SetR(sf, rd, static_cast<u64>(concat >> lsb));
            return true;
        }

        // ---- data processing, register
        if ((insn & 0x1F000000) == 0x0A000000) {  // AND/BIC/ORR/ORN/EOR/EON/ANDS/BICS (shifted register)
            const u32 amount = Bits(insn, 15, 10);
            if (!sf && amount >= 32) return false;
            u64 b = Shift(sf, X(rm), Bits(insn, 23, 22), amount);
            if (Bit(insn, 21)) b = ~b & Mask(sf);
            const u64 a = X(rn);
            u64 r;
            switch (Bits(insn, 30, 29)) {
            case 0: r = a & b; break;
            case 1: r = a | b; break;
            case 2: r = a ^ b; break;
            default:
                r = a & b;
                SetNzcv(LogicFlags(sf, r));
                break;
            }
            SetR(sf, rd, r);
            return true;
        }
        if ((insn & 0x1F200000) == 0x0B000000) {  // ADD/ADDS/SUB/SUBS (shifted register)
            const u32 type = Bits(insn, 23, 22), amount = Bits(insn, 15, 10);
            if (type == 3 || (!sf && amount >= 32)) return false;
            const bool sub = Bit(insn, 30);
            const u64 b = Shift(sf, X(rm), type, amount);
            u32 nzcv;
            const u64 r = Awc(sf, X(rn), sub ? ~b : b, sub, nzcv);
            if (Bit(insn, 29)) SetNzcv(nzcv);
            SetR(sf, rd, r);
            return true;
        }
        if ((insn & 0x1F200000) == 0x0B200000) {  // ADD/ADDS/SUB/SUBS (extended register)
            const u32 shift = Bits(insn, 12, 10);
            if (Bits(insn, 23, 22) != 0 || shift > 4) return false;
            const bool sub = Bit(insn, 30), setflags = Bit(insn, 29);
            const u64 b = Extend(X(rm), Bits(insn, 15, 13), shift);
            u32 nzcv;
            const u64 r = Awc(sf, XSP(rn), sub ? ~b : b, sub, nzcv);
            if (setflags) {
                SetNzcv(nzcv);
                SetR(sf, rd, r);
            } else {
                SetRSP(sf, rd, r);
            }
            return true;
        }
        if ((insn & 0x1FE0FC00) == 0x1A000000) {  // ADC/ADCS/SBC/SBCS
            const bool sub = Bit(insn, 30);
            const u64 b = sub ? ~X(rm) : X(rm);
            u32 nzcv;
            const u64 r = Awc(sf, X(rn), b, (Nzcv() >> 1) & 1, nzcv);
            if (Bit(insn, 29)) SetNzcv(nzcv);
            SetR(sf, rd, r);
            return true;
        }
        if ((insn & 0x3FE00410) == 0x3A400000) {  // CCMN/CCMP (register / immediate)
            if (Cond(Bits(insn, 15, 12))) {
                const u64 b = Bit(insn, 11) ? Bits(insn, 20, 16) : X(rm);
                const bool sub = Bit(insn, 30);
                u32 nzcv;
                Awc(sf, X(rn), sub ? ~b : b, sub, nzcv);
                SetNzcv(nzcv);
            } else {
                SetNzcv(Bits(insn, 3, 0));
            }
            return true;
        }
        if ((insn & 0x3FE00800) == 0x1A800000) {  // CSEL/CSINC/CSINV/CSNEG
            u64 r;
            if (Cond(Bits(insn, 15, 12))) {
                r = X(rn);
            } else {
                r = X(rm);
                if (Bit(insn, 30)) r = ~r;
                if (Bit(insn, 10)) r += 1;
            }
            SetR(sf, rd, r);
            return true;
        }
        if ((insn & 0x7FE00000) == 0x1AC00000) {  // UDIV/SDIV/LSLV/LSRV/ASRV/RORV
            const u32 op = Bits(insn, 15, 10);
            const u64 a = X(rn) & Mask(sf), b = X(rm) & Mask(sf);
            const unsigned size = sf ? 64 : 32;
            u64 r;
            switch (op) {
            case 2: r = b == 0 ? 0 : a / b; break;
            case 3:
                if (b == 0) {
                    r = 0;
                } else if (sf) {
                    const s64 x = static_cast<s64>(a), y = static_cast<s64>(b);
                    r = (x == INT64_MIN && y == -1) ? a : static_cast<u64>(x / y);
                } else {
                    const s32 x = static_cast<s32>(a), y = static_cast<s32>(b);
                    r = (x == INT32_MIN && y == -1) ? a : static_cast<u64>(static_cast<u32>(x / y));
                }
                break;
            case 8: case 9: case 10: case 11: r = Shift(sf, a, op - 8, static_cast<u32>(b % size)); break;
            default: return false;
            }
            SetR(sf, rd, r);
            return true;
        }
        if ((insn & 0x7FFF0000) == 0x5AC00000) {  // RBIT/REV16/REV32/REV/CLZ/CLS
            const u32 op = Bits(insn, 15, 10);
            const u64 a = X(rn) & Mask(sf);
            const unsigned size = sf ? 64 : 32;
            u64 r = 0;
            switch (op) {
            case 0:
                for (unsigned i = 0; i < size; i++) r |= ((a >> i) & 1) << (size - 1 - i);
                break;
            case 1:
                for (unsigned i = 0; i < size; i += 16) r |= static_cast<u64>(__builtin_bswap16(static_cast<u16>(a >> i))) << i;
                break;
            case 2:
                if (sf) {
                    r = static_cast<u64>(__builtin_bswap32(static_cast<u32>(a))) | (static_cast<u64>(__builtin_bswap32(static_cast<u32>(a >> 32))) << 32);
                } else {
                    r = __builtin_bswap32(static_cast<u32>(a));
                }
                break;
            case 3:
                if (!sf) return false;
                r = __builtin_bswap64(a);
                break;
            case 4: r = a == 0 ? size : (sf ? __builtin_clzll(a) : __builtin_clz(static_cast<u32>(a))); break;
            case 5: {
                const u64 x = (a ^ (a >> 1)) & Ones(size - 1);  // bits that differ from the one above
                r = x == 0 ? size - 1 : (sf ? __builtin_clzll(x) : __builtin_clz(static_cast<u32>(x))) - 1;
                break;
            }
            default: return false;
            }
            SetR(sf, rd, r);
            return true;
        }
        if ((insn & 0x7F000000) == 0x1B000000) {  // MADD/MSUB/SMADDL/SMSUBL/SMULH/UMADDL/UMSUBL/UMULH
            const u32 op31 = Bits(insn, 23, 21), ra = Bits(insn, 14, 10);
            const bool o0 = Bit(insn, 15);
            u64 r;
            switch (op31) {
            case 0: {
                const u64 p = X(rn) * X(rm);
                r = o0 ? X(ra) - p : X(ra) + p;
                break;
            }
            case 1: case 5: {
                if (!sf) return false;
                const u64 p = op31 == 1 ? static_cast<u64>(static_cast<s64>(static_cast<s32>(X(rn))) * static_cast<s32>(X(rm)))
                                        : static_cast<u64>(static_cast<u32>(X(rn))) * static_cast<u32>(X(rm));
                r = o0 ? X(ra) - p : X(ra) + p;
                break;
            }
            case 2:
                if (!sf || o0) return false;
                r = static_cast<u64>((static_cast<s128>(static_cast<s64>(X(rn))) * static_cast<s64>(X(rm))) >> 64);
                break;
            case 6:
                if (!sf || o0) return false;
                r = static_cast<u64>((static_cast<u128>(X(rn)) * X(rm)) >> 64);
                break;
            default: return false;
            }
            SetR(sf, rd, r);
            return true;
        }

        // ---- loads and stores
        if ((insn & 0x3B000000) == 0x39000000 || (insn & 0x3B200000) == 0x38000000 || (insn & 0x3B200C00) == 0x38200800) {
            const u32 size = Bits(insn, 31, 30), opc = Bits(insn, 23, 22), rt = rd;
            const bool V = Bit(insn, 26);
            unsigned bytes;
            bool load, sign, to64, prfm;
            if (!Shape(size, V, opc, bytes, load, sign, to64, prfm)) return false;
            u64 addr, wb = 0;
            bool writeback = false;
            if ((insn & 0x3B000000) == 0x39000000) {  // unsigned offset
                const u32 scale = V ? ((opc >> 1) << 2) | size : size;
                addr = XSP(rn) + (static_cast<u64>(Bits(insn, 21, 10)) << scale);
            } else if ((insn & 0x3B200000) == 0x38000000) {  // unscaled / post / unprivileged / pre
                const u32 idx = Bits(insn, 11, 10);
                const u64 imm = static_cast<u64>(SignExtend(Bits(insn, 20, 12), 9));
                const u64 base = XSP(rn);
                if (idx == 1) { addr = base; wb = base + imm; writeback = true; }
                else if (idx == 3) { addr = base + imm; wb = addr; writeback = true; }
                else addr = base + imm;
                if (prfm && writeback) return false;
            } else {  // register offset
                const u32 option = Bits(insn, 15, 13);
                if (!(option & 2)) return false;
                const u32 scale = V ? ((opc >> 1) << 2) | size : size;
                addr = XSP(rn) + Extend(X(rm), option, Bit(insn, 12) ? scale : 0);
            }
            if (prfm) return true;
            if (load) {
                const Val v = Load(addr, bytes);
                if (writeback) SetXSP(rn, wb);
                Commit(v, bytes, V, rt, sign, to64);
            } else {
                Store(addr, bytes, V, rt);
                if (writeback) SetXSP(rn, wb);
            }
            return true;
        }
        if ((insn & 0x3B000000) == 0x18000000) {  // LDR (literal), LDRSW (literal), PRFM (literal)
            const u32 opc = Bits(insn, 31, 30), rt = rd;
            const bool V = Bit(insn, 26);
            const u64 addr = pc + static_cast<u64>(SignExtend(Bits(insn, 23, 5), 19) * 4);
            if (V) {
                if (opc == 3) return false;
                const unsigned bytes = 4u << opc;
                Commit(Load(addr, bytes), bytes, true, rt, false, true);
            } else if (opc == 3) {
                return true;  // prfm
            } else {
                const unsigned bytes = opc == 1 ? 8 : 4;
                Commit(Load(addr, bytes), bytes, false, rt, opc == 2, opc != 0);
            }
            return true;
        }
        if ((insn & 0x3A000000) == 0x28000000) {  // LDP/STP/LDNP/STNP/LDPSW
            const u32 opc = Bits(insn, 31, 30), idx = Bits(insn, 24, 23), rt = rd, rt2 = Bits(insn, 14, 10);
            const bool V = Bit(insn, 26), load = Bit(insn, 22);
            if (opc == 3) return false;
            unsigned bytes;
            bool sign = false, to64;
            if (V) {
                bytes = 4u << opc;
                to64 = true;
            } else if (opc == 1) {
                if (!load || idx == 0) return false;  // STGP (MTE), LDNP of words signed: not here
                bytes = 4;
                sign = true;
                to64 = true;
            } else {
                bytes = opc == 2 ? 8 : 4;
                to64 = opc == 2;
            }
            const u64 imm = static_cast<u64>(SignExtend(Bits(insn, 21, 15), 7)) * bytes;
            const u64 base = XSP(rn);
            const u64 addr = idx == 1 ? base : base + imm;
            const bool writeback = idx == 1 || idx == 3;
            if (load) {
                const Val a = Load(addr, bytes), b = Load(addr + bytes, bytes);
                if (writeback) SetXSP(rn, base + imm);
                Commit(a, bytes, V, rt, sign, to64);
                Commit(b, bytes, V, rt2, sign, to64);
            } else {
                // both values before any write: the writeback may change Rn, and Rt may be Rn
                Store(addr, bytes, V, rt);
                Store(addr + bytes, bytes, V, rt2);
                if (writeback) SetXSP(rn, base + imm);
            }
            return true;
        }

        // ---- NOP and barriers
        if (insn == 0xD503201F) return true;
        if ((insn & 0xFFFFF0FF) == 0xD50330BF || (insn & 0xFFFFF0FF) == 0xD503309F) {  // DMB, DSB
            std::atomic_thread_fence(std::memory_order_seq_cst);
            return true;
        }
        if ((insn & 0xFFFFF0FF) == 0xD50330DF) return true;  // ISB

        return false;
    }
};

}  // namespace

InterpStop InterpretBlock(A64JitState& st, A64::UserCallbacks* cb, size_t max_insns, size_t& executed) {
    Interp in{st, cb};
    executed = 0;
    while (executed < max_insns) {
        const u64 pc = st.pc;
        const auto word = cb->MemoryReadCode(pc);
        if (!word) return InterpStop::Unsupported;
        u64 next;
        bool branch;
        if (!in.Step(*word, pc, next, branch)) return InterpStop::Unsupported;
        st.pc = next;
        executed++;
        if (branch) return InterpStop::Branch;
    }
    return InterpStop::Branch;
}

}  // namespace Dynarmic::Backend::X64
