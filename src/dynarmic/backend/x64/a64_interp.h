/* x86Sim: a small A64 interpreter for cold code (tiered translation, a64_interface.cpp GetBlock).
 *
 * In a boot without the translation cache most translated blocks ran a few times: SpringBoard (iPhone 18 Pro, iOS
 * 27.2) translated 932K blocks, 61% of them ran at most 8 times, and translating took ~45% of its CPU. A block runs
 * here until it has run UserConfig::interpret_threshold times, then it is translated as before.
 *
 * Integer code only: branches, integer data processing, loads and stores (also of FP/SIMD registers: bit copies),
 * NOP and barriers. Anything else (SIMD/FP arithmetic, SVC, system registers, exclusives, atomics, hints other than
 * NOP) stops before it; that PC is translated. Every instruction commits on its own (a pair or a writeback only after
 * all of its memory accesses), so stopping anywhere leaves a consistent guest state with pc at the next instruction.
 */
#pragma once

#include <mcl/stdint.hpp>

namespace Dynarmic::A64 {
struct UserCallbacks;
}

namespace Dynarmic::Backend::X64 {

struct A64JitState;

enum class InterpStop {
    Branch,       // the block ended with a branch (or ran max_insns instructions); pc is where execution goes on
    Unsupported,  // pc is an instruction this interpreter does not run: translate from there
};

/// Runs guest instructions from st.pc until the first branch (included) or an instruction it does not support.
/// executed: how many it ran.
InterpStop InterpretBlock(A64JitState& st, A64::UserCallbacks* cb, size_t max_insns, size_t& executed);

}  // namespace Dynarmic::Backend::X64
