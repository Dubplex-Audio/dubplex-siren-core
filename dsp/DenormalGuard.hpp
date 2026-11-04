// SPDX-License-Identifier: PolyForm-Shield-1.0.0
#pragma once
// DenormalGuard.hpp — arm flush-to-zero (FTZ) on the audio thread.
//
// Denormal floats incur a per-operation penalty on the Cortex-A53 FPU. After
// a trigger the siren's analog states all decay geometrically toward zero
// (C5 discharge, the power de-click ramp, the output HP/LP one-poles), so the
// engine dwells in the denormal range for tens of seconds after every use and
// pays that penalty on every sample. Flushing denormals to zero removes the
// penalty at no audible cost. FPCR is a per-thread register, so we arm it
// once on whichever thread first runs audio.
//
// x86_64 is armed too (MXCSR FTZ + DAZ). It is not a dev-only target: the
// macOS builds are universal, so the Intel slice ships. Measured without it, engine powered and VOLUME at zero: the
// output smoother never reaches zero (volS_ sticks around 3e-42), the FIR
// ring fills with subnormals and its 65 MACs run on them every sample — on
// Intel, where each subnormal op is a microcode assist, that is 10-15 % of a
// core for as long as the knob stays down. Apple silicon has no such penalty.
// Values below 1e-38 become 0: no audible change, and the offline tests,
// which run without it, stay bit-comparable with the reference renders.

#include <cstdint>
#if defined(__SSE__) || defined(_M_X64) || defined(_M_IX86)
#include <xmmintrin.h>
#endif

namespace ftz {

inline void enable() noexcept
{
#if defined(__aarch64__)
    uint64_t fpcr;
    __asm__ __volatile__("mrs %0, fpcr" : "=r"(fpcr));
    fpcr |= (1ull << 24);   // FPCR.FZ — flush denormals (inputs & results) to zero
    __asm__ __volatile__("msr fpcr, %0" : : "r"(fpcr));
#elif defined(__SSE__) || defined(_M_X64) || defined(_M_IX86)
    _mm_setcsr(_mm_getcsr() | 0x8040);   // MXCSR.FTZ (bit 15) + MXCSR.DAZ (bit 6)
#endif
}

// Arms FTZ exactly once per thread, for callers that can afford a
// thread_local (command-line tools, some apps). A plug-in calls enable()
// directly at the top of ProcessBlock instead: a dynamic thread_local is a
// TLS lookup on every block, and re-arming is a register write.
inline void armOnce() noexcept
{
    static thread_local bool armed = false;
    if (!armed) { enable(); armed = true; }
}

} // namespace ftz
