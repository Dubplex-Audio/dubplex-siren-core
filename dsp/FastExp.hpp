// SPDX-License-Identifier: PolyForm-Shield-1.0.0
#pragma once
// exp(x) rapide pour les jonctions du solveur. Erreur relative ~1e-7 : le residu de
// Newton se juge a 1e-3 au mieux, et la jacobienne reprend la meme valeur, donc le
// solveur reste coherent avec lui-meme. libm coute ~51 ns sur un Cortex-A53, et il y en
// a jusqu'a 6 par assemblee. Hors SIREN_FAST_EXP, c'est std::exp.
#include <cmath>
#include <cstdint>
#include <cstring>

namespace siren {
#ifdef SIREN_FAST_EXP
inline double jexpValue(double x)
{
    // x = k ln2 + r, |r| <= ln2/2 ; exp(r) par Taylor de degre 6, schema d'Estrin.
    const double kd = std::floor(x * 1.4426950408889634 + 0.5);
    const double r  = x - kd * 0.6931471805599453;
    const double r2 = r * r;
    const double p  = (1.0 + r) + r2 * (0.5 + r * (1.0 / 6.0))
                    + r2 * r2 * ((1.0 / 24.0 + r * (1.0 / 120.0)) + r2 * (1.0 / 720.0));
    const int64_t bits = (int64_t) (kd + 1023.0) << 52;
    double scale; std::memcpy(&scale, &bits, sizeof scale);
    return p * scale;
}
#else
inline double jexpValue(double x) { return std::exp(x); }
#endif
} // namespace siren
