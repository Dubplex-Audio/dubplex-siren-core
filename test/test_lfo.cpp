// SPDX-License-Identifier: PolyForm-Shield-1.0.0
// Le LFO et le reseau du sweep, resolus comme un circuit.
//
// Deux jeux de references, tous deux exterieurs a ce code :
//   - les cadences mesurees par le solveur de reference sur la CARTE COMPLETE, tous
//     chargements compris : 1,53 / 3,25 / 7,14 Hz, et un carre bien fige a OFF ;
//   - les resistances de decharge de C4 LUES A L'OHMMETRE sur la carte :
//     3,177 kOhm en SPEED 1-3 et 49,93 kOhm a OFF. Celles-la ne viennent
//     d'aucun modele, elles viennent de l'unite.
#include "LfoCore.hpp"
#include <cstdio>
#include <cmath>
#include <vector>

static int failures = 0;
static void check(bool ok, const char* m)
{ std::printf("%s%s\n", ok ? "ok:   " : "FAIL: ", m); if (!ok) ++failures; }
static void near(double a, double b, double tol, const char* m)
{ const bool ok = std::fabs(a - b) <= tol;
  std::printf("%s%s  (%.3f, attendu %.3f +-%.3f)\n", ok ? "ok:   " : "FAIL: ", m, a, b, tol);
  if (!ok) ++failures; }

// Cadence du carre (net 1), mesuree sur 5 s apres 1 s d'etablissement.
static double lfoRate(int speed)
{
    siren::LfoCore l; l.build(12.2f); l.setVcc(12.2f);   // le rocker
    l.setSpeed(speed); l.setButtons(false, false);
    const double dt = 1.0 / 1000.0;
    // 20 s de mesure, pas 5. A 5 s un comptage de fronts ne resout que 0,2 Hz,
    // soit 12 % a SPEED 1 — plus grossier que l'ecart qu'on cherche a juger.
    std::vector<double> y;
    for (int i = 0; i < 21000; ++i) { l.step(dt); y.push_back(l.square()); }
    double lo = 1e9, hi = -1e9;
    for (size_t i = 1000; i < y.size(); ++i) { lo = std::min(lo, y[i]); hi = std::max(hi, y[i]); }
    if (hi - lo < 1.0) return 0.0;
    const double mid = 0.5 * (lo + hi);
    int cr = 0;
    for (size_t i = 1001; i < y.size(); ++i) if (y[i - 1] <= mid && y[i] > mid) ++cr;
    return cr / 20.0;
}

int main()
{
    // 1. Les trois cadences. Le solveur de reference donne 1,53 et 3,25 Hz aux
    //    positions 1 et 2 sur le meme montage — et ce sont, a peu de chose pres,
    //    les constantes que l'ancien moteur portait en dur. La ou sa formule
    //    etait valide, le circuit resolu la retrouve.
    // Les cadences de L'UNITE, mesurees sur un enregistrement (les
    // trois vitesses en 31 s, cadence de modulation par FFT du suivi de
    // hauteur) : 1,74 / 3,66 / 8,12 Hz. C'est une reference meilleure que
    // l'ancienne (1,53 / 3,25 / 7,14, le solveur de reference a C5/C6 = 10 uF) :
    // elle vient de la machine, pas d'un modele.
    //
    // C'est aussi elle qui a donne C5/C6. Le rapport unite/modele valait
    // x1,135 / x1,127 / x1,137 — identique sur trois barreaux d'echelle
    // differents, donc imputable au seul element commun. 10 uF / 1,133 = 8,83.
    near(lfoRate(1), 1.74, 0.15, "SPEED 1 : cadence du LFO, contre l'unite");
    near(lfoRate(2), 3.66, 0.30, "SPEED 2 : cadence du LFO, contre l'unite");
    near(lfoRate(3), 8.12, 0.65, "SPEED 3 : cadence du LFO, contre l'unite");

    // 2. A OFF le 2e pole lache la base de T3 tandis que celle de T4 garde son
    //    10k. Cette dissymetrie — et non une regle ecrite — fige le carre.
    //    C'est elle qui fait caler les MODE 2 et 3, ou une moitie en depend.
    check(lfoRate(4) == 0.0, "SPEED OFF : le carre est fige, le LFO ne tourne plus");

    // 3. La decharge de C4, contre l'OHMMETRE. A OFF, le 3e pole ouvre le net 34
    //    et insere R20 47k : le chemin devient R15 + R18 + R20 = 50,2 kOhm, lu
    //    49,93 kOhm sur la carte. Avec C4 = 125 uF — deduite de la duree des
    //    morts, voir LfoCore — tau = 6,24 s. La RESISTANCE reste mesuree, seule
    //    la capacite est deduite.
    {
        siren::LfoCore l; l.build(12.2f); l.setVcc(12.2f); l.setSpeed(4);
        l.setButtons(true, false);                       // SIREN : charge C4
        const double dt = 1.0 / 1000.0;
        for (int i = 0; i < 3000; ++i) l.step(dt);
        const double v0 = l.sweep();
        check(v0 > 6.0, "SIREN charge bien C4 (par D4 + R15)");
        l.setButtons(false, false);
        double tau = -1.0;
        for (int i = 0; i < 20000 && tau < 0.0; ++i)
        { l.step(dt); if (l.sweep() < v0 * 0.3679) tau = i * dt; }
        near(tau, 6.24, 0.75, "SPEED OFF : tau de decharge = R15+R18+R20 (49,93 k a l'ohmmetre)");
    }

    std::printf(failures == 0 ? "ALL OK\n" : "%d FAILURE(S)\n", failures);
    return failures == 0 ? 0 : 1;
}
