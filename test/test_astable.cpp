// SPDX-License-Identifier: PolyForm-Shield-1.0.0
// L'astable audio resolu comme un circuit, pas comme une formule.
//
// Les references viennent du solveur Python de reference, lui-meme valide
// par 24 tests sur des circuits calculables a la main. On ne verifie donc pas
// une formule contre elle-meme : on verifie que la resolution embarquee retrouve
// ce que donne la resolution de reference sur le meme circuit.
#include "AstableCore.hpp"
#include <cstdio>
#include <algorithm>
#include <cmath>
#include <vector>
#include <cstdlib>

static int failures = 0;
static void check(bool ok, const char* msg)
{
    std::printf("%s%s\n", ok ? "ok:   " : "FAIL: ", msg);
    if (!ok) ++failures;
}
static void near(double a, double b, double tol, const char* msg)
{
    const bool ok = std::fabs(a - b) <= tol;
    std::printf("%s%s  (%.1f, attendu %.1f +-%.1f)\n", ok ? "ok:   " : "FAIL: ",
                msg, a, b, tol);
    if (!ok) ++failures;
}

// Fait tourner le coeur a Vm constant et rend la frequence, mesuree sur les
// passages du collecteur de T2 par son milieu.
static double freqAt(float vm, float rLadder, float vcc = 12.2f,
                     double seconds = 0.25, float fs = 192000.0f)
{
    siren::AstableCore core;
    core.reset();
    const float dt = 1.0f / fs;
    const int n = (int) (seconds * fs);
    std::vector<float> y;
    y.reserve(n);
    for (int i = 0; i < n; ++i)
    {
        core.step(dt, rLadder, vm, rLadder, vm, 0.0f, false, vcc);
        y.push_back(core.out());
    }
    // on jette le premier tiers : amorcage
    const int st = n / 3;
    float lo = 1e9f, hi = -1e9f;
    for (int i = st; i < n; ++i) { lo = std::min(lo, y[i]); hi = std::max(hi, y[i]); }
    if (hi - lo < 1.0f) return 0.0;               // cale
    const float mid = 0.5f * (lo + hi);
    int cr = 0;
    for (int i = st + 1; i < n; ++i)
        if (y[i - 1] <= mid && y[i] > mid) ++cr;
    return cr / ((double) (n - st) / fs);
}

int main()
{
    // 1. Regime lent, contre le SOLVEUR DE REFERENCE sur le
    //    meme montage a quatre noeuds : R = 470k (TONE 1), Rc = 4,66k, Vcc =
    //    12,2 V, C1/C2 = 9,30/9,39 nF sans claquage. A hmax = HMAX_AUDIO il
    //    donne 156 et 111 Hz — et le moteur d'aujourd'hui, avec le claquage a
    //    9,5 V et C1/C2 = 10,22/10,32 nF, redonne 156 et 108 : les deux
    //    changements se compensent, c'est ce qui a permis de deduire C.
    //
    //    Ces valeurs etaient 140 et 100 Hz. Elles n'ont pas bouge parce que le
    //    moteur a change : elles avaient ete calculees a C = 10 nF, avant que
    //    les DOUZE resistances du chemin audio soient lues a l'ohmmetre
    //    (9 septembre 2026, toutes a 1 % du nominal) et que C en soit deduite a
    //    9,30/9,39 nF. Comparer le moteur d'aujourd'hui a une reference calculee
    //    sur d'autres composants n'aurait teste que l'ecart des composants.
    near(freqAt(11.42f, 470e3f), 156.0, 12.0, "Vm 11,42 V -> 156 Hz (regime lent)");
    near(freqAt(7.11f,  470e3f), 111.0, 12.0, "Vm 7,11 V -> 111 Hz");

    // 2. Le point qu'aucune formule a un noeud ne peut donner : sous le seuil de
    //    conduction, le circuit n'est PAS mort. L'echelle ne tient plus la
    //    saturation, le collecteur se relache et entraine la base opposee par le
    //    condensateur de couplage : l'astable repart a ~1 kHz, cadence par Rc*C.
    const double fLow = freqAt(0.346f, 470e3f);
    check(fLow > 300.0, "Vm 0,35 V : ca oscille encore, loin de caler");
    // 1019 Hz sans claquage ; avec lui la base kickee repart de -10 V et non
    // -11,5, atteint le seuil plus tot dans la montee du collecteur oppose, et
    // le regime rapide est plus rapide : ~1,5 kHz.
    near(fLow, 1512.0, 250.0, "Vm 0,35 V -> ~1,5 kHz (regime rapide, Rc*C)");

    // 3. La remontee elle-meme : la hauteur descend puis repart vers le haut
    //    quand Vm continue de baisser. C'est la mort du MODE 4.
    const double f11 = freqAt(11.42f, 470e3f), f7 = freqAt(7.11f, 470e3f);
    const double f3 = freqAt(3.36f, 470e3f), f15 = freqAt(1.52f, 470e3f);
    std::printf("      hauteur vs Vm : 11,42->%.0f  7,11->%.0f  3,36->%.0f  1,52->%.0f Hz\n",
                f11, f7, f3, f15);
    check(f7 < f11, "la hauteur descend d'abord");
    check(f15 > f7 * 1.2, "puis elle REMONTE : c'est la mort du MODE 4");

    // 4. Deux moities sur la meme source : rapport cyclique symetrique.
    {
        siren::AstableCore core; core.reset();
        const float dt = 1.0f / 192000.0f;
        for (int i = 0; i < 96000; ++i)
            core.step(dt, 470e3f, 9.0f, 470e3f, 9.0f, 0.0f, false, 12.2f);
        double sum = 0.0; int n = 0;
        float lo = 1e9f, hi = -1e9f;
        std::vector<float> y;
        for (int i = 0; i < 96000; ++i)
        { core.step(dt, 470e3f, 9.0f, 470e3f, 9.0f, 0.0f, false, 12.2f);
          y.push_back(core.out()); lo = std::min(lo, y.back()); hi = std::max(hi, y.back()); }
        const float mid = 0.5f * (lo + hi);
        for (float v : y) { sum += (v > mid) ? 1.0 : 0.0; ++n; }
        const double duty = sum / n;
        std::printf("      rapport cyclique a Vm identique : %.3f\n", duty);
        check(std::fabs(duty - 0.5) < 0.03, "meme source des deux cotes -> 50 %");
    }

    // La branche D1/D2 (MODE 3, TONE enfonce) est faite de VRAIES diodes, et
    // c'est une diode qui decide quand elle conduit — pas un seuil ecrit.
    //  - net 86 a 12,2 V : polarisee en direct tout le cycle, elle tire les
    //    deux bases par 220k en plus de l'echelle, la frequence monte ;
    //  - net 86 a 0 V : elle conduit ENCORE, mais seulement pendant l'excursion
    //    negative de la base (anode a 0, cathode a -11 V). C'est correct, et
    //    c'est ce que l'ancienne forme (220k vers la masse en permanence des que
    //    V86 < 0,7 V) rendait mal : la frequence doit tomber ENTRE les deux ;
    //  - net 86 a -12,2 V, sous la base la plus negative : jamais en direct,
    //    branche OUVERTE, frequence identique a celle sans branche.
    // La premiere version de ce test attendait « ouverte » a 0 V : c'etait le
    // test qui oubliait que la base descend a -11 V, pas le noyau.
    {
        auto freq = [](float v86, bool on)
        {
            siren::AstableCore c; c.reset();
            const double dt = 1.0 / 192000.0; int work;
            std::vector<float> y;
            for (int i = 0; i < (int) (0.6 / dt); ++i)
            {
                const double t = i * dt;
                const float vcc = 12.2f * (float) std::min(1.0, t / 2e-3);
                c.step((float) dt, 470e3f, vcc, 470e3f, vcc, v86 * (vcc / 12.2f), on, vcc, 1 << 30, &work);
                if (t > 0.3) y.push_back(c.out());
            }
            float lo = 1e9f, hi = -1e9f;
            for (float v : y) { lo = std::min(lo, v); hi = std::max(hi, v); }
            if (hi - lo < 1.0f) return 0.0;
            const float m = 0.5f * (lo + hi), amp = 0.2f * (hi - lo);
            int st = 0, zc = 0;
            for (float v : y) { if (st <= 0 && v > m + amp) { ++zc; st = 1; }
                                else if (st >= 0 && v < m - amp) st = -1; }
            return zc / 0.3;
        };
        const double f0   = freq(0.0f, false),  fFwd = freq(12.2f, true);
        const double fMid = freq(0.0f, true),   fRev = freq(-12.2f, true);
        std::printf("      D1/D2 : sans %.0f Hz, direct %.0f Hz, anode a 0 V %.0f Hz, inverse %.0f Hz\n",
                    f0, fFwd, fMid, fRev);
        check(fFwd > f0 * 1.15, "D1/D2 en direct : la branche tire les bases, la frequence monte");
        check(fMid > f0 * 1.05 && fMid < fFwd * 0.95,
              "D1/D2 anode a 0 V : conduit pendant l'excursion negative seulement (entre les deux)");
        check(std::fabs(fRev - f0) < 0.01 * f0, "D1/D2 en inverse : branche OUVERTE, frequence inchangee");
    }

    // reset() rend un noyau NEUF : la sortie apres un reset est, bit a bit,
    // celle d'un noyau qui n'a jamais tourne. Avant, le predicteur et la
    // fenetre du regime lent gardaient l'etat d'avant le reset.
    {
        const float dt = 1.0f / 192000.0f;
        siren::AstableCore used; used.reset();
        for (int i = 0; i < 60000; ++i)
            used.step(dt, 470e3f, 1.5f, 470e3f, 1.5f, 0.0f, false, 12.2f);   // en pleine mort
        used.reset();
        siren::AstableCore fresh; fresh.reset();
        bool same = true;
        for (int i = 0; i < 20000; ++i)
        {
            const float vcc = 12.2f * std::min(1.0f, i * dt / 2e-3f);
            used.step(dt, 470e3f, vcc, 470e3f, vcc, 0.0f, false, vcc);
            fresh.step(dt, 470e3f, vcc, 470e3f, vcc, 0.0f, false, vcc);
            same = same && used.outAvg() == fresh.outAvg() && used.out() == fresh.out();
        }
        check(same, "apres reset(), la sortie est celle d'un noyau neuf, bit a bit");
    }

    // Une entree non finie ne gele pas le noyau : il repart de la carte
    // eteinte et oscille a nouveau des que les entrees redeviennent saines.
    {
        const float dt = 1.0f / 192000.0f;
        siren::AstableCore c; c.reset();
        for (int i = 0; i < 20000; ++i) c.step(dt, 470e3f, 9.0f, 470e3f, 9.0f, 0.0f, false, 12.2f);
        c.step(dt, 470e3f, NAN, 470e3f, 9.0f, 0.0f, false, 12.2f);
        c.step(dt, 0.0f, 9.0f, 470e3f, 9.0f, 0.0f, false, 12.2f);
        bool finite = true; float lo = 1e9f, hi = -1e9f;
        for (int i = 0; i < 60000; ++i)
        {
            c.step(dt, 470e3f, 9.0f, 470e3f, 9.0f, 0.0f, false, 12.2f);
            const float y = c.out();
            finite = finite && std::isfinite(y);
            if (i > 30000) { lo = std::min(lo, y); hi = std::max(hi, y); }
        }
        check(finite, "entree NaN / echelle a 0 ohm : la sortie reste finie");
        check(hi - lo > 1.0f, "et l'astable oscille a nouveau");
    }

    std::printf(failures == 0 ? "ALL OK\n" : "%d FAILURE(S)\n", failures);
    return failures == 0 ? 0 : 1;
}
