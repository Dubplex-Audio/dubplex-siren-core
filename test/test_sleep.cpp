// SPDX-License-Identifier: PolyForm-Shield-1.0.0
// L'arret du solveur, carte eteinte et dechargee. Une seule chose a tenir : il ne
// change RIEN a ce qu'on entend, et surtout pas la decharge, qui doit rester resolue
// jusqu'au bout. Chaque scenario tourne deux fois, avec et sans l'arret : les sorties
// doivent etre identiques au bit pres.
#include "SirenEngine.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

static int failures = 0;
static void check(bool ok, const char* m)
{ std::printf("%s%s\n", ok ? "ok:   " : "FAIL: ", m); if (!ok) ++failures; }

using siren::SirenEngine;
static constexpr double kFs = 48000.0;
static constexpr int    kBlock = 64;

struct Seg { bool power; double secs; };
struct Out { std::vector<float> y; double sleepAt = -1.0; bool wokeUp = false; };

static SirenEngine::Params params(bool power, bool sleep)
{
    SirenEngine::Params p; p.power = power; p.tone = 1; p.mode = 0; p.speed = 1;
    p.oversample = 2; p.sleepWhenSettled = sleep;
    return p;
}

static Out render(const std::vector<Seg>& segs, bool sleep)
{
    static SirenEngine e;
    e = SirenEngine(); e.prepare(kFs, kBlock);
    Out o; float buf[kBlock]; float* b[1] = { buf }; double t = 0.0;
    for (const Seg& s : segs)
    {
        e.setParameters(params(s.power, sleep));
        const int n = (int) (s.secs * kFs / kBlock);
        for (int i = 0; i < n; ++i)
        {
            for (float& x : buf) x = 0.0f;
            const bool was = e.asleep();
            e.process(b, 1, kBlock);
            o.y.insert(o.y.end(), buf, buf + kBlock);
            t += kBlock / kFs;
            if (!was && e.asleep() && o.sleepAt < 0.0) o.sleepAt = t;
            if (was && !e.asleep()) o.wokeUp = true;
        }
    }
    return o;
}

int main()
{
    {
        const Out o = render({ { false, 0.5 } }, true);
        bool zero = true; for (float x : o.y) if (x != 0.0f) zero = false;
        check(o.sleepAt > 0.0 && o.sleepAt < 0.01, "jamais alimentee : a l'arret des le premier bloc");
        check(zero, "jamais alimentee : zero exact");
    }
    {
        // Coupure apres une seconde de jeu : la decharge doit aller a son terme.
        const double kWake = 1.0 + 60.0;
        const std::vector<Seg> segs = { { true, 1.0 }, { false, 60.0 }, { true, 2.0 } };
        const Out a = render(segs, true), b = render(segs, false);
        std::printf("      arret %.1f s apres la coupure\n", a.sleepAt - 1.0);
        check(a.sleepAt > 0.0, "la carte finit par s'arreter");
        check(a.sleepAt - 1.0 > 8.0, "pas avant la fin de la decharge");
        check(a.wokeUp, "elle repart quand le rail revient");
        check(a.y.size() == b.y.size(), "meme duree");
        const size_t nSleep = (size_t) ((a.sleepAt - kBlock / kFs) * kFs);
        const size_t nWake  = (size_t) (kWake * kFs / kBlock) * kBlock;
        bool same = true; for (size_t i = 0; i < nSleep; ++i) if (a.y[i] != b.y[i]) same = false;
        check(same, "toute la decharge, jusqu'a l'arret : identique au bit pres");
        double worst = 0.0; for (size_t i = nSleep; i < nWake; ++i) worst = std::max(worst, (double) std::fabs(b.y[i]));
        std::printf("      ce que l'arret retire : %.1e, soit %.0f dBFS\n", worst, 20.0 * std::log10(worst + 1e-30));
        check(worst < 1e-8, "ce que l'arret retire est sous -160 dBFS");
        // La reprise : memes hauteur et niveau. Pas au bit pres, le solveur repart
        // d'un etat qui differe de 1e-7 V, et une periode de sirene ne s'y aligne pas.
        double ra = 0.0, rb = 0.0; int ca = 0, cb = 0; const size_t from = nWake + (size_t) (0.5 * kFs);
        for (size_t i = from; i < a.y.size(); ++i)
        {
            ra += a.y[i] * a.y[i]; rb += b.y[i] * b.y[i];
            ca += (a.y[i] >= 0.0f) != (a.y[i - 1] >= 0.0f); cb += (b.y[i] >= 0.0f) != (b.y[i - 1] >= 0.0f);
        }
        const double dB = 10.0 * std::log10((ra + 1e-30) / (rb + 1e-30));
        std::printf("      reprise : %+.3f dB, %d passages par zero contre %d\n", dB, ca, cb);
        check(std::fabs(dB) < 0.1, "reprise : meme niveau");
        check(std::abs(ca - cb) <= cb / 100, "reprise : meme hauteur");
    }
    {
        // Remise sous tension EN PLEINE decharge : le solveur n'a pas le droit
        // d'avoir oublie ce qui reste dans C4.
        const std::vector<Seg> segs = { { true, 1.0 }, { false, 2.0 }, { true, 1.0 }, { false, 10.0 }, { true, 1.0 } };
        const Out a = render(segs, true), b = render(segs, false);
        check(a.sleepAt < 0.0, "pas d'arret pendant une decharge");
        check(a.y == b.y, "reprises en pleine decharge : identiques au bit pres");
    }
    if (failures) { std::printf("%d ECHEC(S)\n", failures); return 1; }
    std::printf("tout passe\n");
    return 0;
}
