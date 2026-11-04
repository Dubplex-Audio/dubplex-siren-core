// SPDX-License-Identifier: PolyForm-Shield-1.0.0
// Le garde-fou qui suit la machine. Deux choses a tenir en meme temps :
//   - au repos, RIEN ne change : la sortie est bit a bit celle du budget fixe,
//     meme quand une preemption simulee frappe un bloc sur dix ;
//   - chargee, le budget se resserre au cout reel, et plus aucun bloc ne
//     depasse son echeance — la ou le budget fixe en laissait passer.
// La machine ne se charge pas de facon reproductible : le cout est injecte.
#include "SirenEngine.hpp"
#include "MachineFactor.hpp"
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

using siren::SirenEngine;
using siren::MachineFactor;
static constexpr double kFs = 48000.0;
static constexpr int    kBlock = 256;
static constexpr double kBlockS = kBlock / kFs;
static constexpr double kC = 0.20e-6;   // la constante, s par assemblee

// Pousse `secs` de blocs a un rapport cout/constante donne, avec une pointe
// toutes les `every` fenetres (0 = jamais).
static void feed(MachineFactor& m, double secs, double ratio, double spikeRatio = 0.0, int every = 0)
{
    const int n = (int) (secs / kBlockS);
    for (int b = 0; b < n; ++b)
    {
        const double r = (every > 0 && b % every == 0) ? spikeRatio : ratio;
        m.push(r * kC, kC, kBlockS);
    }
}

// Le sifflement du MODE 4 a SPEED OFF, TONE 4 : le regime le plus gourmand
// (~75 assemblees par image), celui qui decroche sur machine chargee.
static SirenEngine::Params whistle()
{
    SirenEngine::Params p; p.power = true; p.mode = 3; p.speed = 3; p.tone = 3;
    p.oversample = 4; p.sirenBtn = false;
    return p;
}

struct Render { std::vector<float> y; double energy = 0.0; };
static Render render(SirenEngine& eng, double secs)
{
    Render r; r.y.reserve((size_t) (secs * kFs));
    std::vector<float> bufL(kBlock), bufR(kBlock); float* bufs[2] = { bufL.data(), bufR.data() };
    for (int b = 0; b < (int) (secs * kFs / kBlock); ++b)
    {
        std::fill(bufL.begin(), bufL.end(), 0.0f); std::fill(bufR.begin(), bufR.end(), 0.0f);
        eng.process(bufs, 2, kBlock);
        for (float v : bufL) { r.y.push_back(v); r.energy += (double) v * v; }
    }
    return r;
}

int main()
{
    std::printf("-- MachineFactor, l'estimateur seul\n");
    {   // 1. Une machine plus rapide que la constante ne rend pas de budget.
        MachineFactor m; feed(m, 5.0, 0.75);
        near(m.factor(), 1.0, 1e-6, "au repos (0,15 us) : facteur 1, plancher");
    }
    {   // 2. Chargee : il suit, et vite.
        MachineFactor m; feed(m, 0.5, 2.25);
        check(m.factor() > 1.8, "chargee (0,45 us) : > 1,8 apres 0,5 s");
        feed(m, 2.0, 2.25);
        near(m.factor(), 2.25, 0.05, "chargee (0,45 us) : converge en 2,5 s");
    }
    {   // 3. La preemption ne compte pas : un bloc sur dix prend 25x (5 ms).
        MachineFactor m; feed(m, 5.0, 0.9, 25.0, 10);
        near(m.factor(), 1.0, 1e-6, "pointes 1/10 a 25x : facteur 1, la preemption est invisible");
    }
    {   // 4. Le repit se rend lentement.
        MachineFactor m; feed(m, 3.0, 2.25); feed(m, 1.0, 0.9);
        check(m.factor() > 1.5, "charge retombee depuis 1 s : encore > 1,5");
        feed(m, 10.0, 0.9);
        near(m.factor(), 1.0, 0.01, "charge retombee depuis 11 s : revenu a 1");
    }
    {   // 5. Borne haute.
        MachineFactor m; feed(m, 5.0, 10.0);
        near(m.factor(), MachineFactor::kMax, 1e-3, "machine 10x trop lente : plafonne a kMax");
    }

    std::printf("-- Le moteur, au repos\n");
    {   // 6. Budget fixe et budget suivi rendent le MEME son bit a bit sur une
        //    machine au repos, pointes de preemption comprises.
        SirenEngine fixe;  fixe.prepare(kFs, kBlock);
        SirenEngine suivi; suivi.prepare(kFs, kBlock);
        auto p = whistle(); fixe.setParameters(p);
        p.adaptiveBudget = true; suivi.setParameters(p);
        fixe.dbgFakeCost(0.15e-6);
        suivi.dbgFakeCost(0.15e-6, 5.0e-3, 10);   // une preemption de 5 ms tous les 10 blocs
        const auto a = render(fixe, 4.0), b = render(suivi, 4.0);
        check(a.y == b.y, "au repos : sortie identique bit a bit, budget fixe ou suivi");
        near(suivi.machineFactor(), 1.0, 1e-6, "au repos : facteur 1 malgre les pointes");
        check(suivi.load().blocksLate > 0, "au repos : les pointes comptent bien comme blocs en retard (horloge murale)");
        check(a.energy > 1.0, "au repos : ca siffle");
    }

    std::printf("-- Le moteur, machine chargee (0,55 us par assemblee, le haut de la mesure du 14/10)\n");
    {   // 7. Le budget fixe laisse l'hote decrocher ; le budget suivi non.
        SirenEngine fixe;  fixe.prepare(kFs, kBlock);
        SirenEngine suivi; suivi.prepare(kFs, kBlock);
        auto p = whistle(); fixe.setParameters(p);
        p.adaptiveBudget = true; suivi.setParameters(p);
        fixe.dbgFakeCost(0.55e-6); suivi.dbgFakeCost(0.55e-6);
        render(fixe, 3.0); render(suivi, 3.0);          // le sifflement s'installe, le facteur aussi
        near(suivi.machineFactor(), 2.75, 0.1, "chargee : facteur 2,75");
        near(suivi.asmCost() * 1e6, 0.55, 0.02, "chargee : cout effectif 0,55 us");
        fixe.resetLoad(); suivi.resetLoad();
        const auto a = render(fixe, 5.0), b = render(suivi, 5.0);
        const auto lf = fixe.load(), ls = suivi.load();
        std::printf("      budget fixe  : budget %d, pire %d, rognes %d/%d, en retard %d (pire %.0f %%)\n",
                    lf.budget, lf.worstWork, lf.blocksClipped, lf.blocksSeen, lf.blocksLate, 100.0 * lf.worstTime);
        std::printf("      budget suivi : budget %d, pire %d, rognes %d/%d, en retard %d (pire %.0f %%)\n",
                    ls.budget, ls.worstWork, ls.blocksClipped, ls.blocksSeen, ls.blocksLate, 100.0 * ls.worstTime);
        check(lf.blocksLate > 0, "chargee, budget fixe : des blocs depassent l'echeance (le defaut qu'on corrige)");
        check(ls.blocksLate == 0, "chargee, budget suivi : aucun bloc ne depasse l'echeance");
        check(ls.worstTime <= 1.0, "chargee, budget suivi : le pire bloc tient dans l'echeance");
        const int expected = (int) (kBlock * SIREN_TIME_SHARE / (kFs * suivi.asmCost()));
        check(std::abs(ls.budget - expected) <= kBlock, "chargee, budget suivi : le budget decoule du cout effectif");
        check(b.energy > 0.3 * a.energy, "chargee, budget suivi : ca siffle encore (blocs rognes, pas de silence)");
    }

    std::printf("-- Le facteur est independant du mode\n");
    {   // 8. En MODE 3 l'assemblee coute le double par construction : ce n'est
        //    pas une charge machine. Un cout mesure de 0,40 us y donne 1.
        SirenEngine eng; eng.prepare(kFs, kBlock);
        auto p = whistle(); p.mode = 2; p.adaptiveBudget = true; eng.setParameters(p);
        eng.dbgFakeCost(0.40e-6);
        render(eng, 3.0);
        near(eng.machineFactor(), 1.0, 1e-6, "MODE 3 a 0,40 us : facteur 1");
        p.mode = 3; eng.setParameters(p); eng.dbgFakeCost(0.20e-6);
        render(eng, 3.0);
        near(eng.machineFactor(), 1.0, 1e-6, "retour MODE 4 a 0,20 us : facteur 1");
    }

    std::printf(failures ? "%d FAIL\n" : "ALL OK\n", failures);
    return failures ? 1 : 0;
}
