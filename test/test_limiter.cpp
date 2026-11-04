// SPDX-License-Identifier: PolyForm-Shield-1.0.0
// Le limiteur brickwall. Un seul test compte vraiment : AUCUN echantillon ne
// depasse le plafond, jamais. Les autres verifient qu'il ne fait pas de degats
// en chemin.
#include "Limiter.hpp"
#include <cstdio>
#include <cmath>
#include <vector>

static int failures = 0;
static void check(bool ok, const char* m)
{
    if (!ok) { std::printf("ECHEC : %s\n", m); ++failures; }
}

int main()
{
    const float fs = 48000.f;

    // --- LE test : rien ne depasse, sur tout ce qu'on peut lui jeter -------
    {
        siren::Limiter l;
        l.prepare(fs);
        float worst = 0.f;

        // Echelon brutal au double du plein echelle, sinus a 2,0, impulsions
        // isolees a 3,5, passages sous le plafond : tout ce qui peut prendre
        // un lissage en defaut. (Pas de bruit : la version precedente de ce
        // commentaire en annoncait, aucun n'est genere — et il n'en faut pas,
        // ce sont les transitoires RAIDES qui mettent le filet a l'epreuve,
        // pas un signal dense.)
        for (int i = 0; i < (int) (10.f * fs); ++i)
        {
            float x = 0.f;
            const int phase = (i / (int) (0.5f * fs)) % 5;
            switch (phase)
            {
            case 0: x = 0.f; break;
            case 1: x = 2.0f; break;                                   // echelon
            case 2: x = 2.0f * std::sin(0.31f * (float) i); break;     // sinus fort
            case 3: x = ((i % 977) == 0) ? 3.5f : 0.f; break;          // impulsions
            default: x = 0.9f * std::sin(0.002f * (float) i); break;   // sous le plafond
            }
            const float y = l.process(x);
            check(std::isfinite(y), "ni NaN ni Inf");
            if (failures) break;
            worst = std::max(worst, std::fabs(y));
        }
        // Borne HAUTE et borne BASSE. Sans la seconde, un limiteur qui
        // sortirait du silence passerait ce test sans rien prouver.
        check(worst <= siren::Limiter::kCeiling + 1e-6f,
              "AUCUN echantillon ne depasse le plafond");
        check(worst > 0.9f,
              "et il en sort quelque chose : le signal atteint le plafond");
    }

    // --- Sous le plafond : gain exactement 1, et le signal ressort decale --
    {
        siren::Limiter l;
        l.prepare(fs);
        const int L = l.latencySamples();
        check(L > 0, "l'anticipation n'est pas nulle");

        std::vector<float> in((size_t) (2 * L + 4000)), out(in.size());
        for (size_t i = 0; i < in.size(); ++i)
            in[i] = 0.5f * std::sin(0.023f * (float) i);
        for (size_t i = 0; i < in.size(); ++i) out[i] = l.process(in[i]);

        // Tout reste sous le plafond, donc le gain doit valoir EXACTEMENT 1 :
        // pas 0,999. Sinon le sec ne traverserait pas intact et les tests
        // « bit a bit » de la chaine tomberaient.
        for (size_t i = (size_t) L; i < in.size(); ++i)
        {
            check(out[i] == in[i - (size_t) L], "sous le plafond : bit a bit, decale de L");
            if (failures) break;
        }
    }

    // --- La latence annoncee est la latence REELLE -------------------------
    {
        siren::Limiter l;
        l.prepare(fs);
        const int L = l.latencySamples();

        // Mesuree par correlation d'une impulsion, PAS relue de la constante :
        // un test qui recopie la valeur qu'il verifie ne verifie rien.
        std::vector<float> y((size_t) (4 * L + 64), 0.f);
        y[0] = l.process(0.25f);
        for (size_t i = 1; i < y.size(); ++i) y[i] = l.process(0.f);

        int peak = 0; float best = 0.f;
        for (size_t i = 0; i < y.size(); ++i)
            if (std::fabs(y[i]) > best) { best = std::fabs(y[i]); peak = (int) i; }
        check(peak == L, "l'impulsion ressort exactement L echantillons plus tard");
    }

    // --- Le relachement remonte, il ne reste pas ecrase -------------------
    {
        siren::Limiter l;
        l.prepare(fs);
        for (int i = 0; i < (int) (0.2f * fs); ++i) l.process(3.f);   // ecrase
        float last = 0.f;
        for (int i = 0; i < (int) (2.f * fs); ++i) last = l.process(0.4f);
        check(std::fabs(last - 0.4f) < 1e-3f, "apres 2 s de calme, le gain est revenu a 1");
    }

    // --- Le gain revient EXACTEMENT a 1 APRES avoir ete ecrase ------------
    //
    // Le bloc « sous le plafond » plus haut part d'une instance vierge, dont
    // env_ vaut 1,0 et n'a jamais bouge : il ne peut donc pas voir le defaut
    // que celui-ci vise. Il faut ECRASER le limiteur d'abord — ce qui arrive
    // des la premiere superposition de reprises — puis le laisser relacher.
    //
    // En float32, l'increment (wmin - env_) * aRel_ cesse d'etre representable
    // des que 1 - env_ < 1,43e-4 : l'enveloppe se figeait la, et le limiteur
    // appliquait definitivement 0,999857 (-0,00124 dB). Inaudible, mais le
    // sec ne traversait plus intact et l'invariant « exactement 1,0 » etait
    // faux. Mesure avant correctif : bit-a-bit NON, gain 0,999856890.
    //
    // 5 s de relachement = 50 constantes de temps : bien au-dela de ce qu'il
    // faut, et le test resterait vrai avec beaucoup moins.
    {
        siren::Limiter l;
        l.prepare(fs);
        for (int i = 0; i < (int) (0.5f * fs); ++i) l.process(3.f);   // ecrase
        for (int i = 0; i < (int) (5.0f * fs); ++i) l.process(0.f);   // relache

        const int L = l.latencySamples();
        std::vector<float> in((size_t) (2 * L + 4000)), out(in.size());
        for (size_t i = 0; i < in.size(); ++i)
            in[i] = 0.5f * std::sin(0.023f * (float) i);
        for (size_t i = 0; i < in.size(); ++i) out[i] = l.process(in[i]);

        for (size_t i = (size_t) L; i < in.size(); ++i)
        {
            check(out[i] == in[i - (size_t) L],
                  "apres ecrasement puis relachement : bit a bit, comme a neuf");
            if (failures) break;
        }
    }

    // --- NaN et Inf en entree : rien de non fini ne sort, jamais ------------
    // Dernier rempart avant l'hote. Avant la garde, un NaN passait tel quel
    // (gInst = 1 sur NaN) et un Inf sortait en NaN (plafond / Inf = 0, puis
    // Inf * 0).
    {
        siren::Limiter l;
        l.prepare(fs);
        const int L = l.latencySamples();
        std::vector<float> in((size_t) (0.5f * fs));
        for (size_t i = 0; i < in.size(); ++i) in[i] = 0.5f * std::sin(0.02f * (float) i);
        in[100] = NAN; in[200] = INFINITY; in[300] = -INFINITY;
        bool finite = true;
        std::vector<float> out(in.size());
        for (size_t i = 0; i < in.size(); ++i)
        { out[i] = l.process(in[i]); finite = finite && std::isfinite(out[i]); }
        check(finite, "NaN/Inf en entree : toutes les sorties sont finies");
        // Et apres : bit a bit sous le plafond, comme a neuf.
        for (size_t i = 400 + (size_t) L; i < in.size(); ++i)
        {
            check(out[i] == in[i - (size_t) L], "apres un NaN : bit a bit sous le plafond");
            if (failures) break;
        }
    }

    if (failures == 0) std::printf("test_limiter : OK\n");
    return failures ? 1 : 0;
}
