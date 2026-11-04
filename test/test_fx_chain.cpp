// SPDX-License-Identifier: PolyForm-Shield-1.0.0
// La chaine entiere. Ce qui se teste ici et pas dans les trois blocs pris
// separement : le repos, la latence constante, et l'hypothese mono.
#include "FxChain.hpp"
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
    const int   N  = 512;

    // --- Au repos, la chaine ne coute rien au son -------------------------
    {
        siren::FxChain fx;
        fx.prepare(fs);
        fx.setParameters({});   // echo coupe, dosage 0 : les defauts

        const int L = fx.latencySamples();
        std::vector<float> src((size_t) (N * 40));
        for (size_t i = 0; i < src.size(); ++i)
            src[i] = 0.4f * std::sin(0.017f * (float) i);

        std::vector<float> l(src), r(src);
        for (size_t b = 0; b + (size_t) N <= src.size(); b += (size_t) N)
        {
            float* ch[2] = { l.data() + b, r.data() + b };
            fx.process(ch, 2, N);
        }

        for (size_t i = (size_t) L; i < src.size(); ++i)
        {
            check(l[i] == src[i - (size_t) L],
                  "echo coupe et dosage 0 : bit a bit, au decalage pres");
            if (failures) break;
        }
        check(!failures, "le panneau au repos ne touche pas au son");
    }

    // --- Le mono est une VRAIE economie, pas une copie deguisee -----------
    //
    // Deux versions precedentes de ce bloc ne prouvaient pas ce qu'elles
    // annoncaient, et il vaut mieux l'ecrire que de le redecouvrir :
    //
    //  - comparer l[i] a r[i] apres process est une tautologie : process ecrit
    //    le canal 0 puis le recopie, l'egalite est vraie par construction ;
    //  - lancer une seconde instance sur le canal droit en affirmant que ses
    //    etats internes « divergeraient des le premier echantillon » est faux.
    //    Deux instances fraiches, memes parametres, MEME entree, sont
    //    deterministes et donnent le meme resultat. Ce bloc-la ne prouvait que
    //    le determinisme.
    //
    // Ce qui discrimine, c'est de donner aux deux canaux DES CONTENUS
    // DIFFERENTS. Une seconde chaine sur le canal droit traiterait son propre
    // signal et rendrait autre chose que le canal gauche ; la copie, elle,
    // ecrase ce contenu par le resultat du canal 0. Et on verifie au passage
    // que ce resultat est bien celui d'une chaine mono de reference.
    {
        siren::FxChain mono, stereo;
        mono.prepare(fs);
        stereo.prepare(fs);
        const siren::FxChain::Params p{ true, 0.6f, 180.f, 0.7f };
        mono.setParameters(p);
        stereo.setParameters(p);

        std::vector<float> m((size_t) N), l((size_t) N), r((size_t) N);
        for (int b = 0; b < 60; ++b)
        {
            for (int i = 0; i < N; ++i)
            {
                const float t = (float) (b * N + i);
                m[(size_t) i] = l[(size_t) i] = 0.5f * std::sin(0.02f * t);
                r[(size_t) i] = 0.3f * std::cos(0.031f * t);   // AUTRE CHOSE
            }

            float* one[1] = { m.data() };
            float* two[2] = { l.data(), r.data() };
            mono.process(one, 1, N);
            stereo.process(two, 2, N);

            for (int i = 0; i < N; ++i)
            {
                check(l[(size_t) i] == m[(size_t) i],
                      "le canal gauche est le resultat mono, bit a bit");
                check(r[(size_t) i] == l[(size_t) i],
                      "le canal droit est une COPIE du gauche, pas une seconde "
                      "chaine lancee sur son propre contenu");
            }
            if (failures) break;
        }
    }

    // --- La latence ne depend d'AUCUN parametre ---------------------------
    {
        siren::FxChain fx;
        fx.prepare(fs);
        const int base = fx.latencySamples();
        for (bool on : { false, true })
            for (float f : { 0.f, 0.5f, 1.f })
                for (float t : { 40.f, 300.f, 800.f })
                    for (float m : { 0.f, 0.5f, 1.f })
                    {
                        fx.setParameters({ on, f, t, m });
                        check(fx.latencySamples() == base,
                              "la latence est constante quels que soient les reglages");
                    }
        check(base > 0, "elle n'est pas nulle : le limiteur anticipe");
    }

    // --- Le plafond tient au bout de la chaine ----------------------------
    //
    // « Rien ne depasse, quoi qu'on regle » etait plus categorique que ce qui
    // est ecrit dessous : la borne tolere kCeiling + 1e-6, parce que l'arrondi
    // du produit final du limiteur coute un ULP (voir le titre de
    // Limiter.hpp). Et un reglage, un seul, est essaye ici — pas « quoi qu'on
    // regle ».
    {
        siren::FxChain fx;
        fx.prepare(fs);
        fx.setParameters({ true, 1.f, 90.f, 1.f });
        std::vector<float> l((size_t) N), r((size_t) N);
        float worst = 0.f;
        for (int b = 0; b < 2000; ++b)
        {
            for (int i = 0; i < N; ++i)
                l[(size_t) i] = r[(size_t) i] = 0.55f * std::sin(0.09f * (float) (b * N + i));
            float* ch[2] = { l.data(), r.data() };
            fx.process(ch, 2, N);
            for (int i = 0; i < N; ++i)
                worst = std::max(worst, std::fabs(l[(size_t) i]));
        }
        // Borne HAUTE et borne BASSE. Sans la seconde, une chaine qui
        // sortirait du silence passerait ce test sans rien prouver.
        check(worst <= siren::Limiter::kCeiling + 1e-6f, "le plafond tient au bout de la chaine");
        check(worst > 0.5f, "et il sort reellement du signal, pas du silence");
    }

    if (failures == 0) std::printf("test_fx_chain : OK\n");
    return failures ? 1 : 0;
}
