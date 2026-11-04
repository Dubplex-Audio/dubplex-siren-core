// SPDX-License-Identifier: PolyForm-Shield-1.0.0
// L'echo a bande. Ces proprietes ne sont pas une question de gout : le
// retard tombe ou il doit, un premier reglage s'y pose (pas de glissement a
// l'ouverture d'une session) alors qu'un second glisse (le geste reste un
// geste), les reprises decroissent selon le feedback, coupe l'echo ne touche
// a rien, et a fond il ne diverge pas.
#include "TapeEcho.hpp"
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

    // --- Le retard tombe au bon echantillon -------------------------------
    for (float ms : { 40.f, 300.f, 800.f })
    {
        siren::TapeEcho e;
        e.prepare(fs);
        e.setParameters(true, 0.f, ms);
        // CETTE PRECHAUFFE EST INERTE, et le dire vaut mieux que la laisser
        // se faire lire de travers. Le setParameters ci-dessus est le PREMIER
        // apres prepare(), donc primed_ est faux et dSm_ est POSE sur la
        // cible, pas glisse vers elle : il n'y a rien a laisser converger, et
        // ce bloc mesure une pose, pas un glissement. Le glissement a son
        // propre bloc, plus bas.
        //
        // Elle reste quand meme, comme garde-fou : si la pose disparaissait,
        // c'est elle qui empecherait ce bloc-ci de mesurer le lissage au lieu
        // du retard. Elle etait dimensionnee pour ca — 0,7 s = ~11,7 tau, de
        // quoi ramener sous l'echantillon le pire saut teste ici (300 -> 800
        // ms), la ou 0,5 s laissait 5 a 6 echantillons de residu, au-dela de
        // la tolerance de +/-3.
        for (int i = 0; i < (int) (0.7f * fs); ++i) e.process(0.f);

        const int want = (int) std::lround(ms * 0.001f * fs);
        std::vector<float> y((size_t) want + 200, 0.f);
        y[0] = e.process(1.f);
        for (size_t i = 1; i < y.size(); ++i) y[i] = e.process(0.f);

        // Le pic du MOUILLE : on cherche apres l'echantillon 10 pour ignorer
        // le sec, qui sort tel quel a l'instant 0.
        int peak = 10; float best = 0.f;
        for (size_t i = 10; i < y.size(); ++i)
            if (std::fabs(y[i]) > best) { best = std::fabs(y[i]); peak = (int) i; }

        check(std::abs(peak - want) <= 3, "le retard tombe a +/- 3 echantillons");
        check(best > 0.05f, "la reprise est audible");
    }

    // --- Premier reglage apres prepare() : dSm_ se pose, il n'y glisse pas -
    // Le lissage de TIME modelise un GESTE (tourner le potard en jouant), pas
    // l'ouverture d'une session. Sans un premier reglage qui POSE la cible au
    // lieu d'y glisser, dSm_ partirait du defaut de construction (300 ms) et
    // mettrait des centaines de ms a atteindre la vraie cible — la bande
    // accelererait toute seule au chargement d'un preset, sans manipulation.
    // AUCUNE prechauffe ici : c'est tout le point du test.
    //
    // Cible a 40 ms, PAS 800 : avec une cible eloignee du defaut de 300 ms
    // dans le sens croissant, la fenetre de recherche du pic (want + 200)
    // est elle-meme assez longue pour laisser un glissement non corrige
    // converger avant qu'on l'observe — le test ne verrait alors aucune
    // difference entre pose immediate et glissement, quelle que soit
    // l'implementation. A 40 ms, la fenetre est courte (2120 echantillons)
    // et un dSm_ qui partirait de 14400 (300 ms) et glisserait vers 1920
    // n'a pas le temps de redescendre jusque-la : l'echo reste inaudible
    // dans la fenetre entiere si le reglage n'a pas ete pose immediatement.
    {
        siren::TapeEcho e;
        e.prepare(fs);
        e.setParameters(true, 0.f, 40.f);

        const int want = (int) std::lround(40.f * 0.001f * fs);
        std::vector<float> y((size_t) want + 200, 0.f);
        y[0] = e.process(1.f);
        for (size_t i = 1; i < y.size(); ++i) y[i] = e.process(0.f);

        int peak = 10; float best = 0.f;
        for (size_t i = 10; i < y.size(); ++i)
            if (std::fabs(y[i]) > best) { best = std::fabs(y[i]); peak = (int) i; }

        check(std::abs(peak - want) <= 3,
              "premier reglage apres prepare() : dSm_ se pose sur la cible, sans prechauffe");
        check(best > 0.05f, "la reprise est audible des le premier reglage, pas noyee dans un glissement");
    }

    // --- Un second reglage, lui, glisse toujours --------------------------
    // Si le fait de poser dSm_ au premier reglage se reproduisait a CHAQUE
    // reglage (drapeau jamais leve, ou remis a zero par erreur), TIME
    // sauterait en permanence et le balayage plus bas perdrait tout son sens.
    // On verifie l'inverse sur un cas ou saut et glissement sont
    // distinguables : premier reglage (saute a 40 ms), second reglage vers
    // 300 ms tire dans la foulee, impulsion immediate. Un saut placerait le
    // pic pile sur l'echantillon de 300 ms ; un glissement, parti de 40 ms,
    // ne l'a pas encore rattrape a cet instant-la et tombe nettement avant.
    {
        siren::TapeEcho e;
        e.prepare(fs);
        e.setParameters(true, 0.f, 40.f);    // premier : se pose a 1920 echantillons
        e.setParameters(true, 0.f, 300.f);   // second : doit glisser vers 14400

        const int want = (int) std::lround(300.f * 0.001f * fs);
        std::vector<float> y((size_t) want + 200, 0.f);
        y[0] = e.process(1.f);
        for (size_t i = 1; i < y.size(); ++i) y[i] = e.process(0.f);

        int peak = 10; float best = 0.f;
        for (size_t i = 10; i < y.size(); ++i)
            if (std::fabs(y[i]) > best) { best = std::fabs(y[i]); peak = (int) i; }

        check(peak < want - 20,
              "second reglage d'affilee : dSm_ glisse depuis le premier, il ne saute pas dessus");
    }

    // --- Les reprises decroissent selon le feedback ------------------------
    {
        siren::TapeEcho e;
        e.prepare(fs);
        e.setParameters(true, 0.5f, 100.f);
        for (int i = 0; i < (int) (0.5f * fs); ++i) e.process(0.f);

        const int d = (int) std::lround(100.f * 0.001f * fs);
        std::vector<float> y((size_t) d * 4, 0.f);
        y[0] = e.process(1.f);
        for (size_t i = 1; i < y.size(); ++i) y[i] = e.process(0.f);

        auto peakNear = [&](int centre) {
            float m = 0.f;
            for (int i = centre - 40; i < centre + 40; ++i)
                if (i > 0 && i < (int) y.size()) m = std::max(m, std::fabs(y[i]));
            return m;
        };
        const float r1 = peakNear(d), r2 = peakNear(2 * d), r3 = peakNear(3 * d);
        check(r1 > r2 && r2 > r3, "chaque reprise est plus faible que la precedente");
        check(r2 > 0.05f * r1, "elles ne s'effondrent pas d'un coup");
    }

    // --- Coupe, la sortie est le signal d'entree BIT A BIT -----------------
    {
        siren::TapeEcho e;
        e.prepare(fs);
        e.setParameters(false, 0.7f, 250.f);
        for (int i = 0; i < 20000; ++i)
        {
            const float x = std::sin(0.05f * (float) i) * 0.7f;
            check(e.process(x) == x, "coupe : la sortie EST l'entree, bit a bit");
            if (failures) break;   // ne pas noyer la sortie
        }
    }

    // --- Un NaN en entree ne reste pas dans la ligne ----------------------
    // Avant la garde : un seul NaN, et 100 % des sorties etaient non finies
    // cinq secondes plus tard — et encore apres coupure puis rallumage, la
    // ligne tournant meme coupee.
    {
        siren::TapeEcho e;
        e.prepare(fs);
        e.setParameters(true, 0.9f, 100.f);
        e.process(NAN); e.process(INFINITY); e.process(-INFINITY);
        bool finite = true;
        for (int i = 0; i < (int) (2.f * fs); ++i)
            finite = finite && std::isfinite(e.process(0.1f * std::sin(0.03f * (float) i)));
        check(finite, "NaN/Inf en entree : toutes les sorties suivantes sont finies");
        // Et l'echo vit encore : une impulsion a sa reprise.
        float best = 0.f;
        e.process(1.f);
        for (int i = 0; i < 6000; ++i) best = std::max(best, std::fabs(e.process(0.f)));
        check(best > 0.05f, "apres un NaN, l'echo repond toujours");
    }

    // --- L'interrupteur ne claque pas --------------------------------------
    // Couper net en pleine queue faisait un saut de 0,46 entre deux
    // echantillons (mesure) : le mouille disparaissait d'un coup. Il doit
    // s'eteindre en quelques ms, puis la sortie redevient l'entree bit a bit.
    {
        siren::TapeEcho e;
        e.prepare(fs);
        e.setParameters(true, 0.7f, 100.f);
        for (int i = 0; i < 20000; ++i) e.process(std::sin(0.05f * (float) i) * 0.7f);
        e.setParameters(false, 0.7f, 100.f);
        float prevWet = 0.f, worst = 0.f; bool first = true;
        int exact = 0;
        for (int i = 20000; i < 20000 + 24000; ++i)
        {
            const float x = std::sin(0.05f * (float) i) * 0.7f;
            const float y = e.process(x);
            const float wet = y - x;
            if (!first) worst = std::max(worst, std::fabs(wet - prevWet));
            prevWet = wet; first = false;
            if (i >= 20000 + 4800 && y == x) ++exact;   // 100 ms apres la coupure
        }
        check(worst < 0.1f, "coupure de l'echo : le mouille s'eteint sans saut");
        check(exact == 24000 - 4800, "100 ms apres la coupure : la sortie EST l'entree, bit a bit");
    }

    // --- Coupe, la bande s'arrete et s'efface ; rallume, il repart vide ----
    // Avant, la boucle tournait derriere le mute : rallume, l'echo ressortait
    // la queue d'avant la coupure (et, a feedback 1,0, l'auto-oscillation
    // qui avait continue dans le noir). Couper un appareil, c'est cesser de
    // l'alimenter ET vider ce qu'il tenait.
    {
        siren::TapeEcho e;
        e.prepare(fs);
        e.setParameters(true, 1.0f, 100.f);           // feedback a fond : la queue ne meurt pas seule
        for (int i = 0; i < 20000; ++i) e.process(std::sin(0.05f * (float) i) * 0.7f);
        e.setParameters(false, 1.0f, 100.f);
        for (int i = 0; i < 24000; ++i) e.process(0.f);   // 500 ms coupe, en silence

        // Rallume : dans le silence, la sortie reste le silence. Une queue
        // survivante se serait entendue ici, des les premiers echantillons.
        e.setParameters(true, 1.0f, 100.f);
        float leak = 0.f;
        for (int i = 0; i < 24000; ++i) leak = std::max(leak, std::fabs(e.process(0.f)));
        check(leak == 0.f, "rallume apres la coupure : rien ne ressort, la bande etait vide");

        // Et il fonctionne : une impulsion ressort TIME plus tard, comme neuve.
        const int want = (int) (0.100f * fs);
        int peak = -1; float best = 0.f;
        for (int i = 0; i < 2 * want; ++i)
        {
            const float y = e.process(i == 0 ? 1.f : 0.f);
            if (i > 8 && std::fabs(y) > best) { best = std::fabs(y); peak = i; }
        }
        check(std::abs(peak - want) <= 3, "et la premiere reprise arrive TIME plus tard, comme sur un echo neuf");
        check(best > 0.05f, "audible");
    }

    // --- Coupe, TIME se pose sans glisser au rallumage ---------------------
    // Tourner TIME pendant la coupure n'est pas un geste qu'on entend ; au
    // rallumage la bande ne doit pas accelerer toute seule.
    {
        siren::TapeEcho e;
        e.prepare(fs);
        e.setParameters(true, 0.5f, 100.f);
        for (int i = 0; i < 9600; ++i) e.process(0.f);
        e.setParameters(false, 0.5f, 100.f);
        for (int i = 0; i < 2400; ++i) e.process(0.f);     // 50 ms : fondu fini, bande a l'arret
        e.setParameters(false, 0.5f, 400.f);                // TIME tourne pendant la coupure
        for (int i = 0; i < 2400; ++i) e.process(0.f);
        e.setParameters(true, 0.5f, 400.f);
        const int want = (int) (0.400f * fs);
        int peak = -1; float best = 0.f;
        for (int i = 0; i < want + 4800; ++i)
        {
            const float y = e.process(i == 0 ? 1.f : 0.f);
            if (i > 8 && std::fabs(y) > best) { best = std::fabs(y); peak = i; }
        }
        check(std::abs(peak - want) <= 3, "rallume, la reprise tombe au TIME regle pendant la coupure, sans glissement");
    }

    // --- Feedback a fond : borne, et sans NaN ------------------------------
    {
        siren::TapeEcho e;
        e.prepare(fs);
        e.setParameters(true, 1.f, 120.f);
        float worst = 0.f;
        for (int i = 0; i < (int) (60.f * fs); ++i)
        {
            const float x = (i < 4800) ? std::sin(0.07f * (float) i) : 0.f;
            const float y = e.process(x);
            check(std::isfinite(y), "ni NaN ni Inf a feedback 1,0");
            if (failures) break;
            worst = std::max(worst, std::fabs(y));
        }
        check(worst < 4.f, "auto-oscillation bornee par le tanh, pas divergente");
    }

    // --- TIME qui balaie ne produit pas de saut ---------------------------
    {
        siren::TapeEcho e;
        e.prepare(fs);
        e.setParameters(true, 0.4f, 40.f);
        for (int i = 0; i < (int) (0.5f * fs); ++i) e.process(0.5f);

        float prev = 0.f, jump = 0.f;
        for (int i = 0; i < (int) (4.f * fs); ++i)
        {
            const float t = 40.f + 760.f * (float) i / (4.f * fs);
            e.setParameters(true, 0.4f, t);
            const float y = e.process(std::sin(0.03f * (float) i) * 0.5f);
            if (i > 100) jump = std::max(jump, std::fabs(y - prev));
            prev = y;
        }
        check(jump < 0.5f, "le balayage de TIME glisse, il ne saute pas");
    }

    if (failures == 0) std::printf("test_tape_echo : OK\n");
    return failures ? 1 : 0;
}
