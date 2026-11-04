// SPDX-License-Identifier: PolyForm-Shield-1.0.0
// La reverbe a ressort. On ne teste pas « est-ce que ca sonne bien » — ca ne
// se teste pas. On teste ce qui se mesure : dosage a zero, queue perimee,
// decroissance, et pas de divergence.
#include "Spring.hpp"
#include <cstdio>
#include <algorithm>
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

    // --- Dosage a zero : la sortie EST l'entree, bit a bit -----------------
    {
        siren::Spring s;
        s.prepare(fs);
        s.setParameters(0.f);
        for (int i = 0; i < 20000; ++i)
        {
            const float x = std::sin(0.041f * (float) i) * 0.6f;
            check(s.process(x) == x, "dosage 0 : sortie == entree, bit a bit");
            if (failures) break;
        }
    }

    // --- Une queue PERIMEE ne ressuscite pas ------------------------------
    //
    // Le court-circuit a dosage nul fige l'etat du reseau : sans vidage a la
    // remontee, les retards gardent intact ce qu'ils contenaient a la
    // fermeture du potard, et le rouvrir dix secondes plus tard relache cet
    // audio-la. Le geste teste ici est celui d'un utilisateur, pas un cas
    // limite : on baisse REVERB, on joue, on remonte.
    //
    // Sans le vidage, mesure a 48 kHz : pic 0,0172 (-35 dBFS), energie 0,104.
    // Avec, la sortie est exactement nulle — l'entree l'est aussi. La borne a
    // 1e-5 (-100 dBFS) laisse donc quatre ordres de grandeur de marge et reste
    // trois ordres sous le defaut.
    {
        siren::Spring s;
        s.prepare(fs);
        s.setParameters(1.f);
        s.process(1.f);
        for (int i = 0; i < (int) (0.2f * fs); ++i) s.process(0.f);   // une queue bien remplie

        s.setParameters(0.f);
        for (int i = 0; i < (int) (10.f * fs); ++i) s.process(0.f);   // dix secondes potard ferme

        s.setParameters(1.f);
        float energy = 0.f, peak = 0.f;
        for (int i = 0; i < (int) fs; ++i)
        {
            const float y = s.process(0.f);
            energy += y * y;
            peak = std::max(peak, std::fabs(y));
        }
        check(peak   < 1e-5f, "dix secondes a dosage 0 : aucune queue perimee au retour");
        check(energy < 1e-8f, "et rien non plus en energie cumulee sur la seconde suivante");
    }

    // --- Un NaN en entree n'entre pas dans le reseau ----------------------
    // Avant la garde : 47 000 sorties non finies sur la seconde suivante.
    {
        siren::Spring s;
        s.prepare(fs);
        s.setParameters(0.8f);
        s.process(NAN); s.process(INFINITY);
        bool finite = true;
        for (int i = 0; i < (int) (2.f * fs); ++i)
            finite = finite && std::isfinite(s.process(0.1f * std::sin(0.03f * (float) i)));
        check(finite, "NaN/Inf en entree : toutes les sorties suivantes sont finies");
        float energy = 0.f;
        s.process(1.f);
        for (int i = 0; i < (int) fs; ++i) { const float y = s.process(0.f); energy += y * y; }
        check(energy > 1e-4f, "apres un NaN, la reverbe repond toujours");
    }

    // --- Le dosage glisse, il ne saute pas ----------------------------------
    // Baisser REVERB a zero en pleine queue : la queue s'eteint en quelques
    // ms (elle ne disparait pas entre deux echantillons), puis la sortie est
    // l'entree bit a bit — le court-circuit est repris une fois le lisse
    // retombe a zero.
    {
        siren::Spring s;
        s.prepare(fs);
        s.setParameters(1.f);
        s.process(1.f);
        for (int i = 0; i < (int) (0.1f * fs); ++i) s.process(0.f);
        s.setParameters(0.f);
        float tail = 0.f;
        for (int i = 0; i < 48; ++i) tail += std::fabs(s.process(0.f));   // la premiere ms
        check(tail > 0.f, "dosage ramene a 0 : la queue est encore la pendant la premiere ms");
        for (int i = 0; i < (int) (0.1f * fs); ++i) s.process(0.f);
        int exact = 0;
        for (int i = 0; i < 1000; ++i)
        {
            const float x = std::sin(0.041f * (float) i) * 0.6f;
            if (s.process(x) == x) ++exact;
        }
        check(exact == 1000, "100 ms apres : la sortie EST l'entree, bit a bit");
    }

    // --- Il se passe quelque chose des que le dosage monte ----------------
    {
        siren::Spring s;
        s.prepare(fs);
        s.setParameters(1.f);
        float energy = 0.f;
        s.process(1.f);
        for (int i = 0; i < (int) fs; ++i)
        {
            const float y = s.process(0.f);
            energy += y * y;
        }
        check(energy > 1e-4f, "une impulsion laisse une queue");
    }

    // --- LA DENSITE D'ECHOS ----------------------------------------------
    //
    // LE TEST QUI MANQUAIT. La premiere version de Spring.hpp avait quatre
    // lignes nues de 23,7 a 53,9 ms : quatre reflexions dans les 54 premieres
    // millisecondes, puis une enveloppe qui gardait la periodicite de la plus
    // courte. On les entendait une par une — « on entend les delays ». Et TOUT
    // passait : le dosage a zero, la queue perimee, la decroissance, le T30, la
    // non-divergence. Aucune de ces bornes ne regarde la STRUCTURE de la queue,
    // qui est pourtant tout ce qui separe une reverbe d'une ligne a retard.
    //
    // On mesure deux choses, qui ne disent pas la meme et se completent.
    //
    // 1. LES ECHOS QUE L'OREILLE PEUT COMPTER. Enveloppe RMS par blocs de 1 ms
    //    sur les 500 premieres millisecondes, puis nombre de maxima locaux dont
    //    la PROEMINENCE — le rapport au plus haut des deux creux qui
    //    l'encadrent — depasse 6 dB. Un echo isole est un pic qui emerge d'un
    //    creux ; une queue fondue est une enveloppe ou plus rien n'emerge. Pour
    //    du bruit, l'ecart-type du RMS sur 1 ms vaut ~0,9 dB : un pic a +6 dB
    //    n'y arrive pratiquement jamais, et chaque pic compte ici est donc un
    //    evenement audible, pas une fluctuation.
    //
    // 2. LE TEMPS DE MELANGE, au sens d'Abel et Huang (JAES 2006) : le premier
    //    instant ou la densite d'echos normalisee atteint 0,9. La NED est la
    //    fraction des echantillons d'une fenetre glissante qui depassent
    //    l'ecart-type de cette fenetre, divisee par erfc(1/sqrt(2)) = 0,3173 —
    //    la valeur que prend cette fraction pour du bruit gaussien. Elle vaut
    //    ~1 quand la reponse est devenue diffuse, et nettement moins tant
    //    qu'elle est faite d'echos separes par du silence. Aucun seuil
    //    arbitraire : la fenetre se normalise par son propre ecart-type.
    //
    // Mesures, structure d'AVANT (8 passe-tout, 4 lignes nues) contre
    // structure d'apres (24 passe-tout, 8 lignes a passe-tout interne) :
    //
    //             pics proeminents 0-500 ms        temps de melange
    //   fs        avant      apres                 avant      apres
    //   44,1 kHz  12 (24/s)  1 (2/s)                66 ms      26 ms
    //   48   kHz  13 (26/s)  1 (2/s)               115 ms      25 ms
    //   96   kHz  13 (26/s)  2 (4/s)                74 ms      27 ms
    //
    // Les bornes ci-dessous sont posees entre les deux colonnes, avec un
    // facteur ~3 de marge de chaque cote : l'ancienne structure les echoue aux
    // trois frequences d'echantillonnage, la nouvelle les tient.
    {
        siren::Spring s;
        s.prepare(fs);
        s.setParameters(1.f);

        const int N = (int) (0.5f * fs);
        std::vector<float> h((size_t) N);
        h[0] = s.process(1.f) - 1.f;   // on retire le sec : la sortie est un AJOUT
        for (int i = 1; i < N; ++i) h[(size_t) i] = s.process(0.f);

        // --- 1. pics proeminents de l'enveloppe 1 ms ----------------------
        const int B = (int) std::lround(0.001f * fs);
        std::vector<double> e;
        for (int i = 0; i + B <= N; i += B)
        {
            double a = 0.0;
            for (int k = 0; k < B; ++k) a += (double) h[(size_t) (i + k)] * h[(size_t) (i + k)];
            e.push_back(std::sqrt(a / (double) B));
        }

        int peaks = 0;
        const int n = (int) e.size();
        for (int i = 1; i < n - 1; ++i)
        {
            if (!(e[(size_t) i] >= e[(size_t) (i - 1)] && e[(size_t) i] > e[(size_t) (i + 1)]))
                continue;
            double lv = e[(size_t) i], rv = e[(size_t) i];
            for (int k = i - 1; k >= 0 && e[(size_t) k] <= e[(size_t) i]; --k)
                lv = std::min(lv, e[(size_t) k]);
            for (int k = i + 1; k <  n && e[(size_t) k] <= e[(size_t) i]; ++k)
                rv = std::min(rv, e[(size_t) k]);
            const double valley = std::max(lv, rv);
            if (valley > 0.0 && e[(size_t) i] / valley >= 2.0) ++peaks;
        }
        std::printf("  densite : %d pic(s) proeminent(s) sur 0-500 ms (%.0f /s)\n",
                    peaks, peaks / 0.5);
        check(peaks <= 4,
              "la queue ne contient pas d'echos que l'oreille puisse compter "
              "un par un");

        // --- 2. temps de melange (NED d'Abel-Huang) -----------------------
        auto nedAt = [&](int c) {
            const int w = (int) (0.020f * fs);
            const int a = std::max(0, c - w / 2), b = std::min(N, c + w / 2);
            if (b - a < 8) return 0.0;
            double s2 = 0.0;
            for (int i = a; i < b; ++i) s2 += (double) h[(size_t) i] * h[(size_t) i];
            const double sd = std::sqrt(s2 / (double) (b - a));
            if (sd <= 0.0) return 0.0;
            int c2 = 0;
            for (int i = a; i < b; ++i) if (std::fabs(h[(size_t) i]) > sd) ++c2;
            return ((double) c2 / (double) (b - a)) / 0.3173;
        };

        double mixMs = -1.0;
        const int step = (int) std::lround(0.001f * fs);
        for (int t = (int) (0.010f * fs); t < (int) (0.45f * fs); t += step)
            if (nedAt(t) >= 0.9) { mixMs = 1000.0 * (double) t / (double) fs; break; }

        std::printf("  densite : temps de melange (NED >= 0,9) = %.0f ms\n", mixMs);
        check(mixMs > 0.0 && mixMs <= 40.0,
              "la reponse devient diffuse en moins de 40 ms");
    }

    // --- La queue decroit, et finit par se taire --------------------------
    {
        siren::Spring s;
        s.prepare(fs);
        s.setParameters(1.f);
        s.process(1.f);

        auto rmsOver = [&](int n) {
            float acc = 0.f;
            for (int i = 0; i < n; ++i) { const float y = s.process(0.f); acc += y * y; }
            return std::sqrt(acc / (float) n);
        };
        const int q = (int) (0.4f * fs);
        const float a = rmsOver(q), b = rmsOver(q), c = rmsOver(q);
        check(a > b && b > c, "l'energie decroit, fenetre apres fenetre");

        // A 1,6 s la queue est tombee tres bas MAIS PAS ETEINTE. Le
        // commentaire annoncait deux bornes ; une seule etait ecrite, et la
        // borne basse est la moitie qui compte : sans elle, un reseau qui se
        // tairait completement — coefficients a zero, lignes non alimentees —
        // passerait ce bloc. Mesure a 48 kHz : late / a = 1,55e-3, donc les
        // deux bornes ont chacune plus d'un ordre de grandeur de marge.
        for (int i = 0; i < (int) (0.4f * fs); ++i) s.process(0.f);
        const float late = rmsOver((int) (0.2f * fs));
        check(late < a * 0.05f, "a 1,6 s la queue est descendue d'au moins 26 dB");
        check(late > a * 1e-4f, "mais elle n'est pas eteinte");
    }

    // --- Le RT60 : la cible de 1,6 s est TENUE ----------------------------
    //
    // La spec demandait « RT60 mesure dans +/-20 % de 1,6 s ». Ce fichier
    // avait substitue deux bornes laches sans tracer l'ecart. Le critere
    // d'origine passe, et de loin — a condition de mesurer le RT60 comme il se
    // mesure partout ailleurs : par la PENTE de la decroissance (le T30 de
    // l'ISO 3382 — regression entre -5 et -35 dB, extrapolee a 60 dB), et non
    // par l'instant absolu ou la queue croise -60 dB.
    //
    // Les deux ne disent pas la meme chose ici, et c'est ce qui a brouille
    // l'affaire. Le reseau se REMPLIT avant de decroitre : ses lignes font
    // 13,7 a 96,7 ms, la densite met ~0,125 s a s'etablir, et le pic d'energie
    // arrive la. Tout ce qui se compte depuis l'impulsion herite de ce huitieme
    // de seconde. A 48 kHz :
    //
    //   pente (T30, mesuree ici)                    1,597 s   (-0,2 %)
    //   -60 dB sous le pic, compte depuis le pic    1,65  s
    //   -60 dB sous le pic, compte depuis l'entree  1,775 s   (+11 %)
    //
    // La derniere ligne est celle qui avait fait abandonner le critere : elle
    // prend pour reference une fenetre qui contient le debut du remplissage,
    // sous-estime le niveau de depart et surestime donc la duree. Ce n'est pas
    // un RT60.
    //
    // La pente, elle, ne depend d'aucune reference : 1,603 s a 44,1 kHz,
    // 1,597 a 48, 1,592 a 96.
    {
        siren::Spring s;
        s.prepare(fs);
        s.setParameters(1.f);
        s.process(1.f);

        const int W = (int) (0.05f * fs);
        std::vector<double> db;
        double ref = -1.0;
        for (int b = 0; b < 100; ++b)
        {
            double acc = 0.0;
            for (int i = 0; i < W; ++i)
            { const float y = s.process(0.f); acc += (double) y * (double) y; }
            const double rms = std::sqrt(acc / (double) W);
            if (ref < 0.0) ref = rms;
            db.push_back(20.0 * std::log10(rms / ref));
        }

        double n = 0.0, sx = 0.0, sy = 0.0, sxx = 0.0, sxy = 0.0;
        for (size_t i = 0; i < db.size(); ++i)
            if (db[i] <= -5.0 && db[i] >= -35.0)
            {
                const double t = ((double) i + 0.5) * 0.05;
                n += 1.0; sx += t; sy += db[i]; sxx += t * t; sxy += t * db[i];
            }
        check(n >= 4.0, "assez de fenetres entre -5 et -35 dB pour une regression");

        const double slope = (n * sxy - sx * sy) / (n * sxx - sx * sx);
        const double rt60  = -60.0 / slope;
        std::printf("  RT60 (T30) = %.3f s pour une cible de 1,600 (%+.1f %%)\n",
                    rt60, (rt60 / 1.6 - 1.0) * 100.0);
        check(rt60 > 1.6 * 0.8 && rt60 < 1.6 * 1.2,
              "RT60 dans +/-20 % de la cible : le critere de la spec, retabli");
    }

    // --- Pas de NaN, pas de divergence, sur 60 s --------------------------
    {
        siren::Spring s;
        s.prepare(fs);
        s.setParameters(1.f);
        float worst = 0.f;
        for (int i = 0; i < (int) (60.f * fs); ++i)
        {
            const float y = s.process(std::sin(0.013f * (float) i) * 0.8f);
            check(std::isfinite(y), "ni NaN ni Inf");
            if (failures) break;
            worst = std::max(worst, std::fabs(y));
        }
        check(worst < 6.f, "le reseau ne s'emballe pas");
    }

    if (failures == 0) std::printf("test_spring : OK\n");
    return failures ? 1 : 0;
}
