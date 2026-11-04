// SPDX-License-Identifier: PolyForm-Shield-1.0.0
#pragma once
// L'echo a bande. Pas une ligne a retard : ce qui distingue les deux tient
// entierement a l'endroit ou sont les filtres.
//
// Ici ils sont DANS la boucle, sur le chemin d'ecriture. La reprise 1 est donc
// filtree une fois, la 2 deux fois, la 3 trois fois — elles s'assombrissent et
// s'epaississent en s'eloignant, exactement comme une bande qui repasse devant
// la meme tete. Les mettre en sortie donnerait des reprises identiques a
// niveau decroissant, c'est-a-dire un delai numerique.
//
// Mix FIGE : 100 % sec + 100 % mouille. L'echo n'a pas de dosage, il a un
// interrupteur. Le depassement de plein echelle que ca produit est le travail
// du limiteur en bout de chaine (dsp/Limiter.hpp), pas d'un gain cache ici —
// ce serait remettre un dosage par la bande.
//
// COUPE, LA BANDE S'ARRETE ET S'EFFACE. L'interrupteur n'est pas un mute sur
// la sortie : couper un appareil, c'est cesser de lui envoyer du son et vider
// ce qu'il tenait — le geste qu'on attend d'un device coupe dans Ableton.
// Avant, la boucle continuait de tourner a vide derriere le mute : rallume
// une minute plus tard, l'echo ressortait la queue d'il y a une minute, et
// a feedback haut l'auto-oscillation avait meme continue de vivre dans le
// noir. Maintenant : le mouille s'eteint en 5 ms (le fondu qui evite le
// clic), puis la ligne est effacee et plus rien n'y entre ; rallume, il
// repart vide, et la premiere reprise arrive TIME plus tard.
#include <algorithm>
#include <cmath>
#include <vector>

namespace siren {

class TapeEcho {
public:
    void prepare(double sampleRate)
    {
        fs_ = (float) sampleRate;
        // 0,85 s couvrent les 800 ms du maximum plus la marge d'interpolation.
        // ALLOUE ICI, jamais dans process : prepare tourne sur OnReset, qui
        // n'est pas le fil audio.
        //
        // + 8 la ou le plan disait + 4 : readCubic lit quatre points autour
        // d'un retard lisse qui peut depasser transitoirement la cible, et
        // huit echantillons coutent 32 octets. La marge exacte n'a aucune
        // importance tant qu'elle est superieure a ce que l'interpolation
        // touche ; on ne la rogne pas pour respecter un chiffre de plan.
        buf_.assign((size_t) std::ceil(0.85f * fs_) + 8, 0.f);

        // Coefficients a un pole. a = 1 - exp(-2*pi*fc/fs).
        aHp_   = 1.f - std::exp(-6.2831853f * 150.f  / fs_);
        aLp_   = 1.f - std::exp(-6.2831853f * 3000.f / fs_);
        // Le lissage de TIME : tau = 60 ms. C'est LUI qui fait glisser la
        // hauteur quand on tourne le potard, comme une bande qui accelere.
        // Sans lui, TIME produirait des sauts.
        aTime_ = 1.f - std::exp(-1.f / (0.060f * fs_));
        // L'interrupteur, lui, passe en 5 ms. Mesure a 48 kHz, queue en cours
        // : couper net faisait un saut de 0,46 entre deux echantillons (-7
        // dBFS, un clic franc). Ce n'est pas un dosage par la bande : la
        // cible est 0 ou 1, et une fois arrivee elle y est EXACTEMENT (voir
        // process), donc coupe la sortie reste l'entree bit a bit.
        aGate_ = 1.f - std::exp(-1.f / (0.005f * fs_));

        reset();
    }

    // reset() rend l'objet a l'etat que prepare() produit — TOUT cet etat.
    // dSm_ manquait : prepare() le posait et reset() le laissait ou il en
    // etait, si bien qu'un reset en cours de session rendait un objet qui
    // annonce `primed_ = false` (« la prochaine valeur se pose ») en gardant
    // le retard lisse de la session precedente. Les deux ne peuvent pas etre
    // vrais en meme temps.
    void reset()
    {
        clearLine();
        w_ = 0;
        dSm_ = msToSamples(timeMs_);
        gate_ = on_ ? 1.f : 0.f;
        // Le prochain setParameters() n'est pas un GESTE : voir plus bas.
        primed_ = false;
    }

    void setParameters(bool on, float feedback, float timeMs)
    {
        on_     = on;
        // Gain de boucle 1,05 au maximum : le haut du potard auto-oscille, et
        // le tanh du chemin d'ecriture l'empeche de diverger.
        fb_     = std::clamp(feedback, 0.f, 1.f) * 1.05f;
        timeMs_ = std::clamp(timeMs, 40.f, 800.f);

        // Le lissage de TIME (aTime_, dans process()) existe pour un GESTE :
        // tourner le potard pendant que ca joue, comme une bande qui
        // accelere. Le premier reglage apres prepare()/reset() n'est pas un
        // geste, c'est la pose d'une valeur de depart — le glisser produirait
        // un effet dont personne n'a demande la cause (la bande qui
        // accelere toute seule pendant sept dixiemes de seconde a
        // l'ouverture d'une session ou au chargement d'un preset). On pose
        // donc dSm_ directement dessus une seule fois ; tout reglage suivant
        // lisse normalement.
        if (!primed_)
        {
            dSm_    = msToSamples(timeMs_);
            gate_   = on_ ? 1.f : 0.f;   // meme raison : une pose, pas un geste
            primed_ = true;
        }
    }

    float process(float x)
    {
        // Pas d'armement FTZ ici : SirenPlug::ProcessBlock l'arme une fois par bloc,
        // en tete, avant tout DSP, et cette chaine tourne dans ce meme rappel.
        // SirenEngine ne l'arme pas non plus, pour la meme raison.

        const float dTarget = msToSamples(timeMs_);

        // COUPE ET ETEINT : la bande ne tourne pas. Rien n'est lu, rien n'est
        // ecrit, la sortie EST l'entree. TIME se pose sans glisser : le
        // glissement est un geste qu'on entend, et ici personne n'ecoute —
        // sinon un TIME tourne pendant la coupure ferait accelerer la bande
        // toute seule au rallumage.
        if (gate_ <= 0.f && !on_)
        {
            dSm_ = dTarget;
            return x;
        }

        dSm_ += (dTarget - dSm_) * aTime_;

        const float d = readCubic((float) dSm_);

        // Le chemin d'ECRITURE porte les filtres. L'ordre compte peu entre HP
        // et LP (deux poles reels commutent), mais le tanh doit venir en
        // DERNIER : c'est lui qui borne ce qui est stocke, donc ce qui revient.
        float v = x + d * fb_;
        hpS_ += (v - hpS_) * aHp_;   v -= hpS_;        // passe-haut 150 Hz
        lpS_ += (v - lpS_) * aLp_;   v  = lpS_;        // passe-bas 3 kHz
        // Un NaN ou un Inf qui entre ici ne doit pas entrer dans la LIGNE :
        // avec le feedback il y resterait pour toujours, tant que l'echo
        // tourne. Mesure (quand la ligne tournait aussi echo coupe) : 100 %
        // de sorties non finies cinq secondes apres un seul NaN. Les filtres
        // sont renvoyes a zero pour la meme raison.
        if (std::isfinite(v)) buf_[(size_t) w_] = std::tanh(v);
        else { buf_[(size_t) w_] = 0.f; hpS_ = 0.f; lpS_ = 0.f; }

        if (++w_ == (int) buf_.size()) w_ = 0;

        // L'interrupteur lisse (5 ms). ACCROCHE a 0 et a 1 : sous 1e-6 de la
        // cible on s'y pose, si bien que coupe c'est `x` tout court — bit a
        // bit — et ouvert c'est `x + d`, pas `x + d * 0,99999`.
        const float gTarget = on_ ? 1.f : 0.f;
        gate_ += (gTarget - gate_) * aGate_;
        if (std::fabs(gTarget - gate_) < 1e-6f) gate_ = gTarget;

        if (gate_ <= 0.f)
        {
            // Le fondu vient de finir : la bande s'efface, et la garde en
            // tete de process la laisse a l'arret jusqu'au rallumage. Une
            // fois par coupure, pas par echantillon — c'est ce que la garde
            // garantit. 0,85 s de bande a 48 kHz font 160 Ko : un memset
            // qu'un fil audio absorbe sans peine, une fois.
            clearLine();
            return x;
        }
        return gate_ >= 1.f ? x + d : x + d * gate_;
    }

private:
    float msToSamples(float ms) const { return ms * 0.001f * fs_; }

    // La bande vide et les filtres de la boucle a zero : ce que tient la
    // ligne, et rien d'autre (w_ et le retard lisse ne sont pas « tenus »).
    void clearLine()
    {
        std::fill(buf_.begin(), buf_.end(), 0.f);
        hpS_ = 0.f; lpS_ = 0.f;
    }

    // Catmull-Rom a quatre points. Une interpolation lineaire suffirait a un
    // retard fixe ; elle ne suffit pas a un retard qui GLISSE, ou elle se
    // comporte en passe-bas dont la coupure depend de la partie fractionnaire
    // — donc un timbre qui bat au rythme du balayage.
    float readCubic(float delay) const
    {
        const int    n = (int) buf_.size();
        const float  r = (float) w_ - delay;
        int          i = (int) std::floor(r);
        const float  f = r - (float) i;

        auto at = [&](int k) {
            int j = (i + k) % n;
            if (j < 0) j += n;
            return buf_[(size_t) j];
        };
        const float y0 = at(-1), y1 = at(0), y2 = at(1), y3 = at(2);

        const float c0 = y1;
        const float c1 = 0.5f * (y2 - y0);
        const float c2 = y0 - 2.5f * y1 + 2.f * y2 - 0.5f * y3;
        const float c3 = 0.5f * (y3 - y0) + 1.5f * (y1 - y2);
        return ((c3 * f + c2) * f + c1) * f + c0;
    }

    std::vector<float> buf_;
    int   w_ = 0;
    float fs_ = 48000.f;
    float aHp_ = 0.f, aLp_ = 0.f, aTime_ = 0.f, aGate_ = 0.f;
    float hpS_ = 0.f, lpS_ = 0.f;
    float gate_ = 0.f;     // l'interrupteur lisse : 0 coupe, 1 ouvert, exactement
    // Accumulateur double : en float32, le pas (cible - dSm_) * aTime_ tombe
    // sous l'ULP de dSm_ des que la cible approche 800 ms (~38 000
    // echantillons) bien avant convergence — le lissage se fige a quelques
    // echantillons de la cible et n'en bouge plus, quelle que soit la duree
    // d'attente. dSm_ reste donc en double ; readCubic() continue de lire en
    // float : a 38 400 echantillons l'ULP vaut 1/256, la fraction du retard
    // est donc quantifiee au 1/256 d'echantillon — une erreur a -60 dB sous
    // le mouille a 3 kHz, qui ne s'entend pas.
    double dSm_ = 0.0;

    bool  on_ = false;
    float fb_ = 0.f;
    float timeMs_ = 300.f;
    bool  primed_ = false;
};

} // namespace siren
