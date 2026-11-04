// SPDX-License-Identifier: PolyForm-Shield-1.0.0
#pragma once
// Limiteur brickwall, en bout de chaine. Il existe a cause d'un choix
// deliberé : l'echo melange 100 % de sec et 100 % de mouille, donc la somme
// depasse le plein echelle des que les reprises se superposent au son direct.
// Corriger ca par un gain cache reviendrait a remettre un dosage par la bande ;
// on le traite donc ici, une fois, pour tout le monde.
//
// POURQUOI IL NE DEPASSE LE PLAFOND QUE D'UN ULP — l'argument, pas la
// promesse :
//
//   1. gInst[n] = min(1, plafond / |x[n]|)            le gain qu'il faudrait
//   2. wmin[n]  = min(gInst[k]), k dans [n-L, n]      minimum glissant
//   3. env      suit wmin : rapide a la descente, lent a la remontee
//   4. g        = min(env, plafond / |x[n-L]|)        LE FILET
//
// L'etape 4 porte la garantie : x[n-L] EST l'echantillon qu'on sort, donc
// |x[n-L]| * g <= plafond quel que soit l'etat du lissage. Sans elle, un
// transitoire plus raide que le lissage passerait — rarement, donc sans
// qu'aucun test occasionnel ne le voie. Les etapes 2 et 3 ne servent pas a
// tenir le plafond : elles servent a ce que ca ne s'entende pas.
//
// « Jamais » serait faux d'un ULP, et c'est l'arrondi du produit final
// `xd * g` qui le produit, pas le raisonnement : mesure 0,966050090 contre
// un plafond de 0,966050029. Il reste 0,3 dB jusqu'au plein echelle, donc
// ce depassement-la ne peut ecreter aucun convertisseur. C'est pour ca que
// les tests bornent a kCeiling + 1e-6 et non a kCeiling.
//
// SOUS LE PLAFOND, LE GAIN VAUT EXACTEMENT 1,0. Pas 0,999 : voir env_.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace siren {

class Limiter {
public:
    // -0,3 dBFS. Pas 0 : les pics inter-echantillons reapparaissent a la
    // reconstruction chez l'auditeur, et 0,3 dB de marge coutent moins cher
    // qu'un ecretage au convertisseur.
    static constexpr float kCeiling = 0.96605f;

    void prepare(double sampleRate)
    {
        fs_ = (float) sampleRate;
        L_  = std::max(1, (int) std::lround(0.0015f * fs_));   // 1,5 ms

        // Taille = L_, pas L_+1 : un tampon circulaire de taille N, lu avant
        // d'etre ecrit a chaque appel, produit un retard de N echantillons.
        // Avec N = L_+1 l'echantillon rendu est
        // x[n-L-1], pas x[n-L] comme l'exige l'invariant du filet plus haut
        // (etape 4) — la garantie de plafond tient quand meme (le filet relit
        // xd directement, pas gInst), mais latencySamples() mentirait d'un
        // echantillon et le test bit-a-bit sous le plafond echouerait.
        delay_.assign((size_t) L_, 0.f);
        cap_ = L_ + 2;
        dqV_.assign((size_t) cap_, 1.f);
        dqI_.assign((size_t) cap_, 0);

        // Attaque : L/3 echantillons. La fenetre d'anticipation annonce la
        // baisse L echantillons a l'avance, donc un tiers suffit largement a
        // converger avant que le pic arrive.
        aAtt_ = 1.f - std::exp(-3.f / (float) L_);
        aRel_ = 1.f - std::exp(-1.f / (0.100f * fs_));   // 100 ms

        reset();
    }

    void reset()
    {
        std::fill(delay_.begin(), delay_.end(), 0.f);
        w_ = 0; n_ = 0; head_ = 0; tail_ = 0; count_ = 0; env_ = 1.0;
    }

    int latencySamples() const { return L_; }

    float process(float x)
    {
        // Dernier rempart avant l'hote : un NaN entre ici passerait tel quel
        // (gInst = 1 sur NaN), et un Inf sortirait en NaN — `safe` vaut 0 et
        // Inf * 0 = NaN. Un bus de mixage qui recoit un NaN peut y rester.
        if (!std::isfinite(x)) x = 0.f;

        // --- 1. le gain qu'il faudrait a l'instant present -----------------
        const float ax = std::fabs(x);
        const float gInst = (ax > kCeiling) ? kCeiling / ax : 1.f;

        // --- 2. minimum glissant sur la fenetre, file monotone -------------
        // O(1) amorti. Un simple max sur L echantillons a chaque pas couterait
        // 72 comparaisons par echantillon a 48 kHz, pour le meme resultat.
        while (count_ > 0)
        {
            const int prev = (tail_ - 1 + cap_) % cap_;
            if (dqV_[(size_t) prev] < gInst) break;
            tail_ = prev; --count_;
        }
        dqV_[(size_t) tail_] = gInst;
        dqI_[(size_t) tail_] = n_;
        tail_ = (tail_ + 1) % cap_;
        ++count_;

        while (count_ > 0 && dqI_[(size_t) head_] < n_ - L_)
        { head_ = (head_ + 1) % cap_; --count_; }

        const float wmin = dqV_[(size_t) head_];

        // --- 3. l'enveloppe : vite en bas, lentement en haut ---------------
        env_ += ((double) wmin - env_) * (double) (wmin < env_ ? aAtt_ : aRel_);

        // L'ACCROCHAGE AU PLAFOND. Sans lui, env_ approche 1 sans jamais
        // l'atteindre, et « exactement 1,0 » reste faux d'un ULP. Voir env_
        // pour le pourquoi du double, et le titre pour l'invariant.
        if (wmin >= 1.f && env_ > 1.0 - 1e-9) env_ = 1.0;

        // --- l'echantillon qui sort, et 4. le filet ------------------------
        const float xd  = delay_[(size_t) w_];
        delay_[(size_t) w_] = x;
        if (++w_ == (int) delay_.size()) w_ = 0;
        ++n_;

        const float  axd  = std::fabs(xd);
        const float  safe = (axd > kCeiling) ? kCeiling / axd : 1.f;
        const double g    = std::min(env_, (double) safe);

        return (float) (xd * g);
    }

private:
    std::vector<float> delay_;
    std::vector<float> dqV_;   // valeurs de la file monotone
    std::vector<int64_t> dqI_; // leurs indices, pour l'expiration
    int   cap_ = 0, head_ = 0, tail_ = 0, count_ = 0;
    int   w_ = 0, L_ = 1;
    // n_ compte les echantillons depuis prepare(), SANS rebasage. En int il
    // deborde — comportement indefini — apres 2^31 echantillons, soit 12,4 h
    // a 48 kHz et 6,2 h a 96 kHz : deux durees qu'une session de studio
    // atteint. Et le degat ne s'arrete pas a l'UB : apres le wrap,
    // `dqI_[head_] < n_ - L_` cesse d'expirer quoi que ce soit, count_ passe
    // au-dessus de cap_ et la file monotone s'ecrase. En int64_t la question
    // ne se pose plus — 6 millions d'annees a 48 kHz.
    int64_t n_ = 0;
    float fs_ = 48000.f, aAtt_ = 1.f, aRel_ = 0.f;
    // EN DOUBLE, et c'est le meme defaut que TapeEcho::dSm_, reste ici.
    //
    // env_ += (wmin - env_) * aRel_ avec aRel_ = 2,083e-4 : pres de 1, l'ULP
    // du float vaut 5,96e-8, donc l'increment cesse d'etre representable des
    // que 1 - env_ < 1,43e-4. En float32 l'enveloppe se fige la et n'en
    // repart plus : mesure, apres UNE SEULE reduction — la premiere fois que
    // deux reprises se superposent — le limiteur appliquait definitivement
    // 0,999857, soit -0,00124 dB. Inaudible, mais ca falsifie l'invariant du
    // titre et la garantie que test_fx_chain appelle « le panneau au repos ne
    // coute rien au son ».
    //
    // Le plancher ULP depend de la frequence d'echantillonnage (aRel_ en
    // depend), donc un simple seuil d'accrochage en float serait a refaire a
    // chaque taux : a 384 kHz le gel arrive a 1,1e-3, au-dela de tout seuil
    // raisonnable. Le double supprime le plancher (il tombe a 5e-13), et
    // l'accrochage explicite de process() ne sert plus qu'a rendre le dernier
    // milliardieme exact, pour que « exactement 1,0 » se lise `== 1.0` dans
    // un test au lieu de « a un ULP pres ».
    double env_ = 1.0;
};

} // namespace siren
