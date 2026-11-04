// SPDX-License-Identifier: PolyForm-Shield-1.0.0
// Le facteur machine : de combien une assemblee coute PLUS CHER ici que sur
// la machine ou kAsmCost a ete mesure.
//
// Le garde-fou du moteur compte des assemblees et les convertit en temps par
// une constante. Sur une machine chargee (session de DAW, antivirus, VPN) la
// meme assemblee coute 0,37 a 0,55 us au lieu de 0,20 : le garde-fou se croit
// a 40-90 % du budget pendant que le bloc rate son echeance, et l'hote
// decroche. Mesure du 14/10 : 48 blocs en retard sur 118 s, pire 176 %.
//
// La premiere mesure adaptative (une moyenne glissante du temps du bloc) a ete
// retiree : dans un fil audio temps reel, une preemption de l'OS gonfle un
// bloc de plusieurs millisecondes, la moyenne saute, le budget s'effondre et
// l'oscillateur se fige sur un changement de SPEED. Il faut separer les deux
// choses que l'horloge melange :
//   - la CHARGE DE FOND (caches pollues, frequence qui baisse, coeur partage),
//     qui rencherit TOUTES les assemblees, bloc apres bloc ;
//   - la PREEMPTION, qui frappe un bloc sur dix ou sur cent, enormement.
// Le minimum d'une fenetre de ~100 ms ne voit pas la preemption : il suffit
// qu'un seul bloc de la fenetre ait tourne sans etre interrompu, et c'est le
// cas courant. La charge de fond, elle, releve le minimum autant que le reste.
// Ces minima sont ensuite lisses : vite a la hausse (une charge qui arrive
// doit resserrer le budget en une demi-seconde), lentement a la baisse (ne
// pas rendre du budget sur un repit de quelques fenetres).
//
// Le facteur est borne a [1, kMax] : il ne fait que RESSERRER. Sur une machine
// au repos il vaut 1 et le budget est celui de la constante, a l'identique —
// la sortie reste deterministe bit a bit, ce que les tests et le banc A/B
// supposent. Au-dela de kMax (3x, soit 26 assemblees par image a 48 kHz) le
// budget degraderait autant qu'il protege ; a ce point la machine ne tient
// pas le plugin, quelle que soit la constante.
#pragma once
#include <algorithm>

namespace siren {

class MachineFactor {
public:
    static constexpr float  kMax      = 3.0f;     // 0,60 us pour une constante de 0,20
    static constexpr double kWindow   = 0.100;    // s d'audio par fenetre de minimum
    static constexpr float  kAlphaUp   = 0.25f;   // par fenetre : ~0,4 s pour suivre une charge
    static constexpr float  kAlphaDown = 0.05f;   // par fenetre : ~2 s pour la rendre

    // Un bloc de `seconds` d'audio dont une assemblee a coute `measured`
    // secondes la ou l'on en attendait `expected` (la constante du mode).
    void push(double measured, double expected, double seconds)
    {
        if (measured > 0.0 && expected > 0.0)
        {
            const float r = (float) (measured / expected);
            if (!haveMin_ || r < winMin_) { winMin_ = r; haveMin_ = true; }
        }
        winSeconds_ += seconds;
        if (winSeconds_ < kWindow) return;
        winSeconds_ = 0.0;
        if (haveMin_)
        {
            const float target = std::clamp(winMin_, 1.0f, kMax);
            const float a = (target > factor_) ? kAlphaUp : kAlphaDown;
            factor_ += a * (target - factor_);
            factor_ = std::clamp(factor_, 1.0f, kMax);
        }
        haveMin_ = false;
    }

    float factor() const { return factor_; }
    void  reset()        { factor_ = 1.0f; winMin_ = 0.0f; haveMin_ = false; winSeconds_ = 0.0; }

private:
    float  factor_ = 1.0f;
    float  winMin_ = 0.0f;
    bool   haveMin_ = false;
    double winSeconds_ = 0.0;
};

} // namespace siren
