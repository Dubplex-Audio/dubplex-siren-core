// SPDX-License-Identifier: PolyForm-Shield-1.0.0
#pragma once
// Les trois effets, chaines : sirene -> echo -> reverbe -> limiteur.
//
// Cet ordre-la, et pas un autre : les reprises prennent la piece, l'inverse
// mettrait la reverbe en boucle dans l'echo. Le limiteur est en dernier parce
// qu'il doit voir la somme de tout.
//
// APPELE APRES SirenEngine, jamais dedans. SirenEngine est un modele de
// circuit resolu par analyse nodale, confronte a des mesures ; y verser un
// effet invente le rendrait incomparable a l'unite.
//
// LA CHAINE TOURNE EN MONO. Le plugin est un instrument (PLUG_CHANNEL_IO
// "0-2", aucune entree) et SirenEngine::process ecrit le meme signal mono dans
// chaque canal : les canaux sont identiques, toujours. On travaille donc sur
// le canal 0 et on recopie, ce qui divise le cout par deux. Si le plugin
// redevenait un insert 2-in/2-out, cette hypothese deviendrait fausse et le
// canal droit serait silencieusement ecrase — d'ou le test qui compare les
// canaux et tombe au lieu de laisser faire.
#include "Limiter.hpp"
#include "Spring.hpp"
#include "TapeEcho.hpp"

namespace siren {

class FxChain {
public:
    struct Params {
        bool  echoOn    = false;
        float feedback  = 0.35f;
        float timeMs    = 300.f;
        float reverbMix = 0.f;
    };

    void prepare(double sampleRate)
    {
        echo_.prepare(sampleRate);
        rev_.prepare(sampleRate);
        lim_.prepare(sampleRate);
        echo_.setParameters(p_.echoOn, p_.feedback, p_.timeMs);
        rev_.setParameters(p_.reverbMix);
    }

    void reset() { echo_.reset(); rev_.reset(); lim_.reset(); }

    void setParameters(const Params& p)
    {
        p_ = p;
        echo_.setParameters(p.echoOn, p.feedback, p.timeMs);
        rev_.setParameters(p.reverbMix);
    }

    // CONSTANTE a frequence d'echantillonnage donnee : elle ne depend d'aucun
    // parametre. C'est une exigence, pas un constat — une latence qui
    // changerait avec l'etat de l'interrupteur ferait republier a chaque
    // bascule, et un hote repond a ca par un restartComponent : clic, voire
    // coupure. C'est pour la meme raison que le limiteur reste actif meme
    // effets coupes.
    int latencySamples() const { return lim_.latencySamples(); }

    void process(float** buffers, int numChannels, int numFrames)
    {
        if (numChannels <= 0) return;

        float* ch0 = buffers[0];
        for (int i = 0; i < numFrames; ++i)
            ch0[i] = lim_.process(rev_.process(echo_.process(ch0[i])));

        for (int c = 1; c < numChannels; ++c)
            for (int i = 0; i < numFrames; ++i)
                buffers[c][i] = ch0[i];
    }

private:
    TapeEcho echo_;
    Spring   rev_;
    Limiter  lim_;
    Params   p_;
};

} // namespace siren
