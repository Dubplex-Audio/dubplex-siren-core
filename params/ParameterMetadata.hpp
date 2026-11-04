// SPDX-License-Identifier: PolyForm-Shield-1.0.0
#pragma once
#include <cmath>
#include <cstdint>

namespace siren {

enum class Param : uint32_t {
    // Front panel (mirrors the unit's faceplate; the UI shows exactly these)
    tone,        // S5 rotary 1/2/3/4: osc-rate ladder (470k/220k/100k/68k)
    mode,        // S4 rotary 1/2/3/4: modulation routing per osc half
    speed,       // S3 rotary 1/2/3/OFF: LFO rate, OFF = LFO stopped
    volume,
    toneBtn,     // yellow button: routes the sweep through R16 to net 11, slow rise
    sirenBtn,    // green button: fast charge of C4 and freezes the LFO, steady top note
    trigger,     // momentary power, wired across SIREN ON
    hold,        // rocker: latched power (same path as trigger)
    // Fine-tune (host only, not on the panel)
    // Dev/physics trims (component tolerances of the real unit)
    // `vbe` a ete SUPPRIME : le seuil de conduction n'est plus un reglage, il
    // sort du modele Ebers-Moll des transistors.
    // Anti-alias. Le generateur, lui, n'a plus de variante : l'astable est
    // resolu comme un circuit, donc `model`, `collTau` et `fallTau` ont ete
    // SUPPRIMES. Ils choisissaient entre deux substituts au collecteur — un
    // carre idealise et une remontee approchee par un RC — la ou la tension de
    // collecteur est desormais calculee pour de vrai.
    oversample,  // 0=1x, 1=2x, 2=4x, 3=8x
    vcc,         // supply rail (V) — 12 V measured on the unit, 12.2 V at the board
    // Panneau d'effets. CES QUATRE-LA NE MODELISENT RIEN SUR LA CARTE, et
    // c'est voulu : la doctrine « aucun reglage sans contrepartie sur le
    // circuit » vaut pour LA PLAQUE, qui replique un objet mesure. Le panneau
    // de droite est de l'exterieur — un echo et une reverbe qu'on branche
    // derriere, comme on le fait vraiment. C'est pour ca qu'il ne ressemble
    // pas a de la tole : la frontiere se voit au lieu de se lire.
    reverb,      // dosage de la reverbe a ressort, 0..1
    echo,        // interrupteur du tape echo (mix fige : 100 % sec + 100 % mouille)
    feedback,    // nombre de reprises, 0..1
    time,        // ecart entre reprises, 40..800 ms
    // Output params
    ledLevel,    // LED1: lit whenever the unit is powered
};

// Les commandes du panneau d'effets, qui ne repliquent rien. (Le compteur
// symetrique pour la plaque a ete supprime : il n'etait lu nulle part, et un
// compteur que personne ne lit est un invariant que personne ne verifie.)
constexpr int kNumFxParams    = 4;
// Chaque parametre restant correspond a un COMPOSANT de la carte ou a une
// commande de facade. Ont ete supprimes en chemin : model, collTau, fallTau,
// vbe (le seuil sort d'Ebers-Moll), puis amount, drive et edge — trois
// effets sans contrepartie dans le circuit, et enfin charge, discharge,
// diode et capRatio — quatre resistances et un condensateur dont la valeur
// est LUE sur la carte (R16 10k, R20 47k, R10/R11 220k, C1 = C2 = 10 nF).
// Garder un reglage sur une valeur connue, c'est deguiser une certitude.
constexpr int kNumControlParams = 14;
constexpr int kNumOutputParams  = 1;

struct ParamInfo {
    const char* symbol;
    const char* name;
    float min;
    float max;
    float def;
    bool isBool;
    bool isInteger;
    bool isOutput;
    bool isLogarithmic;
    int numChoices;
    const char* const* choices;
};

static const char* kChoicesTone[]  = { "1", "2", "3", "4" };
static const char* kChoicesMode[]  = { "1", "2", "3", "4" };
static const char* kChoicesSpeed[] = { "1", "2", "3", "OFF" };
static const char* kChoicesOS[]    = { "1x", "2x", "4x", "8x" };

// Ordering must exactly match enum class Param above.
static constexpr ParamInfo kParams[kNumControlParams + kNumOutputParams] = {
    // 0  tone
    { "tone",      "Tone",       0.0f,   3.0f,   1.0f, false, true,  false, false, 4, kChoicesTone },
    // 1  mode
    { "mode",      "Mode",       0.0f,   3.0f,   0.0f, false, true,  false, false, 4, kChoicesMode },
    // 2  speed
    { "speed",     "Speed",      0.0f,   3.0f,   1.0f, false, true,  false, false, 4, kChoicesSpeed },
    // 3  volume : POSITION du potentiometre, 0..1. Un potentiometre de volume
    //    a piste lineaire donne une amplitude proportionnelle a sa position ;
    //    l'ancien parametre en dB (-60..+6) rendait le bouton logarithmique en
    //    amplitude — « trop logarithmique » a l'oreille. 1 = niveau de l'unite.
    { "volume",    "Volume",     0.0f,   1.0f,  1.0f, false, false, false, false, 0, nullptr },
    // 4  toneBtn
    { "toneBtn",   "Tone Button", 0.0f,  1.0f,   0.0f, true,  false, false, false, 0, nullptr },
    // 5  sirenBtn
    { "sirenBtn",  "Siren Button",0.0f,  1.0f,   0.0f, true,  false, false, false, 0, nullptr },
    // 6  trigger
    { "trigger",   "Trigger",    0.0f,   1.0f,   0.0f, true,  false, false, false, 0, nullptr },
    // 7  hold
    { "hold",      "Hold",       0.0f,   1.0f,   0.0f, true,  false, false, false, 0, nullptr },
    // 8  oversample (0=1x, 1=2x, 2=4x, 3=8x)
    { "oversample","Oversample", 0.0f,  3.0f,  2.0f, false, true,  false, false, 4, kChoicesOS },
    // 9  vcc (V) — the unit is played on a 12 V DC adapter; 12.2 V measured at
    //    the board with no sag under load. Sets both the astable period and the
    //    sweep-cap ceiling, so it governs how deep the wail dives.
    { "vcc",       "Supply",     6.0f,  15.0f,  12.2f, false, false, false, false, 0, nullptr },
    // 10 reverb — dosage seul. La decroissance du ressort ne se regle pas :
    //    un potard demande, un potard livre.
    { "reverb",   "Reverb",     0.0f,   1.0f,   0.0f, false, false, false, false, 0, nullptr },
    // 11 echo — interrupteur. Il n'a pas de dosage parce que son mix est fige
    //    a 100 % sec + 100 % mouille : allume, les reprises sortent au niveau
    //    du son direct. C'est ce qui rend la bascule franche.
    { "echo",     "Echo",       0.0f,   1.0f,   0.0f, true,  false, false, false, 0, nullptr },
    // 12 feedback — le nombre de reprises. Le gain de boucle vaut 1,05 fois
    //    cette valeur : le haut du potard auto-oscille, borne par le tanh.
    { "feedback", "Feedback",   0.0f,   1.0f,  0.35f, false, false, false, false, 0, nullptr },
    // 13 time — LOGARITHMIQUE. 40 ms a 800 ms couvrent du slapback au dub
    //    long ; une course lineaire tasserait tout le bas utile dans le
    //    premier vingtieme du potard.
    { "time",     "Time",      40.0f, 800.0f, 300.0f, false, false, false, true,  0, nullptr },
    // 14 ledLevel (output)
    { "ledLevel",  "LED Level",  0.0f,   1.0f,   0.0f, false, false, true,  false, 0, nullptr },
};

inline const ParamInfo& paramInfo(Param p) {
    return kParams[static_cast<uint32_t>(p)];
}

// La conversion valeur <-> position de potard, EN UN SEUL ENDROIT. Trois copies
// d'une loi de conversion finissent toujours par
// diverger — celle qui sert au geste et celle qui sert au MIDI donneraient
// alors deux courses differentes au meme parametre.
inline float toNorm01(const ParamInfo& pi, float v)
{
    if (pi.isLogarithmic)
        return std::log(v / pi.min) / std::log(pi.max / pi.min);
    return (v - pi.min) / (pi.max - pi.min);
}

inline float fromNorm01(const ParamInfo& pi, float n)
{
    if (pi.isLogarithmic)
        return pi.min * std::exp(n * std::log(pi.max / pi.min));
    return pi.min + n * (pi.max - pi.min);
}

} // namespace siren
