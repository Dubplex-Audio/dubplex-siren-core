// SPDX-License-Identifier: PolyForm-Shield-1.0.0
#pragma once
// Modele de circuit de la sirene dub, resolu — pas decrit.
//
// Il n'y a plus AUCUNE formule de comportement dans ce moteur. Ni demi-periode
// calculee, ni enveloppe, ni regle de calage, ni plancher sur une tension. Tout
// ce qu'on entend sort de la resolution des deux astables et du reseau qui les
// relie, par Newton amorti sur le residu, avec pas de temps adaptatif.
//
// ---- Ce que le moteur resout -----------------------------------------------
//
// ASTABLE AUDIO (AstableCore) — 4 noeuds : bases et collecteurs de T1/T2.
//   T1/T2 2N2222A en Ebers-Moll, R9/R13 4k7 aux collecteurs, couplage croise
//   C1 10,0 nF (collecteur T1 -> base T2) et C2 10,1 nF (collecteur T2 ->
//   base T1). L'ecart de 1 % est ce qui amorce l'astable ; le circuit
//   parfaitement symetrique est l'idealisation, pas la realite. Les capacites
//   de jonction cje/cjc sont obligatoires : sans elles les condensateurs de
//   couplage ferment une boucle algebrique et la solution bute sur un pli.
//   La sortie EST la tension du collecteur de T2 (net 71).
//
// LFO + RESEAU DE BALAYAGE (LfoCore, sur le solveur generique Mna) — 11 noeuds :
//   T3/T4 PNP, echelle de cadence du SPEED, C4 100 uF, C5, D3/D4, et les deux
//   echelles du TONE qui chargent leurs sources. Resolu une fois par bloc — ses
//   constantes se comptent en dizaines de ms — puis ses deux noeuds utiles
//   (net 1 le carre, net 24 le balayage) sont reconstruits lineairement a
//   l'interieur du bloc, parce qu'un noeud se deplace, il ne saute pas.
//
// ROUTAGE DU MODE — un vrai commutateur 3 poles. Deux poles portent les
//   echelles de cadence, le troisieme distribue V+. L'appariement pole/moitie
//   vient du selecteur TONE, pas de l'oreille : base T1 <- echelle net 81 <-
//   pole 1, base T2 <- echelle net 76 <- pole 0. Voir kRoute.
//
// SORTIE — collecteur T2 -- R12 100k -- C3 10 uF -- net 45 -- potard. Le signal
//   ne sort pas du collecteur : il traverse 100k en serie, d'ou un diviseur et
//   un passe-haut dont R12 fait partie. Voir kRload, la seule valeur du chemin
//   qui ne soit pas sur la carte.
//
// ALIMENTATION — le rocker et TRIG coupent le RAIL, pas une enveloppe. Hors
//   tension tous les noeuds valent zero, ce qui est le seul etat coherent d'une
//   carte eteinte, et la sortie vaut exactement zero. L'astable s'amorce ensuite
//   sur la montee du rail. La LED est alimentee par ce meme rail : cette carte
//   n'a pas de driver de LED, elle suit donc la tension d'alimentation.
//
// ---- Ce qui a ete retire, et pourquoi --------------------------------------
//
// Le principe qui les gouverne tous : le circuit a toujours raison, le
// moteur presque toujours tort. Quand une mesure et le circuit divergent, on
// corrige le moteur — et souvent c'etait la MESURE qui se trompait.
//
// ---- Surechantillonnage ----------------------------------------------------
// oversample 1/2/4/8 : toute la chaine tourne a OS*fs et un FIR a fenetre
// decime. Ce n'est PAS une question de justesse — le coeur audio donne les
// memes frequences de OS=1 a OS=32 (150 / 320 / 700 / 1030 Hz) — mais
// uniquement d'image spectrale : les fronts du circuit sont bien plus raides
// que ce qu'une frequence d'echantillonnage peut porter.
#include <cmath>
#include <chrono>
#include <algorithm>
#include <cstdint>
#include "AstableCore.hpp"
#include "LfoCore.hpp"
#include "MachineFactor.hpp"

namespace siren {

class SirenEngine {
public:
    struct Params {
        bool  trigBtn  = false;  // momentary power (wired across SIREN ON)
        bool  sirenBtn = false;  // green button, pads 17 // 126: ties the LFO
                                 // square node to V+ -> fast charge of C4 via
                                 // D4+R15 AND freezes the LFO: steady top note
        bool  toneBtn  = false;  // yellow button, pads 149 // 69: routes node X
                                 // through R16 10k to net 11 -> slow rise
        bool  power    = false;  // rocker = latched TRIG (identical behavior)
        int   tone     = 1;      // S5 osc-rate ladder index (470k/220k/100k/68k)
        int   mode     = 0;      // S4 modulation routing, see kRoute
        int   speed    = 1;      // S3 lfo-rate ladder (47k/22k/10k), 3 = OFF
        float volume       = 1.0f;   // position du potentiometre de volume,
                                     // 0..1, gain = position (piste lineaire).
                                     // L'attenuation de la chaine est PHYSIQUE
                                     // (R12 + charge, 1/11) ; 1 = niveau de l'unite.
        float vcc_V        = 12.2f;  // supply rail — the unit is played on a 12 V
                                     // DC adapter. 12.2 V measured at the board,
                                     // no sag under load; re-confirmed at the
                                     // voltmeter on 9 September 2026 (pad 124 /
                                     // pad 122). Sets both the astable period
                                     // and the sweep-cap ceiling.
                                      // real output chain, only 2x HP 106 Hz)
        // Raffinement du regime lent (fin de mort) : -1 = automatique (active
        // quand le cout mesure d'une assemblee permet de le payer), 0 = jamais,
        // 1 = toujours. Ce n'est pas un reglage du circuit : c'est la precision
        // avec laquelle on le resout la ou Euler arriere se trompe le plus.
        int   slowRefine   = -1;
        // false (defaut) : le cout d'une assemblee est la constante du mode
        // (kAsmCost, kAsmCostD86) et la sortie est DETERMINISTE quelle que soit
        // la charge machine — ce que les tests et le banc A/B supposent.
        // true : la constante est multipliee par le facteur machine mesure
        // (MachineFactor), qui ne fait que RESSERRER le budget quand la machine
        // est chargee. C'est ce que demande le plugin ; au repos, c'est 1.
        bool  adaptiveBudget = false;
        // true (defaut) : une fois la carte eteinte ET entierement dechargee, le
        // solveur s'arrete, voir enterSleepIfSettled(). false ne sert qu'aux tests
        // qui verifient que cela ne change rien.
        bool  sleepWhenSettled = true;
        int   oversample   = 4;   // 4x par defaut : les fronts du circuit sont
                                 // plus rapides que 48 kHz ne peut tenir, et
                                 // mesure faite le cout n'est que x1,8 — le
                                 // travail cher est aux basculements, qui ne
                                 // se multiplient pas avec le facteur.      // 1/2/4/8 anti-alias factor for osc+tanh
    };

    void prepare(double sampleRate, int /*maxBlockSize*/)
    {
        fs_ = (float) sampleRate;
        dt_ = 1.0f / fs_;
        // Assemblees autorisees PAR IMAGE : l'echeance d'une image vaut 1/fs,
        // on s'en autorise kTimeShare, et chaque assemblee coute kAsmCost.
        // A 48 kHz cela donne 52, a 96 kHz 26, a 192 kHz 13.
        budgetPerFrame_ = kTimeShare / (fs_ * kAsmCost);
        machine_.reset(); asmCost_ = kAsmCost;

        led_ = 0.0f;   // suit le rail, voir plus bas
        asleep_ = false;
        prevPower_ = false;
        hp1x_ = hp1y_ = 0.0f; lfoSeeded_ = false; lfoLag_ = 0.0; audioLag_ = 0.0; lfoLagBlocks_ = 0;
        core_.reset(); vccS_ = 0.0f;
        lfo_.build(p_.vcc_V);

        volS_ = p_.volume;

        lastOS_ = 0;          // force decimator (re)design on first block
        decimReset();
    }

    void setParameters(const Params& p) { p_ = p; }

    // Panel LED brightness. This board has NO led driver: the inventory holds
    // only T1-T4 and the single 100k (R12) sits in series with the output. The
    // LED is wired between the switched V+ and ground, its limiting resistor
    // inline in the loom — so it is simply lit whenever the unit is powered.
    float ledLevel() const { return led_; }

    // Tension du sweep (net 24, aux bornes de C4) et du carre (net 1,
    // collecteur de T3). Ce sont de vraies tensions du circuit, exposees pour
    // pouvoir les confronter aux mesures faites sur la carte.
    float sweepLevel()  const { return lfoSweep_; }
    float squareLevel() const { return lfoSquare_; }

    // Adds the mono siren to every channel; channel content is preserved.
    void process(float** buffers, int numChannels, int numFrames)
    {
        // numFrames = 0 arrive (VST3 appelle process a vide pour purger les
        // parametres) : sans cette garde, dtBlock = 0, want / dtBlock = 0/0 et
        // (int)(NaN) — comportement indefini — puis frac = el / 0 = +inf, un
        // faux bloc « en retard » et « pire inf % » au journal.
        if (fs_ <= 0.0f || numFrames <= 0)
            return;

        // CARTE ETEINTE ET DECHARGEE : voir enterSleepIfSettled(). La sortie vaut
        // exactement zero et rien ne peut en sortir sans le rail. Le rail revenu,
        // tout repart du zero que le solveur aurait lui-meme calcule.
        if (asleep_)
        {
            if (!p_.power && !p_.trigBtn) { volS_ = p_.volume; return; }
            asleep_ = false;
        }

        const int OS = effectiveOS();
        const float dtEff = dt_ / (float) OS;
        if (OS != lastOS_) { decimDesign(OS); decimReset(); lastOS_ = OS; }

        // Only TRIG and the rocker provide power (they ARE the power path,
        // like the SIREN ON switch in the V+ rail). TONE and SIREN only
        // manipulate the routing / C5 while the unit is already powered —
        // pressed alone they are silent, like the hardware.
        const bool effPower = p_.power || p_.trigBtn;
        prevPower_ = effPower;

        // Le LFO et le reseau du sweep sont un CIRCUIT, resolu une fois par
        // bloc — ses constantes de temps se comptent en dizaines de ms, il n'y
        // a rien a gagner a le resoudre par echantillon. Couper l'alimentation
        // met vraiment le rail a zero : C4 se vide alors par R15+R18+R20, comme
        // sur la carte, au lieu de garder sa charge par un artifice.
        {   // Chaque moitie charge sa source par son echelle : une resistance
            // vers la base, pas une source de courant.
            const uint8_t* rt = kRoute[std::clamp(p_.mode, 0, 3)];
            const int   tn = std::clamp(p_.tone, 0, 3);
            const float gB = 1.0f / kRoscB[tn];   // moitie 0 = base T1
            const float gA = 1.0f / kRoscA[tn];   // moitie 1 = base T2
            // La MOYENNE DU BLOC, sans lissage supplementaire. C'est exactement
            // ce qu'un solveur appele une fois par bloc peut consommer : le
            // circuit charge l'echelle avec la tension de base instantanee, et
            // sur l'intervalle du bloc c'en est la moyenne.
            //
            // Il y avait ici un lissage sur 50 ms, introduit pour masquer le
            // crepitement de fin de mort. Il ne le masquait plus (61 sauts de
            // niveau superieurs a 30 % avec, 56 sans) et representait 37 blocs
            // de retard sur une grandeur que le circuit ne retarde pas. Retire.
            //
            // Le « crepitement » lui-meme etait mal mesure : au moment du pire
            // saut la mort du MODE 1 oscille vers 20-30 Hz, donc une fenetre de
            // 20 ms couvre moins d'un cycle. Ce qu'on mesurait, c'etait la forme
            // d'onde sub-audio, pas un artefact. Ce qu'entend l'oreille reste a
            // trancher sur un enregistrement de l'unite.
            if (loadN_) { vbA_ = loadA_ / (float) loadN_; vbB_ = loadB_ / (float) loadN_; }
            const float vA = vbA_, vB = vbB_;
            lfo_.setLadder(0, rt[0] < 2 ? rt[0] : -1, gB, vA);
            lfo_.setLadder(1, rt[1] < 2 ? rt[1] : -1, gA, vB);
            // LA BRANCHE D1/D2 NE TIRE PLUS SUR LE NET 63. Elle etait alimentee
            // par le net 63 a travers R16, du temps ou l'on croyait que le 3e
            // pole n'amenait au net 86 que le bouton TONE. Le net 86 est au
            // RAIL (voir plus bas, c.d86), et le rail est une source fixe : il
            // n'y a donc aucune charge a renvoyer au LFO. Le courant de D1/D2
            // reste compte par load86_, mais comme diagnostic seulement.
            lfo_.setNet86Load(0.0);
            loadA_ = loadB_ = load86_ = 0.0f; loadN_ = 0;
        }
        lfo_.setVcc(effPower ? p_.vcc_V : 0.0f);
        lfo_.setSpeed(std::clamp(p_.speed, 0, 3) + 1);
        lfo_.setButtons(p_.sirenBtn, p_.toneBtn, std::clamp(p_.mode, 0, 3));

        // Le budget du bloc se pose ICI, avant le LFO, parce que le LFO en
        // consomme aussi — et il n'etait pas compte du tout. Mesure : jusqu'a
        // 937 assemblees de 11 noeuds dans un bloc STABLE et 1516 a un passage
        // SPEED -> OFF, chacune valant ~1,6 assemblee audio. Le garde-fou du
        // noyau audio voyait donc un bloc plein alors qu'une bonne part de
        // l'echeance etait deja partie. Le LFO recoit une tranche plafonnee
        // (kLfoShare) pour ne jamais affamer le son ; s'il ne finit pas, il
        // s'arrete sur son dernier sous-pas COMMIS — un etat physique, juste
        // en retard — et le retard est reporte sur le bloc suivant. Pour des
        // constantes de dizaines de ms, ce retard est inaudible. Puis ce qu'il
        // a reellement depense est retire au noyau audio.
        blockT0_ = nowSeconds();
        // En MODE 3 la branche D1/D2 est raccordee et une assemblee coute le
        // DOUBLE (voir kAsmCostD86) : le budget en assemblees est divise
        // d'autant, sinon un bloc MODE 3 « au budget » a deja depasse son
        // echeance.
        // Le facteur machine (voir MachineFactor.hpp) multiplie cette constante
        // quand le plugin le demande : il vaut 1 au repos et monte, lentement
        // et sans voir la preemption, quand la machine rencherit chaque
        // assemblee. asmCost_ est le cout EFFECTIF retenu pour ce bloc.
        asmBase_ = (p_.mode == 2) ? kAsmCostD86 : kAsmCost;
        asmCost_ = p_.adaptiveBudget ? asmBase_ * machine_.factor() : asmBase_;
        budgetPerFrame_ = kTimeShare / (fs_ * asmCost_);
        // RAFFINEMENT DU REGIME LENT, TOUJOURS ACTIF. Il plafonne le pas a
        // 0,2 us quand une base traine pres du seuil sous un transistor bloque
        // — l'etat de la fin de mort, ou la regeneration n'est plus qu'un mode
        // instable faible qu'Euler arriere amortit a pas grossier.
        //
        // Il etait coupe pendant 2 s des qu'un bloc debordait du budget. Cette
        // coupure etait le crepitement. Mesure sur la mort du MODE 1 a SPEED
        // OFF, irregularite des intervalles entre impulsions dans la derniere
        // demi-seconde :
        //   unite (enregistrement)     35 Hz, 10 %
        //   raffinement actif          41 Hz, 14 %
        //   raffinement coupe          12 Hz, 100 %   <- des evenements epars
        // Autrement dit, « la resolution de base et ses clics » n'est pas une
        // degradation acceptable : c'est une dislocation. Et le declencheur
        // etait un depassement de CINQ assemblees sur 20000, soit 0,025 %.
        //
        // Sous Euler arriere, le pas de ce raffinement ne se relachait pas (a
        // 0,4 us l'irregularite finale passait a 63 %) et son cout — 2,4
        // budgets par bloc a 256 images — s'ecretait a chaque mort. C'est le
        // trapeze, dans cette seule fenetre, qui l'a rendu abordable : 0,3 us,
        // 0,82 budget au pire (AstableCore).
        const bool refine = (p_.slowRefine != 0);
        core_.setSlowRefine(refine);
        slowRefineOn_ = refine;
        workLeft_ = std::max(8, (int) ((float) numFrames * budgetPerFrame_));        workBudget_ = workLeft_;
        {
            const double dtBlock  = (double) numFrames * (double) dt_;
            // Le LFO n'avance que du temps que l'ASTABLE a reellement parcouru
            // au bloc precedent. Sinon un bloc ecrete gele l'astable pendant que
            // le balayage continue de descendre : a la reprise, la base en
            // approche trouve une cible plus basse que son etat ne le suppose,
            // rampe plus longtemps, puis bascule — la « pause puis reprise »
            // entendue a la toute fin de la mort. Un ecretage doit ralentir
            // TOUT le circuit, pas une moitie.
            const double applied = std::min(audioLag_, dtBlock);
            audioLag_ -= applied;
            const double want = std::max(0.0, dtBlock + lfoLag_ - applied);
            // La tranche suit le temps DEMANDE, pas le bloc. Sinon un bloc coupe
            // se coupe lui-meme a jamais : le retard reporte grossit la demande
            // du bloc suivant, la tranche ne grossit pas, il est coupe a son
            // tour — et ainsi de suite. Mesure : SIREN presse en MODE 3 SPEED 2,
            // le pire bloc a besoin de 791 assemblees pour une tranche de 672
            // (18 % de trop), et le LFO est reste coupe 375 blocs sur 375, sa
            // simulation figee pendant toute la pression du bouton. Avec une
            // tranche proportionnelle, un bloc de rattrapage peut monter a deux
            // tranches (80 % du budget, le retard etant borne a un bloc), une
            // fois, puis tout rentre dans l'ordre.
            const int lfoSlice = std::max(4, (int) (kLfoShare * workLeft_ * (want / dtBlock)
                                                    / kLfoAsmCostRatio));
            lfo_.resetWork();
            const double got  = lfo_.stepBudgeted(want, lfoSlice);
            // Report borne a un bloc. Seuil a 1 ns : run() conclut a
            // t >= dt*(1 - 1e-12), donc `want - got` vaut souvent ~1e-15 s de
            // poussiere flottante — qui, comptee comme un retard, faisait passer
            // CHAQUE bloc pour un bloc coupe (376 d'affilee au journal, alors
            // que le LFO livrait tout son temps a 1e-15 s pres).
            const double deficit = want - got;
            lfoLag_ = (deficit > 1e-9) ? std::min(dtBlock, deficit) : 0.0;
            if (lfoLag_ > 0.0) ++lfoLagBlocks_;
            workLeft_ -= (int) (lfo_.work() * kLfoAsmCostRatio);
            if (workLeft_ < 8) workLeft_ = 8;
        }
        // Le LFO est resolu une fois par bloc, mais ses noeuds ne SAUTENT pas :
        // ils se deplacent continument. `lfo_.step()` vient d'avancer le circuit
        // jusqu'a la FIN du bloc, donc la valeur precedente est celle du DEBUT.
        // On parcourt donc le bloc en interpolant entre les deux, ce qui
        // reconstruit le noeud au premier ordre pour zero cout de solveur.
        //
        // Ce n'est pas un raffinement gratuit : en MODE 3 les deux moities de
        // l'astable audio sont sur le carre, et l'astable a une periode de
        // 3,1 ms pour un bloc de 1,33 ms. Tenir la commande constante par bloc,
        // c'est presenter un escalier grossier au signal meme qui fixe le seuil
        // de basculement.
        lfoSquare_ = lfo_.square();
        lfoSweep_  = lfo_.sweep();

        // Premier bloc : pas de valeur anterieure, on evite une fausse rampe
        // depuis zero.
        if (!lfoSeeded_) { sqNow_ = lfoSquare_; swNow_ = lfoSweep_; lfoSeeded_ = true; }
        // On repart d'OU LES NOEUDS SONT ARRIVES, pas de la valeur du LFO au bloc
        // precedent : si l'astable a ete gele, ils n'ont pas atteint leur cible.
        sqStep_ = (lfoSquare_ - sqNow_) / (float) (numFrames * OS);
        swStep_ = (lfoSweep_  - swNow_) / (float) (numFrames * OS);

        // Per-block constants. All dt-dependent coefs are derived from dtEff so
        // the chain runs identically at any oversample factor (and exactly the
        // original values when OS==1).
        Blk c;
        c.effPower   = effPower;
        // Couper l'alimentation coupe le RAIL, comme le rocker sur la carte.
        // L'oscillateur s'arrete alors de lui-meme et les condensateurs se
        // vident par leurs vrais chemins — plus besoin de l'enveloppe de 2 ms
        // qui servait a masquer un oscillateur qu'on laissait tourner.
        // Le rail MONTE et DESCEND, il ne commute pas : c'est le temps
        // d'etablissement de l'alimentation et de son decouplage. Ce qui
        // remplace l'ancienne enveloppe de 2 ms — meme ordre de grandeur, mais
        // appliquee au rail, ou elle a un sens, et non a la sortie.
        vccS_ += (1.0f - std::exp(-(float) numFrames * dt_ / 0.002f))
                 * ((effPower ? p_.vcc_V : 0.0f) - vccS_);
        c.vcc        = vccS_;
        c.RoscB      = kRoscB[std::clamp(p_.tone, 0, 3)];   // base T1
        c.RoscA      = kRoscA[std::clamp(p_.tone, 0, 3)];   // base T2
        c.route      = kRoute[std::clamp(p_.mode, 0, 3)];

        c.smoothCoef = 1.0f - std::exp(-dtEff / 0.010f);
        c.hpCoef     = kHpTau / (kHpTau + dtEff);

        // Budget de travail par bloc — le garde-fou temps reel.
        //
        // Il ne depend PAS du surechantillonnage : l'echeance est la meme, qu'on
        // calcule une ou huit fois plus de sous-echantillons. Mais il depend de
        // la FREQUENCE D'ECHANTILLONNAGE, et c'est ce qui manquait ici.
        //
        // Le budget etait `numFrames * 96`, un nombre fixe cale sur l'echeance
        // a 48 kHz. Or l'echeance d'un bloc vaut numFrames / fs : a 96 kHz elle
        // est deux fois plus courte pour le meme nombre d'images, et a 192 kHz
        // quatre fois. Le garde-fou s'autorisait donc jusqu'a quatre fois le
        // temps disponible — c'est-a-dire qu'il ne gardait plus rien, et un
        // derapage du solveur passait en decrochage audio au lieu d'etre borne.
        //
        // Derive maintenant de l'echeance : budget = part de l'echeance / cout
        // d'une iteration. A 48 kHz cela redonne 93 par image, la calibration
        // d'origine — elle avait ete etablie par la mesure, on la conserve.


        c.volTarget    = p_.volume;
        // D1/D2 (anodes reunies sur le net 86) + R10/R11 220k, vers les DEUX
        // bases. Le net 86 n'a de source par aucun chemin propre : il ne porte
        // que les deux anodes. Il n'est atteint qu'en MODE 3, ou le 3e pole du
        // MODE y amene le net 11, c'est-a-dire le bouton TONE. La branche ne
        // conduit donc QUE en MODE 3 bouton TONE enfonce, et sa source est le
        // net 63 du LFO, pas V+.
        //
        // La version precedente y mettait V+ des que le MODE valait 3, sans
        // condition. C'etait un tirage symetrique de plus par-dessus le carre :
        // aux TONE 3 (100k) et 4 (68k) le courant de base atteignait 9 a 13
        // fois ce qu'exige la saturation, les DEUX transistors saturaient
        // ensemble, et cet etat n'a plus de gain — donc il est stable. Le
        // MODE 3 restait muet sur ces deux crans. Le solveur de reference fait
        // exactement la meme chose quand on force cette alimentation, et
        // retrouve ses quatre positions vivantes quand on ne la force pas.
        // Net 63 charge le net 86 par 10k, les bases le dechargent par 220k :
        // le net 63 impose sa tension a 5 % pres, on le prend comme source. Il
        // est passe BRUT — les diodes sont dans le noyau, ce sont elles qui
        // decident si la branche conduit. Ni chute fixe de 0,7 V ni plancher.
        // LE NET 86 EST AU RAIL EN MODE 3, BOUTON OU PAS. Le 3e pole du MODE
        // y pose son commun, le net 11 — et le net 11 a une piste vers V+ non
        // relevee (hypothese desormais la seule qui
        // explique une mesure). R10/R11, 220 k, alimentent donc les DEUX bases
        // depuis un rail FIXE, en parallele sur le barreau du TONE.
        // C'est ce que la vérité terrain exige, et elle le dit par une
        // ordonnee a l'origine COMMUNE aux deux tons du MODE 3 : 342 Hz en
        // phase haute, 333 Hz en phase basse, soit 217 kOhm vers un noeud que
        // le carre ne bouge pas. Cf. test 23l.
        // Le conditionner au bouton TONE, comme avant, laissait le MODE 3 sans
        // cette conductance : ses deux tons suivaient alors 1/R_echelle, ce que
        // l'unite ne fait pas.
        c.d86 = (p_.mode == 2);
        c.v86 = c.d86 ? vccS_ : 0.0f;

        for (int i = 0; i < numFrames; ++i)
        {
            float sample;
            if (OS == 1)
                sample = genSample(c);
            else
            {
                for (int k = 0; k < OS; ++k)
                    decimPush(genSample(c));
                sample = decimRead();
            }

            for (int ch = 0; ch < numChannels; ++ch)
                buffers[ch][i] += sample;
        }

        // La LED est cablee sur le rail : cette carte n'a pas de driver de LED
        // (les 43 composants du releve n'en comportent aucun). Elle suit donc la
        // tension d'alimentation, y compris pendant sa montee et sa descente,
        // au lieu de commuter — c'est le meme rail lisse que l'astable recoit.
        led_ = (p_.vcc_V > 0.0f) ? std::min(1.0f, vccS_ / p_.vcc_V) : 0.0f;
        const int used = workBudget_ - workLeft_;
        if (used > worstWork_) worstWork_ = used;
        {   // cout mesure de ce bloc, en s par assemblee (LFO compris, converti)
            // Les tests injectent un cout : ils ne peuvent pas charger la
            // machine de facon reproductible, ni attendre qu'elle le soit.
            const double el = dbgFakeAsmCost_ > 0.0
                ? (double) used * dbgFakeAsmCost_
                  + ((dbgFakeSpikeEvery_ > 0 && blocksSeen_ % dbgFakeSpikeEvery_ == 0) ? dbgFakeSpike_ : 0.0)
                : nowSeconds() - blockT0_;
            // MESURE TOUJOURS, N'UTILISE QUE SI ON LE DEMANDE. asmCostMeas_ ne
            // nourrit RIEN : ni le budget, ni le raffinement, ni la sortie. Il
            // n'existe que pour etre LU — le budget du garde-fou est en
            // assemblees, converti en temps par kAsmCost, une constante mesuree
            // sur une machine de bureau ; sur une autre machine cette constante
            // est fausse, et le garde-fou l'est avec elle. Sans ce chiffre on ne
            // peut que le deviner.
            //
            // Ni borne ni ecretee, contrairement a asmCost_ : le budget, lui, a
            // besoin de bornes pour ne pas s'effondrer sur une preemption. Une
            // mesure qu'on lit, non — la brider reviendrait a cacher exactement
            // le cas qu'on cherche. Lissage plus long (~20 blocs) pour la meme
            // raison : c'est l'ordre de grandeur qui interesse, pas le bloc.
            //
            // L'horloge etait DEJA lue ici a chaque bloc, y compris quand
            // adaptiveBudget est faux : cette mesure ne coute donc rien de plus.
            // ET L'ECHEANCE ELLE-MEME. Le travail dit ce que le garde-fou croit
            // depenser ; ceci dit ce qui a ete depense, a l'horloge murale —
            // celle de l'hote, preemption comprise, puisque c'est elle qui
            // decide d'un decrochage. Un bloc « en retard » a pris plus que son
            // echeance entiere : l'hote a forcement manque la sienne, quel que
            // soit le compte du garde-fou. C'est ce qui distingue un crepitement
            // du solveur (bloc rogne) d'un decrochage de l'hote (bloc en retard,
            // zero rogne).
            {
                const double frac = el / ((double) numFrames * (double) dt_);
                if (frac > worstTime_) worstTime_ = frac;
                if (frac > 1.0) ++blocksLate_;
            }
            if (used > 50 && el > 0.0)
                asmCostMeas_ += 0.05f * ((float) (el / (double) used) - asmCostMeas_);

            // Le facteur machine ne lit que les blocs qui ont assez travaille
            // pour que le temps fixe du bloc (LFO, LED, horloge) ne pese pas
            // dans la division ; les autres ne font qu'avancer sa fenetre.
            // Alimente meme quand adaptiveBudget est faux : il ne coute rien
            // et dit, au journal, ce que le plugin ferait.
            machine_.push((used > 200 && el > 0.0) ? el / (double) used : 0.0,
                          (double) asmBase_, (double) numFrames * (double) dt_);
        }
        if (workLeft_ <= 0) ++blocksClipped_;
        ++blocksSeen_;
        if (!effPower && p_.sleepWhenSettled) enterSleepIfSettled();
    }

private:
    // LA DECHARGE EST RESOLUE JUSQU'AU BOUT, c'est tout l'interet du modele : C4 se
    // vide par ses vrais chemins, C3 par la charge, et une remise sous tension en
    // cours de route repart de ce qui reste. Le solveur ne s'arrete donc que quand
    // il n'y a plus rien a entendre ni a garder : tous les noeuds sous 1 uV et la
    // sortie sous -180 dBFS, soit une vingtaine de secondes apres la coupure. Avant
    // ce point, la sortie est identique au bit pres (test_sleep).
    //
    // Pourquoi 1 uV et pas moins : mesure sur 1000 s, le solveur ne descend jamais
    // sous ce plancher. Une base et un noeud du LFO restent a 2,2e-7 V, parce que
    // c'est la que Newton s'estime converge ; la sortie y bruite a -210 dBFS. Au-dela
    // de ce point, il tournait 176 000 fois par seconde pour ne plus rien produire :
    // 100 us par bloc de 16 images sur l'A53, en permanence.
    //
    // Rien n'est remis a zero : l'etat reste ou il est, a l'arret. Une carte jamais
    // alimentee est donc identique au bit pres, reprise comprise.
    void enterSleepIfSettled()
    {
        constexpr double kEpsNode = 1e-6;
        constexpr float  kEpsOut  = 1e-9f;
        if (std::fabs(vccS_) > (float) kEpsNode || std::fabs(sqNow_) > (float) kEpsNode
            || std::fabs(swNow_) > (float) kEpsNode)
            return;
        if (std::fabs(hp1x_) > kEpsOut || std::fabs(hp1y_) > kEpsOut) return;
        if (lfoLag_ != 0.0 || audioLag_ != 0.0) return;
        if (!core_.settled(kEpsNode) || !lfo_.settled(kEpsNode)) return;
        for (int n = 0; n < decimN_; ++n)
            if (std::fabs(decimRing_[n]) > kEpsOut) return;
        led_ = 0.0f;
        asleep_ = true;
    }
    bool asleep_ = false;

public:
    bool asleep() const { return asleep_; }   // le solveur est-il a l'arret ?

public:
    // Compteurs de charge, pour voir ce que fait le solveur sans avoir a le
    // deviner : un bloc « rogne » est un bloc ou le budget a ete epuise, donc
    // ou l'oscillateur a tenu sa derniere tension au lieu d'etre resolu.
    // worstTime : pire bloc en fraction de son echeance, a l'horloge murale ;
    // blocksLate : blocs ayant depasse l'echeance entiere (voir process()).
    struct Load { int worstWork, blocksClipped, blocksSeen, budget; double worstTime; int blocksLate; };
    Load load() const { return { worstWork_, blocksClipped_, blocksSeen_, workBudget_, worstTime_, blocksLate_ }; }
    int   dbgLfoWork() const { return lfo_.work(); }   // assemblees du LFO (cumul)
    void  dbgLfoResetWork()  { lfo_.resetWork(); }
    float dbgToneNode() const { return lfo_.toneNode(); }   // net 63
    void  dbgLfoNodes(double* o) const { lfo_.dbgNodes(o); }
    float dbgVbase(int half) const { return half ? vbB_ : vbA_; }
    float asmCost()     const { return asmCost_; }          // s par assemblee, cout EFFECTIF du dernier bloc
    float machineFactor() const { return machine_.factor(); }   // cout mesure / constante, robuste (voir MachineFactor)
    // Cout injecte, pour les tests : chaque assemblee vaut `perAsm` s, et un
    // bloc sur `every` recoit `spike` s de plus — une preemption simulee.
    void  dbgFakeCost(double perAsm, double spike = 0.0, int every = 0)
    { dbgFakeAsmCost_ = perAsm; dbgFakeSpike_ = spike; dbgFakeSpikeEvery_ = every; }
    // s par assemblee REELLEMENT observes sur cette machine, toujours mesures,
    // jamais utilises par le solveur. A confronter a kAsmCost (0,20 us) : le
    // rapport entre les deux EST le facteur par lequel le garde-fou se trompe.
    float asmCostMeasured() const { return asmCostMeas_; }
    bool  slowRefineOn() const { return slowRefineOn_; }
    int   dbgCoreOk()   const { return core_.diagOk(); }
    int   dbgCoreFail() const { return core_.diagFail(); }
    void  dbgCoreDiagReset()  { core_.diagReset(); }
    float dbgOut()  const { return core_.out(); }
    float dbgOutA() const { return core_.outA(); }
    float dbgB1()   const { return core_.base1(); }
    float dbgB2()   const { return core_.base2(); }

    // asmCostMeas_ repart du nominal : la moyenne glissante rejoint la valeur
    // reelle en une vingtaine de blocs, soit quelques dizaines de ms. Le laisser
    // courir d'une session a l'autre melangerait deux mesures.
    void resetLoad() { worstWork_ = 0; blocksClipped_ = 0; blocksSeen_ = 0; lfoLagBlocks_ = 0;
                       worstTime_ = 0.0; blocksLate_ = 0;
                       asmCostMeas_ = kAsmCost; }

    // Facteur de surechantillonnage REELLEMENT applique : le parametre, plafonne
    // par la cadence interne. Le surechantillonnage n'existe que pour l'image
    // spectrale, et au-dela de ~384 kHz il n'y a plus rien a replier pour un
    // astable a 1 kHz : 8x a 192 kHz (1,5 MHz interne) coute autant que 8x a
    // 48 kHz sans rien apporter. L'audit temps reel l'a montre : c'etait le seul
    // couple qui ecretait. Le plafond ramene 192 kHz a 2x et 96 kHz a 4x.
    int effectiveOS() const
    {
        int OS = (p_.oversample >= 8) ? 8 : (p_.oversample >= 4) ? 4
               : (p_.oversample >= 2) ? 2 : 1;
        const int osMax = std::max(1, (int) (kMaxInternalRate / std::max(1.0f, fs_)));
        return std::min(OS, osMax);
    }

    // Latence introduite par le decimateur, en echantillons de sortie. Le FIR
    // est une sinc fenetree symetrique de 16*OS + 1 coefficients a la cadence
    // interne : retard de groupe 8*OS sous-echantillons, soit 8 echantillons de
    // sortie quel que soit OS >= 2, et 0 a 1x (pas de FIR). Le circuit, lui, n'a
    // aucune latence : celle-ci est la notre, et l'hote doit la connaitre pour
    // aligner la sirene sur le reste du mix.
    int latencySamples() const { return effectiveOS() >= 2 ? 8 : 0; }
    int  lfoLagBlocks() const { return lfoLagBlocks_; }
    double dbgLfoLag()    const { return lfoLag_; }         // retard reporte, en s

private:

private:
    // ---- per-block constant bundle ------------------------------------------
private:
    struct Blk {
        bool  effPower;
        float RoscB, RoscA, vcc;
        const uint8_t* route;
        float smoothCoef;
        float hpCoef;
        float volTarget;
        bool  d86;      // branche D1/D2 raccordee (MODE 3, TONE enfonce)
        float v86;      // tension du net 86 = net 63 du LFO, brute
    };

    // kVbe 0,65 V, kCosc 10 nF et vmax() ont ete retires : trois restes de
    // l'ancien moteur a formules, ou la demi-periode se calculait par
    // t = R*C*ln((Vm + Vcc - 1,3) / (Vm - Vbe)). Plus une seule reference dans
    // le moteur, le plugin ou les tests — le seuil de conduction et la valeur
    // des condensateurs de couplage vivent maintenant dans AstableCore, ou ils
    // sont des proprietes de composants et non les termes d'une formule.
    // Cout d'UNE ASSEMBLEE et part de l'echeance qu'on s'autorise. Les deux
    // viennent de la mesure, et il faut dire de quelle mesure.
    //
    // L'unite de travail est l'assemblee (jacobienne + devices), pas l'iteration
    // de Newton : une iteration en fait jusqu'a neuf quand la recherche lineaire
    // replie, et c'est dans les regimes difficiles qu'elle replie. L'ancien
    // « 0,13 us par iteration » etait mesure en regime facile, ou les deux se
    // confondent ; dans les pires blocs il sous-estimait le temps d'un facteur
    // 5 a 10 — la ou le garde-fou devait mordre, il laissait passer 27 ms pour
    // une echeance de 1,3 ms.
    //
    // Cout mesure par assemblee sur 15 000 blocs, machine a charge 15 sur 8
    // coeurs, FTZ arme comme dans le plugin (sans effet mesurable sur arm64) :
    // minimum 0,14 a 0,40 us selon le regime, mediane 0,21 a 0,50, la branche
    // D1/D2 doublant a peu pres le prix. 0,30 us est la mediane des regimes
    // courants sous forte charge — une session de DAW chargee — et reste 1,5 a
    // 2 fois conservateur sur une machine au repos.
    //
    // La part passe de 58 a 75 % : le garde-fou est un frein d'urgence, pas un
    // objectif de confort, et le quart restant est pour l'hote et ses voisins.
    // Trop serre, il degrade autant qu'absent : a 24 iterations par image il
    // gelait 35 % des blocs du MODE 1 (57 % de blocs rognes au journal d'une
    // session d'ecoute).
    static constexpr float kMaxInternalRate = 384000.0f;   // Hz, cf. process()
    // Cout FIXE par assemblee. 0,20 us : sur une machine courante l'assemblee
    // coute 0,15-0,20 us (microbench), et a 0,20 le budget vaut ~78 assemblees
    // par image a 48 kHz — au-dessus de la demande du sifflement du MODE 4
    // (~75), qui ne s'ecrete donc plus. Une moyenne glissante du temps a
    // l'horloge a ete ESSAYEE et RETIREE : dans le thread audio temps
    // reel, la preemption de l'OS gonfle le temps mesure, le budget s'effondre,
    // et l'oscillateur se fige sur un changement de SPEED ou de bouton — une
    // note haute bloquee, bien pire que le leger buzz qu'un budget fixe laisse
    // sur le seul sifflement du MODE 4. Mais la constante seule ne suit pas la
    // machine : chargee, l'assemblee coute 0,37-0,55 us et l'hote decroche
    // pendant que le garde-fou se croit large. D'ou MachineFactor.hpp : un
    // minimum par fenetre, que la preemption ne voit pas, qui ne fait que
    // resserrer, et que seul le plugin branche (Params::adaptiveBudget).
    static constexpr float kAsmCost   = 0.20e-6f;   // s par assemblee, au repos
    // La meme assemblee quand la branche D1/D2 est raccordee (MODE 3) : dioR
    // est un Newton scalaire SEQUENTIEL appele deux fois par assemblee, et le
    // net 86 au rail le met en direct tout le cycle. Microbench : 123-136 ns
    // sans, 240-265 ns avec ; le banc A/B le confirme (m3-s3-t4 : temps
    // CPU / travail = 1,17 contre 0,55 en MODE 4). Le MODE 3 demande au pire
    // 40 % du budget d'avant, 80 % de celui-ci : rien ne s'ecrete.
    static constexpr float kAsmCostD86 = 0.40e-6f;
    static constexpr float kRefineCostMax = 0.25e-6f; // au-dela, le raffinement ne tient pas dans le budget
    // (kRefineHoldoff a ete supprime : voir le commentaire du raffinement.)
    static double nowSeconds()
    {
        using namespace std::chrono;
        return duration<double>(steady_clock::now().time_since_epoch()).count();
    }
    // Une assemblee du LFO (11 noeuds) coute ~1,6 assemblee audio (0,47 contre
    // 0,30 us, medianes sous charge). 2,0 par prudence. Et le LFO n'a droit
    // qu'a 40 % du bloc : au-dela il prend du retard plutot que d'affamer le son.
    static constexpr float kLfoAsmCostRatio = 2.0f;
#ifndef SIREN_LFO_SHARE
#define SIREN_LFO_SHARE 0.40f
#endif
    static constexpr float kLfoShare        = SIREN_LFO_SHARE;
#ifndef SIREN_TIME_SHARE
// 0,85 et non 0,75 : le pire bloc de la fin de mort demande 75,02 % de
// l'echeance quand le raffinement agit — il debordait donc de cinq assemblees
// sur vingt mille. La marge etait juste trop courte, et c'est ce cheveu qui
// declenchait la coupure et le crepitement.
#define SIREN_TIME_SHARE 0.85f
#endif
    static constexpr float kTimeShare = SIREN_TIME_SHARE;   // part de l'echeance du bloc
    // Reseau de sortie, tel qu'il est cable :
    //   collecteur T2 -- R12 100k -- net 43 -- C3 10uF -- pastille OUT -- potard
    // Le signal ne sort donc PAS du collecteur : il traverse d'abord 100k en
    // serie. Avec une charge de 10k (le potard, ou une entree ligne) cela fait
    // un diviseur 10k/110k = 1/11 : le collecteur bat 12 V cc, la sortie reelle
    // en fait 1,1 — du niveau ligne. C'est pourquoi un casque branche en direct
    // n'entend presque rien.
    //
    // Et surtout, R12 entre dans la constante de temps du passe-haut : C3 voit
    // 100k + 10k = 110k, pas 10k. Le pole est a 0,145 Hz et non 1,6 Hz, soit
    // tau = 1,1 s au lieu de 0,1 s — un facteur ONZE. Quand l'oscillateur cale
    // et se gare sur un rail, ce continu met une seconde a s'evacuer, pas un
    // dixieme : c'est le « poids » qu'on entend apres la mort.
    //
    // La carte ne porte ni 15k ni 100nF, donc le reseau a deux poles a 106 Hz
    // que ce moteur modelisait autrefois n'existe pas ici.
    static constexpr float kR12   = 100e3f;
    // SEULE VALEUR DU CHEMIN DE SORTIE QUI NE SOIT PAS SUR LA CARTE. Les 43
    // composants du releve ne comportent aucun potentiometre : le potard de
    // facade est cable par le faisceau, en aval du net 45. 10k est donc une
    // HYPOTHESE, et elle porte deux consequences a la fois, kOutDiv et kHpTau,
    // toutes deux derivees ci-dessous pour qu'une seule correction propage.
    //
    // Sensibilite, si le potard n'est pas un 10k :
    //   10k  -> sortie /11    tau 1,10 s
    //   50k  -> sortie /3     tau 1,50 s
    //   100k -> sortie /2     tau 2,00 s
    // Le niveau bouge beaucoup, la constante du passe-haut beaucoup moins — le
    // « poids » qu'on entend apres la mort reste du meme ordre dans tous les
    // cas, ce qui explique qu'il sonne juste malgre l'incertitude.
    // A trancher d'une mesure : valeur du potard, et son bas est-il a la masse
    // (diviseur, charge constante) ou en l'air (rheostat, charge variable) ?
    static constexpr float kRload = 10e3f;    // potard / entree ligne — HYPOTHESE
    static constexpr float kC3    = 10e-6f;
    static constexpr float kOutDiv = kRload / (kR12 + kRload);        // 1/11
    static constexpr float kHpTau  = (kR12 + kRload) * kC3;           // 1,1 s

    // S5 osc-rate and S3 lfo-rate ladders. LFO half = 0.693*R*10uF.
    // LES DEUX ECHELLES, MESUREES A L'OHMMETRE le 9 septembre 2026 (nominal
    // 470k/220k/100k/68k chacune). Elles ne sont pas identiques, et le circuit
    // n'a jamais pretendu qu'elles l'etaient : ce sont huit resistances
    // distinctes. L'ecart reste sous 1 %, mais il est MESURE, alors que
    // l'egalite etait supposee.
    //   base T1 <- echelle net 81 <- MODE pole 1 : R5 R6 R7 R8
    //   base T2 <- echelle net 76 <- MODE pole 0 : R1 R2 R3 R4
    static constexpr float kRoscB[4] = { 467.5e3f, 218.8e3f, 100.5e3f, 68.0e3f };
    static constexpr float kRoscA[4] = { 465.5e3f, 219.8e3f, 100.9e3f, 68.0e3f };
    // Moyenne des deux, la ou une seule valeur suffit (chargement du LFO).
    static constexpr float kRosc[4]    = { 466.5e3f, 219.3e3f, 100.7e3f, 68.0e3f };

    // Sweep-cap discharge with the LFO frozen. Measured in-circuit across C4
    // (unit unpowered, ohmmeter): 45.80k at TONE 1 and 33.60k at TONE 4, both
    // MODE 4. The fixed part is the shaper path -- R21 1k + R18 2k2 + the 47k
    // to ground = 50.2k, confirmed by the 49.93k read in MODE 1. The rate
    // ladder hangs in parallel through ~43k of coupling, so a lower ladder
    // value drains the cap faster and the death shortens with TONE.
    static constexpr float kShaperR      = 50.2e3f;
    static constexpr float kLadderCouple = 43e3f;
    static float dischargeR(float Rladder)
    {
        const float par = Rladder + kLadderCouple;
        return (kShaperR * par) / (kShaperR + par);
    }
    static constexpr float kLfoHalf[3] = { 0.326f, 0.152f, 0.069f };

    // S4 routing per MODE: modulation source for each oscillator half.
    // 0 = sweep cap (C4), 1 = raw square, 2 = fixed V+.
    //
    // Read off the board, not guessed. MODE is a real
    // 3-pole switch: two poles carry the rate ladders, the third distributes
    // V+ — to half B's ladder in positions 1 and 2, and through D1/D2 to BOTH
    // bases in position 3. Its four positions are therefore
    //   1 sweep + fixed   2 square + fixed   3 square + square   4 sweep + sweep
    // In position 3 both ladder poles land on the SAME net (net 1 and net 26,
    // joined by jumper J1), and in position 4 both land on the sweep (net 24).
    // Earlier versions had 3 = sweep+square and 4 = fixed+fixed: MODE 4 then
    // never died, though the real unit's MODE 4 is precisely the one whose
    // death is longest and most audible.
    // QUEL POLE VA A QUELLE MOITIE — tranche par le schema releve, pas par l'oreille.
    // Le selecteur TONE donne l'appariement :
    //   TONE pole 0, commun net 66 = BASE T1, contacts 46/48/84/53
    //                = les extremites de R5/R6/R7/R8, echelle de commun net 81
    //   TONE pole 1, commun net 72 = BASE T2, contacts 31/29/27/25
    //                = les extremites de R1/R2/R3/R4, echelle de commun net 76
    // Donc base T1 <- echelle net 81 <- MODE pole 1 = {V+, V+, carre, balayage}
    //      base T2 <- echelle net 76 <- MODE pole 0 = {balayage, carre, carre, balayage}
    //
    // Les deux poles etaient assignes a l'ENVERS : le pole 0 allait a la moitie
    // A (T1). Les MODES 3 et 4 sont symetriques et n'en voyaient rien, mais les
    // MODES 1 et 2 avaient leurs deux sources croisees — et comme la sortie EST
    // le collecteur de T2, cela s'entend : le solveur de reference donne un
    // rapport cyclique de 0,519 en MODE 1 la ou le moteur donnait 0,396, ecarts
    // de signe oppose autour de 0,5.
    static constexpr uint8_t kRoute[4][2] = { {2,0}, {2,1}, {1,1}, {0,0} };

    // ---- one oversampled audio sample (mutates engine state) ----------------
    inline float genSample(const Blk& c)
    {
        // Le lisseur du volume S'ARRETE une fois arrive. Sans ca, vers zero il
        // n'y arrive jamais : sous ~3e-42, (1 - c) * volS_ s'arrondit a
        // lui-meme et la sortie reste sous-normale a perpetuite — 47 870
        // echantillons par seconde mesures, volume a zero. Sur arm64 c'est
        // gratuit, sur Intel chaque operation sous-normale est une assistance
        // microcode (voir DenormalGuard.hpp). A 1e-7 de la cible, l'ecart est
        // a -140 dB : on se pose dessus.
        volS_    += c.smoothCoef * (c.volTarget - volS_);
        if (std::fabs(c.volTarget - volS_) < 1e-7f) volS_ = c.volTarget;


        // ---- oscillateur principal : le CIRCUIT, resolu ---------------------
        // Plus de demi-periode calculee par formule. Les deux echelles imposent
        // chacune leur tension de modulation, le noyau resout T1/T2 + C1/C2 +
        // R9/R13 par Newton, et la sortie EST la tension de collecteur de T2.
        // Le balayage, le calage, la dissymetrie du rapport cyclique et le
        // regime rapide de fin de mort en sortent seuls — aucun n'est ecrit.
        sqNow_ += sqStep_; swNow_ += swStep_;   // le noeud avance, il ne saute pas
        float vmA = srcVoltage(c.route[0]);
        float vmB = srcVoltage(c.route[1]);
        // PAS DE PLANCHER. Le carre du LFO descend a -2,76 V et l'echelle tire
        // vraiment la base la-dessous : c'est cette tension NEGATIVE qui coupe
        // l'astable pendant la phase basse du MODE 3. L'ecreter a zero laissait
        // la base tendre vers 0 V, d'ou elle repart en regime rapide — et le
        // MODE 3 ne se coupait jamais. Le solveur de reference, qui n'ecrete
        // rien, rejette la moitie de ses fenetres de mesure en MODE 3 : chez
        // lui la phase basse est bien muette.
        // Budget de travail : ce qui reste pour ce bloc. Le solveur ne peut
        // donc jamais faire deraper l'echeance temps reel, quoi qu'il arrive.
        int spent = 0;
        double adv = 0.0;
        if (workLeft_ > 0)
        {
            core_.step(dtTick_, c.RoscB, vmA, c.RoscA, vmB, c.v86, c.d86,
                       c.vcc, workLeft_, &spent);
            workLeft_ -= spent;
            adv = core_.lastAdvance();
        }
        {
            // Les noeuds du LFO n'avancent que d'autant que l'astable. Budget
            // epuise, l'astable garde son etat le temps de finir le bloc — et
            // le balayage s'arrete AVEC lui, le retard est rendu au LFO au bloc
            // suivant (audioLag_). Continuer au pas grossier a la place a ete
            // essaye et retire : 12 a 21 % de cycles sautes, des sauts de 0,35
            // apres l'arret. Un ecretage doit ralentir tout le circuit, pas
            // en degrader une moitie.
            const double fr = std::clamp(adv / (double) dtTick_, 0.0, 1.0);
            sqNow_ -= sqStep_ * (float) (1.0 - fr);
            swNow_ -= swStep_ * (float) (1.0 - fr);
            audioLag_ += (double) dtTick_ - adv;
        }
        // Ce que l'echelle tire sur sa source : c'est un vrai courant, pas une
        // estimation, puisqu'on connait les deux tensions. Il est renvoye au
        // circuit du LFO — sans lui, la decharge de C4 ne dependrait plus du
        // TONE, alors qu'elle en depend (4,58 s contre 3,36 s a l'ohmmetre).
        loadA_ += core_.base1();   // tension de base moyenne, vue par l'echelle
        load86_ += core_.i86();    // courant tire au net 86 = au net 63 du LFO
        loadB_ += core_.base2();
        ++loadN_;
        // La tension du collecteur passe par R12 puis le diviseur de charge :
        // c'est la sortie REELLE de la carte, en volts, pas une normalisation.
        //
        // Il y avait ici un `- 0,5 * vcc` cense « ramener le collecteur autour
        // de zero ». C3 le fait deja, et c'est son role : le retrancher en plus
        // revenait a DEVINER la moyenne du collecteur (la moitie du rail, ce qui
        // n'est vrai que pour un creneau parfaitement symetrique) et a annuler
        // d'avance le transitoire de mise sous tension — un vrai transitoire,
        // que C3 met 1,1 s a evacuer sur la carte. Le terme dependait en outre
        // du rail, donc il injectait sa propre marche pendant la montee.
        // Moyenne du collecteur sur le sous-echantillon (anti-repliement avant
        // l'echantillonnage interne) : voir AstableCore::outAvg().
        const float oscOut = core_.outAvg() * kOutDiv;

        const float y = oscOut;

        // ---- output network: C3 into the load (one pole, tau 1.1 s), then the
        // volume -----------------------------------------------------------
        float h = c.hpCoef * (hp1y_ + y - hp1x_); hp1x_ = y; hp1y_ = h;
        return h * volS_;
    }

    // Les trois sources de modulation, telles que le MODE les route. Les deux
    // premieres sont des tensions du circuit, plus des variables d'etat.
    float srcVoltage(uint8_t src) const
    {
        switch (src)
        {
        case 0:  return swNow_;       // net 24, C4+
        case 1:  return sqNow_;       // net 1, collecteur de T3
            // Le 3e pole du MODE relie l'echelle au RAIL, pas a une constante :
            // rocker coupe, ce point est a zero comme tout le reste de la carte.
            // Le lire dans le parametre injectait 12,2 V dans un circuit hors
            // tension — c'etait le dernier residu qui empechait le silence exact.
        default: return vccS_;        // V+, par le 3e pole du MODE
        }
    }

    // Astable half-period for base resistor R pulled to Vm. Returns <= 0
    // when the transistor can never switch: the oscillator stalls (silence).


    // ---- decimator (windowed-sinc FIR, used only when OS > 1) ---------------
    void decimDesign(int OS)
    {
        decimOS_ = OS;
        if (OS <= 1) { decimN_ = 1; return; }
        decimN_ = std::min(kMaxFir, 16 * OS + 1);
        const float fc = 0.45f / (float) OS;        // cycles/sample at OS rate
        const int M = decimN_ - 1;
        float sum = 0.0f;
        for (int n = 0; n < decimN_; ++n)
        {
            const float x = (float) n - 0.5f * (float) M;
            float s = (x == 0.0f) ? 2.0f * fc
                                  : std::sin(6.2831853f * fc * x) / (3.1415927f * x);
            // Blackman window
            const float w = 0.42f
                - 0.5f * std::cos(6.2831853f * (float) n / (float) M)
                + 0.08f * std::cos(12.566371f * (float) n / (float) M);
            decimH_[n] = s * w;
            sum += decimH_[n];
        }
        for (int n = 0; n < decimN_; ++n) decimH_[n] /= sum;  // unity DC gain
    }
    void decimReset()
    {
        for (int n = 0; n < kMaxFir; ++n) decimRing_[n] = 0.0f;
        decimPos_ = 0;
        dtTick_ = dt_ / (float) std::max(1, decimOS_ ? decimOS_ : 1);
    }
    inline void decimPush(float x)
    {
        decimRing_[decimPos_] = x;
        decimPos_ = (decimPos_ + 1 == decimN_) ? 0 : decimPos_ + 1;
    }
    inline float decimRead() const
    {
        float acc = 0.0f;
        int idx = decimPos_;            // oldest sample
        for (int n = 0; n < decimN_; ++n)
        {
            acc += decimH_[n] * decimRing_[idx];
            idx = (idx + 1 == decimN_) ? 0 : idx + 1;
        }
        return acc;
    }

    Params p_;
    float fs_ = 0.0f, dt_ = 0.0f;

    bool  prevPower_ = false;
    float led_ = 0.0f;
    float vccS_ = 0.0f;   // rail lisse : montee/descente de l'alimentation
    float volS_ = 1.0f;              // volume lisse
    float hp1x_ = 0.0f, hp1y_ = 0.0f;  // C3 dans R12 + la charge

    // Oversampling state
    float budgetPerFrame_ = 52.0f; // assemblees autorisees par image, cf. prepare()
    double lfoLag_ = 0.0;          // temps de LFO non simule, reporte au bloc suivant
    double audioLag_ = 0.0;        // temps d'astable non parcouru (budget), a retenir sur le LFO
    int    lfoLagBlocks_ = 0;      // blocs ou le LFO a ete coupe (diagnostic)
    // Calibration ADAPTATIVE du cout d'une assemblee : mesure a chaque bloc
    // (deux lectures d'horloge), lissee, bornee. Le budget en assemblees en
    // decoule. Sans elle, le budget etait fixe sur 0,30 us — mesure sous une
    // charge de 15 sur 8 coeurs — et rognait sur une machine au repos (~0,2 us)
    // des regimes qu'elle rendait sans peine : le sifflement du MODE 4 / TONE 3
    // demande 70 a 84 assemblees par image, soit 67 a 80 % de l'echeance a
    // 0,2 us, 100 a 120 % a 0,30.
    float  asmCost_ = kAsmCost;    // s par assemblee, cout EFFECTIF du bloc (constante x facteur)
    float  asmBase_ = kAsmCost;    // la constante du mode, avant le facteur
    float  asmCostMeas_ = kAsmCost; // le cout brut, TOUJOURS mesure, jamais utilise
    MachineFactor machine_;        // cout mesure / constante, robuste a la preemption
    double dbgFakeAsmCost_ = 0.0;  // tests : cout injecte (0 = horloge reelle)
    double dbgFakeSpike_ = 0.0;
    int    dbgFakeSpikeEvery_ = 0;
    bool   slowRefineOn_ = false;  // etat courant du raffinement (diagnostic)
    double blockT0_ = 0.0;         // debut du bloc, horloge monotone (s)
    float dtTick_ = 0.0f;          // per-subsample dt (= dt_/OS)
    AstableCore core_;             // l'astable audio, resolu comme un circuit
    LfoCore     lfo_;              // le LFO + le reseau du sweep, idem
    float lfoSquare_ = 0.0f, lfoSweep_ = 0.0f;
    // Reconstruction lineaire des deux noeuds du LFO a l'interieur du bloc.
    float sqStep_ = 0.0f, swStep_ = 0.0f;
    float sqNow_ = 0.0f, swNow_ = 0.0f;
    bool  lfoSeeded_ = false;
    float loadA_ = 0.0f, loadB_ = 0.0f;   // tension de base moyenne, accumulee
    float load86_ = 0.0f;                 // courant de la branche D1/D2, accumule
    // A zero, pas a 0,65 : ce noeud alimente l'echelle, et le laisser a une
    // tension de conduction injecterait du courant dans un circuit hors
    // tension — le silence ne serait plus exact.
    float vbA_ = 0.0f, vbB_ = 0.0f;
    int   loadN_ = 0;
    int   workLeft_ = 0, workBudget_ = 0;
    int   worstWork_ = 0, blocksClipped_ = 0, blocksSeen_ = 0;
    double worstTime_ = 0.0;   // pire bloc, fraction de l'echeance (horloge murale)
    int   blocksLate_ = 0;     // blocs au-dela de l'echeance entiere
    int   lastOS_ = 0;

    static constexpr int kMaxFir = 129;        // 16*8 + 1 for OS=8
    float decimH_[kMaxFir] = {0};
    float decimRing_[kMaxFir] = {0};
    int   decimN_ = 1, decimPos_ = 0, decimOS_ = 1;
};

} // namespace siren
