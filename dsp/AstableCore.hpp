// SPDX-License-Identifier: PolyForm-Shield-1.0.0
#pragma once
// Le multivibrateur audio (T1/T2) resolu comme un CIRCUIT, pas comme une
// formule. Analyse nodale, Euler implicite, Newton amorti a chaque pas,
// Ebers-Moll pour les transistors.
//
// Pourquoi ce n'est pas un luxe. Le modele analytique precedent calculait la
// demi-periode par
//     t = R*C*ln((Vm + Vcc - 2*Vbe) / (Vm - Vbe))
// et rendait « cale » des que Vm passait sous Vbe. Cette formule est un modele
// a UN noeud : elle suppose que la base se recharge par l'echelle vers Vm,
// pendant que le collecteur d'en face reste plaque au plancher. C'est vrai tant
// que l'echelle fournit assez de courant pour TENIR la saturation, et la
// formule colle alors a 1 % pres. En dessous, c'est faux, et pas qu'un peu :
//
//   Vm       t mesure    t formule
//   11,42 V   3,57 ms     3,54 ms
//    7,11 V   5,00 ms     4,96 ms
//    4,56 V   5,00 ms     6,63 ms
//    1,52 V   2,78 ms    12,70 ms
//
// Le circuit ACCELERE la ou la formule le fait ralentir jusqu'au silence. La
// forme d'onde relevee a Vm = 0,35 V le montre : la base et le collecteur oppose
// montent ENSEMBLE, le condensateur de couplage gardant leur difference
// constante. La base ne se recharge plus par l'echelle (470k, tau 4,7 ms) mais
// par la resistance de collecteur (4k7, tau 47 us) — cent fois plus vite. C'est
// un systeme a DEUX noeuds couples, qu'aucune formule fermee a un noeud ne peut
// exprimer. D'ou la resolution directe : le balayage, le calage, le regime
// rapide et la dissymetrie du rapport cyclique en sortent seuls, sans qu'aucune
// regle de comportement ne soit ecrite nulle part.
//
// Portage direct du solveur Python de reference, avec ses correctifs durement acquis :
// prolongement lineaire de l'exponentielle, amortissement des jonctions,
// recherche lineaire sur ||F||, convergence jugee sur le residu en ampere, et
// capacites de jonction — sans lesquelles un multivibrateur n'a tout simplement
// aucune vitesse de basculement.
#include <cmath>
#include <algorithm>

namespace siren {

class AstableCore {
public:
    // Valeurs relevees sur la carte.
    //
    // C1 / C2 : 9,30 et 9,39 nF. Ce n'est plus un ajustement, c'est une
    // DEDUCTION a partir de mesures. L'unite n'a pas de capacimetre exploitable,
    // mais l'astable donne f ~ 1/(R.C) : les hauteurs sont enregistrees et les
    // resistances ont ete lues a l'ohmmetre le 9 septembre 2026 —
    //   echelle base T1 (R5..R8) : 467,5 / 218,8 / 100,5 / 68,0 k
    //   echelle base T2 (R1..R4) : 465,5 / 219,8 / 100,9 / 68,0 k
    //   R9 / R13 : 4,657 / 4,663 k    R10 / R11 : 219,3 / 219,5 k
    // Les DOUZE sont a 1 % du nominal. Le decalage uniforme de +5 % des quatre
    // hauteurs ne peut donc pas venir d'elles : il est dans C, seul element
    // commun aux quatre positions du TONE. R connu, f mesuree, C tombe.
    //
    // Deux references INDEPENDANTES, ajustees separement :
    //   MODE 1 SPEED OFF SIREN tenu (160/337/728/1073 Hz) -> C1 ~ 9,35 nF
    //   MODE 3 phase haute (500/675/1067/1433 Hz)         -> C1 ~ 9,28 nF
    // La seconde ne suppose AUCUNE tension : T3 sature tient les deux echelles
    // au rail et R10/R11 y sont aussi, tout le chemin est mesure. Elle est en
    // prime quasi insensible a V+ (rms 0,87 / 0,78 / 0,82 % a 11,6 / 12,2 /
    // 12,8 V) — la propriete classique de l'astable. Cette deduction donnait
    // 9,30 / 9,39 nF SANS claquage base-emetteur. Avec le claquage a 9,5 V
    // (kBVebo) l'excursion negative de la base est bornee a -10 V au
    // lieu de -11,5, la recharge par R est plus courte, et les MEMES hauteurs
    // de l'unite (160 / 337 / 728 / 1073 Hz) redonnent C = 10,29 / 10,29 /
    // 10,26 / 10,24 nF aux quatre crans — uniforme a 0,5 %, et au NOMINAL 10 nF
    // a +3 %. Les 9,3 nF « a -7 % » compensaient l'absence du claquage.
    // Retenu : 10,22 / 10,32 nF (meme ecart relatif de 1 % entre les deux).
#ifndef SIREN_C1
#define SIREN_C1 10.22e-9
#endif
#ifndef SIREN_C2
#define SIREN_C2 10.32e-9
#endif
#ifndef SIREN_RC
#define SIREN_RC 4660.0
#endif
    static constexpr float kC     = (float) SIREN_C1;    // C1
    // C2 : 10 nF comme C1 sur la serigraphie, mais avec 1 % d'ecart. Ce n'est
    // pas une invention de confort : deux condensateurs film portant la meme
    // valeur nominale different toujours de quelques pourcents, et c'est cette
    // dissymetrie qui fait demarrer un astable. Le circuit parfaitement
    // symetrique est l'idealisation, pas la realite — il ne demarrerait jamais.
    static constexpr float kCb    = (float) SIREN_C2;  // C2 : meme +1 % d'ecart relatif
    // R9 / R13 : 4,657 et 4,663 kOhm MESUREES a l'ohmmetre sur l'unite le
    // 9 septembre 2026 (nominal 4k7). Moyenne retenue : 4,66 k.
    static constexpr float kRc    = (float) SIREN_RC;
    // Pas interne maximal. Le seuil mesure est 1,95 us (512 kHz) ; on prend
    // 1,5 us de marge. Voir la mesure dans stepImpl.
#ifndef SIREN_AUDIO_HMAX
#define SIREN_AUDIO_HMAX 1.5e-6
#endif
    static constexpr double kHMaxInt = SIREN_AUDIO_HMAX;
    // Seuil de conduction retenu pour dire « les deux moities conduisent ».
    static constexpr double kEqVb = 0.4;
    static constexpr float kIs    = 1e-14f;
#ifndef SIREN_BF
#define SIREN_BF 200.0f
#endif
    static constexpr float kBf    = SIREN_BF;
    static constexpr float kBr    = 4.0f;
    // CLAQUAGE INVERSE BASE-EMETTEUR. Le condensateur de couplage
    // projette la base a -11,5 V ; un 2N2222A tient 6 V garantis, 8 a 10 V en
    // pratique. Ce claquage avait ete essaye a 6 V et retire : +50 % sur toutes
    // les hauteurs. A 9,5 V il ne mord que sur les 1,5 derniers volts de
    // l'excursion, et c'est LUI qui fait le sifflement du MODE 4 tel que
    // l'unite le fait : la base kickee repart de -10 V, pas de -11,5, donc elle
    // atteint le seuil quand le collecteur oppose est encore a 1,5 V du rail —
    // le transistor qui s'allume est alors SATURE par le reste de la montee
    // (~0,2 ms, rapport cyclique 50 %, comme sur les enregistrements), et le
    // regime tient a 467k, ou il mourait sans claquage (le cran 1 s'eteignait
    // a 23 s). 9,5 V : le cran 3 tombe juste (2,62 contre 2,69 kHz), les
    // hauteurs du MODE 1 redonnent C1/C2 au nominal. Is 1e-12, n = 1 : un coude
    // net, le courant est nul a -9,5 V et milliampere a -10 V.
    static constexpr double kBVebo = 9.5;
    static constexpr double kIsBv  = 1e-12;
    // Capacites de jonction, CONSTANTES. Sans elles un multivibrateur n'a
    // aucune vitesse de basculement.
    //
    // La version dependante de la tension — modele SPICE C(v) = C0/(1-v/vj)^m,
    // physiquement plus juste — a ete essayee puis retiree. Traitee par la
    // capacite instantanee elle ne conserve pas la charge ; traitee
    // correctement par la charge, elle coute 2,5 fois plus cher et fait passer
    // le pire bloc a 60 ms pour une echeance de 10,7 — deux blocs en retard.
    // Et dans les deux cas elle ne corrige pas le crepitement de fin de mort,
    // qui etait la raison de l'essayer. Une amelioration dont le benefice n'est
    // pas demontre ne justifie pas de casser la surete temps reel.
    static constexpr double kRd    = 220e3;     // R10 / R11, en serie avec D1 / D2
    static constexpr float kCje   = 4.5e-12f;
    static constexpr float kCjc   = 4.0e-12f;


    void reset()
    {
        // Etat HORS TENSION : tout a zero. C'est l'etat reel d'une carte non
        // alimentee, et c'est ce qui garantit un silence exact tant que le
        // rocker n'est pas mis. L'astable demarre ensuite sur la montee du rail
        // et l'ecart de 1 % entre C1 et C2, comme la carte. Il n'y a rien a
        // parametrer : une carte eteinte ne connait pas encore son rail.
        for (int i = 0; i < N; ++i) v_[i] = 0.0;
        for (int i = 0; i < N; ++i) vPrev_[i] = v_[i];
        for (int k = 0; k < 6; ++k) iCap_[k] = 0.0;
        tr_ = false;
        // TOUT l'etat, pas seulement les tensions. Le predicteur (vStepPrev_,
        // hPrev_) extrapolait sinon la pente d'avant sur un rail a zero, et
        // sinceEdge_ laisse dans la fenetre du regime lent armait le
        // raffinement des la montee du rail : la sortie dependait de ce qui
        // s'etait passe AVANT le reset — pas de quoi s'entendre, mais le
        // contraire de « deterministe ».
        for (int i = 0; i < N; ++i) vStepPrev_[i] = 0.0;
        hPrev_ = 0.0; holdGrow_ = 0; sinceEdge_ = 1.0;
        vd86_[0] = vd86_[1] = 0.0; i86_ = 0.0;
        outAcc_ = tAcc_ = advanced_ = 0.0; outAvg_ = 0.0f;
    }

    // Un pas de dt. r1/vm1 et r2/vm2 : echelle et tension de modulation de
    // chaque moitie. gd/vd : le tirage 220k des diodes D1/D2 (MODE 3), nul
    // ailleurs. Rend false si meme le sous-decoupage n'a pas converge.
    // budget : nombre maximal d'iterations de Newton autorisees pour ce pas.
    // Un solveur iteratif SANS BORNE n'a rien a faire dans un thread audio :
    // mesure faite, un front du carre (12 V en un echantillon) faisait monter
    // un bloc a 91 ms pour une echeance de 1,33 ms — 68 fois trop. Quand le
    // budget est epuise on garde le dernier etat convergé et on avance : la
    // perte de detail dure quelques microsecondes, une coupure audio non.
    // `work` recoit le travail consomme, pour pouvoir le suivre de l'exterieur.
    // v86 : tension du net 86 (anodes de D1/D2), imposee de l'exterieur ;
    // d86 : la branche est-elle raccordee (MODE 3, bouton TONE enfonce).
    // Quand le budget est epuise, l'appelant garde le dernier etat commis
    // (maintien). Une degradation « basse puissance » — Newton borne a deux
    // iterations, meilleur point commis — a ete essayee et RETIREE : sur ce
    // circuit, un pas sous-converge suffit a faire tomber l'astable du regime
    // rapide dans son verrouillage, et le sifflement permanent du MODE 4 /
    // TONE 3 (reel, verifie a l'oreille) devenait un silence. Le maintien est
    // une deviation ; celle-la en etait une pire.
    bool step(float dt, float r1, float vm1, float r2, float vm2,
              float v86, bool d86, float vcc, int budget = 1 << 30,
              int* work = nullptr)
    {
        work_ = 0; outAcc_ = 0.0; tAcc_ = 0.0;
        // PORTE DE SORTIE d'un etat non fini. Le noyau n'en fabrique pas (dv
        // non fini refuse, point commis seulement apres evaluation), mais une
        // entree non finie — un rail NaN, une echelle a 0 ohm — fait un residu
        // NaN, Newton echoue, h descend a 1e-14 et step() rend false avec
        // advanced_ = 0 : l'appelant retient alors le LFO d'autant, les
        // entrees ne bougent plus, et le moteur est gele a jamais, sans un
        // mot. Ici on repart de la carte eteinte et on laisse passer le pas :
        // un silence d'un echantillon, puis la vie reprend.
        if (!(std::isfinite(v_[0]) && std::isfinite(v_[1]) && std::isfinite(v_[2]) && std::isfinite(v_[3])
              && std::isfinite(vm1) && std::isfinite(vm2) && std::isfinite(v86) && std::isfinite(vcc)
              && std::isfinite(r1) && std::isfinite(r2) && r1 > 0.0f && r2 > 0.0f))
        {
            reset();
            advanced_ = dt; outAvg_ = 0.0f;
            if (work) *work = 0;
            return false;
        }
        const bool ok = stepImpl(dt, r1, vm1, r2, vm2, v86, d86, vcc, budget);
        // MOYENNE TEMPORELLE du collecteur sur le sous-echantillon — un filtre
        // anti-repliement du premier ordre applique AVANT l'echantillonnage
        // interne, la ou le FIR de decimation n'agit qu'apres. Le collecteur est
        // un creneau de 12 V a fronts sub-microseconde : ses harmoniques vont au
        // MHz et se replient a la cadence interne ; a 4,3 kHz le 45e (193,5 kHz,
        // −33 dB) retombe a 1,5 kHz et se deplace de 540 Hz pour 12 Hz de
        // derive — le « phasing aigu qui ralentit » entendu sur le sifflement du
        // MODE 4. La moyenne sur le pas annule la reponse a la cadence interne
        // et ses multiples (~−42 dB sur ce 45e). Le solveur connait l'instant du
        // front a kEdgeH pres : c'est cette information qui passe dans la valeur.
        advanced_ = tAcc_;                       // le temps REELLEMENT parcouru
        if (tAcc_ < dt) { outAcc_ += v_[C2] * ((double) dt - tAcc_); tAcc_ = dt; }
        outAvg_ = (float) (outAcc_ / (double) dt);
        if (work) *work = work_;
        return ok;
    }

    int lastWork() const { return work_; }

private:
    bool stepImpl(float dt, float r1, float vm1, float r2, float vm2,
                  float v86, bool d86, float vcc, int budget)
    {
        // VRAI pas adaptatif, pas un decoupage uniforme. La difference est
        // decisive et elle a ete mesuree : le solveur de reference, sur ce meme
        // circuit a pas fixe de 5,2 us, echoue sur 19944 pas sur 20000 — le
        // chiffre exact qu'obtenait ce code. Avec son pas adaptatif, il tourne.
        //
        // Un decoupage uniforme impose le MEME pas partout : or la transition
        // regenerative demande la picoseconde alors que le reste du cycle se
        // contente de la microseconde. Il faut donc avancer, resserrer la ou ca
        // coince, puis repousser — et surtout ne pas repartir du debut.
        double t = 0.0;
        double h = dt;
        for (int guard = 0; guard < 200000; ++guard)
        {
            if (t >= dt * (1.0 - 1e-12)) return true;
            h = std::min(h, dt - t);
            // PAS INTERNE PLAFONNE. Meme cause que pour le LFO, meme
            // remede. Quand les DEUX moities sont attaquees par la meme tension
            // — c'est le cas des MODES 3 et 4, ou les deux echelles tombent sur
            // le meme noeud — l'astable est SYMETRIQUE : il part exactement de
            // son equilibre instable, et ne demarre que parce que C1 et C2
            // different de 1 %. Ce mode instable est faible, et Euler arriere
            // l'AMORTIT a grand pas : le circuit converge alors sur son point
            // de repos et ne bascule JAMAIS. Mesure au banc, D1/D2 au rail,
            // hauteur du TONE 3 puis du TONE 4 :
            //   192 kHz : 0 / 0 Hz     384 kHz : 1056 / 0 Hz
            //   512 kHz : 1057 / 1400  768 kHz : 1057 / 1401  (convergee)
            // Le silence n'etait donc pas un silence du CIRCUIT, c'etait un
            // artefact d'integration — et il rendait le MODE 3 monotonal.
            //
            // Le plafond est POSE LA OU IL SERT, et la signature est physique :
            // pres de cet equilibre les DEUX transistors conduisent en meme
            // temps. L'oscillation normale ne visite jamais cet etat — une base
            // y est toujours profondement negative (jusqu'a -11 V), une seule
            // moitie conduit a la fois. Plafonner partout coutait 2,5 fois le
            // temps de calcul (MODE 3 a OS 4 : 51,9 % d'un coeur contre 20,6) ;
            // plafonner ici ne coute que le temps passe pres de l'equilibre.
            if (v_[B1] > kEqVb && v_[B2] > kEqVb) h = std::min(h, kHMaxInt);
            // ANTICIPATION DU FRONT. Les echecs de Newton sont les fronts : 87 %
            // tombent a moins d'une microseconde d'un basculement (sifflement
            // du MODE 4 a SPEED OFF, TONE 4 : ~13 echecs par front). Le pas
            // arrivait a 5 us sur une base bloquee en route vers le seuil, s'y
            // cognait, et redescendait par moities jusqu'a l'echelle de la
            // regeneration, un echec par moitie. Or la montee de cette base se
            // voit plusieurs pas a l'avance : on plafonne le pas a la moitie du
            // temps qu'il lui faut, a sa vitesse actuelle, pour atteindre le
            // seuil. Le solveur entre dans le front a petit pas au lieu de s'y
            // briser. Ne touche que les bases BLOQUEES (non saturees, cf.
            // kVceOff) qui MONTENT sous kAnticipVb.
            //
            // Mesure : 44,4 -> 38,7 assemblees par echantillon, -9 % de CPU ;
            // banc A/B sous le bruit de fond, et la hauteur du sifflement se
            // deplace de 2 cents VERS la solution a pas fin (OS 8), pas loin.
            if (hPrev_ > 0.0)
                for (int k = 0; k < 2; ++k)
                {
                    const int bb = k ? B2 : B1, cc = k ? C2 : C1;
                    const double vb = v_[bb], pente = (v_[bb] - vStepPrev_[bb]) / hPrev_;
                    if ((v_[cc] - vb) > kVceOff && vb < kAnticipVb && pente > 0.0)
                        h = std::min(h, std::max(kAnticipHMin, 0.5 * (kAnticipVb - vb) / pente));
                }
            bool trStep = false;    // trapeze pour ce pas ? (regime lent raffine)
            // RAFFINEMENT DU REGIME LENT (optionnel, voir slowRefine_). Quand une
            // base est pres du seuil sous un transistor bloque, et que le dernier
            // front date de plus de kSlowAfter, le pas est plafonne a
            // kHNearTh. La regeneration y est un mode instable faible (tau ~
            // Rc*Cj ~ 40 ns) qu'Euler arriere amortit a grand pas : sans ce
            // plafond, la fin d'une mort produit des clics epars (12 cycles sur
            // la rampe de reference) la ou le modele converge — et l'unite, a
            // l'enregistrement — bourdonnent a ~45 Hz puis s'arretent net (78
            // cycles, motif d'OS 128 reproduit a OS 4). Cout : ~50 assemblees
            // par image dans ce regime seulement. L'appelant l'active quand la
            // machine peut le payer.
            if (slowRefine_)
            {
                // REGIME LENT : pres du seuil, sous un transistor BLOQUE, le pas
                // est plafonne a kHNearTh. La regeneration y est un mode instable
                // faible qu'Euler arriere AMORTIT a pas grossier : le circuit se
                // couche alors sur son equilibre et n'en ressort que par derive
                // numerique — d'ou des cycles sautes, entendus comme des clics.
                //
                // « Bloque » = NON SATURE, Vce > 1 V. Ce n'etait pas le critere
                // d'origine (collecteur au rail, > Vcc - 0,5), qui supposait le
                // regime d'attente classique. A tres basse commande l'astable
                // entre dans le regime couple decrit en tete de fichier — base et
                // collecteur oppose montent ENSEMBLE — et le collecteur n'est plus
                // au rail quand la base arrive au seuil : le raffinement ne
                // s'armait pas, et les crans 3-4 crepitaient. Retirer la
                // condition tout court n'allait pas non plus : le transistor
                // SATURE a lui aussi sa base dans la fenetre, en permanence, et
                // le raffinement tournait alors sans arret (5x le temps reel).
                //
                // Porte BASSE, kSlowAfter : sinon le pas fin s'armerait a chaque
                // cycle du jeu normal. Porte HAUTE, kDeadAfter : au-dela, plus
                // rien n'a bascule depuis longtemps, la mort est finie, le
                // circuit est a un equilibre STABLE et le pas fin n'y sert a rien
                // (il y tournait a plein sur un circuit immobile).
                const bool off1 = (v_[C1] - v_[B1]) > kVceOff, off2 = (v_[C2] - v_[B2]) > kVceOff;
                const bool n1 = v_[B1] > kNearLo && v_[B1] < kNearHi && off1;
                const bool n2 = v_[B2] > kNearLo && v_[B2] < kNearHi && off2;
                if ((n1 || n2) && sinceEdge_ > kSlowAfter && sinceEdge_ < kDeadAfter)
                {
                    h = std::min(h, kHNearTh);
                    trStep = true;      // et l'integrateur change, voir ci-dessous
                }
            }
            // TRAPEZE DANS CETTE FENETRE, Euler arriere partout ailleurs. Le pas
            // fin ne corrigeait qu'un defaut d'Euler arriere : il AMORTIT le mode
            // regeneratif marginal de l'approche du seuil, et il fallait 0,1 us
            // pour que cet amortissement ne couche pas le circuit — 5,33 ms de
            // bloc a 0,1 us, c'est 2,4 fois le budget a 256 images, donc des
            // blocs ecretes a chaque mort, parfois au milieu du front final
            // (plateau de 3 ms a mi-course, un double clic). Le trapeze n'amortit
            // pas ce mode : dans la meme fenetre il est converge des 0,1 us et le
            // reste jusqu'a 0,4 us (memes impulsions, memes dates de fin, 0,0 %
            // de cycles sautes aux quatre crans) ; a 0,5 us il decroche (cran 4 :
            // 3,7 % et un saut apres l'arret). Le trapeze PARTOUT avait ete essaye
            // et retire — 3,5 fois plus d'assemblees en regime calme, sonnerie
            // numerique sur les basculements (voir assemble). Ici il ne voit que
            // des rampes lentes a petit pas, la ou il est chez lui.
            tr_ = trStep;
            for (int i = 0; i < N; ++i) vPrev_[i] = v_[i];
            // PREDICTEUR : Newton part d'une extrapolation lineaire du dernier
            // pas commis, pas du dernier point. A 4 kHz avec des fronts en us,
            // le dernier point est loin de la solution du pas suivant et Newton
            // le paie en iterations et en replis — mesure : 25 % de tentatives
            // echouees et 8,4 assemblees par pas reussi dans le regime rapide
            // de fin de mort, contre 6,6 % et 3,3 en regime calme. La pente est
            // bornee par kDvMax pour qu'un front ne projette pas n'importe ou.
            double start[N];
            for (int i = 0; i < N; ++i)
            {
                double slope = (hPrev_ > 0.0) ? (v_[i] - vStepPrev_[i]) / hPrev_ : 0.0;
                start[i] = v_[i] + std::clamp(slope * h, -kDvMax, kDvMax);
            }
            double vBefore[N]; for (int i = 0; i < N; ++i) vBefore[i] = v_[i];
            if (newton(h, r1, vm1, r2, vm2, v86, d86, vcc, start) && (++nOk_, true))
            {
                // LOCALISATION DES FRONTS. Un pas converge peut contenir un
                // basculement entier : Euler arriere le pose alors a la FIN du
                // pas, quel que soit l'instant reel du seuil dans le pas. Avec h
                // repousse au sous-echantillon (5,2 us) entre deux fronts, chaque
                // front est ainsi date a +-2,6 us — et une periode non entiere
                // bat contre cette grille : c'est le « phasing aigu qui
                // ralentit » entendu sur le sifflement du MODE 4 (periodes
                // etalees de 1,7 %). Si un noeud a bouge de plus de kEdgeDv dans
                // le pas et que h depasse kEdgeH, on REJETTE le pas et on le
                // rejoue en deux : le seuil est alors date a kEdgeH pres.
                // Moyenner la sortie n'y changeait rien : c'est la trajectoire
                // elle-meme qui portait l'erreur.
                double dvMax = 0.0;
                for (int i = 0; i < N; ++i) dvMax = std::max(dvMax, std::fabs(v_[i] - vBefore[i]));
                if (dvMax > kEdgeDv && h > kEdgeH)
                {
                    for (int i = 0; i < N; ++i) v_[i] = vBefore[i];   // on annule le pas
                    ++nEdgeSplit_;
                    h *= 0.5;
                    continue;
                }
                sinceEdge_ = (dvMax > kEdgeDv) ? 0.0 : sinceEdge_ + h;   // age du dernier front
                commitCapCurrents();
                for (int i = 0; i < N; ++i) vStepPrev_[i] = vBefore[i];
                hPrev_ = h;
                outAcc_ += 0.5 * (vBefore[C2] + v_[C2]) * h; tAcc_ += h;   // integrale du collecteur
                t += h;
                // Repousse douce, 1,5x. Un controle du pas par l'erreur de
                // prediction (resserrer quand l'extrapolation se trompe, repousser
                // x2 quand elle tombe juste) a ete essaye et RETIRE : il ajoutait
                // des sous-pas partout (+8 % en regime calme, +12 % dans le regime
                // rapide) sans reduire le taux d'echec de Newton, qui reste a ~25 %
                // dans le regime rapide quel que soit le pas. Les echecs ne
                // viennent pas d'un pas trop grand a l'entree des fronts : ils
                // sont le front lui-meme, regeneratif, ou la linearisation ne
                // tient que sur des pas minuscules. A 4 kHz, ~40 % des sous-pas
                // SONT des fronts : ce cout est celui de la physique, pas d'un
                // reglage.
                //
                // Une retenue, en revanche, paie : apres un echec, le pas ne
                // repousse pas pendant kHoldAfterFail pas reussis. Sans elle le
                // pas regrimpe x1,5 a chaque succes jusqu'a echouer de nouveau,
                // divise par deux, et recommence — une dent de scie qui fait
                // echouer un essai sur quatre. Sifflement du MODE 4 a SPEED OFF,
                // TONE 4 : 49,6 -> 44,1 assemblees par echantillon (echecs 2,8 ->
                // 1,9), -9 % de CPU ; banc A/B sous le bruit
                // de fond d'un changement de taille de bloc sur les onze scenarios.
                if (holdGrow_ > 0) { --holdGrow_; h = std::min((double) dt, h); }
                else h = std::min((double) dt, 1.5 * h);
            }
            else
            {
                ++nFail_;
                holdGrow_ = kHoldAfterFail;
                h *= 0.5;
                if (h < 1e-14 || work_ >= budget) return false;
            }
            if (work_ >= budget) return false;   // budget epuise : on rend la main
        }
        return false;
    }

public:

    void  setSlowRefine(bool on) { slowRefine_ = on; }
    float out()  const { return (float) v_[C2]; }   // collecteur T2, avant R12 (etat)
    float outAvg() const { return outAvg_; }       // idem, moyenne sur le dernier pas (anti-repliement)
    // Temps parcouru par le dernier step() : dt si le budget a suffi, moins
    // sinon. C'est ce qui permet a l'appelant de ne pas laisser le reste du
    // circuit prendre de l'avance sur un astable fige par le budget.
    double lastAdvance() const { return advanced_; }
    int   diagOk()   const { return nOk_; }
    int   diagFail() const { return nFail_; }
    void  diagReset()      { nOk_ = 0; nFail_ = 0; nEdgeSplit_ = 0; }
    float i86()  const { return (float) i86_; }      // courant tire au net 86 (A)
    float outA() const { return (float) v_[C1]; }
    float base1() const { return (float) v_[B1]; }
    float base2() const { return (float) v_[B2]; }

private:
    enum { B1 = 0, C1 = 1, B2 = 2, C2 = 3, N = 4 };

    // Le claquage base-emetteur est dans bjt() : voir kBVebo. Il avait ete
    // ecarte a 6 V (+50 % sur les hauteurs) ; c'est a 9,5 V qu'il est dans le
    // circuit, et la campagne d'enregistrement l'a rendu necessaire.
    static constexpr double kVt    = 0.025852;
    static constexpr double kGmin  = 1e-9;    // fuite : 1 GOhm par noeud
    static constexpr double kVmaxJ = 1.0;     // au-dela, exponentielle prolongee
    static constexpr double kVjOff = -80.0 * kVt; // en-deca, exp(v/Vt) ne compte plus
    static constexpr double kDvMax = 0.5;     // deplacement max d'une jonction
    // exp(kVmaxJ / kVt), calculee au chargement et non en statique locale de
    // jexp() : voir Mna.hpp, meme raison (garde d'initialisation sur le fil
    // audio).
    static inline const double kEmax = std::exp(kVmaxJ / kVt);
    // Iterations de Newton au plus par tentative de pas. Au-dela, resserrer le
    // pas est TOUJOURS le meilleur coup : une tentative qui n'a pas converge en
    // 40 iterations ne convergera pas en 150, elle brule 110 iterations de plus
    // — jusqu'a 9 assemblees chacune avec la recherche lineaire — avant que le
    // pas adaptatif ne fasse ce qu'il aurait du faire tout de suite. C'etait
    // la source des rafales : 60 a 80 assemblees par image dans les pires blocs
    // pour une mediane de 15. SPICE plafonne entre 10 et 50.
    static constexpr int    kNewtonMax = 40;
    // Pas plafonne pres du seuil, sous le TRAPEZE. Convergence en pas sur la
    // mort du MODE 1 a SPEED OFF, budget libre, part des intervalles entre
    // impulsions depassant deux fois la mediane dans la derniere seconde
    // (l'unite : 1,0 a 1,8 % selon le cran), nombre d'impulsions et date de fin :
    //     Euler arriere 0,10 us : 0,0 / 0,0 / 0,4 / 0,8 %   (reference d'avant)
    //     Euler arriere 0,20 us : 0,0 / 4,1 / 5,1 / 5,9 %   (casse)
    //     trapeze 0,10 a 0,40 us : 0,0 / 0,0 / 0,0 / 0,0 %, memes impulsions,
    //                              memes dates de fin a 10 ms pres
    //     trapeze 0,50 us : cran 3 1,3 %, cran 4 3,7 % et un saut apres l'arret
    //     trapeze 0,70 us : cran 3 21,6 %
    // 0,30 us : converge, a bonne distance de la falaise (0,4-0,5), et la
    // demande du regime lent vaut 0,82 budget a 256 images, 0,74 a 512. A 0,25
    // un bloc de 64 images s'ecretait encore au cran 1.
    static constexpr double kHNearTh   = 0.30e-6;
    // Fenetre de base « approche du seuil ». Le bas a ete balaye (budget
    // libre) : 0,50 ne change rien, a 0,55 le cran 1 meurt deja plus tard
    // (1479 impulsions au lieu de 1589), a 0,60 tout s'effondre (25 % de cycles
    // sautes). Le gain de boucle passe donc l'unite entre 0,55 et 0,60 V ; 0,45
    // garde la marge.
    static constexpr double kNearLo    = 0.45;
    static constexpr double kNearHi    = 0.75;
    static constexpr double kVceOff    = 1.0;    // V : « bloque » = pas sature
    // Sans front depuis kDeadAfter, la mort est finie : le circuit est a un
    // equilibre stable et le pas fin n'y sert a rien. 0,15 s et non 0,5 : le
    // plus long intervalle final observe (cran 1) fait ~40 ms, et au-dela le
    // raffinement tournait a plein sur un circuit immobile — 2,4 budgets par
    // bloc pendant 0,5 s, pour rien.
    static constexpr double kDeadAfter = 0.15;
    // 1,5 ms et non 5 : cale d'abord sur la mort du cran 1 (40 Hz, un front
    // toutes les 12,5 ms), le seuil ne s'ouvrait jamais aux crans qui finissent
    // entre 120 et 360 Hz (un front toutes les 1,4 a 4 ms). 
    static constexpr double kSlowAfter = 1.5e-3;    // s depuis le dernier front : regime lent
    static constexpr double kEdgeDv = 2.0;      // V : au-dela, le pas contenait un front
    static constexpr double kEdgeH  = 0.25e-6;  // s : precision de datation des fronts
    // Replis de la recherche lineaire au plus. Chaque repli est une assemblee
    // complete, et un pas sans descente en brulait huit (lambda / 256) avant de
    // rendre la main au decoupage du pas de temps. Cinq (lambda / 32) suffit :
    // si un pas de Newton n'a pas produit de descente a lambda / 32, c'est le
    // pas de temps qui est trop grand, pas lambda qui est trop petit.
    //
    // Trois (lambda / 8), pas cinq. Dans le sifflement du MODE 4 a SPEED OFF,
    // les echecs de Newton pesaient 56 % du travail (TONE 4 : 37,7 assemblees
    // par echantillon sur 67), tous par recherche lineaire bloquee, chacun
    // brulant ses replis avant de rendre la main. A trois : -14 % de CPU sur ce
    // regime, travail 62 -> 47 % du budget ; au banc A/B
    // hauteur a 1 cent pres en mediane, tiers d'octave a 1,2 dB, plancher
    // inharmonique inchange — sous le bruit de fond d'un simple changement de
    // taille de bloc. A deux, le plancher inharmonique du MODE 4 TONE 4
    // remontait de 4,6 dB : trop court.
    static constexpr int    kLineSearchMax = 3;
    // Pas reussis sans repousse apres un echec de Newton (voir stepImpl). 2 :
    // a 1 il reste 3 % de travail en plus, 4 n'en retire pas davantage.
    static constexpr int    kHoldAfterFail = 2;
    // Anticipation du front (voir stepImpl) : seuil vise et plancher du pas.
    // Plancher 1 ns : a 5 ns il reste 2 % d'assemblees de plus.
    static constexpr double kAnticipVb   = 0.62;
    static constexpr double kAnticipHMin = 1e-9;
    static constexpr double kReltolI = 1e-3;  // tolerance SPICE sur les courants
    static constexpr double kAbstol  = 1e-12;
    static constexpr double kReltolV = 1e-6;   // tolerance relative en tension
    static constexpr double kVntol   = 1e-9;   // plancher en volt

    // Entrailles en DOUBLE, interface en float. Le courant d'un condensateur
    // se calcule comme g*((va-vb) - precedent) : une difference de deux
    // tensions voisines de 12 V multipliee par une conductance de ~2e-3. En
    // float32 la precision de 12 V vaut ~1e-6, donc ce produit porte un bruit
    // de ~1e-8 A — au-dessus du residu qu'il faut annuler. Newton butait sur
    // ce plancher d'annulation catastrophique et ne concluait jamais.
    // Unite de travail : UNE ASSEMBLEE (jacobienne + devices), pas une iteration
    // de Newton. Une iteration peut en faire jusqu'a neuf — la recherche
    // lineaire re-assemble a chaque repli — et c'est justement dans les regimes
    // difficiles qu'elle replie. Compter les iterations sous-estimait alors le
    // temps d'un facteur 5 a 10, la ou le garde-fou devait mordre. `mutable`
    // parce qu'assemble() est const : c'est un compteur, pas un etat du circuit.
    mutable int work_ = 0;
    // Diagnostic : tentatives de Newton reussies / echouees et sous-pas commis,
    // pour savoir si un regime couteux l'est par ses fronts ou par du battement
    // (echecs, pas divise). Compteurs cumules, remis a zero par l'appelant.
    mutable int nOk_ = 0, nFail_ = 0, nEdgeSplit_ = 0;
    double v_[N] = {0, 0, 0, 0};
    double vPrev_[N] = {0, 0, 0, 0};
    double vStepPrev_[N] = {0, 0, 0, 0};   // etat AVANT le dernier pas commis (predicteur)
    double hPrev_ = 0.0;                   // et sa duree
    int    holdGrow_ = 0;                  // pas reussis restant sans repousse (cf. kHoldAfterFail)
    double sinceEdge_ = 1.0;               // s depuis le dernier front
    bool   slowRefine_ = false;            // raffinement du regime lent (cf. setSlowRefine)
    double outAcc_ = 0.0, tAcc_ = 0.0;     // integrale du collecteur sur le pas courant
    double advanced_ = 0.0;                // temps parcouru par le dernier step()
    float  outAvg_ = 0.0f;
    // Depart chaud de dioR(), par base. `mutable` parce qu'assemble() est
    // const : c'est un cache numerique, pas un etat du circuit.
    mutable double vd86_[2] = {0, 0};
    // Courant total de la branche D1/D2 au dernier assemblage (entrant dans les
    // bases = SORTANT du net 86). C'est ce que le LFO doit voir comme charge sur
    // son net 63 — sans quoi le net 63 est pris comme une source raide, ce qui
    // etait la derniere approximation documentee du chemin du signal.
    mutable double i86_ = 0.0;

    // exp(x/Vt) prolongee LINEAIREMENT au-dela de kVmaxJ. Ecreter la tension
    // d'un simple min() rendrait le residu constant la ou la jacobienne annonce
    // encore une pente : Newton n'aurait plus de direction de descente.
    static void jexp(double vd, double& val, double& slope)
    {
        // Jonction bloquee : sous -80 Vt (-2 V), exp vaut moins de 2e-35. Ajoutee a -1
        // (ebe - 1) elle disparait dans l'arrondi, et sa pente disparait devant
        // kGmin. Calculer ces exponentielles coutait 15 % du solveur sur l'A53,
        // pour un resultat identique au bit pres : bloquees, elles le sont la
        // plupart du temps (sur 6 par assemblee, le claquage ne s'amorce jamais).
        if (vd < kVjOff)  { val = 0.0; slope = 0.0; }
        else if (vd <= kVmaxJ) { val = std::exp(vd / kVt); slope = val; }
        else              { val = kEmax * (1.0 + (vd - kVmaxJ) / kVt); slope = kEmax; }
    }

    // Branche serie diode + resistance, de l'anode (v86) vers la base (vb) :
    // courant I ENTRANT dans la base, et dI/dvb pour la jacobienne. Le point de
    // fonctionnement est un scalaire implicite sur la tension de diode vd,
    //     v86 - vb = vd + R * Is * (exp(vd/Vt) - 1),
    // strictement monotone en vd : Newton y converge sans condition. En
    // inverse, vd -> v86 - vb et I -> -Is : la branche est ouverte.
    //
    // COUT. Cette fonction tourne dans assemble(), donc a chaque iteration du
    // Newton principal, deux fois. Sa premiere version partait de vd = 0,8 V
    // avec 16 iterations et une tolerance de 1e-12 : l'iteration principale
    // passait de ~0,4 us a 3-5 us, et MODE 3 + TONE au cran 68k coutait 27 ms
    // par bloc pour une echeance de 1,3 ms. D'ou :
    //  - un depart ANALYTIQUE : en direct le courant vaut ~e/R, donc
    //    vd ~ Vt*ln(e/(R*Is)) — a 1 mV pres, une ou deux iterations suffisent ;
    //  - un depart CHAUD, la solution precedente de la meme base, quand elle est
    //    plausible (l'astable bouge de quelques mV entre deux sous-pas) ;
    //  - 1e-7 V de tolerance, et 8 iterations au plus.
    //
    // Deuxieme passe : en MODE 3 une assemblee
    // coutait le double de ce qu'elle coute ailleurs (240-265 ns contre
    // 123-136), et ce Newton imbrique en etait la cause — SEQUENTIEL (exp ->
    // division -> correction, rien ne se pipeline), deux fois par assemblee.
    // Trois retouches, aucune ne touche a la physique :
    //  - le log du depart analytique n'est calcule que si le depart chaud
    //    n'est pas plausible. En direct vd ne bouge que de 60 mV par decade de
    //    e : tout warm dans (0, e) est a moins de 0,4 V de la solution, et le
    //    Newton y converge (monotone, pas borne a kDvMax). Avant, le log etait
    //    calcule a chaque appel pour etre jete aussitot ;
    //  - 1e-7 V de tolerance au lieu de 1e-9 : 1e-7 V / 220 k = 5e-13 A, sous
    //    kAbstol du Newton principal, soit un tour de moins presque toujours ;
    //  - le exp final est celui de la derniere iteration quand elle a conclu :
    //    vd a bouge de moins de 1e-7 V depuis, soit 4e-6 en relatif sur ex et
    //    sl, dix mille fois sous kReltolI.
    static void dioR(double v86, double vb, double R, double& warm,
                     double& I, double& dIdvb)
    {
        const double e = v86 - vb;
        const double RIs = R * kIs;
        double vd;
        if (e <= 0.3)                     vd = e;
        else if (warm > 0.0 && warm < e)  vd = warm;
        else                              vd = std::min(e, kVt * std::log(e / RIs));
        double ex, sl; bool done = false;
        for (int k = 0; k < 8; ++k)
        {
            jexp(vd, ex, sl);
            const double f  = e - vd - RIs * (ex - 1.0);
            const double df = -1.0 - RIs * sl / kVt;
            const double dv = std::clamp(-f / df, -kDvMax, kDvMax);
            vd += dv;
            if (std::fabs(dv) < 1e-7) { done = true; break; }
        }
        warm = vd;
        if (!done) jexp(vd, ex, sl);
        const double gd = kIs * sl / kVt;            // conductance de la diode
        I     = kIs * (ex - 1.0);
        dIdvb = -gd / (1.0 + R * gd);                // diode et R en serie
    }

    // Residu (courants sortants) et jacobienne. absSum accumule la somme des
    // |courants| par noeud : un residu ne se juge que relativement au courant
    // que le noeud brasse reellement.
    //
    // ECRIT A PLAT, EXPRES : des variables locales, pas de tableau ni de lambda.
    // C'est la fonction la plus appelee du moteur, et sur un coeur a execution
    // dans l'ordre (A53) chaque chargement depuis la memoire se paie ; la forme
    // generique y coutait 650 ns. L'ORDRE des additions est celui des composants
    // (echelle, diodes, condensateurs 0 a 5, T1, T2) et il faut le garder : il
    // fixe les arrondis, donc la sortie au bit pres en flags stricts.
    void assemble(const double* v, double vm1,
                  double vm2, double v86, bool d86, double vcc,
                  double* F, double J[N][N], double* absSum) const
    {
        ++work_;
        const double b1 = v[B1], c1 = v[C1], b2 = v[B2], c2 = v[C2];
        double f0 = kGmin * b1, f1 = kGmin * c1, f2 = kGmin * b2, f3 = kGmin * c2;
        double a0 = std::fabs(f0), a1 = std::fabs(f1), a2 = std::fabs(f2), a3 = std::fabs(f3);
        double j00 = kGmin, j11 = kGmin, j22 = kGmin, j33 = kGmin;

        {   // les echelles et R9/R13
            const double gc = 1.0 / kRc;
            double i0;
            i0 = gR1_ * (b1 - vm1); f0 += i0; a0 += std::fabs(i0); j00 += gR1_;
            i0 = gR2_ * (b2 - vm2); f2 += i0; a2 += std::fabs(i0); j22 += gR2_;
            i0 = gc * (c1 - vcc);   f1 += i0; a1 += std::fabs(i0); j11 += gc;
            i0 = gc * (c2 - vcc);   f3 += i0; a3 += std::fabs(i0); j33 += gc;
        }
        if (d86)
        {   // D1 -> R10 -> base 1 et D2 -> R11 -> base 2, anodes reunies sur le
            // net 86 dont la tension vient de l'exterieur (le net 63 du LFO,
            // par le bouton TONE, en MODE 3 seulement). De VRAIES diodes : en
            // inverse la branche est ouverte et ne tire rien. L'ancienne forme
            // — une source de Thevenin (V86 - 0,7 V, 220k) ecretee a zero —
            // laissait un 220k vers la masse sur chaque base des que le net 63
            // passait sous 0,7 V. Ce n'est pas ce que fait une diode.
            double I, dI, iSum = 0.0;
            dioR(v86, b1, kRd, vd86_[0], I, dI); f0 -= I; a0 += std::fabs(I); j00 -= dI; iSum += I;
            dioR(v86, b2, kRd, vd86_[1], I, dI); f2 -= I; a2 += std::fabs(I); j22 -= dI; iSum += I;
            i86_ = iSum;        // courant tire au net 86, donc au net 63 du LFO
        }

        // Euler arriere, premier ordre, sauf dans le regime lent raffine ou
        // c'est le trapeze (tr_, voir stepImpl et capCurrent). Le trapeze
        // PARTOUT (second ordre, defaut de SPICE) a ete essaye : fidelite egale
        // au banc, mais 3,5 fois PLUS d'assemblees en regime calme (52 contre
        // 14,7 par image, 72 blocs rognes sur 93) et 1,6 fois plus dans le
        // regime rapide, avec davantage d'echecs de Newton. Sur un basculement
        // regeneratif il sonne numeriquement, et Newton le paie. Il n'est donc
        // employe que la ou Euler arriere se trompe : l'approche lente du
        // seuil, dont il amortissait le mode regeneratif marginal.
        const double g0 = gCap_[0], g1 = gCap_[1], g2 = gCap_[2], g4 = gCap_[4];
        const double ic0 = capI(0, c1 - b2), ic1 = capI(1, c2 - b1), ic2 = capI(2, b1),
                     ic3 = capI(3, b2),      ic4 = capI(4, b1 - c1), ic5 = capI(5, b2 - c2);
        const double n0 = std::fabs(ic0), n1 = std::fabs(ic1), n2 = std::fabs(ic2),
                     n3 = std::fabs(ic3), n4 = std::fabs(ic4), n5 = std::fabs(ic5);
        f1 += ic0; f2 -= ic0; a1 += n0; a2 += n0; j11 += g0; j22 += g0;   // couplage croise
        f3 += ic1; f0 -= ic1; a3 += n1; a0 += n1; j33 += g1; j00 += g1;
        f0 += ic2;            a0 += n2;           j00 += g2;              // vers la masse
        f2 += ic3;            a2 += n3;           j22 += g2;
        f0 += ic4; f1 -= ic4; a0 += n4; a1 += n4; j00 += g4; j11 += g4;
        f2 += ic5; f3 -= ic5; a2 += n5; a3 += n5; j22 += g4; j33 += g4;
        double j01 = -g4, j10 = -g4, j23 = -g4, j32 = -g4;
        const double j12 = -g0, j21 = -g0, j30 = -g1, j03 = -g1;

        // Pas de stockage de charge (TF / TR de Gummel-Poon) : essaye pour la fin
        // de mort, sans effet sur le train de clics ni sur rien d'autre jusqu'a
        // TR = 300 ns. Retire.
        Bjt q;
        bjt(b1, b1 - c1, q);
        f0 += q.Ib; f1 += q.Ic; a0 += q.absB; a1 += q.absC;
        j00 += q.jbb; j01 -= q.jbc; j10 += q.jcb; j11 += q.jcc;
        bjt(b2, b2 - c2, q);
        f2 += q.Ib; f3 += q.Ic; a2 += q.absB; a3 += q.absC;
        j22 += q.jbb; j23 -= q.jbc; j32 += q.jcb; j33 += q.jcc;

        F[B1] = f0; F[C1] = f1; F[B2] = f2; F[C2] = f3;
        absSum[B1] = a0; absSum[C1] = a1; absSum[B2] = a2; absSum[C2] = a3;
        J[B1][B1] = j00; J[B1][C1] = j01; J[B1][B2] = 0.0; J[B1][C2] = j03;
        J[C1][B1] = j10; J[C1][C1] = j11; J[C1][B2] = j12; J[C1][C2] = 0.0;
        J[B2][B1] = 0.0; J[B2][C1] = j21; J[B2][B2] = j22; J[B2][C2] = j23;
        J[C2][B1] = j30; J[C2][C1] = 0.0; J[C2][B2] = j32; J[C2][C2] = j33;
    }

    // Courant du condensateur k pour la tension dv a ses bornes. dvPrev_ est sa
    // tension au pas precedent, posee une fois par tentative, comme gCap_.
    double capI(int k, double dv) const
    {
        return tr_ ? gCap_[k] * (dv - dvPrev_[k]) - iCap_[k]
                   : gCap_[k] * (dv - dvPrev_[k]);
    }

    // Memorise les courants des condensateurs au point commis v_ (pour le trapeze).
    void commitCapCurrents()
    {
        double ic[6];
        ic[0] = capI(0, v_[C1] - v_[B2]);
        ic[1] = capI(1, v_[C2] - v_[B1]);
        ic[2] = capI(2, v_[B1]);
        ic[3] = capI(3, v_[B2]);
        ic[4] = capI(4, v_[B1] - v_[C1]);
        ic[5] = capI(5, v_[B2] - v_[C2]);
        for (int k = 0; k < 6; ++k) iCap_[k] = ic[k];
    }
    double iCap_[6] = {0, 0, 0, 0, 0, 0};
    double gCap_[6] = {0, 0, 0, 0, 0, 0};   // C/h, ou 2C/h au trapeze, pour le pas en cours
    double gR1_ = 0.0, gR2_ = 0.0;          // conductances des deux echelles
    double dvPrev_[6] = {0, 0, 0, 0, 0, 0}; // tension de chaque condensateur au pas precedent
    bool   tr_ = false;              // trapeze pour ce pas (regime lent raffine)

    // NPN, emetteur a la masse. Ce que le transistor ajoute a ses deux noeuds.
    struct Bjt { double Ib, Ic, absB, absC, jbb, jbc, jcb, jcc; };
    static void bjt(double vbe, double vbc, Bjt& q)
    {
        double ebe, dbe, ebc, dbc;
        jexp(vbe, ebe, dbe);
        jexp(vbc, ebc, dbc);
        const double gbe = kIs / kBf * dbe / kVt, gbc = kIs / kBr * dbc / kVt;
        const double gif = kIs * dbe / kVt,       gir = kIs * dbc / kVt;
        const double ibe = kIs / kBf * (ebe - 1.0), ibc = kIs / kBr * (ebc - 1.0);
        // Pas de courant de base non ideal (ISE / NE de Gummel-Poon) : essaye
        // pour obtenir une fin de mort nette, il ne l'obtient qu'a une force qui
        // detruit les hauteurs normales. Re-essaye ensuite avec la
        // valeur du modele SPICE du 2N2222 (14 fA, n 1,3) : il vole 14 uA a la
        // base a 0,7 V et fait basculer la mort du MODE 1 dans le regime couple
        // des 7 V, la ou l'unite descend jusqu'a 41 Hz. Beta constant.
        // Claquage inverse base-emetteur : courant ENTRANT par la base quand
        // vbe < -kBVebo (l'emetteur est a la masse). Meme exponentielle
        // prolongee que les jonctions, pour que Newton ne saute pas.
        double ebv, dbv;
        jexp(-vbe - kBVebo, ebv, dbv);
        const double ibv = kIsBv * (ebv - 1.0), gbv = kIsBv * dbv / kVt;
        const double it  = kIs * (ebe - ebc);
        q.Ib = ibe + ibc - ibv; q.Ic = it - ibc;
        q.absB = std::fabs(q.Ib) + std::fabs(ibv); q.absC = std::fabs(q.Ic);
        // d/dv[b] = d/dvbe + d/dvbc ; d/dv[c] = -d/dvbc ; d(-ibv)/dv[b] = +gbv
        q.jbb = gbe + gbc + gbv; q.jbc = gbc;
        q.jcb = gif - gir - gbc; q.jcc = gir + gbc;
    }

    // Pivot de Gauss sur la matrice 4x4 du circuit. Ecrit en acces constants pour
    // que tout tienne dans des registres : les lignes s'echangent dans des
    // registres au lieu de passer par la memoire, ce que le coeur de l'A53 ne
    // masque pas. Une ligne dont le coefficient est deja nul n'a rien a eliminer,
    // et le circuit en a toujours : B1-B2 et C1-C2 ne se couplent pas.
    static inline __attribute__((always_inline)) void pivot(double m[N][N + 1], int k)
    {
        for (int i = k + 1; i < N; ++i)
            if (std::fabs(m[i][k]) > std::fabs(m[k][k]))
                for (int j = k; j <= N; ++j) std::swap(m[k][j], m[i][j]);
    }
    static inline __attribute__((always_inline)) void eliminate(double m[N][N + 1], int k)
    {
        for (int i = k + 1; i < N; ++i)
            if (m[i][k] != 0.0)
            {
                const double f = m[i][k] / m[k][k];
                for (int j = k + 1; j <= N; ++j) m[i][j] -= f * m[k][j];
            }
    }
    static bool solve4(const double A[N][N], const double* b, double* x)
    {
        double m[N][N + 1] = {
            { A[0][0], A[0][1], A[0][2], A[0][3], b[0] },
            { A[1][0], A[1][1], A[1][2], A[1][3], b[1] },
            { A[2][0], A[2][1], A[2][2], A[2][3], b[2] },
            { A[3][0], A[3][1], A[3][2], A[3][3], b[3] } };
        pivot(m, 0); if (std::fabs(m[0][0]) < 1e-300) return false; eliminate(m, 0);
        pivot(m, 1); if (std::fabs(m[1][1]) < 1e-300) return false; eliminate(m, 1);
        pivot(m, 2); if (std::fabs(m[2][2]) < 1e-300) return false; eliminate(m, 2);
        if (std::fabs(m[3][3]) < 1e-300) return false;
        x[3] = m[3][4] / m[3][3];
        x[2] = (m[2][4] - m[2][3] * x[3]) / m[2][2];
        x[1] = (m[1][4] - m[1][2] * x[2] - m[1][3] * x[3]) / m[1][1];
        x[0] = (m[0][4] - m[0][1] * x[1] - m[0][2] * x[2] - m[0][3] * x[3]) / m[0][0];
        for (int i = 0; i < N; ++i) if (!std::isfinite(x[i])) return false;
        return true;
    }

    bool newton(double h, double r1, double vm1, double r2, double vm2,
                double v86, bool d86, double vcc, const double* start)
    {
        double v[N]; for (int i = 0; i < N; ++i) v[i] = start[i];
        // Les conductances qui ne dependent que du pas et des echelles se posent
        // une fois par tentative. Elles coutaient 14 divisions par assemblee, et
        // la division double est ce que l'A53 fait de plus lent.
        const double m = tr_ ? 2.0 : 1.0;
        gCap_[0] = m * kC / h;   gCap_[1] = m * kCb / h;
        gCap_[2] = m * kCje / h; gCap_[3] = gCap_[2];
        gCap_[4] = m * kCjc / h; gCap_[5] = gCap_[4];
        gR1_ = 1.0 / r1; gR2_ = 1.0 / r2;
        dvPrev_[0] = vPrev_[C1] - vPrev_[B2]; dvPrev_[1] = vPrev_[C2] - vPrev_[B1];
        dvPrev_[2] = vPrev_[B1];              dvPrev_[3] = vPrev_[B2];
        dvPrev_[4] = vPrev_[B1] - vPrev_[C1]; dvPrev_[5] = vPrev_[B2] - vPrev_[C2];
        // Tampons echanges, pas recopies : la recherche lineaire accepte un point
        // en reprenant son residu, et ce sont 24 doubles a chaque fois.
        double F0[N], J0[N][N], A0[N], F1[N], J1[N][N], A1[N];
        double* F = F0; double* A = A0; double (*J)[N] = J0;
        double* nF = F1; double* nA = A1; double (*nJ)[N] = J1;
        double dv[N];
        double w[N];
        // Quand la recherche lineaire accepte un point, elle vient d'y evaluer
        // le residu : c'est exactement ce dont l'iteration suivante a besoin au
        // meme point. Le recalculer doublait le cout du solveur pour rien.
        bool haveF = false;
        for (int it = 0; it < kNewtonMax; ++it)
        {
            if (!haveF)
                assemble(v, vm1, vm2, v86, d86, vcc, F, J, A);
            haveF = false;
            double neg[N]; for (int i = 0; i < N; ++i) neg[i] = -F[i];
            if (!solve4(J, neg, dv)) return false;

            // DEUX criteres, et il faut les deux — c'est ce que fait SPICE.
            // Le residu se juge en ampere, relativement au courant que le noeud
            // brasse : c'est le critere de fond. Mais sur un noeud qui ne brasse
            // presque rien, cette tolerance relative devient inatteignable alors
            // que le circuit EST resolu ; le pas de Newton, lui, y est
            // negligeable. Omettre ce second filet bloquait le solveur pour de
            // bon, quelle que soit la finesse du pas de temps.
            bool okI = true, okV = true;
            for (int i = 0; i < N; ++i)
            {
                if (std::fabs(F[i]) > kReltolI * A[i] + kAbstol) okI = false;
                if (std::fabs(dv[i]) > kReltolV * std::fabs(v[i]) + kVntol) okV = false;
            }
            // On ne commet QUE ce qu'on a verifie. Si c'est le critere en
            // courant qui est satisfait, le residu a ete verifie AU POINT v :
            // c'est donc v qu'il faut garder, pas v+dv — dv peut etre enorme si
            // la jacobienne est mal conditionnee, et le commettre revient a
            // sauter vers un point jamais evalue. C'est ainsi que l'etat partait
            // a -87 V dans un circuit de 12 V, sans retour possible.
            if (okV)
            { for (int i = 0; i < N; ++i) v_[i] = v[i] + dv[i]; return true; }
            if (okI)
            { for (int i = 0; i < N; ++i) v_[i] = v[i]; return true; }


            // Amortissement par la physique : le prolongement lineaire empeche
            // l'exponentielle de deborder mais rend sa pente constante loin du
            // point de fonctionnement, donc le pas de Newton y est enorme.
            double worst = 0.0;
            worst = std::max(worst, std::fabs(dv[B1]));
            worst = std::max(worst, std::fabs(dv[B1] - dv[C1]));
            worst = std::max(worst, std::fabs(dv[B2]));
            worst = std::max(worst, std::fabs(dv[B2] - dv[C2]));
            double lam = (worst <= kDvMax) ? 1.0 : kDvMax / worst;

            // Recherche lineaire : on n'accepte le pas que s'il fait baisser
            // ||F||. Sans ce garde-fou Newton entre en cycle limite pile au
            // basculement, ou la boucle de reaction est regenerative.
            double f0 = 0.0; for (int i = 0; i < N; ++i) f0 += F[i] * F[i];
            bool moved = false;
            for (int b = 0; b < kLineSearchMax; ++b)
            {
                for (int i = 0; i < N; ++i) w[i] = v[i] + lam * dv[i];
                assemble(w, vm1, vm2, v86, d86, vcc, nF, nJ, nA);
                double f1 = 0.0; for (int i = 0; i < N; ++i) f1 += nF[i] * nF[i];
                if (f1 < f0)
                {   // on reprend le residu deja calcule ici : l'iteration
                    // suivante en a besoin au meme point
                    for (int i = 0; i < N; ++i) v[i] = w[i];
                    std::swap(F, nF); std::swap(A, nA); std::swap(J, nJ);
                    moved = true; haveF = true; break;
                }
                lam *= 0.5;
            }
            if (!moved) return false;   // plancher : on rend la main au decoupage
        }
        return false;
    }
};

} // namespace siren
