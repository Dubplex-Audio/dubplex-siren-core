// SPDX-License-Identifier: PolyForm-Shield-1.0.0
#pragma once
// Le LFO (T3/T4) et le reseau du sweep, resolus comme un circuit.
//
// Ce que ca remplace : trois constantes de demi-periode en dur
// (0,326 / 0,152 / 0,069 s) et une cascade de regles ecrites a la main pour
// choisir vers quoi C4 se charge — « si SIREN alors le rail avec tau rapide,
// sinon si le carre est haut alors le rail avec tau moyen, sinon... ». Toutes
// ces regles etaient des DESCRIPTIONS du circuit. Ici il n'y en a plus une
// seule : on pose les composants et l'integration fait le reste.
//
// Le montage, releve sur la carte :
//   T3 : base net 90, collecteur net 1, emetteur V+ (par le cavalier J2)
//   T4 : base net 2,  collecteur net 14, emetteur V+
//   C6 10,1 uF : net 90 <-> net 14      couplage croise
//   C5 10 uF   : net 2  <-> net 91      l'autre couplage, via D3
//   R14 1k : net 91 -> masse            charge de collecteur de T3, via D3
//   R21 1k : net 14 -> masse            charge de collecteur de T4
//   D3 : net 1 -> net 91               (cathode net 91 — voir plus bas)
//   D4 : net 1 -> net 7                 (cathode net 7) charge de C4
//   R15 1k  : net 7 -> net 24 (C4+)
//   R18 2k2 : net 7 -> net 34
//   R20 47k : net 34 -> masse           court-circuite par le SPEED en 1/2/3
//   R16 10k : net 7 -> net 63           le bouton TONE relie net 63 a V+
//   C4 100 uF : net 24 -> masse         le sweep
//
// Les deux echelles du SPEED tirent les bases vers la masse : 47k/22k/10k, et
// a OFF la base de T3 est LACHEE tandis que celle de T4 garde son 10k. C'est
// cette dissymetrie, et non une regle, qui fige le LFO a l'arret.
#include "Mna.hpp"
#include <algorithm>

namespace siren {

class LfoCore {
public:
    // 11 noeuds, 12 resistances, 8 condensateurs (dont 4 de jonction),
    // 2 diodes, 2 transistors.
    using S = Mna<14, 18, 10, 4, 2>;

    void build(float vcc)
    {
        m_ = S();
        vp_  = m_.fixedNode(vcc);
        n90_ = m_.node(); n1_  = m_.node(); n2_  = m_.node(); n14_ = m_.node();
        n91_ = m_.node(); n7_  = m_.node(); n24_ = m_.node(); n34_ = m_.node();
        n63_ = m_.node();

        m_.addR(n91_, S::GND, 1000.0);          // R14
        m_.addR(n14_, S::GND, 1000.0);          // R21
        m_.addR(n7_,  n24_,   1000.0);          // R15
        m_.addR(n7_,  n34_,   2200.0);          // R18
        m_.addR(n34_, S::GND, 47000.0);         // R20, lu sur la carte
        m_.addR(n7_,  n63_,   10000.0);         // R16, lu sur la carte
        rLad3_ = m_.addR(n90_, S::GND, 47000.0);   // echelle SPEED, base T3
        rLad4_ = m_.addR(n2_,  S::GND, 47000.0);   // echelle SPEED, base T4
        swPole3_ = m_.addR(n34_, S::GND, kRon);    // 3e pole du SPEED
        swSiren_ = m_.addR(n1_,  vp_,    kRoff);   // bouton SIREN (vert)
        swTone_  = m_.addR(n63_, vp_,    kRoff);   // bouton TONE (jaune)
        // Charge du net 63 par la branche D1/D2 de l'astable audio (MODE 3, TONE
        // enfonce) : un puits de courant, pose par l'appelant a partir du courant
        // reellement tire. Le net 63 est alimente par R16 10k, jamais en haute
        // impedance : un puits y est bien pose, contrairement au noeud du carre.
        iLoad63_ = m_.addI(n63_);

        // D3 : anode net 1, cathode net 91.
        //
        // Ce sens avait d'abord ete DEDUIT contre le releve, qui donnait
        // l'inverse : dans l'autre sens D3 bloque des que le collecteur de T3
        // monte, C5 pend a un noeud isole que R14 tire a la masse, le couplage
        // croise T3 -> T4 n'existe pas, et deux solveurs independants donnaient
        // 0,00 Hz — alors que la machine balaie.
        //
        // Verifie depuis sur la carte : le trait noir est sur la pastille 174
        // (net 91), pas 19. Le releve initial etait faux, il est
        // corrige. La deduction et la mesure concordent.
        m_.addD(n1_, n91_);                     // D3
        m_.addD(n1_, n7_);                       // D4
        // C5 / C6 : 8,83 et 8,91 uF. Nominal 10 uF, valeur DEDUITE d'un
        // enregistrement (les trois vitesses de l'unite
        // en 31 s). L'unite ne sera plus demontee : ces deux electrolytiques ne
        // seront jamais mesurees, mais on n'en a pas besoin.
        //   cadences de l'unite : 1,74 / 3,66 / 8,12 Hz
        //   modele a 10 uF      : 1,53 / 3,25 / 7,14 Hz
        //   rapport             : x1,135 / x1,127 / x1,137
        // Un facteur IDENTIQUE sur trois barreaux d'echelle differents ne peut
        // pas venir des resistances : il vient de l'element commun aux trois.
        // Deux verifications internes le confirment — la dispersion des trois
        // rapports est de 0,4 %, et les cadences mesurees suivent 1 : 2,10 :
        // 4,67 la ou l'echelle 47k/22k/10k predit 1 : 2,14 : 4,70. L'echelle
        // SPEED est donc au nominal sans avoir ete mesuree. 10 uF / 1,133 =
        // 8,83. Un electrolytique a -12 % est dans sa tolerance courante.
        m_.addC(n90_, n14_, 8.91e-6);            // C6
        m_.addC(n2_,  n91_, 8.83e-6);            // C5
#ifndef SIREN_C4
// 125 uF et non 100 (nominal). DEDUITE de la duree des morts, comme C1/C2 et
// C5/C6 l'ont ete de hauteurs et de cadences — l'unite ne sera plus demontee.
// Duree entre le lacher de SIREN et l'arret de l'oscillation, MODE 1 SPEED OFF :
//     cran        1        3        4
//     unite     14,03 s   8,95 s   7,65 s
//     100 uF    12,07     7,26     5,89     (systematiquement ~20 % trop court)
//     125 uF    15,09     9,06     7,38
// Et le rapport cyclique suit alors l'unite tout au long de la mort au cran 4 :
// 59/70/83/88/84 % contre 56/69/82/87/91 %. Un electrolytique a +25 % est dans
// sa tolerance ; les series a -20/+80 % sont courantes.
#define SIREN_C4 125e-6
#endif
        m_.addC(n24_, S::GND, SIREN_C4);         // C4
        // C4 est un electrolytique aluminium : polarise en INVERSE il conduit.
        // Une diode de la masse (anode) vers C4+ (cathode) l'empeche de
        // descendre loin sous zero. C'est ce qui BORNE le sifflement du MODE 4 :
        // sur l'unite les plateaux sont 1,90 / 2,19 / 2,69 / 3,07 kHz aux quatre
        // TONE ; sans clamp le modele les etale jusqu'a 4 kHz au cran 4.
        // Is = 1e-22 : le coude est vers -1,1 V, la ou un electrolytique
        // commence reellement a conduire en inverse (la couche d'oxyde de la
        // cathode tient ~1 V). L'ancien 1e-13 (coude a -0,5 V) avait ete regle
        // SANS le claquage base-emetteur de l'astable audio et
        // compensait en partie son absence ; avec lui c'est 1e-22 qui
        // pose les crans 2/3/4 a 2,15 / 2,62 / 2,91 kHz (-1 / -3 / -5 %). Le
        // cran 1 ne touche pas le clamp (son echelle 467k pompe trop peu) : il
        // se pose a 1,69 kHz contre 1,90, encore en montee a 24 s comme l'unite.
        m_.addD(S::GND, n24_, 1e-22);
        m_.addQ(n90_, n1_,  vp_, -1, 180.0);     // T3, PNP
        // T4, PNP : base = net 2, collecteur = net 14, emetteur = V+.
        // L'ordre des pins du releve ne dit PAS l'electronique :
        // les SIX permutations ont ete testees contre les cadences mesurees. Une
        // seule les reproduit — celle-ci : 1,600 / 3,200 / 7,000 Hz contre
        // 1,53 / 3,25 / 7,14 mesurees, et tau de decharge 5,02 s. Les cinq
        // autres donnent 0 Hz ou 2,8/5,4 Hz. C'est aussi la seule qui forme un
        // astable croise de manuel : les deux bases sur leur echelle, les deux
        // collecteurs charges par 1k, et le couplage croise par C6 (T4.C ->
        // T3.B) et D3+C5 (T3.C -> T4.B). L'ordre lu sur le releve ("e","b","c")
        // donne un autre cablage ; il ne tient pas a l'epreuve des cadences.
        m_.addQ(n2_,  n14_, vp_, -1, 180.0);     // T4, PNP
        // Chargement de l'astable audio. Ce n'est PAS une source de courant :
        // l'echelle est une resistance vers la base, dont le courant depend de
        // la tension du noeud. Une source constante sur le noeud du carre, qui
        // devient haute impedance des que T3 se bloque, n'a nulle part ou
        // debiter — le solveur doit alors pousser le noeud a des tensions
        // absurdes, et le pas explose (39,8 ms contre 1,2 ms, mesure).
        // A ZERO, pas a 0,65 V : ces deux noeuds portent la tension de base que
        // l'echelle voit, et une carte eteinte les a a zero comme tout le reste.
        // Ce sont des noeuds FIXES, donc la mise a zero de l'etat plus bas ne
        // les touche pas — il faut les poser ici. L'appelant les reecrit a
        // chaque bloc par setLadder, mais l'etat de depart doit etre coherent.
        vbA_ = m_.fixedNode(0.0); vbB_ = m_.fixedNode(0.0);
        gA24_ = m_.addR(n24_, vbA_, kRoff); gA1_ = m_.addR(n1_, vbA_, kRoff);
        gB24_ = m_.addR(n24_, vbB_, kRoff); gB1_ = m_.addR(n1_, vbB_, kRoff);

        // Etat HORS TENSION : tout a zero, y compris le rail. C'est l'etat reel
        // d'une carte non alimentee, et c'est ce qui garantit un silence EXACT
        // tant que le rocker n'est pas mis. Le LFO demarre ensuite sur la montee
        // du rail : son circuit est franchement dissymetrique (D3, D4, charges
        // de collecteur differentes), il n'a pas besoin qu'on l'amorce.
        m_.setFixed(vp_, 0.0);
        for (int i = 0; i < m_.numNodes(); ++i) m_.setV(i, 0.0);
        vcc_ = vcc;
    }

    // pos 1/2/3/4 en facade, 4 = OFF.
    void setSpeed(int pos)
    {
        static const double lad[4] = { 47000.0, 22000.0, 10000.0, 10000.0 };
        // pos est 1-BASE (la facade est graduee 1 2 3 OFF) : il faut donc
        // pos-1 pour indexer. Sans ce decalage, SPEED 1 prenait le 22k du
        // SPEED 2 et toute l'echelle glissait d'un cran.
        const int p = std::min(3, std::max(0, pos - 1));
        m_.setR(rLad4_, lad[p]);
        // A OFF la base de T3 est lachee : c'est le contact mort du selecteur.
        m_.setG(rLad3_, (p == 3) ? 0.0 : 1.0 / lad[p]);
        // Le 3e pole met le net 34 a la masse en 1/2/3 et l'ouvre a OFF, ce qui
        // insere R20 47k dans le chemin de decharge de C4. C'est ce qui donne
        // les 3,177 kOhm et 49,93 kOhm lus a l'ohmmetre.
        m_.setR(swPole3_, (p == 3) ? kRoff : kRon);
    }

    // Le bouton SIREN a sa seconde pastille directement sur V+ : il agit dans
    // toutes les positions. Le bouton TONE, lui, passe par le NET 11, et le
    // net 11 est le commun du 3e pole du MODE. Il n'atteint donc V+ que la ou
    // ce pole l'y mene : en MODE 1 et 2 (contacts 15 et 59). En MODE 3 le pole
    // le pose sur le net 86 (anodes de D1/D2) — le bouton alimente alors les
    // bases de l'astable audio au lieu de charger C5. Cette lecture a ete
    // abandonnee : voir le corps de la fonction.
    void setButtons(bool siren, bool tone, int mode = 0)
    {
        m_.setR(swSiren_, siren ? kRon : kRoff);
        // LE NET 11 EST A V+ PAR SA PROPRE PISTE, pas par le 3e pole du MODE.
        // C'est ce que la hauteur du MODE 3 impose (voir SirenEngine et le test
        // 23l) : le pole y pose le net 11 sur le net 86, et le net 86 doit s'y
        // trouver au rail. Le bouton TONE, qui relie le net 11 au net 63, agit
        // donc dans les QUATRE positions du MODE et non plus dans deux.
        // PREDICTION A ECOUTER : appuyer sur TONE doit s'entendre aussi en
        // MODE 3 et en MODE 4. L'hypothese precedente (V+ sur les contacts 15
        // et 59) predisait l'inverse.
        (void) mode;
        m_.setR(swTone_,  tone ? kRon : kRoff);
    }

    float toneNode() const { return (float) m_.v(n63_); }   // net 63, apres R16
    // Diagnostic : tensions des noeuds internes, pour comparer a l'arbitre.
    void dbgNodes(double* out) const {
        out[0]=m_.v(n1_); out[1]=m_.v(n90_); out[2]=m_.v(n2_); out[3]=m_.v(n14_);
        out[4]=m_.v(n91_); out[5]=m_.v(n7_); out[6]=m_.v(n24_); out[7]=m_.v(n34_); }
    // Tous les noeuds a moins de eps de la masse : le LFO n'a plus rien a resoudre.
    bool settled(double eps) const {
        for (int i = 0; i < m_.numNodes(); ++i) if (std::fabs(m_.v(i)) > eps) return false;
        return true; }
    void  setNet86Load(double amps) { m_.setI(iLoad63_, amps); }   // > 0 = sort du net 63
    int   work() const { return m_.work(); }                 // assemblees cumulees
    // Pas borne : voir Mna::run(dt, budget, advanced). Renvoie le temps parcouru.
    // Le budget vaut pour l'APPEL ENTIER, pas pour chaque sous-pas de kHMax :
    // Mna::run compte depuis son propre depart, donc lui repasser le meme
    // budget a chaque tour le multipliait par dt / kHMax — 5 fois a 64 images,
    // 341 fois a 4096 — et la tranche « plafonnee » du LFO ne plafonnait rien.
    double stepBudgeted(double dt, int budget) {
        const int w0 = m_.work();
        double done = 0.0;
        while (dt - done > 1e-12)
        {
            const int left = budget - (m_.work() - w0);
            if (left <= 0) break;              // budget epuise : on rend la main
            const double h = std::min(dt - done, kHMax);
            double adv = 0.0; m_.run(h, left, &adv); done += adv;
            if (adv < h - 1e-12) break;
        }
        return done; }
    void  resetWork()  { m_.resetWork(); }

    void setVcc(float v) { m_.setFixed(vp_, v); vcc_ = v; }


    // Une moitie de l'astable audio, vue depuis le LFO : son echelle (de
    // conductance g) tire vers sa base (a vbase). dest : 0 = le sweep,
    // 1 = le carre, autre = nulle part (la moitie est sur V+ fixe).
    // Sans ce chargement, la decharge de C4 ne dependrait plus du TONE — or
    // elle en depend, et c'est mesure a l'ohmmetre.
    void setLadder(int half, int dest, double g, double vbase)
    {
        const int r24 = half ? gB24_ : gA24_, r1 = half ? gB1_ : gA1_;
        m_.setFixed(half ? vbB_ : vbA_, vbase);
        m_.setG(r24, dest == 0 ? g : 0.0);
        m_.setG(r1,  dest == 1 ? g : 0.0);
    }

    // PAS INTERNE PLAFONNE. Le LFO est resolu une fois par bloc, donc son pas
    // valait la duree du bloc (1,3 ms a 64 images, 5,3 ms a 256, 10,7 a 512).
    // A pas grossier, Euler arriere AMORTIT le basculement regeneratif de
    // l'astable, qui se couche alors sur son equilibre continu : le LFO se fige
    // (apres SIREN, ou sur un changement de SPEED) et il faut le perturber pour
    // le relancer. Le son dependait donc de la taille de tampon de l'hote, ce
    // qui n'a aucun sens physique. On decoupe en sous-pas de kHMax au plus.
    bool step(double dt) {
        bool ok = true;
        while (dt > 0.0) { const double h = std::min(dt, kHMax); ok = m_.run(h) && ok; dt -= h; }
        return ok; }


    float square() const { return (float) m_.v(n1_); }   // net 1, collecteur T3
    float sweep()  const { return (float) m_.v(n24_); }  // net 24, C4+
    float nodeX()  const { return (float) m_.v(n7_); }

private:
#ifndef SIREN_KRON
#define SIREN_KRON 1.0
#endif
#ifndef SIREN_LFO_HMAX
#define SIREN_LFO_HMAX 0.25e-3
#endif
    static constexpr double kHMax = SIREN_LFO_HMAX;   // pas interne max du LFO
    static constexpr double kRon  = SIREN_KRON;   // resistance d'un contact ferme
    static constexpr double kRoff = 1e12;

    S m_;
    int vp_ = 0, n90_ = 0, n1_ = 0, n2_ = 0, n14_ = 0, n91_ = 0;
    int n7_ = 0, n24_ = 0, n34_ = 0, n63_ = 0;
    int iLoad63_ = 0;
    int rLad3_ = 0, rLad4_ = 0, swPole3_ = 0, swSiren_ = 0, swTone_ = 0;
    int vbA_ = 0, vbB_ = 0, gA24_ = 0, gA1_ = 0, gB24_ = 0, gB1_ = 0;

    double vcc_ = 12.2;
};

} // namespace siren
