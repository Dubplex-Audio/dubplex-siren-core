// SPDX-License-Identifier: PolyForm-Shield-1.0.0
#pragma once
// La reverbe a ressort. Sur une sirene dub, un ressort est LA reponse : une
// plaque ou une salle sonneraient « plugin generique ».
//
// Ce qui fait un ressort et pas une salle, c'est la cascade de passe-tout du
// premier ordre en tete. Un ressort transmet les aigus plus vite que les
// graves ; chaque impact s'y etale donc en un glissement de frequence — le
// « boing ». Le reseau de retards qui suit n'apporte que la densite, et ne
// suffirait pas : sans la dispersion on obtient une reverbe quelconque.
//
// LA DENSITE D'ECHOS EST UNE COTE, PAS UN DETAIL. La premiere version de ce
// fichier avait quatre lignes de 23,7 a 53,9 ms et rien dedans : quatre
// premieres reflexions dans les 54 premieres millisecondes, et une enveloppe
// qui gardait ensuite la periodicite de la plus courte. L'oreille les comptait
// une par une — « on entend les delays ». Tous les tests passaient : ils
// bornaient la decroissance, la duree, l'absence de divergence, et aucun ne
// regardait la STRUCTURE de la queue. Deux mesures a 48 kHz, avant puis apres
// (le detail de la methode est ecrit dans test_spring, qui les borne) :
//
//     pics d'enveloppe proeminents, 0-500 ms   13 (26/s)  ->  1 (2/s)
//     temps de melange (NED >= 0,9)              115 ms   ->   25 ms
//
// Trois changements, par ordre de rendement :
//
//   1. UN PASSE-TOUT DANS CHAQUE LIGNE (Schroeder/Dattorro). Chaque passage
//      dans une ligne rend une gerbe d'echos au lieu d'un seul. Un passe-tout
//      est a gain unite : il ne touche pas a la stabilite du reseau, et la
//      decroissance reste gouvernee par les seuls g_[i].
//   2. HUIT LIGNES AU LIEU DE QUATRE, melangees par une Hadamard 8x8
//      normalisee. Elle reste orthogonale, donc le reseau reste stable par
//      construction. Huit lignes font 8^(t/L) chemins au lieu de 4^(t/L).
//   3. UNE CASCADE DE DISPERSION TROIS FOIS PLUS LONGUE : 24 passe-tout,
//      ~47 ms de retard moyen au lieu de 8. Un vrai ressort en a plusieurs
//      dizaines de millisecondes ; 8 ms n'etalaient rien.
//
// La decroissance NE SE REGLE PAS. Le potard est un dosage, et c'est tout :
// un second reglage ferait deux potards la ou on en a demande un.
#include <algorithm>
#include <cmath>
#include <vector>

namespace siren {

class Spring {
public:
    void prepare(double sampleRate)
    {
        fs_ = (float) sampleRate;

        // Vingt-quatre passe-tout etages de 0,4 a 3,5 ms. Des longueurs
        // egales feraient s'additionner leurs retards en un echo net au lieu
        // de les etaler en glissement. L'echelonnement regulier est voulu : un
        // ressort a une courbe de temps de groupe LISSE, et une rampe de
        // longueurs l'approche mieux qu'un tirage disperse.
        //
        // Retard moyen de la cascade = somme des longueurs = 46,8 ms. Un
        // passe-tout de Schroeder de longueur M a pour retard moyen pondere
        // par l'energie exactement M : son energie vaut c^2 a n = 0 et
        // (1-c^2)^2 c^(2k-2) a n = kM, dont la moyenne en k vaut 1. C'est ce
        // qui autorise a additionner ces longueurs sans se mentir — et c'est
        // le meme argument qui sert plus bas pour les passe-tout INTERNES,
        // ou il compte vraiment puisqu'ils allongent la boucle.
        for (int i = 0; i < kAP; ++i)
        {
            const float ms = 0.4f + 3.1f * (float) i / (float) (kAP - 1);
            ap_[i].assign((size_t) std::max(2, (int) std::lround(ms * 0.001f * fs_)), 0.f);
            apW_[i] = 0;
        }

        // Huit lignes de longueurs volontairement INCOMMENSURABLES : aucun
        // rapport entier simple entre elles, pour que les reflexions ne
        // coincident pas et ne sonnent pas metallique.
        //
        // On ne dit PAS "mutuellement premieres" : la primalite porte sur des
        // entiers, or le nombre d'echantillons depend de la frequence
        // d'echantillonnage. Un jeu regle copremier a 48 kHz cesse de l'etre a
        // 44,1. C'est l'incommensurabilite des DUREES qui est recherchee, pas
        // la primalite des comptes, qu'aucun choix de valeurs ne peut tenir
        // aux deux frequences a la fois.
        //
        // L'ETALEMENT 13,7 -> 96,7 ms n'est pas non plus un gout. La densite
        // MODALE d'un reseau de retards vaut la somme de ses retards de
        // boucle, en modes par hertz, et le critere de Jot demande 0,15 x RT60
        // pour ne pas colorer : 0,15 x 1,6 = 0,24 mode/Hz, soit 240 ms de
        // retard total. Les quatre lignes d'origine en faisaient 150 — sous le
        // critere d'un facteur 1,6, ce qui s'entend comme un timbre metallique
        // et creux. Les huit lignes ci-dessous font 366,8 ms, et les
        // passe-tout internes portent la boucle a 417,5 ms : 0,42 mode/Hz.
        static const float kMs[kFDN] =
            { 13.7f, 19.3f, 25.9f, 34.1f, 44.7f, 58.3f, 74.1f, 96.7f };
        // Le passe-tout interne fait environ un septieme de sa ligne : assez
        // long pour etaler, assez court pour ne pas devenir un echo a son
        // tour. Les valeurs ne sont pas ligne/7 exactement — un rapport
        // rationnel simple entre la ligne et son passe-tout ferait retomber
        // les gerbes sur les reflexions de la ligne elle-meme, ce qui revient
        // a ne rien avoir ajoute. Les rapports ci-dessous vont de 7,19 a 7,34.
        static const float kInnerMs[kFDN] =
            {  1.87f, 2.63f,  3.59f,  4.73f,  6.13f,  8.11f, 10.27f, 13.39f };

        for (int i = 0; i < kFDN; ++i)
        {
            const int n = std::max(2, (int) std::lround(kMs[i] * 0.001f * fs_));
            fdn_[i].assign((size_t) n, 0.f);
            fdnW_[i] = 0;

            const int m = std::max(2, (int) std::lround(kInnerMs[i] * 0.001f * fs_));
            inner_[i].assign((size_t) m, 0.f);
            innerW_[i] = 0;

            // g = 10^(-3 * L / kLineDecayTarget), la formule classique du
            // RT60 — avec pour L le retard de la BOUCLE ENTIERE, ligne plus
            // passe-tout interne. Oublier le passe-tout allongerait le RT60
            // d'environ 14 % : c'est la part qu'il prend dans la boucle
            // (50,7 ms sur 417,5), et le passe-tout est a gain unite, donc il
            // ne compense rien de lui-meme.
            const float loopMs = kMs[i] + kInnerMs[i];
            g_[i] = std::pow(10.f, -3.f * (loopMs * 0.001f)
                                   / (kLineDecayTarget * kDecayTrim));
        }

        // Un ressort est un passe-bande : rien sous 80 Hz, rien au-dessus de
        // 4,5 kHz. Sans ces deux bornes la queue serait grave et sifflante a
        // la fois, ce qu'aucun ressort ne fait.
        aHp_ = 1.f - std::exp(-6.2831853f * 80.f   / fs_);
        aLp_ = 1.f - std::exp(-6.2831853f * 4500.f / fs_);
        // Le dosage glisse en 5 ms. Une automation ou un potard tourne a la
        // main livrent des pas a la cadence du bloc ; sur une queue forte,
        // mesure a 48 kHz, un pas de 0,3 a 1,0 faisait un saut de 0,47 entre
        // deux echantillons : un escalier qui s'entend. Le court-circuit a
        // dosage nul reste entier (voir process) : il attend que le lisse
        // soit retombe a 0 EXACTEMENT.
        aMix_ = 1.f - std::exp(-1.f / (0.005f * fs_));

        reset();
    }

    void reset()
    {
        for (int i = 0; i < kAP;  ++i) { std::fill(ap_[i].begin(),  ap_[i].end(),  0.f); apW_[i]  = 0; }
        for (int i = 0; i < kFDN; ++i)
        {
            std::fill(fdn_[i].begin(),   fdn_[i].end(),   0.f); fdnW_[i]   = 0;
            std::fill(inner_[i].begin(), inner_[i].end(), 0.f); innerW_[i] = 0;
        }
        hpS_ = lpS_ = 0.f;
        mixSm_ = mix_;
        // Un objet remis a neuf n'a pas de queue a vider : voir setParameters.
        bypassed_ = false;
    }

    // Le dosage, et rien d'autre — la decroissance ne se regle pas.
    //
    // LA REMONTEE DU DOSAGE VIDE LES LIGNES. Le court-circuit de process()
    // fige l'etat du reseau : tant que le dosage est nul, les retards ne
    // tournent pas, donc ils ne DECROISSENT pas. Sans ce vidage, baisser
    // REVERB a zero, jouer dix secondes et rouvrir le potard relance la queue
    // d'il y a dix secondes. Mesure a 48 kHz (impulsion, 0,2 s de queue,
    // dix secondes a dosage 0, reouverture) :
    //
    //     etat fige            : energie = 0,1035   pic = 0,0172  (-35 dBFS)
    //     etat qui a decru     : energie = 3,3e-39  pic = 2,4e-21 (du silence)
    //
    // -35 dBFS d'audio perime sur un geste ordinaire, pas sur un cas limite.
    //
    // On vide UNE FOIS, a la transition 0 -> >0, et seulement si le
    // court-circuit a reellement ete pris entre-temps : passer de 0,5 a 0 puis
    // revenir a 0,5 sans avoir traite un seul echantillon ne coupe aucune
    // queue, parce qu'aucun temps n'a passe. std::fill sur des vecteurs deja
    // dimensionnes : aucune allocation, et setParameters n'est de toute facon
    // pas process().
    //
    // TapeEcho repond au MEME probleme a l'oppose : sa ligne et ses filtres
    // continuent de tourner quand `on_` est faux, seul le `return on_ ? x + d
    // : x` change. Les deux reponses sont defendables — garder le calcul, ou
    // le couper et nettoyer derriere. Ce qui ne l'etait pas, c'etait de figer
    // l'etat sans le dire.
    void setParameters(float mix)
    {
        const float m = std::clamp(mix, 0.f, 1.f);
        if (m > 0.f && bypassed_) reset();
        mix_ = m;
    }

    float process(float x)
    {
        // A dosage nul on SORT AVANT tout calcul. Deux raisons : la sortie est
        // alors le signal d'entree bit a bit — ce que le test exige — et le
        // reseau ne coute rien quand personne ne l'ecoute.
        //
        // Son PRIX : l'etat ne tourne plus, donc il ne decroit plus. On note
        // qu'on l'a pris, et setParameters vide les lignes a la remontee du
        // dosage — c'est la-bas que l'argument complet est ecrit.
        //
        // Le dosage LISSE (mixSm_) suit la cible en 5 ms et s'y accroche sous
        // 1e-6 ; le court-circuit n'est pris que quand cible et lisse sont a
        // zero, pour que la descente du dosage ne coupe pas la queue net.
        if (mix_ <= 0.f && mixSm_ <= 0.f) { bypassed_ = true; return x; }
        mixSm_ += (mix_ - mixSm_) * aMix_;
        if (std::fabs(mix_ - mixSm_) < 1e-6f) mixSm_ = mix_;

        // Un NaN ou un Inf en entree n'entre pas dans le reseau : il y
        // tournerait jusqu'au prochain vidage (mesure : 47 000 sorties non
        // finies sur la seconde suivante). La sortie, elle, reste x + mouille :
        // c'est au limiteur, en bout de chaine, de ne rien laisser sortir.
        const float xin = std::isfinite(x) ? x : 0.f;

        // --- dispersion : le boing ---------------------------------------
        float v = xin;
        for (int i = 0; i < kAP; ++i)
        {
            const size_t n = ap_[i].size();
            const float  d = ap_[i][(size_t) apW_[i]];
            const float  in = v - kApG * d;
            ap_[i][(size_t) apW_[i]] = in;
            if (++apW_[i] == (int) n) apW_[i] = 0;
            v = d + kApG * in;
        }

        // --- densite : huit lignes, un passe-tout dans chacune ------------
        //
        // La sortie de chaque ligne traverse SON passe-tout avant d'aller au
        // melangeur et a la sortie. C'est ce qui transforme chaque tour de
        // boucle en gerbe : une ligne nue rend une reflexion par tour, une
        // ligne passe-tout en rend une trainee geometrique.
        float d[kFDN];
        for (int i = 0; i < kFDN; ++i)
        {
            const float raw = fdn_[i][(size_t) fdnW_[i]];

            const float ad  = inner_[i][(size_t) innerW_[i]];
            const float ain = raw - kInnerG * ad;
            inner_[i][(size_t) innerW_[i]] = ain;
            if (++innerW_[i] == (int) inner_[i].size()) innerW_[i] = 0;
            d[i] = ad + kInnerG * ain;
        }

        // Hadamard 8x8 normalisee, en trois etages de papillons : 24
        // additions et 8 multiplications, contre 64 multiplications-additions
        // pour le produit matriciel ecrit naivement. Orthogonale, donc elle
        // redistribue l'energie sans en creer : c'est ce qui rend la
        // decroissance gouvernee par g_[] seul, et le reseau stable par
        // construction.
        const float a0 = d[0] + d[1], a1 = d[0] - d[1];
        const float a2 = d[2] + d[3], a3 = d[2] - d[3];
        const float a4 = d[4] + d[5], a5 = d[4] - d[5];
        const float a6 = d[6] + d[7], a7 = d[6] - d[7];

        const float b0 = a0 + a2, b1 = a1 + a3, b2 = a0 - a2, b3 = a1 - a3;
        const float b4 = a4 + a6, b5 = a5 + a7, b6 = a4 - a6, b7 = a5 - a7;

        const float mixed[kFDN] = {
            (b0 + b4) * kHad, (b1 + b5) * kHad, (b2 + b6) * kHad, (b3 + b7) * kHad,
            (b0 - b4) * kHad, (b1 - b5) * kHad, (b2 - b6) * kHad, (b3 - b7) * kHad };

        // Les signes d'injection et de prelevement ne sont pas les memes, et
        // aucun des deux n'est le vecteur tout-a-un. Injecter en phase dans
        // les huit lignes et resommer en phase revient a n'exciter qu'une
        // direction propre du melangeur : les huit lignes repartent ensemble a
        // chaque tour, ce qui redonne exactement le battement periodique qu'on
        // cherche a supprimer. Deux motifs de signes differents cassent cette
        // coincidence pour le prix de rien.
        static constexpr float kIn [kFDN] = { 1.f, -1.f,  1.f,  1.f, -1.f,  1.f, -1.f, -1.f };
        static constexpr float kOut[kFDN] = { 1.f,  1.f, -1.f,  1.f, -1.f, -1.f,  1.f, -1.f };

        const float inj = v * kInject;
        float wet = 0.f;
        for (int i = 0; i < kFDN; ++i)
        {
            fdn_[i][(size_t) fdnW_[i]] = inj * kIn[i] + mixed[i] * g_[i];
            if (++fdnW_[i] == (int) fdn_[i].size()) fdnW_[i] = 0;
            wet += d[i] * kOut[i];
        }
        wet *= kInject;   // 1/sqrt(8) des deux cotes : injection a energie unite

        // --- la bande du ressort ------------------------------------------
        hpS_ += (wet - hpS_) * aHp_;  wet -= hpS_;
        lpS_ += (wet - lpS_) * aLp_;  wet  = lpS_;

        // AJOUT, pas fondu enchaine : a 0 la sortie est seche bit a bit (voir
        // le court-circuit plus haut) et a fond le son direct ne disparait
        // pas, ce qui n'aurait aucun sens pour un instrument.
        return x + wet * mixSm_ * kWetGain;
    }

private:
    static constexpr int   kAP      = 24;
    static constexpr int   kFDN     = 8;
    static constexpr float kApG     = 0.72f;  // dispersion d'entree
    static constexpr float kInnerG  = 0.5f;   // passe-tout interne aux lignes
    static constexpr float kHad     = 0.35355339f;   // 1/sqrt(8)
    static constexpr float kInject  = 0.35355339f;   // idem, a l'entree et a la sortie
    // La cible passee a la formule de gain des lignes. Elle est TENUE : le
    // reseau decroit a la pente demandee. Ce qui est plus long que 1,6 s,
    // c'est le temps qu'il faut pour que la queue retombe a -60 dB DEPUIS
    // L'IMPULSION, et c'est autre chose. Mesures a 48 kHz :
    //
    //   pente de la decroissance (T30, ISO 3382)   1,597 s   (-0,2 %)
    //   -60 dB sous le PIC, compte depuis le pic   1,65  s
    //   -60 dB sous le pic, compte depuis l'entree 1,775 s   (+11 %)
    //
    // L'ecart tient au REMPLISSAGE du reseau, pas a un allongement de la
    // decroissance : le pic d'energie arrive vers 125 ms, contre 75 ms avec
    // les quatre lignes d'avant. Ce remplissage plus lent est le prix des
    // huit lignes et de la dispersion allongee — c'est exactement ce qui rend
    // la queue dense, et ca ne se paie qu'une fois, au debut. Le passe-bande,
    // lui, ne change pas la pente : c'est ce que la mesure T30 verifie, et
    // test_spring la borne a +/-20 %, le critere de la spec.
    //
    // Regler la decroissance a l'oreille se fait donc ici en sachant qu'il
    // faut ajouter le dixieme de seconde de remplissage.
    static constexpr float kLineDecayTarget = 1.6f;   // secondes, FIXE
    // Le recalage, et d'ou il vient. La formule du RT60 suppose une boucle a
    // retard UNIQUE. Le passe-tout interne n'en a plus un : il etale l'energie
    // sur M, 2M, 3M... Or exp(-a/L) est CONVEXE en L, donc une distribution de
    // longueurs decroit moins vite que sa moyenne (Jensen). Mesure a 48 kHz,
    // sans recalage : T30 = 1,649 s, soit +3,0 % — dans les +/-20 % de la spec,
    // mais gratuitement faux. Ce facteur ramene la mesure sur la cible.
    static constexpr float kDecayTrim = 0.970f;
    // Compense les deux 1/sqrt(8) : a dosage 1 le mouille arrive au meme ordre
    // de grandeur que le sec. Recale avec la structure a huit lignes — huit
    // lignes rendent plus d'energie que quatre a injection egale, et la valeur
    // ci-dessous ramene le niveau du mouille a celui mesure avant le
    // changement : RMS du mouille sur trois secondes de queue, impulsion
    // unitaire, 1,69e-3 avant / 1,67e-3 apres. Le PIC du mouille, lui, tombe
    // de 0,049 a 0,024 — une queue dense n'a plus de pointe qui emerge, et
    // c'est le meme fait que mesure le test de densite.
    static constexpr float kWetGain = 2.05f;

    std::vector<float> ap_[kAP];
    int   apW_[kAP] = {};
    std::vector<float> fdn_[kFDN];
    int   fdnW_[kFDN] = {};
    std::vector<float> inner_[kFDN];
    int   innerW_[kFDN] = {};
    float g_[kFDN] = {};

    float fs_ = 48000.f;
    float aHp_ = 0.f, aLp_ = 0.f, hpS_ = 0.f, lpS_ = 0.f;
    float mix_ = 0.f;
    float mixSm_ = 0.f, aMix_ = 0.f;   // le dosage lisse, et son coefficient
    // « le court-circuit a ete pris depuis la derniere fois que le reseau a
    // tourne ». Lu par setParameters, et par lui seul.
    bool  bypassed_ = false;
};

} // namespace siren
