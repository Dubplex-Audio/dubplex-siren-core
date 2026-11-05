// SPDX-License-Identifier: PolyForm-Shield-1.0.0
#pragma once
// Petit solveur de circuit generique : nodal, Euler implicite, Newton amorti,
// pas adaptatif. Transcription en C++ du solveur Python de reference, valide par 24 tests
// sur des circuits dont la reponse se calcule a la main.
//
// Tout est en double et sans allocation : les tableaux sont dimensionnes a la
// compilation, pour pouvoir tourner dans un thread audio.
//
// Les correctifs durement acquis sont TOUS ici, et chacun a coute cher :
//   - jexp() prolonge l'exponentielle lineairement plutot que d'ecreter la
//     tension : ecreter rend le residu constant la ou la jacobienne annonce
//     encore une pente, et Newton perd sa direction de descente ;
//   - la convergence se juge sur le residu en AMPERE, relativement au courant
//     que le noeud brasse — avec un repli sur la petitesse du pas, sans lequel
//     un noeud a tres haute impedance bloque le solveur pour de bon ;
//   - recherche lineaire sur ||F||, sinon Newton entre en cycle limite pile au
//     basculement d'un multivibrateur ;
//   - les transistors portent leurs capacites de jonction : sans elles, un
//     multivibrateur n'a aucune vitesse de basculement et la branche de
//     solution atteint un pli qu'aucun pas de temps ne franchit ;
//   - le pas est VRAIMENT adaptatif (avancer, resserrer, repousser). Un
//     decoupage uniforme impose le meme pas partout, alors que la transition
//     demande la nanoseconde quand le reste du cycle se contente de la
//     microseconde : mesure faite, il echoue sur 19944 pas sur 20000.
#include "FastExp.hpp"
#include <cmath>
#include <algorithm>

namespace siren {

template <int NMAX, int RMAX, int CMAX, int DMAX, int QMAX>
class Mna {
public:
    static constexpr int GND = 0;

    // ---- construction ------------------------------------------------------
    int node()            { return n_++; }
    int fixedNode(double v) { const int i = n_++; fixed_[i] = true; fx_[i] = v; return i; }
    void setFixed(int i, double v) { fx_[i] = v; }

    int addR(int a, int b, double ohms)
    { res_[nr_] = {a, b, 1.0 / ohms}; return nr_++; }
    void setR(int h, double ohms) { res_[h].g = 1.0 / ohms; }
    void setG(int h, double g)    { res_[h].g = g; }

    void addC(int a, int b, double farads) { cap_[nc_++] = {a, b, farads}; }

    // Source de courant SORTANT du noeud. Sert a rendre un chargement calcule
    // ailleurs : l'echelle de l'astable audio tire son courant de base sur le
    // noeud du sweep, et ce chargement est mesurable — c'est lui qui rend la
    // decharge de C4 dependante du TONE (4,58 s contre 3,36 s a l'ohmmetre).
    int addI(int a) { isrc_[ni_] = {a, 0.0}; return ni_++; }
    void setI(int h, double amps) { isrc_[h].i = amps; }
    void addD(int anode, int cathode, double is = 1e-14)
    { dio_[nd_++] = {anode, cathode, is}; }

    // type : +1 NPN, -1 PNP. cje/cjc valent pour un petit signal courant.
    // bvebo : tension de claquage inverse base-emetteur (0 = pas de claquage),
    // isbv : son courant de saturation. Voir AstableCore::kBVebo.
    void addQ(int b, int c, int e, int type, double bf = 200.0,
              double is = 1e-14, double br = 4.0,
              double cje = 4.5e-12, double cjc = 4.0e-12,
              double bvebo = 0.0, double isbv = 1e-12)
    {
        bjt_[nq_++] = {b, c, e, (double) type, is, bf, br, bvebo, isbv};
        if (cje > 0.0) addC(b, e, cje);
        if (cjc > 0.0) addC(b, c, cjc);
    }

    // ---- etat --------------------------------------------------------------
    void clear() { for (int i = 0; i < n_; ++i) v_[i] = fixed_[i] ? fx_[i] : 0.0; }
    void setV(int i, double val) { v_[i] = val; }
    double v(int i) const { return v_[i]; }
    int    numNodes() const { return n_; }

    // Avance de dt en resserrant le pas la ou Newton coince.
    int  work() const { return work_; }
    void keepStepAcrossCuts(bool on) { keepH_ = on; }
    void resetWork()  { work_ = 0; }


    bool run(double dt, double hmin = 1e-14)
    { double adv; return run(dt, 1 << 30, &adv, hmin); }

    // Avec BUDGET (en assemblees) : on s'arrete au dernier sous-pas COMMIS des
    // que le budget est depasse, et *advanced recoit le temps reellement
    // parcouru (< dt si coupe). L'etat est alors celui d'un instant anterieur,
    // physiquement valide — le circuit a simplement un peu de retard, que
    // l'appelant reporte sur le bloc suivant. Pour un circuit dont les
    // constantes se comptent en dizaines de ms, un retard d'une fraction de
    // bloc est inaudible ; c'est tout autre chose qu'un oscillateur gele.
    bool run(double dt, int budget, double* advanced, double hmin = 1e-14)
    {
        const int w0 = work_;
        // Coupe au dernier appel, on reprend au pas qu'il avait atteint. Repartir
        // du pas plein, c'est redescendre par moities jusqu'a l'echelle du front a
        // chaque appel : avec un budget serre, tout y passerait et le circuit
        // n'avancerait plus.
        double t = 0.0, h = (resumeH_ > 0.0) ? std::min(dt, resumeH_) : dt;
        resumeH_ = 0.0;
        bool ok = false;
        for (int guard = 0; guard < 400000; ++guard)
        {
            if (t >= dt * (1.0 - 1e-12)) { ok = true; break; }
            if (work_ - w0 >= budget) { if (keepH_) resumeH_ = h; break; }   // coupe : etat commis
            h = std::min(h, dt - t);
            for (int i = 0; i < n_; ++i) vp_[i] = v_[i];
            if (newton(h)) { t += h; h = std::min(dt, 1.5 * h); }
            else           { h *= 0.5; if (h < hmin) break; }
        }
        if (advanced) *advanced = t;
        return ok;
    }

private:
    struct R { int a, b; double g; };
    struct C { int a, b; double c; };
    struct D { int a, k; double is; };
    struct Q { int b, c, e; double ty, is, bf, br, bv, isbv; };
    struct I { int a; double i; };

    static constexpr double kVt     = 0.025852;
    static constexpr double kGmin   = 1e-9;
    static constexpr double kVmaxJ  = 1.0;
    static constexpr double kVjOff = -80.0 * kVt; // en-deca, exp(v/Vt) ne compte plus
    static constexpr double kDvMax  = 0.5;
    static constexpr double kReltolI = 1e-3;
    static constexpr double kAbstol  = 1e-12;
    static constexpr double kReltolV = 1e-6;
    static constexpr double kVntol   = 1e-9;
    // exp(kVmaxJ / kVt), calculee au chargement. Pas une statique locale dans
    // jexp() : celle-ci porte un garde d'initialisation (__cxa_guard_acquire,
    // un verrou possible) dont le premier passage a lieu dans le premier
    // process(), sur le fil audio.
    static inline const double kEmax = std::exp(kVmaxJ / kVt);
#ifndef SIREN_MNA_NEWTON
#define SIREN_MNA_NEWTON 40
#endif
#ifndef SIREN_MNA_LS
#define SIREN_MNA_LS 5
#endif
    static constexpr int    kNewtonMax     = SIREN_MNA_NEWTON;
    static constexpr int    kLineSearchMax = SIREN_MNA_LS;

    int n_ = 1, nr_ = 0, nc_ = 0, nd_ = 0, nq_ = 0, ni_ = 0;   // le noeud 0 est la masse
    bool   fixed_[NMAX] = {true};
    double fx_[NMAX] = {0.0};
    double v_[NMAX] = {0.0}, vp_[NMAX] = {0.0};
    mutable int work_ = 0;             // assemblees depuis resetWork()
    bool   keepH_ = false;             // voir run() : reprendre au pas atteint apres une coupe
    double resumeH_ = 0.0;
    R res_[RMAX]; C cap_[CMAX]; D dio_[DMAX]; Q bjt_[QMAX]; I isrc_[4];

    static void jexp(double vd, double& val, double& slope)
    {
        // Jonction bloquee : voir AstableCore::jexp, meme coupure, meme raison.
        if (vd < kVjOff)  { val = 0.0; slope = 0.0; }
        else if (vd <= kVmaxJ) { val = jexpValue(vd / kVt); slope = val; }
        else              { val = kEmax * (1.0 + (vd - kVmaxJ) / kVt); slope = kEmax; }
    }

    void assemble(const double* v, double h, double* F, double J[NMAX][NMAX],
                  double* A) const
    {
        ++work_;                       // une assemblee = l'unite de travail
        for (int i = 0; i < n_; ++i)
        {
            F[i] = kGmin * v[i]; A[i] = std::fabs(F[i]);
            for (int j = 0; j < n_; ++j) J[i][j] = 0.0;
            J[i][i] = kGmin;
        }
        for (int k = 0; k < nr_; ++k)
        {
            const R& r = res_[k];
            const double i0 = r.g * (v[r.a] - v[r.b]);
            F[r.a] += i0; F[r.b] -= i0;
            A[r.a] += std::fabs(i0); A[r.b] += std::fabs(i0);
            J[r.a][r.a] += r.g; J[r.a][r.b] -= r.g;
            J[r.b][r.b] += r.g; J[r.b][r.a] -= r.g;
        }
        for (int k = 0; k < ni_; ++k)
        {   // constante : pas de contribution a la jacobienne
            F[isrc_[k].a] += isrc_[k].i;
            A[isrc_[k].a] += std::fabs(isrc_[k].i);
        }
        for (int k = 0; k < nc_; ++k)
        {
            const C& c = cap_[k];
            const double g = c.c / h;
            const double ic = g * ((v[c.a] - v[c.b]) - (vp_[c.a] - vp_[c.b]));
            F[c.a] += ic; F[c.b] -= ic;
            A[c.a] += std::fabs(ic); A[c.b] += std::fabs(ic);
            J[c.a][c.a] += g; J[c.a][c.b] -= g;
            J[c.b][c.b] += g; J[c.b][c.a] -= g;
        }
        for (int k = 0; k < nd_; ++k)
        {
            const D& d = dio_[k];
            double ex, sl; jexp(v[d.a] - v[d.k], ex, sl);
            const double i0 = d.is * (ex - 1.0), g = d.is * sl / kVt;
            F[d.a] += i0; F[d.k] -= i0;
            A[d.a] += std::fabs(i0); A[d.k] += std::fabs(i0);
            J[d.a][d.a] += g; J[d.a][d.k] -= g;
            J[d.k][d.k] += g; J[d.k][d.a] -= g;
        }
        for (int k = 0; k < nq_; ++k)
        {
            const Q& q = bjt_[k];
            double ebe, dbe, ebc, dbc;
            jexp(q.ty * (v[q.b] - v[q.e]), ebe, dbe);
            jexp(q.ty * (v[q.b] - v[q.c]), ebc, dbc);
            const double gbe = q.is / q.bf * dbe / kVt, gbc = q.is / q.br * dbc / kVt;
            const double gif = q.is * dbe / kVt,        gir = q.is * dbc / kVt;
            const double ibe = q.is / q.bf * (ebe - 1.0), ibc = q.is / q.br * (ebc - 1.0);
            const double it  = q.is * (ebe - ebc);
            const double Ib = ibe + ibc, Ic = it - ibc, Ie = -(ibe + it);
            F[q.b] += q.ty * Ib; F[q.c] += q.ty * Ic; F[q.e] += q.ty * Ie;
            A[q.b] += std::fabs(Ib); A[q.c] += std::fabs(Ic); A[q.e] += std::fabs(Ie);
            // vbe = ty(vb-ve), vbc = ty(vb-vc) : le double ty s'annule, la
            // jacobienne est donc la meme pour un NPN et un PNP.
            const int   rows[3] = { q.b, q.c, q.e };
            const double dbeR[3] = { gbe, gif, -(gbe + gif) };
            const double dbcR[3] = { gbc, -gir - gbc, gir };
            for (int r = 0; r < 3; ++r)
            {
                J[rows[r]][q.b] += dbeR[r] + dbcR[r];
                J[rows[r]][q.e] -= dbeR[r];
                J[rows[r]][q.c] -= dbcR[r];
            }
            if (q.bv > 0.0)
            {   // claquage inverse base-emetteur : courant de l'emetteur vers la
                // base quand vbe < -bv (en convention du type)
                double ebv, dbv;
                jexp(-q.ty * (v[q.b] - v[q.e]) - q.bv, ebv, dbv);
                const double ibv = q.isbv * (ebv - 1.0), g = q.isbv * dbv / kVt;
                F[q.b] -= q.ty * ibv; F[q.e] += q.ty * ibv;
                A[q.b] += std::fabs(ibv); A[q.e] += std::fabs(ibv);
                J[q.b][q.b] += g; J[q.b][q.e] -= g;
                J[q.e][q.b] -= g; J[q.e][q.e] += g;
            }
        }
        // Noeuds imposes : la ligne devient v - consigne.
        for (int i = 0; i < n_; ++i)
            if (fixed_[i])
            {
                F[i] = v[i] - fx_[i];
                for (int j = 0; j < n_; ++j) J[i][j] = 0.0;
                J[i][i] = 1.0;
            }
    }

    bool solve(double A[NMAX][NMAX], const double* rhs, double* x) const
    {
        double M[NMAX][NMAX + 1];
        for (int i = 0; i < n_; ++i)
        { for (int j = 0; j < n_; ++j) M[i][j] = A[i][j]; M[i][n_] = rhs[i]; }
        for (int k = 0; k < n_; ++k)
        {
            int p = k;
            for (int i = k + 1; i < n_; ++i)
                if (std::fabs(M[i][k]) > std::fabs(M[p][k])) p = i;
            if (std::fabs(M[p][k]) < 1e-300) return false;
            if (p != k) for (int j = k; j <= n_; ++j) std::swap(M[k][j], M[p][j]);
            for (int i = k + 1; i < n_; ++i)
            {
                const double f = M[i][k] / M[k][k];
                if (f == 0.0) continue;
                for (int j = k; j <= n_; ++j) M[i][j] -= f * M[k][j];
            }
        }
        for (int i = n_ - 1; i >= 0; --i)
        {
            double s = M[i][n_];
            for (int j = i + 1; j < n_; ++j) s -= M[i][j] * x[j];
            x[i] = s / M[i][i];
        }
        for (int i = 0; i < n_; ++i) if (!std::isfinite(x[i])) return false;
        return true;
    }

    bool newton(double h)
    {
        double v[NMAX], F[NMAX], A[NMAX], dv[NMAX], neg[NMAX];
        double J[NMAX][NMAX];
        double nF[NMAX], nA[NMAX], nJ[NMAX][NMAX], w[NMAX];
        for (int i = 0; i < n_; ++i) v[i] = v_[i];
        // Memes trois reglages que le noyau audio, valides la au hertz pres :
        // 40 iterations au plus (une tentative qui n'a pas converge en 40 ne
        // convergera pas en 150, c'est le pas qu'il faut resserrer), 5 replis
        // de recherche lineaire, et REUTILISATION du residu que la recherche
        // lineaire vient d'evaluer au point accepte — le recalculer doublait le
        // cout. Mesure avant : 3491 assemblees dans un bloc STABLE (MODE 4,
        // SPEED 3), 8755 a un passage SPEED -> OFF, pour un circuit de 11 noeuds.
        bool haveF = false;
        for (int it = 0; it < kNewtonMax; ++it)
        {
            if (!haveF) assemble(v, h, F, J, A);
            haveF = false;
            for (int i = 0; i < n_; ++i) neg[i] = -F[i];
            if (!solve(J, neg, dv)) return false;

            bool okI = true, okV = true;
            for (int i = 0; i < n_; ++i)
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
            { for (int i = 0; i < n_; ++i) v_[i] = v[i] + dv[i]; return true; }
            if (okI)
            { for (int i = 0; i < n_; ++i) v_[i] = v[i]; return true; }


            double worst = 0.0;
            for (int k = 0; k < nd_; ++k)
                worst = std::max(worst, std::fabs(dv[dio_[k].a] - dv[dio_[k].k]));
            for (int k = 0; k < nq_; ++k)
            {
                worst = std::max(worst, std::fabs(dv[bjt_[k].b] - dv[bjt_[k].e]));
                worst = std::max(worst, std::fabs(dv[bjt_[k].b] - dv[bjt_[k].c]));
            }
            double lam = (worst <= kDvMax) ? 1.0 : kDvMax / worst;

            double f0 = 0.0; for (int i = 0; i < n_; ++i) f0 += F[i] * F[i];
            bool moved = false;
            for (int b = 0; b < kLineSearchMax; ++b)
            {
                for (int i = 0; i < n_; ++i) w[i] = v[i] + lam * dv[i];
                assemble(w, h, nF, nJ, nA);
                double f1 = 0.0; for (int i = 0; i < n_; ++i) f1 += nF[i] * nF[i];
                if (f1 < f0)
                {   // le point est accepte ET son residu vient d'etre evalue :
                    // on le garde pour l'iteration suivante au lieu de le refaire
                    for (int i = 0; i < n_; ++i)
                    {
                        v[i] = w[i]; F[i] = nF[i]; A[i] = nA[i];
                        for (int j = 0; j < n_; ++j) J[i][j] = nJ[i][j];
                    }
                    moved = true; haveF = true; break;
                }
                lam *= 0.5;
            }
            if (!moved) return false;
        }
        return false;
    }
};

} // namespace siren
