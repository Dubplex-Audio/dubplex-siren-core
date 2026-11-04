// SPDX-License-Identifier: PolyForm-Shield-1.0.0
// Le solveur generique, verifie sur des circuits dont la reponse se calcule a
// la main — les memes que ceux du solveur de reference — puis recoupe contre
// AstableCore sur l'astable audio, qui est deja valide.
#include "Mna.hpp"
#include "AstableCore.hpp"
#include <cstdio>
#include <cmath>
#include <vector>

static int failures = 0;
static void check(bool ok, const char* m)
{ std::printf("%s%s\n", ok ? "ok:   " : "FAIL: ", m); if (!ok) ++failures; }
static void near(double a, double b, double tol, const char* m)
{ const bool ok = std::fabs(a - b) <= tol;
  std::printf("%s%s  (%.4f, attendu %.4f +-%.4f)\n", ok ? "ok:   " : "FAIL: ", m, a, b, tol);
  if (!ok) ++failures; }

using Sm = siren::Mna<12, 16, 16, 8, 4>;

int main()
{
    // 1. Diviseur resistif : 10 V, deux 1k -> 5 V. La fuite GMIN (1 GOhm par
    //    noeud) tire le point milieu de 2,5 uV vers la masse : c'est assume.
    {
        Sm m;
        const int vcc = m.fixedNode(10.0), mid = m.node();
        m.addR(vcc, mid, 1000.0); m.addR(mid, Sm::GND, 1000.0);
        m.clear(); m.run(1e-3);
        near(m.v(mid), 5.0, 1e-4, "diviseur resistif 1k/1k");
    }

    // 2. Charge RC : 10 V, 1k, 1uF -> tau = 1 ms, on attend 10*(1-1/e).
    {
        Sm m;
        const int vcc = m.fixedNode(10.0), out = m.node();
        m.addR(vcc, out, 1000.0); m.addC(out, Sm::GND, 1e-6);
        m.clear();
        for (int i = 0; i < 1000; ++i) m.run(1e-6);
        near(m.v(out), 10.0 * (1.0 - std::exp(-1.0)), 0.02, "charge RC a t = tau");
    }

    // 3. Diode en direct : 10 V a travers 10k, on attend Vt*ln(I/Is).
    {
        Sm m;
        const int vcc = m.fixedNode(10.0), a = m.node();
        m.addR(vcc, a, 10000.0); m.addD(a, Sm::GND);
        m.clear(); m.run(1e-3);
        const double i = (10.0 - m.v(a)) / 10000.0;
        near(m.v(a), 0.025852 * std::log(i / 1e-14), 0.01, "diode : Vd = Vt*ln(I/Is)");
    }

    // 4. Astable de manuel : Vcc 5 V, Rc 1k, Rb 100k, C 10 nF -> 721 Hz par la
    //    formule ideale ; l'ecart vient de la tension de saturation et du beta
    //    fini, que la formule ignore.
    {
        Sm m;
        const int vcc = m.fixedNode(5.0);
        const int b1 = m.node(), c1 = m.node(), b2 = m.node(), c2 = m.node();
        m.addR(vcc, b1, 100e3); m.addR(vcc, c1, 1000.0);
        m.addR(vcc, b2, 100e3); m.addR(vcc, c2, 1000.0);
        m.addQ(b1, c1, Sm::GND, +1); m.addQ(b2, c2, Sm::GND, +1);
        m.addC(c1, b2, 10e-9); m.addC(c2, b1, 10e-9);
        m.clear();
        m.setV(b1, 0.7); m.setV(c1, 0.1); m.setV(b2, -3.0); m.setV(c2, 5.0);
        std::vector<double> y;
        const double h = 5e-6;
        bool ok = true;
        for (int i = 0; i < 1600; ++i) { ok &= m.run(h); y.push_back(m.v(c1)); }
        check(ok, "astable : le pas adaptatif traverse les basculements");
        double lo = 1e9, hi = -1e9;
        for (size_t i = y.size() / 4; i < y.size(); ++i)
        { lo = std::min(lo, y[i]); hi = std::max(hi, y[i]); }
        check(hi - lo > 3.0, "astable : le collecteur bascule");
        const double mid = 0.5 * (lo + hi);
        int cr = 0;
        for (size_t i = y.size() / 4 + 1; i < y.size(); ++i)
            if (y[i - 1] <= mid && y[i] > mid) ++cr;
        const double f = cr / ((y.size() - y.size() / 4) * h);
        near(f, 721.0, 120.0, "astable : frequence 1/(2*0,693*Rb*C)");
    }

    // 5. Recoupement : le meme astable audio monte avec le solveur generique
    //    doit donner ce que donne AstableCore, qui est deja valide contre la
    //    simulation de reference. Deux implementations independantes du meme
    //    circuit : si elles concordent, ce n'est pas un hasard.
    {
        auto freqGeneric = [](double vm) {
            Sm m;
            const int vcc = m.fixedNode(12.2), vs = m.fixedNode(vm);
            const int b1 = m.node(), c1 = m.node(), b2 = m.node(), c2 = m.node();
            m.addR(vs, b1, 470e3); m.addR(vs, b2, 470e3);
            // Les valeurs viennent d'AstableCore, elles ne sont PAS recopiees :
            // ce test compare deux IMPLEMENTATIONS, pas deux circuits. Recopier
            // les constantes les laissait diverger en silence — c'est arrive
            // quand C est passee de 9,5 a 9,30 nF, et le test a echoue en
            // accusant le solveur alors que seuls les composants differaient.
            using A = siren::AstableCore;
            m.addR(vcc, c1, A::kRc); m.addR(vcc, c2, A::kRc);
            m.addQ(b1, c1, Sm::GND, +1, A::kBf, A::kIs, A::kBr, A::kCje, A::kCjc, A::kBVebo, A::kIsBv);
            m.addQ(b2, c2, Sm::GND, +1, A::kBf, A::kIs, A::kBr, A::kCje, A::kCjc, A::kBVebo, A::kIsBv);
            m.addC(c1, b2, A::kC); m.addC(c2, b1, A::kCb);
            m.clear();
            m.setV(b1, -0.20); m.setV(c1, 12.2); m.setV(b2, 0.65); m.setV(c2, 0.10);
            const double h = 1.0 / 192000.0;
            std::vector<double> y;
            for (int i = 0; i < 48000; ++i) { m.run(h); y.push_back(m.v(c2)); }
            const size_t st = y.size() / 3;
            double lo = 1e9, hi = -1e9;
            for (size_t i = st; i < y.size(); ++i)
            { lo = std::min(lo, y[i]); hi = std::max(hi, y[i]); }
            if (hi - lo < 1.0) return 0.0;
            const double mid = 0.5 * (lo + hi);
            int cr = 0;
            for (size_t i = st + 1; i < y.size(); ++i)
                if (y[i - 1] <= mid && y[i] > mid) ++cr;
            return cr / ((y.size() - st) * h);
        };
        auto freqCore = [](float vm) {
            siren::AstableCore c; c.reset();
            const float dt = 1.0f / 192000.0f;
            std::vector<float> y;
            for (int i = 0; i < 48000; ++i)
            { c.step(dt, 470e3f, vm, 470e3f, vm, 0.0f, false, 12.2f); y.push_back(c.out()); }
            const size_t st = y.size() / 3;
            float lo = 1e9f, hi = -1e9f;
            for (size_t i = st; i < y.size(); ++i)
            { lo = std::min(lo, y[i]); hi = std::max(hi, y[i]); }
            if (hi - lo < 1.0f) return 0.0;
            const float mid = 0.5f * (lo + hi);
            int cr = 0;
            for (size_t i = st + 1; i < y.size(); ++i)
                if (y[i - 1] <= mid && y[i] > mid) ++cr;
            return cr / ((double) (y.size() - st) * (double) dt);
        };
        for (double vm : { 11.42, 7.11, 1.52 })
        {
            const double a = freqGeneric(vm), b = freqCore((float) vm);
            std::printf("      Vm %5.2f V : generique %.0f Hz, AstableCore %.0f Hz\n", vm, a, b);
            char msg[96];
            std::snprintf(msg, sizeof(msg),
                          "les deux implementations concordent a Vm = %.2f V", vm);
            check(b > 0.0 && std::fabs(a - b) < 0.10 * b, msg);
        }
    }

    std::printf(failures == 0 ? "ALL OK\n" : "%d FAILURE(S)\n", failures);
    return failures == 0 ? 0 : 1;
}
