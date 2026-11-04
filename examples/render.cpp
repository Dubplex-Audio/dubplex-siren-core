// SPDX-License-Identifier: PolyForm-Shield-1.0.0
// Le plus petit programme complet : une seconde de sirene dans un fichier WAV.
// Usage : render_example sortie.wav
// Sort en erreur si le rendu est silencieux, ce qui en fait aussi un test.
#include "FxChain.hpp"
#include "SirenEngine.hpp"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

static constexpr float kFs = 48000.0f;
static constexpr int kBlock = 64;

static void u32(std::FILE* f, uint32_t v)
{
    for (int i = 0; i < 4; ++i) std::fputc((v >> (8 * i)) & 0xff, f);
}

static void u16(std::FILE* f, uint32_t v)
{
    for (int i = 0; i < 2; ++i) std::fputc((v >> (8 * i)) & 0xff, f);
}

// WAV mono, flottant 32 bits.
static bool writeWav(const char* path, const std::vector<float>& x)
{
    std::FILE* f = std::fopen(path, "wb");
    if (!f) return false;
    const uint32_t bytes = (uint32_t) (x.size() * sizeof(float));
    std::fwrite("RIFF", 1, 4, f); u32(f, 36 + bytes); std::fwrite("WAVEfmt ", 1, 8, f);
    u32(f, 16); u16(f, 3); u16(f, 1); u32(f, (uint32_t) kFs); u32(f, (uint32_t) kFs * 4);
    u16(f, 4); u16(f, 32);
    std::fwrite("data", 1, 4, f); u32(f, bytes);
    std::fwrite(x.data(), sizeof(float), x.size(), f);
    return std::fclose(f) == 0;
}

int main(int argc, char** argv)
{
    if (argc != 2) { std::fprintf(stderr, "usage: %s output.wav\n", argv[0]); return 2; }

    siren::SirenEngine engine;
    siren::FxChain fx;
    engine.prepare(kFs, kBlock);
    fx.prepare(kFs);

    siren::SirenEngine::Params p;
    p.power = true;
    p.volume = 0.8f;
    engine.setParameters(p);

    std::vector<float> out;
    float left[kBlock], right[kBlock];
    float* buffers[2] = { left, right };
    float peak = 0.0f;
    for (int b = 0; b < (int) kFs / kBlock; ++b)
    {
        for (int i = 0; i < kBlock; ++i) left[i] = right[i] = 0.0f;
        engine.process(buffers, 2, kBlock);
        fx.process(buffers, 2, kBlock);
        for (int i = 0; i < kBlock; ++i) peak = std::fmax(peak, std::fabs(left[i]));
        out.insert(out.end(), left, left + kBlock);
    }
    if (!writeWav(argv[1], out)) { std::fprintf(stderr, "cannot write %s\n", argv[1]); return 1; }
    std::printf("%s: peak %.3f\n", argv[1], peak);
    return peak > 0.001f ? 0 : 1;
}
