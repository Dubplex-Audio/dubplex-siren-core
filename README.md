# dubplex-siren-core

The DSP of a dub siren, and nothing else: a C++20 header-only library with no
framework and no dependency. It is modelled from measurements of a real NJD-style
unit. The oscillators are not a synth recipe: the circuit is **solved at every
sample** (nodal analysis, backward Euler, damped Newton, Ebers-Moll transistors).
A tape echo, a spring reverb and a brickwall limiter follow it.

This is the engine behind Dubplex Audio Siren-1. It is here so that people can
build their own siren hardware. It has no user interface and no plug-in wrapper.

See [`docs/circuit-model.md`](docs/circuit-model.md) for the model.

## Build and test

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure   # siren_engine takes several minutes
```

`examples/render.cpp` is the smallest complete program: it plays one second of
siren into a WAV file. To use the library, add this repository as a subdirectory
and link `dubplex_siren_core`, or put `dsp/` and `params/` on your include path.

| File | Role |
|---|---|
| `SirenEngine.hpp` | The whole siren: controls in, audio out. Start here. |
| `FxChain.hpp` | Echo, reverb and limiter, called after the engine. |
| `params/ParameterMetadata.hpp` | The list of parameters, their order and their ranges. It sees nothing of the circuit. |
| `AstableCore.hpp`, `LfoCore.hpp`, `Mna.hpp` | The solved circuit and its solver. |
| `TapeEcho.hpp`, `Spring.hpp`, `Limiter.hpp` | The effects. |
| `MachineFactor.hpp`, `DenormalGuard.hpp` | Real-time safety. |

## Licence

Source-available: [PolyForm Shield 1.0.0](LICENSE), with an
[additional permission](LICENSE-ADDITIONAL.md) for people who build their own.
This summary is not the licence; the licence decides. Whoever receives a copy,
including inside a device, must also receive the licence terms.

| You want to | |
|---|---|
| Read, change and compile it for yourself | Yes |
| Play paid shows with what you built, sell the music you make | Yes |
| Publish your hardware designs, or patches and forks of this code alone | Yes |
| Give or lend what you built to people you know personally, for free and privately, with these licence terms | Yes |
| Sell a product that does not compete with Siren-1 or OpenPreamp | Yes |
| Sell a siren, a kit, or a unit built to order containing this code, even at cost | **No** |
| Publish a competing plug-in, app or firmware built on this code, even for free | **No** |
| Publish the complete, ready-to-build source of such a product | **No** |

Contributions are welcome under the [CLA](CLA.md). See
[`CONTRIBUTING.md`](CONTRIBUTING.md).
