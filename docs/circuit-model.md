# The circuit model

`dubplex-siren-core` models a dub siren from measurements of a real NJD-style
unit: readings taken on the board with a multimeter, and recordings of the unit.
It is a circuit model, not a synth recipe. Net numbers and pads below refer to
that board. Some values could not be read on the board and were deduced from the
recordings, notably the capacitors; the code says which ones, and marks its one
assumption about the output load.

- **Main oscillator**: the two-transistor astable (T1/T2, C1 and C2 10 nF nominal,
  10.22 and 10.32 nF deduced, R9/R13 4k7) is **solved as a circuit**, not evaluated from a formula. Nodal
  analysis, backward Euler, damped Newton per sample, Ebers-Moll transistors —
  `dsp/AstableCore.hpp`. The wail, the stall, the duty-cycle asymmetry and the
  fast end-of-death regime all fall out of it; none of them is written anywhere.

  The formula it replaces, `t = R*C*ln((Vm + Vcc - 2*Vbe)/(Vm - Vbe))`, is a
  ONE-node model: it assumes the base recovers through the ladder toward Vm
  while the opposite collector stays pinned at the floor. That holds only while
  the ladder can supply enough current to *hold* saturation — and it matches to
  1 % there. Below that it is wrong by a factor of five, in the wrong direction:

  | Vm | measured | formula |
  |---|---|---|
  | 11.42 V | 3.57 ms | 3.54 ms |
  | 7.11 V | 5.00 ms | 4.96 ms |
  | 1.52 V | **2.78 ms** | 12.70 ms |

  The circuit *speeds up* where the formula says it should slow to silence: the
  collector releases and drags the opposite base up through the coupling cap, so
  the period is set by Rc*C (47 us) instead of R*C (4.7 ms). That is the MODE 4
  death "coming back up".
- **LFO**: also solved as a circuit — T3/T4 (PNP), C5/C6, D3/D4, the R14/R21
  collector loads and the SPEED ladder, in `dsp/LfoCore.hpp`. The three
  hard-coded half-periods are gone; the rates come out of the real resistor
  values (47k/22k/10k). The unit runs at 1.74 / 3.66 / 8.12 Hz, measured from a recording. At SPEED OFF the second
  pole releases T3's base while T4 keeps its 10k — that asymmetry, not a rule,
  is what freezes the square.
- **Sweep network**: D4/R15/R18/R20/R16 into C4 (100uF nominal, 125 uF deduced), solved with the LFO.
  The five RC coefficients and the "if SIREN then… else if the square is high
  then…" cascade are gone. Discharge time constants land within ~6 % of the
  ohmmeter readings taken on the board (49.93k and 45.80k).
- **Buttons**: TRIG and the OFF/ON rocker are the SAME control — both are
  wired across the SIREN ON header in the V+ rail (momentary vs latched pure
  power). Holding TRIG plays the full siren; release = 2 ms cut, unless the
  rocker still holds power, in which case the authentic falling discharge
  tail sounds. SIREN (green) ties the LFO square node to V+: fast C4 charge and
  a frozen LFO, so a steady top note while held. TONE (yellow) ties node X through
  R16 to net 11, which is V+: a slow charge of the sweep cap. TONE and SIREN do NOT
  power the unit: pressed alone they are silent, like the hardware. The D1/D2
  routing is MODE position 3.
- **MODE** is a real 3-pole switch, read off the board.
  Two poles carry the rate ladders; the third distributes V+ — to half B's
  ladder in positions 1 and 2, and through D1/D2 to BOTH bases in position 3:

  | MODE | half A | half B | what you hear |
  |---|---|---|---|
  | 1 | sweep (C4) | fixed V+ | the wail |
  | 2 | square | fixed V+ | two-tone with stall gaps |
  | 3 | square | square (same net, via J1) | square, up and down, sharp — never chops |
  | 4 | sweep | sweep | pure, and the longest death |

  Earlier versions had 3 = sweep/square and 4 = fixed/fixed, which made MODE 4's
  death *impossible* — yet it is the mode whose death is longest on the unit.

  In MODE 3 the third pole also puts V+ on net 86, so D1/D2 hang 220k (R10/R11)
  from V+-0.7 onto BOTH bases, in parallel with the rate ladder. Thevenin: it
  moves the modulation voltage *and* the base resistor. That is why MODE 3
  alternates between two pitches instead of chopping — when the square drops to
  zero the base stays above the conduction threshold. MODE 2, where net 86
  floats, does chop.

  Net 11 — the third pole's common, and the TONE button's far side — carries V+
  from the loom. Measured, not inferred: 0 Ω from pad 69 to V+ with MODE on 4,
  where the switch isolates net 11 from the rest of the board.
- **Output**: collector -> **R12 100k** -> C3 (10uF) -> OUT -> pot. R12 matters
  twice: it attenuates by 10k/110k = 1/11 (the collector swings 12 V, the real
  output is 1.1 V — which is why a headphone plugged straight in hears almost
  nothing), and C3 therefore sees 110k, not 10k. The pole is at **0.145 Hz, not
  1.6 Hz**: tau = 1.1 s. That is the weight heard after a stall, and it lasted
  ten times too little. This board carries no 15k and no 100nF, so the 2x 106 Hz
  network earlier versions modelled does not exist here: earlier versions
  removed low end the real unit keeps. The pole stays only to block the DC the
  astable parks on when it stalls, bleeding it over ~1.1 s — the weight heard
  after the death.
- **PWR LED**: simply lit whenever the unit is powered. This board has NO led
  driver — the inventory holds only T1-T4 and the single 100k (R12) sits in
  series with the output; the LED is wired between the switched V+ and ground
  with its limiting resistor inline in the loom.
- **Supply**: `vcc` parameter, default **12.2 V** — the unit is played on a 12 V DC
  adapter (12.2 V measured at the board, no sag under load). Vcc sets both the
  astable period and the sweep-cap ceiling, so it governs how deep the wail
  dives: at 12 V the bottom of the sweep sits ~2.3 semitones below 9 V, and the
  death lasts ~13 % longer.

## Front panel (the parameters mirror the unit's faceplate)

- **TONE** rotary 1/2/3/4 — osc-rate ladder S5 (470k/220k/100k/68k), read off the
  board silkscreen (R1-R4 half A, R5-R8 half B); the faceplate is graduated 1-4
- **MODE** rotary 1/2/3/4 — S4 routing: wail / two-tone with gaps / square up and down / pure sweep
- **SPEED** rotary 1/2/3/OFF — LFO ladder S3; OFF = LFO frozen (manual play)
- **VOLUME** — output level
- **SIREN** button (green, pads 17 // 126) — ties the LFO square node to V+:
  fast C4 charge via D4+R15 (0.1 s) **and** freezes the LFO. Steady top note.
- **TONE** button (yellow, pads 149 // 69) — ties node X, through R16 (10k), to
  net 11, which is V+: a slow charge of the sweep cap. Both are silent unless
  powered. Net 11 is also the MODE switch's third-pole common, which is what
  makes the TONE button act in a different place in each MODE.

  These two were **swapped** in earlier versions; the traced board settles it.
- **TRIG** button — momentary power (SIREN ON rail); **OFF/ON rocker** — latched TRIG (identical)
- **PWR LED** — lit whenever the unit is powered (`ledLevel` output param)

**There are no fine-tune parameters left on the plate.** Every knob that had no
counterpart on the board has been removed — `model`, `collTau`, `fallTau`,
`vbe`, `amount`, `drive`, `edge`, then `charge`, `discharge`, `diode` and
`capRatio`. Those last four were resistances and a capacitor whose values are
*read off the board* (R16 10k, R20 47k, R10/R11 220k, C1 = C2 = 10 nF nominal): keeping
a knob on a known value dresses a certainty up as a setting.

That rule governs **the plate**, which replicates a measured object. It does not
govern the effects, which are *outboard*: a tape echo and a spring reverb you
patch behind the unit. Their four controls model no component and claim none.

What remains is the faceplate itself, plus two honest extras:

- **`vcc`** — the supply is not on the board. Measured 12.2 V at the terminals
  on a 12 V DC adapter, no sag under load; that is the default.
- **`oversample`** — not a tone control but how finely the circuit is sampled.
  The collector's edges (~20 ns) are faster than any sample rate can hold, so
  this is the only band-limiting in the engine.

## Effects panel (outboard, not on the board)

- **REVERB** — spring reverb, mix only. A cascade of dispersive allpasses gives
  the chirp a spring makes (highs travel faster than lows); a feedback network of
  eight lines behind it supplies density. Decay is **not a knob**, deliberately.
- **ECHO** — switch, no mix knob: the mix is fixed at **100 % dry + 100 % wet**,
  so switching it on puts the repeats at the level of the direct sound.
- **FEEDBACK** — repeats. Loop gain is 1.05x the knob, so the top of its travel
  self-oscillates, bounded by the tanh in the write path.
- **TIME** — 40 to 800 ms, logarithmic. Smoothed with a 60 ms time constant, so
  moving it *glides* the pitch the way a tape speeding up does.

The echo is a tape echo, and what makes it one is where the filters sit: a
150 Hz highpass, a 3 kHz lowpass and a soft clip live **inside the loop, on the
write path**. Repeat 1 is filtered once, repeat 2 twice, repeat 3 three times —
they darken and thicken as they recede. The same filters on the output path
would give identical repeats at falling level, which is a digital delay.

A **brickwall limiter** closes the chain at -0.3 dBFS with 1.5 ms of lookahead.
It exists because of the 100 % + 100 % choice above: the sum clips as soon as
repeats overlap the direct sound. Fixing that with a hidden gain would be a mix
control smuggled back in. **It costs latency** — 72 samples at 48 kHz, reported
to the host along with the engine's own — and it stays active even with both
effects off, because a latency that changed with the switch would make the host
restart the component on every flick.
