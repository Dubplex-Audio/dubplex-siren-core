# Contributing

`dubplex-siren-core` is **source-available** under the PolyForm Shield License
1.0.0, with the additional permission in `LICENSE-ADDITIONAL.md`. Read them once
before contributing: they are short.

1. **Sign the CLA** (`CLA.md`). Your first pull request gets a comment that tells
   you the sentence to post. The e-mail address of your commits must be linked to
   your GitHub account, otherwise the signature cannot be matched to them.
2. **Keep the rule of the repository:** the headers see only the C++ standard
   library. No framework, no dependency.
3. **A change to the sound needs a reason measured on the circuit.** The tests
   compare the model with what the board does, and with a reference solver that
   is not in this repository; do not loosen one to make it pass.
4. **Run the tests:** `cmake -S . -B build -DCMAKE_BUILD_TYPE=Release`, then
   `cmake --build build` and `ctest --test-dir build`.
