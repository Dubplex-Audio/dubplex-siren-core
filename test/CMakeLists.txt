# Un executable par fichier, un test CTest par executable.
function(siren_core_test name source)
    add_executable(${source} ${source}.cpp)
    target_link_libraries(${source} PRIVATE dubplex_siren_core)
    add_test(NAME ${name} COMMAND ${source})
endfunction()

# Le moteur complet : le test le plus long, plusieurs minutes.
siren_core_test(siren_engine test_siren_engine)

# Le garde-fou qui suit la machine (MachineFactor) : identique au repos,
# resserre sous charge. Le cout est injecte, la machine ne se charge pas sur
# commande.
siren_core_test(budget test_budget)

# L'astable resolu comme un circuit.
siren_core_test(astable_core test_astable)

# Le solveur generique, verifie sur des circuits calculables a la main (diviseur,
# RC, diode...) puis recoupe contre AstableCore sur l'astable audio.
siren_core_test(mna test_mna)

siren_core_test(lfo test_lfo)

# L'echo a bande : retard juste, reprises decroissantes, coupe bit a bit,
# feedback a fond borne.
siren_core_test(tape_echo test_tape_echo)

# La reverbe a ressort : dosage 0 bit a bit, queue decroissante, pas de
# divergence.
siren_core_test(spring test_spring)

# Le brickwall : rien ne depasse, jamais. Et la latence annoncee est mesuree,
# pas recopiee.
siren_core_test(limiter test_limiter)

# La chaine entiere : repos bit a bit, latence constante, canaux identiques,
# plafond tenu.
siren_core_test(fx_chain test_fx_chain)
