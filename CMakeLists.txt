cmake_minimum_required(VERSION 3.22)
project(DubplexSirenCore LANGUAGES CXX)

# Le coeur DSP : des en-tetes, sans framework. Il ne voit que la bibliotheque
# standard, c'est ce qui le rend portable tel quel.
add_library(dubplex_siren_core INTERFACE)
# params/ est a part : une interface utilisateur doit pouvoir connaitre la liste
# des parametres sans voir le circuit.
target_include_directories(dubplex_siren_core INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/dsp
    ${CMAKE_CURRENT_SOURCE_DIR}/params)
target_compile_features(dubplex_siren_core INTERFACE cxx_std_20)

# Les tests ne se construisent par defaut que si ce projet est a la racine. Un
# projet qui l'inclut comme sous-module choisit lui-meme.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(DUBPLEX_SIREN_CORE_TOPLEVEL ON)
else()
    set(DUBPLEX_SIREN_CORE_TOPLEVEL OFF)
endif()
option(DUBPLEX_SIREN_CORE_BUILD_TESTS "Construire les tests hors ligne du DSP"
       ${DUBPLEX_SIREN_CORE_TOPLEVEL})

if(DUBPLEX_SIREN_CORE_BUILD_TESTS)
    if(DUBPLEX_SIREN_CORE_TOPLEVEL)
        set(CMAKE_CXX_EXTENSIONS OFF)
        enable_testing()
    endif()
    add_subdirectory(test)

    # L'exemple sert aussi de test : il sort en erreur si le rendu est silencieux.
    if(DUBPLEX_SIREN_CORE_TOPLEVEL)
        add_executable(render_example examples/render.cpp)
        target_link_libraries(render_example PRIVATE dubplex_siren_core)
        add_test(NAME render_example
                 COMMAND render_example ${CMAKE_CURRENT_BINARY_DIR}/render_example.wav)
    endif()
endif()
