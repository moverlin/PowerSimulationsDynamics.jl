using PowerSimulationsDynamics
using Sundials
using DelimitedFiles

const PSID = PowerSimulationsDynamics

"""
Case 25:
This case study a three-bus system, with two Marconato generators (in buses 1 and 2), and a constant impedance load in bus 3.
The perturbation increase the reference of mechanical power of generator-2 from 0.8 to 0.9 at t=1.0s.
"""

##################################################
############### LOAD DATA ########################
##################################################

#include(joinpath(TEST_FILES_DIR, "data_tests/test25.jl"))
include("data_tests/test25.jl")

include("utils/get_results.jl") # fun. get_init_values_for_comparison(sim)

##################################################
############### SOLVE PROBLEM ####################
##################################################

# time span
tspan = (0.0, 40.0);

# PSCAD benchmark data
#csv_file = joinpath(TEST_FILES_DIR, "benchmarks/pscad/Test25/Test25_v102.csv")
csv_file = "benchmarks/pscad/Test25/Test25_v102.csv"
t_offset = 49.0

# Define Fault using Callbacks
gen2 = get_dynamic_injector(get_component(Generator, sys, "generator-102-1"));
Pref_change = ControlReferenceChange(1.0, gen2, :P_ref, 0.9);

function simulate_example()
    # @testset "Test 25 Marconato with Dynamic Lines ResidualModel" begin
    try
        mkdir("test-25")
    catch e
        println("Cannot make new directory")
    end

    path = (joinpath(pwd(), "test-25"))
    # !isdir(path) && mkdir(path)
    try
        # Define Simulation Problem
        sim = Simulation!(ResidualModel, sys, path, tspan, Pref_change)

        # Test Initial Condition
        diff_val = [0.0]
        res = get_init_values_for_comparison(sim)
        #for (k, v) in test25_x0_init
        #    diff_val[1] += LinearAlgebra.norm(res[k] - v)
        #end
        #@test (diff_val[1] < 1e-3)
        #println(" (diff_val[1] < 1e-3): ", (diff_val[1] < 1e-3))

        print("Showing state initial values..")
        show_states_initial_value(sim)

        # Obtain small signal results for initial conditions
        small_sig = small_signal_analysis(sim)
        #@test small_sig.stable
        println(" small_sig.stable: ", small_sig.stable)

        # Solve problem in equilibrium
        #@test execute!(sim, Sundials.IDA(), dtmax = 0.01, saveat = 0.01) ==
        #        PSID.SIMULATION_FINALIZED
        execute!(sim, Sundials.IDA(), dtmax = 0.01, saveat = 0.01) ==
                PSID.SIMULATION_FINALIZED
        results = read_results(sim)

        #print(results.fields())

        println("typeof(results): ", typeof(results))

        # Obtain voltage magnitude data
        series = get_voltage_magnitude_series(results, 102)
        t = series[1]
        v = series[2]

        # Obtain series from results.
        # https://nrel-siip.github.io/PowerSimulationsDynamics.jl/stable/quick_start_guide/#Make-a-plot-of-the-results
        #angle_gen_102 = get_state_series(results, ("generator-102-1", :δ));
        #state_series = get_state_series(results, "generator");
        # Differential States, "generator-102-1": ψq, ψd, eq_p, ed_p, eq_pp, ed_pp, δ, ω, Vf
        # Differential States, "generator-101-1": ψq, ψd, eq_p, ed_p, eq_pp, ed_pp, δ, ω, Vf, xg

        # Obtain benchmark data from PSCAD
        #M = get_csv_data(csv_file)
        #t_pscad = M[:, 1] .- t_offset
        #v_pscad = M[:, 2]

        # Relaxed constraint to account for mismatch in damping
        #@test LinearAlgebra.norm(v - v_pscad) <= 0.1
        #@test LinearAlgebra.norm(t - round.(t_pscad, digits = 3)) == 0.0

    finally
        @info("removing test files")
        rm(path, force = true, recursive = true)
    end # try / finally block

    # put this here if nothing is to be returned from this function.
    return 

end # function: simulate_example

simulate_example()

