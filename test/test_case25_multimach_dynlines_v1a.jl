using PowerSimulationsDynamics # 
using PowerSystems             # 
using Sundials                 # To get solvers for numerical integration
using DelimitedFiles           #
using PyPlot, Plots            # Plotting packages
using  LaTeXStrings;           # To write latex strings in plots

const PSID = PowerSimulationsDynamics
const PSY  = PowerSystems

"""
Case 25:
This case study a three-bus system, with two Marconato generators (in buses 1 and 2), and a constant impedance load in bus 3.
The perturbation increase the reference of mechanical power of generator-2 from 0.8 to 0.9 at t=1.0s.
"""

##################################################
############### LOAD DATA ########################
##################################################

#include(joinpath(TEST_FILES_DIR, "data_tests/test25.jl"))
include("data_tests/test25.jl") # data_tests/dynamic_test_data.jl has a lot of the numbers for inertia, machine impedances, etc.
                                # data_tests/data_utils.jl

include("utils/get_results.jl") # fun. get_init_values_for_comparison(sim)

##################################################
############### SOLVE PROBLEM ####################
##################################################

# time span
tspan = (0.0, 120.0);

# PSCAD benchmark data
#csv_file = joinpath(TEST_FILES_DIR, "benchmarks/pscad/Test25/Test25_v102.csv")
csv_file = "benchmarks/pscad/Test25/Test25_v102.csv"
t_offset = 49.0

# name = PSY.get_name(bus)
# name = PSY.get_name(device)
# name = PSY.get_name(br)


# Define Fault using Callbacks
gen2 = get_dynamic_injector(get_component(Generator, sys, "generator-102-1"));
Pref_change = ControlReferenceChange(1.0, gen2, :P_ref, 0.9); # P_ref: 0.8 --> 0.9

# See get_init_values_for_comparison function in get_results.jl file
# https://docs.juliahub.com/PowerSimulationsDynamics/T1QyN/0.1.2/Examples/example_data/
system_PowerLoads = collect(PSY.get_components(PowerLoad, sys))
println("system_PowerLoads: ", system_PowerLoads)

#bus_103_name = PSY.get_name(103)
#println("bus_103_name: ", bus_103_name)

# https://nrel-siip.github.io/PowerSimulationsDynamics.jl/stable/perturbations/
load_device = get_component(ElectricLoad, sys, "load1031")
load_change = LoadChange(20.0, load_device, :P_ref, 1.6) # 1.8 --> 1.6

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
        sim = Simulation!(ResidualModel, sys, path, tspan, [Pref_change, load_change])

        # Test Initial Condition
        diff_val = [0.0]
        res = get_init_values_for_comparison(sim)
        #for (k, v) in test25_x0_init
        #    diff_val[1] += LinearAlgebra.norm(res[k] - v)
        #end
        #@test (diff_val[1] < 1e-3)
        #println(" (diff_val[1] < 1e-3): ", (diff_val[1] < 1e-3))

        set_points = get_setpoints(sim)
        println("Showing set points for all dynamic devices.."); println(set_points)

        println("\nShowing state initial values..")
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

        return results

    finally
        @info("removing test files")
        rm(path, force = true, recursive = true)
    end # try / finally block

    # put this here if nothing is to be returned from this function.
    return 

end # function: simulate_example

results = simulate_example()

# Obtain series from results.
# https://nrel-siip.github.io/PowerSimulationsDynamics.jl/stable/quick_start_guide/#Make-a-plot-of-the-results
        #angle_gen_102 = get_state_series(results, ("generator-102-1", :δ));
        #state_series = get_state_series(results, "generator");
        # Differential States, "generator-102-1": ψq, ψd, eq_p, ed_p, eq_pp, ed_pp, δ, ω, Vf
        # Differential States, "generator-101-1": ψq, ψd, eq_p, ed_p, eq_pp, ed_pp, δ, ω, Vf, xg

println("typeof(results): ", typeof(results))

# Get branch current flows
series = get_real_current_branch_flow(results,      "BUS 1-BUS 3-i_1"); t_real_101_to_103 = series[1]; I_real_101_to_103 = series[2];
series = get_imaginary_current_branch_flow(results, "BUS 1-BUS 3-i_1"); t_imag_101_to_103 = series[1]; I_imag_101_to_103 = series[2];
series = get_real_current_branch_flow(results,      "BUS 1-BUS 2-i_1"); t_real_101_to_102 = series[1]; I_real_101_to_102 = series[2];
series = get_imaginary_current_branch_flow(results, "BUS 1-BUS 2-i_1"); t_imag_101_to_102 = series[1]; I_imag_101_to_102 = series[2];
series = get_real_current_branch_flow(results,      "BUS 2-BUS 3-i_1"); t_real_102_to_103 = series[1]; I_real_102_to_103 = series[2];
series = get_imaginary_current_branch_flow(results, "BUS 2-BUS 3-i_1"); t_imag_102_to_103 = series[1]; I_imag_102_to_103 = series[2];

# Obtain bus voltage data
series = get_voltage_magnitude_series(results, 101); t_mag_v101 = series[1]; v_mag_101  = series[2]
series = get_voltage_angle_series(results, 101); t_angle_v101 = series[1]; v_angle_101  = series[2]
series = get_voltage_magnitude_series(results, 102); t_mag_v102 = series[1]; v_mag_102  = series[2]
series = get_voltage_angle_series(results, 102); t_angle_v102 = series[1]; v_angle_102  = series[2]
series = get_voltage_magnitude_series(results, 103); t_mag_v103 = series[1]; v_mag_103  = series[2]
series = get_voltage_angle_series(results, 103); t_angle_v103 = series[1]; v_angle_103  = series[2]
# println("size(t_v101), size(v_101): ", size(t_v101), size(v_101))

# Active and Reactive powers at load bus, bus 3:
bus_3_active_power   = real(v_mag_103 .* exp.(1im*v_angle_103)) .* (I_real_101_to_103 .+ I_real_102_to_103) + 
                       imag(v_mag_103 .* exp.(1im*v_angle_103)) .* (I_imag_101_to_103 .+ I_imag_102_to_103)
bus_3_reactive_power = imag(v_mag_103 .* exp.(1im*v_angle_103)) .* (I_real_101_to_103 .+ I_real_102_to_103) - 
                       real(v_mag_103 .* exp.(1im*v_angle_103)) .* (I_imag_101_to_103 .+ I_imag_102_to_103)

# Get active power series
P_series = get_activepower_series(results, "generator-101-1"); t_P101 = P_series[1]; P_101  = P_series[2]
P_series = get_activepower_series(results, "generator-102-1"); t_P102 = P_series[1]; P_102  = P_series[2]
P_series = get_activepower_series(results, "load1031"); t_P103 = P_series[1]; P_103  = P_series[2]
#println("size(t_v101), size(v_101): ", size(t_v101), size(v_101))

# Get reactive power series
Q_series = get_reactivepower_series(results, "generator-101-1"); t_Q101 = Q_series[1]; Q_101  = Q_series[2]
Q_series = get_reactivepower_series(results, "generator-102-1"); t_Q102 = Q_series[1]; Q_102  = Q_series[2]
Q_series = get_reactivepower_series(results, "load1031"); t_Q103 = Q_series[1]; Q_103  = Q_series[2]

PyPlot.close("all");

    # --------------------------------
	# Plot 1: Active Powers 
    # --------------------------------
	#PyPlot.figure(figsize=[9.6, 7.2], dpi=500.0); # New plot figure window: width/height in inches, dots-per-inch.
	PyPlot.figure(); # New plot figure window: width/height in inches, dots-per-inch.
	PyPlot.suptitle("Bus Active Powers");
    PyPlot.plot(t_P101, P_101, linewidth=1.0, label="Bus 101 Active Power");
    PyPlot.plot(t_Q101, Q_101, linewidth=1.0, label="Bus 101 Reactive Power");
    PyPlot.plot(t_P102, P_102, linewidth=1.0, label="Bus 102 Active Power");
    PyPlot.plot(t_Q102, Q_102, linewidth=1.0, label="Bus 102 Reactive Power");
    PyPlot.plot(t_P103, bus_3_active_power, linewidth=1.0, label="Bus 103 Active Power");
    PyPlot.plot(t_Q103, bus_3_reactive_power, linewidth=1.0, label="Bus 103 Reactive Power");
      PyPlot.legend();
	  PyPlot.xlabel("Time [sec.]");
	  PyPlot.ylabel(L"$P$, $Q$ [p.u.]");
	  PyPlot.grid(b=true, which="both", axis="both");
	PyPlot.savefig(string("Plots/Bus_Active_Powers", ".png"), bbox_inches="tight");
	PyPlot.show(block=false); # if block=true, we must close figure to continue julia script execution.

PyPlot.close("all");

    # --------------------------------
	# Plot 2: Bus voltage magnitudes
    # --------------------------------
	#PyPlot.figure(figsize=[9.6, 7.2], dpi=500.0); # New plot figure window: width/height in inches, dots-per-inch.
	PyPlot.figure(); # New plot figure window: width/height in inches, dots-per-inch.
	PyPlot.suptitle("Bus Voltage Magnitudes");
    PyPlot.plot(t_mag_v101, v_mag_101, linewidth=1.0, label="Bus 101");
    PyPlot.plot(t_mag_v102, v_mag_102, linewidth=1.0, label="Bus 102");
    PyPlot.plot(t_mag_v103, v_mag_103, linewidth=1.0, label="Bus 103");
      PyPlot.legend();
	  PyPlot.xlabel("Time [sec.]");
	  PyPlot.ylabel(L"$V_{mag}$ [p.u.]");
	  PyPlot.grid(b=true, which="both", axis="both");
	PyPlot.savefig(string("Plots/Bus_Voltage_Magnitudes", ".png"), bbox_inches="tight");
	PyPlot.show(block=false); # if block=true, we must close figure to continue julia script execution.

PyPlot.close("all");


