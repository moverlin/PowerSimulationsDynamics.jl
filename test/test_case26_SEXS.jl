"""
Validation PSSE/SEXS:
This case study defines a three bus system with an infinite bus, GENROU and a load.
The GENROU machine has connected an SEXS Excitation System.
The fault drop the line connecting the infinite bus and GENROU.
"""

##################################################
############### SOLVE PROBLEM ####################
##################################################

# Define dyr files

names = ["SEXS", "SEXS: TE=0"]

dyr_files = [
    joinpath(TEST_FILES_DIR, "benchmarks/psse/SEXS/ThreeBus_SEXS.dyr"),
    joinpath(TEST_FILES_DIR, "benchmarks/psse/SEXS/ThreeBus_SEXS_noTE.dyr"),
]

csv_files = [
    joinpath(TEST_FILES_DIR, "benchmarks/psse/SEXS/SEXS_RESULTS.csv"),
    joinpath(TEST_FILES_DIR, "benchmarks/psse/SEXS/SEXS_RESULTS_NoTE.csv"),
]

init_conditions = test26_x0_init

eigs_values = [test26_eigvals, test26_eigvals_noTE]

raw_file_dir = joinpath(TEST_FILES_DIR, "benchmarks/psse/SEXS/ThreeBusMulti.raw")
tspan = (0.0, 20.0)

function test_sexs_implicit(dyr_file, csv_file, init_cond, eigs_value)
    path = (joinpath(pwd(), "test-psse-sexs"))
    !isdir(path) && mkdir(path)
    try
        sys = System(raw_file_dir, dyr_file)
        for l in get_components(PSY.PowerLoad, sys)
            PSY.set_model!(l, PSY.LoadModels.ConstantImpedance)
        end

        # Define Simulation Problem
        sim = Simulation!(
            ResidualModel,
            sys, #system
            path,
            tspan, #time span
            BranchTrip(1.0, Line, "BUS 1-BUS 2-i_1"), #Type of Fault
        ) #Type of Fault

        # Test Initial Condition
        diff_val = [0.0]
        res = get_init_values_for_comparison(sim)
        for (k, v) in init_cond
            diff_val[1] += LinearAlgebra.norm(res[k] - v)
        end

        @test (diff_val[1] < 1e-3)

        # Obtain small signal results for initial conditions. Testing the simulation reset
        small_sig = small_signal_analysis(sim)
        eigs = small_sig.eigenvalues
        @test small_sig.stable

        # Test Eigenvalues
        @test LinearAlgebra.norm(eigs - eigs_value) < 1e-3

        # Solve problem
        @test execute!(sim, IDA(), dtmax = 0.005, saveat = 0.005) ==
              PSID.SIMULATION_FINALIZED
        results = read_results(sim)

        # Obtain data for voltage magnitude at bus 102
        series = get_voltage_magnitude_series(results, 102)
        t = series[1]
        V = series[2]
        # Test Vf, τm and branch series flows with PSSE
        _, P101_103 = get_activepower_branch_flow(results, "BUS 1-BUS 3-i_1", :from)
        _, Q101_103 = get_reactivepower_branch_flow(results, "BUS 1-BUS 3-i_1", :from)
        _, P103_101 = get_activepower_branch_flow(results, "BUS 1-BUS 3-i_1", :to)
        _, Q103_101 = get_reactivepower_branch_flow(results, "BUS 1-BUS 3-i_1", :to)
        _, Vf = get_field_voltage_series(results, "generator-102-1")
        _, τm = get_mechanical_torque_series(results, "generator-102-1")

        # TODO: Get PSSE CSV files and enable tests
        M = get_csv_data(csv_file)
        t_psse = M[:, 1]
        V_psse = M[:, 2]
        P101_103_psse = M[:, 3] ./ 100.0 # convert to pu
        Q101_103_psse = M[:, 4] ./ 100.0 # convert to pu
        P103_101_psse = M[:, 5] ./ 100.0 # convert to pu
        Q103_101_psse = M[:, 6] ./ 100.0 # convert to pu
        Vf_psse = M[:, 7]
        τm_psse = M[:, 8]

        # Test Transient Simulation Results
        @test LinearAlgebra.norm(V - V_psse, Inf) <= 1e-2
        @test LinearAlgebra.norm(P101_103 - P101_103_psse, Inf) <= 1e-2
        @test LinearAlgebra.norm(Q101_103 - Q101_103_psse, Inf) <= 1e-2
        @test LinearAlgebra.norm(P103_101 - P103_101_psse, Inf) <= 1e-2
        @test LinearAlgebra.norm(Q103_101 - Q103_101_psse, Inf) <= 1e-2
        @test LinearAlgebra.norm(Vf - Vf_psse, Inf) <= 1e-2
        @test LinearAlgebra.norm(τm - τm_psse, Inf) <= 1e-2
        @test LinearAlgebra.norm(t - round.(t_psse, digits = 3)) == 0.0
    finally
        @info("removing test files")
        rm(path, force = true, recursive = true)
    end
end

function test_sexs_mass_matrix(dyr_file, csv_file, init_cond, eigs_value)
    path = (joinpath(pwd(), "test-psse-sexs"))
    !isdir(path) && mkdir(path)
    try
        sys = System(raw_file_dir, dyr_file)
        for l in get_components(PSY.PowerLoad, sys)
            PSY.set_model!(l, PSY.LoadModels.ConstantImpedance)
        end

        # Define Simulation Problem
        sim = Simulation!(
            MassMatrixModel,
            sys, #system
            path,
            tspan, #time span
            BranchTrip(1.0, Line, "BUS 1-BUS 2-i_1"), #Type of Fault
        ) #Type of Fault

        # Test Initial Condition
        diff_val = [0.0]
        res = get_init_values_for_comparison(sim)
        for (k, v) in init_cond
            diff_val[1] += LinearAlgebra.norm(res[k] - v)
        end

        @test (diff_val[1] < 1e-3)

        # Obtain small signal results for initial conditions. Testing the simulation reset
        small_sig = small_signal_analysis(sim)
        eigs = small_sig.eigenvalues
        @test small_sig.stable

        # Test Eigenvalues
        @test LinearAlgebra.norm(eigs - eigs_value) < 1e-3

        # Solve problem
        @test execute!(sim, Rodas4(), dtmax = 0.005, saveat = 0.005) ==
              PSID.SIMULATION_FINALIZED
        results = read_results(sim)

        # Obtain data for voltage magnitude at bus 102
        series = get_voltage_magnitude_series(results, 102)
        t = series[1]
        V = series[2]
        # Test Vf, τm and branch series flows with PSSE
        _, P101_103 = get_activepower_branch_flow(results, "BUS 1-BUS 3-i_1", :from)
        _, Q101_103 = get_reactivepower_branch_flow(results, "BUS 1-BUS 3-i_1", :from)
        _, P103_101 = get_activepower_branch_flow(results, "BUS 1-BUS 3-i_1", :to)
        _, Q103_101 = get_reactivepower_branch_flow(results, "BUS 1-BUS 3-i_1", :to)
        _, Vf = get_field_voltage_series(results, "generator-102-1")
        _, τm = get_mechanical_torque_series(results, "generator-102-1")

        # TODO: Get PSSE CSV files and enable tests
        M = get_csv_data(csv_file)
        t_psse = M[:, 1]
        V_psse = M[:, 2]
        P101_103_psse = M[:, 3] ./ 100.0 # convert to pu
        Q101_103_psse = M[:, 4] ./ 100.0 # convert to pu
        P103_101_psse = M[:, 5] ./ 100.0 # convert to pu
        Q103_101_psse = M[:, 6] ./ 100.0 # convert to pu
        Vf_psse = M[:, 7]
        τm_psse = M[:, 8]

        # Test Transient Simulation Results
        @test LinearAlgebra.norm(V - V_psse, Inf) <= 1e-2
        @test LinearAlgebra.norm(P101_103 - P101_103_psse, Inf) <= 1e-2
        @test LinearAlgebra.norm(Q101_103 - Q101_103_psse, Inf) <= 1e-2
        @test LinearAlgebra.norm(P103_101 - P103_101_psse, Inf) <= 1e-2
        @test LinearAlgebra.norm(Q103_101 - Q103_101_psse, Inf) <= 1e-2
        @test LinearAlgebra.norm(Vf - Vf_psse, Inf) <= 1e-2
        @test LinearAlgebra.norm(τm - τm_psse, Inf) <= 1e-2
        @test LinearAlgebra.norm(t - round.(t_psse, digits = 3)) == 0.0
    finally
        @info("removing test files")
        rm(path, force = true, recursive = true)
    end
end

@testset "Test 26 SEXS ResidualModel" begin
    for (ix, name) in enumerate(names)
        @testset "$(name)" begin
            dyr_file = dyr_files[ix]
            csv_file = csv_files[ix]
            init_cond = init_conditions
            eigs_value = eigs_values[ix]
            test_sexs_implicit(dyr_file, csv_file, init_cond, eigs_value)
        end
    end
end

@testset "Test 26 SEXS MassMatrixModel" begin
    for (ix, name) in enumerate(names)
        @testset "$(name)" begin
            dyr_file = dyr_files[ix]
            csv_file = csv_files[ix]
            init_cond = init_conditions
            eigs_value = eigs_values[ix]
            test_sexs_mass_matrix(dyr_file, csv_file, init_cond, eigs_value)
        end
    end
end
