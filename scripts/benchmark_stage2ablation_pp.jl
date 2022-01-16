using JLD2
using Infiltrator
using LightGraphs
using Random
using JSON
using BenchmarkTools
using MultiAgentPathFinding


using AerialGroundPathFinding

const BASE_SEED = 6789
const MANHATTAN_NODEFILE = "../data/manhattan_node_attribs.json"
const MANHATTAN_WEIGHTFILE = "../data/manhattan_sparse_wts.jld2"

# Only caring about distance for now
const ALPHA_WEIGHT_DISTANCE = 1.0

const NCARS = parse(Int64, ARGS[1])
const NDRONES = parse(Int64, ARGS[2])
const OUTFILEDIR = ARGS[3]
const CAPACITY = parse(Int64, ARGS[4])
const TRIALS = parse(Int64, ARGS[5])


## Stage 1 paths are computed directly
## Stage 2 paths are drones over transit
function main()

    @load MANHATTAN_WEIGHTFILE manhattan_sparse_wts

    graph = LightGraphs.SimpleDiGraph(manhattan_sparse_wts)
    location_list = get_location2D_list(MANHATTAN_NODEFILE)

    for t = 1:TRIALS+1

        rng = MersenneTwister(BASE_SEED+t)

        ground_task_list = get_tasks_with_valid_path(graph, manhattan_sparse_wts, location_list, NCARS, rng)
        aerial_task_list = get_tasks_with_valid_path(graph, manhattan_sparse_wts, location_list, NDRONES, rng)

        env = CoordinatedMAPFEnv(ground_task_list=ground_task_list, aerial_task_list=aerial_task_list, road_graph=graph, road_graph_wts=manhattan_sparse_wts, location_list=location_list, alpha_weight_distance=ALPHA_WEIGHT_DISTANCE, car_capacity=CAPACITY)

        time_comps = Float64[]

        t1 = @elapsed dir_drone_paths = compute_independent_paths(env, env.aerial_task_list)
        t2 = @elapsed dir_car_paths = compute_independent_paths(env, env.ground_task_list)
        push!(time_comps, t1+t2)
        
        # Just set env ground and aerial paths to direct and update 
        env.ground_paths = dir_car_paths
        env.aerial_paths = dir_drone_paths
        t3 = @elapsed set_ground_transit_graph!(env, [ddp.total_dist for ddp in dir_drone_paths])
        push!(time_comps, t3)

        initial_amapf_states = [AerialMAPFState(idx=1, ground_transit_idx=sg[1]) for (_, sg) in enumerate(env.gtg_drone_start_goal_idxs)]
        t4 = @elapsed aerial_pp_ordering = get_increasing_dist_ordering(dir_drone_paths)
        push!(time_comps, t4)

        t5 = @elapsed aerial_pp_soln = plan_prioritized_paths!(env, aerial_pp_ordering, initial_amapf_states, GroundTransitConstraint, PlanResult{AerialMAPFState,GroundTransitAction,Float64})
        push!(time_comps, t5)

        t6 = @elapsed update_aerial_ground_paths_cbs_solution!(env, aerial_pp_soln, dir_drone_paths)
        push!(time_comps, t6)

        total_dist = compute_total_cost(env, env.aerial_paths, env.ground_paths)

        res_dict = Dict{String,Float64}("compute_time" => sum(time_comps), "total_path_cost" => total_dist)

        if t > 1
            fname = string(OUTFILEDIR, "_Cars_", NCARS, "_Drones_", NDRONES, "_Capacity_", CAPACITY, "_Trial_", (t-1), ".json")
            println(fname)
            open(fname, "w") do f
                JSON.print(f, res_dict, 2)
            end
        end
    end
end

main()