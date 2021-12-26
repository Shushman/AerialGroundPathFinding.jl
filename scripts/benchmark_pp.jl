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

        # New decay function
        t3 = @elapsed augment_road_graph_with_aerial_paths!(env, dir_drone_paths, x -> 0.5*(1 + tanh(x)))
        push!(time_comps, t3)

        initial_gmapf_states = [GroundMAPFState(task.origin, false) for task in env.ground_task_list]
        t4 = @elapsed ground_pp_ordering = get_increasing_dist_ordering(dir_car_paths)
        push!(time_comps, t4)

        t5 = @elapsed ground_pp_soln = plan_prioritized_paths!(env, ground_pp_ordering, initial_gmapf_states, GroundMAPFConstraint, PlanResult{GroundMAPFState,GroundMAPFAction,Float64})
        push!(time_comps, t5)

        t6 = @elapsed update_ground_paths_with_ground_mapf_result!(env, ground_pp_soln)
        push!(time_comps, t6)

        t7 = @elapsed set_ground_transit_graph!(env, [ddp.total_dist for ddp in dir_drone_paths])
        push!(time_comps, t7)

        initial_amapf_states = [AerialMAPFState(idx=1, ground_transit_idx=sg[1]) for (i, sg) in enumerate(env.gtg_drone_start_goal_idxs)]
        t8 = @elapsed aerial_pp_ordering = get_increasing_dist_ordering(dir_drone_paths)
        push!(time_comps, t8)

        t9 = @elapsed aerial_pp_soln = plan_prioritized_paths!(env, aerial_pp_ordering, initial_amapf_states, GroundTransitConstraint, PlanResult{AerialMAPFState,GroundTransitAction,Float64})
        push!(time_comps, t9)

        t10 = @elapsed update_aerial_ground_paths_cbs_solution!(env, aerial_pp_soln, dir_drone_paths)
        push!(time_comps, t10)

        not_coord_ids = count_cars_not_coordinating(env.ground_paths)
        for ncid in not_coord_ids
            env.ground_paths[ncid] = dir_car_paths[ncid]
        end

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