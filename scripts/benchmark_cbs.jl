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
const ECBS_WEIGHT = 1.5
# const THRESHOLD_CONFLICTS = 100

const NCARS = parse(Int64, ARGS[1])
const NDRONES = parse(Int64, ARGS[2])
const OUTFILEDIR = ARGS[3]
const CAPACITY = parse(Int64, ARGS[4])
const TRIALS = parse(Int64, ARGS[5])
const THRESHOLD_CONFLICTS = parse(Int64, ARGS[6])

function main()

    @load MANHATTAN_WEIGHTFILE manhattan_sparse_wts

    graph = LightGraphs.SimpleDiGraph(manhattan_sparse_wts)
    location_list = get_location2D_list(MANHATTAN_NODEFILE)

    for t = 1:TRIALS+1

        rng = MersenneTwister(BASE_SEED+t)

        ground_task_list = get_tasks_with_valid_path(graph, manhattan_sparse_wts, location_list, NCARS, rng)
        aerial_task_list = get_tasks_with_valid_path(graph, manhattan_sparse_wts, location_list, NDRONES, rng)

        env = CoordinatedMAPFEnv(ground_task_list=ground_task_list, aerial_task_list=aerial_task_list, road_graph=graph, road_graph_wts=manhattan_sparse_wts, location_list=location_list, alpha_weight_distance=ALPHA_WEIGHT_DISTANCE, car_capacity=CAPACITY, threshold_global_conflicts=THRESHOLD_CONFLICTS)

        time_comps = Float64[]

        t1 = @elapsed dir_drone_paths = compute_independent_paths(env, env.aerial_task_list)
        t2 = @elapsed dir_car_paths = compute_independent_paths(env, env.ground_task_list)
        push!(time_comps, t1+t2)

        t3 = @elapsed augment_road_graph_with_aerial_paths!(env, dir_drone_paths)
        push!(time_comps, t3)

        initial_gmapf_states = [GroundMAPFState(task.origin, false) for task in env.ground_task_list]

        t4 = @elapsed gmapf_solver = ECBSSolver{GroundMAPFState,GroundMAPFAction,Float64,SumOfCosts,GroundMAPFConflict,GroundMAPFConstraint,CoordinatedMAPFEnv}(env=env, weight=ECBS_WEIGHT)
        push!(time_comps, t4)
        
        ground_cbs_soln = PlanResult[]
        try
            t5 = @elapsed ground_cbs_soln = search!(gmapf_solver, initial_gmapf_states)
            push!(time_comps, t5)
        catch y
            if isa(y, DomainError)
                println("Too many conflicts in stage 1 of trial $(t-1)!")
                continue
            elseif  isa(y, BoundsError)
                println("ECBS ran out of nodes in stage 1 of trial $(t-1)")
                continue
            else
                throw(y)
            end
        end

        ground_conflicts = env.num_global_conflicts

        env.num_global_conflicts = 0

        t6 = @elapsed update_ground_paths_with_ground_mapf_result!(env, ground_cbs_soln)
        push!(time_comps, t6)

        t7 = @elapsed set_ground_transit_graph!(env, [ddp.total_dist for ddp in dir_drone_paths])
        push!(time_comps, t7)

        initial_amapf_states = [AerialMAPFState(idx=1, ground_transit_idx=sg[1]) for (i, sg) in enumerate(env.gtg_drone_start_goal_idxs)]
        
        t8 = @elapsed aerial_mapf_solver = ECBSSolver{AerialMAPFState,GroundTransitAction,Float64,SumOfCosts,GroundTransitConflict,GroundTransitConstraint,CoordinatedMAPFEnv}(env=env, weight=ECBS_WEIGHT)
        push!(time_comps, t8)

        aerial_cbs_soln = PlanResult[]
        try
            t9 = @elapsed aerial_cbs_soln = search!(aerial_mapf_solver, initial_amapf_states)
            push!(time_comps, t9)
        catch y
            if isa(y, DomainError)
                println("Too many conflicts in stage 2 of trial $(t-1)!")
                continue
            elseif  isa(y, BoundsError)
                println("ECBS ran out of nodes in stage 2 of trial $(t-1)")
                continue
            else
                throw(y)
            end
        end

        aerial_conflicts = env.num_global_conflicts

        t10 = @elapsed update_aerial_ground_paths_cbs_solution!(env, aerial_cbs_soln, dir_drone_paths)
        push!(time_comps, t10)

        not_coord_ids = count_cars_not_coordinating(env.ground_paths)
        for ncid in not_coord_ids
            env.ground_paths[ncid] = dir_car_paths[ncid]
        end

        total_dist = compute_total_cost(env, env.aerial_paths, env.ground_paths)

        res_dict = Dict("compute_time" => sum(time_comps), "total_path_cost" => total_dist, "ground_conflicts" => ground_conflicts, "aerial_conflicts" => aerial_conflicts)

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