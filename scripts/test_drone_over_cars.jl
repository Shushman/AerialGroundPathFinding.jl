using JLD2
using Infiltrator
using LightGraphs
using Random
using JSON
using BenchmarkTools
using MultiAgentPathFinding

using AerialGroundPathFinding

const MANHATTAN_NODEFILE = "../data/manhattan_node_attribs.json"
const MANHATTAN_WEIGHTFILE = "../data/manhattan_sparse_wts.jld2"
const SANFRANCISCO_NODEFILE = "../data/sanfrancisco_node_attribs.json"
const SANFRANCISCO_WEIGHTFILE = "../data/sanfrancisco_sparse_wts.jld2"

const NCARS = 2
const NDRONES = 4
const ECBS_WEIGHT = 1.5
const ALPHA_WEIGHT_DISTANCE = 1.0
const CAPACITY = 5


function main(graph_weights::AbstractMatrix, loc_list_file::String, num_cars::Int64, num_drones::Int64, seed::Int64)

    rng = MersenneTwister(seed)
    graph = LightGraphs.SimpleDiGraph(graph_weights)
    nvg = nv(graph)
    location_list = get_location2D_list(loc_list_file)
    @infiltrate

    # Generate task list
    # Ensure that path exist

    ground_task_list = get_tasks_with_valid_path(graph, graph_weights, location_list, num_cars, rng)
    aerial_task_list = get_tasks_with_valid_path(graph, graph_weights, location_list, num_drones, rng)

    # Create environment
    env = CoordinatedMAPFEnv(ground_task_list=ground_task_list, aerial_task_list=aerial_task_list, road_graph=graph, road_graph_wts=graph_weights, location_list=location_list, alpha_weight_distance=ALPHA_WEIGHT_DISTANCE, car_capacity=CAPACITY)

    # Compute drone paths and costs
    dir_drone_paths = compute_independent_paths(env, env.aerial_task_list)
    dir_car_paths = compute_independent_paths(env, env.ground_task_list)

    augment_road_graph_with_aerial_paths!(env, dir_drone_paths)

    # gmapf_solver = ECBSSolver{GroundMAPFState,GroundMAPFAction,Float64,SumOfCosts,GroundMAPFConflict,GroundMAPFConstraint,CoordinatedMAPFEnv}(env=env, weight=ECBS_WEIGHT)

    initial_gmapf_states = [GroundMAPFState(task.origin, false) for task in env.ground_task_list]
    ground_pp_ordering = get_increasing_dist_ordering(dir_car_paths)
    
    @time ground_pp_soln = plan_prioritized_paths!(env, ground_pp_ordering, initial_gmapf_states, GroundMAPFConstraint, PlanResult{GroundMAPFState,GroundMAPFAction,Float64})

    # @time gmapf_soln = search!(gmapf_solver, initial_gmapf_states)
    # println("$(env.num_global_conflicts) conflicts in ground CBS")

    update_ground_paths_with_ground_mapf_result!(env, ground_pp_soln)

    drone_dists = [ddp.total_dist for ddp in dir_drone_paths]

    @time set_ground_transit_graph!(env, drone_dists)

    println("Size of GTG: $(length(env.ground_transit_graph.vertices))")


    # env.num_global_conflicts = 0
    # solver = CBSSolver{AerialMAPFState,GroundTransitAction,Float64,SumOfCosts,GroundTransitConflict,GroundTransitConstraint,CoordinatedMAPFEnv}(env=env)


    initial_amapf_states = [AerialMAPFState(idx=1, ground_transit_idx=sg[1]) for (i, sg) in enumerate(env.gtg_drone_start_goal_idxs)]
    aerial_pp_ordering = get_increasing_dist_ordering(dir_drone_paths)

    @time aerial_pp_soln = plan_prioritized_paths!(env, aerial_pp_ordering, initial_amapf_states, GroundTransitConstraint, PlanResult{AerialMAPFState,GroundTransitAction,Float64})

    # println("$(env.num_global_conflicts) conflicts in aerial CBS")

    # # println("Done")
    update_aerial_ground_paths_cbs_solution!(env, aerial_pp_soln, dir_drone_paths)

    # # Postprocessing; redirect cars not helping to direct paths
    not_coord_ids = count_cars_not_coordinating(env.ground_paths)
    @show not_coord_ids
    for ncid in not_coord_ids
        env.ground_paths[ncid] = dir_car_paths[ncid]
    end

    total_dir_cost = compute_total_cost(env, dir_drone_paths, dir_car_paths)
    total_coord_cost = compute_total_cost(env, env.aerial_paths, env.ground_paths)

    println("Total Dir Cost: $(total_dir_cost) versus Total Coord Cost: $(total_coord_cost)")
    
    return env, ground_pp_soln, aerial_pp_soln, dir_car_paths, dir_drone_paths
end

@load MANHATTAN_WEIGHTFILE manhattan_sparse_wts

env, gpp_soln, app_soln, dcp, ddp = main(manhattan_sparse_wts, MANHATTAN_NODEFILE, NCARS, NDRONES, 1234)


# Benchmark script points
# Set alpha_weight to 1.0
# But also output the makespan of ground and aerial paths as a secondary metric