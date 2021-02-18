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

const NCARS = 10
const NDRONES = 10


function main(graph_weights::AbstractMatrix, loc_list_file::String, num_cars::Int64, num_drones::Int64, seed::Int64)

    rng = MersenneTwister(seed)
    graph = LightGraphs.SimpleDiGraph(graph_weights)
    nvg = nv(graph)
    location_list = get_location2D_list(loc_list_file)

    # Generate task list
    ground_task_list = [AgentTask((rand(rng, 1:nvg), rand(rng, 1:nvg))) for _ = 1:num_cars]
    aerial_task_list = [AgentTask((rand(rng, 1:nvg), rand(rng, 1:nvg))) for _ = 1:num_drones]

    # Create environment
    env = CoordinatedMAPFEnv(ground_task_list=ground_task_list, aerial_task_list=aerial_task_list, road_graph=graph, road_graph_wts=graph_weights, location_list=location_list)

    # Compute ground paths and assign
    env.ground_paths = compute_independent_paths(env, env.ground_task_list)

    # Compute drone paths and costs
    direct_drone_paths = compute_independent_paths(env, env.aerial_task_list)

    # aerial_ground_coord_costs = zeros(Float64, NCARS, NDRONES)
    # aerial_ground_coord_segments = Dict{Tuple{Int64,Int64},Tuple{Int64,Int64}}()

    # for c = 1:num_cars
    #     for d = 1:num_drones
    #         aerial_ground_coord_costs[c, d], aerial_ground_coord_segments[(c, d)] = aerial_ground_coord_path_cost(env, d, c)
    #     end
    # end
    drone_dists = [ddp.total_dist for ddp in direct_drone_paths]

    set_ground_transit_graph!(env, drone_dists)

    return env, direct_drone_paths
end

@load MANHATTAN_WEIGHTFILE manhattan_sparse_wts

env, dir_drone_paths = main(manhattan_sparse_wts, MANHATTAN_NODEFILE, NCARS, NDRONES, 1234)

solver = CBSSolver{AerialMAPFState,GroundTransitAction,Float64,SumOfCosts,GroundTransitConflict,GroundTransitConstraint,CoordinatedMAPFEnv}(env=env)

initial_states = [AerialMAPFState(idx=i, ground_transit_idx=sg[1]) for (i, sg) in enumerate(env.gtg_drone_start_goal_idxs)]

solution = search!(solver, initial_states)