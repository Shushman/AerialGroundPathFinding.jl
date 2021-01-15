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

const NCARS = 5
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
    compute_ground_paths!(env)

    # Compute drone paths and costs
    direct_drone_sps = Vector{Vector{Int64}}(undef, num_drones)
    direct_drone_sp_costs = Vector{Float64}(undef, num_drones)

    for (aerial_id, task) in enumerate(env.aerial_task_list)

        bidir_sp = compute_bidir_astar_euclidean(env.road_graph, task.origin, task.dest, env.road_graph_wts, env.location_list)

        direct_drone_sps[aerial_id] = bidir_sp.path
        direct_drone_sp_costs[aerial_id] = bidir_sp.dist
    end

    return env, direct_drone_sp_costs, direct_drone_sps

end

@load MANHATTAN_WEIGHTFILE manhattan_sparse_wts

env, costs, sps = main(manhattan_sparse_wts, MANHATTAN_NODEFILE, NCARS, NDRONES, 1234)

solver = CBSSolver{GroundTransitState,GroundTransitAction,Float64,SumOfCosts,GroundTransitConflict,GroundTransitConstraint,CoordinatedMAPFEnv}(env=env)

set_ground_transit_graph!(env, costs)

initial_states = [env.ground_transit_graph.vertices[u] for (u, _) in env.gtg_drone_start_goal_idxs]