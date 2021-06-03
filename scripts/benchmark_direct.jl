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
const TRIALS = parse(Int64, ARGS[4])

function main()

    @load MANHATTAN_WEIGHTFILE manhattan_sparse_wts

    graph = LightGraphs.SimpleDiGraph(manhattan_sparse_wts)
    location_list = get_location2D_list(MANHATTAN_NODEFILE)

    for t = 1:TRIALS+1

        rng = MersenneTwister(BASE_SEED+t)

        ground_task_list = get_tasks_with_valid_path(graph, manhattan_sparse_wts, location_list, NCARS, rng)
        aerial_task_list = get_tasks_with_valid_path(graph, manhattan_sparse_wts, location_list, NDRONES, rng)

        env = CoordinatedMAPFEnv(ground_task_list=ground_task_list, aerial_task_list=aerial_task_list, road_graph=graph, road_graph_wts=manhattan_sparse_wts, location_list=location_list, alpha_weight_distance=ALPHA_WEIGHT_DISTANCE, car_capacity=1)

        t1 = @elapsed dir_drone_paths = compute_independent_paths(env, env.aerial_task_list)
        t2 = @elapsed dir_car_paths = compute_independent_paths(env, env.ground_task_list)

        total_dist = compute_total_cost(env, dir_drone_paths, dir_car_paths)

        res_dict = Dict{String,Float64}("compute_time" => (t1+t2), "total_path_cost" => total_dist)

        if t > 1
            fname = string(OUTFILEDIR, "_Cars_", NCARS, "_Drones_", NDRONES, "_Trial_", (t-1), ".json")
            println(fname)
            open(fname, "w") do f
                JSON.print(f, res_dict, 2)
            end
        end
    end
end

main()