using JLD2
using Infiltrator
using LightGraphs
using Random
using JSON
using BenchmarkTools

using AerialGroundPathFinding

const MANHATTAN_NODEFILE = "../data/manhattan_node_attribs.json"
const MANHATTAN_WEIGHTFILE = "../data/manhattan_sparse_wts.jld2"

function main(seed::Int64, trials::Int64=10)

    rng = MersenneTwister(seed)

    # Load graph and landmarks
    @load MANHATTAN_WEIGHTFILE manhattan_sparse_wts
    manhattan_graph = LightGraphs.SimpleDiGraph(manhattan_sparse_wts)
    nvg = nv(manhattan_graph)

    astar_times = Float64[]
    eucl_times = Float64[]

    location_list = get_location2D_list(MANHATTAN_NODEFILE)

    for i = 1:trials

        source = rand(rng, 1:nvg)
        target = rand(rng, 1:nvg)

        astar_sp = LightGraphs.ShortestPaths.shortest_paths(manhattan_graph, source, target, manhattan_sparse_wts, LightGraphs.ShortestPaths.BidirAStar())

        bidir_astar = LightGraphs.ShortestPaths.BidirAStar()

        if isempty(astar_sp.path) == false

            ast_time = @belapsed LightGraphs.ShortestPaths.shortest_paths($manhattan_graph, $source, $target, $manhattan_sparse_wts, $bidir_astar)


            eucl_time = @belapsed compute_bidir_astar_euclidean($manhattan_graph, $source, $target, $manhattan_sparse_wts, $location_list)

            push!(astar_times, ast_time)
            push!(eucl_times, eucl_time)
        end
    end

    return astar_times, eucl_times
end