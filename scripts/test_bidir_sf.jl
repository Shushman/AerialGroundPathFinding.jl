using JLD2
using Infiltrator
using LightGraphs
using Random
using JSON
using BenchmarkTools

using AerialGroundPathFinding

const SANFRANCISCO_NODEFILE = "../data/sanfrancisco_node_attribs.json"
const SANFRANCISCO_WEIGHTFILE = "../data/sanfrancisco_sparse_wts.jld2"

function main(seed::Int64, trials::Int64=10)

    rng = MersenneTwister(seed)

    # Load graph and landmarks
    @load SANFRANCISCO_WEIGHTFILE sanfrancisco_sparse_wts
    sanfrancisco_graph = LightGraphs.SimpleDiGraph(sanfrancisco_sparse_wts)
    nvg = nv(sanfrancisco_graph)

    astar_times = Float64[]
    eucl_times = Float64[]

    location_list = get_location2D_list(SANFRANCISCO_NODEFILE)

    for i = 1:trials

        source = rand(rng, 1:nvg)
        target = rand(rng, 1:nvg)

        astar_sp = LightGraphs.ShortestPaths.shortest_paths(sanfrancisco_graph, source, target, sanfrancisco_sparse_wts, LightGraphs.ShortestPaths.BidirAStar())
        
        bidir_astar = LightGraphs.ShortestPaths.BidirAStar()

        if isempty(astar_sp.path) == false

            ast_time = @belapsed LightGraphs.ShortestPaths.shortest_paths($sanfrancisco_graph, $source, $target, $sanfrancisco_sparse_wts, $bidir_astar)
            
            eucl_time = @belapsed compute_bidir_astar_euclidean($sanfrancisco_graph, $source, $target, $sanfrancisco_sparse_wts, $location_list)

            push!(astar_times, ast_time)
            push!(eucl_times, eucl_time)
        end
    end

    return astar_times, eucl_times
end