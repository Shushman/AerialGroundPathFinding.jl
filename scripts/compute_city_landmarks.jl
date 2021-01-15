using JLD2
using Infiltrator
using LightGraphs
using Random
using JSON

using AerialGroundPathFinding

const MANHATTAN_NODEFILE = "../data/manhattan_node_attribs.json"
const MANHATTAN_WEIGHTFILE = "../data/manhattan_sparse_wts.jld2"
const SANFRANCISCO_NODEFILE = "../data/sanfrancisco_node_attribs.json"
const SANFRANCISCO_WEIGHTFILE = "../data/sanfrancisco_sparse_wts.jld2"

const RNG = MersenneTwister(1234)

function main(;manhattan_outfile::String, sanfrancisco_outfile::String, num_manhattan_landmarks::Int64, num_sanfrancisco_landmarks::Int64)

    @load MANHATTAN_WEIGHTFILE manhattan_sparse_wts
    @load SANFRANCISCO_WEIGHTFILE sanfrancisco_sparse_wts

    mh_node_attribs = JSON.parsefile(MANHATTAN_NODEFILE)
    sf_node_attribs = JSON.parsefile(SANFRANCISCO_NODEFILE)

    # First do for manhattan
    manhattan_graph = LightGraphs.SimpleDiGraph(manhattan_sparse_wts)
    manhattan_landmarks = greedy_landmarks(manhattan_graph, manhattan_sparse_wts, num_manhattan_landmarks, RNG)
    manhattan_lm_from_dists, manhattan_lm_to_dists = landmark_distances(manhattan_graph, manhattan_landmarks, manhattan_sparse_wts)

    sanfrancisco_graph = LightGraphs.SimpleDiGraph(sanfrancisco_sparse_wts)
    sanfrancisco_landmarks = greedy_landmarks(sanfrancisco_graph, sanfrancisco_sparse_wts, num_sanfrancisco_landmarks, RNG)
    sanfrancisco_lm_from_dists, sanfrancisco_lm_to_dists = landmark_distances(sanfrancisco_graph, sanfrancisco_landmarks, sanfrancisco_sparse_wts)

    @assert endswith(manhattan_outfile, ".jld2") && endswith(sanfrancisco_outfile, ".jld2")

    @save manhattan_outfile manhattan_landmarks manhattan_lm_from_dists manhattan_lm_to_dists
    @save sanfrancisco_outfile sanfrancisco_landmarks sanfrancisco_lm_from_dists sanfrancisco_lm_to_dists
end # main

# TODO: Note that SF_Landmarks need to be postprocessed

main(manhattan_outfile="manhattan_8_landmarks.jld2", sanfrancisco_outfile="sanfrancisco_12_landmarks.jld2", num_manhattan_landmarks=8, num_sanfrancisco_landmarks=12)