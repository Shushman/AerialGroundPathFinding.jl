module AerialGroundPathFinding

# Write your package code here.
using Random

using StaticArrays
using Distances
using Parameters
using TOML

using LightGraphs
using LightGraphs.ShortestPaths

export
    Location2D,
    LatLonCoords,
    EuclideanLatLongMetric,
    parse_city_params

export
    ALTInfo,
    greedy_landmarks,
    landmark_distances

include("utils.jl")
include("astar_landmarks.jl")

end
