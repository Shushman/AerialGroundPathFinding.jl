module AerialGroundPathFinding

# Write your package code here.
using Random

using StaticArrays
using Distances
using Parameters
using TOML
using Infiltrator
using SparseArrays
using JSON

using Graphs
using LightGraphs
using LightGraphs.ShortestPaths

using MultiAgentPathFinding

export
    AgentTask,
    Location2D,
    LatLonCoords,
    EuclideanLatLongMetric,
    GroundTransitState,
    AerialMAPFState,
    GroundTransitAction,
    GroundTransitConflict,
    GroundTransitConstraint,
    get_location2D_list,
    compute_bidir_astar_euclidean,
    CoordinatedMAPFEnv,
    # update_aerial_ground_paths_cbs_solution!,
    set_ground_transit_graph!,
    compute_independent_paths
    # aerial_ground_coord_path_cost

# export
#     ALTInfo,
#     greedy_landmarks,
#     landmark_distances,
#     BidirALT

include("types.jl")
include("utils.jl")
include("stage1_paths.jl")
include("ground_transit_pathfinding.jl")
# include("astar_landmarks.jl")

end
