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
using DataStructures
using StatsFuns
using IterTools

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
    GroundMAPFState,
    GroundMAPFAction,
    GroundMAPFConflict,
    GroundMAPFConstraint

export
    get_location2D_list,
    compute_bidir_astar_euclidean,
    CoordinatedMAPFEnv,
    update_aerial_ground_paths_cbs_solution!,
    set_ground_transit_graph!,
    compute_independent_paths,
    get_tasks_with_valid_path,
    augment_road_graph_with_aerial_paths!,
    update_ground_paths_with_ground_mapf_result!,
    compute_total_cost,
    count_cars_not_coordinating


include("types.jl")
include("utils.jl")
include("stage1_paths.jl")
include("ground_transit_pathfinding.jl")

end
