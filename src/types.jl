## Various helper functions for working with latitude and longitude
const Location2D = SVector{2, Float64}
const LatLonCoords = NamedTuple{(:lat, :lon), Tuple{Float64,Float64}}


"""
Return the vector form of a latitude-longitude point [lat, lon]
"""
function convert_to_vector(c::LatLonCoords)
    return Location2D(c.lat, c.lon)
end

struct EuclideanLatLongMetric <: Metric
end


const AgentTask = NamedTuple{(:origin, :dest), Tuple{Int64, Int64}}
const AvgSpeed = 8.0  # 8 m/s for drones and cars 
const MaxHop = 3


@enum ActionType Fly=1 Stay=2
struct GroundTransitAction <: MAPFAction
    action::ActionType
end

@with_kw struct GroundTransitConflict <: MAPFConflict
    overlap_gtg_vertices::Set{Int64}          # Which TRANSIT GRAPH vertex IDs are in the conflicting set
    aerial_agent_ids::Vector{Int64}
end

@with_kw struct GroundTransitConstraint <: MAPFConstraints
    avoid_gid_vertex_set::Set{Int64}    = Set{Int64}()
end
Base.isempty(gtc::GroundTransitConstraint) = isempty(gtc.avoid_gtg_vertex_set)


# For the graph of ground time-stamped waypoints
# Must be mutable as cars can wait for drones
@with_kw mutable struct GroundTransitState
    idx::Int64                  # Only for reverse lookup
    ground_agent_id::Int64 = 0      # When 0, indicates aerial start-goal
    vertex_along_path::Int64 = 0    # Goes from 1 to path length
    road_vtx_id::Int64          # Vertex ID in road graph
    timeval::Float64 = 0.0              # Global time of ground vehicle
end

Graphs.vertex_index(g::SimpleVListGraph{GroundTransitState}, v::GroundTransitState) = v.idx

# MAPF State for aerial
# Thin wrapper around GTG vertices, with just drone time 
@with_kw struct AerialMAPFState <: MAPFState
    idx::Int64
    ground_transit_idx::Int64   # Which TRANSIT GRAPH vertex ID
    timeval::Float64 = 0.0  # Only relevant for transit points
    distval::Float64 = 0.0
    considered_cars::Set{Int64} = Set{Int64}()
end

## Types for ground MAPF
@with_kw struct GroundMAPFState <: MAPFState
    road_vtx_id::Int64
    is_aerial::Bool
end
Base.isequal(s1::GroundMAPFState, s2::GroundMAPFState) = (s1.road_vtx_id == s2.road_vtx_id && s1.is_aerial == s2.is_aerial)

@with_kw mutable struct GMAPFStateInfo
    idx::Int64
    gval::Float64
    aerial_ids::Set{Int64}
end

# What is the drone ID for the edge it is taking? (0 if standard)
@with_kw struct GroundMAPFAction <: MAPFAction
    edge_id::LightGraphs.Edge
    edge_aerial_ids::Set{Int64}
end

# Two or more cars that traverse the same edge for a particular drone ID
@with_kw struct GroundMAPFConflict <: MAPFConflict
    ground_id_pair::Tuple{Int64,Int64}
    conflicting_edge_aerial_ids::Vector{Tuple{LightGraphs.Edge,Set{Int64}}}
end

# Constraint is to avoid a particular copy of the edge
# Maps edge ID to the aerial IDs to avoid
@with_kw struct GroundMAPFConstraint <: MAPFConstraints
    avoid_edge_copy_set::Dict{LightGraphs.Edge,Set{Int64}} = Dict{LightGraphs.Edge,Set{Int64}}()
end
Base.isempty(gpc::GroundMAPFConstraint) = isempty(gpc.avoid_edge_copy_set)


@with_kw mutable struct AgentPathState
    road_vtx_id::Int64
    timeval::Float64
    coord_agent_id::Int64 = 0     # Of the other type
end
@with_kw mutable struct AgentPathInfo
    path_states::Vector{AgentPathState} = AgentPathState[]
    total_dist::Float64                 = 0.0
    total_time::Float64                 = 0.0
end

@with_kw mutable struct EdgeCopyInfo
    aerial_id_to_weight::Dict{Int64, Float64} = Dict{Int64, Float64}()
end

@with_kw mutable struct CoordinatedMAPFEnv <: MAPFEnvironment
    ground_task_list::Vector{AgentTask}
    aerial_task_list::Vector{AgentTask}
    road_graph::LightGraphs.AbstractGraph
    road_graph_wts::AbstractMatrix{Float64}
    location_list::Vector{Location2D}
    alpha_weight_distance::Float64
    car_capacity::Int64
    ground_paths::Vector{AgentPathInfo} = AgentPathInfo[]
    aerial_paths::Vector{AgentPathInfo} = AgentPathInfo[]
    ground_transit_graph::SimpleVListGraph{GroundTransitState} = SimpleVListGraph{GroundTransitState}()
    aerial_mapf_graph::SimpleVListGraph{AerialMAPFState} = SimpleVListGraph{AerialMAPFState}()
    ground_transit_weights::Dict{Tuple{Int64,Int64}, Float64} = Dict{Tuple{Int64,Int64}, Float64}()
    ground_transit_expanded_paths::Dict{Tuple{Int64,Int64}, Vector{Int64}} = Dict{Tuple{Int64,Int64}, Vector{Int64}}()
    gtg_ground_path_start_ends::Vector{Tuple{Int64,Int64}} = Tuple{Int64,Int64}[]
    gtg_drone_start_goal_idxs::Vector{Tuple{Int64,Int64}} = Tuple{Int64,Int64}[]
    next_gtg_goal_idx::Int64 = 0
    next_amapfg_goal_idx::Int64=0
    num_global_conflicts::Int64 = 0
    threshold_global_conflicts::Int64   = typemax(Int64)
    gtg_idx_gval::Dict{Int64, Float64} = Dict{Int64, Float64}()
    edge_to_copy_info::Dict{LightGraphs.Edge, EdgeCopyInfo} = Dict{LightGraphs.Edge, EdgeCopyInfo}()
    ground_mapf_graph::SimpleVListGraph{GroundMAPFState} = SimpleVListGraph{GroundMAPFState}()
    unique_gmapf_states::Dict{GroundMAPFState,GMAPFStateInfo} = Dict{GroundMAPFState,Tuple{Int64,Float64}}()
    next_gmapfg_goal_idx::Int64 = 0
end

## NOTES
# 1. Transit Graph must be different from MAPF States if we are tracking time