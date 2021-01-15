## Various helper functions for working with latitude and longitude
const Location2D = SVector{2, Float64}
const LatLonCoords = NamedTuple{(:lat, :lon), Tuple{Float64,Float64}}
LatLonCoords() = (lat = 0.0, lon = 0.0)

"""
Return the vector form of a latitude-longitude point [lat, lon]
"""
function convert_to_vector(c::LatLonCoords)
    return Location2D(c.lat, c.lon)
end

struct EuclideanLatLongMetric <: Metric
end


const AgentTask = NamedTuple{(:origin, :dest), Tuple{Int64, Int64}}


# MAPF types - for drones over cars
@with_kw struct GroundTransitState <: MAPFState
    idx::Int64                  # Only for reverse lookup
    ground_agent_id::Int64      = 0     # When 0, indicates aerial start-goal
    vertex_along_path::Int64    = 0 # Goes from 1 to path length
    road_vtx_id::Int64          # vertex ID in road graph
end

Graphs.vertex_index(g::SimpleVListGraph{GroundTransitState}, v::GroundTransitState) = v.idx


@enum ActionType Fly=1 Stay=2
struct GroundTransitAction <: MAPFAction
    action::ActionType
end

@with_kw struct GroundTransitConflict <: MAPFConflict
    overlap_vertices::Set{Int64}          # Which TRANSIT GRAPH vertex IDs are in the conflicting set
    aerial_agent_ids::Tuple{Int64,Int64}
end

@with_kw struct GroundTransitConstraint <: MAPFConstraints
    avoid_vertex_set::Set{Int64}    = Set{Int64}()
end
Base.isempty(gtc::GroundTransitConstraint) = isempty(gtc.avoid_vertex_set)

@with_kw mutable struct AgentPathState
    road_vtx_id::Int64
    coord_agent_id::Int64=0     # Of the other type
end
@with_kw mutable struct AgentPathInfo
    path_states::Vector{AgentPathState} = AgentPathState[]
    cost_to_agent::Float64              = 0.0
end

@with_kw mutable struct CoordinatedMAPFEnv <: MAPFEnvironment
    ground_task_list::Vector{AgentTask}
    aerial_task_list::Vector{AgentTask}
    road_graph::LightGraphs.AbstractGraph
    road_graph_wts::AbstractMatrix{Float64}
    location_list::Vector{Location2D}
    ground_paths::Vector{AgentPathInfo} = AgentPathInfo[]
    aerial_paths::Vector{AgentPathInfo} = AgentPathInfo[]
    ground_transit_graph::SimpleVListGraph{GroundTransitState} = SimpleVListGraph{GroundTransitState}()
    ground_transit_weights::Dict{Tuple{Int64,Int64}, Float64} = Dict{Tuple{Int64,Int64}, Float64}()
    ground_transit_expanded_paths::Dict{Tuple{Int64,Int64}, Vector{Int64}} = Dict{Tuple{Int64,Int64}, Vector{Int64}}()
    gtg_ground_path_start_ends::Vector{Tuple{Int64,Int64}} = Tuple{Int64,Int64}[]
    gtg_drone_start_goal_idxs::Vector{Tuple{Int64,Int64}} = Tuple{Int64,Int64}[]
    next_gtg_goal_idx::Int64 = 0
    num_global_conflicts::Int64 = 0
    threshold_global_conflicts::Int64   = typemax(Int64)
end