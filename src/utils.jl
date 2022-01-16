"""
    distance_lat_lon_euclidean(coords1::Location2D, coords2::Location2D)

Compute the Euclidean distance in metres between two lat-long coordinates close to each other.
"""
function distance_lat_lon_euclidean(coords1::Location2D, coords2::Location2D)
    deglen = 110.25
    x = coords1[1]- coords2[1]
    y = (coords1[2] - coords2[2])*cos(coords2[1])
    return deglen*sqrt(x^2 + y^2)*1000.0
end

Distances.evaluate(::EuclideanLatLongMetric, coords1::Location2D, coords2::Location2D) = distance_lat_lon_euclidean(coords1, coords2)

"""
    get_location2D_list(node_attribs_file::String)

Returns the list of parsed 2D locations for the road graph nodes.
"""
function get_location2D_list(node_attribs_file::String)

    node_attribs_dict = JSON.parsefile(node_attribs_file)
    nnodes = length(keys(node_attribs_dict))

    location_list = Vector{Location2D}(undef, nnodes)
    for (k, v) in node_attribs_dict
        location_list[parse(Int64,k)] = Location2D(parse(Float64, v["y"]), parse(Float64, v["x"]))
    end

    return location_list
end

function compute_bidir_astar_euclidean(g::LightGraphs.AbstractGraph, s::Int64, t::Int64, distmx::AbstractMatrix{Float64}, node_locations::Vector{Location2D}, nbrs_to_exclude::Set{Int64}=Set{Int64}())

    if s == t
        return (path=[s], dist=0.0)
    end

    metric = EuclideanLatLongMetric()
    fwd_heuristic(u, t) = Distances.evaluate(metric, node_locations[u], node_locations[t])
    bwd_heuristic(u, s) = Distances.evaluate(metric, node_locations[s], node_locations[u])

    bidir_astar = ShortestPaths.BidirAStar(fwd_heuristic, bwd_heuristic, nbrs_to_exclude)

    return ShortestPaths.shortest_paths(g, s, t, distmx, bidir_astar)
end

"""
    get_non_dominated_transit_points(env::CoordinatedMAPFEnv, gstart::Int64, gend::Int64, avoid_vertex_set::Set{Int64}, v::AerialMAPFState, flight_heur::Bool)

Returns the ground transit points that are non-dominated on both time and flight distance.
"""
function get_non_dominated_transit_points(env::CoordinatedMAPFEnv, gstart::Int64, gend::Int64, avoid_vertex_set::Set{Int64}, v::AerialMAPFState, flight_heur::Bool)

    time_dist_set = NamedTuple{(:idx, :timeval, :distval), Tuple{Int64, Float64, Float64}}[]
    vgtg = env.ground_transit_graph.vertices[v.ground_transit_idx]

    for seq = gstart:gend-1
        # As soon as you hit a conflict vertex, stop
        if (seq in avoid_vertex_set)
            break
        end
        seq_gtg = env.ground_transit_graph.vertices[seq]

        ddiff = flight_heur ? distance_lat_lon_euclidean(env.location_list[vgtg.road_vtx_id], env.location_list[seq_gtg.road_vtx_id]) : env.ground_transit_weights[(v.ground_transit_idx, seq_gtg.idx)]
        tval = max(v.timeval + ddiff/AvgSpeed, seq_gtg.timeval)

        push!(time_dist_set, (idx=seq_gtg.idx, timeval=tval, distval=v.distval+ddiff))
    end # seq = gstart:gend-1

    if isempty(time_dist_set)
        return []
    end

    # Already sorted in increasing time
    non_dom_idxs = [1]

    for (i, tup) in enumerate(time_dist_set)
        dom = false
        for ndi in non_dom_idxs
            if tup.distval >= time_dist_set[ndi].distval
                dom=true
                break
            end
        end

        if dom==false
            push!(non_dom_idxs, i)
        end
    end

    non_dom_set = time_dist_set[non_dom_idxs]

    return non_dom_set
end

"""
    get_best_transit_connection(env::CoordinatedMAPFEnv, gstart::Int64, gend::Int64, v::AerialMAPFState, flight_heur::Bool)

Returns the lowest-cost ground transit connection point.
"""
function get_best_transit_connection(env::CoordinatedMAPFEnv, gstart::Int64, gend::Int64, v::AerialMAPFState, flight_heur::Bool)

    best_weighted_cost = Inf

    vgtg = env.ground_transit_graph.vertices[v.ground_transit_idx]

    result = (idx=0, timeval=0.0, distval=0.0)

    for seq = gstart:gend-1
        # As soon as you hit a conflict vertex, stop    
        seq_gtg = env.ground_transit_graph.vertices[seq]
        ddiff = flight_heur ? distance_lat_lon_euclidean(env.location_list[vgtg.road_vtx_id], env.location_list[seq_gtg.road_vtx_id]) : env.ground_transit_weights[(v.ground_transit_idx, seq_gtg.idx)]
        tval = max(v.timeval + ddiff/AvgSpeed, seq_gtg.timeval)
        dval = v.distval + ddiff
        tdiff = tval - seq_gtg.timeval

        weighted_cost = env.alpha_weight_distance*ddiff + (1.0-env.alpha_weight_distance)*tdiff
        if weighted_cost < best_weighted_cost
            best_weighted_cost = weighted_cost
            result = (idx=seq_gtg.idx, timeval=tval, distval=dval)
        end
    end # seq = gstart:gend-1

    if best_weighted_cost < Inf
        return result
    else
        return nothing
    end
end


function get_tasks_with_valid_path(road_graph::LightGraphs.AbstractGraph, road_graph_wts::AbstractMatrix{Float64}, location_list::Vector{Location2D}, num_tasks::Int64, rng::AbstractRNG)

    nvg = nv(road_graph)
    task_list = Vector{AgentTask}(undef, num_tasks)

    for t = 1:num_tasks

        path_exists = false

        start = -1
        goal = -1

        while ~path_exists
            start = rand(rng, 1:nvg)
            goal = rand(rng, 1:nvg)

            bidir_sp = compute_bidir_astar_euclidean(road_graph, start, goal, road_graph_wts, location_list)

            path_exists = (isempty(bidir_sp.path) == false)
                
        end
        task_list[t] = AgentTask((start, goal))
    end

    return task_list
end

"""
    Computes sum-of-costs of the paths.
"""
function compute_total_cost(env::CoordinatedMAPFEnv, ground_paths::Vector{AgentPathInfo}, aerial_paths::Vector{AgentPathInfo})

    total_dist = sum(ap.total_dist for ap in aerial_paths) + sum(gp.total_dist for gp in ground_paths)
    total_time = sum(ap.total_time for ap in aerial_paths) + sum(gp.total_time for gp in ground_paths)

    return env.alpha_weight_distance*total_dist + (1.0-env.alpha_weight_distance)*total_time
end

function count_cars_not_coordinating(ground_paths::Vector{AgentPathInfo})

    not_coord_ids = Set{Int64}()

    for (gid, gpath) in enumerate(ground_paths)

        is_coord = false
        for gps in gpath.path_states
            if ~isempty(gps.coord_agent_id)
                is_coord = true
                break
            end
        end
        if ~is_coord
            push!(not_coord_ids, gid)
        end
    end
    return not_coord_ids
end

function get_increasing_dist_ordering(agent_paths::Vector{AgentPathInfo})

    id_dist_tuples = [(id, path.total_dist) for (id, path) in enumerate(agent_paths)]
    sort!(id_dist_tuples, by=x->x[2], rev=true)

    return [idd[1] for idd in id_dist_tuples]
end