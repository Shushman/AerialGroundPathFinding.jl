function Distances.evaluate(::EuclideanLatLongMetric,
                            coords1::Location2D,
                            coords2::Location2D)
    deglen = 110.25
    x = coords1[1]- coords2[1]
    y = (coords1[2] - coords2[2])*cos(coords2[1])
    return deglen*sqrt(x^2 + y^2)*1000.0
end

function get_location2D_list(node_attribs_file::String)

    node_attribs_dict = JSON.parsefile(node_attribs_file)
    nnodes = length(keys(node_attribs_dict))

    location_list = Vector{Location2D}(undef, nnodes)
    for (k, v) in node_attribs_dict
        location_list[parse(Int64,k)] = Location2D(parse(Float64, v["y"]), parse(Float64, v["x"]))
    end

    return location_list
end

function compute_bidir_astar_euclidean(g::LightGraphs.AbstractGraph, s::Int64, t::Int64, distmx::AbstractMatrix{Float64}, node_locations::Vector{Location2D})

    metric = EuclideanLatLongMetric()
    fwd_heuristic(u, t) = Distances.evaluate(metric, node_locations[u], node_locations[t])
    bwd_heuristic(u, s) = Distances.evaluate(metric, node_locations[s], node_locations[u])

    bidir_astar = ShortestPaths.BidirAStar(fwd_heuristic, bwd_heuristic)

    return ShortestPaths.shortest_paths(g, s, t, distmx, bidir_astar)
end