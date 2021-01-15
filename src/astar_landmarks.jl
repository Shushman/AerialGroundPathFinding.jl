## Implements the routines for ALT algorithms
@with_kw struct ALTInfo{T <: Real, U <: Integer}
    all_landmarks::Vector{U}
    from_distances::Matrix{T}
    to_distances::Matrix{T}
end


function greedy_landmarks(g::AbstractGraph{U}, distmx::AbstractMatrix{T}, num_landmarks::Int64, rng::RNG=Random.GLOBAL_RNG) where {T <: Real, U <: Integer, RNG <: AbstractRNG}

    all_landmarks = Vector{U}()
    nvg = nv(g)
    dijkstra = ShortestPaths.Dijkstra()

    # choose start landmark at random
    start_lm = rand(rng, 1:nvg)
    start_dijkstra_res = ShortestPaths.shortest_paths(g, start_lm, distmx, dijkstra)
    start_dists = start_dijkstra_res.dists .* (start_dijkstra_res.dists .!= Inf)
    _, furthest = findmax(start_dists)
    push!(all_landmarks, furthest)

    for i = 2:num_landmarks

        max_lm_dists = zeros(T, nvg)

        for l in all_landmarks
            dijkstra_res = ShortestPaths.shortest_paths(g, l, distmx, dijkstra)
            noninf_dists = dijkstra_res.dists .* (dijkstra_res.dists .!= Inf)

            max_lm_dists = max_lm_dists.*(max_lm_dists .> noninf_dists) + noninf_dists.*(noninf_dists .> max_lm_dists)
        end

        # Get rid of inf terms
        for l in all_landmarks
            max_lm_dists[l] = zero(T)
        end

        dist, ind = findmax(max_lm_dists)

        # If every vertex unreachable from at least one landmark, stop
        if dist == zero(T)
            @warn "Can add no more landmarks; exiting!"
            return all_landmarks
        end
        
        push!(all_landmarks, ind)
    end

    return all_landmarks
end # greedy_landmarks


"""
Returns 2 |LANDMARKS| x |V| matrices of distances from and to landmark
"""
function landmark_distances(g::AbstractGraph, landmarks::Vector{U}, distmx::AbstractMatrix{T}) where {T <: Real, U <: Integer}

    from_dijkstra = ShortestPaths.Dijkstra(neighborfn=outneighbors)
    to_dijkstra = ShortestPaths.Dijkstra(neighborfn=inneighbors)

    nvg = nv(g)
    nlandmarks = length(landmarks)
    from_distances = fill(typemax(T), nlandmarks, nvg)
    to_distances = fill(typemax(T), nlandmarks, nvg)

    for (i, l) in enumerate(landmarks)

        # TODO: Can we use the multiple src option in Dijkstra here? What does that even do...
        from_dijkstra_res = ShortestPaths.shortest_paths(g, l, distmx, from_dijkstra)
        from_distances[i, :] = transpose(from_dijkstra_res.dists)

        to_dijkstra_res = ShortestPaths.shortest_paths(g, l, distmx, to_dijkstra)
        to_distances[i, :] = transpose(to_dijkstra_res.dists)

    end # enumerate(landmarks)

    return from_distances, to_distances
end # function compute_landmark_distances

function lower_bound_distance(alt_info::ALTInfo{T,U}, s::U, t::U) where {T <: Real, U <: Integer}

    lower_bound = zero(T)

    # Iterate over landmarks to compute tightest lower bound on s-t distance
    for (i, l) in enumerate(alt_info.all_landmarks)

        ldist = max(alt_info.to_distances[i, s] - alt_info.to_distances[i, t], 
                    alt_info.from_distances[i, t] - alt_info.from_distances[i, s])
        
        if ldist != Inf && ldist > lower_bound
            lower_bound = ldist
        end

    end # enumerate alt_info.all_landmarks

    return lower_bound
end # function active_landmark_ids

struct BidirALT <: ShortestPaths.ShortestPathAlgorithm
    alt_info::ALTInfo
end

function ShortestPaths.shortest_paths(g::AbstractGraph{U}, s::U, t::U, distmx::AbstractMatrix{T}, alg::BidirALT) where {U<:Integer, T<:Real}

    # Bind heuristic methods to alt info and s/t
    fwd_heuristic(u, t) = lower_bound_distance(alg.alt_info, u, t)
    bwd_heuristic(u, s) = lower_bound_distance(alg.alt_info, s, u)

    bidir_astar = ShortestPaths.BidirAStar(fwd_heuristic, bwd_heuristic)

    return ShortestPaths.shortest_paths(g, s, t, distmx, bidir_astar)
end # function shortest_paths