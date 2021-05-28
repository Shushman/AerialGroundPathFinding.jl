function compute_independent_paths(env::CoordinatedMAPFEnv, task_list::Vector{AgentTask})

    independent_paths = Vector{AgentPathInfo}(undef, length(task_list))

    for (agent_id, task) in enumerate(task_list)

        bidir_sp = compute_bidir_astar_euclidean(env.road_graph, task.origin, task.dest, env.road_graph_wts, env.location_list)

        path_states = AgentPathState[AgentPathState(road_vtx_id=bidir_sp.path[1], timeval=0.0)]
        
        cumul_time = 0.0
        for (idx, idx_next) in zip(bidir_sp.path[1:end-1], bidir_sp.path[2:end])
            cumul_time += env.road_graph_wts[idx, idx_next]/AvgSpeed
            push!(path_states, AgentPathState(road_vtx_id=idx_next, timeval=cumul_time))
        end
        
        independent_paths[agent_id] = AgentPathInfo(path_states=path_states, total_dist=bidir_sp.dist, total_time=cumul_time)
    end

    return independent_paths
end

struct EdgeCopyQueueType
    hop::Int64
    src::Int64
end

# Use drone paths to augment road graph
# For k hops from each drone path edge, annotate edges with drone ID and reduced weight
# Where W' = W*logistic(k) where k is the shortest of hops from the drone path
function augment_road_graph_with_aerial_paths!(env::CoordinatedMAPFEnv, direct_drone_paths::Vector{AgentPathInfo})


    # Iterate over drone paths
    for (aerial_id, ddp) in enumerate(direct_drone_paths)
        
        ddp_states = ddp.path_states

        # Temp data structure for closest distance to actual edge
        edge_copy_min_hop = Dict{Tuple{Int64,Int64},Int64}()
        edge_copy_src_to_hop = Dict{Int64,Int64}()

        # Iterate over path states and run outward search from
        for i = 1:length(ddp_states) - 1

            # First add edge on path to copy dict; must be unique
            edge_copy_min_hop[(ddp_states[i].road_vtx_id, ddp_states[i+1].road_vtx_id)] = 0
            edge_copy_src_to_hop[ddp_states[i].road_vtx_id] = 0

            # Now work outward from ddp_states[i]
            edge_copy_queue = Queue{EdgeCopyQueueType}()
            for onbr in outneighbors(env.road_graph, ddp_states[i].road_vtx_id)
                if onbr != ddp_states[i+1].road_vtx_id
                    enqueue!(edge_copy_queue, EdgeCopyQueueType(1, onbr))
                end
            end
            
            while ~(isempty(edge_copy_queue))
                
                qtop = dequeue!(edge_copy_queue)
                curr_id = qtop.src

                # Compute the min of current hop of qtop.src and MaxHop
                min_hop_for_src = get(edge_copy_src_to_hop, curr_id, MaxHop)

                # Only jump into it if it was not already in edge_copy_src_to_hop
                if qtop.hop < min_hop_for_src

                    # First update for the src itself
                    edge_copy_src_to_hop[curr_id] = qtop.hop

                    curr_hop = qtop.hop + 1
                    
                    # Iterate over outneighbors and add to edge_copy_min_dist if curr_hop < min_hop
                    # In any case, add to queue with curr_hop
                    for onbr in outneighbors(env.road_graph, curr_id)

                        # TODO: Check later that curr_hop is necessarily better than whatever edge_copy_min_hop has
                        edge_copy_min_hop[(curr_id, onbr)] = curr_hop

                        # Enqueue onbr with curr_hop
                        enqueue!(edge_copy_queue, EdgeCopyQueueType(curr_hop, onbr))

                    end # onbr in out_neighbors(ddp_states[i])
                end # qtop.hop < min_hop_for_src
            end # ~(isempty(edge_copy_queue))
        end # i = 1:length(ddp_states) - 1

        # now iterate over edge_copy_min_hop and add copies
        for (src_dst, hop) in edge_copy_min_hop

            e = LightGraphs.Edge(src_dst[1], src_dst[2])

            # Initialize dictionary for edge if it doesn't exist
            if ~haskey(env.edge_to_copy_info, e)
                env.edge_to_copy_info[e] = EdgeCopyInfo()
            end

            reduced_wt = env.road_graph_wts[e.src, e.dst] * logistic(hop)
            env.edge_to_copy_info[e].aerial_id_to_weight[aerial_id] = reduced_wt

        end # (src_dst, hop) in edge_copy_min_hop

    end # ddp in direct_drone_paths


end # function augment_road_graph_with_aerial_paths!


# Define MAPF functions for ground paths
## Solve the subproblem of finding aerial paths over the ground transit network
function MultiAgentPathFinding.add_constraint!(cbase::GroundMAPFConstraint, cadd::GroundMAPFConstraint)
    
    # iterate over edges and IDs of cadd and add new key to cbase or augment existing set
    for (edge, aerial_ids) in cadd.avoid_edge_copy_set
        if haskey(cbase.avoid_edge_copy_set, edge)
            union!(cbase.avoid_edge_copy_set[edge], aerial_ids)
        else
            cbase.avoid_edge_copy_set[edge] = deepcopy(aerial_ids)
        end
    end # (edge, aerial_ids) in cadd.avoid_edge_copy_set
end

function MultiAgentPathFinding.overlap_between_constraints(cbase::GroundMAPFConstraint, cadd::GroundMAPFConstraint)
    for common_key in intersect(keys(cbase.avoid_edge_copy_set, cadd.avoid_edge_copy_set))
        if ~(isempty(intersect(cbase.avoid_edge_copy_set[common_key], cadd.avoid_edge_copy_set[common_key])))
            return true
        end
    end
    return false
end

get_mapf_ground_action(env::CoordinatedMAPFEnv, u::GroundMAPFState, v::GroundMAPFState) = GroundMAPFAction(LightGraphs.Edge(u.road_vtx_id, v.road_vtx_id), env.unique_gmapf_states[v].aerial_ids)


MultiAgentPathFinding.get_empty_constraint(::Type{GroundMAPFConstraint}) = GroundMAPFConstraint()

# TODO
function MultiAgentPathFinding.create_constraints_from_conflict(env::CoordinatedMAPFEnv, conflict::GroundMAPFConflict)

    # Iterate over conflicting edge Ids and create one element sets of edges to avoid of a drone ID
    edge_set_to_avoid = Dict{LightGraphs.Edge,Set{Int64}}()
    for (edge, aerial_ids) in conflict.conflicting_edge_aerial_ids
        edge_set_to_avoid[edge] = aerial_ids
    end

    # Now just repeat the constraint for each member of pair
    gid_pair = conflict.ground_id_pair
    constraint = Dict{Int64,GroundMAPFConstraint}(gid_pair[1] => GroundMAPFConstraint(edge_set_to_avoid), gid_pair[2] => GroundMAPFConstraint(edge_set_to_avoid))
    # @infiltrate
    # @show constraint
    return [constraint]
end

# TODO: Review whether to add all edges or just some
function MultiAgentPathFinding.get_first_conflict(env::CoordinatedMAPFEnv, solution::Vector{PlanResult{GroundMAPFState,GroundMAPFAction,Float64}})

    # For the first two trucks that share one or more edges of same drones, return
    for (i, sol_i) in enumerate(solution[1:end-1])
        for (j, sol_j) in enumerate(solution[i+1:end])

            other_ground_id = i+j
            overlapping_aerial_edges = Vector{Tuple{LightGraphs.Edge,Set{Int64}}}(undef, 0)

            for (i_action, _) in sol_i.actions
                for (j_action, _) in sol_j.actions
                    if i_action.edge_id == j_action.edge_id && ~isempty(i_action.edge_aerial_ids) && i_action.edge_aerial_ids == j_action.edge_aerial_ids
                        push!(overlapping_aerial_edges, (i_action.edge_id, i_action.edge_aerial_ids))
                    end # i_action.edge_aerial_id != 0 && i_action == j_action
                end # (sj, (state_j, _)) in enumerate(sol_j.states[2:end])
            end # (si, (state_i, _)) in enumerate(sol_i.states[2:end])

            # NOTE: Any overlap blows up at 15 cars
            if ~isempty(overlapping_aerial_edges)
                env.num_global_conflicts += 1
                # println("Ground conflict no. $(env.num_global_conflicts)")
                if env.num_global_conflicts > env.threshold_global_conflicts
                    throw(DomainError("Too many conflicts!"))
                end
                # @infiltrate
                return GroundMAPFConflict((i, other_ground_id), overlapping_aerial_edges)
            end
        end # (j, sol_j) in enumerate(solution[i+1:end])
    end # (i, sol_i) in enumerate(solution[1:end-1])

    return nothing
end

function ground_mapf_heuristic(env::CoordinatedMAPFEnv, road_vtx_goal::Int64, u::GroundMAPFState)
    return distance_lat_lon_euclidean(env.location_list[u.road_vtx_id], env.location_list[road_vtx_goal])
end

function generate_ground_mapf_edge_weight(env::CoordinatedMAPFEnv, u::GroundMAPFState, v::GroundMAPFState, aerial_ids::Set{Int64})
    if isempty(aerial_ids)
        return env.road_graph_wts[u.road_vtx_id, v.road_vtx_id]
    else
        # Subtract cost from aerial IDs carried
        base_wt = env.road_graph_wts[u.road_vtx_id, v.road_vtx_id]
        uv_edge = LightGraphs.Edge(u.road_vtx_id, v.road_vtx_id)
        cumul_wt = base_wt

        # Subtract cost diff from base for each drone edge
        for aid in aerial_ids
            # if !haskey(env.edge_to_copy_info[uv_edge].aerial_id_to_weight, aid)
            #     @infiltrate
            # end
            cumul_wt -= (base_wt - env.edge_to_copy_info[uv_edge].aerial_id_to_weight[aid])
        end
        # @infiltrate
        return cumul_wt
    end
end

function get_ground_mapf_edge_weight(env::CoordinatedMAPFEnv, u::GroundMAPFState, v::GroundMAPFState)
    return (env.unique_gmapf_states[v].gval - env.unique_gmapf_states[u].gval) 
end


function MultiAgentPathFinding.set_low_level_context!(::CoordinatedMAPFEnv, ::Int64, ::GroundMAPFConstraint)
end

function MultiAgentPathFinding.focal_heuristic(env::CoordinatedMAPFEnv, solution::Vector{PlanResult{GroundMAPFState,GroundMAPFAction,Float64}})
    
    num_potential_conflicts = 0
    
    for (i, sol_i) in enumerate(solution[1:end-1])
        for (j, sol_j) in enumerate(solution[i+1:end])

            other_ground_id = i+j
            num_overlapping_aerial_edges = 0

            for (i_action, _) in sol_i.actions
                for (j_action, _) in sol_j.actions
                    if i_action.edge_id == j_action.edge_id && ~isempty(i_action.edge_aerial_ids) && i_action.edge_aerial_ids == j_action.edge_aerial_ids
                        num_overlapping_aerial_edges += 1
                    end # i_action.edge_aerial_id != 0 && i_action == j_action
                end # (sj, (state_j, _)) in enumerate(sol_j.states[2:end])
            end # (si, (state_i, _)) in enumerate(sol_i.states[2:end])

            # NOTE: Think carefully about conflict criterion
            if num_overlapping_aerial_edges > 0
                num_potential_conflicts += 1
            end
        end # (j, sol_j) in enumerate(solution[i+1:end])
    end # (i, sol_i) in enumerate(solution[1:end-1])

    return num_potential_conflicts
end


# Search over augmented road graph
function MultiAgentPathFinding.low_level_search!(solver::ECBSSolver, ground_agent_id::Int64, s::GroundMAPFState, constraint::GroundMAPFConstraint, solution::Vector{PlanResult{GroundMAPFState,GroundMAPFAction,Float64}})

    env = solver.env

    # Create temp graph as thin wrapper on road graph for search
    env.ground_mapf_graph = SimpleVListGraph{GroundMAPFState}()
    empty!(env.unique_gmapf_states)
    
    Graphs.add_vertex!(env.ground_mapf_graph, s)
    env.unique_gmapf_states[s] = GMAPFStateInfo(1, 0.0, Set{Int64}())
    
    road_vtx_goal = env.ground_task_list[ground_agent_id].dest
    admissible_heuristic(u) = ground_mapf_heuristic(env, road_vtx_goal, u)
    edge_wt_fn(u, v) = get_ground_mapf_edge_weight(env, u, v)

    vis = GroundMAPFGoalVisitor(env, road_vtx_goal, constraint.avoid_edge_copy_set)

    start_idx = 1
    env.next_gmapfg_goal_idx = 0

    # @infiltrate

    a_star_states = a_star_implicit_shortest_path!(env.ground_mapf_graph, edge_wt_fn, start_idx, vis, admissible_heuristic)

    # @show length(env.ground_mapf_graph.vertices)


    @assert haskey(a_star_states.parent_indices, env.next_gmapfg_goal_idx)

    states = Tuple{GroundMAPFState,Float64}[]
    actions = Tuple{GroundMAPFAction,Float64}[]

    sp_cost = a_star_states.dists[env.next_gmapfg_goal_idx]

    sp_idxs = shortest_path_indices(a_star_states.parent_indices, env.ground_mapf_graph, start_idx, env.next_gmapfg_goal_idx)

    push!(states, (s, 0.0))

    for i = 2:length(sp_idxs)

        u_idx, v_idx =  (sp_idxs[i-1], sp_idxs[i])

        new_state = deepcopy(env.ground_mapf_graph.vertices[v_idx])
        push!(states, (new_state, a_star_states.dists[v_idx]))

        push!(actions, (get_mapf_ground_action(env, env.ground_mapf_graph.vertices[u_idx], env.ground_mapf_graph.vertices[v_idx]), a_star_states.dists[v_idx] - a_star_states.dists[u_idx]))
    end

    # @infiltrate

    return PlanResult{GroundMAPFState,GroundMAPFAction,Float64}(states, actions, sp_cost, sp_cost)
end


struct GroundMAPFGoalVisitor <: Graphs.AbstractDijkstraVisitor
    env::CoordinatedMAPFEnv
    road_vtx_goal::Int64
    avoid_edge_copy_set::Dict{LightGraphs.Edge,Set{Int64}}
end


function Graphs.include_vertex!(vis::GroundMAPFGoalVisitor, u::GroundMAPFState, v::GroundMAPFState, d::Float64, nbrs::Vector{Int64})

    env = vis.env
    road_vtx_goal = vis.road_vtx_goal
    avoid_edge_copy_set = vis.avoid_edge_copy_set

    # When we expand actual goal, exit
    if v.road_vtx_id == road_vtx_goal
        env.next_gmapfg_goal_idx = env.unique_gmapf_states[v].idx
        return false
    end

    # Simple, just look up outneighbors of v.road_vtx_id in the true road graph
    # For each outneighbor, if no edge copies, just add onbr with aerial ID 0
    # Otherwise, add the outnbr with lowest cost among edge copies if not in avoid_edge_set
    idx_ctr = Graphs.num_vertices(env.ground_mapf_graph)

    for onbr in outneighbors(env.road_graph, v.road_vtx_id)

        vnbr_edge = LightGraphs.Edge(v.road_vtx_id, onbr)
        aids_to_add_with = Set{Int64}()
        is_aerial = false
        v_aids = env.unique_gmapf_states[v].aerial_ids

        if haskey(env.edge_to_copy_info, vnbr_edge)
            
            edge_copies_id_to_wt = env.edge_to_copy_info[vnbr_edge].aerial_id_to_weight
            
            aerial_ids_to_avoid = get(avoid_edge_copy_set, vnbr_edge, Set{Int64}())
            aerial_ids_to_consider = setdiff(Set{Int64}(keys(edge_copies_id_to_wt)), aerial_ids_to_avoid)

            # Find lowest weight edges up to min(capacity, aerial_ids)
            if ~(isempty(aerial_ids_to_consider))

                # NOTE: First prefer what it is already carrying
                union!(aids_to_add_with, intersect(v_aids, aerial_ids_to_consider))

                # Then remove from aerial_ids_to_consider
                setdiff!(aerial_ids_to_consider, v_aids)
                
                # Now choose best remaining
                if ~(isempty(aerial_ids_to_consider))
                    sorted_wt_edges = [(aid, edge_copies_id_to_wt[aid]) for aid in aerial_ids_to_consider]
                    sort!(sorted_wt_edges, by=x->x[2])
                    
                    n_aids = min(length(sorted_wt_edges), env.car_capacity-length(aids_to_add_with))
                    for (aid, wt) in sorted_wt_edges[1:n_aids]
                        push!(aids_to_add_with, aid)
                    end
                end

                if ~isempty(aids_to_add_with)
                    is_aerial = true
                end
            end
        end

        # Check if id_to_add_with is already in ground_mapf_graph
        new_vert = GroundMAPFState(onbr, is_aerial)
        gval = d + generate_ground_mapf_edge_weight(env, v, new_vert, aids_to_add_with)
        if ~(haskey(env.unique_gmapf_states, new_vert))
            idx_ctr += 1
            Graphs.add_vertex!(env.ground_mapf_graph, new_vert)
            push!(nbrs, idx_ctr)
            env.unique_gmapf_states[new_vert] = GMAPFStateInfo(idx_ctr, gval, aids_to_add_with)  
        else
            # Only add updated if gval improves
            # NOTE: Should only happen for aerial-aerial or drone-drone
            if gval < env.unique_gmapf_states[new_vert].gval
                push!(nbrs, env.unique_gmapf_states[new_vert].idx)
                env.unique_gmapf_states[new_vert].gval = gval
                env.unique_gmapf_states[new_vert].aerial_ids = aids_to_add_with
            end
        end
    end # onbr in out_neighbors(v.road_vtx_id, env.road_graph)

    return true
end


# Incorporate time and true distance info here
function update_ground_paths_with_ground_mapf_result!(env::CoordinatedMAPFEnv, solution::Vector{PlanResult{GroundMAPFState,GroundMAPFAction,Float64}})
    
    env.ground_paths = Vector{AgentPathInfo}(undef, length(env.ground_task_list))

    for (ground_id, ground_soln) in enumerate(solution)
    
        ground_soln_states = ground_soln.states

        path_states = AgentPathState[AgentPathState(road_vtx_id=ground_soln_states[1][1].road_vtx_id, timeval=0.0)]
        
        cumul_time = 0.0
        cumul_dist = 0.0
        for (state, state_next) in zip(ground_soln_states[1:end-1], ground_soln_states[2:end])
            dist = env.road_graph_wts[state[1].road_vtx_id, state_next[1].road_vtx_id]
            cumul_dist += dist
            cumul_time += dist/AvgSpeed
            push!(path_states, AgentPathState(road_vtx_id=state_next[1].road_vtx_id, timeval=cumul_time))
        end
        
        env.ground_paths[ground_id] = AgentPathInfo(path_states=path_states, total_dist=cumul_dist, total_time=cumul_time)
    end
end

## For prioritized planning
