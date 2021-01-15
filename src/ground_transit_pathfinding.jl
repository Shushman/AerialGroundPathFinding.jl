## Solve the subproblem of finding aerial paths over the ground transit network
function MultiAgentPathFinding.add_constraint!(cbase::GroundTransitConstraint, cadd::GroundTransitConstraint)
    union!(cbase.avoid_vertex_set, cadd.avoid_vertex_set)
end

function MultiAgentPathFinding.overlap_between_constraints(cbase::GroundTransitConstraint, cadd::GroundTransitConstraint)
    return ~(isempty(intersect(cbase.avoid_vertex_set, cadd.avoid_vertex_set)))
end

MultiAgentPathFinding.get_mapf_state_from_idx(env::CoordinatedMAPFEnv, idx::Int64) = env.ground_transit_graph.vertices[idx]

function get_mapf_transit_action(env::CoordinatedMAPFEnv, u::GroundTransitState, v::GroundTransitState)
    if u.ground_agent_id == v.ground_agent_id
        return GroundTransitAction(Stay)
    else
        return GroundTransitAction(Fly)
    end
end


function MultiAgentPathFinding.get_mapf_action(env::CoordinatedMAPFEnv, u::Int64, v::Int64)
    vtx_u = env.ground_transit_graph.vertices[u]
    vtx_v = env.ground_transit_graph.vertices[v]

    return get_mapf_transit_action(env, vtx_u, vtx_v)
end

function MultiAgentPathFinding.set_low_level_context!(::CoordinatedMAPFEnv, ::Int64, ::GroundTransitConstraint)
end

MultiAgentPathFinding.get_empty_constraint(::Type{GroundTransitConstraint}) = GroundTransitConstraint()

function MultiAgentPathFinding.get_first_conflict(env::CoordinatedMAPFEnv, solution::Vector{PR}) where {PR <: PlanResult}

    for (i, sol_i) in enumerate(solution[1:end-1])
        for (j, sol_j) in enumerate(solution[i+1:end])

            pair_overlap_verts = Set{Int64}()

            # Now we have the two solutions
            for (si, (state_i, _)) in enumerate(sol_i.states[2:end])
                for (sj, (state_j, _)) in enumerate(sol_j.states[2:end])

                    # Conflict - boarding vertex at same time
                    if state_i.ground_agent_id == state_j.ground_agent_id && state_i.vertex_along_path == state_j.vertex_along_path && state_i.ground_agent_id != 0
                        push!(pair_overlap_verts, state_i.road_vtx_id)
                    end
                end
            end

            # If any overlap verts, there is a conflict
            if !isempty(pair_overlap_verts)
                return GroundTransitConflict(pair_overlap_verts, (i, j))
            end
        end
    end

    return nothing
end

function MultiAgentPathFinding.create_constraints_from_conflict(env::CoordinatedMAPFEnv, conflict::GroundTransitConflict)

    env.num_global_conflicts += 1
    if env.num_global_conflicts > env.threshold_global_conflicts
        throw(DomainError("Too many conflicts!"))
    end
    @infiltrate

    constraint = Dict(conflict.aerial_agent_ids[1]=>GroundTransitConstraint(conflict.overlap_vertices), conflict.aerial_agent_ids[2]=> GroundTransitConstraint(conflict.overlap_vertices))

    return [constraint]
end

# Look at current ground paths and generate the transit graph over which the drones will plan
# Assumes we separately calculated and stored shortest S-T graph paths by drone
# TODO: Ensure correct cross-ref between road graph and transit graph
# TODO: What if a drone_sp_cost is Inf (no road network path?)
function set_ground_transit_graph!(env::CoordinatedMAPFEnv, drone_sp_costs::Vector{Float64})

    gtg = SimpleVListGraph{GroundTransitState}()
    weights_dict = Dict{Tuple{Int64,Int64}, Float64}()  # Tracks the edge weights as added
    expanded_paths_dict = Dict{Tuple{Int64,Int64}, Vector{Int64}}()
    drone_sg_idxs = Tuple{Int64,Int64}[]    # Notes the start-goal idxs of drones relative to gtg
    ground_path_start_ends = Tuple{Int64,Int64}[]

    v_idx = 1
    
    # First add drone start and goal vertices
    for (ai, atask) in enumerate(env.aerial_task_list)
        push!(drone_sg_idxs, (v_idx, v_idx+1))
        
        # Compute road graph distance between drone starts and goals
        weights_dict[(v_idx, v_idx+1)] = drone_sp_costs[ai]

        Graphs.add_vertex!(gtg, GroundTransitState(idx=v_idx, road_vtx_id=atask.origin))
        v_idx += 1
        Graphs.add_vertex!(gtg, GroundTransitState(idx=v_idx, road_vtx_id=atask.dest))
        v_idx += 1
    end

    # Loop over ground paths
    for (gi, gpath) in enumerate(env.ground_paths)

        # Now loop over agent's path
        # NOTE : Assuming coord_agent_id = 0 all the way here?
        ground_start = v_idx
        for (path_idx, state) in enumerate(gpath.path_states)

            new_gts = GroundTransitState(idx=v_idx, ground_agent_id=gi, vertex_along_path=path_idx, road_vtx_id=state.road_vtx_id)
            Graphs.add_vertex!(gtg, new_gts)

            # Compute to and from paths relative to drone starts and goals over ROAD GRAPH
            # And update the gtg weights correctly
            for (ai, (ais, aig)) in enumerate(drone_sg_idxs)

                atask = env.aerial_task_list[ai]
                
                if atask.origin == state.road_vtx_id
                    weights_dict[(ais, v_idx)] = 0.0
                else
                    bidir_sp_to = compute_bidir_astar_euclidean(env.road_graph, atask.origin, state.road_vtx_id, env.road_graph_wts, env.location_list)
                    weights_dict[(ais, v_idx)] = bidir_sp_to.dist
                    expanded_paths_dict[(ais, v_idx)] = bidir_sp_to.path
                end

                # NOTE: But this does not make weights_dict an admissible heuristic to GOAL!
                # Because this distance assumes it takes no cars along the way
                if state.road_vtx_id == atask.dest
                    weights_dict[(v_idx, aig)] = 0.0
                else
                    bidir_sp_from = compute_bidir_astar_euclidean(env.road_graph, state.road_vtx_id, atask.dest, env.road_graph_wts, env.location_list)
                    weights_dict[(v_idx, aig)] = bidir_sp_from.dist
                    expanded_paths_dict[(v_idx, aig)] = bidir_sp_from.path
                end
            end
            
            # Add zero cost weights on route graph itself
            if path_idx < length(gpath.path_states)
                weights_dict[(v_idx, v_idx+1)] = 0.0
            end
            
            v_idx += 1
        end # enumerate(gpath.path_states)
        ground_end = v_idx-1
        push!(ground_path_start_ends, (ground_start, ground_end))
    end

    # Now update weights BETWEEN CAR ROUTES
    for (vi, vtx_i) in enumerate(gtg.vertices)

        if vtx_i.ground_agent_id > 0

            for (vj, vtx_j) in enumerate(gtg.vertices)

                # If a different ground vertex
                if vtx_j.ground_agent_id > 0 && vtx_i.ground_agent_id != vtx_j.ground_agent_id

                    # Compute paths in both directions
                    if vtx_i.road_vtx_id == vtx_j.road_vtx_id
                        weights_dict[(vtx_i.idx, vtx_j.idx)] = 0.0
                        weights_dict[(vtx_j.idx, vtx_i.idx)] = 0.0
                    else
                        bidir_sp_to = compute_bidir_astar_euclidean(env.road_graph, vtx_i.road_vtx_id, vtx_j.road_vtx_id, env.road_graph_wts, env.location_list)
                        weights_dict[(vtx_i.idx, vtx_j.idx)] = bidir_sp_to.dist
                        expanded_paths_dict[(vtx_i.idx, vtx_j.idx)] = bidir_sp_to.path
   
                        bidir_sp_from = compute_bidir_astar_euclidean(env.road_graph, vtx_j.road_vtx_id, vtx_i.road_vtx_id, env.road_graph_wts, env.location_list)
                        weights_dict[(vtx_j.idx, vtx_i.idx)] = bidir_sp_from.dist
                        expanded_paths_dict[(vtx_j.idx, vtx_i.idx)] = bidir_sp_from.path
                    end
                end
            end

        end
    end

    # Set values
    env.ground_transit_graph = gtg
    env.ground_transit_weights = weights_dict
    env.ground_transit_expanded_paths = expanded_paths_dict
    env.gtg_drone_start_goal_idxs = drone_sg_idxs
    env.gtg_ground_path_start_ends = ground_path_start_ends
end # function generate_ground_transit_graph



# Focal transition heuristic just counts boarding/capacity constraints
function focal_transition_heuristic_transit(env::CoordinatedMAPFEnv, solution::Vector{PR}, aerial_agent_id::Int64, u::GroundTransitState, v::GroundTransitState) where {PR <: PlanResult}

    # Trivial check that it doesn't fly to goal
    if v.ground_agent_id == 0
        return 0
    end

    num_conflicts = 0

    # Target vertex is car vertex
    for (i, aerial_soln) in enumerate(solution)
        if i != aerial_agent_id && (~isempty(aerial_soln))

            for (vtx_state, _) in aerial_soln.states[2:end-1]

                # Add if any other drone uses at any point
                if vtx_state.ground_agent_id == v.ground_agent_id && vtx_state.vertex_along_path == v.vertex_along_path
                    num_conflicts += 1
                    continue
                end
            end
        end # if i && isempty
    end # enumerate(solution)

    return num_conflicts
end


function MultiAgentPathFinding.focal_heuristic(env::CoordinatedMAPFEnv, solution::Vector{PR}) where {PR <: PlanResult}

    num_potential_conflicts = 0

    for (i, sol_i) in enumerate(solution[1:end-1])
        for (j, sol_j) in enumerate(solution[i+1:end])

            for (si, _) in sol_i.states[2:end-1]
                for (sj, _) in sol_j.states[2:end-1]

                    if si.ground_agent_id == sj.ground_agent_id && si.vertex_along_path == sj.vertex_along_path
                        num_potential_conflicts += 1
                    end
                end
            end
        end
    end

    return num_potential_conflicts
end


function MultiAgentPathFinding.low_level_search!(solver::ECBSSolver, aerial_agent_id::Int64, s::GroundTransitState, constraints::GroundTransitConstraint, solution::Vector{PR}) where {PR <: PlanResult}

    ## TODO
    # distance_heuristic(u) = 0 (can always take a car all the way there)
    env = solver.env

    edge_wt_fn(u, v) = env.ground_transit_weights[(u.idx, v.idx)]
    admissible_heuristic(u) = 0.0   # Can always take a car to the goal
    focal_state_heuristic(u) = 0.0
    focal_transition_heuristic(u, v) = focal_transition_heuristic_transit(env, solution, aerial_agent_id, u, v)

    # Set start and goal
    start_idx = env.gtg_drone_start_goal_idxs[aerial_agent_id][1]
    env.next_gtg_goal_idx = env.gtg_drone_start_goal_idxs[aerial_agent_id][2]
    @assert start_idx == s.idx "Mismatch in start index!" # TODO: Remove later

    states = Tuple{GroundTransitState, Float64}[]
    actions = Tuple{GroundTransitAction, Float64}[]

    vis = GroundTransitGoalVisitor(env, constraints.avoid_vertex_set)

    a_star_eps_states = a_star_implicit_epsilon_path!(env.ground_transit_graph,
                                                      edge_wt_fn,
                                                      start_idx,
                                                      vis,
                                                      solver.weight,
                                                      admissible_heuristic,
                                                      focal_state_heuristic,
                                                      focal_transition_heuristic
                                                      )
    
    # Get solution from a_star_eps_states
    # NOTE: It should always have a solution because drone can just fly
    a_star_states = a_star_eps_states.a_star_states
    @assert haskey(a_star_states.parent_indices, env.next_gtg_goal_idx)

    # Get path cost
    sp_cost = a_star_states.dists[env.next_gtg_goal_idx]

    # Get path
    sp_idxs = shortest_path_indices(a_star_states.parent_indices, env.ground_transit_graph, start_idx, env.next_gtg_goal_idx)

    push!(states, (s, 0.0))

    # Traverse sp_idxs and set up states and actions
    for i = 2:length(sp_idxs)

        u, v =  (sp_idxs[i-1], sp_idxs[i])
        push!(actions, (get_mapf_action(env, u, v), env.ground_transit_weights[(u, v)]))

        push!(states, (env.ground_transit_graph.vertices[v], a_star_states.dists[v]))
    end

    return PlanResult{GroundTransitState,GroundTransitAction,Float64}(states, actions, sp_cost, a_star_eps_states.best_fvalue)
end # function low_level_search

function MultiAgentPathFinding.low_level_search!(solver::CBSSolver, aerial_agent_id::Int64, s::GroundTransitState, constraints::GroundTransitConstraint) where {PR <: PlanResult}

    ## TODO
    # distance_heuristic(u) = 0 (can always take a car all the way there)
    env = solver.env

    edge_wt_fn(u, v) = env.ground_transit_weights[(u.idx, v.idx)]
    admissible_heuristic(u) = 0.0   # Can always take a car to the goal

    # Set start and goal
    start_idx = env.gtg_drone_start_goal_idxs[aerial_agent_id][1]
    env.next_gtg_goal_idx = env.gtg_drone_start_goal_idxs[aerial_agent_id][2]
    @assert start_idx == s.idx "Mismatch in start index!" # TODO: Remove later

    states = Tuple{GroundTransitState, Float64}[]
    actions = Tuple{GroundTransitAction, Float64}[]

    vis = GroundTransitGoalVisitor(env, constraints.avoid_vertex_set)

    a_star_states = a_star_implicit_shortest_path!(env.ground_transit_graph,
                                                      edge_wt_fn,
                                                      start_idx,
                                                      vis,
                                                      admissible_heuristic
                                                      )
    
    # Get solution from a_star_eps_states
    # NOTE: It should always have a solution because drone can just fly
    @assert haskey(a_star_states.parent_indices, env.next_gtg_goal_idx)

    # Get path cost
    sp_cost = a_star_states.dists[env.next_gtg_goal_idx]

    # Get path
    sp_idxs = shortest_path_indices(a_star_states.parent_indices, env.ground_transit_graph, start_idx, env.next_gtg_goal_idx)

    push!(states, (s, 0.0))

    # Traverse sp_idxs and set up states and actions
    for i = 2:length(sp_idxs)

        u, v =  (sp_idxs[i-1], sp_idxs[i])
        push!(actions, (get_mapf_action(env, u, v), env.ground_transit_weights[(u, v)]))

        push!(states, (env.ground_transit_graph.vertices[v], a_star_states.dists[v]))
    end

    return PlanResult{GroundTransitState,GroundTransitAction,Float64}(states, actions, sp_cost, sp_cost)
end # function low_level_search



struct GroundTransitGoalVisitor <: Graphs.AbstractDijkstraVisitor
    env::CoordinatedMAPFEnv
    avoid_vertex_set::Set{Int64}
end


function Graphs.include_vertex!(vis::GroundTransitGoalVisitor, u::GroundTransitState, v::GroundTransitState, d::Float64, nbrs::Vector{Int64})

    env = vis.env
    avoid_vertex_set = vis.avoid_vertex_set
    gtg_weights = env.ground_transit_weights
    
    # When you expand goal, return
    if v.idx == env.next_gtg_goal_idx
        return false
    end

    if v.ground_agent_id == 0
        
        # Drone's start point on transit graph
        # First add the drone goal
        push!(nbrs, env.next_gtg_goal_idx)

        # Iterate over ground path and keep adding UNTIL remaining are all higher cost
        # TODO: Verify this logic
        for (gstart, gend) in env.gtg_ground_path_start_ends

            for i = gstart:gend-1
                
                any_nearer = false
                dist_to_i = gtg_weights[(v.idx, i)]

                for j = i+1:gend
                    if gtg_weights[(v.idx, j)] < dist_to_i
                        any_nearer = true
                        break
                    end
                end

                if any_nearer
                    push!(nbrs, i)
                else
                    break
                end # any_nearer
            end # i = gstart:gend-1
        end # (,) in gtg_ground_path_start_ends

    else
        # On a ground route point
        if u.ground_agent_id != v.ground_agent_id
            # Transfer, only add next route vertex and continue
            next_vtx = env.ground_transit_graph.vertices[v.idx+1]

            # TODO: Remove this assert later
            @assert next_vtx.ground_agent_id == v.ground_agent_id && next_vtx.vertex_along_path == v.vertex_along_path+1

            push!(nbrs, v.idx+1)
        else

            # First add goal as usual
            push!(nbrs, env.next_gtg_goal_idx)

            # Now add other route points BESIDES this one
            for (gidx, (gstart, gend)) in enumerate(env.gtg_ground_path_start_ends)

                # Filter out the same route (edge weights don't exist)
                if gidx != v.ground_agent_id

                    for i = gstart:gend-1

                        any_nearer = false
                        dist_to_i = gtg_weights[(v.idx, i)]

                        for j = i+1:gend
                            if gtg_weights[(v.idx, j)] < dist_to_i
                                any_nearer = true
                                break
                            end
                        end

                        if any_nearer
                            push!(nbrs, i)
                        else
                            break
                        end # if any_nearer
                    end
                
                end # gidx ~= v.ground_agent_id
            end # enumerate(env.gtg_ground_path_start_ends)
        end # u.ground_agent_id != v.ground_agent_id

    end # v.ground_agent_id == 0

    return true
end


# TODO: Update ground and aerial paths with CBS solution
# Remember, drone needs EXPANDED paths
function update_aerial_ground_paths_cbs_solution!(env::CoordinatedMAPFEnv, solution::Vector{PR}, drone_flight_paths::Vector{Vector{Int64}}) where {PR <: PlanResult}

    aerial_paths = Vector{AgentPathInfo}(undef, length(solution))

    # Iterate over solution
    for (aerial_agent_id, aerial_soln) in enumerate(solution)

        path_states = AgentPathState[]

        # Direct flight path
        if length(aerial_soln.states) == 2
            flight_path = drone_flight_paths[aerial_agent_id]
            for idx in flight_path
                push!(path_states, AgentPathState(road_vtx_id=idx))
            end
        else

            for i = 1:length(aerial_soln.states) - 1

                # First add i
                si, sip = (aerial_soln.states[i][1], aerial_soln.states[i+1][1])
                push!(path_states, AgentPathState(road_vtx_id=si.road_vtx_id, coord_agent_id=si.ground_agent_id))

                # First update car route if si is a car vertex
                if si.ground_agent_id != 0
                    
                    @assert env.ground_paths[si.ground_agent_id].path_states[si.vertex_along_path].road_vtx_id == si.road_vtx_id "ROAD VTX ID mismatch"  # TODO: Remove later
                    
                    env.ground_paths[si.ground_agent_id].path_states[si.vertex_along_path].coord_agent_id = aerial_agent_id
                end

                # Now check if i -> i+1 is an expanded flight path
                if haskey(env.ground_transit_expanded_paths, (si.road_vtx_id, sip.road_vtx_id))

                    @assert si.ground_agent_id != sip.ground_agent_id   # TODO : Remove assert later
                    exp_path = env.ground_transit_expanded_paths[(si.road_vtx_id, sip.road_vtx_id)]

                    # Add intermediate vertices; necessarily flight
                    for idx in exp_path[2:end-1]
                        push!(path_states, AgentPathState(road_vtx_id=idx))
                    end

                end # haskey(expanded_paths)
            end # i = 1: length(states) - 1
            push!(path_states, AgentPathState(road_vtx_id=aerial_soln.states[end][1].road_vtx_id))
        end # length(states) == or != 2

        aerial_paths[aerial_agent_id] = AgentPathInfo(path_states=path_states, cost_to_agent=aerial_soln.cost)
    end # enumerate(solution)
    env.aerial_paths = aerial_paths
end


function compute_ground_paths!(env::CoordinatedMAPFEnv)

    ground_paths = Vector{AgentPathInfo}(undef, length(env.ground_task_list))

    for (ground_id, task) in enumerate(env.ground_task_list)

        bidir_sp = compute_bidir_astar_euclidean(env.road_graph, task.origin, task.dest, env.road_graph_wts, env.location_list)

        path_states = AgentPathState[]
        for idx in bidir_sp.path
            push!(path_states, AgentPathState(road_vtx_id=idx))
        end

        ground_paths[ground_id] = AgentPathInfo(path_states=path_states, cost_to_agent=bidir_sp.dist)
    end

    env.ground_paths = ground_paths
end