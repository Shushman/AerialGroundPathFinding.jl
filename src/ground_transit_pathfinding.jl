## Solve the subproblem of finding aerial paths over the ground transit network
function MultiAgentPathFinding.add_constraint!(cbase::GroundTransitConstraint, cadd::GroundTransitConstraint)
    union!(cbase.avoid_gtg_vertex_set, cadd.avoid_gtg_vertex_set)
end

function MultiAgentPathFinding.overlap_between_constraints(cbase::GroundTransitConstraint, cadd::GroundTransitConstraint)
    return ~(isempty(intersect(cbase.avoid_gtg_vertex_set, cadd.avoid_gtg_vertex_set)))
end

MultiAgentPathFinding.get_mapf_state_from_idx(env::CoordinatedMAPFEnv, idx::Int64) = env.aerial_mapf_graph.vertices[idx]

function get_mapf_transit_action(env::CoordinatedMAPFEnv, u::AerialMAPFState, v::AerialMAPFState)
    ugtg = env.ground_transit_graph.vertices[u.ground_transit_idx]
    vgtg = env.ground_transit_graph.vertices[v.ground_transit_idx]
    if ugtg.ground_agent_id == vgtg.ground_agent_id
        return GroundTransitAction(Stay)
    else
        return GroundTransitAction(Fly)
    end
end

function MultiAgentPathFinding.get_mapf_action(env::CoordinatedMAPFEnv, u::Int64, v::Int64)
    vtx_u = env.aerial_mapf_graph.vertices[u]
    vtx_v = env.aerial_mapf_graph.vertices[v]

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

                    si_gtg = env.ground_transit_graph.vertices[state_i.ground_transit_idx]
                    sj_gtg = env.ground_transit_graph.vertices[state_j.ground_transit_idx]

                    # Conflict - boarding vertex at same time
                    if si_gtg.ground_agent_id == sj_gtg.ground_agent_id && si_gtg.vertex_along_path == sj_gtg.vertex_along_path && si_gtg.ground_agent_id != 0
                        push!(pair_overlap_verts, si_gtg.idx)
                    end
                end
            end

            # If any overlap verts, there is a conflict
            if !isempty(pair_overlap_verts)
                return GroundTransitConflict(pair_overlap_verts, (i, i+j))
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

    constraint = Dict(conflict.aerial_agent_ids[1]=>GroundTransitConstraint(conflict.overlap_gtg_vertices), conflict.aerial_agent_ids[2]=> GroundTransitConstraint(conflict.overlap_gtg_vertices))
    println("High level conflict number $(env.num_global_conflicts)!")
    @show constraint
    # @infiltrate

    return [constraint]
end

# Look at current ground paths and generate the transit graph over which the drones will plan
# Assumes we separately calculated and stored shortest S-T graph paths by drone
function set_ground_transit_graph!(env::CoordinatedMAPFEnv, drone_sp_dists::Vector{Float64})

    gtg = env.ground_transit_graph
    weights_dict = env.ground_transit_weights  # Tracks the edge weights as added
    expanded_paths_dict = env.ground_transit_expanded_paths
    drone_sg_idxs = env.gtg_drone_start_goal_idxs    # Notes the start-goal idxs of drones relative to gtg
    ground_path_start_ends = env.gtg_ground_path_start_ends

    v_idx = 1
    
    # First add drone start and goal vertices
    for (ai, atask) in enumerate(env.aerial_task_list)
        push!(drone_sg_idxs, (v_idx, v_idx+1))
        
        # Compute road graph distance between drone starts and goals
        weights_dict[(v_idx, v_idx+1)] = drone_sp_dists[ai]

        Graphs.add_vertex!(gtg, GroundTransitState(idx=v_idx, road_vtx_id=atask.origin))
        v_idx += 1
        Graphs.add_vertex!(gtg, GroundTransitState(idx=v_idx, road_vtx_id=atask.dest))
        v_idx += 1
    end

    # Saved for future use; only the initial states

    # Loop over ground paths
    for (gi, gpath) in enumerate(env.ground_paths)

        # Now loop over agent's path
        # NOTE : Assuming coord_agent_id = 0 all the way here?
        ground_start = v_idx
        for (path_idx, state) in enumerate(gpath.path_states)

            new_gts = GroundTransitState(idx=v_idx, ground_agent_id=gi, vertex_along_path=path_idx, road_vtx_id=state.road_vtx_id, timeval=state.timeval)
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
            # We can generate this on the fly now 
            # if path_idx < length(gpath.path_states)
            #     weights_dict[(v_idx, v_idx+1)] = 0.0
            # end
            
            v_idx += 1
        end # enumerate(gpath.path_states)
        ground_end = v_idx-1
        push!(ground_path_start_ends, (ground_start, ground_end))
    end

    # Now update distances BETWEEN CAR ROUTES
    # TODO: Currently
    # for (vi, vtx_i) in enumerate(gtg.vertices)

    #     if vtx_i.ground_agent_id > 0

    #         for (vj, vtx_j) in enumerate(gtg.vertices)

    #             # If a different ground vertex
    #             if vtx_j.ground_agent_id > 0 && vtx_i.ground_agent_id != vtx_j.ground_agent_id

    #                 # Compute paths in both directions
    #                 if vtx_i.road_vtx_id == vtx_j.road_vtx_id
    #                     weights_dict[(vtx_i.idx, vtx_j.idx)] = 0.0
    #                     weights_dict[(vtx_j.idx, vtx_i.idx)] = 0.0
    #                 else
    #                     bidir_sp_to = compute_bidir_astar_euclidean(env.road_graph, vtx_i.road_vtx_id, vtx_j.road_vtx_id, env.road_graph_wts, env.location_list)
    #                     weights_dict[(vtx_i.idx, vtx_j.idx)] = bidir_sp_to.dist
    #                     expanded_paths_dict[(vtx_i.idx, vtx_j.idx)] = bidir_sp_to.path
   
    #                     bidir_sp_from = compute_bidir_astar_euclidean(env.road_graph, vtx_j.road_vtx_id, vtx_i.road_vtx_id, env.road_graph_wts, env.location_list)
    #                     weights_dict[(vtx_j.idx, vtx_i.idx)] = bidir_sp_from.dist
    #                     expanded_paths_dict[(vtx_j.idx, vtx_i.idx)] = bidir_sp_from.path
    #                 end
    #             end
    #         end

    #     end
    # end
end # function generate_ground_transit_graph


# TODO: Review
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

# TODO: Review
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

# # TODO: Change as for CBS
# function MultiAgentPathFinding.low_level_search!(solver::ECBSSolver, aerial_agent_id::Int64, s::AerialMAPFState, constraints::GroundTransitConstraint, solution::Vector{PR}) where {PR <: PlanResult}

#     ## TODO
#     # distance_heuristic(u) = 0 (can always take a car all the way there)
#     env = solver.env

#     edge_wt_fn(u, v) = amapfg_edge_wt_fn(env, u, v)
#     admissible_heuristic(u) = 0.0   # Can always take a car to the goal
#     focal_state_heuristic(u) = 0.0
#     focal_transition_heuristic(u, v) = focal_transition_heuristic_transit(env, solution, aerial_agent_id, u, v)

#     # Set start and goal
#     start_idx = env.gtg_drone_start_goal_idxs[aerial_agent_id][1]
#     env.next_gtg_goal_idx = env.gtg_drone_start_goal_idxs[aerial_agent_id][2]
#     @assert start_idx == s.idx "Mismatch in start index!" # TODO: Remove later

#     states = Tuple{GroundTransitState, Float64}[]
#     actions = Tuple{GroundTransitAction, Float64}[]

#     vis = GroundTransitGoalVisitor(env, constraints[1].avoid_gtg_vertex_set)

#     a_star_eps_states = a_star_implicit_epsilon_path!(env.ground_transit_graph,
#                                                       edge_wt_fn,
#                                                       start_idx,
#                                                       vis,
#                                                       solver.weight,
#                                                       admissible_heuristic,
#                                                       focal_state_heuristic,
#                                                       focal_transition_heuristic
#                                                       )
    
#     # Get solution from a_star_eps_states
#     # NOTE: It should always have a solution because drone can just fly
#     a_star_states = a_star_eps_states.a_star_states
#     @assert haskey(a_star_states.parent_indices, env.next_gtg_goal_idx)

#     # Get path cost
#     sp_cost = a_star_states.dists[env.next_gtg_goal_idx]

#     # Get path
#     sp_idxs = shortest_path_indices(a_star_states.parent_indices, env.ground_transit_graph, start_idx, env.next_gtg_goal_idx)

#     push!(states, (s, 0.0))

#     # Traverse sp_idxs and set up states and actions
#     for i = 2:length(sp_idxs)

#         u, v =  (sp_idxs[i-1], sp_idxs[i])
#         push!(actions, (get_mapf_action(env, u, v), env.ground_transit_weights[(u, v)]))

#         push!(states, (env.ground_transit_graph.vertices[v], a_star_states.dists[v]))
#     end

#     return PlanResult{GroundTransitState,GroundTransitAction,Float64}(states, actions, sp_cost, a_star_eps_states.best_fvalue)
# end # function low_level_search



function MultiAgentPathFinding.low_level_search!(solver::CBSSolver, aerial_agent_id::Int64, s::AerialMAPFState, constraint::GroundTransitConstraint)

    env = solver.env

    # IMP: Resetting aerial mapf incremental graph to initial snapshot
    # Insert initial state in aerial_mapf_graph here
    env.aerial_mapf_graph = SimpleVListGraph{AerialMAPFState}()
    Graphs.add_vertex!(env.aerial_mapf_graph, s)

    env.gtg_idx_gval =  Dict{Int64,Float64}()


    edge_wt_fn(u, v) = amapfg_edge_wt_fn(env, u, v)
    
    admissible_heuristic(u) = direct_flight_time_heuristic(env, u)
    # admissible_heuristic(u) = 0.0

    # Set start and goal
    start_idx = 1   # Always one since graph reinitialized
    env.next_gtg_goal_idx = env.gtg_drone_start_goal_idxs[aerial_agent_id][2]
    env.next_amapfg_goal_idx = 0

    vis = GroundTransitGoalVisitor(env, constraint.avoid_gtg_vertex_set)


    a_star_states = a_star_implicit_shortest_path!(env.aerial_mapf_graph,
                                                    edge_wt_fn,
                                                    start_idx,
                                                    vis,
                                                    admissible_heuristic
                                                    )
    
    # Get solution from a_star_eps_states
    # NOTE: It should always have a solution because drone can just fly
    @assert haskey(a_star_states.parent_indices, env.next_amapfg_goal_idx)


    # TODO: HAVE TO CREATE STATES AS WE ARE DELETING AERIAL MAPF GRAPH
    states = Tuple{AerialMAPFState,Float64}[]
    actions = Tuple{GroundTransitAction, Float64}[]

    # Get path cost
    sp_cost = a_star_states.dists[env.next_amapfg_goal_idx]

    # Get path
    sp_idxs = shortest_path_indices(a_star_states.parent_indices, env.aerial_mapf_graph, start_idx, env.next_amapfg_goal_idx)

    push!(states, (s, 0.0))

    # Traverse sp_idxs and set up states and actions
    for i = 2:length(sp_idxs)

        u, v =  (sp_idxs[i-1], sp_idxs[i])

        # NOTE: Create new state because we are going to delete graph later
        new_state = deepcopy(env.aerial_mapf_graph.vertices[v])
        push!(states, (new_state, a_star_states.dists[v]))

        # All derived info but can be useful
        push!(actions, (get_mapf_action(env, u, v), a_star_states.dists[v] - a_star_states.dists[u]))
    end

    # if ~isempty(constraint.avoid_gtg_vertex_set)
    #     @infiltrate
    # end

    return PlanResult{AerialMAPFState,GroundTransitAction,Float64}(states, actions, sp_cost, sp_cost)
end # function low_level_search

# NOTE: Have to use dist + time here!
function amapfg_edge_wt_fn(env::CoordinatedMAPFEnv, u::AerialMAPFState, v::AerialMAPFState)

    ddiff = v.distval - u.distval
    tdiff = v.timeval - u.timeval

    return env.alpha_weight_distance*ddiff + (1.0 - env.alpha_weight_distance)*tdiff
end

function direct_flight_time_heuristic(env::CoordinatedMAPFEnv, u::AerialMAPFState)

    if u.ground_transit_idx == env.next_gtg_goal_idx
        return 0.0
    else
        ddiff = env.ground_transit_weights[(u.ground_transit_idx, env.next_gtg_goal_idx)]
        return (1.0 - env.alpha_weight_distance)*ddiff/AvgSpeed
    end
end



struct GroundTransitGoalVisitor <: Graphs.AbstractDijkstraVisitor
    env::CoordinatedMAPFEnv
    avoid_gtg_vertex_set::Set{Int64}
end


function Graphs.include_vertex!(vis::GroundTransitGoalVisitor, u::AerialMAPFState, v::AerialMAPFState, d::Float64, nbrs::Vector{Int64})

    env = vis.env
    avoid_gtg_vertex_set = vis.avoid_gtg_vertex_set
    gtg_weights = env.ground_transit_weights

    
    # When you expand goal, return
    if v.ground_transit_idx == env.next_gtg_goal_idx
        env.next_amapfg_goal_idx = v.idx
        return false
    end

    # Now lookup where drone is on the GTG
    ugtg = env.ground_transit_graph.vertices[u.ground_transit_idx]
    vgtg = env.ground_transit_graph.vertices[v.ground_transit_idx]
    idx_ctr = Graphs.num_vertices(env.aerial_mapf_graph)

    # Rethink everything here
    # Going with fully implicit search; not worth the effort to track
    # If drone start point, then add goal (NO ADD VERTEX) and car connections being mindful of time (ADD VERTEX)
    # If on first car point, just add the next and add timeval based on car's time difference at next point (ADD VERTEX)
    # If transfer-ready, then add goal (NO ADD VERTEX) and car connections tracking time (ADD VERTEX)
    
    # Case 1: Drone start
    if vgtg.ground_agent_id == 0
        
        ddiff = gtg_weights[(v.ground_transit_idx, env.next_gtg_goal_idx)]
        tdiff = ddiff / AvgSpeed

        idx_ctr += 1
        new_goal_vert = AerialMAPFState(idx=idx_ctr, ground_transit_idx=env.next_gtg_goal_idx, timeval=v.timeval+tdiff, distval=v.distval+ddiff)

        # Add to graph and add neighbor
        Graphs.add_vertex!(env.aerial_mapf_graph, new_goal_vert)
        push!(nbrs, idx_ctr)

        env.gtg_idx_gval[env.next_gtg_goal_idx] = env.alpha_weight_distance*new_goal_vert.distval + (1.0 - env.alpha_weight_distance)*new_goal_vert.timeval

        # Now add other car connections while accounting for time
        # First get non-dominated gtg indices and then add aerial mapf state versions of them
        # For timeval use the max of gtg time and drone timeval
        for (gstart, gend) in env.gtg_ground_path_start_ends
            gts_non_dom = get_non_dominated_transit_points(env, gstart, gend, avoid_gtg_vertex_set, v, false)

            # Add AerialMAPFState versions of them
            for gts_tup in gts_non_dom

                idx_ctr += 1
                new_vert = AerialMAPFState(idx=idx_ctr, ground_transit_idx=gts_tup.idx, timeval=gts_tup.timeval, distval=gts_tup.distval)

                # if gts_tup.idx in avoid_gtg_vertex_set
                #     @infiltrate
                # end

                Graphs.add_vertex!(env.aerial_mapf_graph, new_vert)
                push!(nbrs, idx_ctr)

                env.gtg_idx_gval[gts_tup.idx] = env.alpha_weight_distance*new_vert.distval + (1.0 - env.alpha_weight_distance)*new_vert.timeval

            end
        end

    else
        # Either first transfer to new ground point OR along the way
        if vgtg.ground_agent_id != ugtg.ground_agent_id

            new_considered_cars = union(v.considered_cars, Set{Int64}([vgtg.ground_agent_id]))
            # @infiltrate

            next_gtg = env.ground_transit_graph.vertices[vgtg.idx + 1]
            # @assert next_gtg.ground_agent_id == vgtg.ground_agent_id && next_gtg.vertex_along_path == vgtg.vertex_along_path+1

            if ~(next_gtg.idx in avoid_gtg_vertex_set)

                idx_ctr += 1
                # IMP NOTE: Timeval may be greater than vgtg.timeval (probably will be)
                # So the new timeval should be v.timeval + car_time_diff (next_gtg.timeval - vgtg.timeval)
                tval = v.timeval + (next_gtg.timeval - vgtg.timeval)
                new_vert = AerialMAPFState(idx=idx_ctr, ground_transit_idx=next_gtg.idx, timeval=tval, distval=v.distval, considered_cars=new_considered_cars)

                Graphs.add_vertex!(env.aerial_mapf_graph, new_vert)
                push!(nbrs, idx_ctr)
            end

        else
            # Ongoing car route vertex. First try to add goal as usual
            ddiff = gtg_weights[(v.ground_transit_idx, env.next_gtg_goal_idx)]
            tdiff = ddiff / AvgSpeed
            goal_gval = env.alpha_weight_distance*(v.distval+ddiff) + (1.0 - env.alpha_weight_distance)*(v.timeval + tdiff)

            if goal_gval < env.gtg_idx_gval[env.next_gtg_goal_idx]
        
                idx_ctr += 1
                new_goal_vert = AerialMAPFState(idx=idx_ctr, ground_transit_idx=env.next_gtg_goal_idx, timeval=v.timeval+tdiff, distval=v.distval+ddiff, considered_cars=v.considered_cars)

                # Add to graph and add neighbor
                Graphs.add_vertex!(env.aerial_mapf_graph, new_goal_vert)
                push!(nbrs, idx_ctr)

                env.gtg_idx_gval[env.next_gtg_goal_idx] = goal_gval
            end

            # Then add next along car route IF not last
            if vgtg.idx < env.gtg_ground_path_start_ends[vgtg.ground_agent_id][2]
                next_gtg = env.ground_transit_graph.vertices[vgtg.idx + 1]
                # @assert next_gtg.ground_agent_id == vgtg.ground_agent_id && next_gtg.vertex_along_path == vgtg.vertex_along_path+1

                if ~(next_gtg.idx in avoid_gtg_vertex_set)

                    idx_ctr += 1
                    # IMP NOTE: Timeval may be greater than vgtg.timeval (probably will be)
                    # So the new timeval should be v.timeval + car_time_diff (next_gtg.timeval - vgtg.timeval)
                    tval = v.timeval + (next_gtg.timeval - vgtg.timeval)
                    new_vert = AerialMAPFState(idx=idx_ctr, ground_transit_idx=next_gtg.idx, timeval=tval, distval=v.distval, considered_cars=v.considered_cars)


                    Graphs.add_vertex!(env.aerial_mapf_graph, new_vert)
                    push!(nbrs, idx_ctr)
                end
            end

            for (gid, (gstart, gend)) in enumerate(env.gtg_ground_path_start_ends)
                
                if gid != vgtg.ground_agent_id && ~(gid in v.considered_cars)
                    gts_non_dom = get_non_dominated_transit_points(env, gstart, gend, avoid_gtg_vertex_set, v, true)
        
                    if ~isempty(gts_non_dom)
                        # Add AerialMAPFState versions of them
                        for gts_tup in gts_non_dom

                            temp_gval = env.alpha_weight_distance*gts_tup.distval + (1.0 - env.alpha_weight_distance)*gts_tup.timeval

                            if ~haskey(env.gtg_idx_gval, gts_tup.idx) || temp_gval < env.gtg_idx_gval[gts_tup.idx]

                                idx_ctr += 1
                                new_vert = AerialMAPFState(idx=idx_ctr, ground_transit_idx=gts_tup.idx, timeval=gts_tup.timeval, distval=gts_tup.distval, considered_cars=v.considered_cars)

                                # if gts_tup.idx in avoid_gtg_vertex_set
                                #     @infiltrate
                                # end

                                Graphs.add_vertex!(env.aerial_mapf_graph, new_vert)
                                push!(nbrs, idx_ctr)
                                
                                env.gtg_idx_gval[gts_tup.idx] = temp_gval
                            end
                        end # gts_vert in gts_subseq
                    end
                end
            end # (gstart, gend) in gtg_ground_path_start_ends
        end # vgtg.ground_agent_id != ugtg.ground_agent_id
    end # vgtg.ground == or != 0

    return true
end


# TODO: Reframe to include time!
# Remember, drone needs EXPANDED paths
# Also need to update wait times for cars
# IMP: How to handle cars that were used by multiple drones???
# Will also end up updating drone routes
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