## Solve the subproblem of finding aerial paths over the ground transit network
function MultiAgentPathFinding.add_constraint!(cbase::GroundTransitConstraint, cadd::GroundTransitConstraint)
    union!(cbase.avoid_gid_vertex_set, cadd.avoid_gid_vertex_set)
end

function MultiAgentPathFinding.overlap_between_constraints(cbase::GroundTransitConstraint, cadd::GroundTransitConstraint)
    return ~(isempty(intersect(cbase.avoid_gid_vertex_set, cadd.avoid_gid_vertex_set)))
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

function MultiAgentPathFinding.get_first_conflict(env::CoordinatedMAPFEnv, solution::Vector{PlanResult{AerialMAPFState,GroundTransitAction,Float64}})

    # Maps GTG indices to which drones are boarding or on vehicle there
    gtg_idx_to_aerial_ids = Dict{Int64,Vector{Int64}}()

    for (i, sol_i) in enumerate(solution)
        for ((si, _), (sip,_)) in zip(sol_i.states[1:end-1], sol_i.states[2:end])
            si_gtg = env.ground_transit_graph.vertices[si.ground_transit_idx]
            sip_gtg = env.ground_transit_graph.vertices[sip.ground_transit_idx]
            if si_gtg.ground_agent_id !=0 && si_gtg.ground_agent_id == sip_gtg.ground_agent_id
                if ~haskey(gtg_idx_to_aerial_ids, si_gtg.idx)
                    gtg_idx_to_aerial_ids[si_gtg.idx] = Int64[]
                end
                push!(gtg_idx_to_aerial_ids[si_gtg.idx], i) 
            end
        end
    end

    # Now iterate over keys of gtg_idx_to_aerial_ids in order
    # Whenever we hit a key with more than capacity drones, continue until fewer than capacity OR non-consec IDXs
    gid_to_aerial_ids = Dict{Int64,Vector{Int64}}()
    for (gtgidx, aerial_ids) in gtg_idx_to_aerial_ids

        if length(aerial_ids) > env.car_capacity
            
            gid = env.ground_transit_graph.vertices[gtgidx].ground_agent_id

            # Now just continue until different running agent set
            if ~haskey(gid_to_aerial_ids, gid)
                gid_to_aerial_ids[gid] = aerial_ids
            end
        end
    end

    if ~isempty(gid_to_aerial_ids)

        env.num_global_conflicts += 1
        # println("Aerial CBS conflict number $(env.num_global_conflicts)")
        
        overlap_gids = collect(keys(gid_to_aerial_ids))
        aerial_agent_ids = collect(values(gid_to_aerial_ids))

        # @infiltrate
        return GroundTransitConflict(overlap_gids, aerial_agent_ids)
    end

    return nothing
end # function get_first_conflict

function MultiAgentPathFinding.create_constraints_from_conflict(env::CoordinatedMAPFEnv, conflict::GroundTransitConflict)

    # constraint = Dict(conflict.aerial_agent_ids[1]=>GroundTransitConstraint(conflict.overlap_gtg_vertices), conflict.aerial_agent_ids[2]=> GroundTransitConstraint(conflict.overlap_gtg_vertices))

    # Generate all NC2 
    aerial_gids_to_avoid = Dict{Int64,Set{Int64}}()

    # Get all car IDs from overlap_gtg_vertices
    for (gid, aids) in zip(conflict.overlap_gids, conflict.aerial_agent_ids)
        for aid in aids
            if ~haskey(aerial_gids_to_avoid, aid)
                aerial_gids_to_avoid[aid] = Set{Int64}(gid)
            else
                push!(aerial_gids_to_avoid[aid], gid)
            end
        end
    end

    constraint = Dict{Int64,GroundTransitConstraint}(aid => GroundTransitConstraint(gids) for (aid,gids) in aerial_gids_to_avoid)

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
# Focal transition heuristic just counts boarding constraints
function focal_transition_heuristic_transit(env::CoordinatedMAPFEnv, solution::Vector{PlanResult{AerialMAPFState,GroundTransitAction,Float64}}, aerial_agent_id::Int64, u::AerialMAPFState, v::AerialMAPFState) 

    vgtg = env.ground_transit_graph.vertices[v.ground_transit_idx]

    # Trivial check that it doesn't fly to goal
    if vgtg.ground_agent_id == 0
        return 0
    end

    num_conflicts = 0

    # Target vertex is car vertex
    for (i, aerial_soln) in enumerate(solution)
        if i != aerial_agent_id && (~isempty(aerial_soln))

            for (amapf_state, _) in aerial_soln.states[2:end-1]

                vtx_state = env.ground_transit_graph.vertices[amapf_state.ground_transit_idx]
                # Add if any other drone uses at any point
                if vtx_state.ground_agent_id == vgtg.ground_agent_id && vtx_state.vertex_along_path == vgtg.vertex_along_path 
                    num_conflicts += 1
                    continue
                end
            end
        end # if i && isempty
    end # enumerate(solution)

    return num_conflicts
end

# TODO: Review
function MultiAgentPathFinding.focal_heuristic(env::CoordinatedMAPFEnv, solution::Vector{PlanResult{AerialMAPFState,GroundTransitAction,Float64}}) 

    num_potential_conflicts = 0

    gtg_idx_to_aerial_ids = Dict{Int64,Int64}()

    for (i, sol_i) in enumerate(solution)
        for ((si, _), (sip,_)) in zip(sol_i.states[1:end-1], sol_i.states[2:end])
            si_gtg = env.ground_transit_graph.vertices[si.ground_transit_idx]
            sip_gtg = env.ground_transit_graph.vertices[sip.ground_transit_idx]
            if si_gtg.ground_agent_id !=0 && si_gtg.ground_agent_id == sip_gtg.ground_agent_id
                if ~haskey(gtg_idx_to_aerial_ids, si_gtg.idx)
                    gtg_idx_to_aerial_ids[si_gtg.idx] = 0
                end
                gtg_idx_to_aerial_ids[si_gtg.idx] = gtg_idx_to_aerial_ids[si_gtg.idx] + 1
            end
        end
    end

    for v in values(gtg_idx_to_aerial_ids)
        if v > env.car_capacity
            num_potential_conflicts += 1
        end
    end

    return num_potential_conflicts
end

# Just use BFS here
# function MultiAgentPathFinding.low_level_search!(solver::ECBSSolver, aerial_agent_id::Int64, s::AerialMAPFState, constraint::GroundTransitConstraint, solution::Vector{PlanResult{AerialMAPFState,GroundTransitAction,Float64}}) 

function MultiAgentPathFinding.low_level_search!(solver::CBSSolver, aerial_agent_id::Int64, s::AerialMAPFState, constraint::GroundTransitConstraint) 

    ## TODO
    env = solver.env

    env.aerial_mapf_graph = SimpleVListGraph{AerialMAPFState}()
    Graphs.add_vertex!(env.aerial_mapf_graph, s)

    env.gtg_idx_gval =  Dict{Int64,Float64}()

    edge_wt_fn(u, v) = amapfg_edge_wt_fn(env, u, v)
    admissible_heuristic(u) = direct_flight_time_heuristic(env, u)

    # Set start and goal
    start_idx = 1   # Always one since graph reinitialized
    env.next_gtg_goal_idx = env.gtg_drone_start_goal_idxs[aerial_agent_id][2]
    env.next_amapfg_goal_idx = 0

    vis = GroundTransitGoalVisitor(env, constraint.avoid_gid_vertex_set)

    # println("Size of constraints: $(length(constraint.avoid_gid_vertex_set))")

    a_star_states = a_star_implicit_shortest_path!(env.aerial_mapf_graph,
                                                    edge_wt_fn,
                                                    start_idx,
                                                    vis,
                                                    admissible_heuristic
                                                    )
    # println("Size of amapf_graph: $(length(env.aerial_mapf_graph.vertices))")
    
    # Get solution from a_star_eps_states
    # NOTE: It should always have a solution because drone can just fly
    @assert haskey(a_star_states.parent_indices, env.next_amapfg_goal_idx)

    # Get path cost
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
    avoid_gid_vertex_set::Set{Int64}
end


function Graphs.include_vertex!(vis::GroundTransitGoalVisitor, u::AerialMAPFState, v::AerialMAPFState, d::Float64, nbrs::Vector{Int64})

    env = vis.env
    avoid_gid_vertex_set = vis.avoid_gid_vertex_set
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
        for (gid, (gstart, gend)) in enumerate(env.gtg_ground_path_start_ends)
            # gts_non_dom = get_non_dominated_transit_points(env, gstart, gend, avoid_gid_vertex_set, v, false)

            # Add AerialMAPFState versions of them
            # for gts_tup in gts_non_dom
            if ~(gid in avoid_gid_vertex_set)

                gts_tup = get_best_transit_connection(env, gstart, gend, v, false)

                if ~isnothing(gts_tup)

                    idx_ctr += 1
                    new_vert = AerialMAPFState(idx=idx_ctr, ground_transit_idx=gts_tup.idx, timeval=gts_tup.timeval, distval=gts_tup.distval)

                    Graphs.add_vertex!(env.aerial_mapf_graph, new_vert)
                    push!(nbrs, idx_ctr)

                    env.gtg_idx_gval[gts_tup.idx] = env.alpha_weight_distance*new_vert.distval + (1.0 - env.alpha_weight_distance)*new_vert.timeval

                end # isnothing(gts_tup)
            end # ~(gid in avoid_gid_vertex_set)
        end

    else
        # Either first transfer to new ground point OR along the way
        if vgtg.ground_agent_id != ugtg.ground_agent_id

            new_considered_cars = union(v.considered_cars, Set{Int64}([vgtg.ground_agent_id]))

            next_gtg = env.ground_transit_graph.vertices[vgtg.idx + 1]

            idx_ctr += 1
            # IMP NOTE: Timeval may be greater than vgtg.timeval (probably will be)
            # So the new timeval should be v.timeval + car_time_diff (next_gtg.timeval - vgtg.timeval)
            tval = v.timeval + (next_gtg.timeval - vgtg.timeval)
            new_vert = AerialMAPFState(idx=idx_ctr, ground_transit_idx=next_gtg.idx, timeval=tval, distval=v.distval, considered_cars=new_considered_cars)

            Graphs.add_vertex!(env.aerial_mapf_graph, new_vert)
            push!(nbrs, idx_ctr)

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

                idx_ctr += 1
                # IMP NOTE: Timeval may be greater than vgtg.timeval (probably will be)
                # So the new timeval should be v.timeval + car_time_diff (next_gtg.timeval - vgtg.timeval)
                tval = v.timeval + (next_gtg.timeval - vgtg.timeval)
                new_vert = AerialMAPFState(idx=idx_ctr, ground_transit_idx=next_gtg.idx, timeval=tval, distval=v.distval, considered_cars=v.considered_cars)


                Graphs.add_vertex!(env.aerial_mapf_graph, new_vert)
                push!(nbrs, idx_ctr)

            end

            for (gid, (gstart, gend)) in enumerate(env.gtg_ground_path_start_ends)
                
                if gid != vgtg.ground_agent_id && ~(gid in v.considered_cars) && ~(gid in avoid_gid_vertex_set)
                    # gts_non_dom = get_non_dominated_transit_points(env, gstart, gend, avoid_gid_vertex_set, v, true)
                    
                    gts_tup = get_best_transit_connection(env, gstart, gend, v, true)

                    # Add AerialMAPFState versions of them
                    if ~isnothing(gts_tup)

                        # At least here compute the real thing, the true SP between transit connections
                        # bidir_sp = compute_bidir_astar_euclidean(env.road_graph, vgtg.road_vtx_id, env.ground_transit_graph.vertices[gts_tup.idx].road_vtx_id, env.road_graph_wts, env.location_list)

                        true_timeval = gts_tup.timeval
                        true_distval = gts_tup.distval
                        
                        # Compute the true connection cost between them
                        # if length(bidir_sp.path) > 1
                        #     true_distval = v.distval + bidir_sp.dist
                        #     true_timeval = max(v.timeval + bidir_sp.dist/AvgSpeed, gts_tup.timeval)
                        # end

                        temp_gval = env.alpha_weight_distance*true_distval + (1.0 - env.alpha_weight_distance)*true_timeval

                        if ~haskey(env.gtg_idx_gval, gts_tup.idx) || temp_gval < env.gtg_idx_gval[gts_tup.idx]

                            idx_ctr += 1
                            new_vert = AerialMAPFState(idx=idx_ctr, ground_transit_idx=gts_tup.idx, timeval=true_timeval, distval=true_distval, considered_cars=v.considered_cars)

                            Graphs.add_vertex!(env.aerial_mapf_graph, new_vert)
                            push!(nbrs, idx_ctr)
                            
                            env.gtg_idx_gval[gts_tup.idx] = temp_gval
                        end
                    end # isnothing(gts_tup)
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
function update_aerial_ground_paths_cbs_solution!(env::CoordinatedMAPFEnv, solution::Vector{PlanResult{AerialMAPFState,GroundTransitAction,Float64}}, drone_flight_paths::Vector{AgentPathInfo}) 

    final_aerial_paths = Vector{AgentPathInfo}(undef, length(solution))
    ground_paths = env.ground_paths

    # Iterate over solution
    for (aerial_agent_id, aerial_soln) in enumerate(solution)

        # Direct flight path
        if length(aerial_soln.states) == 2
            # Just copy over drone flight paths
            final_aerial_paths[aerial_agent_id] = drone_flight_paths[aerial_agent_id]
        else
            # Iterate over drone car usage
            # If time val of boarding is greater than drone's timestamp, edit drone path
            # If time val of drone is greater than car vertex then edit car path
            # TODO define function to do this update
            path_states = AgentPathState[]
            running_time = 0.0
            running_dist = 0.0

            for i = 1:length(aerial_soln.states) - 1

                si, sip = (aerial_soln.states[i][1], aerial_soln.states[i+1][1])

                si_gtg = env.ground_transit_graph.vertices[si.ground_transit_idx]
                sip_gtg = env.ground_transit_graph.vertices[sip.ground_transit_idx]

                push!(path_states, AgentPathState(road_vtx_id=si_gtg.road_vtx_id, timeval=running_time, coord_agent_id=Set{Int64}(si_gtg.ground_agent_id)))

                # Update ground path coord_id for drone
                if si_gtg.ground_agent_id != 0
                    push!(env.ground_paths[si_gtg.ground_agent_id].path_states[si_gtg.vertex_along_path].coord_agent_id, aerial_agent_id)
                end

                # Two cases: transit or transfer 
                # Case 1 - flight expanded path between vertices (first or last hop)
                if si_gtg.ground_agent_id != sip_gtg.ground_agent_id

                    if haskey(env.ground_transit_expanded_paths, (si_gtg.idx, sip_gtg.idx))

                        running_dist += env.ground_transit_weights[(si_gtg.idx, sip_gtg.idx)]
                        
                        # Add expanded paths up to last but one
                        exp_path = env.ground_transit_expanded_paths[(si_gtg.idx, sip_gtg.idx)]
                        for idx = 2:length(exp_path)-1
                            running_time += env.road_graph_wts[exp_path[idx-1], exp_path[idx]] / AvgSpeed
                            push!(path_states, AgentPathState(road_vtx_id=exp_path[idx], timeval=running_time))
                        end
                        running_time += env.road_graph_wts[exp_path[end-1], exp_path[end]] / AvgSpeed
                    else
                        # Actually run bidir_eucl for transfer and add path states
                        bidir_sp = compute_bidir_astar_euclidean(env.road_graph, si_gtg.road_vtx_id, sip_gtg.road_vtx_id, env.road_graph_wts, env.location_list)
                        # @infiltrate

                        if isempty(bidir_sp.path)

                            # Just add flight path; don't need to add any states
                            flight_dist = distance_lat_lon_euclidean(env.location_list[si_gtg.road_vtx_id], env.location_list[sip_gtg.road_vtx_id])
                            running_dist += flight_dist
                            running_time += flight_dist / AvgSpeed

                        elseif length(bidir_sp.path) > 1

                            running_dist += bidir_sp.dist
                            exp_path = bidir_sp.path
                            for idx = 2:length(exp_path)-1
                                running_time += env.road_graph_wts[exp_path[idx-1], exp_path[idx]] / AvgSpeed
                                push!(path_states, AgentPathState(road_vtx_id=exp_path[idx], timeval=running_time))
                            end
                            running_time += env.road_graph_wts[exp_path[end-1], exp_path[end]] / AvgSpeed

                        end
                    end # haskey(env.ground_transit_expanded_paths, (si_gtg.idx, sip_gtg.idx))

                    # Do some extra work if last is 
                    if sip_gtg.ground_agent_id != 0

                        # Big TODO
                        # If running time is more than car, update car
                        # If running time is less than car, update running time
                        if sip_gtg.timeval < running_time

                            # @infiltrate

                            ground_route_running_time = running_time

                            # Continue until subsumed by existing time...may go all the way to end
                            for gtg_idx = sip_gtg.idx : env.gtg_ground_path_start_ends[sip_gtg.ground_agent_id][2]-1

                                gtg_vtx = env.ground_transit_graph.vertices[gtg_idx]
                                if gtg_vtx.timeval >= ground_route_running_time
                                    break
                                end

                                # Compute difference to next one BEFORE updating ground route running time
                                time_to_incr = env.ground_transit_graph.vertices[gtg_idx+1].timeval - env.ground_transit_graph.vertices[gtg_idx].timeval

                                # Edit timeval for BOTH GTS and ground paths at vertex along path
                                env.ground_transit_graph.vertices[gtg_idx].timeval = ground_route_running_time
                                env.ground_paths[sip_gtg.ground_agent_id].path_states[gtg_vtx.vertex_along_path].timeval = ground_route_running_time

                                # Update ground route running time by difference
                                ground_route_running_time += time_to_incr

                            end

                            env.ground_transit_graph.vertices[env.gtg_ground_path_start_ends[sip_gtg.ground_agent_id][2]].timeval = ground_route_running_time
                            env.ground_paths[sip_gtg.ground_agent_id].path_states[end].timeval = ground_route_running_time

                            
                            # @infiltrate
                            
                        else
                            # TODO: Infiltrate and check
                            # @infiltrate
                            running_time = sip_gtg.timeval
                        end # if sip_gtg.timeval < running_time
                    end # sip_gtg.ground_agent_id != 0
                else
                    # Transfer, just update running time
                    running_time += sip_gtg.timeval - si_gtg.timeval
                end
            end # i = 1: length(states) - 1

            # Add last state
            push!(path_states, AgentPathState(road_vtx_id = env.aerial_task_list[aerial_agent_id].dest, timeval=running_time))

            final_aerial_paths[aerial_agent_id] = AgentPathInfo(path_states=path_states, total_dist=running_dist, total_time=running_time)

        end # length(states) == or != 2
    end # enumerate(solution)
    
    env.aerial_paths = final_aerial_paths

    # Update total time of ground paths
    for (i, gp) in enumerate(env.ground_paths)
        env.ground_paths[i].total_time = gp.path_states[end].timeval
    end

end
