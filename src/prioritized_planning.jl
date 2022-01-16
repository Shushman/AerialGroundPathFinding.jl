"""
    plan_prioritized_paths!(env::CoordinatedMAPFEnv, agent_ordering::Vector{Int64}, initial_states::Vector{MS}, cnr::Type{CNR}, pr::Type{PR})

Run the Prioritized Planning algorithm. Given an ordering over agents, plan a path for each of them
one-by-one and impose constraints derived from the path on subsequent agents.
"""
function plan_prioritized_paths!(env::CoordinatedMAPFEnv, agent_ordering::Vector{Int64}, initial_states::Vector{MS}, cnr::Type{CNR}, pr::Type{PR}) where {MS <: MAPFState, CNR <: MAPFConstraints, PR <: PlanResult}

    running_constraint = MultiAgentPathFinding.get_empty_constraint(cnr)
    paths = pr[]

    # Iterate over agent ordering
    for id in agent_ordering

        # Run low level search for each agent
        id_path = low_level_pp_search!(env, id, initial_states[id], running_constraint)

        # Update constraints
        add_constraint_from_pp_search!(env, running_constraint, id, id_path)
        
        # Add to path set
        push!(paths, id_path)

    end # id in agent_ordering
    return paths
end

## Just implement the various functions for aerial and ground here
function low_level_pp_search!(env::CoordinatedMAPFEnv, ground_agent_id::Int64, s::GroundMAPFState, constraint::GroundMAPFConstraint)

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

    a_star_states = a_star_implicit_shortest_path!(env.ground_mapf_graph, edge_wt_fn, start_idx, vis, admissible_heuristic)

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

    return PlanResult{GroundMAPFState,GroundMAPFAction,Float64}(states, actions, sp_cost, sp_cost)
end

function add_constraint_from_pp_search!(env::CoordinatedMAPFEnv, constraint::GroundMAPFConstraint, ground_id::Int64, id_path::PlanResult)

    for (id_action, _) in id_path.actions
        
        # Just add the aerial IDs for the edge
        if !isempty(id_action.edge_aerial_ids)
            if ~haskey(constraint.avoid_edge_copy_set, id_action.edge_id)
                constraint.avoid_edge_copy_set[id_action.edge_id] = Set{Int64}()
            end
            union!(constraint.avoid_edge_copy_set[id_action.edge_id], id_action.edge_aerial_ids)
        end
    end
end


function low_level_pp_search!(env::CoordinatedMAPFEnv, aerial_agent_id::Int64, s::AerialMAPFState, constraint::GroundTransitConstraint)

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

    a_star_states = a_star_implicit_shortest_path!(env.aerial_mapf_graph,
                                                    edge_wt_fn,
                                                    start_idx,
                                                    vis,
                                                    admissible_heuristic
                                                    )
    
    # Get solution from a_star_eps_states
    # NOTE: It should always have a solution because drone can just fly
    @assert haskey(a_star_states.parent_indices, env.next_amapfg_goal_idx)

    # Get path cost
    states = Tuple{AerialMAPFState,Float64}[]
    actions = Tuple{GroundTransitAction, Float64}[]

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
end

function add_constraint_from_pp_search!(env::CoordinatedMAPFEnv, constraint::GroundTransitConstraint, aerial_id::Int64, id_path::PlanResult)

    for ((si, _), (sip, _)) in zip(id_path.states[1:end-1], id_path.states[2:end])
        si_gtg = env.ground_transit_graph.vertices[si.ground_transit_idx]
        sip_gtg = env.ground_transit_graph.vertices[sip.ground_transit_idx]
        if si_gtg.ground_agent_id !=0 && si_gtg.ground_agent_id == sip_gtg.ground_agent_id
            if ~haskey(env.gtg_idx_to_aerial_ids, si_gtg.idx)
                env.gtg_idx_to_aerial_ids[si_gtg.idx] = Int64[]
            end
            push!(env.gtg_idx_to_aerial_ids[si_gtg.idx], aerial_id)
        end
    end

    for (gtgidx, aerial_ids) in env.gtg_idx_to_aerial_ids
        if length(aerial_ids) > env.car_capacity
            gid = env.ground_transit_graph.vertices[gtgidx].ground_agent_id
            union!(constraint.avoid_gid_vertex_set, gid)
        end
    end

end


struct GroundTransitVertexGoalVisitor <: Graphs.AbstractDijkstraVisitor
    env::CoordinatedMAPFEnv
    avoid_gtg_vertex_set::Set{Int64}
end

function Graphs.include_vertex!(vis::GroundTransitVertexGoalVisitor, u::AerialMAPFState, v::AerialMAPFState, d::Float64, nbrs::Vector{Int64})

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
        for (_, (gstart, gend)) in enumerate(env.gtg_ground_path_start_ends)
            gts_non_dom = get_non_dominated_transit_points(env, gstart, gend, avoid_gtg_vertex_set, v, false)

            # Add AerialMAPFState versions of them
            for gts_tup in gts_non_dom

                idx_ctr += 1
                new_vert = AerialMAPFState(idx=idx_ctr, ground_transit_idx=gts_tup.idx, timeval=gts_tup.timeval, distval=gts_tup.distval)

                Graphs.add_vertex!(env.aerial_mapf_graph, new_vert)
                push!(nbrs, idx_ctr)

                env.gtg_idx_gval[gts_tup.idx] = env.alpha_weight_distance*new_vert.distval + (1.0 - env.alpha_weight_distance)*new_vert.timeval
            end
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
                
                if gid != vgtg.ground_agent_id && ~(gid in v.considered_cars)
                    gts_non_dom = get_non_dominated_transit_points(env, gstart, gend, avoid_gtg_vertex_set, v, true)
                    
                    # Add AerialMAPFState versions of them
                    for gts_tup in gts_non_dom

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
                    end
                end
            end # (gstart, gend) in gtg_ground_path_start_ends
        end # vgtg.ground_agent_id != ugtg.ground_agent_id
    end # vgtg.ground == or != 0

    return true
end


function low_level_pp_search!(env::CoordinatedMAPFEnv, aerial_agent_id::Int64, s::AerialMAPFState, constraint::GroundTransitVertexConstraint)

    env.aerial_mapf_graph = SimpleVListGraph{AerialMAPFState}()
    Graphs.add_vertex!(env.aerial_mapf_graph, s)

    env.gtg_idx_gval =  Dict{Int64,Float64}()

    edge_wt_fn(u, v) = amapfg_edge_wt_fn(env, u, v)
    admissible_heuristic(u) = direct_flight_time_heuristic(env, u)

    start_idx = 1   # Always one since graph reinitialized
    env.next_gtg_goal_idx = env.gtg_drone_start_goal_idxs[aerial_agent_id][2]
    env.next_amapfg_goal_idx = 0

    vis = GroundTransitVertexGoalVisitor(env, constraint.avoid_gtg_vertex_set)
    a_star_states = a_star_implicit_shortest_path!(env.aerial_mapf_graph,
                                                    edge_wt_fn,
                                                    start_idx,
                                                    vis,
                                                    admissible_heuristic
                                                    )
    
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

function add_constraint_from_pp_search!(env::CoordinatedMAPFEnv, constraint::GroundTransitVertexConstraint, aerial_id::Int64, id_path::PlanResult)
    
    for ((si, _), (sip, _)) in zip(id_path.states[1:end-1], id_path.states[2:end])
        si_gtg = env.ground_transit_graph.vertices[si.ground_transit_idx]
        sip_gtg = env.ground_transit_graph.vertices[sip.ground_transit_idx]
        if si_gtg.ground_agent_id !=0 && si_gtg.ground_agent_id == sip_gtg.ground_agent_id
            if ~haskey(env.gtg_idx_to_aerial_ids, si_gtg.idx)
                env.gtg_idx_to_aerial_ids[si_gtg.idx] = Int64[]
            end
            push!(env.gtg_idx_to_aerial_ids[si_gtg.idx], aerial_id)
        end
    end

    for (gtgidx, aerial_ids) in env.gtg_idx_to_aerial_ids
        if length(aerial_ids) > env.car_capacity
            union!(constraint.avoid_gtg_vertex_set, gtgidx)
        end
    end
end