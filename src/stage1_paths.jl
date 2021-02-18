
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

# # TODO : Redo to reflect time
# function aerial_ground_coord_path_cost(env::CoordinatedMAPFEnv, aerial_id::Int64, ground_id::Int64)

#     aerial_task = env.aerial_task_list[aerial_id]
#     ground_path_states = env.ground_paths[ground_id].path_states

#     path_idxs = length(ground_path_states)
#     to_costs = Vector{Float64}(undef, path_idxs-1)
#     from_costs = Vector{Float64}(undef, path_idxs-1)

#     # Compute to_costs from origin to ground_path[i]
#     # Compute from_costs from ground_path[i+1] to dest
#     for i = 1:path_idxs-1

#         bidir_sp_to = compute_bidir_astar_euclidean(env.road_graph, aerial_task.origin, ground_path_states[i].road_vtx_id, env.road_graph_wts, env.location_list)
#         to_costs[i] = bidir_sp_to.dist

#         bidir_sp_from = compute_bidir_astar_euclidean(env.road_graph, ground_path_states[i+1].road_vtx_id, aerial_task.dest, env.road_graph_wts, env.location_list)
#         from_costs[i] = bidir_sp_from.dist

#     end

#     best_cost = Inf
#     best_segment = (0, 0)
#     # Find best_start and best_end
#     for i = 1:path_idxs-1
#         for j = i:path_idxs-1
#             if to_costs[i] + from_costs[j] < best_cost
#                 best_cost = to_costs[i] + from_costs[j]
#                 best_segment = (i, j)
#             end
#         end
#     end

#     return best_cost, best_segment
# end # function aerial_ground_coord_path_cost