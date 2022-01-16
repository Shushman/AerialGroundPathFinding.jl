#!/bin/bash

# julia1.5 benchmark_pp.jl 5 10 ../data/results/pp_unordered/tanh/PP 5 20
# julia1.5 benchmark_pp.jl 5 10 ../data/results/pp_unordered/tanh/PP 10 20
# julia1.5 benchmark_pp.jl 5 15 ../data/results/pp_unordered/tanh/PP 5 20
# julia1.5 benchmark_pp.jl 5 15 ../data/results/pp_unordered/tanh/PP 10 20
# julia1.5 benchmark_pp.jl 5 20 ../data/results/pp_unordered/tanh/PP 5 20
# julia1.5 benchmark_pp.jl 5 20 ../data/results/pp_unordered/tanh/PP 10 20

# julia1.5 benchmark_pp.jl 10 20 ../data/results/pp_unordered/tanh/PP 5 20
# julia1.5 benchmark_pp.jl 10 20 ../data/results/pp_unordered/tanh/PP 10 20
# julia1.5 benchmark_pp.jl 10 30 ../data/results/pp_unordered/tanh/PP 5 20
# julia1.5 benchmark_pp.jl 10 30 ../data/results/pp_unordered/tanh/PP 10 20
# julia1.5 benchmark_pp.jl 10 40 ../data/results/pp_unordered/tanh/PP 5 20
# julia1.5 benchmark_pp.jl 10 40 ../data/results/pp_unordered/tanh/PP 10 20

# julia1.5 benchmark_pp.jl 15 30 ../data/results/pp_unordered/tanh/PP 5 20
# julia1.5 benchmark_pp.jl 15 30 ../data/results/pp_unordered/tanh/PP 10 20
# julia1.5 benchmark_pp.jl 15 45 ../data/results/pp_unordered/tanh/PP 5 20
# julia1.5 benchmark_pp.jl 15 45 ../data/results/pp_unordered/tanh/PP 10 20
# julia1.5 benchmark_pp.jl 15 60 ../data/results/pp_unordered/tanh/PP 5 20
# julia1.5 benchmark_pp.jl 15 60 ../data/results/pp_unordered/tanh/PP 10 20

# julia1.5 benchmark_pp.jl 20 40 ../data/results/pp_unordered/tanh/PP 5 20
# julia1.5 benchmark_pp.jl 20 40 ../data/results/pp_unordered/tanh/PP 10 20
# julia1.5 benchmark_pp.jl 20 60 ../data/results/pp_unordered/tanh/PP 5 20
# julia1.5 benchmark_pp.jl 20 60 ../data/results/pp_unordered/tanh/PP 10 20
# julia1.5 benchmark_pp.jl 20 80 ../data/results/pp_unordered/tanh/PP 5 20
# julia1.5 benchmark_pp.jl 20 80 ../data/results/pp_unordered/tanh/PP 10 20

julia1.5 benchmark_stage2ablation_pp.jl 10 50 ../data/results/stage2_ablation/PP 5 20
julia1.5 benchmark_stage2ablation_pp.jl 10 50 ../data/results/stage2_ablation/PP 10 20
julia1.5 benchmark_stage2ablation_pp.jl 15 75 ../data/results/stage2_ablation/PP 5 20
julia1.5 benchmark_stage2ablation_pp.jl 15 75 ../data/results/stage2_ablation/PP 10 20

julia1.5 benchmark_pp.jl 10 50 ../data/results/pp_unordered/tanh/PP 5 20
julia1.5 benchmark_pp.jl 10 50 ../data/results/pp_unordered/tanh/PP 10 20
julia1.5 benchmark_pp.jl 15 75 ../data/results/pp_unordered/tanh/PP 5 20
julia1.5 benchmark_pp.jl 15 75 ../data/results/pp_unordered/tanh/PP 10 20