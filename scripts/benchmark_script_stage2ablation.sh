#!/bin/bash

julia1.5 benchmark_stage2ablation_pp.jl 5 20 ../data/results/stage2_ablation/PP 5 20
julia1.5 benchmark_stage2ablation_pp.jl 10 40 ../data/results/stage2_ablation/PP 10 20
julia1.5 benchmark_stage2ablation_pp.jl 15 60 ../data/results/stage2_ablation/PP 5 20
julia1.5 benchmark_stage2ablation_pp.jl 20 80 ../data/results/stage2_ablation/PP 10 20