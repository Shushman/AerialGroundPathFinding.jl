#!/bin/bash

# 10 drones; capacity 5
julia1.5 benchmark_direct.jl 5 10 ../data/results/direct/Direct 20
julia1.5 benchmark_cbs.jl 5 10 ../data/results/ecbs/ECBS 5 20 500
julia1.5 benchmark_pp.jl 5 10 ../data/results/pp/PP 5 20

# 10 drones; capacity 10
julia1.5 benchmark_direct.jl 5 10 ../data/results/direct/Direct 20
julia1.5 benchmark_cbs.jl 5 10 ../data/results/cbs/CBS 10 20 500
julia1.5 benchmark_pp.jl 5 10 ../data/results/pp/PP 10 20

# 15 drones; capacity 5
julia1.5 benchmark_direct.jl 5 15 ../data/results/direct/Direct 20
julia1.5 benchmark_cbs.jl 5 15 ../data/results/cbs/CBS 5 20 500
julia1.5 benchmark_pp.jl 5 15 ../data/results/pp/PP 5 20

# 15 drones; capacity 10
julia1.5 benchmark_direct.jl 5 15 ../data/results/direct/Direct 20
julia1.5 benchmark_cbs.jl 5 15 ../data/results/cbs/CBS 10 20
julia1.5 benchmark_pp.jl 5 15 ../data/results/pp/PP 10 20


# 20 drones; capacity 5
julia1.5 benchmark_direct.jl 5 20 ../data/results/direct/Direct 20
julia1.5 benchmark_cbs.jl 5 20 ../data/results/cbs/CBS 5 20 1000
julia1.5 benchmark_pp.jl 5 20 ../data/results/pp/PP 5 20

# 20 drones; capacity 10
julia1.5 benchmark_direct.jl 5 20 ../data/results/direct/Direct 20
julia1.5 benchmark_cbs.jl 5 20 ../data/results/cbs/CBS 10 20
julia1.5 benchmark_pp.jl 5 20 ../data/results/pp/PP 10 20