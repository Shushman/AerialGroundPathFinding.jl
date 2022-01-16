#!/bin/bash

# julia1.5 benchmark_direct.jl 10 20 ../data/results/direct/Direct 20
# julia1.5 benchmark_cbs.jl 10 20 ../data/results/cbs/CBS 5 20 500
# julia1.5 benchmark_pp.jl 10 20 ../data/results/pp/PP 5 20


# julia1.5 benchmark_direct.jl 10 20 ../data/results/direct/Direct 20
# julia1.5 benchmark_cbs.jl 10 20 ../data/results/cbs/CBS 10 20
# julia1.5 benchmark_pp.jl 10 20 ../data/results/pp/PP 10 20


# julia1.5 benchmark_direct.jl 10 30 ../data/results/direct/Direct 20
# julia1.5 benchmark_cbs.jl 10 30 ../data/results/cbs/CBS 5 20 500
# julia1.5 benchmark_pp.jl 10 30 ../data/results/pp/PP 5 20


# julia1.5 benchmark_direct.jl 10 30 ../data/results/direct/Direct 20
# julia1.5 benchmark_cbs.jl 10 30 ../data/results/cbs/CBS 10 20 500
# julia1.5 benchmark_pp.jl 10 30 ../data/results/pp/PP 10 20

## NOTE - Repurposing for stage-wise timing; delete block later!

julia1.5 benchmark_cbs.jl 5 10 ../data/results/stagewise_timing/ECBS 5 20 500
julia1.5 benchmark_cbs.jl 5 15 ../data/results/stagewise_timing/ECBS 5 20 100
julia1.5 benchmark_cbs.jl 5 20 ../data/results/stagewise_timing/ECBS 5 20 100

julia1.5 benchmark_cbs.jl 5 10 ../data/results/stagewise_timing/ECBS 10 20 500
julia1.5 benchmark_cbs.jl 5 15 ../data/results/stagewise_timing/ECBS 10 20 100
julia1.5 benchmark_cbs.jl 5 20 ../data/results/stagewise_timing/ECBS 10 20 100


julia1.5 benchmark_pp.jl 5 10 ../data/results/stagewise_timing/PP 5 20 500
julia1.5 benchmark_pp.jl 5 15 ../data/results/stagewise_timing/PP 5 20 100
julia1.5 benchmark_pp.jl 5 20 ../data/results/stagewise_timing/PP 5 20 100

julia1.5 benchmark_pp.jl 5 10 ../data/results/stagewise_timing/PP 10 20 500
julia1.5 benchmark_pp.jl 5 15 ../data/results/stagewise_timing/PP 10 20 100
julia1.5 benchmark_pp.jl 5 20 ../data/results/stagewise_timing/PP 10 20 100

julia1.5 benchmark_pp.jl 10 20 ../data/results/stagewise_timing/PP 5 20 500
julia1.5 benchmark_pp.jl 10 30 ../data/results/stagewise_timing/PP 5 20 100
julia1.5 benchmark_pp.jl 10 40 ../data/results/stagewise_timing/PP 5 20 100

julia1.5 benchmark_pp.jl 10 20 ../data/results/stagewise_timing/PP 10 20 500
julia1.5 benchmark_pp.jl 10 30 ../data/results/stagewise_timing/PP 10 20 100
julia1.5 benchmark_pp.jl 10 40 ../data/results/stagewise_timing/PP 10 20 100

# julia1.5 benchmark_cbs.jl 10 40 ../data/results/ecbs/ECBS 10 20 100


# julia1.5 benchmark_direct.jl 10 40 ../data/results/direct/Direct 20
# julia1.5 benchmark_cbs.jl 10 40 ../data/results/cbs/CBS 5 20
# julia1.5 benchmark_pp.jl 10 40 ../data/results/pp/PP 5 20



# julia1.5 benchmark_direct.jl 10 40 ../data/results/direct/Direct 20
# julia1.5 benchmark_cbs.jl 10 40 ../data/results/cbs/CBS 10 20
# julia1.5 benchmark_pp.jl 10 40 ../data/results/pp/PP 10 20