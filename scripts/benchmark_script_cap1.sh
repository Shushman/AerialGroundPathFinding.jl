#!/bin/bash

# julia1.5 benchmark_pp.jl 5 5 ../data/results/pp/PP 1 20
# julia1.5 benchmark_pp.jl 10 20 ../data/results/pp/PP 1 20
# julia1.5 benchmark_pp.jl 15 30 ../data/results/pp/PP 1 20
# julia1.5 benchmark_pp.jl 20 40 ../data/results/pp/PP 1 20


julia1.5 benchmark_cbs.jl 5 10 ../data/results/ecbs/ECBS 1 20 5000
# julia1.5 benchmark_cbs.jl 10 10 ../data/results/ecbs/ECBS 1 20 1000
# julia1.5 benchmark_cbs.jl 15 15 ../data/results/ecbs/ECBS 1 20 500
# julia1.5 benchmark_cbs.jl 20 20 ../data/results/ecbs/ECBS 1 20 500