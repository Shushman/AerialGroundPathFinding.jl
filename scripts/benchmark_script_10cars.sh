#!/bin/bash

julia1.5 benchmark_direct.jl 10 20 ../data/results/direct/Direct 20
julia1.5 benchmark_cbs.jl 10 20 ../data/results/cbs/CBS 5 20
julia1.5 benchmark_pp.jl 10 20 ../data/results/pp/PP 5 20


julia1.5 benchmark_direct.jl 10 20 ../data/results/direct/Direct 20
julia1.5 benchmark_cbs.jl 10 20 ../data/results/cbs/CBS 10 20
julia1.5 benchmark_pp.jl 10 20 ../data/results/pp/PP 10 20


julia1.5 benchmark_direct.jl 10 30 ../data/results/direct/Direct 20
julia1.5 benchmark_cbs.jl 10 30 ../data/results/cbs/CBS 5 20
julia1.5 benchmark_pp.jl 10 30 ../data/results/pp/PP 5 20


julia1.5 benchmark_direct.jl 10 30 ../data/results/direct/Direct 20
julia1.5 benchmark_cbs.jl 10 30 ../data/results/cbs/CBS 10 20
julia1.5 benchmark_pp.jl 10 30 ../data/results/pp/PP 10 20


julia1.5 benchmark_direct.jl 10 40 ../data/results/direct/Direct 20
julia1.5 benchmark_cbs.jl 10 40 ../data/results/cbs/CBS 5 20
julia1.5 benchmark_pp.jl 10 40 ../data/results/pp/PP 5 20



julia1.5 benchmark_direct.jl 10 40 ../data/results/direct/Direct 20
julia1.5 benchmark_cbs.jl 10 40 ../data/results/cbs/CBS 10 20
julia1.5 benchmark_pp.jl 10 40 ../data/results/pp/PP 10 20