#!/bin/bash

julia1.5 benchmark_direct.jl 20 40 ../data/results/direct/Direct 20
julia1.5 benchmark_pp.jl 20 40 ../data/results/pp/PP 5 20
julia1.5 benchmark_pp.jl 20 40 ../data/results/pp/PP 10 20


julia1.5 benchmark_direct.jl 20 60 ../data/results/direct/Direct 20
julia1.5 benchmark_pp.jl 20 60 ../data/results/pp/PP 5 20
julia1.5 benchmark_pp.jl 20 60 ../data/results/pp/PP 10 20


julia1.5 benchmark_direct.jl 20 80 ../data/results/direct/Direct 20
julia1.5 benchmark_pp.jl 20 80 ../data/results/pp/PP 5 20
julia1.5 benchmark_pp.jl 20 80 ../data/results/pp/PP 10 20