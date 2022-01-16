#!/bin/bash

## Use the TANH decay function with benchmark_pp here!

julia1.5 benchmark_pp.jl 5 10 ../data/results/pp/new_decay/PP 5 20 TANH
julia1.5 benchmark_pp.jl 5 10 ../data/results/pp/new_decay/PP 10 20 TANH
julia1.5 benchmark_pp.jl 5 15 ../data/results/pp/new_decay/PP 5 20 TANH
julia1.5 benchmark_pp.jl 5 15 ../data/results/pp/new_decay/PP 10 20 TANH
julia1.5 benchmark_pp.jl 5 20 ../data/results/pp/new_decay/PP 5 20 TANH
julia1.5 benchmark_pp.jl 5 20 ../data/results/pp/new_decay/PP 10 20 TANH

julia1.5 benchmark_pp.jl 10 20 ../data/results/pp/new_decay/PP 5 20 TANH
julia1.5 benchmark_pp.jl 10 20 ../data/results/pp/new_decay/PP 10 20 TANH
julia1.5 benchmark_pp.jl 10 30 ../data/results/pp/new_decay/PP 5 20 TANH
julia1.5 benchmark_pp.jl 10 30 ../data/results/pp/new_decay/PP 10 20 TANH
julia1.5 benchmark_pp.jl 10 40 ../data/results/pp/new_decay/PP 5 20 TANH
julia1.5 benchmark_pp.jl 10 40 ../data/results/pp/new_decay/PP 10 20 TANH

julia1.5 benchmark_pp.jl 15 30 ../data/results/pp/new_decay/PP 5 20 TANH
julia1.5 benchmark_pp.jl 15 30 ../data/results/pp/new_decay/PP 10 20 TANH
julia1.5 benchmark_pp.jl 15 45 ../data/results/pp/new_decay/PP 5 20 TANH
julia1.5 benchmark_pp.jl 15 45 ../data/results/pp/new_decay/PP 10 20 TANH
julia1.5 benchmark_pp.jl 15 60 ../data/results/pp/new_decay/PP 5 20 TANH
julia1.5 benchmark_pp.jl 15 60 ../data/results/pp/new_decay/PP 10 20 TANH

julia1.5 benchmark_pp.jl 20 40 ../data/results/pp/new_decay/PP 5 20 TANH
julia1.5 benchmark_pp.jl 20 40 ../data/results/pp/new_decay/PP 10 20 TANH
julia1.5 benchmark_pp.jl 20 60 ../data/results/pp/new_decay/PP 5 20 TANH
julia1.5 benchmark_pp.jl 20 60 ../data/results/pp/new_decay/PP 10 20 TANH
julia1.5 benchmark_pp.jl 20 80 ../data/results/pp/new_decay/PP 5 20 TANH
julia1.5 benchmark_pp.jl 20 80 ../data/results/pp/new_decay/PP 10 20 TANH