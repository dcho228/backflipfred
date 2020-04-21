import Pkg;
Pkg.activate(@__DIR__);
using RigidBodyDynamics, RigidBodySim, DifferentialEquations
using LinearAlgebra
using StaticArrays, Plots
using MeshCat, MeshCatMechanisms
vis = Visualizer()
#open(vis)
