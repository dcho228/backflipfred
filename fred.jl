#using RigidBodySim, RigidBodyDynamics, MeshCat, MeshCatMechanisms

urdfpath="deps/fred/fred.urdf"

mechanism = parse_urdf(Float64, urdfpath)
state = MechanismState(mechanism)
#set_configuration!(state, [0,0,0,0,0,0,0,0,0])

#=function control!(tau, t, state)
    # Proportional-Derivative  Control
    tau .= -20 .* velocity(state) - 100*(configuration(state) - [1.0;1.0])
end

problem = ODEProblem(Dynamics(mechanism), state, (0., 10.))
sol = solve(problem, Tsit5())=#
vis = MechanismVisualizer(mechanism, URDFVisuals(urdfpath),vis)
open(vis) # uncomment to open the visualizer window
#animation = MeshCat.Animation(vis, sol; fps=25)
#setanimation!(mvis, animation, play=true)
#setanimation!(mvis, sol; realtime_rate = 1.0);