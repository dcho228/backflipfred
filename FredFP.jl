#using Pkg
#Pkg.activate(@__DIR__);

#pkg"instantiate"
#pkg"precompile"

using RigidBodyDynamics, MeshCat, MeshCatMechanisms
using LinearAlgebra, Printf

# load mechanism
#urdfpath="deps/fred/fredFP.urdf"

mechanism= MeshCatMechanisms.parse_urdf("fredFP.urdf";scalar_type=Float64,floating=true)
MeshCatMechanisms.remove_fixed_tree_joints!(mechanism)

# create a half space representing ground
hs_p = Point3D(root_frame(mechanism),0.0,0.0,0.0)
hs_v = FreeVector3D(root_frame(mechanism),0.0,0.0,1.0)
hs = RigidBodyDynamics.Contact.HalfSpace3D(hs_p,hs_v)
ce = RigidBodyDynamics.Contact.ContactEnvironment{Float64}()
# add half space to mechanism
push!(ce.halfspaces,hs)
mechanism.environment = ce

# create a soft contact model conmod3
import RigidBodyDynamics.Contact.ViscoelasticCoulombModel
import RigidBodyDynamics.Contact.HuntCrossleyModel
import RigidBodyDynamics.Contact.SoftContactModel
conmod1 = RigidBodyDynamics.HuntCrossleyModel(50e3,1.5*0.2*50e3,1.5)
conmod2 = RigidBodyDynamics.ViscoelasticCoulombModel(0.5,10e5,10e3)
conmod3 = RigidBodyDynamics.SoftContactModel(conmod1,conmod2)

# create contact points on bodies in the Mechanism
cp1 = RigidBodyDynamics.Contact.ContactPoint(Point3D(default_frame(bodies(mechanism)[7]),0.0,0.0,-1.0),conmod3)
RigidBodyDynamics.add_contact_point!(bodies(mechanism)[7],cp1)
cp1 = RigidBodyDynamics.Contact.ContactPoint(Point3D(default_frame(bodies(mechanism)[8]),0.0,0.0,-1.0),conmod3)
RigidBodyDynamics.add_contact_point!(bodies(mechanism)[8],cp1)
cp1 = RigidBodyDynamics.Contact.ContactPoint(Point3D(default_frame(bodies(mechanism)[9]),0.0,0.0,-1.0),conmod3)
RigidBodyDynamics.add_contact_point!(bodies(mechanism)[9],cp1)
cp1 = RigidBodyDynamics.Contact.ContactPoint(Point3D(default_frame(bodies(mechanism)[10]),0.0,0.0,-1.0),conmod3)
RigidBodyDynamics.add_contact_point!(bodies(mechanism)[10],cp1)

# set the robot "Squat" State
state= MeshCatMechanisms.MechanismState(mechanism)
MeshCatMechanisms.zero_velocity!(state)
frontShoulderSq = pi/2-pi/12;
rearShoulderSq = pi/2-pi/6;
frontElbowSq = pi - 2*pi/12;
rearElbowSq = pi - 2*pi/6;

#1; 0; 0.09034844511447723; 0; 0; 0; -1.3703532738271416
#1; r;        p           ; y; x; y;         z

#MeshCatMechanisms.set_configuration!(state,[1;0;0;0;0;0;0;frontShoulderSq;frontShoulderSq;-rearShoulderSq;-rearShoulderSq;-frontElbowSq;-frontElbowSq;rearElbowSq;rearElbowSq])

MeshCatMechanisms.set_configuration!(state,[1;0;0.09034844511447723;0;0;0;-1.3703532738271416;frontShoulderSq;frontShoulderSq;-rearShoulderSq;-rearShoulderSq;-frontElbowSq;-frontElbowSq;rearElbowSq;rearElbowSq])

# Liftoff Joint Angles
ShoulderLi = pi/2-pi/3;
ElbowLi = pi - 2*pi/3;

# PD control on shoulders and joints
function mytorque!(torques::AbstractVector, t, state::MechanismState)
    desvec = [1;0;0;0;0;0;1.5;ShoulderLi;ShoulderLi;-ShoulderLi;-ShoulderLi;-ElbowLi;-ElbowLi;ElbowLi;ElbowLi]
    #desvec = [1;0;0;0;0;0;0;frontShoulderSq;frontShoulderSq;-rearShoulderSq;-rearShoulderSq;-frontElbowSq;-frontElbowSq;rearElbowSq;rearElbowSq]
    torques .= 0
#=    
    for i=1:8
    torques[6+i]=-50*state.v[6+i] - 2700*(state.q[7+i] - desvec[7+i])
    end
=#    

    
    if t<1.00
         frontGain = 2700;
         rearGain  = 2000;
    torques[7] =-50*state.v[7]  - frontGain*(state.q[8]  - desvec[8] )
    torques[8] =-50*state.v[8]  - frontGain*(state.q[9]  - desvec[9] )
    torques[9] =-50*state.v[9]  - rearGain *(state.q[10] - desvec[10])
    torques[10]=-50*state.v[10] - rearGain *(state.q[11] - desvec[11])
    torques[11]=-50*state.v[11] - frontGain*(state.q[12] - desvec[12])
    torques[12]=-50*state.v[12] - frontGain*(state.q[13] - desvec[13])
    torques[13]=-50*state.v[13] - rearGain *(state.q[14] - desvec[14])
    torques[14]=-50*state.v[14] - rearGain *(state.q[15] - desvec[15])

    else
        frontGain = 60;
        rearGain  = 60;
        dampingGain = 20;
    torques[7] =-dampingGain*state.v[7]  - frontGain*(state.q[8]  - desvec[8] )
    torques[8] =-dampingGain*state.v[8]  - frontGain*(state.q[9]  - desvec[9] )
    torques[9] =-dampingGain*state.v[9]  - rearGain *(state.q[10] - desvec[10])
    torques[10]=-dampingGain*state.v[10] - rearGain *(state.q[11] - desvec[11])
    torques[11]=-dampingGain*state.v[11] - frontGain*(state.q[12] - desvec[12])
    torques[12]=-dampingGain*state.v[12] - frontGain*(state.q[13] - desvec[13])
    torques[13]=-dampingGain*state.v[13] - rearGain *(state.q[14] - desvec[14])
    torques[14]=-dampingGain*state.v[14] - rearGain *(state.q[15] - desvec[15])
    end
    
    
    
   
        
end

# define simulation time
final_time = 5.00

# simulate
ts, qs, vs = MeshCatMechanisms.simulate(state, final_time, mytorque!)


# display
mvis = MeshCatMechanisms.MechanismVisualizer(mechanism, URDFVisuals("fredFP.urdf"))
MeshCatMechanisms.animate(mvis, ts, qs)
anim = MeshCatMechanisms.Animation(mvis, ts, qs)
#MeshCatMechanisms.setanimation!(vis, anim, play=true)
setanimation!(mvis,ts,qs)
MeshCatMechanisms.open(mvis)
