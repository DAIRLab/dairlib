import pydrake.systems.framework
import pydrake.multibody.plant
from pydrake.math import RigidTransform
from pydrake.geometry import (
    ConnectDrakeVisualizer, Capsule, Ellipsoid, SceneGraph, Sphere, HalfSpace)
from pydrake.multibody.plant import CoulombFriction
from pydrake.multibody.tree import (
    UnitInertia, SpatialInertia, PrismaticJoint, RevoluteJoint)
from pydrake.systems.analysis import Simulator
from pydrake.systems.primitives import SignalLogger, Multiplexer
import numpy as np

def AddBin(plant, scene_graph, mu):
    # Add a V-shaped bin, fixed to the world.
    friction = CoulombFriction(mu, mu)
    normal = [np.sqrt(2), 0, np.sqrt(2)]
    body = plant.world_body()
    transform = HalfSpace.MakePose(Hz_dir_F=normal, p_FB=[0, 0, 0])
    plant.RegisterCollisionGeometry(
        body=body, X_BG=transform, shape=HalfSpace(), name="bin1_collision",
        coulomb_friction=friction)
    plant.RegisterVisualGeometry(
        body=body, X_BG=transform, shape=HalfSpace(), name="bin1_visual",
        diffuse_color=[.5, .5, .5, 1])

    normal = [-np.sqrt(2), 0, np.sqrt(2)]
    transform = HalfSpace.MakePose(Hz_dir_F=normal, p_FB=[0, 0, 0])
    plant.RegisterCollisionGeometry(
        body=body, X_BG=transform, shape=HalfSpace(), name="bin2_collision",
        coulomb_friction=friction)
    plant.RegisterVisualGeometry(
        body=body, X_BG=transform, shape=HalfSpace(), name="bin2_visual",
        diffuse_color=[.5, .5, .5, 1])


def AddEllipsoid(plant, scene_graph, mass, a, b, c, mu, rgba, name):
    # Add a planar ellipsoid object to the plant and SceneGraph. While specified in
    # 3D, the y-axis (specified by b) will not matter.
    # @param plant
    # @param scene_graph
    # @param mass
    # @param a,b,c, are the principal semi-axes
    # @param mu is the coefficient of friction
    # @param rgba is the length-4 vector for color and transparency
    # @param name must be a unique string name
    unit_inertia = UnitInertia(
        Ixx=.2 * (b**2 + c**2), Iyy=.2 * (a**2 + c**2), Izz=.2 * (a**2 + b**2))
    spatial_inertia = SpatialInertia(
        mass=mass, p_PScm_E=[0, 0, 0], G_SP_E=unit_inertia)
    body = plant.AddRigidBody(name, spatial_inertia)
    friction = CoulombFriction(mu, mu)
    ellipsoid = Ellipsoid(a, b, c)
    transform = RigidTransform()
    plant.RegisterCollisionGeometry(
        body=body, X_BG=transform, shape=ellipsoid, name=name + "_collision",
        coulomb_friction=friction)
    plant.RegisterVisualGeometry(
        body=body, X_BG=transform, shape=ellipsoid, name=name + "_visual",
        diffuse_color=rgba)

    # Create and attach planar joint
    AddPlanarJoint(plant, scene_graph, body)
    return body


def AddCapsule(plant, scene_graph, mass, radius, length, mu, rgba, name):
    # Add a planar capsule object to the plant and SceneGraph.
    # @param plant
    # @param scene_graph
    # @param mass
    # @param radius
    # @param length
    # @param mu is the coefficient of friction
    # @param rgba is the length-4 vector for color and transparency
    # @param name must be a unique string name
    Ixx = (3 * radius + 2 * length) / 8 * length

    unit_inertia = UnitInertia(
        Ixx=Ixx, Iyy=Ixx, Izz=.4 * radius**2)
    spatial_inertia = SpatialInertia(
        mass=mass, p_PScm_E=[0, 0, 0], G_SP_E=unit_inertia)
    body = plant.AddRigidBody(name, spatial_inertia)
    friction = CoulombFriction(mu, mu)
    capsule = Capsule(radius, length)
    transform = RigidTransform()
    plant.RegisterCollisionGeometry(
        body=body, X_BG=transform, shape=capsule, name=name + "_collision",
        coulomb_friction=friction)
    plant.RegisterVisualGeometry(
        body=body, X_BG=transform, shape=capsule, name=name + "_visual",
        diffuse_color=rgba)

    # Create and attach planar joint
    AddPlanarJoint(plant, scene_graph, body)
    return body


def AddSphere(plant, scene_graph, mass, radius, mu, rgba, name):
    # Add a planar sphere object (effectively a circle) to the plant and SceneGraph.
    # @param plant
    # @param scene_graph
    # @param mass
    # @param radius
    # @param mu is the coefficient of friction
    # @param rgba is the length-4 vector for color and transparency
    # @param name must be a unique string name
    unit_inertia = UnitInertia(
        Ixx=.4 * radius**2, Iyy=.4 * radius**2, Izz=.4 * radius**2)
    spatial_inertia = SpatialInertia(
        mass=mass, p_PScm_E=[0, 0, 0], G_SP_E=unit_inertia)
    body = plant.AddRigidBody(name, spatial_inertia)
    friction = CoulombFriction(mu, mu)
    sphere = Sphere(radius)
    transform = RigidTransform()
    plant.RegisterCollisionGeometry(
        body=body, X_BG=transform, shape=sphere, name=name + "_collision",
        coulomb_friction=friction)
    plant.RegisterVisualGeometry(
        body=body, X_BG=transform, shape=sphere, name=name + "_visual",
        diffuse_color=rgba)

    # Create and attach planar joint
    AddPlanarJoint(plant, scene_graph, body)
    return body


def AddPlanarJoint(plant, scene_graph, body):
    # Add a 3DOF (x,y,theta) planar joint between the body and the world
    # @param plant
    # @param scene_graph
    # @param body
    massless_inertia = SpatialInertia(
        mass=0, p_PScm_E=[0, 0, 0], G_SP_E=UnitInertia(0, 0, 0))
    link_x = plant.AddRigidBody(body.name() + "_link_x", massless_inertia)
    link_xz = plant.AddRigidBody(body.name() + "_link_xz", massless_inertia)
    joint_x = PrismaticJoint(
        body.name() + "joint_x", plant.world_frame(), link_x.body_frame(),
        [1, 0, 0])
    joint_xz = PrismaticJoint(
        body.name() + "joint_xz", link_x.body_frame(), link_xz.body_frame(),
        [0, 0, 1])
    joint_y = RevoluteJoint(
        body.name() + "joint_y", link_xz.body_frame(), body.body_frame(),
        [0, 1, 0])
    plant.AddJoint(joint_x)
    plant.AddJoint(joint_xz)
    plant.AddJoint(joint_y)


def SimulateScene(T, sim_dt, output_dt, circles, ellipses, capsules, x0,
                  visualize):
    # Simulate a number of objects (circles, ellipses, capsules). Note that for
    # input (x0) and output purposes, objects are ordered circles, ellipses, and
    # then capsules.
    # At the moment, use of ellipsoids is not recommended as their contact
    # geometry is not well-implemented.
    # @param T the duration of the simualtion
    # @param sim_dt the timestep of the simuliator
    # @param output_dt the timestep of the logged output
    # @param circles [C x 3] list of circle radii [m], mass [kg], mu
    # @param ellipses [E x 4] list of 2 semi-axes [m], mass [kg],  mu
    # @param capsules [P x 4] list of radii [m], lengths [m], mass [kg],  mu
    # @param x0 [6(C+E+P) x 1] initial state as
    #     (x0,z0,theta0,x1,z1,theta1,...xdot0,zdot0,thetadot0,....)
    # @param visualize If true, will output to Drake Visualizer and try to
    # simulate at 1x speed. If fase, will simulate as fast as possible without
    # visualization.
    # @return t [n x 1] Array of times
    # @return x [6(C+E+P) x n] array of states
    # @return F [3(C+E+P) x n] array of generalized contact forces

    C = circles.shape[0]
    E = ellipses.shape[0]
    P = capsules.shape[0]
    if C > 0:
        assert (circles.shape[1] == 3)
    if E > 0:
        assert (ellipses.shape[1] == 4)
    if P > 0:
        assert(capsules.shape[1] == 4)

    num_objects = C + E + P

    builder = pydrake.systems.framework.DiagramBuilder()
    plant, scene_graph = pydrake.multibody.plant.AddMultibodyPlantSceneGraph(
        builder, sim_dt)

    # Add bin
    AddBin(plant, scene_graph, .6)

    # Add objects
    for i in range(0, C):
        name = "sphere_" + str(i)
        rgba = np.concatenate((np.random.rand(3), np.array([1])))
        AddSphere(plant, scene_graph, mass=circles[i, 1], radius=circles[i, 0],
                  mu=circles[i, 2], rgba=rgba, name=name)
    for i in range(0, E):
        name = "ellipsoid_" + str(i)
        rgba = np.concatenate((np.random.rand(3), np.array([1])))
        a = ellipses[i, 0]
        c = ellipses[i, 2]
        b = min(a, c)
        AddEllipsoid(plant, scene_graph, mass=ellipses[i, 2], a=a, b=b, c=c,
                     mu=ellipses[i, 3], rgba=rgba, name=name)
    for i in range(0, P):
        name = "capsule_" + str(i)
        rgba = np.concatenate((np.random.rand(3), np.array([1])))
        AddCapsule(plant, scene_graph, mass=capsules[i, 2],
                   radius=capsules[i, 0], length=capsules[i, 1],
                   mu=capsules[i, 3], rgba=rgba, name=name)

    plant.Finalize()

    mux = builder.AddSystem(Multiplexer([num_objects * 6, num_objects * 3]))

    logger = builder.AddSystem(SignalLogger(num_objects * 9))
    logger.set_publish_period(output_dt)
    builder.Connect(plant.get_state_output_port(), mux.get_input_port(0))
    builder.Connect(plant.get_generalized_contact_forces_output_port(
        pydrake.multibody.tree.ModelInstanceIndex(1)), mux.get_input_port(1))
    builder.Connect(mux.get_output_port(0), logger.get_input_port(0))

    if visualize:
        ConnectDrakeVisualizer(builder=builder, scene_graph=scene_graph)
    diagram = builder.Build()

    diagram_context = diagram.CreateDefaultContext()
    plant_context = diagram.GetMutableSubsystemContext(plant, diagram_context)

    # Reshape initial state
    # x0 is (x0,z0,theta0,x1,z1,theta1,...xdot0,zdot0,thetadot0,....)
    # but needs to be shaped into
    # (x0,x1,..., z0, z1,...,theta0,...,xdot0,...)
    init_x = x0[0:num_objects*3:3]
    init_z = x0[1:num_objects * 3:3]
    init_theta = x0[2:num_objects * 3:3]
    init_xdot = x0[num_objects * 3::3]
    init_zdot = x0[1+num_objects * 3::3]
    init_thetadot = x0[2 + num_objects * 3::3]
    x0_shaped = np.concatenate((init_x, init_z, init_theta,
                                init_xdot, init_zdot, init_thetadot))
    plant.SetPositionsAndVelocities(plant_context, x0_shaped)

    simulator = Simulator(diagram, diagram_context)
    simulator.set_publish_every_time_step(False)
    if visualize:
        simulator.set_target_realtime_rate(1)
    else:
        simulator.set_target_realtime_rate(1e6)
    simulator.Initialize()
    simulator.AdvanceTo(T)

    t = logger.sample_times()
    data = logger.data()

    # Reshape data
    # state and forces are (x0,x1,..., z0, z1,...,theta0,...,xdot0,...)
    # but should be (x0,z0,theta0,x1,z1,theta1,...xdot0,zdot0,thetadot0,....)
    x = np.ndarray((6 * num_objects, len(t)))
    F = np.ndarray((3 * num_objects, len(t)))
    for i in range(0, num_objects):
        x[i * 3:(i + 1) * 3, :] = data[i:3 * num_objects:num_objects, :]
        x[i * 3 + 3 * num_objects:(i + 1) * 3 + 3 * num_objects,
          :] = data[i+3 * num_objects:6*num_objects:num_objects, :]
        F[i * 3:(i + 1) * 3, :] = data[i+6*num_objects::num_objects, :]
    return t, x, F
