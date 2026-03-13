"""Main SOFA scene assembly for the catheter deployment simulation.

This file now focuses on scene graph construction only.
Configuration parsing, action mappings, and runtime controls are split into
separate modules for readability:
- catheter_config.py
- catheter_actions.py
- catheter_controls.py
"""

import math
import os
from typing import Tuple

import Sofa
import Sofa.Core

from catheter_config import parse_args, scene_dir
from catheter_controls import CatheterControlAndLoggingController, parse_controls_file


def _quat_from_x_axis(direction: Tuple[float, float, float]) -> Tuple[float, float, float, float]:
    """Quaternion that rotates +X onto the given direction vector."""
    tx, ty, tz = direction
    norm = math.sqrt(tx * tx + ty * ty + tz * tz)
    if norm < 1.0e-12:
        return 0.0, 0.0, 0.0, 1.0
    tx /= norm
    ty /= norm
    tz /= norm

    dot = max(-1.0, min(1.0, tx))
    if dot > 1.0 - 1.0e-9:
        return 0.0, 0.0, 0.0, 1.0
    if dot < -1.0 + 1.0e-9:
        return 0.0, 1.0, 0.0, 0.0

    ax = 0.0
    ay = -tz
    az = ty
    an = math.sqrt(ax * ax + ay * ay + az * az)
    ax /= an
    ay /= an
    az /= an
    angle = math.acos(dot)
    s = math.sin(angle / 2.0)
    return ax * s, ay * s, az * s, math.cos(angle / 2.0)


def createScene(rootNode):
    """SOFA entry point used by runSofa."""
    cfg = parse_args()

    # Validate paths early to fail fast with clear messages.
    if cfg.controls_file and not os.path.exists(cfg.controls_file):
        raise FileNotFoundError(f"Controls file does not exist: {cfg.controls_file}")
    if not os.path.exists(cfg.vessel_file):
        raise FileNotFoundError(f"Vessel STL does not exist: {cfg.vessel_file}")

    # ---- Root setup: plugins, solvers, collision pipeline ----
    rootNode.addObject(
        "RequiredPlugin",
        pluginName=[
            "BeamAdapter",
            "Sofa.Component.AnimationLoop",
            "Sofa.Component.Collision.Detection.Algorithm",
            "Sofa.Component.Collision.Detection.Intersection",
            "Sofa.Component.Collision.Geometry",
            "Sofa.Component.Collision.Response.Contact",
            "Sofa.Component.Constraint.Lagrangian.Correction",
            "Sofa.Component.Constraint.Lagrangian.Solver",
            "Sofa.Component.Constraint.Projective",
            "Sofa.Component.IO.Mesh",
            "Sofa.Component.LinearSolver.Direct",
            "Sofa.Component.Mapping.Linear",
            "Sofa.Component.ODESolver.Backward",
            "Sofa.Component.StateContainer",
            "Sofa.Component.Setting",
            "Sofa.Component.SolidMechanics.Spring",
            "Sofa.Component.Topology.Container.Constant",
            "Sofa.Component.Topology.Container.Dynamic",
            "Sofa.Component.Topology.Container.Grid",
            "Sofa.Component.Topology.Mapping",
            "Sofa.GL.Component.Rendering3D",
            "Sofa.Component.Visual",
            "MultiThreading",
        ],
    )

    rootNode.findData("gravity").value = [0.0, 0.0, 0.0]
    rootNode.findData("dt").value = cfg.dt
    rootNode.findData("animate").value = bool(cfg.auto_start_animation)
    rootNode.addObject(
        "VisualStyle",
        displayFlags=(
            "showVisualModels showBehaviorModels hideCollisionModels "
            "hideBoundingCollisionModels hideWireframe"
        ),
    )
    rootNode.addObject("BackgroundSetting", color="0.01 0.01 0.01")

    rootNode.addObject("FreeMotionAnimationLoop")
    rootNode.addObject(
        "LCPConstraintSolver",
        tolerance=1.0e-6,
        maxIt=2000,
        mu=cfg.friction_mu,
        build_lcp=False,
    )
    rootNode.addObject("CollisionPipeline")
    if cfg.use_parallel_collision:
        rootNode.addObject("ParallelBruteForceBroadPhase")
        rootNode.addObject("ParallelBVHNarrowPhase")
    else:
        rootNode.addObject("BruteForceBroadPhase")
        rootNode.addObject("BVHNarrowPhase")

    # Keep LocalMinDistance as a small numerical margin and model physical
    # catheter thickness directly on catheter collision primitives.
    effective_contact_distance = max(0.0, cfg.contact_distance)
    effective_alarm_distance = max(cfg.alarm_distance, effective_contact_distance + cfg.collision_radius + 0.1)
    rootNode.addObject(
        "LocalMinDistance",
        alarmDistance=effective_alarm_distance,
        contactDistance=effective_contact_distance,
        angleCone=0.2,
    )
    rootNode.addObject("CollisionResponse", response="FrictionContactConstraint", responseParams=f"mu={cfg.friction_mu}")

    # ---- Vessel geometry / collision ----
    vessel = rootNode.addChild("Vessel")
    vessel.addObject(
        "MeshSTLLoader",
        name="loader",
        filename=cfg.vessel_file,
        triangulate=True,
    )
    vessel.addObject("MeshTopology", src="@loader")
    vessel.addObject("MechanicalObject", src="@loader")
    # Keep only triangle primitives to avoid static vessel self-contacts.
    vessel_triangle_cm = vessel.addObject(
        "TriangleCollisionModel",
        name="VesselTriangleCM",
        moving=False,
        simulated=False,
        group=2,
        bothSide=True,
        contactDistance=0.0,
    )
    vessel_line_cm = vessel.addObject(
        "LineCollisionModel",
        name="VesselLineCM",
        moving=False,
        simulated=False,
        group=2,
        bothSide=True,
        contactDistance=0.0,
    )
    vessel_point_cm = vessel.addObject(
        "PointCollisionModel",
        name="VesselPointCM",
        moving=False,
        simulated=False,
        group=2,
        bothSide=True,
        contactDistance=0.0,
    )
    if not cfg.headless:
        vessel_visu = vessel.addChild("Visual")
        vessel_visu.addObject("OglModel", src="@../loader", color=f"0.8 0.1 0.1 {cfg.vessel_alpha}")
        vessel_visu.addObject("IdentityMapping")

    # ---- BeamAdapter wire definition ----
    topo = rootNode.addChild("topoLines_cath")
    topo.addObject(
        "RodStraightSection",
        name="shaftSection",
        length=cfg.shaft_length,
        radius=cfg.shaft_radius,
        youngModulus=cfg.shaft_young_modulus,
        poissonRatio=cfg.poisson_ratio,
        massDensity=cfg.mass_density,
        nbBeams=cfg.shaft_nb_beams,
        nbEdgesVisu=cfg.shaft_nb_edges_visu,
        nbEdgesCollis=cfg.shaft_nb_edges_collis,
    )
    topo.addObject(
        "RodSpireSection",
        name="tipSection",
        length=cfg.tip_length,
        radius=cfg.tip_radius,
        youngModulus=cfg.tip_young_modulus,
        poissonRatio=cfg.poisson_ratio,
        massDensity=cfg.mass_density,
        nbBeams=cfg.tip_nb_beams,
        nbEdgesVisu=cfg.tip_nb_edges_visu,
        nbEdgesCollis=cfg.tip_nb_edges_collis,
        spireDiameter=2.0 * cfg.hook_radius,
        spireHeight=0.0,
    )
    topo.addObject(
        "WireRestShape",
        name="catheterRestShape",
        template="Rigid3d",
        wireMaterials="@shaftSection @tipSection",
        printLog=False,
    )
    topo.addObject("EdgeSetTopologyContainer", name="meshLines")
    topo.addObject("EdgeSetTopologyModifier", name="Modifier")
    topo.addObject("EdgeSetGeometryAlgorithms", name="GeomAlgo", template="Rigid3d")
    topo.addObject("MechanicalObject", name="dofTopo2", template="Rigid3d")

    total_length = cfg.shaft_length + cfg.tip_length
    total_beams = cfg.shaft_nb_beams + cfg.tip_nb_beams
    nx = total_beams + 1

    # ---- Simulated instrument state ----
    instrument = rootNode.addChild("InstrumentCombined")
    instrument.addObject("EulerImplicitSolver", rayleighStiffness=0.02, rayleighMass=0.01)
    instrument.addObject("BTDLinearSolver", template="BTDMatrix6d", verification=False)
    instrument.addObject(
        "RegularGridTopology",
        name="grid",
        nx=nx,
        ny=1,
        nz=1,
        xmin=0.0,
        xmax=total_length,
        ymin=0.0,
        ymax=0.0,
        zmin=0.0,
        zmax=0.0,
    )
    instrument.addObject(
        "MechanicalObject",
        name="DOFs",
        template="Rigid3d",
        showIndices=False,
    )
    instrument.addObject(
        "WireBeamInterpolation",
        name="BeamInterpolation",
        WireRestShape="@../topoLines_cath/catheterRestShape",
    )
    instrument.addObject(
        "AdaptiveBeamForceFieldAndMass",
        name="BeamForceField",
        interpolation="@BeamInterpolation",
        massDensity=cfg.mass_density,
    )

    qx, qy, qz, qw = _quat_from_x_axis(cfg.starting_direction)
    sx, sy, sz = cfg.starting_position
    instrument.addObject("FixedProjectiveConstraint", name="FixedConstraint", indices="0")
    instrument.addObject(
        "InterventionalRadiologyController",
        name="m_ircontroller",
        listening=True,
        instruments="BeamInterpolation",
        topology="@grid",
        step=cfg.insertion_step,
        angularStep=cfg.rotation_step,
        speed=max(1.0e-6, cfg.insertion_step),
        controlledInstrument=0,
        xtip=[cfg.initial_xtip],
        rotationInstrument=[0.0],
        startingPos=f"{sx} {sy} {sz} {qx} {qy} {qz} {qw}",
        fixedConstraint="@FixedConstraint",
    )
    instrument.addObject(
        "RestShapeSpringsForceField",
        points="@m_ircontroller.indexFirstNode",
        angularStiffness=1.0e8,
        stiffness=1.0e8,
    )
    instrument.addObject("LinearSolverConstraintCorrection", wire_optimization=True)

    action_controller = instrument.addObject(
        "BeamAdapterActionController",
        name="actionController",
        interventionController="@m_ircontroller",
        listening=False,
        writeMode=False,
    )

    # ---- Catheter collision primitives ----
    catheter_collision = instrument.addChild("CollisionModel")
    catheter_collision.addObject("EdgeSetTopologyContainer", name="collisEdgeSet")
    catheter_collision.addObject("EdgeSetTopologyModifier", name="collisModifier")
    catheter_collision.addObject("MechanicalObject", name="CollisionDOFs")
    catheter_collision.addObject(
        "MultiAdaptiveBeamMapping",
        name="collisMap",
        controller="@../m_ircontroller",
        useCurvAbs=True,
    )
    catheter_collision.addObject("UncoupledConstraintCorrection", defaultCompliance=1.0e-12)
    catheter_line_cm = catheter_collision.addObject(
        "LineCollisionModel",
        name="CatheterLineCM",
        group=1,
        contactDistance=cfg.collision_radius,
    )
    catheter_point_cm = catheter_collision.addObject(
        "PointCollisionModel",
        name="CatheterPointCM",
        group=1,
        contactDistance=cfg.collision_radius,
    )

    # ---- Catheter visual mesh (tube around beam edges) ----
    if not cfg.headless:
        catheter_visu = instrument.addChild("VisualCatheter")
        catheter_visu.addObject("QuadSetTopologyContainer", name="visuContainer")
        catheter_visu.addObject("QuadSetTopologyModifier", name="visuModifier")
        catheter_visu.addObject("MechanicalObject", name="visuDOFs", template="Vec3d")
        catheter_visu.addObject("QuadSetGeometryAlgorithms", name="visuGeomAlgo")
        catheter_visu.addObject(
            "Edge2QuadTopologicalMapping",
            name="visuTopologicalMapping",
            input="@../../topoLines_cath/meshLines",
            output="@visuContainer",
            nbPointsOnEachCircle=10,
            radius=cfg.shaft_radius,
            flipNormals=True,
        )
        catheter_visu.addObject(
            "AdaptiveBeamMapping",
            name="visuBeamMapping",
            interpolation="@../BeamInterpolation",
            input="@../DOFs",
            output="@visuDOFs",
            useCurvAbs=True,
            isMechanical=False,
        )
        catheter_visu_ogl = catheter_visu.addChild("Ogl")
        catheter_visu_ogl.addObject(
            "OglModel",
            name="visuModel",
            quads="@../visuContainer.quads",
            color="0.95 0.95 0.95 1.0",
        )
        catheter_visu_ogl.addObject("IdentityMapping", input="@../visuDOFs", output="@visuModel")

    # Optional prerecorded control script.
    if cfg.controls_file:
        playback_times, playback_actions = parse_controls_file(cfg.controls_file, cfg.dt)
        action_controller.findData("timeSteps").value = playback_times
        action_controller.findData("actions").value = playback_actions
        print(f"Loaded {len(playback_actions)} actions from {cfg.controls_file}")

    # Runtime control + recording + headless contact/tip logging.
    rootNode.addObject(
        CatheterControlAndLoggingController(
            name="CatheterControlAndLoggingController",
            listening=True,
            root=rootNode,
            action_controller=action_controller,
            ircontroller=instrument.getObject("m_ircontroller"),
            tip_collision_mo=catheter_collision.getObject("CollisionDOFs"),
            collision_models=[
                ("vesselTri", vessel_triangle_cm),
                ("vesselLine", vessel_line_cm),
                ("vesselPoint", vessel_point_cm),
                ("cathLine", catheter_line_cm),
                ("cathPoint", catheter_point_cm),
            ],
            record_file=cfg.record_file,
            headless=cfg.headless,
            print_every_steps=cfg.print_every_steps,
            head_direction=cfg.starting_direction,
            enable_control_window=cfg.show_control_window,
            control_state_file=cfg.control_state_file,
            control_window_script=str((scene_dir() / "catheter_control_window.py").resolve()),
        )
    )

    broadphase_mode = "parallel" if cfg.use_parallel_collision else "sequential"
    print(
        "Catheter scene ready."
        f" vessel='{cfg.vessel_file}', headless={cfg.headless},"
        f" catheter='{cfg.catheter_profile_file}', controls='{cfg.controls_file}', record='{cfg.record_file}',"
        f" vesselCollision=double-sided, collisionPipeline={broadphase_mode}"
    )
    return rootNode

