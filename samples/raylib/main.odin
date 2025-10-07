package main

import "base:runtime"
import "core:math/linalg"
import "core:log"
import rl "vendor:raylib"
import rlgl "vendor:raylib/rlgl"
import jolt "../../jolt"

g_context: runtime.Context
g_body_iface: ^jolt.BodyInterface

PHYS_LAYER_MOVING :: jolt.ObjectLayer(0)
PHYS_LAYER_NON_MOVING :: jolt.ObjectLayer(1)
PHYS_BROAD_LAYER_MOVING :: jolt.BroadPhaseLayer(0)
PHYS_BROAD_LAYER_NON_MOVING :: jolt.BroadPhaseLayer(1)

Vec3 :: [3]f32
Quat :: quaternion128

QUAT_IDENTITY: Quat = 1
VEC3_ZERO: Vec3 = 0

Box :: struct {
    // for interpolating physics
    prev_position: Vec3,
    prev_rotation: Quat,

    position: Vec3,
    rotation: Quat,
    extent: Vec3,
    color: rl.Color,
    body: ^jolt.Body,
}

boxes: [dynamic]Box

add_box :: proc(box: Box) {
    box := box

    box_shape := jolt.BoxShape_Create(&box.extent, 0)
    defer jolt.Shape_Destroy(auto_cast box_shape)
    box_settings := jolt.BodyCreationSettings_Create3(
        shape = auto_cast box_shape,
        position = &box.position,
        rotation = &box.rotation,
        motionType = .Dynamic,
        objectLayer = PHYS_LAYER_MOVING,
    )
    box.body = jolt.BodyInterface_CreateBody(g_body_iface, box_settings)
    jolt.BodyInterface_AddBody(g_body_iface, jolt.Body_GetID(box.body), .Activate)
    jolt.BodyCreationSettings_Destroy(box_settings)

    box.prev_position = box.position
    box.prev_rotation = box.rotation

    append(&boxes, box)
}

add_floor :: proc() {
    floor_extent := Vec3 {100, 0.05, 100}
    floot_position := Vec3 {0, -0.05, 0}
    floor_shape := jolt.BoxShape_Create(&floor_extent, 0)
    defer jolt.Shape_Destroy(auto_cast floor_shape)
    floor_settings := jolt.BodyCreationSettings_Create3(
        shape = auto_cast floor_shape,
        position = &floot_position,
        rotation = &QUAT_IDENTITY,
        motionType = .Static,
        objectLayer = PHYS_LAYER_NON_MOVING,
    )
    floor := jolt.BodyInterface_CreateAndAddBody(g_body_iface, floor_settings, .Activate)
    jolt.BodyCreationSettings_Destroy(floor_settings)
}

main :: proc() {
    context.logger = log.create_console_logger()
    g_context = context

    ok := jolt.Init(); assert(ok, "Failed to init Jolt Physics")
    defer jolt.Shutdown()

    jolt.SetTraceHandler(proc "c" (message: cstring) {
        context = g_context
        log.debugf("JOLT: %v", message)
    })

    jolt_job_system := jolt.JobSystemThreadPool_Create(nil)
    defer jolt.JobSystem_Destroy(jolt_job_system)

    object_layer_pair_filter := jolt.ObjectLayerPairFilterTable_Create(2)
    jolt.ObjectLayerPairFilterTable_EnableCollision(object_layer_pair_filter, PHYS_LAYER_MOVING, PHYS_LAYER_NON_MOVING)
    jolt.ObjectLayerPairFilterTable_EnableCollision(object_layer_pair_filter, PHYS_LAYER_MOVING, PHYS_LAYER_MOVING)

    broad_phase_layer_interface := jolt.BroadPhaseLayerInterfaceTable_Create(2, 2)
    jolt.BroadPhaseLayerInterfaceTable_MapObjectToBroadPhaseLayer(broad_phase_layer_interface, PHYS_LAYER_MOVING, PHYS_BROAD_LAYER_MOVING)
    jolt.BroadPhaseLayerInterfaceTable_MapObjectToBroadPhaseLayer(broad_phase_layer_interface, PHYS_LAYER_NON_MOVING, PHYS_BROAD_LAYER_NON_MOVING)

    object_vs_broad_phase_layer_filter := jolt.ObjectVsBroadPhaseLayerFilterTable_Create(broad_phase_layer_interface, 2, object_layer_pair_filter, 2)

    physics_system := jolt.PhysicsSystem_Create(&{
        maxBodies = 65536,
        numBodyMutexes = 0,
        maxBodyPairs = 65536,
        maxContactConstraints = 65536,
        broadPhaseLayerInterface = broad_phase_layer_interface,
        objectLayerPairFilter = object_layer_pair_filter,
        objectVsBroadPhaseLayerFilter = object_vs_broad_phase_layer_filter,
    })
    defer jolt.PhysicsSystem_Destroy(physics_system)

    g_body_iface = jolt.PhysicsSystem_GetBodyInterface(physics_system)

    add_floor()
    add_box({ position = { 0,    0.75, -3   }, extent = 0.75, rotation = 1, color = rl.RED })
    add_box({ position = { 0.75, 2.5,  -3   }, extent = 0.5,  rotation = 1, color = rl.BLUE })
    add_box({ position = { 0.25, 5,    -2.5 }, extent = 0.25, rotation = 1, color = rl.GREEN })
    add_box({ position = { 0.25, 5,    -2.5 }, extent = {1, 0.25, 0.5}, rotation = 1, color = rl.PURPLE })

    rl.InitWindow(1200, 900, "Jolt")
    rl.DisableCursor()

    camera := rl.Camera3D {
        projection = .PERSPECTIVE,
        fovy = 80,
        up = {0, 1, 0},
        position = {0, 2, 2},
        target = {0, 2, -1},
    }

	fixed_update_accumulator: f32
	fixed_step: f32 = 1.0 / 30.0

    for !rl.WindowShouldClose() {
        dt := rl.GetFrameTime()

        fixed_update_accumulator += dt
		for fixed_update_accumulator >= fixed_step {
			fixed_update_accumulator -= fixed_step
            jolt.PhysicsSystem_Update(physics_system, fixed_step, 1, jolt_job_system)

            for &box in boxes {
                box.prev_position = box.position
                box.prev_rotation = box.rotation
                jolt.Body_GetPosition(box.body, &box.position)
                jolt.Body_GetRotation(box.body, &box.rotation)
            }

        }
		fixed_interpolation_delta := fixed_update_accumulator / fixed_step

        rl.UpdateCamera(&camera, .FIRST_PERSON)

        rl.BeginDrawing()
        rl.ClearBackground(rl.WHITE)

        rl.BeginMode3D(camera)
        rl.DrawGrid(100, 1)

        for &box in boxes {
            pos := linalg.lerp(box.prev_position, box.position, fixed_interpolation_delta)
            rot := linalg.quaternion_slerp(box.prev_rotation, box.rotation, fixed_interpolation_delta)
            angle, axis := linalg.angle_axis_from_quaternion(rot)

            rlgl.PushMatrix()
            rlgl.Translatef(pos.x, pos.y, pos.z)
            rlgl.Rotatef(linalg.to_degrees(angle), axis.x, axis.y, axis.z)
            rl.DrawCubeV(0, box.extent * 2, box.color)
            rl.DrawCubeWiresV(0, box.extent * 2, rl.BLACK)
            rlgl.PopMatrix()
        }

        rl.EndMode3D()

        rl.EndDrawing()
    }
}