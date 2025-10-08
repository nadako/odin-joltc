package main

import "base:runtime"
import "core:math"
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
    body_id: jolt.BodyID,
}

boxes: [dynamic]Box

CHARACTER_CAPSULE_HALF_HEIGHT : f32 : 1
CHARACTER_CAPSULE_RADIUS : f32 : 0.3

Character :: struct {
    up: Vec3,
    prev_position: Vec3,
    position: Vec3,
    look_yaw: f32,
    look_pitch: f32,
}

character: Character

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
    box.body_id = jolt.BodyInterface_CreateAndAddBody(g_body_iface, box_settings, .Activate)
    jolt.BodyCreationSettings_Destroy(box_settings)

    box.prev_position = box.position
    box.prev_rotation = box.rotation

    append(&boxes, box)
}

g_floor_body_id: jolt.BodyID
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
    g_floor_body_id = jolt.BodyInterface_CreateAndAddBody(g_body_iface, floor_settings, .Activate)
    jolt.BodyCreationSettings_Destroy(floor_settings)
}

add_character :: proc(physics_system: ^jolt.PhysicsSystem) -> ^jolt.CharacterVirtual {
    // capsule shape with origin at the bottom
    capsule_shape := jolt.RotatedTranslatedShape_Create(
        position = &{ 0, CHARACTER_CAPSULE_HALF_HEIGHT, 0 },
        rotation = &QUAT_IDENTITY,
        shape = auto_cast jolt.CapsuleShape_Create(CHARACTER_CAPSULE_HALF_HEIGHT, CHARACTER_CAPSULE_RADIUS)
    )

    settings: jolt.CharacterVirtualSettings; jolt.CharacterVirtualSettings_Init(&settings)
    settings.base.shape = auto_cast capsule_shape
    settings.innerBodyShape = auto_cast capsule_shape // "inner shape" that actually participates in physics (e.g. reacts to raycast and stuff)

    character := jolt.CharacterVirtual_Create(&settings, &VEC3_ZERO, &QUAT_IDENTITY, 0, physics_system)

    // use static var so the pointers survive
    @static listener_procs: jolt.CharacterContactListener_Procs
    listener_procs = {
        OnContactAdded = proc "c" (context_ptr: rawptr, character: ^jolt.CharacterVirtual, other_body_id: jolt.BodyID, _: jolt.SubShapeID, contact_point: ^Vec3, contact_normal: ^Vec3, contact_settings: ^jolt.CharacterContactSettings) {
            if other_body_id == g_floor_body_id do return

            context = (cast(^runtime.Context)context_ptr)^

            log.debugf("Contact added: %v", other_body_id)
        },
        OnContactPersisted = proc "c" (context_ptr: rawptr, character: ^jolt.CharacterVirtual, other_body_id: jolt.BodyID, _: jolt.SubShapeID, contact_point: ^Vec3, contact_normal: ^Vec3, contact_settings: ^jolt.CharacterContactSettings) {
            if other_body_id == g_floor_body_id do return

            context = (cast(^runtime.Context)context_ptr)^

            log.debugf("Contact persisted: %v", other_body_id)
        },
        OnContactRemoved = proc "c" (context_ptr: rawptr, character: ^jolt.CharacterVirtual, other_body_id: jolt.BodyID, _: jolt.SubShapeID) {
            if other_body_id == g_floor_body_id do return

            context = (cast(^runtime.Context)context_ptr)^

            log.debugf("Contact removed: %v", other_body_id)
        },
    }

    listener := jolt.CharacterContactListener_Create(&g_context)
    jolt.CharacterContactListener_SetProcs(&listener_procs)
    jolt.CharacterVirtual_SetListener(character, listener)

    return character
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
    physics_character := add_character(physics_system)

    rl.InitWindow(1200, 900, "Jolt")
    rl.DisableCursor()

    camera := rl.Camera3D {
        projection = .PERSPECTIVE,
        fovy = 80,
    }

	fixed_update_accumulator: f32
	fixed_step: f32 = 1.0 / 30.0

    MOVE_SPEED :: 10
    AIR_MOVE_SPEED :: 0.5
    AIR_DRAG :: 0.9
    JUMP_SPEED :: 5
    LOOK_SENSITIVITY :: 0.2
    CAMERA_DISTANCE :: 3

    jump_requested: bool
    for !rl.WindowShouldClose() {
        dt := rl.GetFrameTime()

        look_input := rl.GetMouseDelta() * LOOK_SENSITIVITY
        character.look_yaw = math.wrap(character.look_yaw - look_input.x, 360)
        character.look_pitch = math.clamp(character.look_pitch - look_input.y, -89, 89)
        look_quat := linalg.quaternion_from_pitch_yaw_roll(linalg.to_radians(character.look_pitch), linalg.to_radians(character.look_yaw), 0)

        move_input: Vec3
        if rl.IsKeyDown(.W) do move_input.z = -1
        else if rl.IsKeyDown(.S) do move_input.z = 1
        if rl.IsKeyDown(.A) do move_input.x = -1
        else if rl.IsKeyDown(.D) do move_input.x = 1
        move_input = linalg.normalize0(move_input)
        if !jump_requested do jump_requested = rl.IsKeyPressed(.SPACE)

        fixed_update_accumulator += dt
		for fixed_update_accumulator >= fixed_step {
			fixed_update_accumulator -= fixed_step

            // update player character with custom logic, since jolt's Character(Virtual) is not a normal body
            // this code is mostly ported from Jolt's own CharacterVirtualTest (but made more stupid)
            {
                character.prev_position = character.position
                jump_pressed := jump_requested
                jump_requested = false

                // get up vector (and update it in the character struct just in case)
                up: Vec3; jolt.CharacterBase_GetUp(auto_cast physics_character, &up)
                character.up = up

                // A cheaper way to update the character's ground velocity, the platforms that the character is standing on may have changed velocity
                jolt.CharacterVirtual_UpdateGroundVelocity(physics_character)
                ground_velocity: Vec3; jolt.CharacterBase_GetGroundVelocity(auto_cast physics_character, &ground_velocity)

                current_velocity: Vec3; jolt.CharacterVirtual_GetLinearVelocity(physics_character, &current_velocity)
                current_vertical_velocity := linalg.dot(current_velocity, up) * up

                new_velocity: Vec3
                if jolt.CharacterBase_GetGroundState(auto_cast physics_character) == .OnGround {
            		// Assume velocity of ground when on ground
                    new_velocity = ground_velocity

                    // Jump
                    moving_towards_ground := (current_vertical_velocity.y - ground_velocity.y) < 0.1
                    if jump_pressed && moving_towards_ground {
                        new_velocity += JUMP_SPEED * up
                    }
                } else {
                    new_velocity = current_vertical_velocity
                }

                // Add gravity
                gravity: Vec3; jolt.PhysicsSystem_GetGravity(physics_system, &gravity)
                new_velocity += gravity * fixed_step

                // add rotated horizontal movement input
                input := linalg.mul(look_quat, move_input)
                input.y = 0
                input = linalg.normalize0(input)

                if jolt.CharacterBase_IsSupported(auto_cast physics_character) {
                    new_velocity += input * MOVE_SPEED
                } else {
                    // preserve horizontal velocity
                    current_horizontal_velocity := current_velocity - current_vertical_velocity
                    new_velocity += current_horizontal_velocity * AIR_DRAG
                    new_velocity += input * AIR_MOVE_SPEED
                }

                // set the velocity to the character
                jolt.CharacterVirtual_SetLinearVelocity(physics_character, &new_velocity)

                // update the character physics (btw there's also CharacterVirtual_ExtendedUpdate with stairs support)
                jolt.CharacterVirtual_Update(physics_character, fixed_step, PHYS_LAYER_MOVING, physics_system, nil, nil)

                // read the new position into our structure
                jolt.CharacterVirtual_GetPosition(physics_character, &character.position)

                // if we're on the ground, try pushing currect contacts away
                if jolt.CharacterBase_GetGroundState(auto_cast physics_character) == .OnGround {
                    for i in 0..<jolt.CharacterVirtual_GetNumActiveContacts(physics_character) {
                        contact:jolt.CharacterVirtualContact; jolt.CharacterVirtual_GetActiveContact(physics_character, i, &contact)
                        if contact.bodyB == g_floor_body_id do continue
                        if contact.motionTypeB == .Dynamic {
                            PUSH_FORCE :: 100
                            push_vector := -contact.contactNormal * PUSH_FORCE
                            jolt.BodyInterface_AddImpulse2(g_body_iface, contact.bodyB, &push_vector, &contact.position)
                        }
                    }
                }
            }

            // update normal physics
            jolt.PhysicsSystem_Update(physics_system, fixed_step, 1, jolt_job_system)

            // read the new transforms for boxes
            for &box in boxes {
                box.prev_position = box.position
                box.prev_rotation = box.rotation
                jolt.BodyInterface_GetPositionAndRotation(g_body_iface, box.body_id, &box.position, &box.rotation)
            }
        }

        // leftover time till the next fixed update step - use for visual interpolation
		fixed_interpolation_delta := fixed_update_accumulator / fixed_step

        character_pos := linalg.lerp(character.prev_position, character.position, fixed_interpolation_delta)

        // update camera to follow the player
        look_target := character_pos + character.up * 2
        camera.position = look_target + linalg.mul(look_quat, Vec3 {0,0,CAMERA_DISTANCE})
        camera.target = look_target
        camera.up = character.up

        rl.BeginDrawing()
        rl.ClearBackground(rl.WHITE)

        rl.BeginMode3D(camera)
        rl.DrawGrid(100, 1)

        // draw interpolated boxes
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

        // draw interpolated character
        rl.DrawCapsule(character_pos, character_pos + character.up * CHARACTER_CAPSULE_HALF_HEIGHT * 2, CHARACTER_CAPSULE_RADIUS, 16, 8, rl.ORANGE)

        rl.EndMode3D()

        rl.EndDrawing()
    }
}