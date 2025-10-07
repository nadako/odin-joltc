package main

import "base:runtime"
import "core:fmt"
import "core:log"
import "../../jolt"

g_context: runtime.Context

LAYER_NON_MOVING : jolt.ObjectLayer : 0
LAYER_MOVING : jolt.ObjectLayer : 1

BROAD_LAYER_NON_MOVING : jolt.BroadPhaseLayer : 0
BROAD_LAYER_MOVING : jolt.BroadPhaseLayer : 1

main :: proc() {
    context.logger = log.create_console_logger()
    g_context = context

    jolt.Init()
    defer jolt.Shutdown()

    jolt.SetTraceHandler(proc "c" (message:cstring) {
        context = g_context
        log.debugf("JOLT: %v", message)
    })

    job_system := jolt.JobSystemThreadPool_Create(nil)
    defer jolt.JobSystem_Destroy(job_system)

	// We use only 2 layers: one for non-moving objects and one for moving objects
	object_layer_pair_filter := jolt.ObjectLayerPairFilterTable_Create(2)
	jolt.ObjectLayerPairFilterTable_EnableCollision(object_layer_pair_filter, LAYER_NON_MOVING, LAYER_MOVING)
	jolt.ObjectLayerPairFilterTable_EnableCollision(object_layer_pair_filter, LAYER_MOVING, LAYER_NON_MOVING)

	// We use a 1-to-1 mapping between object layers and broadphase layers
	broad_phase_layer_interface := jolt.BroadPhaseLayerInterfaceTable_Create(2, 2)
	jolt.BroadPhaseLayerInterfaceTable_MapObjectToBroadPhaseLayer(broad_phase_layer_interface, LAYER_NON_MOVING, BROAD_LAYER_NON_MOVING)
	jolt.BroadPhaseLayerInterfaceTable_MapObjectToBroadPhaseLayer(broad_phase_layer_interface, LAYER_MOVING, BROAD_LAYER_MOVING)

	object_vs_broad_phase_layer_filter := jolt.ObjectVsBroadPhaseLayerFilterTable_Create(broad_phase_layer_interface, 2, object_layer_pair_filter, 2)

	system := jolt.PhysicsSystem_Create(&{
        maxBodies = 65536,
        numBodyMutexes = 0,
        maxBodyPairs = 65536,
        maxContactConstraints = 65536,
        broadPhaseLayerInterface = broad_phase_layer_interface,
        objectLayerPairFilter = object_layer_pair_filter,
        objectVsBroadPhaseLayerFilter = object_vs_broad_phase_layer_filter,
    })
    defer jolt.PhysicsSystem_Destroy(system)
	body_interface := jolt.PhysicsSystem_GetBodyInterface(system)

	floor_id: jolt.BodyID
	{
		// Next we can create a rigid body to serve as the floor, we make a large box
		// Create the settings for the collision volume (the shape).
		// Note that for simple shapes (like boxes) you can also directly construct a BoxShape.
		box_half_extents := jolt.Vec3 { 100.0, 1.0, 100.0 }
		floor_shape := jolt.BoxShape_Create(&box_half_extents, jolt.DEFAULT_CONVEX_RADIUS)

		floor_position := jolt.Vec3 { 0.0, -1.0, 0.0 }
		floor_settings := jolt.BodyCreationSettings_Create3(
			auto_cast floor_shape,
			&floor_position,
			nil, // Identity,
			.Static,
			LAYER_NON_MOVING)

		// Create the actual rigid body
		floor_id = jolt.BodyInterface_CreateAndAddBody(body_interface, floor_settings, .DontActivate)
		jolt.BodyCreationSettings_Destroy(floor_settings)
	}

	// Sphere
	sphere_id: jolt.BodyID
	{
		sphere_shape := jolt.SphereShape_Create(50.0)
		sphere_position := jolt.Vec3 { 0.0, 2.0, 0.0 }
		sphere_settings := jolt.BodyCreationSettings_Create3(
			auto_cast sphere_shape,
			&sphere_position,
			nil, // Identity,
			.Dynamic,
			LAYER_MOVING)

		sphere_id = jolt.BodyInterface_CreateAndAddBody(body_interface, sphere_settings, .Activate)
		jolt.BodyCreationSettings_Destroy(sphere_settings)
	}


	// Now you can interact with the dynamic body, in this case we're going to give it a velocity.
	// (note that if we had used CreateBody then we could have set the velocity straight on the body before adding it to the physics system)
	sphere_linear_velocity := jolt.Vec3 { 0.0, -5.0, 0.0 }
	jolt.BodyInterface_SetLinearVelocity(body_interface, sphere_id, &sphere_linear_velocity)

	{
		CHARACTER_HEIGHT_STANDING :: 1.35
		CHARACTER_RADIUS_STANDING :: 0.3
		CHARACTER_HEIGHT_CROUCHING :: 0.8
		CHARACTER_RADIUS_CROUCHING :: 0.3
		INNER_SHAPE_FRACTION :: 0.9

		capsule_shape := jolt.CapsuleShape_Create(0.5 * CHARACTER_HEIGHT_STANDING, CHARACTER_RADIUS_STANDING)
		position := jolt.Vec3 { 0, 0.5 * CHARACTER_HEIGHT_STANDING + CHARACTER_RADIUS_STANDING, 0 }
		standing_shape := jolt.RotatedTranslatedShape_Create(&position, nil, auto_cast capsule_shape)

		character_settings: jolt.CharacterVirtualSettings
		jolt.CharacterVirtualSettings_Init(&character_settings)
		character_settings.base.shape = auto_cast standing_shape
		character_settings.base.supportingVolume = { {0, 1, 0}, -CHARACTER_RADIUS_STANDING } // Accept contacts that touch the lower sphere of the capsule
		character_virtual_position := jolt.Vec3 { -5.0, 0, 3.0 }

		animated_character_virtual := jolt.CharacterVirtual_Create(&character_settings, &character_virtual_position, nil, 0, system);
	}

	joint_settings: jolt.SixDOFConstraintSettings
	jolt.SixDOFConstraintSettings_Init(&joint_settings)

	// Optional step: Before starting the physics simulation you can optimize the broad phase. This improves collision detection performance (it's pointless here because we only have 2 bodies).
	// You should definitely not call this every frame or when e.g. streaming in a new level section as it is an expensive operation.
	// Instead insert all new objects in batches instead of 1 at a time to keep the broad phase efficient.
	jolt.PhysicsSystem_OptimizeBroadPhase(system)

	// Now we're ready to simulate the body, keep simulating until it goes to sleep
	step := 0
	for jolt.BodyInterface_IsActive(body_interface, sphere_id)
	{
		// Next step
		step += 1

		// Output current position and velocity of the sphere
		position: jolt.Vec3
		velocity: jolt.Vec3

		jolt.BodyInterface_GetCenterOfMassPosition(body_interface, sphere_id, &position)
		jolt.BodyInterface_GetLinearVelocity(body_interface, sphere_id, &velocity)
        fmt.printfln("Step %v: Position = (%v), Velocity = (%v)", step, position, velocity)

		// If you take larger steps than 1 / 60th of a second you need to do multiple collision steps in order to keep the simulation stable. Do 1 collision step per 1 / 60th of a second (round up).
		collision_steps :: 1

        // We simulate the physics world in discrete time steps. 60 Hz is a good rate to update the physics system.
        delta_time :: 1.0 / 60.0

		// Step the world
		jolt.PhysicsSystem_Update(system, delta_time, collision_steps, job_system)
	}

	// Remove the destroy sphere from the physics system. Note that the sphere itself keeps all of its state and can be re-added at any time.
	jolt.BodyInterface_RemoveAndDestroyBody(body_interface, sphere_id)

	// Remove and destroy the floor
	jolt.BodyInterface_RemoveAndDestroyBody(body_interface, floor_id)
}