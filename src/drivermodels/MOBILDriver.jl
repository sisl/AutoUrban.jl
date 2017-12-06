import AutomotiveDrivingModels.observe!

mutable struct MOBILDriver <: LaneChangeModel

    dir::Int
    #rec::SceneRecord
    mlon::LaneFollowingDriver
    safe_decel::Float64 # safe deceleration (positive value)
    politeness::Float64 # politeness factor (suggested p ∈ [0.2,0.5])
    advantage_threshold::Float64 # Δaₜₕ
    max_horizon::Float64

    function MOBILDriver(
        timestep::Float64;
        #rec::SceneRecord=SceneRecord(2,timestep),
        mlon::LaneFollowingDriver=IDMDriver(),
        safe_decel::Float64=2.0, # [m/s²]
        politeness::Float64=0.35,
        advantage_threshold::Float64=0.1,
        max_horizon::Float64=20.0,
        )

        retval = new()
        retval.dir = DIR_MIDDLE
        #retval.rec = rec
        retval.mlon = mlon
        retval.safe_decel = safe_decel
        retval.politeness = politeness
        retval.advantage_threshold = advantage_threshold
        retval.max_horizon = max_horizon
        retval
    end
end
get_name(::MOBILDriver) = "MOBILDriver"
function set_desired_speed!(model::MOBILDriver, v_des::Float64)
    set_desired_speed!(model.mlon, v_des)
    model
end
function AutomotiveDrivingModels.observe!(model::MOBILDriver, scene::Scene, roadway::Roadway, egoid::Int)
    observe_helper!(model,scene,roadway,egoid)
end
function AutomotiveDrivingModels.observe!(model::MOBILDriver, scene::Frame{Entity{VehicleState, BicycleModel, Int}}, roadway::Roadway, egoid::Int)
    observe_helper!(model,scene,roadway,egoid)
end


function observe_helper!(model::MOBILDriver, scene::Union{Scene,Frame{Entity{VehicleState, BicycleModel, Int}}}, roadway::Roadway, egoid::Int)
    #rec = model.rec
    #update!(rec, scene)

    #vehicle_index = findfirst(rec[0], egoid)
    vehicle_index = egoid
    veh_ego = scene[vehicle_index]
    v = veh_ego.state.v
    egostate_M = veh_ego.state

    #left_lane_exists = convert(Float64, get(N_LANE_LEFT, rec, roadway, vehicle_index)) > 0
    #right_lane_exists = convert(Float64, get(N_LANE_RIGHT, rec, roadway, vehicle_index)) > 0
    lane = roadway[scene[egoid].state.posF.roadind.tag]
    left_lane_exists = (n_lanes_left(lane, roadway) > 0 )
    right_lane_exists = (n_lanes_right(lane, roadway) > 0 )
    fore_M = get_neighbor_fore_along_lane(scene, vehicle_index, roadway, VehicleTargetPointFront(), VehicleTargetPointRear(), VehicleTargetPointFront(),max_distance_fore=retval.max_horizon)
    rear_M = get_neighbor_rear_along_lane(scene, vehicle_index, roadway, VehicleTargetPointFront(), VehicleTargetPointFront(), VehicleTargetPointRear(),max_distance_rear=retval.max_horizon)

    # accel if we do not make a lane change
    accel_M_orig = rand(observe!(reset_hidden_state!(model.mlon), scene, roadway, egoid)).a
    model.dir = DIR_MIDDLE

    advantage_threshold = model.advantage_threshold
    if right_lane_exists

        rear_R = get_neighbor_rear_along_right_lane(scene, vehicle_index, roadway, VehicleTargetPointFront(), VehicleTargetPointFront(), VehicleTargetPointRear(),max_distance_rear=retval.max_horizon)

        # candidate position after lane change is over
        footpoint = get_footpoint(convert2vehicle(veh_ego))
        lane = roadway[veh_ego.state.posF.roadind.tag]
        lane_R = roadway[LaneTag(lane.tag.segment, lane.tag.lane - 1)]
        roadproj = proj(footpoint, lane_R, roadway)
        frenet_R = Frenet(RoadIndex(roadproj), roadway)
        egostate_R = VehicleState(frenet_R, roadway, veh_ego.state.v)

        Δaccel_n = 0.0
        passes_safety_criterion = true
        if rear_R.ind != 0
            id = scene[rear_R.ind].id
            accel_n_orig = rand(observe!(reset_hidden_state!(model.mlon), scene, roadway, id)).a
            veh_ego = Entity(veh_ego, egostate_R)
            scene[vehicle_index] = veh_ego
            accel_n_test = rand(observe!(reset_hidden_state!(model.mlon), scene, roadway, id)).a

            # body = inertial2body(get_rear(convert2vehicle(scene[rear_R.ind])), get_front(convert2vehicle(veh_ego))) # project target to be relative to ego
            body = inertial2body(get_rear(convert2vehicle(veh_ego)), get_front(convert2vehicle(scene[rear_R.ind])))
            s_gap = body.x

            veh_ego = Entity(veh_ego, egostate_M)
            scene[vehicle_index] = veh_ego
            passes_safety_criterion = accel_n_test ≥ -model.safe_decel && s_gap ≥ 0
            Δaccel_n = accel_n_test - accel_n_orig
        end

        if passes_safety_criterion

            Δaccel_o = 0.0
            if rear_M.ind != 0
                id = scene[rear_M.ind].id
                accel_o_orig = rand(observe!(reset_hidden_state!(model.mlon), scene, roadway, id)).a
                veh_ego = Entity(veh_ego, egostate_R)
                scene[vehicle_index] = veh_ego
                accel_o_test = rand(observe!(reset_hidden_state!(model.mlon), scene, roadway, id)).a
                veh_ego = Entity(veh_ego, egostate_M)
                scene[vehicle_index] = veh_ego
                Δaccel_o = accel_o_test - accel_o_orig
            end

            veh_ego = Entity(veh_ego, egostate_R)
            scene[vehicle_index] = veh_ego
            accel_M_test = rand(observe!(reset_hidden_state!(model.mlon), scene, roadway, egoid)).a
            veh_ego = Entity(veh_ego, egostate_M)
            scene[vehicle_index] = veh_ego
            Δaccel_M = accel_M_test - accel_M_orig

            Δaₜₕ = Δaccel_M + model.politeness*(Δaccel_n + Δaccel_o)
            if Δaₜₕ > advantage_threshold
                model.dir = DIR_RIGHT
                advantage_threshold = Δaₜₕ
            end
        end
    end

    if left_lane_exists
        rear_L = get_neighbor_rear_along_left_lane(scene, vehicle_index, roadway, VehicleTargetPointFront(), VehicleTargetPointFront(), VehicleTargetPointRear(),max_distance_rear=retval.max_horizon)

        # candidate position after lane change is over
        footpoint = get_footpoint(convert2vehicle(veh_ego))
        lane = roadway[veh_ego.state.posF.roadind.tag]
        lane_L = roadway[LaneTag(lane.tag.segment, lane.tag.lane + 1)]
        roadproj = proj(footpoint, lane_L, roadway)
        frenet_L = Frenet(RoadIndex(roadproj), roadway)
        egostate_L = VehicleState(frenet_L, roadway, veh_ego.state.v)

        Δaccel_n = 0.0
        passes_safety_criterion = true
        if rear_L.ind != 0
            id = scene[rear_L.ind].id
            accel_n_orig = rand(observe!(reset_hidden_state!(model.mlon), scene, roadway, id)).a
            veh_ego = Entity(veh_ego, egostate_L)
            scene[vehicle_index] = veh_ego
            accel_n_test = rand(observe!(reset_hidden_state!(model.mlon), scene, roadway, id)).a
            # body = inertial2body(get_rear(convert2vehicle(scene[rear_L.ind])), get_front(convert2vehicle(veh_ego))) # project target to be relative to ego
            body = inertial2body(get_rear(convert2vehicle(veh_ego)), get_front(convert2vehicle(scene[rear_L.ind])))
            s_gap = body.x
            veh_ego = Entity(veh_ego, egostate_M)
            scene[vehicle_index] = veh_ego
            passes_safety_criterion = accel_n_test ≥ -model.safe_decel && s_gap ≥ 0
            Δaccel_n = accel_n_test - accel_n_orig
        end

        if passes_safety_criterion


            Δaccel_o = 0.0
            if rear_M.ind != 0
                id = scene[rear_M.ind].id
                accel_o_orig = rand(observe!(reset_hidden_state!(model.mlon), scene, roadway, id)).a
                veh_ego = Entity(veh_ego, egostate_L)
                scene[vehicle_index] = veh_ego
                accel_o_test = rand(observe!(reset_hidden_state!(model.mlon), scene, roadway, id)).a
                veh_ego = Entity(veh_ego, egostate_M)
                scene[vehicle_index] = veh_ego
                Δaccel_o = accel_o_test - accel_o_orig
            end

            veh_ego = Entity(veh_ego, egostate_L)
            scene[vehicle_index] = veh_ego
            accel_M_test = rand(observe!(reset_hidden_state!(model.mlon), scene, roadway, egoid)).a
            veh_ego = Entity(veh_ego, egostate_M)
            scene[vehicle_index] = veh_ego
            Δaccel_M = accel_M_test - accel_M_orig

            Δaₜₕ = Δaccel_M + model.politeness*(Δaccel_n + Δaccel_o)
            if Δaₜₕ > advantage_threshold
                model.dir = DIR_LEFT
                advantage_threshold = Δaₜₕ
            end
        end
    end

    # chack if front vehicle is doing the same action
    fore = get_neighbor_fore_along_lane(scene, vehicle_index, roadway, VehicleTargetPointFront(), VehicleTargetPointRear(), VehicleTargetPointFront(),max_distance_fore=retval.max_horizon)
    if fore.ind != 0
        if model.dir == DIR_LEFT && scene[fore.ind].state.posF.t > 0.0 && sin(scene[fore.ind].state.posF.ϕ) > 0.0
            model.dir == DIR_MIDDLE
        elseif model.dir == DIR_RIGHT && scene[fore.ind].state.posF.t < 0.0 && sin(scene[fore.ind].state.posF.ϕ) < 0.0
            model.dir == DIR_MIDDLE
        end
    end

    model
end

Base.rand(model::MOBILDriver) = LaneChangeChoice(model.dir)