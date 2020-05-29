export UrbanDriver

mutable struct UrbanDriver <: DriverModel{LatLonAccelDirection}
    mlon::LaneFollowingDriver
    mlat::LateralDriverModel
    mlane::LaneChangeModel
    longAcc::Float64
    latAcc::Float64
    direction::Int
    a_cp_max::Float64
    color::Colorant
    
    function UrbanDriver(
        timestep::Float64;
        mlon::LaneFollowingDriver=IDMDriver(),
        mlat::LateralDriverModel=ProportionalLaneTracker(),
        mlane::LaneChangeModel=MOBIL(timestep,mlon=IDMDriver()),
        longAcc::Float64 = 0.0,
        latAcc::Float64 = 0.0,
        direction::Int = 1,
        a_cp_max::Float64 = (rand()*0.4+0.5)*9.8,
        color::Colorant=RGB(0, 1, 0)
        )

        retval = new()
        retval.mlon = mlon
        retval.mlat = mlat
        retval.mlane = mlane
        retval.longAcc = longAcc
        retval.latAcc = latAcc
        retval.direction = direction
        retval.a_cp_max = a_cp_max
        retval.color = color
        retval
    end
end

function set_desired_speed!(model::UrbanDriver, v_des::Float64)
    AutomotiveDrivingModels.set_desired_speed!(model.mlon, v_des)
    AutomotiveDrivingModels.set_desired_speed!(model.mlane, v_des)
    model
end

function track_longitudinal!(driver::LaneFollowingDriver, scene::Union{Scene,Scene{Entity{VehicleState, BicycleModel, Int}}}, roadway::Roadway, vehicle_index::Int, fore::NeighborLongitudinalResult)
    v_ego = scene[vehicle_index].state.v
    if fore.ind != 0
        headway, v_oth = fore.Δs, scene[fore.ind].state.v
    else
        headway, v_oth = NaN, NaN
    end
    return track_longitudinal!(driver, v_ego, v_oth, headway)
end

function AutomotiveDrivingModels.observe!(driver::UrbanDriver, scene2::Union{Scene,Scene{Entity{VehicleState, BicycleModel, Int}}}, roadway::Roadway, egoid::Int)
    scene = Scene()
    for veh in scene2
        push!(scene,Vehicle(veh.state,veh.def.def,veh.id))
    end
    AutomotiveDrivingModels.update!(driver.rec, scene)
    observe!(driver.mlane, scene, roadway, egoid)
    lane_change_action = rand(driver.mlane)
   
    vehicle_index = findfirst(scene, egoid)
    #lane_change_action = LaneChangeChoice(DIR_MIDDLE)
    #currentLane = scene[vehicle_index].state.posF.roadind.tag.lane
    
    laneoffset = get_lane_offset(lane_change_action, driver.rec, roadway, vehicle_index)
    lateral_speed = convert(Float64, get(VELFT, driver.rec, roadway, vehicle_index))

    
    if lane_change_action.dir == DIR_MIDDLE
        fore = get_neighbor_fore_along_lane(scene, vehicle_index, roadway, VehicleTargetPointFront(), VehicleTargetPointRear(), VehicleTargetPointFront())
    elseif lane_change_action.dir == DIR_LEFT
        fore = get_neighbor_fore_along_left_lane(scene, vehicle_index, roadway, VehicleTargetPointFront(), VehicleTargetPointRear(), VehicleTargetPointFront())
    else
        @assert(lane_change_action.dir == DIR_RIGHT)
        fore = get_neighbor_fore_along_right_lane(scene, vehicle_index, roadway, VehicleTargetPointFront(), VehicleTargetPointRear(), VehicleTargetPointFront())
    end

    
    AutomotiveDrivingModels.track_lateral!(driver.mlat, laneoffset, lateral_speed)
    #AutomotiveDrivingModels.track_longitudinal!(driver.mlon, scene, roadway, vehicle_index, fore.ind)
    roadind = scene[vehicle_index].state.posF.roadind
    max_k,distance = get_max_curvature(roadind, roadway, 25.0,direction = driver.direction)
    #println(max_k,distance)
    v_max = sqrt(driver.a_cp_max/abs(max_k))
    v_des_old = driver.mlon.v_des
    set_desired_speed!(driver.mlon, min(v_des_old,v_max))
    track_longitudinal!(driver.mlon, scene, roadway, vehicle_index, fore)
    set_desired_speed!(driver.mlon, v_des_old)
    
    driver.latAcc = rand(driver.mlat)
    driver.longAcc = rand(driver.mlon).a
    driver
end

function Base.rand(driver::UrbanDriver)
    LatLonAccelDirection(driver.latAcc, driver.longAcc, driver.direction)
end

Distributions.pdf(driver::UrbanDriver, a::LatLonAccelDirection) = pdf(driver.mlat, a.a_lat) * pdf(driver.mlon, a.a_lon)
Distributions.logpdf(driver::UrbanDriver, a::LatLonAccelDirection) = logpdf(driver.mlat, a.a_lat) * logpdf(driver.mlon, a.a_lon)
