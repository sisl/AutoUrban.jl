abstract IntersectionEvent

################################################################
type AllWayStop <: IntersectionEvent
    connections::Array{Tuple{Int,Int}}
    entrances::Array{Int}
    exits::Array{Int}
    
    function AllWayStop()
        new(Tuple{Int,Int}[],Int[],Int[])
    end
    
    function AllWayStop(connections::Array{Tuple{Int,Int}})
        entrances = []
        exits = []
        for connection in connections
            if isempty(find(entrances .== connection[1]))
                push!(entrances,connection[1])
            end
            if isempty(find(exits .== connection[2]))
                push!(exits,connection[2])
            end
        end
        new(connections,entrances,exits)
    end
end

function updateAllWayStops!(driver::DriverModel,allwaystops::Array{AllWayStop})
    driver.allwaystops = allwaystops
    driver
end

function get_distance_to_stop(veh::Vehicle,allwaystop::AllWayStop,roadway::Roadway)
    vehSegid = veh.state.posF.roadind.tag.segment
    vehLaneid = veh.state.posF.roadind.tag.lane
    vehTag = veh.state.posF.roadind.tag
    distance_to_stop = -1.0
    if !isempty(find(allwaystop.entrances .== vehSegid))
        lane = roadway[vehTag]
        cindS = curveindex_end(lane.curve)
        endPoint = lane.curve[cindS]
        distance_to_stop = endPoint.s - veh.state.posF.s
    end
    return distance_to_stop
end

function stop_before(v::Float64,distance_to_stop::Float64,horizon::Float64=MAX_HORIZON,model::IDMDriver = IDMDriver())
    s_min = CAR_LENGTH/2

    s_gap = distance_to_stop
    acc = Inf
    if s_gap < horizon
        if s_gap > 0.0
            Δv = 0.0 - v
            s_des = s_min + v*model.T - v*Δv / (2*sqrt(model.a_max*model.d_cmf))
            v_ratio = model.v_des > 0.0 ? (v/model.v_des) : 1.0
            acc = model.a_max * (1.0 - v_ratio^model.δ - (s_des/s_gap)^2)
        elseif s_gap > -CAR_LENGTH
            acc = -model.d_max
        else
            Δv = model.v_des - v
            acc = Δv*model.k_spd
        end

        if isnan(acc)

            warn("IDM acceleration was NaN!")
            if s_gap > 0.0
                Δv = veh_target.state.v - v
                s_des = model.s_min + v*model.T - v*Δv / (2*sqrt(model.a_max*model.d_cmf))
                println("\tΔv: ", Δv)
                println("\ts_des: ", s_des)
                println("\tv_des: ", model.v_des)
                println("\tδ: ", model.δ)
                println("\ts_gap: ", s_gap)
            elseif s_gap > -veh_ego.def.length
                println("\td_max: ", model.d_max)
            end

            acc = 0.0
        end
    else
        # no lead vehicle, just drive to match desired speed
        Δv = model.v_des - v
        acc = Δv*model.k_spd # predicted accel to match target speed
    end
    acc = clamp(acc, -model.d_max, model.a_max)
    return acc
end


function update_waitlist!(driver, new::Bool,ego_distance::Float64,allwaystop::AllWayStop, scene::Scene, roadway::Roadway, egoid::Int)
    vehicle_index = findfirst(scene, egoid)
    ego_veh = scene[vehicle_index]
    if new #add veh
        empty!(driver.allwaystop_waitlist)
        for (i,veh) in enumerate(scene)
            dist = get_distance_to_stop(veh,allwaystop,roadway)
            if dist > 0.0 && dist < ego_distance
                push!(driver.allwaystop_waitlist,i)
            end
        end
    else #delete veh
        new_waitlist = []
        for veh_ind in driver.allwaystop_waitlist
            veh = scene.entities[veh_ind]
            vehSegid = veh.state.posF.roadind.tag.segment
            if isempty(find(allwaystop.exits .== vehSegid)) #this veh is still in waiting zone
                push!(new_waitlist,veh_ind)
            end
        end
        driver.allwaystop_waitlist = new_waitlist
    end
end

function reactAllWayStop!(driver, scene::Scene, roadway::Roadway, egoid::Int)
    vehicle_index = findfirst(scene, egoid)
    ego_veh = scene[vehicle_index]
    acc = Inf
    for i = 1:length(driver.allwaystops)
        distance_to_stop = get_distance_to_stop(ego_veh,driver.allwaystops[i],roadway)
        if distance_to_stop > 0.0 && distance_to_stop < driver.allwaystop_horizon
            allwaystop = driver.allwaystops[i]
            if distance_to_stop < CAR_LENGTH #in the waiting zone
                if i == driver.allwaystop_ind 
                    update_waitlist!(driver, false,distance_to_stop,allwaystop, scene, roadway, egoid)
                    if isempty(driver.allwaystop_waitlist) && driver.allwaystop_stopcount > driver.allwaystop_waittime #time to go!
                        return acc
                    else #wait
                        if abs(ego_veh.state.v) < 0.1
                            driver.allwaystop_stopcount += 1
                        end
                        acc = stop_before(ego_veh.state.v,distance_to_stop,driver.allwaystop_horizon,driver.mlon)
                        return acc
                    end
                else # encounter new waiting zone
                    driver.allwaystop_stopcount = 0
                    acc = stop_before(ego_veh.state.v,distance_to_stop,driver.allwaystop_horizon,driver.mlon)
                    driver.allwaystop_ind = i
                    update_waitlist!(driver, true,distance_to_stop,allwaystop, scene, roadway, egoid)
                    return acc
                end
            else #outside the waiting zone
                acc = stop_before(ego_veh.state.v,distance_to_stop,driver.allwaystop_horizon,driver.mlon)
                break
            end
        end
    end
    #no allwaystop is encounterd
    driver.allwaystop_ind = 0
    empty!(driver.allwaystop_waitlist)
    return acc
end

##########################################################
type TrafficLight <: IntersectionEvent
    color::Int #0 green 1 red 2 yellow
    pos::VecSE2
end

function updateTrafficLights!(driver::DriverModel,trafficlights::Array{TrafficLight})
    driver.trafficlights = trafficlights
    driver
end

function get_yellowlight_decisison(distance)
    decision = 0
    if distance > MAX_HORIZON
        decision = 0
    else
        prob = distance*0.7/MAX_HORIZON
        #println("distance is : ",distance, " prob is : ",prob)
        if rand() < prob
            decision = -1
        end
    end
    return decision
end

function reac_yellowlight(driver,scene,roadway,foreResult)
    acc = Inf
    if driver.yellowlight_decision == -1
        acc = reac_redlight(driver,scene,roadway,foreResult)
    end
    return acc
end

function reac_redlight(driver,scene,roadway,foreResult)
    s_min_old = driver.mlon.s_min
    driver.mlon.s_min = 2.0
    AutomotiveDrivingModelsOld.track_longitudinal!(driver.mlon, scene, roadway, 1, foreResult.ind)
    driver.mlon.s_min = s_min_old
    acc = rand(driver.mlon)
    return acc
end

function reactTrafficLights(driver, scene::Scene, roadway::Roadway, egoid::Int)
    vehicle_index = findfirst(scene, egoid)
    ego_veh = scene[vehicle_index]
    scene= Scene(length(driver.trafficlights)+1)
    push!(scene,Vehicle(ego_veh.state,
                        VehicleDef(1, AgentClass.CAR, CAR_LENGTH, CAR_WIDTH)))
    for i=1:length(driver.trafficlights)
        push!(scene,Vehicle(VehicleState(driver.trafficlights[i].pos, roadway, 0.0),
            VehicleDef(AgentClass.CAR, 1.0, 3.0),i))
    end
    foreResult = get_neighbor_fore_along_lane(scene, 1, roadway)
    light_ind = 0
    distance = 0.0
    acc = Inf
    if foreResult.ind != 0 && foreResult.Δs < MAX_HORIZON
        light_ind = foreResult.ind - 1
        distance = foreResult.Δs
        if driver.trafficlights[light_ind].color == 2
            #println("light_id : ",light_ind," driver.yellowlight_id: ",driver.yellowlight_id)
            if light_ind == driver.yellowlight_id
                #println("continue previous decision ", driver.yellowlight_decision)
            else
                driver.yellowlight_id = light_ind
                driver.yellowlight_decision = get_yellowlight_decisison(distance)
            end
            acc = reac_yellowlight(driver,scene,roadway,foreResult)
        else
            driver.yellowlight_decision = 0
            driver.yellowlight_id = 0
        end
        if driver.trafficlights[light_ind].color == 1
            acc = reac_redlight(driver,scene,roadway,foreResult)
        end
    end
    return acc
end