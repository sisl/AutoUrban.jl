mutable struct LatLonAccelDirection
    a_lat::Float64
    a_lon::Float64
    direction::Int
end

Base.show(
    io::IO,
    a::LatLonAccelDirection
) = @printf(
    io,
    "LatLonAccelDirection(%6.3f, %6.3f, %d)",
    a.a_lat,
    a.a_lon,
    a.direction
)

Base.length(::Type{LatLonAccelDirection}) = 3

Base.convert(
    ::Type{LatLonAccelDirection},
    v::Vector{Float64}
) = LatLonAccel(v[1], v[2], convert(Int, v[3]))

function Base.copyto!(v::Vector{Float64}, a::LatLonAccelDirection)
    v[1] = a.a_lat
    v[2] = a.a_lon
    v[3] = convert(Float64, a.direction)
    v
end

function AutomotiveDrivingModels.propagate(
    veh::Entity{VehicleState, D, Int},
    action::LatLonAccelDirection,
    roadway::Roadway,
    Δt::Float64
) where {D <: Union{VehicleDef, BicycleModel}}
    previousInd = veh.state.posF.roadind
    a_lat = action.a_lat
    a_lon = action.a_lon

    v = veh.state.v
    ϕ = veh.state.posF.ϕ
    ds = v * cos(ϕ)
    t = veh.state.posF.t
    dt = v * sin(ϕ)

    ΔT = Δt
    ΔT² = ΔT * ΔT
    Δs = ds * ΔT + 0.5 * a_lon * ΔT²
    Δt = dt * ΔT + 0.5 * a_lat * ΔT²

    ds₂ = ds + a_lon * ΔT
    dt₂ = dt + a_lat * ΔT
    speed₂ = sqrt(dt₂ * dt₂ + ds₂ * ds₂)
    v₂ = sign(ds₂) * speed₂
    ϕ₂ = atan(dt₂, ds₂) + (v₂ < 0.0) * π # handle negative speeds
    roadind = move_along_with_direction(
        veh.state.posF.roadind,
        roadway,
        Δs,
        direction = action.direction
    )

    if t + Δt <= roadway[roadind.tag].width / 2.0
        posF = Frenet(roadind, roadway, t = t + Δt, ϕ = ϕ₂)
        return VehicleState(posF, roadway, v₂)
    else
        footpoint = roadway[roadind]
        posG = convert(VecE2, footpoint.pos) + polar(t + Δt, footpoint.pos.θ + π / 2)
        posG = VecSE2(posG.x, posG.y, footpoint.pos.θ + ϕ₂)

        state = VehicleState(posG, roadway, v₂)
        projections = in_lanes(posG, roadway)
        if length(roadway[previousInd.tag].exits) > 0
            for projection in projections
                dir = clamp(action.direction, 1, length(roadway[previousInd.tag].exits))
                if roadway[previousInd.tag].exits[dir].target.tag.segment == projection.tag.segment && !isempty(roadway[previousInd.tag].exits)
                    posF = Frenet(projection, roadway)
                    state = VehicleState(posG, posF, v₂)
                    # println(posF)
                    return state
                end
            end
        end

        for projection in projections
            dir = clamp(action.direction, 1, length(roadway[previousInd.tag].exits))
            if previousInd.tag.segment == projection.tag.segment
                posF = Frenet(projection, roadway)
                state = VehicleState(posG, posF, v₂)
                # println(posF)
                return state
            end
        end
        return state
    end
end

function Base.get(
    ::Type{LatLonAccelDirection},
    rec::SceneRecord,
    roadway::Roadway,
    vehicle_index::Int,
    pastframe::Int = 0
)
    accel_lat = get(ACCFT, rec, roadway, vehicle_index, pastframe)
    accel_lon = get(ACCFS, rec, roadway, vehicle_index, pastframe)
    LatLonAccelDirection(accel_lat, accel_lon, 1)
end

function pull_action!(
    ::Type{LatLonAccelDirection},
    a::Vector{Float64},
    rec::SceneRecord,
    roadway::Roadway,
    vehicle_index::Int,
    pastframe::Int = 0
)
    a[1] = get(ACCFT, rec, roadway, vehicle_index, pastframe)
    a[2] = get(ACCFS, rec, roadway, vehicle_index, pastframe)
    a
end

mutable struct AccelSteeringDirection
    a::Float64 # accel [m/s²]
    δ::Float64 # steering angle [rad]
    direction::Int64 # turning direction
end

Base.show(
    io::IO,
    a::AccelSteeringDirection
) = @printf(io, "AccelSteeringDirection(%6.3f,%6.3f,%d)", a.a, a.δ,a.direction)

Base.length(::Type{AccelSteeringDirection}) = 3

Base.convert(
    ::Type{AccelSteeringDirection},
    v::Vector{Float64}
) = AccelSteeringDirection(v[1], v[2], convert(Int, v[3]))

function Base.copyto!(v::Vector{Float64}, a::AccelSteeringDirection)
    v[1] = a.a
    v[2] = a.δ
    v[3] = convert(Float64, a.direction)
    v
end

function AutomotiveDrivingModels.propagate(
    veh::Entity{VehicleState,D,Int},
    action::AccelSteeringDirection,
    roadway::Roadway,
    Δt::Float64
) where {D <: Union{VehicleDef, BicycleModel}}
    previousInd = veh.state.posF.roadind
    
    L = veh.def.a + veh.def.b
    l = -veh.def.b

    a = action.a # accel [m/s²]
    δ = action.δ # steering wheel angle [rad]

    x = veh.state.posG.x
    y = veh.state.posG.y
    θ = veh.state.posG.θ
    v = veh.state.v

    s = v * Δt + a * Δt * Δt / 2 # distance covered
    v′ = v + a * Δt

    posG = nothing
    if abs(δ) < 0.01 # just drive straight
        posG = veh.state.posG + polar(s, θ)
    else # drive in circle
        R = L / tan(δ) # turn radius

        β = s / R
        xc = x - R * sin(θ) + l * cos(θ)
        yc = y + R * cos(θ) + l * sin(θ)

        θ′ = mod(θ + β, 2π)
        x′ = xc + R * sin(θ + β) - l * cos(θ′)
        y′ = yc - R * cos(θ + β) - l * sin(θ′)

        posG = VecSE2(x′, y′, θ′)
    end

    # VehicleState(posG, roadway, v′)
    state = VehicleState(posG, roadway, v′)
    projections = in_lanes(posG, roadway)
    if length(roadway[previousInd.tag].exits) > 0
        for projection in projections
            dir = clamp(action.direction, 1, length(roadway[previousInd.tag].exits))
            if roadway[previousInd.tag].exits[dir].target.tag.segment == projection.tag.segment && !isempty(roadway[previousInd.tag].exits)
                posF = Frenet(projection, roadway)
                state = VehicleState(posG, posF, v′)
                # println(posF)
                return state
            end
        end
    end

    for projection in projections
        dir = clamp(action.direction, 1, length(roadway[previousInd.tag].exits))
        if previousInd.tag.segment == projection.tag.segment
            posF = Frenet(projection, roadway)
            state = VehicleState(posG, posF, v′)
            # println(posF)
            return state
        end
    end
    return state
end

mutable struct NextState
    x::Float64
    y::Float64
    theta::Float64
    speed::Float64
    direction::Int
end

function NextState()
    return NextState(0.0, 0.0, 0.0, 0.0, 1)
end

function AutomotiveDrivingModels.propagate(
    veh::Entity{VehicleState, D, Int},
    action::NextState,
    roadway::Roadway,
    Δt::Float64
) where {D <: Union{VehicleDef, BicycleModel}}
    previousInd = veh.state.posF.roadind

    posG = VecSE2(action.x, action.y, action.theta)
    v′ = action.speed

    state = VehicleState(posG, roadway, v′)
    projections = in_lanes(posG, roadway)
    if length(roadway[previousInd.tag].exits) > 0
        for projection in projections
            dir = clamp(action.direction, 1, length(roadway[previousInd.tag].exits))
            if roadway[previousInd.tag].exits[dir].target.tag.segment == projection.tag.segment && !isempty(roadway[previousInd.tag].exits)
                posF = Frenet(projection, roadway)
                state = VehicleState(posG, posF, v′)
                # println(posF)
                return state
            end
        end
    end

    for projection in projections
        dir = clamp(action.direction, 1, length(roadway[previousInd.tag].exits))
        if previousInd.tag.segment == projection.tag.segment
            posF = Frenet(projection, roadway)
            state = VehicleState(posG, posF, v′)
            # println(posF)
            return state
        end
    end
    return state
end
