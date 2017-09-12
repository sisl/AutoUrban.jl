import AutomotiveDrivingModels.observe!

mutable struct IDMDriver <: LaneFollowingDriver
    a::Float64 # predicted acceleration
    σ::Float64 # optional stdev on top of the model, set to zero or NaN for deterministic behavior

    k_spd::Float64 # proportional constant for speed tracking when in freeflow [s⁻¹]

    δ::Float64 # acceleration exponent [-]
    T::Float64 # desired time headway [s]
    v_des::Float64 # desired speed [m/s]
    s_min::Float64 # minimum acceptable gap [m]
    a_max::Float64 # maximum acceleration ability [m/s²]
    d_cmf::Float64 # comfortable deceleration [m/s²] (positive)
    d_max::Float64 # maximum decelleration [m/s²] (positive)
    
    function IDMDriver(;
        σ::Float64     =   NaN,
        k_spd::Float64 =   1.0,
        δ::Float64     =   4.0,
        T::Float64     =   1.5,
        v_des::Float64 =  29.0, # typically overwritten
        s_min::Float64 =   5.0,
        a_max::Float64 =   3.0,
        d_cmf::Float64 =   2.0,
        d_max::Float64 =   9.0,
        )

        retval = new()
        retval.a     = NaN
        retval.σ     = σ
        retval.k_spd = k_spd
        retval.δ     = δ
        retval.T     = T
        retval.v_des = v_des
        retval.s_min = s_min
        retval.a_max = a_max
        retval.d_cmf = d_cmf
        retval.d_max = d_max
        retval
    end
end
get_name(::IDMDriver) = "IDM"
function set_desired_speed!(model::IDMDriver, v_des::Float64)
    model.v_des = v_des
    model
end

function AutomotiveDrivingModels.observe!(model::IDMDriver, scene::Frame{Entity{VehicleState, BicycleModel, Int}}, roadway::Roadway, egoid::Int)
    vehicle_index = findfirst(scene, egoid)

    fore = get_neighbor_fore_along_lane(scene, vehicle_index, roadway, VehicleTargetPointFront(), VehicleTargetPointRear(), VehicleTargetPointFront())

    v_ego = scene[vehicle_index].state.v
    v_oth = NaN
    headway = NaN

    if fore.ind != 0
        v_oth = scene[fore.ind].state.v
        headway = fore.Δs
    end

    track_longitudinal!(model, v_ego, v_oth, headway)

    return model
end

function track_longitudinal!(model::IDMDriver, v_ego::Float64, v_oth::Float64, headway::Float64)
    if !isnan(v_oth)
        #@assert !isnan(headway) && headway > 0
        if headway > 0.0
            Δv = v_oth - v_ego
            s_des = model.s_min + v_ego*model.T - v_ego*Δv / (2*sqrt(model.a_max*model.d_cmf))
            v_ratio = model.v_des > 0.0 ? (v_ego/model.v_des) : 1.0
            model.a = model.a_max * (1.0 - v_ratio^model.δ - (s_des/headway)^2)
        elseif headway > -3.0
            acc = -model.d_max
        else
            Δv = model.v_des - v_ego
            acc = Δv*model.k_spd
        end
    else
        # no lead vehicle, just drive to match desired speed
        Δv = model.v_des - v_ego
        model.a = Δv*model.k_spd # predicted accel to match target speed
    end

    @assert !isnan(model.a)

    model.a = clamp(model.a, -model.d_max, model.a_max)

    model
end

function Base.rand(model::IDMDriver)
    if isnan(model.σ) || model.σ ≤ 0.0
        LaneFollowingAccel(model.a)
    else
        LaneFollowingAccel(rand(Normal(model.a, model.σ)))
    end
end
function Distributions.pdf(model::IDMDriver, a::LaneFollowingAccel)
    if isnan(model.σ) || model.σ ≤ 0.0
        Inf
    else
        pdf(Normal(model.a, model.σ), a.a)
    end
end
function Distributions.logpdf(model::IDMDriver, a::LaneFollowingAccel)
    if isnan(model.σ) || model.σ ≤ 0.0
        Inf
    else
        logpdf(Normal(model.a, model.σ), a.a)
    end
end
