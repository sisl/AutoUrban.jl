export AccSteerDriver

mutable struct AccSteerDriver <: DriverModel{AccelSteeringDirection}
    acc::Float64
    steer::Float64
    direction::Int
end

function AccSteerDriver(acc::Float64, steer::Float64)
    return AccSteerDriver(acc, steer, 1)
end

AutomotiveDrivingModels.get_name(model::AccSteerDriver) = "AccSteerDriver"

function AutomotiveDrivingModels.observe!(model::AccSteerDriver, scene::Scene, roadway::Roadway, egoid::Int)
    model
end
Base.rand(model::AccSteerDriver) = AccelSteeringDirection(model.acc,model.steer,model.direction)
Distributions.pdf(model::AccSteerDriver, a::Float64) = a == model.acc ? Inf : 0.0
Distributions.logpdf(model::AccSteerDriver, a::Float64) = a == model.acc ? Inf : -Inf