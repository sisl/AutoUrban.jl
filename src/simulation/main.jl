export
	LatLonAccelDirection,
	AccelSteeringDirection,
    NextState,
    convert2vehicle

include("actions.jl")

function convert2vehicle(veh::Records.Entity{AutomotiveDrivingModels.VehicleState,AutomotiveDrivingModels.BicycleModel,Int64})
    return Entity(veh.state,veh.def.def,veh.id)
end