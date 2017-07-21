type MultiPtsTurningDriver <: MultiPtsDriver
    Δt::Float64
    siji::AccSteerDriver
    N::Int #number of points
    Pts::Matrix{Float64}  # 2×N
    steermax::Float64 # pi/2;
    steerdotmax::Float64 # 20;
    accmax::Float64 # 10;
    
    commands::Array{Float64} #4x1
    index::Int
    subindex::Int
    laneNum_desire::Int
    acc_desire::Float64
    v_desire::Float64
    v_max::Float64 #20m/s
    v_min::Float64 #0
    turning_direction::Int
    
    color::Colorant
    
    MultiPtsTurningDriver(Δt::Float64; siji::AccSteerDriver=AccSteerDriver(0.0,0.0),N::Int=20,Pts::Matrix{Float64}=zeros(2,20),steermax::Float64=pi/4,steerdotmax::Float64=20.0,accmax::Float64=10.0,commands::Array{Float64}=zeros(4),index::Int=0,subindex::Int=0,laneNum_desire::Int=1,acc_desire::Float64=0.0,v_desire::Float64=0.0,v_max::Float64=20.0,v_min::Float64=0.0,turning_direction::Int=1,color::Colorant=RGB(1, 0, 0))=new(Δt,siji,N,Pts,steermax,steerdotmax,accmax,commands,index,subindex,laneNum_desire,acc_desire,v_desire,v_max,v_min,turning_direction,color)
end

AutomotiveDrivingModels.get_name(model::MultiPtsTurningDriver) = "MultiPtsTurningDriver"

Base.rand(model::MultiPtsTurningDriver) = rand(model.siji)
Distributions.pdf(model::MultiPtsTurningDriver, a::Float64) = a == model.siji.acc ? Inf : 0.0
Distributions.logpdf(model::MultiPtsTurningDriver, a::Float64) = a == model.siji.acc ? Inf : -Inf


function excute_action!(model::MultiPtsTurningDriver,acc::Float64,direction::Int,scene::Union{Scene,Frame{Entity{VehicleState, BicycleModel, Int}}}, roadway::Roadway, ego_index::Int)
    veh=scene[ego_index]
    laneNum=veh.state.posF.roadind.tag.lane
    model.acc_desire=acc
    model.laneNum_desire=laneNum+direction
    model.v_desire=veh.state.v
    model.index=0
    model.subindex=0
end

function set_pts!(model::MultiPtsTurningDriver,PtsIn::Matrix{Float64})
    model.Pts=PtsIn
    model.index=0
    model.subindex=0
    model
end

function AutomotiveDrivingModels.observe!(model::MultiPtsTurningDriver, scene::Union{Scene,Frame{Entity{VehicleState, BicycleModel, Int}}}, roadway::Roadway, egoid::Int)
    ego_index = findfirst(scene, egoid)
    get_acc_steer!(model,scene,roadway,ego_index)
    model
end

function get_acc_steer!(model::MultiPtsTurningDriver,scene::Union{Scene,Frame{Entity{VehicleState, BicycleModel, Int}}}, roadway::Roadway, ego_index::Int)
    if model.index==6 || model.index>model.N
    #if model.index>model.N
        #println("index reach ", model.index)
        set_lane_changing_pts_with_direction!(model, scene, roadway, ego_index,model.turning_direction)
    end
    if model.index==0
        #println("index is 0!")
        set_lane_changing_pts_with_direction!(model, scene, roadway, ego_index,model.turning_direction)
    else
        if model.subindex==0
            #compute new command
            get_commands!(model,scene, roadway,ego_index)
            model.siji.acc=model.commands[1]
            model.siji.steer=model.commands[3]
            model.siji.direction=model.turning_direction
            model.subindex=1
        elseif model.subindex==1
            #follow the commands
            model.siji.acc=model.commands[2]
            model.siji.steer=model.commands[4]
            model.siji.direction=model.turning_direction
            model.subindex=0
            model.index=model.index+1
        end 
    end
end