function set_lane_changing_pts!(model::MultiPtsDriver, scene::Union{Scene,Frame{Entity{VehicleState, BicycleModel, Int}}}, roadway::Roadway, ego_index::Int)
    # direction is like -1: turn to right lane, 1: turn to left lane
    #println("enter function")
            
    veh = scene[ego_index]    
    x = veh.state.posG.x
    y = veh.state.posG.y
    θ = veh.state.posG.θ
    steer=model.siji.steer
    steerdotmax=model.steerdotmax
    steermax=model.steermax
    accmax=model.accmax
    v = veh.state.v
    roadind=veh.state.posF.roadind
    Δt = model.Δt
    
    #determine transitionStep
    #if it is current lane
    #first transition
    #second transition
    #smoothing
    laneNum=roadind.tag.lane
    transitionStep=0
    v_desire=model.v_desire

    pos=veh.state.posG
    transitionMagnitude=0.2
    if model.laneNum_desire>laneNum #turn to left lane
        transitionStep=transitionMagnitude
        roadind1=roadind
        frenet=Frenet(roadind1, roadway[roadind1].s,veh.state.posF.t,veh.state.posF.ϕ)
        i=1
        while frenet.t+transitionStep<DEFAULT_LANE_WIDTH/2 && i<=20
        
            v_desire_n=v_desire+model.acc_desire*(Δt*2)
            v_desire_n=clamp(v_desire_n,model.v_min,model.v_max)
            Δs=(v_desire+v_desire_n)*(Δt*2)/2
            v_desire=v_desire_n
            
            if i==6
                model.v_desire=v_desire
            end
        
            roadind1=move_along(roadind1, roadway, Δs)
            frenet=Frenet(roadind1, roadway[roadind1].s,frenet.t+transitionStep,veh.state.posF.ϕ)
            pos=get_posG(frenet,roadway)
            model.Pts[1,i]=pos.x
            model.Pts[2,i]=pos.y
            i=i+1
        end
        if frenet.t+transitionStep>=DEFAULT_LANE_WIDTH/2 && i<=20
        
            v_desire_n=v_desire+model.acc_desire*(Δt*2)
            v_desire_n=clamp(v_desire_n,model.v_min,model.v_max)
            Δs=(v_desire+v_desire_n)*(Δt*2)/2
            v_desire=v_desire_n
            
            if i==6
                model.v_desire=v_desire
            end
        
            lane = roadway[roadind1.tag]
            lane_desire = roadway[LaneTag(lane.tag.segment, lane.tag.lane + 1)]
            posG=pos
            roadproj = proj(posG, lane_desire, roadway)
            roadind2=RoadIndex(roadproj)
            roadind2=move_along(roadind2, roadway, Δs)
            frenet=Frenet(roadind2, roadway[roadind2].s,-DEFAULT_LANE_WIDTH/2+frenet.t+transitionStep-DEFAULT_LANE_WIDTH/2,veh.state.posF.ϕ)
            pos=get_posG(frenet,roadway)
            model.Pts[1,i]=pos.x
            model.Pts[2,i]=pos.y
            i=i+1
            while frenet.t+transitionStep<=0 && i<=20
            
                v_desire_n=v_desire+model.acc_desire*(Δt*2)
                v_desire_n=clamp(v_desire_n,model.v_min,model.v_max)
                Δs=(v_desire+v_desire_n)*(Δt*2)/2
                v_desire=v_desire_n
            
                if i==6
                    model.v_desire=v_desire
                end
            
                roadind2=move_along(roadind2, roadway, Δs)
                frenet=Frenet(roadind2, roadway[roadind2].s,frenet.t+transitionStep,veh.state.posF.ϕ)
                pos=get_posG(frenet,roadway)
                model.Pts[1,i]=pos.x
                model.Pts[2,i]=pos.y
                i=i+1
            end
            while frenet.t+transitionStep>=0 && i<=20
            
                v_desire_n=v_desire+model.acc_desire*(Δt*2)
                v_desire_n=clamp(v_desire_n,model.v_min,model.v_max)
                Δs=(v_desire+v_desire_n)*(Δt*2)/2
                v_desire=v_desire_n
            
                if i==6
                    model.v_desire=v_desire
                end
            
                roadind2=move_along(roadind2, roadway, Δs)
                frenet=Frenet(roadind2, roadway[roadind2].s,0.0,veh.state.posF.ϕ)
                pos=get_posG(frenet,roadway)
                model.Pts[1,i]=pos.x
                model.Pts[2,i]=pos.y
                i=i+1
            end
        end   
        
    elseif model.laneNum_desire<laneNum #turn to right lane
        transitionStep=-transitionMagnitude
        roadind1=roadind
        frenet=Frenet(roadind1, roadway[roadind1].s,veh.state.posF.t,veh.state.posF.ϕ)
        i=1
        while frenet.t+transitionStep>-DEFAULT_LANE_WIDTH/2 && i<=20
        
            v_desire_n=v_desire+model.acc_desire*(Δt*2)
            v_desire_n=clamp(v_desire_n,model.v_min,model.v_max)
            Δs=(v_desire+v_desire_n)*(Δt*2)/2
            v_desire=v_desire_n
            
            if i==6
                model.v_desire=v_desire
            end
        
            roadind1=move_along(roadind1, roadway, Δs)
            frenet=Frenet(roadind1, roadway[roadind1].s,frenet.t+transitionStep,veh.state.posF.ϕ)
            pos=get_posG(frenet,roadway)
            model.Pts[1,i]=pos.x
            model.Pts[2,i]=pos.y
            i=i+1
        end
        if frenet.t+transitionStep<=DEFAULT_LANE_WIDTH/2 && i<=20
        
            v_desire_n=v_desire+model.acc_desire*(Δt*2)
            v_desire_n=clamp(v_desire_n,model.v_min,model.v_max)
            Δs=(v_desire+v_desire_n)*(Δt*2)/2
            v_desire=v_desire_n
            
            if i==6
                model.v_desire=v_desire
            end
        
            lane = roadway[roadind1.tag]
            lane_desire = roadway[LaneTag(lane.tag.segment, lane.tag.lane - 1)]
            posG=pos
            roadproj = proj(posG, lane_desire, roadway)
            roadind2=RoadIndex(roadproj)
            roadind2=move_along(roadind2, roadway, Δs)
            frenet=Frenet(roadind2, roadway[roadind2].s,DEFAULT_LANE_WIDTH/2+(frenet.t+transitionStep+DEFAULT_LANE_WIDTH/2),veh.state.posF.ϕ)
            pos=get_posG(frenet,roadway)
            model.Pts[1,i]=pos.x
            model.Pts[2,i]=pos.y
            i=i+1
            while frenet.t+transitionStep>=0 && i<=20
            
                v_desire_n=v_desire+model.acc_desire*(Δt*2)
                v_desire_n=clamp(v_desire_n,model.v_min,model.v_max)
                Δs=(v_desire+v_desire_n)*(Δt*2)/2
                v_desire=v_desire_n
            
                if i==6
                    model.v_desire=v_desire
                end
            
                roadind2=move_along(roadind2, roadway, Δs)
                frenet=Frenet(roadind2, roadway[roadind2].s,frenet.t+transitionStep,veh.state.posF.ϕ)
                pos=get_posG(frenet,roadway)
                model.Pts[1,i]=pos.x
                model.Pts[2,i]=pos.y
                i=i+1
            end
            while frenet.t+transitionStep<=0 && i<=20
            
                v_desire_n=v_desire+model.acc_desire*(Δt*2)
                v_desire_n=clamp(v_desire_n,model.v_min,model.v_max)
                Δs=(v_desire+v_desire_n)*(Δt*2)/2
                v_desire=v_desire_n
            
                if i==6
                    model.v_desire=v_desire
                end
            
                roadind2=move_along(roadind2, roadway, Δs)
                frenet=Frenet(roadind2, roadway[roadind2].s,0.0,veh.state.posF.ϕ)
                pos=get_posG(frenet,roadway)
                model.Pts[1,i]=pos.x
                model.Pts[2,i]=pos.y
                i=i+1
            end
        end 
        
    else #on the same lane try keep driving on the center lane
        if veh.state.posF.t>0
            transitionStep=-transitionMagnitude
            roadind1=roadind
            frenet=Frenet(roadind1, roadway[roadind1].s,veh.state.posF.t,veh.state.posF.ϕ)
            i=1
            while frenet.t+transitionStep>0 && i<=20
            
                v_desire_n=v_desire+model.acc_desire*(Δt*2)
                v_desire_n=clamp(v_desire_n,model.v_min,model.v_max)
                Δs=(v_desire+v_desire_n)*(Δt*2)/2
                v_desire=v_desire_n
            
                if i==6
                    model.v_desire=v_desire
                end
            
                roadind1=move_along(roadind1, roadway, Δs)
                frenet=Frenet(roadind1, roadway[roadind1].s,frenet.t+transitionStep,veh.state.posF.ϕ)
                pos=get_posG(frenet,roadway)
                model.Pts[1,i]=pos.x
                model.Pts[2,i]=pos.y
                i=i+1
            end
            while frenet.t+transitionStep<=0 && i<=20
            
                v_desire_n=v_desire+model.acc_desire*(Δt*2)
                v_desire_n=clamp(v_desire_n,model.v_min,model.v_max)
                Δs=(v_desire+v_desire_n)*(Δt*2)/2
                v_desire=v_desire_n
            
                if i==6
                    model.v_desire=v_desire
                end
            
                roadind1=move_along(roadind1, roadway, Δs)
                frenet=Frenet(roadind1, roadway[roadind1].s,0.0,veh.state.posF.ϕ)
                pos=get_posG(frenet,roadway)
                model.Pts[1,i]=pos.x
                model.Pts[2,i]=pos.y
                i=i+1
            end
        elseif veh.state.posF.t<0
            transitionStep=transitionMagnitude
            roadind1=roadind
            frenet=Frenet(roadind1, roadway[roadind1].s,veh.state.posF.t,veh.state.posF.ϕ)
            i=1
            while frenet.t+transitionStep<0 && i<=20
            
                v_desire_n=v_desire+model.acc_desire*(Δt*2)
                v_desire_n=clamp(v_desire_n,model.v_min,model.v_max)
                Δs=(v_desire+v_desire_n)*(Δt*2)/2
                v_desire=v_desire_n
            
                if i==6
                    model.v_desire=v_desire
                end
            
                roadind1=move_along(roadind1, roadway, Δs)
                frenet=Frenet(roadind1, roadway[roadind1].s,frenet.t+transitionStep,veh.state.posF.ϕ)
                pos=get_posG(frenet,roadway)
                model.Pts[1,i]=pos.x
                model.Pts[2,i]=pos.y
                i=i+1
            end
            while frenet.t+transitionStep>=0 && i<=20
            
                v_desire_n=v_desire+model.acc_desire*(Δt*2)
                v_desire_n=clamp(v_desire_n,model.v_min,model.v_max)
                Δs=(v_desire+v_desire_n)*(Δt*2)/2
                v_desire=v_desire_n
            
                if i==6
                    model.v_desire=v_desire
                end
            
                roadind1=move_along(roadind1, roadway, Δs)
                frenet=Frenet(roadind1, roadway[roadind1].s,0.0,veh.state.posF.ϕ)
                pos=get_posG(frenet,roadway)
                model.Pts[1,i]=pos.x
                model.Pts[2,i]=pos.y
                i=i+1
            end
        elseif veh.state.posF.t==0
            roadind1=roadind
            frenet=Frenet(roadind1, roadway[roadind1].s,0.0,veh.state.posF.ϕ)
            i=1
            while i<=20
            
                v_desire_n=v_desire+model.acc_desire*(Δt*2)
                v_desire_n=clamp(v_desire_n,model.v_min,model.v_max)
                Δs=(v_desire+v_desire_n)*(Δt*2)/2
                v_desire=v_desire_n
            
                if i==6
                    model.v_desire=v_desire
                end
            
                roadind1=move_along(roadind1, roadway, Δs)
                frenet=Frenet(roadind1, roadway[roadind1].s,0.0,veh.state.posF.ϕ)
                pos=get_posG(frenet,roadway)
                model.Pts[1,i]=pos.x
                model.Pts[2,i]=pos.y
                i=i+1
            end           
        end
    end
    model.index=1
    model.subindex=0
    #getAccSteer!(model,scene,roadway,ego_index)
    model
end


#########

function set_lane_changing_pts_with_direction!(model::MultiPtsDriver, scene::Union{Scene,Frame{Entity{VehicleState, BicycleModel, Int}}}, roadway::Roadway, ego_index::Int, turning_direction::Int = 1)
    
    veh = scene[ego_index]    
    x = veh.state.posG.x
    y = veh.state.posG.y
    θ = veh.state.posG.θ
    steer=model.siji.steer
    steerdotmax=model.steerdotmax
    steermax=model.steermax
    accmax=model.accmax
    v = veh.state.v
    roadind=veh.state.posF.roadind
    Δt = model.Δt
    
    #determine transitionStep
    #if it is current lane
    #first transition
    #second transition
    #smoothing
    laneNum=roadind.tag.lane
    transitionStep=0
    v_desire=model.v_desire

    pos=veh.state.posG
    transitionMagnitude=0.2
    if model.laneNum_desire>laneNum #turn to left lane
        transitionStep=transitionMagnitude
        roadind1=roadind
        frenet=Frenet(roadind1, roadway[roadind1].s,veh.state.posF.t,veh.state.posF.ϕ)
        i=1
        while frenet.t+transitionStep<DEFAULT_LANE_WIDTH/2 && i<=20
        
            v_desire_n=v_desire+model.acc_desire*(Δt*2)
            v_desire_n=clamp(v_desire_n,model.v_min,model.v_max)
            Δs=(v_desire+v_desire_n)*(Δt*2)/2
            v_desire=v_desire_n
            
            if i==6
                model.v_desire=v_desire
            end
        
            roadind1=move_along_with_direction(roadind1, roadway, Δs,direction=turning_direction)
            frenet=Frenet(roadind1, roadway[roadind1].s,frenet.t+transitionStep,veh.state.posF.ϕ)
            pos=get_posG(frenet,roadway)
            model.Pts[1,i]=pos.x
            model.Pts[2,i]=pos.y
            i=i+1
        end
        if frenet.t+transitionStep>=DEFAULT_LANE_WIDTH/2 && i<=20
        
            v_desire_n=v_desire+model.acc_desire*(Δt*2)
            v_desire_n=clamp(v_desire_n,model.v_min,model.v_max)
            Δs=(v_desire+v_desire_n)*(Δt*2)/2
            v_desire=v_desire_n
            
            if i==6
                model.v_desire=v_desire
            end
        
            lane = roadway[roadind1.tag]
            lane_desire = roadway[LaneTag(lane.tag.segment, lane.tag.lane + 1)]
            posG=pos
            roadproj = proj(posG, lane_desire, roadway)
            roadind2=RoadIndex(roadproj)
            roadind2=move_along_with_direction(roadind2, roadway, Δs,direction=turning_direction)
            frenet=Frenet(roadind2, roadway[roadind2].s,-DEFAULT_LANE_WIDTH/2+frenet.t+transitionStep-DEFAULT_LANE_WIDTH/2,veh.state.posF.ϕ)
            pos=get_posG(frenet,roadway)
            model.Pts[1,i]=pos.x
            model.Pts[2,i]=pos.y
            i=i+1
            while frenet.t+transitionStep<=0 && i<=20
            
                v_desire_n=v_desire+model.acc_desire*(Δt*2)
                v_desire_n=clamp(v_desire_n,model.v_min,model.v_max)
                Δs=(v_desire+v_desire_n)*(Δt*2)/2
                v_desire=v_desire_n
            
                if i==6
                    model.v_desire=v_desire
                end
            
                roadind2=move_along_with_direction(roadind2, roadway, Δs,direction=turning_direction)
                frenet=Frenet(roadind2, roadway[roadind2].s,frenet.t+transitionStep,veh.state.posF.ϕ)
                pos=get_posG(frenet,roadway)
                model.Pts[1,i]=pos.x
                model.Pts[2,i]=pos.y
                i=i+1
            end
            while frenet.t+transitionStep>=0 && i<=20
            
                v_desire_n=v_desire+model.acc_desire*(Δt*2)
                v_desire_n=clamp(v_desire_n,model.v_min,model.v_max)
                Δs=(v_desire+v_desire_n)*(Δt*2)/2
                v_desire=v_desire_n
            
                if i==6
                    model.v_desire=v_desire
                end
            
                roadind2=move_along_with_direction(roadind2, roadway, Δs,direction=turning_direction)
                frenet=Frenet(roadind2, roadway[roadind2].s,0.0,veh.state.posF.ϕ)
                pos=get_posG(frenet,roadway)
                model.Pts[1,i]=pos.x
                model.Pts[2,i]=pos.y
                i=i+1
            end
        end   
        
    elseif model.laneNum_desire<laneNum #turn to right lane
        transitionStep=-transitionMagnitude
        roadind1=roadind
        frenet=Frenet(roadind1, roadway[roadind1].s,veh.state.posF.t,veh.state.posF.ϕ)
        i=1
        while frenet.t+transitionStep>-DEFAULT_LANE_WIDTH/2 && i<=20
        
            v_desire_n=v_desire+model.acc_desire*(Δt*2)
            v_desire_n=clamp(v_desire_n,model.v_min,model.v_max)
            Δs=(v_desire+v_desire_n)*(Δt*2)/2
            v_desire=v_desire_n
            
            if i==6
                model.v_desire=v_desire
            end
        
            roadind1=move_along_with_direction(roadind1, roadway, Δs,direction=turning_direction)
            frenet=Frenet(roadind1, roadway[roadind1].s,frenet.t+transitionStep,veh.state.posF.ϕ)
            pos=get_posG(frenet,roadway)
            model.Pts[1,i]=pos.x
            model.Pts[2,i]=pos.y
            i=i+1
        end
        if frenet.t+transitionStep<=DEFAULT_LANE_WIDTH/2 && i<=20
        
            v_desire_n=v_desire+model.acc_desire*(Δt*2)
            v_desire_n=clamp(v_desire_n,model.v_min,model.v_max)
            Δs=(v_desire+v_desire_n)*(Δt*2)/2
            v_desire=v_desire_n
            
            if i==6
                model.v_desire=v_desire
            end
        
            lane = roadway[roadind1.tag]
            lane_desire = roadway[LaneTag(lane.tag.segment, lane.tag.lane - 1)]
            posG=pos
            roadproj = proj(posG, lane_desire, roadway)
            roadind2=RoadIndex(roadproj)
            roadind2=move_along_with_direction(roadind2, roadway, Δs,direction=turning_direction)
            frenet=Frenet(roadind2, roadway[roadind2].s,DEFAULT_LANE_WIDTH/2+(frenet.t+transitionStep+DEFAULT_LANE_WIDTH/2),veh.state.posF.ϕ)
            pos=get_posG(frenet,roadway)
            model.Pts[1,i]=pos.x
            model.Pts[2,i]=pos.y
            i=i+1
            while frenet.t+transitionStep>=0 && i<=20
            
                v_desire_n=v_desire+model.acc_desire*(Δt*2)
                v_desire_n=clamp(v_desire_n,model.v_min,model.v_max)
                Δs=(v_desire+v_desire_n)*(Δt*2)/2
                v_desire=v_desire_n
            
                if i==6
                    model.v_desire=v_desire
                end
            
                roadind2=move_along_with_direction(roadind2, roadway, Δs,direction=turning_direction)
                frenet=Frenet(roadind2, roadway[roadind2].s,frenet.t+transitionStep,veh.state.posF.ϕ)
                pos=get_posG(frenet,roadway)
                model.Pts[1,i]=pos.x
                model.Pts[2,i]=pos.y
                i=i+1
            end
            while frenet.t+transitionStep<=0 && i<=20
            
                v_desire_n=v_desire+model.acc_desire*(Δt*2)
                v_desire_n=clamp(v_desire_n,model.v_min,model.v_max)
                Δs=(v_desire+v_desire_n)*(Δt*2)/2
                v_desire=v_desire_n
            
                if i==6
                    model.v_desire=v_desire
                end
            
                roadind2=move_along_with_direction(roadind2, roadway, Δs,direction=turning_direction)
                frenet=Frenet(roadind2, roadway[roadind2].s,0.0,veh.state.posF.ϕ)
                pos=get_posG(frenet,roadway)
                model.Pts[1,i]=pos.x
                model.Pts[2,i]=pos.y
                i=i+1
            end
        end 
        
    else #on the same lane try keep driving on the center lane
        if veh.state.posF.t>0
            transitionStep=-transitionMagnitude
            roadind1=roadind
            frenet=Frenet(roadind1, roadway[roadind1].s,veh.state.posF.t,veh.state.posF.ϕ)
            i=1
            while frenet.t+transitionStep>0 && i<=20
            
                v_desire_n=v_desire+model.acc_desire*(Δt*2)
                v_desire_n=clamp(v_desire_n,model.v_min,model.v_max)
                Δs=(v_desire+v_desire_n)*(Δt*2)/2
                v_desire=v_desire_n
            
                if i==6
                    model.v_desire=v_desire
                end
            
                roadind1=move_along_with_direction(roadind1, roadway, Δs,direction=turning_direction)
                frenet=Frenet(roadind1, roadway[roadind1].s,frenet.t+transitionStep,veh.state.posF.ϕ)
                pos=get_posG(frenet,roadway)
                model.Pts[1,i]=pos.x
                model.Pts[2,i]=pos.y
                i=i+1
            end
            while frenet.t+transitionStep<=0 && i<=20
            
                v_desire_n=v_desire+model.acc_desire*(Δt*2)
                v_desire_n=clamp(v_desire_n,model.v_min,model.v_max)
                Δs=(v_desire+v_desire_n)*(Δt*2)/2
                v_desire=v_desire_n
            
                if i==6
                    model.v_desire=v_desire
                end
            
                roadind1=move_along_with_direction(roadind1, roadway, Δs,direction=turning_direction)
                frenet=Frenet(roadind1, roadway[roadind1].s,0.0,veh.state.posF.ϕ)
                pos=get_posG(frenet,roadway)
                model.Pts[1,i]=pos.x
                model.Pts[2,i]=pos.y
                i=i+1
            end
        elseif veh.state.posF.t<0
            transitionStep=transitionMagnitude
            roadind1=roadind
            frenet=Frenet(roadind1, roadway[roadind1].s,veh.state.posF.t,veh.state.posF.ϕ)
            i=1
            while frenet.t+transitionStep<0 && i<=20
            
                v_desire_n=v_desire+model.acc_desire*(Δt*2)
                v_desire_n=clamp(v_desire_n,model.v_min,model.v_max)
                Δs=(v_desire+v_desire_n)*(Δt*2)/2
                v_desire=v_desire_n
            
                if i==6
                    model.v_desire=v_desire
                end
            
                roadind1=move_along_with_direction(roadind1, roadway, Δs,direction=turning_direction)
                frenet=Frenet(roadind1, roadway[roadind1].s,frenet.t+transitionStep,veh.state.posF.ϕ)
                pos=get_posG(frenet,roadway)
                model.Pts[1,i]=pos.x
                model.Pts[2,i]=pos.y
                i=i+1
            end
            while frenet.t+transitionStep>=0 && i<=20
            
                v_desire_n=v_desire+model.acc_desire*(Δt*2)
                v_desire_n=clamp(v_desire_n,model.v_min,model.v_max)
                Δs=(v_desire+v_desire_n)*(Δt*2)/2
                v_desire=v_desire_n
            
                if i==6
                    model.v_desire=v_desire
                end
            
                roadind1=move_along_with_direction(roadind1, roadway, Δs,direction=turning_direction)
                frenet=Frenet(roadind1, roadway[roadind1].s,0.0,veh.state.posF.ϕ)
                pos=get_posG(frenet,roadway)
                model.Pts[1,i]=pos.x
                model.Pts[2,i]=pos.y
                i=i+1
            end
        elseif veh.state.posF.t==0
            roadind1=roadind
            frenet=Frenet(roadind1, roadway[roadind1].s,0.0,veh.state.posF.ϕ)
            i=1
            while i<=20
            
                v_desire_n=v_desire+model.acc_desire*(Δt*2)
                v_desire_n=clamp(v_desire_n,model.v_min,model.v_max)
                Δs=(v_desire+v_desire_n)*(Δt*2)/2
                v_desire=v_desire_n
            
                if i==6
                    model.v_desire=v_desire
                end
            
                roadind1=move_along_with_direction(roadind1, roadway, Δs,direction=turning_direction)
                frenet=Frenet(roadind1, roadway[roadind1].s,0.0,veh.state.posF.ϕ)
                pos=get_posG(frenet,roadway)
                model.Pts[1,i]=pos.x
                model.Pts[2,i]=pos.y
                i=i+1
            end           
        end
    end
    model.index=1
    model.subindex=0
    #getAccSteer!(model,scene,roadway,ego_index)
    model
end


