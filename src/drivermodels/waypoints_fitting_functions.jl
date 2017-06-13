export 
    fit_lane,
    move_along_curve,
    get_curvePt_cec_per
        
#####under developing

curveModel(x, p) = p[1]*x.^3+p[2]*x.^2+p[3]*x+p[4]

function fit_lane(model::MultiPtsDriver, scene::Union{Scene,Frame{Entity{VehicleState, BicycleModel, Int}}}, roadway::Roadway, ego_index::Int)
    sampleDistance=25.0;
    sampleInterval=1.0;
    #model(x, p) = p[1]*x^3+p[2]*x^2+p[3]*x+p[4]
    xdata=[];
    ydata=[];
    distance=0.0;
    veh = scene[ego_index]
    roadind=veh.state.posF.roadind
    #frenet=Frenet(roadind, roadway[roadind].s,0,0)
    mode=1
    x=-Inf
    y=-Inf
    while distance<=sampleDistance
        roadind_n=move_along(roadind, roadway, distance)
        frenet=Frenet(roadind_n, roadway[roadind_n].s,0.0,0.0)
        pos=get_posG(frenet,roadway)
        if pos.x==x #repeating on x, change to mode 2
            mode=2
        end
        x=pos.x
        y=pos.y
        push!(xdata,pos.x)
        push!(ydata,pos.y)
        distance=distance+sampleInterval
    end
    #println("xdata : ",xdata)
    #println("ydata : ",ydata)
    if mode==1
        fit = curve_fit(curveModel, xdata, ydata, [0.5, 0.5, 0.5, 0.5])
        return fit,xdata[1],ydata[1],xdata[2]>xdata[1],mode
    else
        #println("change to mode 2!")
        fit = curve_fit(curveModel, ydata, xdata, [0.5, 0.5, 0.5, 0.5])
        return fit,xdata[1],ydata[1],ydata[2]>ydata[1],mode
    end
end

function move_along_curve(fit::LsqFit.LsqFitResult,x::Float64,y::Float64,Δs::Float64,direction::Bool,mode::Int)
    p=fit.param
    x_n=x
    y_n=y
    if mode==1
        for i=1:10
            slope=3*p[1]*x_n^2+2*p[2]*x_n+p[3]
            dx=Δs/sqrt(slope^2+1)/10
            if direction==1
                x_n=x_n+dx
            else
                x_n=x_n-dx
            end
            y_n=p[1]*x_n^3+p[2]*x_n^2+p[3]*x_n+p[4]
        end
    else
        for i=1:10
            slope=3*p[1]*y_n^2+2*p[2]*y_n+p[3]
            dy=Δs/sqrt(slope^2+1)/10
            if direction==1
                y_n=y_n+dy
            else
                y_n=y_n-dy
            end
            x_n=p[1]*y_n^3+p[2]*y_n^2+p[3]*y_n+p[4]
        end
    end
    return x_n,y_n
end

function get_curvePt_vec_per(fit::LsqFit.LsqFitResult,x::Float64,y::Float64,turnDirection::Int,direction::Bool,mode::Int)
    p=fit.param
    if mode==1
        slope=3*p[1]*x^2+2*p[2]*x+p[3]
        curvePtVecTan=[1/sqrt(slope^2+1);slope/sqrt(slope^2+1)];
        if  direction==0
            curvePtVecTan=-curvePtVecTan
        end
        if turnDirection==1 #turn left
            curvePtVecPer=[0 -1;1 0]* curvePtVecTan
        else #turn right
            curvePtVecPer=[0 1;-1 0]* curvePtVecTan
        end
    else
        slope=3*p[1]*y^2+2*p[2]*y+p[3]
        curvePtVecTan=[slope/sqrt(slope^2+1);1/sqrt(slope^2+1)];
        if  direction==0
            curvePtVecTan=-curvePtVecTan
        end
        if turnDirection==1 #turn left
            curvePtVecPer=[0 -1;1 0]* curvePtVecTan
        else #turn right
            curvePtVecPer=[0 1;-1 0]* curvePtVecTan
        end
    end
    return curvePtVecPer
end

function set_lane_changing_pts!(model::MultiPtsDriver, scene::Union{Scene,Frame{Entity{VehicleState, BicycleModel, Int}}}, roadway::Roadway, ego_index::Int)
    #println("enter function")
    #laneFunction=fitLane(model, scene, roadway, ego_index)
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
    Δt = model.C.Δt
    
    laneNum=roadind.tag.lane
    transitionStep=0
    
    frenet=Frenet(roadind, roadway[roadind].s,0.0,0.0)
    pos=get_posG(frenet,roadway)
    fit,xstart,ystart,direction,mode=fit_lane(model::DriverModel, scene::Scene, roadway::Roadway, ego_index::Int)
    
       
    transitionMagnitude=0.2
    offSetDistance=sqrt((x-xstart)^2+(y-ystart)^2)
    offSetVec=[x-xstart;y-ystart]
    
    v_desire=v
    
    v_desire=model.v_desire
    
    if model.laneNum_desire==laneNum
        curvePtVecPer=get_curvePt_vec_per(fit,x,y,1,direction,mode) #curvePtVecPer pointing to left
        offSetDistance=sqrt((x-xstart)^2+(y-ystart)^2)
        if (curvePtVecPer'*offSetVec)[1]>0
            turnDirection=1
        else
            turnDirection=-1
        end
        i=1
        x_n=x
        y_n=y
        while offSetDistance-transitionMagnitude>0 && i<=20
            v_desire_n=v_desire+model.acc_desire*(Δt*2)
            v_desire_n=clamp(v_desire_n,model.v_min,model.v_max)
            Δs=(v_desire+v_desire_n)*(Δt*2)/2
            v_desire=v_desire_n
            
            if i==6
                model.v_desire=v_desire
            end
            
            offSetDistance=offSetDistance-transitionMagnitude
            x_n,y_n=move_along_curve(fit,x_n,y_n,Δs,direction,mode)
            curvePtVecPer=get_curvePt_vec_per(fit,x_n,y_n,turnDirection,direction,mode)
            pos=[x_n;y_n]+offSetDistance*curvePtVecPer
            model.Pts[1,i]=pos[1]
            model.Pts[2,i]=pos[2]
            i=i+1
        end
        while offSetDistance-transitionMagnitude<=0 && i<=20
            v_desire_n=v_desire+model.acc_desire*(Δt*2)
            v_desire_n=clamp(v_desire_n,model.v_min,model.v_max)
            Δs=(v_desire+v_desire_n)*(Δt*2)/2
            v_desire=v_desire_n
            
            if i==6
                model.v_desire=v_desire
            end
            
            x_n,y_n=move_along_curve(fit,x_n,y_n,Δs,direction,mode)
            pos=[x_n;y_n]
            model.Pts[1,i]=pos[1]
            model.Pts[2,i]=pos[2]
            i=i+1
        end
    else
        if model.laneNum_desire>laneNum #turn to left lane
            turnDirection=1
            #println("turn left")
        else #turn to left right
            turnDirection=-1
            #println("right")
        end
        curvePtVecPer=curvePtVecPer=get_curvePt_vec_per(fit,x,y,turnDirection,direction,mode)
        if (curvePtVecPer'*offSetVec)[1]>0
            offSetDistance=sqrt((x-xstart)^2+(y-ystart)^2)
        else
            offSetDistance=-sqrt((x-xstart)^2+(y-ystart)^2)
        end
        i=1
        x_n=x
        y_n=y
        while offSetDistance+transitionMagnitude<1.0*DEFAULT_LANE_WIDTH && i<=20
            v_desire_n=v_desire+model.acc_desire*(Δt*2)
            v_desire_n=clamp(v_desire_n,model.v_min,model.v_max)
            Δs=(v_desire+v_desire_n)*(Δt*2)/2
            v_desire=v_desire_n
            
            if i==6
                model.v_desire=v_desire
            end
            
            offSetDistance=offSetDistance+transitionMagnitude
            x_n,y_n=move_along_cure(fit,x_n,y_n,Δs,direction,mode)
            curvePtVecPer=get_curvePt_vec_per(fit,x_n,y_n,turnDirection,direction,mode)
            pos=[x_n;y_n]+offSetDistance*curvePtVecPer
            model.Pts[1,i]=pos[1]
            model.Pts[2,i]=pos[2]
            i=i+1
        end
        while offSetDistance+transitionMagnitude>=1.0*DEFAULT_LANE_WIDTH && i<=20
            v_desire_n=v_desire+model.acc_desire*(Δt*2)
            v_desire_n=clamp(v_desire_n,model.v_min,model.v_max)
            Δs=(v_desire+v_desire_n)*(Δt*2)/2
            v_desire=v_desire_n
            
            if i==6
                model.v_desire=v_desire
            end
            
            x_n,y_n=move_along_curve(fit,x_n,y_n,Δs,direction,mode)
            curvePtVecPer=get_curvePt_vec_per(fit,x_n,y_n,turnDirection,direction,mode)
            pos=[x_n;y_n]+1.0*DEFAULT_LANE_WIDTH*curvePtVecPer
            model.Pts[1,i]=pos[1]
            model.Pts[2,i]=pos[2]
            i=i+1
        end
    end    
    
    #println("x waypoint : ",model.Pts[1,:])
    #println("y waypoint : ",model.Pts[2,:])
    model.index=1
    model.subindex=0
    model 
end