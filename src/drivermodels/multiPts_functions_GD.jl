export getCommands!

function getCommands!(model::DriverModel,scene::Union{Scene,Frame{Entity{VehicleState, BicycleModel, Int}}}, roadway::Roadway,ego_index::Int)
    veh = scene[ego_index]
    x = veh.state.posG.x
    y = veh.state.posG.y
    θ = veh.state.posG.θ
    steer=model.siji.steer
    steerdotmax=model.steerdotmax
    steermax=model.steermax
    accmax=model.accmax
    v = veh.state.v
    qStart=[x y θ steer v]';
    Pts=model.Pts
    xdata=Pts[1,:]
    ydata=Pts[2,:]
    numPts=length(xdata)
    
    Δt = model.Δt
    
    i=model.index
#    if i==numPts
#        qEnd=[xdata[i] ydata[i] 0 0 0]';
#        Y,u_solved,converged=computeTrajectory(qStart,qEnd,"xy",Δt,steermax,steerdotmax,accmax);
    if i==numPts
        qEnd=[xdata[i] ydata[i] atan2((ydata[i]-ydata[i-1]),(xdata[i]-xdata[i-1])) 0 0]';  
    else
        qEnd=[xdata[i] ydata[i] atan2((ydata[i+1]-ydata[i]),(xdata[i+1]-xdata[i])) 0 0]';
    end
    #println("qStart : ", qStart)
    #println("qEnd : ", qEnd)
    
    Y1,u_solved1,converged1,cost1=computeTrajectory(veh,qStart,qEnd,"xyp",Δt,steermax,steerdotmax,accmax);

    #=
    if converged1==0
        println("xyp converge fail")
        Y2,u_solved2,converged2,cost2=computeTrajectory(qStart,qEnd,"xy",Δt,steermax,steerdotmax,accmax);
        if converged2==0
            println("xy converge fail")
            if cost1<=cost2
                println("choose xyp u")
                println("u : ",u_solved1)
                model.commands=u_solved1
            else
                println("choose xy u")
                println("u : ",u_solved2)
                model.commands=u_solved2
            end
        else
            println("xy converge success")
            println("u : ",u_solved2)
            model.commands=u_solved2
        end
    else
        println("xyp converge success")
        println("u : ",u_solved1)
        model.commands=u_solved1
    end
    =#
    model.commands=u_solved1
end

function computeTrajectory(veh::Entity,initialXYPKV::Matrix{Float64},targetXYPKV::Matrix{Float64},mode::String,Δt::Float64,kmax::Float64,kdotmax::Float64,vdotmax::Float64)
    h=Δt
    N=2
    #x=sqrt((initialXYPKV[1]-targetXYPKV[1])^2+(initialXYPKV[2]-targetXYPKV[2])^2)
    #guess_a=2*(x-initialXYPKV[5]*h)/h^2
    #uguess = float([guess_a guess_a 0 0]')
    uguess = float([0 0 0 0]')
    Y,u_solved,converged,cost = solveTrajXYPKVLinear(veh,initialXYPKV,N,h,uguess,kmax,kdotmax,vdotmax,targetXYPKV,mode);
    #=
    if u_solved[3]*u_solved[4]<0.0 && converged==false
        println("u is ",u_solved)
        println("counter steer! average them!")
        u_solved[3]=u_solved[3]+u_solved[4]
        u_solved[4]=u_solved[3]
    end
    =#
    return Y,u_solved,converged,cost
end

function solveTrajXYPKVLinear(veh::Entity,q0::Matrix{Float64},N::Int,h::Float64,uguess::Matrix{Float64},kmax::Float64,kdotmax::Float64,vdotmax::Float64,targetXYPKV::Matrix{Float64},constraint_mode::String)
    
    w=copy(uguess)
    stepsize_multiplier = 0.5
    converged = false
    bestCost=Inf
    bestY=zeros(N+1,5);
    bestU=zeros(4,1);
    for i=1:500
        #println("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        #println("w is ",w)
        u=copy(w)
        Y,dY = simXYPKVLinear(veh,q0,N,h,u,kmax,kdotmax,vdotmax)
        if constraint_mode=="xypkv"
            index=1:5
        elseif constraint_mode=="xypv"
            index=[1,2,3,5]
        elseif constraint_mode== "xyv"
            index=[1,2,5]
        elseif constraint_mode== "xyp"
            index=[1,2,3]
        elseif constraint_mode== "xy"
            index=[1,2]
        else
            index=[1,2]
        end
        f = Y[end,index] - targetXYPKV[index]
        
        cost=0;
        for j=1:length(index)
            if index[j]==3 || index[j]==4
                fjs=[f[j] f[j]+2*pi f[j]-2*pi];
                (abs_fj,ind_fj)=findmin(abs(fjs))
                f[j]=fjs[ind_fj]
            end
            cost=cost+f[j]^2
        end
        dfdw = dY[index,:,end]
        #cost=maximum(abs(f))
        dcostdw=zeros(4,1)
        for j=1:length(index)
            #println("j = ",j," f[j] is ",f[j])
            #normalize dfdw
            ####dfdw[j,:]=dfdw[j,:]/sqrt(dfdw[j,:]'*dfdw[j,:])
            #println("dfdw is ",dfdw[j,:])
            #dcostdw=dcostdw+2*f[j]*dY[index[j],:,end]
            dcostdw=dcostdw+2*f[j]*dfdw[j,:]
        end
        
        if cost<bestCost
            bestCost=cost
            bestY=copy(Y)
            bestU=copy(u)
        end
        if cost<0.0001
            converged=true
            break
        end
        
        alpha=0.01*[vdotmax vdotmax kmax kmax]'
        #lambda=0.01*[vdotmax vdotmax kmax kmax]'
        lambda=0.001*[vdotmax vdotmax kmax kmax]'
        #cost_constrain=-log((w[1]-vdotmax)^2)-log((w[1]+vdotmax)^2)-log((w[2]-vdotmax)^2)-log((w[2]+vdotmax)^2)-log((w[3]-kmax)^2)-log((w[3]+kmax)^2)-log((w[4]-kmax)^2)-log((w[4]+kmax)^2);
        dcost_constraindw=zeros(4,1);
        for j=1:4
            if j<3 && abs(w[j])>vdotmax*0.0
                dcost_constraindw[j]=-2/(w[j]-vdotmax)-2/(w[j]+vdotmax)
                if w[j]>(vdotmax-0.1)
                    #println("command ",j," is ",w[j])
                    dcost_constraindw[j]=-2/(-0.1)
                elseif w[j]<(-vdotmax+0.1)
                    #println("command ",j," is ",w[j])
                    dcost_constraindw[j]=-2/(0.1)
                end
            elseif j>2 && abs(w[j])>kmax*0.0
                dcost_constraindw[j]=-2/(w[j]-kmax)-2/(w[j]+kmax)
                if w[j]>(kmax-0.1)
                    #println("command ",j," is ",w[j])
                    dcost_constraindw[j]=-2/(-0.1)
                elseif w[j]<(-kmax+0.1)
                    #println("command ",j," is ",w[j])
                    dcost_constraindw[j]=-2/(0.1)
                end
            end
        end
        #=
        println("cost at i = ",i," is ",cost)
        println("w is ",w)
        println("dcostdw is ",dcostdw)
        println("dcost_constraindw is ",lambda.*dcost_constraindw)
        =#
        gradient=dcostdw+lambda.*dcost_constraindw;
        #normalize whole gradient
        gradient=gradient/sqrt(gradient'*gradient)
        #=
        if sqrt(gradient'*gradient)[1]*alpha > min(vdotmax,kmax)
            println("gradient too large, is ",sqrt(gradient'*gradient)[1]*alpha)
            gradient=gradient/sqrt(gradient'*gradient)
        end
        =#
        #println("gradiet is : ",alpha.*gradient)
        w = w - alpha.*gradient;
    end
    Y=bestY;
    
    if  bestU[1]!=(bestU[1]=customclamp(bestU[1],-vdotmax,vdotmax))
        converged=false
    end
    if bestU[2]!=(bestU[2]=customclamp(bestU[2],-vdotmax,vdotmax))
        converged=false
    end
    if bestU[3]!=(bestU[3]=customclamp(bestU[3],-kmax,kmax))
        converged=false
    end
    if bestU[4]!=(bestU[4]=customclamp(bestU[4],-kmax,kmax)) 
        converged=false
    end  
    #=
    @assert bestU[1]==(bestU[1]=customclamp(bestU[1],-vdotmax,vdotmax))

    @assert bestU[2]==(bestU[2]=customclamp(bestU[2],-vdotmax,vdotmax))

    @assert bestU[3]==(bestU[3]=customclamp(bestU[3],-kmax,kmax))

    @assert bestU[4]==(bestU[4]=customclamp(bestU[4],-kmax,kmax)) 
    =#
    u=bestU;
    uvec=u;
    return Y,uvec,converged,bestCost
end

function simXYPKVLinear(veh::Entity,q0::Matrix{Float64},N::Int,h::Float64,u::Matrix{Float64},kmax::Float64,kdotmax::Float64,vdotmax::Float64)
    M = length(u);
    Y = zeros(N+1,5);
    Y[1,:] = q0;
    #u=[a1 a2 k1 k2]
    # Derivative of [each state at each time step] with respect to [each decision variable]
    dY = zeros(5,M,N+1);

    # Simulate forward N timesteps
    for i=1:N
        if i==1
            steer = u[3];
            acc = u[1];
            dsteerdu=[0 0 1 0];
            daccdu=[1 0 0 0];
        else
            steer = u[4];
            acc = u[2];
            dsteerdu=[0 0 0 1];
            daccdu=[0 1 0 0];
        end
    
    
        # Max change in curvature (i.e. discrete limit on kdot)
        kdeltamax = kdotmax*h;    
    
        # h (timestep) multiplied by f(x), where xdot=f(x)
        L = veh.def.a + veh.def.b
        l = -veh.def.b
        
        x=Y[i,1];
        dxdu=dY[1,:,i];
        dxdu=reshape(dxdu,1,M)
        y=Y[i,2];
        dydu=dY[2,:,i];
        dydu=reshape(dydu,1,M)
        theta=Y[i,3];
        dthetadu=dY[3,:,i];
        dthetadu=reshape(dthetadu,1,M)
        v=Y[i,5];
        dvdu=dY[5,:,i];
        dvdu=reshape(dvdu,1,M)
        acc=customclamp(acc,-vdotmax,vdotmax);
        steer=Y[i,4]+customclamp(steer-Y[i,4],-kdeltamax,kdeltamax);
        steer=customclamp(steer,-kmax,kmax);
        s=v*h+0.5*acc*h^2;
        dsdu=dvdu*h+0.5*h^2*daccdu;
        if(abs(steer)<0.01)
         # Integrate dynamics
            theta_n=theta+steer*v*h;
            theta_n=mod(theta_n,2*pi)
            dtheta_ndu=dthetadu+dsteerdu*v*h+steer*dvdu*h;            
            
            Y[i+1,1] = x+s*cos(theta_n);
            Y[i+1,2] = y+s*sin(theta_n);
            Y[i+1,3] = theta_n;
            Y[i+1,4] = steer;
            Y[i+1,5] = v+acc*h; 
    
            #calculate derivative
            dY[1,:,i+1]=dxdu+dsdu*cos(theta_n)-s*sin(theta_n)*dtheta_ndu;
            dY[2,:,i+1]=dydu+dsdu*sin(theta_n)+s*cos(theta_n)*dtheta_ndu;
            dY[3,:,i+1]=dtheta_ndu;#dthetadu+dsteerdu*v*h+steer*dvdu*h;
            dY[4,:,i+1]=dsteerdu;
            dY[5,:,i+1]=dvdu+daccdu*h;
        else
            R=L/tan(steer);
            dRdu=-L*tan(steer)^(-2)*sec(steer)^2*dsteerdu;
            beta=s/R;
            dbetadu=(dsdu*R-s*dRdu)/R^2;
            xc=x-R*sin(theta)+l*cos(theta);
            dxcdu=dxdu-dRdu*sin(theta)-R*cos(theta)*dthetadu-l*sin(theta)*dthetadu;
            yc=y+R*cos(theta)+l*sin(theta);
            dycdu=dydu+dRdu*cos(theta)-R*sin(theta)*dthetadu+l*cos(theta)*dthetadu;
            theta_n=mod(theta+beta,2*pi);
            dtheta_ndu=dthetadu+dbetadu;
            # Integrate dynamics
    
            Y[i+1,1] = xc+R*sin(theta_n)-l*cos(theta_n);
            Y[i+1,2] = yc-R*cos(theta_n)-l*sin(theta_n);
            Y[i+1,3] = theta_n;
            Y[i+1,4] = steer;
            Y[i+1,5] = v+acc*h; 
    
            #calculate derivative
            dY[1,:,i+1]=dxcdu+dRdu*sin(theta_n)+R*cos(theta_n)*dtheta_ndu+l*sin(theta_n)*dtheta_ndu;
            dY[2,:,i+1]=dycdu-dRdu*cos(theta_n)+R*sin(theta_n)*dtheta_ndu-l*cos(theta_n)*dtheta_ndu;
            dY[3,:,i+1]=dtheta_ndu;
            dY[4,:,i+1]=dsteerdu;
            dY[5,:,i+1]=dvdu+daccdu*h;
        end
    end
    return Y,dY
end

function customclamp(x::Float64,xmin::Float64,xmax::Float64)
    xout = max(min(x,xmax),xmin);
    xout
end