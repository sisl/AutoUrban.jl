export get_commands!

#####under developing

function get_commands!(model::MultiPtsDriver,scene::Union{Scene,Frame{Entity{VehicleState, BicycleModel, Int}}}, roadway::Roadway,ego_index::Int)
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
        qEnd=[xdata[i] ydata[i] atan((ydata[i]-ydata[i-1]),(xdata[i]-xdata[i-1])) 0 0]';  
    else
        qEnd=[xdata[i] ydata[i] atan((ydata[i+1]-ydata[i]),(xdata[i+1]-xdata[i])) 0 0]';
    end
    println("qStart : ", qStart)
    println("qEnd : ", qEnd)
    
    Y1,u_solved1,converged1,cost1=computeTrajectory(qStart,qEnd,"xyp",Δt,steermax,steerdotmax,accmax);

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
end

function computeTrajectory(initialXYPKV::Matrix{Float64},targetXYPKV::Matrix{Float64},mode::String,Δt::Float64,kmax::Float64,kdotmax::Float64,vdotmax::Float64)
    h=Δt
    N=2
    #x=sqrt((initialXYPKV[1]-targetXYPKV[1])^2+(initialXYPKV[2]-targetXYPKV[2])^2)
    #guess_a=2*(x-initialXYPKV[5]*h)/h^2
    #uguess = float([guess_a guess_a 0 0]')
    uguess = float([0 0 0 0]')
    Y,u_solved,converged,cost = solveTrajXYPKVLinear(initialXYPKV,N,h,uguess,kmax,kdotmax,vdotmax,targetXYPKV,mode);
    return Y,u_solved,converged,cost
end

function solveTrajXYPKVLinear(q0::Matrix{Float64},N::Int,h::Float64,uguess::Matrix{Float64},kmax::Float64,kdotmax::Float64,vdotmax::Float64,targetXYPKV::Matrix{Float64},constraint_mode::String)
    w=copy(uguess)
    converged = false
    bestCost=Inf
    bestY=zeros(N+1,5);
    bestU=zeros(4,1);
    for i=1:500
        u=copy(w)
        Y,dY,H = simXYPKVLinear(q0,N,h,u,kmax,kdotmax,vdotmax)
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
        Hf = H[index,end,:,:]

        dcostdw=zeros(4,1)
        Hcost=zeros(4,4)
        for j=1:length(index)
            #println("j = ",j," f[j] is ",f[j])
            #normalize dfdw
            ####dfdw[j,:]=dfdw[j,:]/sqrt(dfdw[j,:]'*dfdw[j,:])
            #println("dfdw is ",dfdw[j,:])
            #dcostdw=dcostdw+2*f[j]*dY[index[j],:,end]
            dcostdw=dcostdw+2*f[j]*dfdw[j,:]
            Hcost=Hcost+2*dfdw[j,:]*dfdw[j,:]'+2*f[j]*Hf[j,:,:]
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
        
        alpha=1.0
        lambda=10.0
        maxstep =0.4
        #delta=pinv(dfdw)*f
        println("w at i =,", i," is ",w)
        println("cost is ",cost)
        println("dcostdw is ",dcostdw)
        println("Hcost is ",Hcost)
        println("inverse Hcost is",pinv(Hcost))
        println("newton gradient is ",pinv(Hcost)*dcostdw)
        println("dcost_constraindw is ",lambda*dcost_constraindw)
        gradient = pinv(Hcost)*dcostdw + lambda*dcost_constraindw
        println("gradiet is : ",gradient)
        if sqrt(gradient'*gradient)[1] > maxstep
            println("gradient too large")
            gradient = (gradient/sqrt(gradient'*gradient))*maxstep
            println("gradiet is : ",gradient)
        end
        w = w - gradient;
        #w = w - delta;
    end
    Y=bestY;

    if  bestU[1]!=(bestU[1]=clamp(bestU[1],-vdotmax,vdotmax))
        converged=false
    end
    if bestU[2]!=(bestU[2]=clamp(bestU[2],-vdotmax,vdotmax))
        converged=false
    end
    if bestU[3]!=(bestU[3]=clamp(bestU[3],-kmax,kmax))
        converged=false
    end
    if bestU[4]!=(bestU[4]=clamp(bestU[4],-kmax,kmax)) 
        converged=false
    end  
    
    #=
    @assert bestU[1]==(bestU[1]=clamp(bestU[1],-vdotmax,vdotmax))

    @assert bestU[2]==(bestU[2]=clamp(bestU[2],-vdotmax,vdotmax))

    @assert bestU[3]==(bestU[3]=clamp(bestU[3],-kmax,kmax))

    @assert bestU[4]==(bestU[4]=clamp(bestU[4],-kmax,kmax)) 
    =#
    u=bestU;
    uvec=u;
    return Y,uvec,converged,bestCost
end

function simXYPKVLinear(q0::Matrix{Float64},N::Int,h::Float64,u::Matrix{Float64},kmax::Float64,kdotmax::Float64,vdotmax::Float64)
    M = length(u);
    Y = zeros(N+1,5);
    Y[1,:] = q0;

    # Derivative of [each state at each time step] with respect to [each decision variable]
    dY = zeros(5,M,N+1);
    H = zeros(5,N+1,M,M);

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
        
        Hsteer=zeros(M,M);
        Hacc=zeros(M,M);
    
    
        # Max change in curvature (i.e. discrete limit on kdot)
        kdeltamax = kdotmax*h;    
    
        # h (timestep) multiplied by f(x), where xdot=f(x)
        geom=BicycleGeom()
        L = geom.wheel_base
        l = geom.wheel_base_offset
        x=Y[i,1]
        dxdu=dY[1,:,i]
        dxdu=reshape(dxdu,1,M)
        
        Hx=H[1,i,:,:]
        Hx=reshape(Hx,M,M)
        
        y=Y[i,2]
        dydu=dY[2,:,i]
        dydu=reshape(dydu,1,M)
        
        Hy=H[2,i,:,:]
        Hy=reshape(Hy,M,M)
        
        theta=Y[i,3];
        dthetadu=dY[3,:,i];
        dthetadu=reshape(dthetadu,1,M)
        
        Htheta=H[3,i,:,:]
        Htheta=reshape(Htheta,M,M)
        
        v=Y[i,5];
        dvdu=dY[5,:,i];
        dvdu=reshape(dvdu,1,M)
        
        Hv=H[5,i,:,:]
        Hv=reshape(Hv,M,M)
        
        acc=clamp(acc,-vdotmax,vdotmax);
        steer=Y[i,4]+clamp(steer-Y[i,4],-kdeltamax,kdeltamax);
        steer=clamp(steer,-kmax,kmax);
        s=v*h+0.5*acc*h^2;
        dsdu=dvdu*h+0.5*h^2*daccdu;
        
        Hs=Hv*h+0.5*h^2*Hacc;
        
        if(abs(steer)<0.01)
         # Integrate dynamics
            theta_n=theta+steer*v*h;
            dtheta_ndu=dthetadu+dsteerdu*v*h+steer*dvdu*h;
            
            Htheta_n=Htheta+h*(Hsteer*v+steer*Hv+dsteerdu'*dvdu+dvdu'*dsteerdu)
            
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
            
            Hcostheta_n=-cos(theta_n)*dtheta_ndu'*dtheta_ndu-sin(theta_n)*Htheta_n
            Hsintheta_n=-sin(theta_n)*dtheta_ndu'*dtheta_ndu+cos(theta_n)*Htheta_n
            dcostheta_ndu=-sin(theta_n)*dtheta_ndu
            dsintheta_ndu=cos(theta_n)*dtheta_ndu
            H[1,i+1,:,:]=Hx+Hs*cos(theta_n)+s*Hcostheta_n+dsdu'*dcostheta_ndu+dcostheta_ndu'*dsdu
            H[2,i+1,:,:]=Hy+Hs*sin(theta_n)+s*Hsintheta_n+dsdu'*dsintheta_ndu+dsintheta_ndu'*dsdu
            H[3,i+1,:,:]=Htheta_n
            H[4,i+1,:,:]=Hsteer
            H[5,i+1,:,:]=Hv+Hacc*h
            
        else
            R=L/tan(steer);
            dRdu=-L/sin(steer)^2*dsteerdu;
            HR=-L/sin(steer)^2*Hsteer+2*L/sin(steer)^3*cos(steer)*dsteerdu'*dsteerdu
            
            beta=s/R;
            dbetadu=(dsdu*R-s*dRdu)/R^2;
            Hbeta=(Hs*R^3-(dsdu'*dRdu+dRdu'*dsdu)*R^2-HR*s*R^2+dRdu'*dRdu*2*R*s)/R^4
            
            xc=x-R*sin(theta)+l*cos(theta);
            dxcdu=dxdu-dRdu*sin(theta)-R*cos(theta)*dthetadu-l*sin(theta)*dthetadu;
            Hcostheta=-cos(theta)*dthetadu'*dthetadu-sin(theta)*Htheta
            Hsintheta=-sin(theta)*dthetadu'*dthetadu+cos(theta)*Htheta
            dcosthetadu=-sin(theta)*dthetadu
            dsinthetadu=cos(theta)*dthetadu
            Hxc=Hx-(HR*sin(theta)+R*Hsintheta+dRdu'*dsinthetadu+dsinthetadu'*dRdu)+l*Hcostheta
            
            yc=y+R*cos(theta)+l*sin(theta);
            dycdu=dydu+dRdu*cos(theta)-R*sin(theta)*dthetadu+l*cos(theta)*dthetadu;
            Hyc=Hy+(HR*cos(theta)+R*Hcostheta+dRdu'*dcosthetadu+dcosthetadu'*dRdu)+l*Hsintheta
            
            theta_n=mod(theta+beta,2*pi);
            dtheta_ndu=dthetadu+dbetadu;
            Htheta_n=Htheta+Hbeta
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
            
            Hcostheta_n=-cos(theta_n)*dtheta_ndu'*dtheta_ndu-sin(theta_n)*Htheta_n
            Hsintheta_n=-sin(theta_n)*dtheta_ndu'*dtheta_ndu+cos(theta_n)*Htheta_n
            dcostheta_ndu=-sin(theta_n)*dtheta_ndu
            dsintheta_ndu=cos(theta_n)*dtheta_ndu
            H[1,i+1,:,:]=Hxc+HR*sin(theta_n)+R*Hsintheta_n+dRdu'*dsintheta_ndu+dsintheta_ndu'*dRdu-l*Hcostheta_n
            H[2,i+1,:,:]=Hyc-HR*cos(theta_n)-R*Hcostheta_n-dRdu'*dcostheta_ndu-dcostheta_ndu'*dRdu-l*Hsintheta_n
            H[3,i+1,:,:]=Htheta_n
            H[4,i+1,:,:]=Hsteer
            H[5,i+1,:,:]=Hv+Hacc*h
        end
    end
    return Y,dY,H
end