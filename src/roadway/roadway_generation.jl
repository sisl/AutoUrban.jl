const ANG_THD = 1e-2

mutable struct Connection
    source::Int
    dest::Int
    path::Int
    laneConnections::Array{Tuple{Int,Int}}
end

function Connection(source::Int,dest::Int)
    Connection(source,dest,0,Array{Tuple{Int,Int}}(0))
end

mutable struct Junction
    connections::Array{Connection}
end

function get_min_difference_angles(A::Float64,B::Float64)
    b=B
    as=[A A+2*pi A-2*pi];
    (abs_diff,ind_a)=findmin(abs(as-b))
    a=as[ind_a]
    return a,b,abs_diff
end

function get_min_difference_angles_pos(A::Float64,B::Float64)
    b = B
    as = [A A+2*pi A-2*pi];
    diffs = b - as
    pos_inds = find(x->x>=0,diffs)
    pos_as = as[pos_inds]
    pos_diffs = diffs[pos_inds]
    pos_diff,ind_a = findmin(abs(pos_diffs))
    a = pos_as[ind_a]
    return a,b,pos_diff
end

function get_min_difference_angles_neg(A::Float64,B::Float64)
    b = B
    as = [A A+2*pi A-2*pi];
    diffs = b - as
    neg_inds = find(x->x<=0,diffs)
    neg_as = as[neg_inds]
    neg_diffs = diffs[neg_inds]
    neg_diff,ind_a = findmin(abs(neg_diffs))
    neg_diff = - neg_diff
    a = neg_as[ind_a]
    return a,b,neg_diff
end


function get_intersection_point(A::VecSE2, B::VecSE2)
    x1 = A.x
    y1 = A.y
    x2 = B.x
    y2 = B.y
    θ1,θ2,_ = get_min_difference_angles(A.θ,B.θ)
    a1 = tan(θ1)
    a2 = tan(θ2)
    @assert a1 != a2
    @assert abs(a1) < 1e4 || abs(a2) < 1e4
    if abs(a1) < 1e4 && abs(a2) < 1e4
            x = -(-a1*x1+a2*x2+y1-y2)/(a1-a2)
            y = -(-a1*a2*x1+a1*a2*x2+a2*y1-a1*y2)/(a1-a2)
    else
        if abs(a1) > 1e4
            x = x1
            y = a2*x1-a2*x2+y2
        elseif abs(a2) > 1e4
            x = x2
            y = -a1*x1+a1*x2+y1
        end
    end
    #println("A ",A)
    #println("B ",B)
    #println("Intersection ",VecE2(x,y))
    return VecE2(x,y)
end

function get_center_of_circle(A::VecSE2, B::VecSE2)
    x1 = A.x
    y1 = A.y
    x2 = B.x
    y2 = B.y
    θ1,θ2,angle_diff = get_min_difference_angles(A.θ,B.θ)
    x = -((-x2*cos(θ2)*sin(θ1)+x1*cos(θ1)*sin(θ2)+y1*sin(θ1)*sin(θ2)-y2*sin(θ1)*sin(θ2))/(cos(θ2)*sin(θ1)-cos(θ1)*sin(θ2)))
    y = -((-x1*cos(θ1)*cos(θ2)+x2*cos(θ1)*cos(θ2)-y1*cos(θ2)*sin(θ1)+y2*cos(θ1)*sin(θ2))/(cos(θ2)*sin(θ1)-cos(θ1)*sin(θ2)))

    if abs(abs(angle_diff)-pi) < ANG_THD
        x = (x1+x2)/2
        y = (y1+y2)/2
    end
    return VecE2(x,y)
end

function connect_two_points_by_circle(A::VecSE2, B::VecSE2;s_base::Float64=0.0)
    ncurvepts_per_turn = 25

    distance = norm(VecE2(A-B))

    C = get_center_of_circle(A, B)
    CA = VecE2(A.x-C.x,A.y-C.y)
    CB = VecE2(B.x-C.x,B.y-C.y)

    if (CA.x>ANG_THD && sin(A.θ)>ANG_THD) || (CA.x<-ANG_THD && sin(A.θ)<-ANG_THD) || (CA.y<-ANG_THD && cos(A.θ)>ANG_THD) || (CA.y>ANG_THD && cos(A.θ)<-ANG_THD)
        attangle_a,attangle_b,angle_diff = get_min_difference_angles_pos(A.θ,B.θ)
        posangle_a,posangle_b,angle_diff = get_min_difference_angles_pos(atan2(CA.y,CA.x),atan2(CB.y,CB.x))
    else
        attangle_a,attangle_b,angle_diff = get_min_difference_angles_neg(A.θ,B.θ)
        posangle_a,posangle_b,angle_diff = get_min_difference_angles_neg(atan2(CA.y,CA.x),atan2(CB.y,CB.x))
    end

    curvepts = Array{CurvePt}(ncurvepts_per_turn)

    #@assert abs(angle_diff) > ANG_THD
    if abs(abs(angle_diff)-pi) < ANG_THD
        radius = distance/2
    else
        radius = sqrt(distance^2/(2*(1-cos(angle_diff))))
    end
    r = radius
    for j in 1:ncurvepts_per_turn
        t = (j-1)/(ncurvepts_per_turn-1) # ∈ [0,1]
        s = r*abs(angle_diff)*t+s_base
        k = 1.0/r
        if attangle_b < attangle_a
            k = -k
        end
        kd = 0.0
        curvepts[j] = CurvePt(VecSE2(C + polar(r, lerp(posangle_a,posangle_b, t)), lerp(attangle_a,attangle_b,t)),  s,k,kd)
    end
    total_length = r*abs(angle_diff)+s_base
    return curvepts,total_length
end

function connect_two_points(A::VecSE2, B::VecSE2)
    #connect any two points with one points array
    _,_,abs_diff = get_min_difference_angles(B.θ,A.θ)

    curvepts = Array{CurveP}(0)
    push!(curvepts,CurvePt(A,  0.0,0.0,0.0))
    if abs_diff < ANG_THD
        push!(curvepts,CurvePt(B,  norm(VecE2(A-B)),0.0,0.0))
    else
        if abs(abs_diff - pi) < ANG_THD
            #println("difference is pi")
            BA = VecE2(B.x-A.x,B.y-A.y)
            pojection_length = BA.x*cos(A.θ)+BA.y*sin(A.θ)
            if pojection_length > ANG_THD
                P = VecSE2(VecE2(A + polar(pojection_length,A.θ)),A.θ)
                push!(curvepts,CurvePt(P, pojection_length,0.0,0.0))
                circlepts,total_length = connect_two_points_by_circle(P, B;s_base = pojection_length)
                curvepts = vcat(curvepts,circlepts)
                push!(curvepts,CurvePt(B,total_length,0.0,0.0))
            elseif pojection_length < -ANG_THD
                P = VecSE2(VecE2(B + polar(pojection_length,B.θ)),B.θ)
                circlepts,total_length = connect_two_points_by_circle(A, P;s_base = 0.0)
                curvepts = vcat(curvepts,circlepts)
                push!(curvepts,CurvePt(P, total_length,0.0,0.0))
                push!(curvepts,CurvePt(B,total_length+abs(pojection_length),0.0,0.0))
            else #no need of P
                circlepts,total_length = connect_two_points_by_circle(A, B;s_base = 0.0)
                curvepts = vcat(curvepts,circlepts)
            end
        else
            #println("difference is general")
            I = get_intersection_point(A, B)
            dis1 = norm(VecE2(A-I))
            dis2 = norm(VecE2(B-I))
            AI = VecE2(I.x-A.x,I.y-A.y)
            BI = VecE2(I.x-B.x,I.y-B.y)
            dis_diff = abs(dis1-dis2)
            projection_length = AI.x*cos(A.θ)+ AI.y*sin(A.θ)
            projection_lengthB = BI.x*cos(B.θ)+ BI.y*sin(B.θ)
            if projection_length>=0 && projection_lengthB>=0
                #println("0")
                P = VecSE2(I+polar(projection_lengthB,A.θ),A.θ)
                push!(curvepts,CurvePt(P, projection_length+projection_lengthB,0.0,0.0))
                circlepts,total_length = connect_two_points_by_circle(P, B;s_base = projection_length+projection_lengthB)
                curvepts = vcat(curvepts,circlepts)
                push!(curvepts,CurvePt(B,total_length,0.0,0.0))
            elseif (dis1-dis2 >= 0.0 && projection_length >= 0.0) || (dis1-dis2 <= 0.0 && projection_length <= 0.0)
                #println("1")
                P = VecSE2(VecE2(A + polar(dis_diff,A.θ)),A.θ)
                push!(curvepts,CurvePt(P, dis_diff,0.0,0.0))
                circlepts,total_length = connect_two_points_by_circle(P, B;s_base = dis_diff)
                curvepts = vcat(curvepts,circlepts)
                push!(curvepts,CurvePt(B,total_length,0.0,0.0))
            elseif (dis1-dis2 >= 0.0 && projection_length <= 0.0) || (dis1-dis2 <= 0.0 && projection_length >= 0.0)
                #println("2")
                P = VecSE2(VecE2(B + polar(-dis_diff,B.θ)),B.θ)
                circlepts,total_length = connect_two_points_by_circle(A, P;s_base = 0.0)
                curvepts = vcat(curvepts,circlepts)
                push!(curvepts,CurvePt(P, total_length,0.0,0.0))
                push!(curvepts,CurvePt(B,total_length+dis_diff,0.0,0.0))
            #else
            #    println("3")
            #    circlepts,total_length = connect_two_points_by_circle(A, B;s_base = 0.0)
            #    curvepts = vcat(curvepts,circlepts)
            end
        end
    end
    return curvepts
end

function connect_two_points_seperate(A::VecSE2, B::VecSE2)
    #connect any two points with an array of curpve points
    _,_,abs_diff = get_min_difference_angles(B.θ,A.θ)

    curvepts = Array{CurvePt}[]
    if abs_diff < ANG_THD
        push!(curvepts,[CurvePt(A,  0.0,0.0,0.0)])
        push!(curvepts[1],CurvePt(B,  norm(VecE2(A-B)),0.0,0.0))
    else
        if abs(abs_diff - pi) < ANG_THD
            #println("difference is pi")
            BA = VecE2(B.x-A.x,B.y-A.y)
            pojection_length = BA.x*cos(A.θ)+BA.y*sin(A.θ)
            if pojection_length > ANG_THD
                push!(curvepts,[CurvePt(A,  0.0,0.0,0.0)])
                P = VecSE2(VecE2(A + polar(pojection_length,A.θ)),A.θ)
                push!(curvepts[1],CurvePt(P, pojection_length,0.0,0.0))
                circlepts,total_length = connect_two_points_by_circle(P, B;s_base = 0.0)
                push!(curvepts,circlepts)
                push!(curvepts[2],CurvePt(B,total_length,0.0,0.0))
            elseif pojection_length < -ANG_THD
                P = VecSE2(VecE2(B + polar(pojection_length,B.θ)),B.θ)
                circlepts,total_length = connect_two_points_by_circle(A, P;s_base = 0.0)
                push!(curvepts,circlepts)
                push!(curvepts,[CurvePt(P, 0.0,0.0,0.0)])
                push!(curvepts[2],CurvePt(B,abs(pojection_length),0.0,0.0))
            else #no need of P
                circlepts,total_length = connect_two_points_by_circle(A, B;s_base = 0.0)
                push!(curvepts,circlepts)
            end
        else
            #println("difference is general")
            I = get_intersection_point(A, B)
            dis1 = norm(VecE2(A-I))
            dis2 = norm(VecE2(B-I))
            AI = VecE2(I.x-A.x,I.y-A.y)
            BI = VecE2(I.x-B.x,I.y-B.y)
            dis_diff = abs(dis1-dis2)
            projection_length = AI.x*cos(A.θ)+ AI.y*sin(A.θ)
            projection_lengthB = BI.x*cos(B.θ)+ BI.y*sin(B.θ)
            if dis_diff < ANG_THD
                circlepts,total_length = connect_two_points_by_circle(A, B;s_base = 0.0)
                push!(curvepts,circlepts)
            else
                if projection_length>=0 && projection_lengthB>=0
                    #println("0")
                    push!(curvepts,[CurvePt(A,  0.0,0.0,0.0)])
                    P = VecSE2(I+polar(projection_lengthB,A.θ),A.θ)
                    push!(curvepts[1],CurvePt(P, projection_length+projection_lengthB,0.0,0.0))
                    circlepts,total_length = connect_two_points_by_circle(P, B;s_base = 0.0)
                    push!(curvepts,circlepts)
                elseif (dis1-dis2 >= 0.0 && projection_length >= 0.0) || (dis1-dis2 <= 0.0 && projection_length <= 0.0)
                    #println("1")
                    push!(curvepts,[CurvePt(A,  0.0,0.0,0.0)])
                    P = VecSE2(VecE2(A + polar(dis_diff,A.θ)),A.θ)
                    push!(curvepts[1],CurvePt(P, dis_diff,0.0,0.0))
                    circlepts,total_length = connect_two_points_by_circle(P, B;s_base = 0.0)
                    push!(curvepts,circlepts)
                elseif (dis1-dis2 >= 0.0 && projection_length <= 0.0) || (dis1-dis2 <= 0.0 && projection_length >= 0.0)
                    #println("2")
                    P = VecSE2(VecE2(B + polar(-dis_diff,B.θ)),B.θ)
                    circlepts,total_length = connect_two_points_by_circle(A, P;s_base = 0.0)
                    push!(curvepts,circlepts)
                    push!(curvepts,[CurvePt(P, 0.0,0.0,0.0)])
                    push!(curvepts[2],CurvePt(B,dis_diff,0.0,0.0))
                end
            end
        end
    end
    return curvepts
end


function connect_two_lane_general!(source::Lane, dest::Lane,seg_id::Int,lane_width::Float64;
    boundary_left::LaneBoundary=LaneBoundary(:solid, :white),
    boundary_right::LaneBoundary=LaneBoundary(:solid, :white),
    boundary_middle::LaneBoundary=LaneBoundary(:solid, :white)
    )
    cindS = curveindex_end(source.curve)
    cindD = CURVEINDEX_START
    A = source.curve[cindS].pos
    B = dest.curve[cindD].pos
    seg12 = RoadSegment(seg_id, Array{Lane}(1))
    curvepts = connect_two_points(A, B)
    tag12 = LaneTag(seg_id,1)

    seg12.lanes[1] = Lane(tag12, curvepts, width=lane_width,
                                        boundary_left=boundary_left, boundary_right=boundary_right,
                                        )
    connect!(source, seg12.lanes[1])
    connect!(seg12.lanes[1], dest)
    return seg12
end

function connect_two_lane_general!(source::Lane, dest::Lane,roadway::Roadway,lane_width::Float64;
    boundary_left::LaneBoundary=LaneBoundary(:solid, :white),
    boundary_right::LaneBoundary=LaneBoundary(:solid, :white),
    boundary_middle::LaneBoundary=LaneBoundary(:solid, :white)
    )
    cindS = curveindex_end(source.curve)
    cindD = CURVEINDEX_START
    A = source.curve[cindS].pos
    B = dest.curve[cindD].pos
    curveptss = connect_two_points_seperate(A::VecSE2, B::VecSE2)
    segs = []
    seg_id = length(roadway.segments)
    for curvepts in curveptss
        seg_id += 1
        seg12 = RoadSegment(seg_id, Array{Lane}(1))
        tag12 = LaneTag(seg_id,1)
        seg12.lanes[1] = Lane(tag12, curvepts, width=lane_width,
                                        boundary_left=boundary_left, boundary_right=boundary_right,
                                        )
        push!(segs,seg12)
    end

    before = source
    next = nothing
    for i = 1:length(segs)
        connect!(before,segs[i].lanes[1])
        if i == length(segs)
            next = dest
        else
            next = segs[i+1].lanes[1]
        end
        connect!(segs[i].lanes[1],next)
        before = segs[i].lanes[1]
    end
    for i = 1:length(segs)
        push!(roadway.segments, segs[i])
    end
end

function connect_two_seg!(source::RoadSegment,dest::RoadSegment,roadway::Roadway;
    connections::Array{Tuple{Int,Int}}=Array{Tuple{Int,Int}}(0),
    boundary_left::LaneBoundary=LaneBoundary(:solid, :white),
    boundary_right::LaneBoundary=LaneBoundary(:solid, :white),
    boundary_middle::LaneBoundary=LaneBoundary(:solid, :white)
    )
    if isempty(connections)
        for i=1:length(source.lanes)
            push!(connections,(i,i))
        end
    end
    curveptsss = []
    for i = 1:length(connections)
        cindS = curveindex_end(source.lanes[connections[i][1]].curve)
        cindD = CURVEINDEX_START
        A = source.lanes[connections[i][1]].curve[cindS].pos
        B = dest.lanes[connections[i][2]].curve[cindD].pos
        curveptss = connect_two_points_seperate(A::VecSE2, B::VecSE2)
        push!(curveptsss,curveptss)
    end
    segs = []
    seg_id = length(roadway.segments)

    for i = 1:length(curveptsss[1])
        seg_id += 1
        seg12 = RoadSegment(seg_id, Array{Lane}(length(connections)))
        for j =  1:length(connections)
            tag12 = LaneTag(seg_id,j)
            seg12.lanes[j] = Lane(tag12, curveptsss[j][i], width=source.lanes[connections[j][1]].width,
                                        boundary_left=boundary_left, boundary_right=boundary_right,
                                        )
        end
        push!(segs,seg12)
    end


    for i = 1:length(connections)
        before = source.lanes[connections[i][1]]
        next = segs[1].lanes[i]
        connect!(before,next)
        before = next
        for j = 1:length(segs)
            if j == length(segs)
                next = dest.lanes[connections[i][2]]
            else
                next = segs[j+1].lanes[i]
            end
            connect!(before,next)
            before = next
        end
    end
    for i = 1:length(segs)
        push!(roadway.segments, segs[i])
    end

end

function connect_two_seg_general!(source::RoadSegment,dest::RoadSegment,roadway::Roadway;
    direction::Tuple{Int,Int}=(1,1),
    connections::Array{Tuple{Int,Int}}=Array{Tuple{Int,Int}}(0),
    boundary_left::LaneBoundary=LaneBoundary(:solid, :white),
    boundary_right::LaneBoundary=LaneBoundary(:solid, :white),
    boundary_middle::LaneBoundary=LaneBoundary(:solid, :white)
    )
    if isempty(connections)
        for i=1:length(source.lanes)
            push!(connections,(i,i))
        end
    end
    curveptsss = []
    for i = 1:length(connections)
        cindS = curveindex_end(source.lanes[connections[i][1]].curve)
        if direction[1]==-1
            cindS = CURVEINDEX_START
        end
        print()
        cindD = CURVEINDEX_START
        if direction[2]==-1
            cindD = curveindex_end(dest.lanes[connections[i][2]].curve)
        end
        A = source.lanes[connections[i][1]].curve[cindS].pos
        if direction[1]==-1
            A = VecSE2(A.x,A.y,mod2pi(A.θ+pi))
        end
        print(A)
        B = dest.lanes[connections[i][2]].curve[cindD].pos
        if direction[2]==-1
            B = VecSE2(B.x,B.y,mod2pi(B.θ+pi))
        end
        print(B)
        curveptss = connect_two_points_seperate(A::VecSE2, B::VecSE2)
        push!(curveptsss,curveptss)
    end
    segs = []
    seg_id = length(roadway.segments)

    for i = 1:length(curveptsss[1])
        seg_id += 1
        seg12 = RoadSegment(seg_id, Array{Lane}(length(connections)))
        for j =  1:length(connections)
            tag12 = LaneTag(seg_id,j)
            seg12.lanes[j] = Lane(tag12, curveptsss[j][i], width=source.lanes[connections[j][1]].width,
                                        boundary_left=boundary_left, boundary_right=boundary_right,
                                        )
        end
        push!(segs,seg12)
    end

    for i = 1:length(connections)
        before = source.lanes[connections[i][1]]
        next = segs[1].lanes[i]
        if direction[1] == 1
            connect_general!(before,next,(1,1))
        elseif direction[1] == -1
            connect_general!(before,next,(-1,1))
        end
        before = next
        for j = 1:length(segs)
            if j == length(segs)
                next = dest.lanes[connections[i][2]]
                if direction[2] == -1
                    connect_general!(before,next,(1,-1))
                elseif direction[1] == 1
                    connect_general!(before,next,(1,1))
                end
            else
                next = segs[j+1].lanes[i]
                connect_general!(before,next,(1,1))
            end
            before = next
        end
    end

    for i = 1:length(segs)
        push!(roadway.segments, segs[i])
    end

end

function connect_general!(source::Lane, dest::Lane,direction::Tuple{Int,Int}=(1,1))
    # place these at the front
    if direction[1]==1 && direction[2] ==1
        cindS = curveindex_end(source.curve)
        cindD = CURVEINDEX_START

        unshift!(source.exits,   LaneConnection(true,  cindS, RoadIndex(cindD, dest.tag)))
        unshift!(dest.entrances, LaneConnection(false, cindD, RoadIndex(cindS, source.tag)))
        (source, dest)
    elseif direction[1]==1 && direction[2] ==-1
        cindS = curveindex_end(source.curve)
        cindD = curveindex_end(dest.curve)

        unshift!(source.exits,   LaneConnection(true,  cindS, RoadIndex(cindD, dest.tag)))
        unshift!(dest.exits, LaneConnection(true, cindD, RoadIndex(cindS, source.tag)))
        (source, dest)
    elseif direction[1]==-1 && direction[2] ==1
        cindS = CURVEINDEX_START
        cindD = CURVEINDEX_START

        unshift!(source.entrances,   LaneConnection(false,  cindS, RoadIndex(cindD, dest.tag)))
        unshift!(dest.entrances, LaneConnection(false, cindD, RoadIndex(cindS, source.tag)))
        (source, dest)
    elseif direction[1]==-1 && direction[2] ==-1
        cindS = CURVEINDEX_START
        cindD = curveindex_end(dest.curve)

        unshift!(source.entrances,   LaneConnection(false,  cindS, RoadIndex(cindD, dest.tag)))
        unshift!(dest.exits, LaneConnection(true, cindD, RoadIndex(cindS, source.tag)))
        (source, dest)
    end
end

function add_connection!(connection::Connection,roadway::Roadway;
    boundary_left::LaneBoundary=LaneBoundary(:solid, :white),
    boundary_right::LaneBoundary=LaneBoundary(:solid, :white),
    boundary_middle::LaneBoundary=LaneBoundary(:solid, :white)
    )
    source = roadway.segments[connection.source]
    dest = roadway.segments[connection.dest]
    if isempty(connection.laneConnections)
        for i=1:length(source.lanes)
            push!(connection.laneConnections,(i,i))
        end
    end
    curveptsss = []
    for i = 1:length(connection.laneConnections)
        cindS = curveindex_end(source.lanes[connection.laneConnections[i][1]].curve)
        cindD = CURVEINDEX_START
        A = source.lanes[connection.laneConnections[i][1]].curve[cindS].pos
        B = dest.lanes[connection.laneConnections[i][2]].curve[cindD].pos
        curveptss = connect_two_points_seperate(A, B)
        push!(curveptsss,curveptss)
    end
    segs = []
    seg_id = length(roadway.segments)

    for i = 1:length(curveptsss[1])
        seg_id += 1
        seg12 = RoadSegment(seg_id, Array{Lane}(length(connection.laneConnections)))
        if i==1
            connection.path = seg_id
        end
        for j =  1:length(connection.laneConnections)
            tag12 = LaneTag(seg_id,j)
            seg12.lanes[j] = Lane(tag12, curveptsss[j][i], width=source.lanes[connection.laneConnections[j][1]].width,
                                        boundary_left=boundary_left, boundary_right=boundary_right,
                                        )
        end
        push!(segs,seg12)
    end


    for i = 1:length(connection.laneConnections)
        before = source.lanes[connection.laneConnections[i][1]]
        next = nothing
        for j = 1:length(segs)
            connect!(before,segs[j].lanes[i])
            if j == length(segs)
                next = dest.lanes[connection.laneConnections[i][2]]
            else
                next = segs[j+1].lanes[i]
            end
            connect!(segs[j].lanes[i],next)
            before = segs[j].lanes[1]
        end
    end
    for i = 1:length(segs)
        push!(roadway.segments, segs[i])
    end

end


function add_junction!(junction::Junction,roadway::Roadway)
    for connection in junction.connections
        add_connection!(connection,roadway)
    end
end

function gen_connected_lanes(;nlanes::Int=1,roadlength::Float64=5.0,
    lane_width::Float64=DEFAULT_LANE_WIDTH, # [m]
    lane_widths::Vector{Float64} = fill(lane_width, nlanes),
    origin1::VecSE2 = VecSE2(0.0,0.0,0.0),
    origin2::VecSE2 = VecSE2(origin1.x+roadlength,origin1.y-roadlength,1.25*pi),
    boundary_leftmost::LaneBoundary=LaneBoundary(:solid, :white),
    boundary_rightmost::LaneBoundary=LaneBoundary(:solid, :white),
    boundary_middle::LaneBoundary=LaneBoundary(:broken, :white)
    )


    segments = RoadSegment[]
    seg1 = gen_straight_segment(1, nlanes, roadlength,
                                                origin=origin1, lane_widths=lane_widths,
                                                boundary_leftmost=boundary_leftmost,
                                                boundary_rightmost=boundary_rightmost,
                                                boundary_middle=boundary_middle)
    seg2 = gen_straight_segment(2, nlanes, roadlength,
                                                origin=origin2, lane_widths=lane_widths,
                                                boundary_leftmost=boundary_leftmost,
                                                boundary_rightmost=boundary_rightmost,
                                                boundary_middle=boundary_middle)

    retval = Roadway()
    push!(retval.segments, seg1)
    push!(retval.segments, seg2)

    boundary_left =  boundary_middle
    boundary_right = boundary_middle

    connect_two_seg!(retval.segments[1],retval.segments[2],retval)
    retval
end


function gen_intersection(;nlanes::Int=2,roadlength::Float64=5.0,
    lane_width::Float64=DEFAULT_LANE_WIDTH, # [m]
    lane_widths::Vector{Float64} = fill(lane_width, nlanes),
    crosswidth::Float64=nlanes*lane_width,
    origin::VecSE2 = VecSE2(0.0,0.0,0.0),
    #connections::Array{Tuple{Int,Int}}=[(1,6),(3,8),(5,2),(7,4),(1,2),(3,4),(5,6),(7,8),(1,4),(3,6),(5,8),(7,2)],
    junction::Junction=Junction([Connection(1,6),Connection(3,8),Connection(5,2),Connection(7,4),Connection(1,2),Connection(3,4),Connection(5,6),Connection(7,8),Connection(1,4),Connection(3,6),Connection(5,8),Connection(7,2)]),
    boundary_leftmost::LaneBoundary=LaneBoundary(:solid, :white),
    boundary_rightmost::LaneBoundary=LaneBoundary(:solid, :white),
    boundary_middle::LaneBoundary=LaneBoundary(:broken, :white)
    )


    segments = RoadSegment[]
    seg1 = gen_straight_segment(1, nlanes, roadlength,
                                                origin=origin, lane_widths=lane_widths,
                                                boundary_leftmost=boundary_leftmost,
                                                boundary_rightmost=boundary_rightmost,
                                                boundary_middle=boundary_middle)
    seg2 = gen_straight_segment(2, nlanes, roadlength,
                      origin=VecSE2(origin.x+roadlength+0.5*lane_width,origin.y-0.5*lane_width,-pi/2),
                                                lane_widths=lane_widths,
                                                boundary_leftmost=boundary_leftmost,
                                                boundary_rightmost=boundary_rightmost,
                                                boundary_middle=boundary_middle)
    seg3 = gen_straight_segment(3, nlanes, roadlength,
        origin=VecSE2(origin.x+roadlength+(2*nlanes-0.5)*lane_width,origin.y-roadlength-0.5*lane_width,pi/2),
                                                lane_widths=lane_widths,
                                                boundary_leftmost=boundary_leftmost,
                                                boundary_rightmost=boundary_rightmost,
                                                boundary_middle=boundary_middle)
    seg4 = gen_straight_segment(4, nlanes, roadlength,
        origin=VecSE2(origin.x+roadlength+2*nlanes*lane_width,origin.y,0.0),
                                                lane_widths=lane_widths,
                                                boundary_leftmost=boundary_leftmost,
                                                boundary_rightmost=boundary_rightmost,
                                                boundary_middle=boundary_middle)

    seg5 = gen_straight_segment(5, nlanes, roadlength,
        origin=VecSE2(origin.x+roadlength+2*nlanes*lane_width+roadlength,origin.y+(2*nlanes-1.0)lane_width,pi),
                                                lane_widths=lane_widths,
                                                boundary_leftmost=boundary_leftmost,
                                                boundary_rightmost=boundary_rightmost,
                                                boundary_middle=boundary_middle)

    seg6 = gen_straight_segment(6, nlanes, roadlength,
        origin=VecSE2(origin.x+roadlength+(2*nlanes-0.5)*lane_width,origin.y+(2*nlanes-0.5)*lane_width,pi/2),
                                                lane_widths=lane_widths,
                                                boundary_leftmost=boundary_leftmost,
                                                boundary_rightmost=boundary_rightmost,
                                                boundary_middle=boundary_middle)
    seg7 = gen_straight_segment(7, nlanes, roadlength,
    origin=VecSE2(origin.x+roadlength+0.5*lane_width,origin.y+(2*nlanes-0.5)*lane_width+roadlength,-pi/2),
                                                lane_widths=lane_widths,
                                                boundary_leftmost=boundary_leftmost,
                                                boundary_rightmost=boundary_rightmost,
                                                boundary_middle=boundary_middle)
    seg8 = gen_straight_segment(8, nlanes, roadlength,
        origin=VecSE2(origin.x+roadlength,origin.y+(2*nlanes-1.0)*lane_width,pi),
                                                lane_widths=lane_widths,
                                                boundary_leftmost=boundary_leftmost,
                                                boundary_rightmost=boundary_rightmost,
                                                boundary_middle=boundary_middle)


    retval = Roadway()
    push!(retval.segments, seg1)
    push!(retval.segments, seg2)
    push!(retval.segments, seg3)
    push!(retval.segments, seg4)
    push!(retval.segments, seg5)
    push!(retval.segments, seg6)
    push!(retval.segments, seg7)
    push!(retval.segments, seg8)


    add_junction!(junction,retval)
    return retval,junction
end


function gen_loop_roadway(nlanes::Int;
    length::Float64=100.0,
    width::Float64=10.0,
    radius::Float64=25.0,
    ncurvepts_per_turn::Int=25, # includes start and end
    lane_width::Float64=DEFAULT_LANE_WIDTH, # [m]
    lane_widths::Vector{Float64} = fill(lane_width, nlanes),
    boundary_leftmost::LaneBoundary=LaneBoundary(:solid, :white),
    boundary_rightmost::LaneBoundary=LaneBoundary(:solid, :white),
    boundary_middle::LaneBoundary=LaneBoundary(:broken, :white),
    )

    ncurvepts_per_turn ≥ 2 || error("must have at least 2 pts per turn")

    A = VecSE2(0.0,0.0,0.0)
    B = VecSE2(length+radius, radius,pi/2)
    C = VecSE2(length, width + 2*radius,-pi)
    D = VecSE2(-radius, width + radius, -pi/2)

    seg1 = gen_straight_segment(1, nlanes, length,
                                                origin=A, lane_widths=lane_widths,
                                                boundary_leftmost=boundary_leftmost,
                                                boundary_rightmost=boundary_rightmost,
                                                boundary_middle=boundary_middle)
    seg2 = gen_straight_segment(2, nlanes, width,
                                                origin=B, lane_widths=lane_widths,
                                                boundary_leftmost=boundary_leftmost,
                                                boundary_rightmost=boundary_rightmost,
                                                boundary_middle=boundary_middle)
    seg3 = gen_straight_segment(3, nlanes, length,
                                                origin=C, lane_widths=lane_widths,
                                                boundary_leftmost=boundary_leftmost,
                                                boundary_rightmost=boundary_rightmost,
                                                boundary_middle=boundary_middle)
    seg4 = gen_straight_segment(4, nlanes, width,
                                                origin=D, lane_widths=lane_widths,
                                                boundary_leftmost=boundary_leftmost,
                                                boundary_rightmost=boundary_rightmost,
                                                boundary_middle=boundary_middle)
    retval = Roadway()
    push!(retval.segments, seg1)
    push!(retval.segments, seg2)
    push!(retval.segments, seg3)
    push!(retval.segments, seg4)

    for i=1:3
         connect_two_seg!(retval.segments[i],retval.segments[i+1],retval)
    end
    connect_two_seg!(retval.segments[4],retval.segments[1],retval)
    retval
end

function add_line!(origin::VecSE2,nlanes::Int,laneLength::Float64,roadway::Roadway;
    lane_width::Float64=DEFAULT_LANE_WIDTH, # [m]
    lane_widths::Vector{Float64} = fill(lane_width, nlanes),
    boundary_leftmost::LaneBoundary=LaneBoundary(:solid, :white),
    boundary_rightmost::LaneBoundary=LaneBoundary(:solid, :white),
    boundary_middle::LaneBoundary=LaneBoundary(:broken, :white)
    )
    id = length(roadway.segments) + 1
    seg = gen_straight_segment(id, nlanes, laneLength,
                                                origin=origin, lane_widths=lane_widths,
                                                boundary_leftmost=boundary_leftmost,
                                                boundary_rightmost=boundary_rightmost,
                                                boundary_middle=boundary_middle)
    push!(roadway.segments,seg)
end
