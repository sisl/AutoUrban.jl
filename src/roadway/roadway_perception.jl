function get_max_curvature(roadind::RoadIndex, roadway::Roadway, Δs::Float64; max_k::Float64 = 0.0, distance::Float64 = -Inf, base_distance::Float64 = 0.0)
    lane = roadway[roadind.tag]
    current_curvept = lane[roadind.ind, roadway]

    startind = CURVEINDEX_START
    lastind = curveindex_end(lane.curve)

    if abs(current_curvept.k) > abs(max_k) && current_curvept.s >= lane.curve[startind].s && current_curvept.s <= lane.curve[lastind].s
        max_k = current_curvept.k
        distance = 0.0 + base_distance
    end

    #println("current s :",current_curvept.s)
    #println("current k :",current_curvept.k)

    if current_curvept.s + Δs < 0.0
        for curvept in lane.curve
            if curvept.s < current_curvept.s && abs(curvept.k) > abs(max_k)
                max_k = curvept.k
                distance = (current_curvept.s - curvept.s) + base_distance
            end
        end

        if has_prev(lane)
            pt_lo = prev_lane_point(lane, roadway)
            pt_hi = lane.curve[1]
            s_gap = norm(VecE2(pt_hi.pos - pt_lo.pos))

            if current_curvept.s + Δs < -s_gap
                lane_prev = prev_lane(lane, roadway)
                curveind = curveindex_end(lane_prev.curve)
                roadind = RoadIndex(curveind, lane_prev.tag)
                return get_max_curvature(roadind, roadway, Δs + current_curvept.s + s_gap, max_k = max_k, distance = distance, base_distance = base_distance+current_curvept.s + s_gap)
            else # in the gap between lanes
                #t = (s_gap + curvept.s + Δs) / s_gap
                #curveind = CurveIndex(0, t)
                #RoadIndex(curveind, lane.tag)
                return max_k, distance
            end

        else # no prev lane, return the beginning of this one
            return max_k, distance
        end
    elseif current_curvept.s + Δs > lane.curve[end].s
        for curvept in lane.curve
            if curvept.s > current_curvept.s && abs(curvept.k) > abs(max_k)
                max_k = curvept.k
                distance = (curvept.s - current_curvept.s) + base_distance
            end
        end

        if has_next(lane)
            pt_lo = lane.curve[end]
            pt_hi = next_lane_point(lane, roadway)
            s_gap = abs(pt_hi.pos - pt_lo.pos)

            if current_curvept.s + Δs ≥ pt_lo.s + s_gap # extends beyond the gap
                curveind = lane.exits[1].target.ind
                roadind = RoadIndex(curveind, lane.exits[1].target.tag)
                return get_max_curvature(roadind, roadway, Δs - (lane.curve[end].s + s_gap - current_curvept.s), max_k = max_k, distance = distance, base_distance = base_distance+lane.curve[end].s + s_gap - current_curvept.s)
            else # in the gap between lanes
                #t = (Δs - (lane.curve[end].s - curvept.s)) / s_gap
                #curveind = CurveIndex(0, t)
                #RoadIndex(curveind, lane.exits[1].target.tag)
                return max_k, distance
            end
        else # no next lane, return the end of this lane
            return max_k, distance
        end
    else
        #if roadind.ind.i == 0
        #    return max_k, distance
        #elseif roadind.ind.i == length(lane.curve)
        #    return max_k, distance
        #else
            if Δs < 0.0
                for curvept in lane.curve
                    if  current_curvept.s + Δs < curvept.s && curvept.s < current_curvept.s && abs(curvept.k) > abs(max_k)
                        max_k = curvept.k
                        distance = abs(current_curvept.s - curvept.s) + base_distance
                    end
                end
            elseif Δs > 0.0
                for curvept in lane.curve
                    if  current_curvept.s + Δs > curvept.s && curvept.s > current_curvept.s && abs(curvept.k) > abs(max_k)
                        max_k = curvept.k
                        distance = abs(current_curvept.s - curvept.s) + base_distance
                    end
                end
            end
            return max_k, distance
        #end
    end
end

####

function get_max_curvature(roadind::RoadIndex, roadway::Roadway, Δs::Float64; max_k::Float64 = 0.0, distance::Float64 = -Inf, base_distance::Float64 = 0.0, direction::Int = 1)
    lane = roadway[roadind.tag]
    current_curvept = lane[roadind.ind, roadway]

    startind = CURVEINDEX_START
    lastind = curveindex_end(lane.curve)

    if abs(current_curvept.k) > abs(max_k) && current_curvept.s >= lane.curve[startind].s && current_curvept.s <= lane.curve[lastind].s
        max_k = current_curvept.k
        distance = 0.0 + base_distance
    end

    #println("current s :",current_curvept.s)
    #println("current k :",current_curvept.k)

    if current_curvept.s + Δs < 0.0
        for curvept in lane.curve
            if curvept.s < current_curvept.s && abs(curvept.k) > abs(max_k)
                max_k = curvept.k
                distance = (current_curvept.s - curvept.s) + base_distance
            end
        end

        if has_prev(lane)
            pt_lo = prev_lane_point(lane, roadway,direction)
            pt_hi = lane.curve[1]
            s_gap = norm(VecE2(pt_hi.pos - pt_lo.pos))

            if current_curvept.s + Δs < -s_gap
                lane_prev = prev_lane(lane, roadway,direction)
                curveind = curveindex_end(lane_prev.curve)
                roadind = RoadIndex(curveind, lane_prev.tag)
                return get_max_curvature(roadind, roadway, Δs + current_curvept.s + s_gap, max_k = max_k, distance = distance, base_distance = base_distance+current_curvept.s + s_gap,direction=direction)
            else # in the gap between lanes
                #t = (s_gap + curvept.s + Δs) / s_gap
                #curveind = CurveIndex(0, t)
                #RoadIndex(curveind, lane.tag)
                return max_k, distance
            end

        else # no prev lane, return the beginning of this one
            return max_k, distance
        end
    elseif current_curvept.s + Δs > lane.curve[end].s
        for curvept in lane.curve
            if curvept.s > current_curvept.s && abs(curvept.k) > abs(max_k)
                max_k = curvept.k
                distance = (curvept.s - current_curvept.s) + base_distance
            end
        end

        if has_next(lane)
            pt_lo = lane.curve[end]
            pt_hi = next_lane_point(lane, roadway,direction)
            s_gap = norm(VecE2(pt_hi.pos - pt_lo.pos))

            if current_curvept.s + Δs ≥ pt_lo.s + s_gap # extends beyond the gap
                curveind = lane.exits[min(length(lane.exits),direction)].target.ind
                roadind = RoadIndex(curveind, lane.exits[min(length(lane.exits),direction)].target.tag)
                return get_max_curvature(roadind, roadway, Δs - (lane.curve[end].s + s_gap - current_curvept.s), max_k = max_k, distance = distance, base_distance = base_distance+lane.curve[end].s + s_gap - current_curvept.s, direction = direction)
            else # in the gap between lanes
                #t = (Δs - (lane.curve[end].s - curvept.s)) / s_gap
                #curveind = CurveIndex(0, t)
                #RoadIndex(curveind, lane.exits[1].target.tag)
                return max_k, distance
            end
        else # no next lane, return the end of this lane
            return max_k, distance
        end
    else
        #if roadind.ind.i == 0
        #    return max_k, distance
        #elseif roadind.ind.i == length(lane.curve)
        #    return max_k, distance
        #else
            if Δs < 0.0
                for curvept in lane.curve
                    if  current_curvept.s + Δs < curvept.s && curvept.s < current_curvept.s && abs(curvept.k) > abs(max_k)
                        max_k = curvept.k
                        distance = abs(current_curvept.s - curvept.s) + base_distance
                    end
                end
            elseif Δs > 0.0
                for curvept in lane.curve
                    if  current_curvept.s + Δs > curvept.s && curvept.s > current_curvept.s && abs(curvept.k) > abs(max_k)
                        max_k = curvept.k
                        distance = abs(current_curvept.s - curvept.s) + base_distance
                    end
                end
            end
            return max_k, distance
        #end
    end
end
