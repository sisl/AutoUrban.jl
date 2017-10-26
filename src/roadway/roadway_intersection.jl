const MOVE_STRAIGHT_INDEX = 1
const TURN_RIGHT_INDEX = 2
const TURN_LEFT_INDEX = 3


next_lane(lane::Lane, roadway::Roadway, direction::Int) = roadway[lane.exits[clamp(direction,1,length(lane.exits))].target.tag]

prev_lane(lane::Lane, roadway::Roadway, direction::Int) = roadway[lane.entrances[clamp(direction,1,length(lane.entrances))].target.tag]


next_lane_point(lane::Lane, roadway::Roadway, direction::Int) = roadway[lane.exits[clamp(direction,1,length(lane.exits))].target]

prev_lane_point(lane::Lane, roadway::Roadway, direction::Int) = roadway[lane.entrances[clamp(direction,1,length(lane.entrances))].target]

function move_along_with_direction(roadind::RoadIndex, roadway::Roadway, Δs::Float64; direction::Int = 1)

    lane = roadway[roadind.tag]
    curvept = lane[roadind.ind, roadway]

    if curvept.s + Δs < 0.0
        if has_prev(lane)
            pt_lo = prev_lane_point(lane, roadway, direction)
            pt_hi = lane.curve[1]
            s_gap = abs(pt_hi.pos - pt_lo.pos)

            if curvept.s + Δs < -s_gap
                lane_prev = prev_lane(lane, roadway, direction)
                curveind = curveindex_end(lane_prev.curve)
                roadind = RoadIndex(curveind, lane_prev.tag)
                return move_along(roadind, roadway, Δs + curvept.s + s_gap, direction)
            else # in the gap between lanes
                t = (s_gap + curvept.s + Δs) / s_gap
                curveind = CurveIndex(0, t)
                RoadIndex(curveind, lane.tag)
            end

        else # no prev lane, return the beginning of this one
            curveind = CurveIndex(1, 0.0)
            return RoadIndex(curveind, roadind.tag)
        end
    elseif curvept.s + Δs > lane.curve[end].s
        if has_next(lane)
            pt_lo = lane.curve[end]
            pt_hi = next_lane_point(lane, roadway, direction)
            s_gap = abs(pt_hi.pos - pt_lo.pos)

            if curvept.s + Δs ≥ pt_lo.s + s_gap # extends beyond the gap
                curveind = lane.exits[min(length(lane.exits),direction)].target.ind
                roadind = RoadIndex(curveind, lane.exits[min(length(lane.exits),direction)].target.tag)
                return move_along(roadind, roadway, Δs - (lane.curve[end].s + s_gap - curvept.s),direction)
            else # in the gap between lanes
                t = (Δs - (lane.curve[end].s - curvept.s)) / s_gap
                curveind = CurveIndex(0, t)
                RoadIndex(curveind, lane.exits[clamp(direction,1,length(lane.exits))].target.tag)
            end
        else # no next lane, return the end of this lane
            curveind = curveindex_end(lane.curve)
            return RoadIndex(curveind, roadind.tag)
        end
    else
        if Δs >= 0.0
            if roadind.ind.i == 0
                ind = get_curve_index(CurveIndex(1,0.0), lane.curve, curvept.s+Δs)
            elseif roadind.ind.i == length(lane.curve)
                ind = get_curve_index(curveindex_end(lane.curve), lane.curve, curvept.s+Δs)
            else
                ind = get_curve_index(roadind.ind, lane.curve, Δs)
            end
            RoadIndex(ind, roadind.tag)
        else
            if roadind.ind.i == 0
                ind = get_curve_index(CurveIndex(1,0.0), lane.curve, curvept.s+Δs)
            elseif roadind.ind.i == length(lane.curve)
                ind = get_curve_index(curveindex_end(lane.curve), lane.curve, Δs)
            else
                ind = get_curve_index(roadind.ind, lane.curve, Δs)
            end
            RoadIndex(ind, roadind.tag)
        end
    end
end

function in_lanes(posG::VecSE2, roadway::Roadway)
    projections = Array{RoadProjection}(0)
    for seg in roadway.segments
        for lane in seg.lanes
            roadproj = proj(posG, lane, roadway, move_along_curves=false)
            targetlane = roadway[roadproj.tag]
            footpoint = targetlane[roadproj.curveproj.ind, roadway]
            dist = abs2(posG - footpoint.pos)
            if dist < targetlane.width/2.0
                push!(projections,roadproj)
            end
        end
    end

    return projections
end

