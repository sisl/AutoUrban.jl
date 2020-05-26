let
    roadway, junction = gen_intersection(roadlength = 30.0)
    connect_two_seg!(roadway.segments[2], roadway.segments[3], roadway)
    connect_two_seg!(roadway.segments[4], roadway.segments[5], roadway)
    connect_two_seg!(roadway.segments[6], roadway.segments[7], roadway)
    connect_two_seg!(roadway.segments[8], roadway.segments[1], roadway)
    doc, r = initialize_XML()
    convert_roadway!(r, roadway)
    junctions = [junction]
    handle_junctions(r, junctions, roadway)
    r1 = root(doc)
    r1_id = []
    for child1 in eachelement(r1)
        if haskey(child1, "id")
            push!(r1_id, child1["id"])
        end
        for child2 in eachelement(child1)
            if haskey(child2, "id")
                push!(r1_id, child2["id"])
            end
        end
    end
    doc2 = readxml("test_out.xodr")
    r2 = root(doc2)
    r2_id = []
    for child1 in eachelement(r2)
        if haskey(child1, "id")
            push!(r2_id, child1["id"])
        end
        for child2 in eachelement(child1)
            if haskey(child2, "id")
                push!(r2_id, child2["id"])
            end
        end
    end
    @test r1_id == r2_id
end
