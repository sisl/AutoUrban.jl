function  create_test_roadway()
    # Initialize roadway for test
    two_road_connect = Roadway()
    
    # Create two individual roads
    origin = VecSE2(0.0,0.0,0.0)
    lane_length = 50.0
    num_lanes = 3
    add_line!(origin, num_lanes, lane_length, two_road_connect)
    
    origin = VecSE2(70.0,2.0,0.0)
    lane_length = 50.0
    num_lanes = 3
    add_line!(origin, num_lanes, lane_length, two_road_connect)
    
    # Simply connect the two roads
    connect_two_seg!(
        two_road_connect.segments[1],
        two_road_connect.segments[2],
        two_road_connect
    ) 

    return two_road_connect
end

@testset "Convert XODR" begin
    two_road_connect = create_test_roadway()

    # Convert the roadway to xodr file
    xodr, root = initialize_XML()
    convert_roadway!(root, two_road_connect)

    ## Actually test conversion
    roads = findall("road", root)
    @test length(roads) == 3

    ### Verify first road
    road = roads[1]
    @test road["name"] == ""
    @test road["id"] == "1"
    @test parse(Float64, road["length"]) == 50.0
    @test road["junction"] == "-1"

    #### Verify the road has the correct children
    road_type = findfirst("type", road)
    @test parse(Float64, road_type["s"]) == 0.0
    @test road_type["type"] == "rural"

    road_link = findfirst("link", road)
    predessor = findfirst("predecessor", road_link)
    @test isnothing(predessor)
    successor = findfirst("successor", road_link)
    @test successor["elementType"] == "road"
    @test successor["elementId"] == "3"
    @test successor["contactPoint"] == "start"

    road_plan_view = findfirst("planView", road)
    @test road_plan_view.type == EzXML.ELEMENT_NODE
    geometry = findfirst("geometry", road_plan_view)
    @test parse(Float64, geometry["s"]) == 0.0
    @test parse(Float64, geometry["x"]) == 0.0
    @test parse(Float64, geometry["y"]) == 0.0
    @test parse(Float64, geometry["hdg"]) == 0.0
    @test parse(Float64, geometry["length"]) == 50.0
    @test findfirst("line", geometry).name == "line"

    ##### Verify lanes
    road_lane = findfirst("lanes", road)
    @test road_lane.name == "lanes"

    lane_offset = findfirst("laneOffset", road_lane)
    @test parse(Float64, lane_offset["s"]) == 0.0
    @test parse(Float64, lane_offset["a"]) == 7.5
    @test parse(Float64, lane_offset["b"]) == 0.0
    @test parse(Float64, lane_offset["c"]) == 0.0
    @test parse(Float64, lane_offset["d"]) == 0.0

    lane_section = findfirst("laneSection", road_lane)
    @test parse(Float64, lane_section["s"]) == 0.0

    left = findfirst("left", lane_section)
    @test left.name == "left"

    center = findfirst("center", lane_section)
    @test center.name == "center"
    lane = findfirst("lane", center)
    @test lane["id"] == "0"
    @test lane["type"] == "driving"
    road_mark = findfirst("roadMark", lane)
    @test parse(Float64, road_mark["sOffset"]) == 0.0
    @test road_mark["type"] == "solid"
    @test road_mark["color"] == "standard"

    right = findfirst("right", lane_section)
    @test right.name == "right"
    lanes = findall("lane", right)

    lane = lanes[1]
    @test lane["id"] == "-1"
    @test lane["type"] == "driving"
    road_mark = findfirst("roadMark", lane)
    @test parse(Float64, road_mark["sOffset"]) == 0.0
    @test road_mark["type"] == "broken"
    @test road_mark["color"] == "standard"
    width = findfirst("width", lane)
    @test parse(Float64, width["sOffset"]) == 0.0
    @test parse(Float64, width["a"]) == 3.0
    @test parse(Float64, width["b"]) == 0.0
    @test parse(Float64, width["c"]) == 0.0
    @test parse(Float64, width["d"]) == 0.0
    link = findfirst("link", lane)
    predessor = findfirst("predecessor", link)
    @test isnothing(predessor)
    successor = findfirst("successor", link)
    @test successor["id"] == "-1"

    lane = lanes[2]
    @test lane["id"] == "-2"
    @test lane["type"] == "driving"
    road_mark = findfirst("roadMark", lane)
    @test parse(Float64, road_mark["sOffset"]) == 0.0
    @test road_mark["type"] == "broken"
    @test road_mark["color"] == "standard"
    width = findfirst("width", lane)
    @test parse(Float64, width["sOffset"]) == 0.0
    @test parse(Float64, width["a"]) == 3.0
    @test parse(Float64, width["b"]) == 0.0
    @test parse(Float64, width["c"]) == 0.0
    @test parse(Float64, width["d"]) == 0.0
    link = findfirst("link", lane)
    predessor = findfirst("predecessor", link)
    @test isnothing(predessor)
    successor = findfirst("successor", link)
    @test successor["id"] == "-2"

    lane = lanes[3]
    @test lane["id"] == "-3"
    @test lane["type"] == "driving"
    road_mark = findfirst("roadMark", lane)
    @test parse(Float64, road_mark["sOffset"]) == 0.0
    @test road_mark["type"] == "broken"
    @test road_mark["color"] == "standard"
    width = findfirst("width", lane)
    @test parse(Float64, width["sOffset"]) == 0.0
    @test parse(Float64, width["a"]) == 3.0
    @test parse(Float64, width["b"]) == 0.0
    @test parse(Float64, width["c"]) == 0.0
    @test parse(Float64, width["d"]) == 0.0
    link = findfirst("link", lane)
    predessor = findfirst("predecessor", link)
    @test isnothing(predessor)
    successor = findfirst("successor", link)
    @test successor["id"] == "-3"
end