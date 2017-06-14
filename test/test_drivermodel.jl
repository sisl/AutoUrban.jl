let
    roadlength = 50.0
    lane_width = 5.0
    roadway,junction = gen_intersection(roadlength = roadlength,lane_width = lane_width)
    scene = Frame(Entity{VehicleState, BicycleModel, Int},100)
    id=1
    push!(scene,Entity(VehicleState(VecSE2(0.0,0.0,0.0), roadway, 10.0), 
        BicycleModel(VehicleDef(AgentClass.CAR, 4.826, 1.81)),id))
    id += 1
    push!(scene,Entity(VehicleState(VecSE2(10.0,0.0,0.0), roadway, 10.0), 
        BicycleModel(VehicleDef(AgentClass.CAR, 4.826, 1.81)),id))
    id += 1
    push!(scene,Entity(VehicleState(VecSE2(20.0,0.0,0.0), roadway, 9.0), 
        BicycleModel(VehicleDef(AgentClass.CAR, 4.826, 1.81)),id))
    connect_two_seg!(roadway.segments[2],roadway.segments[3],roadway)
    connect_two_seg!(roadway.segments[4],roadway.segments[5],roadway)
    connect_two_seg!(roadway.segments[6],roadway.segments[7],roadway)
    connect_two_seg!(roadway.segments[8],roadway.segments[1],roadway)
    framerate = 24
    timeStep = 0.05
    models = Dict{Int, DriverModel}()

    models[1]=UrbanDriver(timeStep,direction=1,a_cp_max=0.7*9.8)
    models[2]=UrbanDriver(timeStep,direction=2,a_cp_max=0.6*9.8)
    models[3]=MultiPtsTurningDriver(timeStep,v_max=20.0,v_min=0.0,turning_direction=3)
    excute_action!(models[3],0.0,0,scene, roadway, 1)
    actions = get_actions!(Array(Any, length(scene)), scene, roadway, models)
    for i=1:100
        get_actions!(actions, scene, roadway, models)
        tick!(scene, roadway, actions, timeStep)
    end
    actions = get_actions!(Array(Any, length(scene)), scene, roadway, models)
    @test abs(actions[1].a_lat-0.0)<0.001
    @test abs(actions[1].a_lon-(-0.05641834275191615))<0.001
    @test actions[1].direction==1
    @test abs(actions[2].a_lat-0.0)<0.001
    @test abs(actions[2].a_lon-(-0.0023799116323655056))<0.001
    @test actions[2].direction==2
    @test abs(actions[3].a-0.07816860295178636)<0.001
    @test abs(actions[3].δ-0.00531000497036248)<0.001
    @test abs(actions[3].direction-3 )<0.001
end
