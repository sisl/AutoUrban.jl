import AutoViz.render
import AutoViz.render!

function render_multipoint_drivers!(
    rendermodel::RenderModel,
    model::DriverModel,
    color::Colorant=RGB(1, 0, 0)
    )
    
    add_instruction!(rendermodel, render_point_trail, (model.Pts,color,0.25))
    rendermodel
end

function render_env_drivers!(
    rendermodel::RenderModel,
    model::DriverModel,
    vehid::Int,
    car_colors::Dict{Int,Colorant},
    color::Colorant=RGB(1, 0, 0)
    )

    car_colors[vehid]=colorant"green"
end
#=
function render_trafficlights!(rendermodel::RenderModel,trafficlights::Vector{TrafficLight})
    for trafficlight in trafficlights
        P = convert(VecE2,trafficlight.pos)
        radius = 1.0
        color_fill = colorant"green"
        if trafficlight.color == 1
            color_fill = colorant"red"
        elseif trafficlight.color == 2
            color_fill = colorant"yellow"
        end
        add_instruction!(rendermodel,render_circle,(P,radius,color_fill))
    end
    rendermodel
end
=#
function render(ctx::CairoContext, scene::Union{Scene,Frame{Entity{VehicleState, BicycleModel, Int}}}, roadway::Roadway,models::Dict{Int, DriverModel};
    text::Vector{String}=["Nothing"],
    #trafficlights::Vector{TrafficLight}=TrafficLight[],
    rendermodel::RenderModel=RenderModel(),
    cam::Camera=SceneFollowCamera(),
    car_colors::Dict{Int,Colorant}=Dict{Int,Colorant}()
    )

    canvas_width = floor(Int, Cairo.width(ctx))
    canvas_height = floor(Int, Cairo.height(ctx))

    clear_setup!(rendermodel)

    render!(rendermodel, roadway)
    render!(rendermodel, scene, car_colors=car_colors)
    for (i,veh) in enumerate(scene)
        model = models[veh.id]
        name = AutomotiveDrivingModels.get_name(model)
        if  name == "MultiPtsTurningDriver"
            render_multipoint_drivers!(rendermodel,model)
        end
        if name == "EnvDriver"
            render_env_drivers!(rendermodel,model,veh.id,car_colors)
        end
    end
    
    textoverlay=TextOverlay(text,colorant"white",20,VecE2(10, 20),1.5,false)
    render!(rendermodel, textoverlay, scene, roadway)
    
    #render_trafficlights!(rendermodel,trafficlights)
    
    camera_set!(rendermodel, cam, scene, roadway, canvas_width, canvas_height)

    render(rendermodel, ctx, canvas_width, canvas_height)
    ctx
end

function render(scene::Union{Scene,Frame{Entity{VehicleState, BicycleModel, Int}}}, roadway::Roadway,models::Dict{Int, DriverModel};
    text::Vector{String}=["Nothing"],
    #trafficlights::Vector{TrafficLight}=TrafficLight[],
    canvas_width::Int=DEFAULT_CANVAS_WIDTH,
    canvas_height::Int=DEFAULT_CANVAS_HEIGHT,
    rendermodel::RenderModel=RenderModel(),
    cam::Camera=SceneFollowCamera(),
    car_colors::Dict{Int,Colorant}=Dict{Int,Colorant}(), # id
    )

    s, ctx = get_surface_and_context(canvas_width, canvas_height)
    render(ctx, scene, roadway,models, rendermodel=rendermodel, cam=cam, car_colors=car_colors,text=text#=,trafficlights=trafficlights=#)
    s
end