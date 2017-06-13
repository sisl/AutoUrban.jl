function render_waypoints!(
    rendermodel::RenderModel,
    pts::Matrix{Float64,1},
    color::Colorant=RGB(1, 0, 0)
    )
    
    add_instruction!(rendermodel, render_point_trail, (pts,color,0.25))
    rendermodel
end

function render(ctx::CairoContext, scene::Scene, roadway::Roadway,pts::Matrix{Float64,1};
    rendermodel::RenderModel=RenderModel(),
    cam::Camera=SceneFollowCamera(),
    car_colors::Dict{Int,Colorant}=Dict{Int,Colorant}(),
    )

    canvas_width = floor(Int, Cairo.width(ctx))
    canvas_height = floor(Int, Cairo.height(ctx))

    clear_setup!(rendermodel)

    render!(rendermodel, roadway)
    render!(rendermodel, scene, car_colors=car_colors)
    render_waypoints!(rendermodel,pts)

    camera_set!(rendermodel, cam, scene, roadway, canvas_width, canvas_height)

    render(rendermodel, ctx, canvas_width, canvas_height)
    ctx
end

function render(scene::Scene, roadway::Roadway,pts::Matrix{Float64,1};
    canvas_width::Int=DEFAULT_CANVAS_WIDTH,
    canvas_height::Int=DEFAULT_CANVAS_HEIGHT,
    rendermodel::RenderModel=RenderModel(),
    cam::Camera=SceneFollowCamera(),
    car_colors::Dict{Int,Colorant}=Dict{Int,Colorant}(), # id
    )

    s, ctx = get_surface_and_context(canvas_width, canvas_height)
    render(ctx, scene, roadway,pts, rendermodel=rendermodel, cam=cam, car_colors=car_colors)
    s
end