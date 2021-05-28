__precompile__()

module AutoUrban

using AutomotiveSimulator
using AutomotiveVisualization
using EzXML
using Printf
import Records: QueueRecord, Frame
using LinearAlgebra
using Distributions
import Cairo

export
	LatLonAccelDirection,
	AccelSteeringDirection,
    NextState

include("actions.jl")

export
    initialize_XML,
    get_lane_length,
    handle_junctions,
    convert_roadway!,
    Connection,
    Junction,
    connect_two_lane_general!,
    connect_two_seg!,
    connect_two_seg_general!,
    add_connection!,
    add_junction!,
    gen_connected_lanes,
    gen_intersection,
    gen_loop_roadway,
    add_line!,
    in_lanes,
    get_max_curvature

include("roadway/roadway_generation.jl")
include("roadway/roadway_perception.jl")
include("roadway/roadway_intersection.jl")
include("roadway/convert2xodr.jl")

export
    AccSteerDriver,
    IDMDriver,
    UrbanDriver,
    MultiPtsDriver,
    MultiPtsTurningDriver,
    MOBILDriver,
    excute_action!

include("drivermodels/acc_steer_drivers.jl")
include("drivermodels/IDMDriver.jl")
include("drivermodels/MOBILDriver.jl")
include("drivermodels/urban_drivers.jl")
include("drivermodels/multiPts_driver.jl")
include("drivermodels/multiPts_functions_GD.jl")
include("drivermodels/waypoints_setting_functions.jl")
include("drivermodels/multiPtsTurning_driver.jl")
include("drivermodels/render_drivers.jl")

end
