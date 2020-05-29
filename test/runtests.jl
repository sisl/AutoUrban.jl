using Test

using AutomotiveDrivingModels
using EzXML
using AutoUrban

include("test_convert2xodr.jl")
include("test_convert_connected_roads.jl")
# broken
# include("test_drivermodel.jl")