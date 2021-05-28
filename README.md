# AutoUrban
[![Build Status](https://travis-ci.org/sisl/AutoUrban.jl.svg)](https://travis-ci.org/sisl/AutoUrban.jl)
[![Coverage Status](https://coveralls.io/repos/github/sisl/AutoUrban.jl/badge.svg?branch=master)](https://coveralls.io/github/sisl/AutoUrban.jl?branch=master)

A Julia package expanding the [AutomotiveSimulator.jl](https://github.com/sisl/AutomotiveSimulator.jl) package with intersections, enabling auto-converting roadway in AutomotiveSimulator.jl into [OpenDrive format (.xodr)](http://www.opendrive.org/) used in [VIRES VirtualTestDrive](https://vires.com/vtd-vires-virtual-test-drive/). It also contains some driver models.

## Installation

Preferred way, by adding the SISL registry: 

```
] registry add https://github.com/sisl/Registry
] add AutoUrban
```

Other way, by manually adding all the dependencies:

```julia 
using Pkg
Pkg.add(PackageSpec(url="https://github.com/sisl/Vec.jl"))
Pkg.add(PackageSpec(url="https://github.com/sisl/Records.jl"))
Pkg.add(PackageSpec(url="https://github.com/sisl/AutomotiveSimulator.jl"))
Pkg.add(PackageSpec(url="https://github.com/sisl/AutomotiveVisualization.jl"))
Pkg.add(PackageSpec(url="https://github.com/sisl/AutoUrban.jl"))
```

## Documentaion
Detailed documentation can be found [here](https://github.com/sisl/UrbanDrivingSimulation.jl/tree/master/docs).
