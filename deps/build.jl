using Pkg
packages = keys(Pkg.installed())
if !in("Vec", packages)
    Pkg.add(PackageSpec(url="https://github.com/sisl/Vec.jl.git"))
end
if !in("Records", packages)
    Pkg.add(PackageSpec(url="https://github.com/sisl/Records.jl.git"))
end
if !in("AutomotiveSimulator", packages)
    Pkg.add(PackageSpec(url="https://github.com/sisl/AutomotiveSimulator.jl.git"))
end
if !in("AutomotiveVisualization", packages)
    Pkg.add(PackageSpec(url="https://github.com/sisl/AutomotiveVisualization.jl.git"))
end

