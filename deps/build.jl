packages = keys(Pkg.installed())
if !in("Vec", packages)
    Pkg.clone("https://github.com/tawheeler/Vec.jl.git")
end
if !in("Records", packages)
    Pkg.clone("https://github.com/tawheeler/Records.jl.git")
end
if !in("AutomotiveDrivingModels", packages)
    Pkg.clone("https://github.com/sisl/AutomotiveDrivingModels.jl.git")
end
if !in("AutoViz", packages)
    Pkg.clone("https://github.com/sisl/AutoViz.jl.git")
end

