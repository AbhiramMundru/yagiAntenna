'# MWS Version: Version 2019.0 - Sep 20 2018 - ACIS 28.0.2 -

'# length = mm
'# frequency = GHz
'# time = ns
'# frequency range: fmin = 1 fmax = 2
'# created = '[VERSION]2019.0|28.0.2|20180920[/VERSION]


'@ use template: Antenna - Planar_2.cfg

'[VERSION]2019.0|28.0.2|20180920[/VERSION]
'set the units
With Units
    .Geometry "mm"
    .Frequency "GHz"
    .Voltage "V"
    .Resistance "Ohm"
    .Inductance "H"
    .TemperatureUnit  "Kelvin"
    .Time "ns"
    .Current "A"
    .Conductance "Siemens"
    .Capacitance "F"
End With
'----------------------------------------------------------------------------
Plot.DrawBox True
With Background
     .Type "Normal"
     .Epsilon "1.0"
     .Mu "1.0"
     .XminSpace "0.0"
     .XmaxSpace "0.0"
     .YminSpace "0.0"
     .YmaxSpace "0.0"
     .ZminSpace "0.0"
     .ZmaxSpace "0.0"
End With
With Boundary
     .Xmin "expanded open"
     .Xmax "expanded open"
     .Ymin "expanded open"
     .Ymax "expanded open"
     .Zmin "expanded open"
     .Zmax "expanded open"
     .Xsymmetry "none"
     .Ysymmetry "none"
     .Zsymmetry "none"
End With
' optimize mesh settings for planar structures
With Mesh
     .MergeThinPECLayerFixpoints "True"
     .RatioLimit "20"
     .AutomeshRefineAtPecLines "True", "6"
     .FPBAAvoidNonRegUnite "True"
     .ConsiderSpaceForLowerMeshLimit "False"
     .MinimumStepNumber "5"
     .AnisotropicCurvatureRefinement "True"
     .AnisotropicCurvatureRefinementFSM "True"
End With
With MeshSettings
     .SetMeshType "Hex"
     .Set "RatioLimitGeometry", "20"
     .Set "EdgeRefinementOn", "1"
     .Set "EdgeRefinementRatio", "6"
End With
With MeshSettings
     .SetMeshType "HexTLM"
     .Set "RatioLimitGeometry", "20"
End With
With MeshSettings
     .SetMeshType "Tet"
     .Set "VolMeshGradation", "1.5"
     .Set "SrfMeshGradation", "1.5"
End With
' change mesh adaption scheme to energy
' 		(planar structures tend to store high energy
'     	 locally at edges rather than globally in volume)
MeshAdaption3D.SetAdaptionStrategy "Energy"
' switch on FD-TET setting for accurate farfields
FDSolver.ExtrudeOpenBC "True"
PostProcess1D.ActivateOperation "vswr", "true"
PostProcess1D.ActivateOperation "yz-matrices", "true"
With FarfieldPlot
	.ClearCuts ' lateral=phi, polar=theta
	.AddCut "lateral", "0", "1"
	.AddCut "lateral", "90", "1"
	.AddCut "polar", "90", "1"
End With
'----------------------------------------------------------------------------
With MeshSettings
     .SetMeshType "Hex"
     .Set "Version", 1%
End With
With Mesh
     .MeshType "PBA"
End With
'set the solver type
ChangeSolverType("HF Time Domain")
'----------------------------------------------------------------------------

'@ define units

'[VERSION]2019.0|28.0.2|20180920[/VERSION]
With Units 
     .Geometry "mm" 
     .Frequency "MHz" 
     .Time "ns" 
     .TemperatureUnit "Kelvin" 
     .Voltage "V" 
     .Current "A" 
     .Resistance "Ohm" 
     .Conductance "Siemens" 
     .Capacitance "F" 
     .Inductance "H" 
     .SetResultUnit "frequency", "frequency", "" 
End With

'@ new component: component1

'[VERSION]2019.0|28.0.2|20180920[/VERSION]
Component.New "component1"

'@ define cylinder: component1:reflector

'[VERSION]2019.0|28.0.2|20180920[/VERSION]
With Cylinder 
     .Reset 
     .Name "reflector" 
     .Component "component1" 
     .Material "Vacuum" 
     .OuterRadius "d" 
     .InnerRadius "0" 
     .Axis "z" 
     .Zrange "-l_reflector/2", "l_reflector/2" 
     .Xcenter "0" 
     .Ycenter "0" 
     .Segments "0" 
     .Create 
End With

'@ delete shape: component1:reflector

'[VERSION]2019.0|28.0.2|20180920[/VERSION]
Solid.Delete "component1:reflector"

'@ define material: Copper (annealed)

'[VERSION]2019.0|28.0.2|20180920[/VERSION]
With Material
     .Reset
     .Name "Copper (annealed)"
     .Folder ""
     .FrqType "static"
     .Type "Normal"
     .SetMaterialUnit "Hz", "mm"
     .Epsilon "1"
     .Mu "1.0"
     .Kappa "5.8e+007"
     .TanD "0.0"
     .TanDFreq "0.0"
     .TanDGiven "False"
     .TanDModel "ConstTanD"
     .KappaM "0"
     .TanDM "0.0"
     .TanDMFreq "0.0"
     .TanDMGiven "False"
     .TanDMModel "ConstTanD"
     .DispModelEps "None"
     .DispModelMu "None"
     .DispersiveFittingSchemeEps "Nth Order"
     .DispersiveFittingSchemeMu "Nth Order"
     .UseGeneralDispersionEps "False"
     .UseGeneralDispersionMu "False"
     .FrqType "all"
     .Type "Lossy metal"
     .SetMaterialUnit "GHz", "mm"
     .Mu "1.0"
     .Kappa "5.8e+007"
     .Rho "8930.0"
     .ThermalType "Normal"
     .ThermalConductivity "401.0"
     .HeatCapacity "0.39"
     .MetabolicRate "0"
     .BloodFlow "0"
     .VoxelConvection "0"
     .MechanicsType "Isotropic"
     .YoungsModulus "120"
     .PoissonsRatio "0.33"
     .ThermalExpansionRate "17"
     .Colour "1", "1", "0"
     .Wireframe "False"
     .Reflection "False"
     .Allowoutline "True"
     .Transparentoutline "False"
     .Transparency "0"
     .Create
End With

'@ define cylinder: component1:reflector

'[VERSION]2019.0|28.0.2|20180920[/VERSION]
With Cylinder 
     .Reset 
     .Name "reflector" 
     .Component "component1" 
     .Material "Copper (annealed)" 
     .OuterRadius "d" 
     .InnerRadius "0" 
     .Axis "z" 
     .Zrange "-l_reflector/2", "l_reflector/2" 
     .Xcenter "0" 
     .Ycenter "0" 
     .Segments "0" 
     .Create 
End With

'@ delete shape: component1:reflector

'[VERSION]2019.0|28.0.2|20180920[/VERSION]
Solid.Delete "component1:reflector"

'@ define cylinder: component1:reflector

'[VERSION]2019.0|28.0.2|20180920[/VERSION]
With Cylinder 
     .Reset 
     .Name "reflector" 
     .Component "component1" 
     .Material "Copper (annealed)" 
     .OuterRadius "d/2" 
     .InnerRadius "0" 
     .Axis "z" 
     .Zrange "-l_reflector/2", "l_reflector/2" 
     .Xcenter "0" 
     .Ycenter "0" 
     .Segments "0" 
     .Create 
End With

'@ delete shape: component1:reflector

'[VERSION]2019.0|28.0.2|20180920[/VERSION]
Solid.Delete "component1:reflector"

'@ define cylinder: component1:reflector

'[VERSION]2019.0|28.0.2|20180920[/VERSION]
With Cylinder 
     .Reset 
     .Name "reflector" 
     .Component "component1" 
     .Material "Copper (annealed)" 
     .OuterRadius "d/2" 
     .InnerRadius "0" 
     .Axis "z" 
     .Zrange "-l_reflector/2", "l_reflector/2" 
     .Xcenter "0" 
     .Ycenter "0" 
     .Segments "0" 
     .Create 
End With

'@ define units

'[VERSION]2019.0|28.0.2|20180920[/VERSION]
With Units 
     .Geometry "mm" 
     .Frequency "Hz" 
     .Time "ns" 
     .TemperatureUnit "Kelvin" 
     .Voltage "V" 
     .Current "A" 
     .Resistance "Ohm" 
     .Conductance "Siemens" 
     .Capacitance "F" 
     .Inductance "H" 
     .SetResultUnit "frequency", "frequency", "" 
End With

'@ define cylinder: component1:dipole

'[VERSION]2019.0|28.0.2|20180920[/VERSION]
With Cylinder 
     .Reset 
     .Name "dipole" 
     .Component "component1" 
     .Material "Copper (annealed)" 
     .OuterRadius "d/2" 
     .InnerRadius "0.0" 
     .Axis "z" 
     .Zrange "-(l_dipole/2)", "l_dipole/2" 
     .Xcenter "reflector_dipole" 
     .Ycenter "0" 
     .Segments "0" 
     .Create 
End With

'@ define cylinder: component1:director1

'[VERSION]2019.0|28.0.2|20180920[/VERSION]
With Cylinder 
     .Reset 
     .Name "director1" 
     .Component "component1" 
     .Material "Copper (annealed)" 
     .OuterRadius "d/2" 
     .InnerRadius "0.0" 
     .Axis "z" 
     .Zrange "-(l_director_1/2)", "(l_director_1/2)" 
     .Xcenter "reflector_dipole+dipole_director" 
     .Ycenter "0" 
     .Segments "0" 
     .Create 
End With

'@ define cylinder: component1:director2

'[VERSION]2019.0|28.0.2|20180920[/VERSION]
With Cylinder 
     .Reset 
     .Name "director2" 
     .Component "component1" 
     .Material "Copper (annealed)" 
     .OuterRadius "d/2" 
     .InnerRadius "0.0" 
     .Axis "z" 
     .Zrange "-(l_director_2/2)", "(l_director_2/2)" 
     .Xcenter "reflector_dipole+dipole_director+director_director" 
     .Ycenter "0" 
     .Segments "0" 
     .Create 
End With

'@ define brick: component1:slot

'[VERSION]2019.0|28.0.2|20180920[/VERSION]
With Brick
     .Reset 
     .Name "slot" 
     .Component "component1" 
     .Material "Vacuum" 
     .Xrange "-s/2+reflector_dipole", "s/2+reflector_dipole" 
     .Yrange "-s/2", "s/2" 
     .Zrange "-s/2", "s/2" 
     .Create
End With

'@ boolean subtract shapes: component1:dipole, component1:slot

'[VERSION]2019.0|28.0.2|20180920[/VERSION]
Solid.Subtract "component1:dipole", "component1:slot"

'@ pick circle center point

'[VERSION]2019.0|28.0.2|20180920[/VERSION]
Pick.PickCirclecenterFromId "component1:dipole", "16"

'@ pick circle center point

'[VERSION]2019.0|28.0.2|20180920[/VERSION]
Pick.PickCirclecenterFromId "component1:dipole", "15"

''@ define discrete port: 1
'
''[VERSION]2019.0|28.0.2|20180920[/VERSION]
'With DiscretePort 
'     .Reset 
'     .PortNumber "1" 
'     .Type "SParameter" 
'     .Label "" 
'     .Folder "" 
'     .Impedance "50.0" 
'     .VoltagePortImpedance "0.0" 
'     .Voltage "1.0" 
'     .Current "1.0" 
'     .SetP1 "True", "583.33333333333", "0", "-33.5" 
'     .SetP2 "True", "583.33333333333", "0", "33.5" 
'     .InvertDirection "False" 
'     .LocalCoordinates "False" 
'     .Monitor "True" 
'     .Radius "0.0" 
'     .Wire "" 
'     .Position "end1" 
'     .Create 
'End With
'
''@ define monitor: e-field (f=1)
'
''[VERSION]2019.0|28.0.2|20180920[/VERSION]
'With Monitor 
'     .Reset 
'     .Name "e-field (f=1)" 
'     .Dimension "Volume" 
'     .Domain "Frequency" 
'     .FieldType "Efield" 
'     .MonitorValue "1" 
'     .UseSubvolume "False" 
'     .Coordinates "Structure" 
'     .SetSubvolume "-10", "1135", "-10", "10", "-458.33333333334", "458.33333333334" 
'     .SetSubvolumeOffset "0.0", "0.0", "0.0", "0.0", "0.0", "0.0" 
'     .SetSubvolumeInflateWithOffset "False" 
'     .Create 
'End With
'
''@ define units
'
''[VERSION]2019.0|28.0.2|20180920[/VERSION]
'With Units 
'     .Geometry "mm" 
'     .Frequency "GHz" 
'     .Time "ns" 
'     .TemperatureUnit "Kelvin" 
'     .Voltage "V" 
'     .Current "A" 
'     .Resistance "Ohm" 
'     .Conductance "Siemens" 
'     .Capacitance "F" 
'     .Inductance "H" 
'     .SetResultUnit "frequency", "frequency", "" 
'End With
'
''@ define monitor: e-field (f=1)
'
''[VERSION]2019.0|28.0.2|20180920[/VERSION]
'With Monitor 
'     .Reset 
'     .Name "e-field (f=1)" 
'     .Dimension "Volume" 
'     .Domain "Frequency" 
'     .FieldType "Efield" 
'     .MonitorValue "1" 
'     .UseSubvolume "False" 
'     .Coordinates "Structure" 
'     .SetSubvolume "-10", "1135", "-10", "10", "-458.33333333334", "458.33333333334" 
'     .SetSubvolumeOffset "0.0", "0.0", "0.0", "0.0", "0.0", "0.0" 
'     .SetSubvolumeInflateWithOffset "False" 
'     .Create 
'End With
'
''@ define monitor: h-field (f=1)
'
''[VERSION]2019.0|28.0.2|20180920[/VERSION]
'With Monitor 
'     .Reset 
'     .Name "h-field (f=1)" 
'     .Dimension "Volume" 
'     .Domain "Frequency" 
'     .FieldType "Hfield" 
'     .MonitorValue "1" 
'     .UseSubvolume "False" 
'     .Coordinates "Structure" 
'     .SetSubvolume "-10", "1135", "-10", "10", "-458.33333333334", "458.33333333334" 
'     .SetSubvolumeOffset "0.0", "0.0", "0.0", "0.0", "0.0", "0.0" 
'     .SetSubvolumeInflateWithOffset "False" 
'     .Create 
'End With
'
''@ define farfield monitor: farfield (f=1)
'
''[VERSION]2019.0|28.0.2|20180920[/VERSION]
'With Monitor 
'     .Reset 
'     .Name "farfield (f=1)" 
'     .Domain "Frequency" 
'     .FieldType "Farfield" 
'     .MonitorValue "1" 
'     .ExportFarfieldSource "False" 
'     .UseSubvolume "False" 
'     .Coordinates "Structure" 
'     .SetSubvolume "-10", "1135", "-10", "10", "-458.33333333334", "458.33333333334" 
'     .SetSubvolumeOffset "10", "10", "10", "10", "10", "10" 
'     .SetSubvolumeInflateWithOffset "False" 
'     .SetSubvolumeOffsetType "FractionOfWavelength" 
'     .Create 
'End With
'
''@ define frequency range
'
''[VERSION]2019.0|28.0.2|20180920[/VERSION]
'Solver.FrequencyRange "0.0", "3"
'
''@ define time domain solver parameters
'
''[VERSION]2019.0|28.0.2|20180920[/VERSION]
'Mesh.SetCreator "High Frequency" 
'With Solver 
'     .Method "Hexahedral"
'     .CalculationType "TD-S"
'     .StimulationPort "All"
'     .StimulationMode "All"
'     .SteadyStateLimit "-40"
'     .MeshAdaption "False"
'     .AutoNormImpedance "False"
'     .NormingImpedance "50"
'     .CalculateModesOnly "False"
'     .SParaSymmetry "False"
'     .StoreTDResultsInCache  "False"
'     .FullDeembedding "False"
'     .SuperimposePLWExcitation "False"
'     .UseSensitivityAnalysis "False"
'End With
'
''@ farfield plot options
'
''[VERSION]2019.0|28.0.2|20180920[/VERSION]
'With FarfieldPlot 
'     .Plottype "3D" 
'     .Vary "angle1" 
'     .Theta "90" 
'     .Phi "90" 
'     .Step "5" 
'     .Step2 "5" 
'     .SetLockSteps "True" 
'     .SetPlotRangeOnly "False" 
'     .SetThetaStart "0" 
'     .SetThetaEnd "180" 
'     .SetPhiStart "0" 
'     .SetPhiEnd "360" 
'     .SetTheta360 "False" 
'     .SymmetricRange "False" 
'     .SetTimeDomainFF "False" 
'     .SetFrequency "-1" 
'     .SetTime "0" 
'     .SetColorByValue "True" 
'     .DrawStepLines "False" 
'     .DrawIsoLongitudeLatitudeLines "False" 
'     .ShowStructure "False" 
'     .ShowStructureProfile "False" 
'     .SetStructureTransparent "False" 
'     .SetFarfieldTransparent "False" 
'     .SetSpecials "enablepolarextralines" 
'     .SetPlotMode "Directivity" 
'     .Distance "1" 
'     .UseFarfieldApproximation "True" 
'     .SetScaleLinear "False" 
'     .SetLogRange "40" 
'     .SetLogNorm "0" 
'     .DBUnit "0" 
'     .SetMaxReferenceMode "abs" 
'     .EnableFixPlotMaximum "False" 
'     .SetFixPlotMaximumValue "1" 
'     .SetInverseAxialRatio "False" 
'     .SetAxesType "user" 
'     .SetAntennaType "isotropic" 
'     .Phistart "1.000000e+00", "0.000000e+00", "0.000000e+00" 
'     .Thetastart "0.000000e+00", "0.000000e+00", "1.000000e+00" 
'     .PolarizationVector "0.000000e+00", "1.000000e+00", "0.000000e+00" 
'     .SetCoordinateSystemType "spherical" 
'     .SetAutomaticCoordinateSystem "True" 
'     .SetPolarizationType "Abs" 
'     .SlantAngle 0.000000e+00 
'     .Origin "bbox" 
'     .Userorigin "0.000000e+00", "0.000000e+00", "0.000000e+00" 
'     .SetUserDecouplingPlane "False" 
'     .UseDecouplingPlane "False" 
'     .DecouplingPlaneAxis "X" 
'     .DecouplingPlanePosition "0.000000e+00" 
'     .LossyGround "False" 
'     .GroundEpsilon "1" 
'     .GroundKappa "0" 
'     .EnablePhaseCenterCalculation "False" 
'     .SetPhaseCenterAngularLimit "3.000000e+01" 
'     .SetPhaseCenterComponent "boresight" 
'     .SetPhaseCenterPlane "both" 
'     .ShowPhaseCenter "True" 
'     .ClearCuts 
'     .AddCut "lateral", "0", "1"  
'     .AddCut "lateral", "90", "1"  
'     .AddCut "polar", "90", "1"  
'     .StoreSettings
'End With
'
''@ define monitor: e-field (f=1.8)
'
''[VERSION]2019.0|28.0.2|20180920[/VERSION]
'With Monitor 
'     .Reset 
'     .Name "e-field (f=1.8)" 
'     .Dimension "Volume" 
'     .Domain "Frequency" 
'     .FieldType "Efield" 
'     .MonitorValue "1.8" 
'     .UseSubvolume "False" 
'     .Coordinates "Structure" 
'     .SetSubvolume "-10", "1135", "-10", "10", "-458.33333333334", "458.33333333334" 
'     .SetSubvolumeOffset "0.0", "0.0", "0.0", "0.0", "0.0", "0.0" 
'     .SetSubvolumeInflateWithOffset "False" 
'     .Create 
'End With
'
''@ define monitor: h-field (f=1.8)
'
''[VERSION]2019.0|28.0.2|20180920[/VERSION]
'With Monitor 
'     .Reset 
'     .Name "h-field (f=1.8)" 
'     .Dimension "Volume" 
'     .Domain "Frequency" 
'     .FieldType "Hfield" 
'     .MonitorValue "1.8" 
'     .UseSubvolume "False" 
'     .Coordinates "Structure" 
'     .SetSubvolume "-10", "1135", "-10", "10", "-458.33333333334", "458.33333333334" 
'     .SetSubvolumeOffset "0.0", "0.0", "0.0", "0.0", "0.0", "0.0" 
'     .SetSubvolumeInflateWithOffset "False" 
'     .Create 
'End With
'
''@ define farfield monitor: farfield (f=1.8)
'
''[VERSION]2019.0|28.0.2|20180920[/VERSION]
'With Monitor 
'     .Reset 
'     .Name "farfield (f=1.8)" 
'     .Domain "Frequency" 
'     .FieldType "Farfield" 
'     .MonitorValue "1.8" 
'     .ExportFarfieldSource "False" 
'     .UseSubvolume "False" 
'     .Coordinates "Structure" 
'     .SetSubvolume "-10", "1135", "-10", "10", "-458.33333333334", "458.33333333334" 
'     .SetSubvolumeOffset "10", "10", "10", "10", "10", "10" 
'     .SetSubvolumeInflateWithOffset "False" 
'     .SetSubvolumeOffsetType "FractionOfWavelength" 
'     .Create 
'End With
'
''@ farfield plot options
'
''[VERSION]2019.0|28.0.2|20180920[/VERSION]
'With FarfieldPlot 
'     .Plottype "3D" 
'     .Vary "angle1" 
'     .Theta "90" 
'     .Phi "90" 
'     .Step "5" 
'     .Step2 "5" 
'     .SetLockSteps "True" 
'     .SetPlotRangeOnly "False" 
'     .SetThetaStart "0" 
'     .SetThetaEnd "180" 
'     .SetPhiStart "0" 
'     .SetPhiEnd "360" 
'     .SetTheta360 "False" 
'     .SymmetricRange "False" 
'     .SetTimeDomainFF "False" 
'     .SetFrequency "-1" 
'     .SetTime "0" 
'     .SetColorByValue "True" 
'     .DrawStepLines "False" 
'     .DrawIsoLongitudeLatitudeLines "False" 
'     .ShowStructure "False" 
'     .ShowStructureProfile "False" 
'     .SetStructureTransparent "False" 
'     .SetFarfieldTransparent "False" 
'     .SetSpecials "enablepolarextralines" 
'     .SetPlotMode "Directivity" 
'     .Distance "1" 
'     .UseFarfieldApproximation "True" 
'     .SetScaleLinear "False" 
'     .SetLogRange "40" 
'     .SetLogNorm "0" 
'     .DBUnit "0" 
'     .SetMaxReferenceMode "abs" 
'     .EnableFixPlotMaximum "False" 
'     .SetFixPlotMaximumValue "1" 
'     .SetInverseAxialRatio "False" 
'     .SetAxesType "user" 
'     .SetAntennaType "unknown" 
'     .Phistart "1.000000e+00", "0.000000e+00", "0.000000e+00" 
'     .Thetastart "0.000000e+00", "0.000000e+00", "1.000000e+00" 
'     .PolarizationVector "0.000000e+00", "1.000000e+00", "0.000000e+00" 
'     .SetCoordinateSystemType "spherical" 
'     .SetAutomaticCoordinateSystem "True" 
'     .SetPolarizationType "Linear" 
'     .SlantAngle 0.000000e+00 
'     .Origin "bbox" 
'     .Userorigin "0.000000e+00", "0.000000e+00", "0.000000e+00" 
'     .SetUserDecouplingPlane "False" 
'     .UseDecouplingPlane "False" 
'     .DecouplingPlaneAxis "X" 
'     .DecouplingPlanePosition "0.000000e+00" 
'     .LossyGround "False" 
'     .GroundEpsilon "1" 
'     .GroundKappa "0" 
'     .EnablePhaseCenterCalculation "False" 
'     .SetPhaseCenterAngularLimit "3.000000e+01" 
'     .SetPhaseCenterComponent "boresight" 
'     .SetPhaseCenterPlane "both" 
'     .ShowPhaseCenter "True" 
'     .ClearCuts 
'     .AddCut "lateral", "0", "1"  
'     .AddCut "lateral", "90", "1"  
'     .AddCut "polar", "90", "1"  
'     .StoreSettings
'End With
'
''@ define monitor: e-field (f=1.8)
'
''[VERSION]2019.0|28.0.2|20180920[/VERSION]
'With Monitor 
'     .Reset 
'     .Name "e-field (f=1.8)" 
'     .Dimension "Volume" 
'     .Domain "Frequency" 
'     .FieldType "Efield" 
'     .MonitorValue "1.8" 
'     .UseSubvolume "False" 
'     .Coordinates "Structure" 
'     .SetSubvolume "-10", "1135", "-10", "10", "-458.33333333334", "458.33333333334" 
'     .SetSubvolumeOffset "0.0", "0.0", "0.0", "0.0", "0.0", "0.0" 
'     .SetSubvolumeInflateWithOffset "False" 
'     .Create 
'End With
'
''@ define monitor: h-field (f=1.8)
'
''[VERSION]2019.0|28.0.2|20180920[/VERSION]
'With Monitor 
'     .Reset 
'     .Name "h-field (f=1.8)" 
'     .Dimension "Volume" 
'     .Domain "Frequency" 
'     .FieldType "Hfield" 
'     .MonitorValue "1.8" 
'     .UseSubvolume "False" 
'     .Coordinates "Structure" 
'     .SetSubvolume "-10", "1135", "-10", "10", "-458.33333333334", "458.33333333334" 
'     .SetSubvolumeOffset "0.0", "0.0", "0.0", "0.0", "0.0", "0.0" 
'     .SetSubvolumeInflateWithOffset "False" 
'     .Create 
'End With
'
''@ define farfield monitor: farfield (f=1.8)
'
''[VERSION]2019.0|28.0.2|20180920[/VERSION]
'With Monitor 
'     .Reset 
'     .Name "farfield (f=1.8)" 
'     .Domain "Frequency" 
'     .FieldType "Farfield" 
'     .MonitorValue "1.8" 
'     .ExportFarfieldSource "False" 
'     .UseSubvolume "False" 
'     .Coordinates "Structure" 
'     .SetSubvolume "-10", "1135", "-10", "10", "-458.33333333334", "458.33333333334" 
'     .SetSubvolumeOffset "10", "10", "10", "10", "10", "10" 
'     .SetSubvolumeInflateWithOffset "False" 
'     .SetSubvolumeOffsetType "FractionOfWavelength" 
'     .Create 
'End With
'
''@ define fieldsource monitor: field-source (f=1.8)
'
''[VERSION]2019.0|28.0.2|20180920[/VERSION]
'With Monitor 
'     .Reset 
'     .Name "field-source (f=1.8)" 
'     .Domain "Frequency" 
'     .FieldType "Fieldsource" 
'     .Samples "1" 
'     .MonitorValueRange "1.8", "1.8" 
'     .InvertOrientation "False" 
'     .UseSubvolume "False" 
'     .Coordinates "Structure" 
'     .SetSubvolume "-10", "1135", "-10", "10", "-458.33333333334", "458.33333333334" 
'     .SetSubvolumeOffset "10", "10", "10", "10", "10", "10" 
'     .SetSubvolumeInflateWithOffset "False" 
'     .SetSubvolumeOffsetType "FractionOfWavelength" 
'     .Create 
'End With
'
''@ define frequency range
'
''[VERSION]2019.0|28.0.2|20180920[/VERSION]
'Solver.FrequencyRange "0", "2"
'
'@ define units

'[VERSION]2019.0|28.0.2|20180920[/VERSION]
With Units 
     .Geometry "mm" 
     .Frequency "GHz" 
     .Time "ns" 
     .TemperatureUnit "Kelvin" 
     .Voltage "V" 
     .Current "A" 
     .Resistance "Ohm" 
     .Conductance "Siemens" 
     .Capacitance "F" 
     .Inductance "H" 
     .SetResultUnit "frequency", "frequency", "" 
End With

'@ define discrete port: 1

'[VERSION]2019.0|28.0.2|20180920[/VERSION]
With DiscretePort 
     .Reset 
     .PortNumber "1" 
     .Type "SParameter" 
     .Label "" 
     .Folder "" 
     .Impedance "50.0" 
     .VoltagePortImpedance "0.0" 
     .Voltage "1.0" 
     .Current "1.0" 
     .SetP1 "True", "583.33333333333", "0", "-15" 
     .SetP2 "True", "583.33333333333", "0", "15" 
     .InvertDirection "False" 
     .LocalCoordinates "False" 
     .Monitor "True" 
     .Radius "0.0" 
     .Wire "" 
     .Position "end1" 
     .Create 
End With

'@ define monitor: e-field (f=1.8)

'[VERSION]2019.0|28.0.2|20180920[/VERSION]
With Monitor 
     .Reset 
     .Name "e-field (f=1.8)" 
     .Dimension "Volume" 
     .Domain "Frequency" 
     .FieldType "Efield" 
     .MonitorValue "1.8" 
     .UseSubvolume "False" 
     .Coordinates "Structure" 
     .SetSubvolume "-7.5", "1132.5", "-7.5", "7.5", "-393.75", "393.75" 
     .SetSubvolumeOffset "0.0", "0.0", "0.0", "0.0", "0.0", "0.0" 
     .SetSubvolumeInflateWithOffset "False" 
     .Create 
End With

'@ define frequency range

'[VERSION]2019.0|28.0.2|20180920[/VERSION]
Solver.FrequencyRange "1", "2"

'@ define time domain solver parameters

'[VERSION]2019.0|28.0.2|20180920[/VERSION]
Mesh.SetCreator "High Frequency" 
With Solver 
     .Method "Hexahedral"
     .CalculationType "TD-S"
     .StimulationPort "All"
     .StimulationMode "All"
     .SteadyStateLimit "-40"
     .MeshAdaption "False"
     .AutoNormImpedance "False"
     .NormingImpedance "50"
     .CalculateModesOnly "False"
     .SParaSymmetry "False"
     .StoreTDResultsInCache  "False"
     .FullDeembedding "False"
     .SuperimposePLWExcitation "False"
     .UseSensitivityAnalysis "False"
End With

'@ delete port: port1

'[VERSION]2019.0|28.0.2|20180920[/VERSION]
Port.Delete "1"

'@ pick circle center point

'[VERSION]2019.0|28.0.2|20180920[/VERSION]
Pick.PickCirclecenterFromId "component1:dipole", "15"

'@ pick circle center point

'[VERSION]2019.0|28.0.2|20180920[/VERSION]
Pick.PickCirclecenterFromId "component1:dipole", "16"

'@ define discrete port: 1

'[VERSION]2019.0|28.0.2|20180920[/VERSION]
With DiscretePort 
     .Reset 
     .PortNumber "1" 
     .Type "SParameter" 
     .Label "" 
     .Folder "" 
     .Impedance "50.0" 
     .VoltagePortImpedance "0.0" 
     .Voltage "1.0" 
     .Current "1.0" 
     .SetP1 "True", "583.33333333333", "0", "15" 
     .SetP2 "True", "583.33333333333", "0", "-15" 
     .InvertDirection "False" 
     .LocalCoordinates "False" 
     .Monitor "True" 
     .Radius "0.0" 
     .Wire "" 
     .Position "end1" 
     .Create 
End With

'@ set optimizer parameters

'[VERSION]2019.0|28.0.2|20180920[/VERSION]
With Optimizer
  .SetMinMaxAuto "10" 
  .SetAlwaysStartFromCurrent "True" 
  .ResetParameterList
  .SelectParameter "d", "False" 
  .SetParameterInit "15" 
  .SetParameterMin "13.5" 
  .SetParameterMax "16.5" 
  .SetParameterAnchors "5" 
  .SelectParameter "freq", "False" 
  .SetParameterInit "1800*10^6" 
  .SetParameterMin "1.62e+9" 
  .SetParameterMax "1.98e+9" 
  .SetParameterAnchors "5" 
  .SelectParameter "reflector_dipole", "True" 
  .SetParameterInit "580" 
  .SetParameterMin "525" 
  .SetParameterMax "641.667" 
  .SetParameterAnchors "5" 
  .SelectParameter "s", "False" 
  .SetParameterInit "30" 
  .SetParameterMin "27" 
  .SetParameterMax "33" 
  .SetParameterAnchors "5" 
  .SelectParameter "velocity", "False" 
  .SetParameterInit "3*10^12" 
  .SetParameterMin "2.7e+12" 
  .SetParameterMax "3.3e+12" 
  .SetParameterAnchors "5" 
End With

'@ set optimizer settings

'[VERSION]2019.0|28.0.2|20180920[/VERSION]
With Optimizer 
  .SetOptimizerType "Trust_Region" 
  .SetSimulationType "Time Domain Solver" 
  .SetAccuracy "0.01" 
  .SetNumRefinements "1" 
  .SetGenerationSize "32", "Genetic_Algorithm" 
  .SetGenerationSize "30", "Particle_Swarm" 
  .SetMaxIt "30", "Genetic_Algorithm" 
  .SetMaxIt "15", "Particle_Swarm" 
  .SetMaxEval "3000", "CMAES" 
  .SetUseMaxEval "True", "CMAES" 
  .SetSigma "0.2", "CMAES" 
  .SetSeed "1", "CMAES" 
  .SetSeed "1", "Genetic_Algorithm" 
  .SetSeed "1", "Particle_Swarm" 
  .SetSeed "1", "Nelder_Mead_Simplex" 
  .SetMaxEval "500", "Trust_Region" 
  .SetUseMaxEval "False", "Trust_Region" 
  .SetSigma "0.2", "Trust_Region" 
  .SetDomainAccuracy "0.01", "Trust_Region" 
  .SetFiniteDiff "0", "Trust_Region" 
  .SetMaxEval "250", "Nelder_Mead_Simplex" 
  .SetUseMaxEval "False", "Nelder_Mead_Simplex" 
  .SetUseInterpolation "No_Interpolation", "Genetic_Algorithm" 
  .SetUseInterpolation "No_Interpolation", "Particle_Swarm" 
  .SetInitialDistribution "Latin_Hyper_Cube", "Genetic_Algorithm" 
  .SetInitialDistribution "Latin_Hyper_Cube", "Particle_Swarm" 
  .SetInitialDistribution "Noisy_Latin_Hyper_Cube", "Nelder_Mead_Simplex" 
  .SetUsePreDefPointInInitDistribution "True", "Nelder_Mead_Simplex" 
  .SetUsePreDefPointInInitDistribution "True", "CMAES" 
  .SetGoalFunctionLevel "0.0", "Genetic_Algorithm" 
  .SetGoalFunctionLevel "0.0", "Particle_Swarm" 
  .SetGoalFunctionLevel "0.0", "Nelder_Mead_Simplex" 
  .SetMutaionRate "60", "Genetic_Algorithm" 
  .SetMinSimplexSize "1e-6" 
  .SetGoalSummaryType "Sum_All_Goals" 
  .SetUseDataOfPreviousCalculations "False" 
  .SetDataStorageStrategy "None" 
  .SetOptionMoveMesh "False" 
End With

'@ define monitor: e-field (f=1.8)

'[VERSION]2019.0|28.0.2|20180920[/VERSION]
With Monitor 
     .Reset 
     .Name "e-field (f=1.8)" 
     .Dimension "Volume" 
     .Domain "Frequency" 
     .FieldType "Efield" 
     .MonitorValue "1.8" 
     .UseSubvolume "False" 
     .Coordinates "Structure" 
     .SetSubvolume "-0.70833333333334", "100.70833333333", "-0.70833333333334", "0.70833333333334", "-40.166666666667", "40.166666666667" 
     .SetSubvolumeOffset "0.0", "0.0", "0.0", "0.0", "0.0", "0.0" 
     .SetSubvolumeInflateWithOffset "False" 
     .Create 
End With

'@ define monitor: h-field (f=1.8)

'[VERSION]2019.0|28.0.2|20180920[/VERSION]
With Monitor 
     .Reset 
     .Name "h-field (f=1.8)" 
     .Dimension "Volume" 
     .Domain "Frequency" 
     .FieldType "Hfield" 
     .MonitorValue "1.8" 
     .UseSubvolume "False" 
     .Coordinates "Structure" 
     .SetSubvolume "-0.70833333333334", "100.70833333333", "-0.70833333333334", "0.70833333333334", "-40.166666666667", "40.166666666667" 
     .SetSubvolumeOffset "0.0", "0.0", "0.0", "0.0", "0.0", "0.0" 
     .SetSubvolumeInflateWithOffset "False" 
     .Create 
End With

'@ define farfield monitor: farfield (f=1.8)

'[VERSION]2019.0|28.0.2|20180920[/VERSION]
With Monitor 
     .Reset 
     .Name "farfield (f=1.8)" 
     .Domain "Frequency" 
     .FieldType "Farfield" 
     .MonitorValue "1.8" 
     .ExportFarfieldSource "False" 
     .UseSubvolume "False" 
     .Coordinates "Structure" 
     .SetSubvolume "-0.70833333333334", "100.70833333333", "-0.70833333333334", "0.70833333333334", "-40.166666666667", "40.166666666667" 
     .SetSubvolumeOffset "10", "10", "10", "10", "10", "10" 
     .SetSubvolumeInflateWithOffset "False" 
     .SetSubvolumeOffsetType "FractionOfWavelength" 
     .Create 
End With

'@ define fieldsource monitor: field-source (f=1.8)

'[VERSION]2019.0|28.0.2|20180920[/VERSION]
With Monitor 
     .Reset 
     .Name "field-source (f=1.8)" 
     .Domain "Frequency" 
     .FieldType "Fieldsource" 
     .Samples "1" 
     .MonitorValueRange "1.8", "1.8" 
     .InvertOrientation "False" 
     .UseSubvolume "False" 
     .Coordinates "Structure" 
     .SetSubvolume "-0.70833333333334", "100.70833333333", "-0.70833333333334", "0.70833333333334", "-40.166666666667", "40.166666666667" 
     .SetSubvolumeOffset "10", "10", "10", "10", "10", "10" 
     .SetSubvolumeInflateWithOffset "False" 
     .SetSubvolumeOffsetType "FractionOfWavelength" 
     .Create 
End With

'@ farfield plot options

'[VERSION]2019.0|28.0.2|20180920[/VERSION]
With FarfieldPlot 
     .Plottype "3D" 
     .Vary "angle1" 
     .Theta "90" 
     .Phi "90" 
     .Step "5" 
     .Step2 "5" 
     .SetLockSteps "True" 
     .SetPlotRangeOnly "False" 
     .SetThetaStart "0" 
     .SetThetaEnd "180" 
     .SetPhiStart "0" 
     .SetPhiEnd "360" 
     .SetTheta360 "False" 
     .SymmetricRange "False" 
     .SetTimeDomainFF "False" 
     .SetFrequency "-1" 
     .SetTime "0" 
     .SetColorByValue "True" 
     .DrawStepLines "False" 
     .DrawIsoLongitudeLatitudeLines "False" 
     .ShowStructure "False" 
     .ShowStructureProfile "False" 
     .SetStructureTransparent "False" 
     .SetFarfieldTransparent "False" 
     .SetSpecials "enablepolarextralines" 
     .SetPlotMode "Gain" 
     .Distance "1" 
     .UseFarfieldApproximation "True" 
     .SetScaleLinear "False" 
     .SetLogRange "40" 
     .SetLogNorm "0" 
     .DBUnit "0" 
     .SetMaxReferenceMode "abs" 
     .EnableFixPlotMaximum "False" 
     .SetFixPlotMaximumValue "1" 
     .SetInverseAxialRatio "False" 
     .SetAxesType "user" 
     .SetAntennaType "isotropic" 
     .Phistart "1.000000e+00", "0.000000e+00", "0.000000e+00" 
     .Thetastart "0.000000e+00", "0.000000e+00", "1.000000e+00" 
     .PolarizationVector "0.000000e+00", "1.000000e+00", "0.000000e+00" 
     .SetCoordinateSystemType "spherical" 
     .SetAutomaticCoordinateSystem "True" 
     .SetPolarizationType "Abs" 
     .SlantAngle 0.000000e+00 
     .Origin "bbox" 
     .Userorigin "0.000000e+00", "0.000000e+00", "0.000000e+00" 
     .SetUserDecouplingPlane "False" 
     .UseDecouplingPlane "False" 
     .DecouplingPlaneAxis "X" 
     .DecouplingPlanePosition "0.000000e+00" 
     .LossyGround "False" 
     .GroundEpsilon "1" 
     .GroundKappa "0" 
     .EnablePhaseCenterCalculation "False" 
     .SetPhaseCenterAngularLimit "3.000000e+01" 
     .SetPhaseCenterComponent "boresight" 
     .SetPhaseCenterPlane "both" 
     .ShowPhaseCenter "True" 
     .ClearCuts 
     .AddCut "lateral", "0", "1"  
     .AddCut "lateral", "90", "1"  
     .AddCut "polar", "90", "1"  

     .StoreSettings
End With 


