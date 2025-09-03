# PyChrono script generated from SolidWorks using Chrono::SolidWorks add-in 
# Assembly: C:\Users\lucan\Desktop\OneDrive - Politecnico di Torino\NANU_THRUSTPOD\Estero\Project\Thrust_Stand_Gyro\CAD\ThrustStand_UAV\thruststand_uav.SLDASM


import pychrono as chrono 
import builtins 

# some global settings: 
sphereswept_r = 0.001
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.003)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.003)
chrono.ChCollisionSystemBullet.SetContactBreakingThreshold(0.002)

shapes_dir = 'thruststand_uav_shapes/' 

if hasattr(builtins, 'exported_system_relpath'): 
    shapes_dir = builtins.exported_system_relpath + shapes_dir 

exported_items = [] 

body_0= chrono.ChBodyAuxRef()
body_0.SetName('ground')
body_0.SetBodyFixed(True)
exported_items.append(body_0)

# Rigid body part
body_1= chrono.ChBodyAuxRef()
body_1.SetName('thruststand_uav-1')
body_1.SetPos(chrono.ChVectorD(0,0,0))
body_1.SetRot(chrono.ChQuaternionD(1,0,0,0))
body_1.SetMass(1.152)
body_1.SetInertiaXX(chrono.ChVectorD(0.01067891,0.02660994,0.01731767))
body_1.SetInertiaXY(chrono.ChVectorD(-7.299e-05,1.14e-06,6.66e-06))
body_1.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(0.00010337,0.01518067,0.00012454),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_1_1_shape = chrono.ChObjFileShape() 
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj') 
body_1.AddVisualShape(body_1_1_shape, chrono.ChFrameD(chrono.ChVectorD(0,0,0), chrono.ChQuaternionD(1,0,0,0)))

# Collision material 
mat_1 = chrono.ChMaterialSurfaceNSC()

# Collision shapes 
body_1.GetCollisionModel().ClearModel()

# Triangle mesh collision shape 
body_1_1_collision_mesh = chrono.ChTriangleMeshConnected.CreateFromWavefrontFile(shapes_dir + 'body_1_1_collision.obj', False, True) 
mr = chrono.ChMatrix33D()
mr[0,0]=1; mr[1,0]=0; mr[2,0]=0 
mr[0,1]=0; mr[1,1]=1; mr[2,1]=0 
mr[0,2]=0; mr[1,2]=0; mr[2,2]=1 
body_1_1_collision_mesh.Transform(chrono.ChVectorD(0, 0, 0), mr) 
body_1.GetCollisionModel().AddTriangleMesh(mat_1, body_1_1_collision_mesh, False, False, chrono.ChVectorD(0,0,0), chrono.ChMatrix33D(chrono.ChQuaternionD(1,0,0,0)), sphereswept_r) 
body_1.GetCollisionModel().BuildModel()
body_1.SetCollide(True)

exported_items.append(body_1)




# Auxiliary marker (coordinate system feature)
marker_0_1 =chrono.ChMarker()
marker_0_1.SetName('Coordinate System1')
body_0.AddMarker(marker_0_1)
marker_0_1.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(0.15,0.02905,-0.17),chrono.ChQuaternionD(1,0,0,0)))

# Auxiliary marker (coordinate system feature)
marker_0_2 =chrono.ChMarker()
marker_0_2.SetName('Coordinate System2')
body_0.AddMarker(marker_0_2)
marker_0_2.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(0.15,0.02905,0.17),chrono.ChQuaternionD(1,0,0,0)))

# Auxiliary marker (coordinate system feature)
marker_0_3 =chrono.ChMarker()
marker_0_3.SetName('Coordinate System3')
body_0.AddMarker(marker_0_3)
marker_0_3.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(-0.15,0.02905,0.17),chrono.ChQuaternionD(1,0,0,0)))

# Auxiliary marker (coordinate system feature)
marker_0_4 =chrono.ChMarker()
marker_0_4.SetName('Coordinate System4')
body_0.AddMarker(marker_0_4)
marker_0_4.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(-0.15,0.02905,-0.17),chrono.ChQuaternionD(1,0,0,0)))
