# PyChrono script generated from SolidWorks using Chrono::SolidWorks add-in 
# Assembly: C:\Users\lucan\Desktop\OneDrive - Politecnico di Torino\NANU_THRUSTPOD\Estero\Project\Thrust_Stand_Gyro\CAD\UAV\X8 copter\x8_copter_big_box.SLDASM


import pychrono as chrono 
import builtins 

# some global settings: 
sphereswept_r = 0.001
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.003)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.003)
chrono.ChCollisionSystemBullet.SetContactBreakingThreshold(0.002)

shapes_dir = 'x8copter_shapes/' 

if hasattr(builtins, 'exported_system_relpath'): 
    shapes_dir = builtins.exported_system_relpath + shapes_dir 

exported_items = [] 

body_0= chrono.ChBodyAuxRef()
body_0.SetName('ground')
body_0.SetBodyFixed(True)
exported_items.append(body_0)

# Rigid body part
body_1= chrono.ChBodyAuxRef()
body_1.SetName('x8_copter_big_box-1')
body_1.SetPos(chrono.ChVectorD(0,0,0))
body_1.SetRot(chrono.ChQuaternionD(0.707106781186548,0.707106781186547,0,0))
body_1.SetMass(1.99636134)
body_1.SetInertiaXX(chrono.ChVectorD(0.02167468,0.02225747,0.01555559))
body_1.SetInertiaXY(chrono.ChVectorD(5.81999999999797e-06,1.08000000000092e-06,0.000269180000000023))
body_1.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(-4.00500000000224e-05,-0.0064221899999998,0.05773638),chrono.ChQuaternionD(1,0,0,0)))

# Visualization shape 
body_1_1_shape = chrono.ChObjFileShape() 
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj') 
body_1.AddVisualShape(body_1_1_shape, chrono.ChFrameD(chrono.ChVectorD(0,0,0), chrono.ChQuaternionD(1,0,0,0)))

# Auxiliary marker (coordinate system feature)
marker_1_1 =chrono.ChMarker()
marker_1_1.SetName('Solidworks_inertial_coord_system')
body_1.AddMarker(marker_1_1)
marker_1_1.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(0,0,0),chrono.ChQuaternionD(0.707106781186549,-1.23442506554113E-15,-0.707106781186546,1.23442506554114E-15)))

# Auxiliary marker (coordinate system feature)
marker_1_2 =chrono.ChMarker()
marker_1_2.SetName('Solidworks_CoG_coord_system')
body_1.AddMarker(marker_1_2)
marker_1_2.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(-4.00500000000206E-05,-0.05773638,-0.0064221899999998),chrono.ChQuaternionD(0.707106781186549,-1.23442506554113E-15,-0.707106781186546,1.23442506554114E-15)))

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
marker_0_1.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(-0.0876060135845791,-0.00850000000000054,-0.108591504360591),chrono.ChQuaternionD(1,0,0,0)))

# Auxiliary marker (coordinate system feature)
marker_0_2 =chrono.ChMarker()
marker_0_2.SetName('Coordinate System2')
body_0.AddMarker(marker_0_2)
marker_0_2.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(0.0881269161570954,-0.00850000000000054,-0.108345020493486),chrono.ChQuaternionD(1,0,0,0)))

# Auxiliary marker (coordinate system feature)
marker_0_3 =chrono.ChMarker()
marker_0_3.SetName('Coordinate System3')
body_0.AddMarker(marker_0_3)
marker_0_3.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(0.0876060135845781,-0.00850000000000131,0.108591504360591),chrono.ChQuaternionD(1,0,0,0)))

# Auxiliary marker (coordinate system feature)
marker_0_4 =chrono.ChMarker()
marker_0_4.SetName('Coordinate System4')
body_0.AddMarker(marker_0_4)
marker_0_4.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(-0.0881269161570956,-0.00850000000000131,0.108345020493486),chrono.ChQuaternionD(1,0,0,0)))
