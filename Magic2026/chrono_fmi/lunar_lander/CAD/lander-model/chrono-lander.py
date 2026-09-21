# PyChrono model automatically generated using Chrono::SolidWorks add-in
# Assembly: C:\Users\rdrah\sbel\lander-model\lander.SLDASM


import pychrono as chrono 
import builtins 

# Some global settings 
sphereswept_r = 0.001
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.003)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.003)
chrono.ChCollisionSystemBullet.SetContactBreakingThreshold(0.002)

shapes_dir = 'chrono-lander_shapes/' 

if hasattr(builtins, 'exported_system_relpath'): 
    shapes_dir = builtins.exported_system_relpath + shapes_dir 

exported_items = [] 

body_0 = chrono.ChBodyAuxRef()
body_0.SetName('SLDW_GROUND')
body_0.SetFixed(True)
exported_items.append(body_0)

# Rigid body part
body_1 = chrono.ChBodyAuxRef()
body_1.SetName('thruster-1')
body_1.SetPos(chrono.ChVector3d(0,0.139711392309965,0))
body_1.SetRot(chrono.ChQuaterniond(0.707106781186548,0,0.707106781186547,0))
body_1.SetMass(13.4812808624291)
body_1.SetInertiaXX(chrono.ChVector3d(0.511407274060236,0.51140727335552,0.499206259873547))
body_1.SetInertiaXY(chrono.ChVector3d(-7.23064275500502e-18,-1.93547984857144e-05,4.41015245082247e-18))
body_1.SetFrameCOMToRef(chrono.ChFramed(chrono.ChVector3d(0.290323606271142,-0.139711392309965,7.23005021746425e-06),chrono.ChQuaterniond(1,0,0,0)))

# Visualization shape 
body_1_1_shape = chrono.ChVisualShapeModelFile() 
body_1_1_shape.SetFilename(shapes_dir +'body_1_1.obj') 
body_1.AddVisualShape(body_1_1_shape, chrono.ChFramed(chrono.ChVector3d(0,0,0), chrono.ChQuaterniond(1,0,0,0)))

exported_items.append(body_1)



# Rigid body part
body_2 = chrono.ChBodyAuxRef()
body_2.SetName('leg-7/support-leg-1')
body_2.SetPos(chrono.ChVector3d(2.27595720048157e-14,2.10444179524214,0.267512881320046))
body_2.SetRot(chrono.ChQuaterniond(0.684221015078133,0.729274709917635,-3.65854944071676e-15,-3.73161203535652e-15))
body_2.SetMass(5.00758134437888)
body_2.SetInertiaXX(chrono.ChVector3d(0.0921873741742719,0.00885197806073956,0.0911481341030378))
body_2.SetInertiaXY(chrono.ChVector3d(-2.71923964947077e-09,1.18617878742307e-07,0.00529511167600013))
body_2.SetFrameCOMToRef(chrono.ChFramed(chrono.ChVector3d(-1.31864365640215e-08,-1.68277632481496e-05,0.0194886452274037),chrono.ChQuaterniond(1,0,0,0)))

# Visualization shape 
body_2_1_shape = chrono.ChVisualShapeModelFile() 
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj') 
body_2.AddVisualShape(body_2_1_shape, chrono.ChFramed(chrono.ChVector3d(0,0,0), chrono.ChQuaterniond(1,0,0,0)))

exported_items.append(body_2)



# Rigid body part
body_3 = chrono.ChBodyAuxRef()
body_3.SetName('leg-7/upper-leg-1')
body_3.SetPos(chrono.ChVector3d(2.42028619368284e-14,2.21160622912489,0.77380926851456))
body_3.SetRot(chrono.ChQuaterniond(0.454375116873351,0.890810447382791,-4.6338434729397e-15,-2.44147575392569e-15))
body_3.SetMass(13.8197277164169)
body_3.SetInertiaXX(chrono.ChVector3d(0.543668248317559,0.383579560861648,0.240037286551421))
body_3.SetInertiaXY(chrono.ChVector3d(-1.70672149665304e-07,1.25356639516862e-07,-0.219598149721329))
body_3.SetFrameCOMToRef(chrono.ChFramed(chrono.ChVector3d(-7.42752493944034e-10,0.289578374463467,-3.61943931725074e-08),chrono.ChQuaterniond(1,0,0,0)))

# Visualization shape 
body_3_1_shape = chrono.ChVisualShapeModelFile() 
body_3_1_shape.SetFilename(shapes_dir +'body_3_1.obj') 
body_3.AddVisualShape(body_3_1_shape, chrono.ChFramed(chrono.ChVector3d(0,0,0), chrono.ChQuaterniond(1,0,0,0)))

exported_items.append(body_3)



# Rigid body part
body_4 = chrono.ChBodyAuxRef()
body_4.SetName('leg-7/foot-pad-1')
body_4.SetPos(chrono.ChVector3d(3.41948691584548e-14,3.23900761519898,-0.767858085468066))
body_4.SetRot(chrono.ChQuaterniond(0.707106781186548,0.707106781186547,-3.64306164085106e-15,-3.74841675537474e-15))
body_4.SetMass(12.7623080096321)
body_4.SetInertiaXX(chrono.ChVector3d(0.220788812256542,0.222882689275691,0.42084860192368))
body_4.SetInertiaXY(chrono.ChVector3d(3.27629571156573e-08,9.08277011026837e-10,-3.32062566957794e-10))
body_4.SetFrameCOMToRef(chrono.ChFramed(chrono.ChVector3d(4.09467288537524e-09,0.0333119155790477,-2.12885734171292e-09),chrono.ChQuaterniond(1,0,0,0)))

# Visualization shape 
body_4_1_shape = chrono.ChVisualShapeModelFile() 
body_4_1_shape.SetFilename(shapes_dir +'body_4_1.obj') 
body_4.AddVisualShape(body_4_1_shape, chrono.ChFramed(chrono.ChVector3d(0,0,0), chrono.ChQuaterniond(1,0,0,0)))

exported_items.append(body_4)



# Rigid body part
body_5 = chrono.ChBodyAuxRef()
body_5.SetName('leg-7/lower-leg-1')
body_5.SetPos(chrono.ChVector3d(3.40838468559923e-14,3.1509446392506,-0.521429455124817))
body_5.SetRot(chrono.ChQuaterniond(0.32129172634358,0.629898108096196,0.321291726343573,0.629898108096192))
body_5.SetMass(26.5959831956266)
body_5.SetInertiaXX(chrono.ChVector3d(3.32297010741747,6.26223249725577,9.51150680936975))
body_5.SetInertiaXY(chrono.ChVector3d(4.49662892327225,0.0102008581437233,-0.00739613583174147))
body_5.SetFrameCOMToRef(chrono.ChFramed(chrono.ChVector3d(-0.00406351494729366,0.954936438278202,1.57256589739703e-07),chrono.ChQuaterniond(1,0,0,0)))

# Visualization shape 
body_5_1_shape = chrono.ChVisualShapeModelFile() 
body_5_1_shape.SetFilename(shapes_dir +'body_5_1.obj') 
body_5.AddVisualShape(body_5_1_shape, chrono.ChFramed(chrono.ChVector3d(0,0,0), chrono.ChQuaterniond(1,0,0,0)))

exported_items.append(body_5)



# Rigid body part
body_6 = chrono.ChBodyAuxRef()
body_6.SetName('chassis-v2-2')
body_6.SetPos(chrono.ChVector3d(0,0,0))
body_6.SetRot(chrono.ChQuaterniond(1,0,0,0))
body_6.SetMass(33685.3156280072)
body_6.SetInertiaXX(chrono.ChVector3d(60729.8911005872,60729.8911005878,48654.6689261419))
body_6.SetInertiaXY(chrono.ChVector3d(1.60278658645048e-15,1.88587600265932e-13,0.00879310111416166))
body_6.SetFrameCOMToRef(chrono.ChFramed(chrono.ChVector3d(-4.40843263160354e-17,-1.34849154165214e-07,1.76750943966413),chrono.ChQuaterniond(1,0,0,0)))
body_6.SetFixed(True)

# Visualization shape 
body_6_1_shape = chrono.ChVisualShapeModelFile() 
body_6_1_shape.SetFilename(shapes_dir +'body_6_1.obj') 
body_6.AddVisualShape(body_6_1_shape, chrono.ChFramed(chrono.ChVector3d(0,0,0), chrono.ChQuaterniond(1,0,0,0)))

exported_items.append(body_6)



# Rigid body part
body_7 = chrono.ChBodyAuxRef()
body_7.SetName('leg-5/lower-leg-1')
body_7.SetPos(chrono.ChVector3d(3.15094463924861,1.01252339845814e-13,-0.521429455130245))
body_7.SetRot(chrono.ChQuaterniond(0.672592782126696,0.672592782126696,-0.218217665258959,0.218217665258958))
body_7.SetMass(26.5959831956266)
body_7.SetInertiaXX(chrono.ChVector3d(9.51557211430411,9.51923230161429,0.0619049981245901))
body_7.SetInertiaXY(chrono.ChVector3d(0.00560414880262083,-0.0102009819064618,0.00739596517714099))
body_7.SetFrameCOMToRef(chrono.ChFramed(chrono.ChVector3d(-0.00406351494729366,0.954936438278202,1.57256589739703e-07),chrono.ChQuaterniond(1,0,0,0)))

# Visualization shape 
body_5_1_shape = chrono.ChVisualShapeModelFile() 
body_5_1_shape.SetFilename(shapes_dir +'body_5_1.obj') 
body_7.AddVisualShape(body_5_1_shape, chrono.ChFramed(chrono.ChVector3d(0,0,0), chrono.ChQuaterniond(1,0,0,0)))

exported_items.append(body_7)



# Rigid body part
body_8 = chrono.ChBodyAuxRef()
body_8.SetName('leg-5/support-leg-1')
body_8.SetPos(chrono.ChVector3d(2.1044417952051,-2.1094237467878e-15,0.267512881425508))
body_8.SetRot(chrono.ChQuaterniond(-0.483817319593942,-0.515675092728874,0.515675092728874,0.483817319593943))
body_8.SetMass(5.00758134437888)
body_8.SetInertiaXX(chrono.ChVector3d(0.0914874282825112,0.00885203212718103,0.0918480259283571))
body_8.SetInertiaXY(chrono.ChVector3d(2.18656222224749e-05,-1.51374949983231e-06,0.00531786631521547))
body_8.SetFrameCOMToRef(chrono.ChFramed(chrono.ChVector3d(-1.31864365640215e-08,-1.68277632481496e-05,0.0194886452274037),chrono.ChQuaterniond(1,0,0,0)))

# Visualization shape 
body_2_1_shape = chrono.ChVisualShapeModelFile() 
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj') 
body_8.AddVisualShape(body_2_1_shape, chrono.ChFramed(chrono.ChVector3d(0,0,0), chrono.ChQuaterniond(1,0,0,0)))

exported_items.append(body_8)



# Rigid body part
body_9 = chrono.ChBodyAuxRef()
body_9.SetName('leg-5/foot-pad-1')
body_9.SetPos(chrono.ChVector3d(3.23900761521096,-1.55431223447522e-15,-0.767858085461681))
body_9.SetRot(chrono.ChQuaterniond(-0.5,-0.5,0.5,0.5))
body_9.SetMass(12.7623080096321)
body_9.SetInertiaXX(chrono.ChVector3d(0.42084860192368,0.222882689275691,0.220788812256542))
body_9.SetInertiaXY(chrono.ChVector3d(-3.32062544129545e-10,-9.08274943707625e-10,-3.27629571156496e-08))
body_9.SetFrameCOMToRef(chrono.ChFramed(chrono.ChVector3d(4.09467288537524e-09,0.0333119155790477,-2.12885734171292e-09),chrono.ChQuaterniond(1,0,0,0)))

# Visualization shape 
body_4_1_shape = chrono.ChVisualShapeModelFile() 
body_4_1_shape.SetFilename(shapes_dir +'body_4_1.obj') 
body_9.AddVisualShape(body_4_1_shape, chrono.ChFramed(chrono.ChVector3d(0,0,0), chrono.ChQuaterniond(1,0,0,0)))

exported_items.append(body_9)



# Rigid body part
body_10 = chrono.ChBodyAuxRef()
body_10.SetName('leg-5/upper-leg-1')
body_10.SetPos(chrono.ChVector3d(2.21160622910132,8.33244584441672e-12,0.773809268514178))
body_10.SetRot(chrono.ChQuaterniond(-0.321291726339607,-0.629898108098218,0.629898108098219,0.321291726339608))
body_10.SetMass(13.8197277164169)
body_10.SetInertiaXX(chrono.ChVector3d(0.080779327076785,0.543124049100656,0.543381719553187))
body_10.SetInertiaXY(chrono.ChVector3d(-8.63579648368936e-08,6.10478035006021e-08,0.000394878176191743))
body_10.SetFrameCOMToRef(chrono.ChFramed(chrono.ChVector3d(-7.42752493944034e-10,0.289578374463467,-3.61943931725074e-08),chrono.ChQuaterniond(1,0,0,0)))

# Visualization shape 
body_3_1_shape = chrono.ChVisualShapeModelFile() 
body_3_1_shape.SetFilename(shapes_dir +'body_3_1.obj') 
body_10.AddVisualShape(body_3_1_shape, chrono.ChFramed(chrono.ChVector3d(0,0,0), chrono.ChQuaterniond(1,0,0,0)))

exported_items.append(body_10)



# Rigid body part
body_11 = chrono.ChBodyAuxRef()
body_11.SetName('leg-6/support-leg-1')
body_11.SetPos(chrono.ChVector3d(-8.21565038222616e-15,-2.10444179526678,0.267512881363877))
body_11.SetRot(chrono.ChQuaterniond(1.23371909118396e-15,1.22892199307795e-15,0.72927470988866,0.684221015109014))
body_11.SetMass(5.00758134437888)
body_11.SetInertiaXX(chrono.ChVector3d(0.0921873741742719,0.00884640625070765,0.0911537059130696))
body_11.SetInertiaXY(chrono.ChVector3d(1.77744504292445e-08,1.17310120346218e-07,0.0052516319433508))
body_11.SetFrameCOMToRef(chrono.ChFramed(chrono.ChVector3d(-1.31864365640215e-08,-1.68277632481496e-05,0.0194886452274037),chrono.ChQuaterniond(1,0,0,0)))

# Visualization shape 
body_2_1_shape = chrono.ChVisualShapeModelFile() 
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj') 
body_11.AddVisualShape(body_2_1_shape, chrono.ChFramed(chrono.ChVector3d(0,0,0), chrono.ChQuaterniond(1,0,0,0)))

exported_items.append(body_11)



# Rigid body part
body_12 = chrono.ChBodyAuxRef()
body_12.SetName('leg-6/upper-leg-1')
body_12.SetPos(chrono.ChVector3d(-7.7715611723761e-15,-2.21160622909629,0.773809268519279))
body_12.SetRot(chrono.ChQuaterniond(8.55033512538207e-16,1.5235954194503e-15,0.890810447379685,0.454375116879441))
body_12.SetMass(13.8197277164169)
body_12.SetInertiaXX(chrono.ChVector3d(0.543668248317559,0.383579761901431,0.240037085511639))
body_12.SetInertiaXY(chrono.ChVector3d(1.72174996187578e-07,-1.23284388462269e-07,-0.219598084015511))
body_12.SetFrameCOMToRef(chrono.ChFramed(chrono.ChVector3d(-7.42752493944034e-10,0.289578374463467,-3.61943931725074e-08),chrono.ChQuaterniond(1,0,0,0)))

# Visualization shape 
body_3_1_shape = chrono.ChVisualShapeModelFile() 
body_3_1_shape.SetFilename(shapes_dir +'body_3_1.obj') 
body_12.AddVisualShape(body_3_1_shape, chrono.ChFramed(chrono.ChVector3d(0,0,0), chrono.ChQuaterniond(1,0,0,0)))

exported_items.append(body_12)



# Rigid body part
body_13 = chrono.ChBodyAuxRef()
body_13.SetName('leg-6/lower-leg-1')
body_13.SetPos(chrono.ChVector3d(-1.14352971536391e-14,-3.15094463923166,-0.521429455176397))
body_13.SetRot(chrono.ChQuaterniond(0.629898108093997,0.321291726347882,-0.629898108093998,-0.321291726347884))
body_13.SetMass(26.5959831956266)
body_13.SetInertiaXX(chrono.ChVector3d(3.32296697814238,6.26223562653086,9.51150680936975))
body_13.SetInertiaXY(chrono.ChVector3d(4.49662790053149,0.0101991729588917,-0.0073984595060653))
body_13.SetFrameCOMToRef(chrono.ChFramed(chrono.ChVector3d(-0.00406351494729366,0.954936438278202,1.57256589739703e-07),chrono.ChQuaterniond(1,0,0,0)))

# Visualization shape 
body_5_1_shape = chrono.ChVisualShapeModelFile() 
body_5_1_shape.SetFilename(shapes_dir +'body_5_1.obj') 
body_13.AddVisualShape(body_5_1_shape, chrono.ChFramed(chrono.ChVector3d(0,0,0), chrono.ChQuaterniond(1,0,0,0)))

exported_items.append(body_13)



# Rigid body part
body_14 = chrono.ChBodyAuxRef()
body_14.SetName('leg-6/foot-pad-1')
body_14.SetPos(chrono.ChVector3d(-1.13242748511766e-14,-3.23900761518929,-0.767858085526826))
body_14.SetRot(chrono.ChQuaterniond(1.27767814077807e-15,1.175069143363e-15,0.707106781186548,0.707106781186547))
body_14.SetMass(12.7623080096321)
body_14.SetInertiaXX(chrono.ChVector3d(0.220788812256542,0.222882689275691,0.42084860192368))
body_14.SetInertiaXY(chrono.ChVector3d(-3.27629571162731e-08,9.08275613725139e-10,3.3206258069421e-10))
body_14.SetFrameCOMToRef(chrono.ChFramed(chrono.ChVector3d(4.09467288537524e-09,0.0333119155790477,-2.12885734171292e-09),chrono.ChQuaterniond(1,0,0,0)))

# Visualization shape 
body_4_1_shape = chrono.ChVisualShapeModelFile() 
body_4_1_shape.SetFilename(shapes_dir +'body_4_1.obj') 
body_14.AddVisualShape(body_4_1_shape, chrono.ChFramed(chrono.ChVector3d(0,0,0), chrono.ChQuaterniond(1,0,0,0)))

exported_items.append(body_14)



# Rigid body part
body_15 = chrono.ChBodyAuxRef()
body_15.SetName('leg-8/support-leg-1')
body_15.SetPos(chrono.ChVector3d(-2.10444179526026,3.50519613334654e-12,0.26751288137458))
body_15.SetRot(chrono.ChQuaterniond(0.483817319600993,0.51567509272226,0.51567509272226,0.483817319600993))
body_15.SetMass(5.00758134437888)
body_15.SetInertiaXX(chrono.ChVector3d(0.0914874282825112,0.00885202951668249,0.0918480285388556))
body_15.SetInertiaXY(chrono.ChVector3d(-2.18806774331811e-05,1.27782150013095e-06,0.00531784594413716))
body_15.SetFrameCOMToRef(chrono.ChFramed(chrono.ChVector3d(-1.31864365640215e-08,-1.68277632481496e-05,0.0194886452274037),chrono.ChQuaterniond(1,0,0,0)))

# Visualization shape 
body_2_1_shape = chrono.ChVisualShapeModelFile() 
body_2_1_shape.SetFilename(shapes_dir +'body_2_1.obj') 
body_15.AddVisualShape(body_2_1_shape, chrono.ChFramed(chrono.ChVector3d(0,0,0), chrono.ChQuaterniond(1,0,0,0)))

exported_items.append(body_15)



# Rigid body part
body_16 = chrono.ChBodyAuxRef()
body_16.SetName('leg-8/upper-leg-1')
body_16.SetPos(chrono.ChVector3d(-2.21160622912227,-5.20072873655408e-12,0.773809268509846))
body_16.SetRot(chrono.ChQuaterniond(0.321291726345014,0.629898108095461,0.629898108095461,0.321291726345013))
body_16.SetMass(13.8197277164169)
body_16.SetInertiaXX(chrono.ChVector3d(0.080779327076785,0.543123646538779,0.543382122115065))
body_16.SetInertiaXY(chrono.ChVector3d(8.48551150158734e-08,-6.31200584486561e-08,0.000394746606920626))
body_16.SetFrameCOMToRef(chrono.ChFramed(chrono.ChVector3d(-7.42752493944034e-10,0.289578374463467,-3.61943931725074e-08),chrono.ChQuaterniond(1,0,0,0)))

# Visualization shape 
body_3_1_shape = chrono.ChVisualShapeModelFile() 
body_3_1_shape.SetFilename(shapes_dir +'body_3_1.obj') 
body_16.AddVisualShape(body_3_1_shape, chrono.ChFramed(chrono.ChVector3d(0,0,0), chrono.ChQuaterniond(1,0,0,0)))

exported_items.append(body_16)



# Rigid body part
body_17 = chrono.ChBodyAuxRef()
body_17.SetName('leg-8/lower-leg-1')
body_17.SetPos(chrono.ChVector3d(-3.15094463925514,-1.82820425465025e-12,-0.521429455153892))
body_17.SetRot(chrono.ChQuaterniond(-0.218217665253186,0.218217665253186,0.672592782128569,0.672592782128569))
body_17.SetMass(26.5959831956266)
body_17.SetInertiaXX(chrono.ChVector3d(9.51556938590803,9.51923503001037,0.0619049981245901))
body_17.SetInertiaXY(chrono.ChVector3d(0.00560325708111398,-0.0101990491959138,0.00739863016081882))
body_17.SetFrameCOMToRef(chrono.ChFramed(chrono.ChVector3d(-0.00406351494729366,0.954936438278202,1.57256589739703e-07),chrono.ChQuaterniond(1,0,0,0)))

# Visualization shape 
body_5_1_shape = chrono.ChVisualShapeModelFile() 
body_5_1_shape.SetFilename(shapes_dir +'body_5_1.obj') 
body_17.AddVisualShape(body_5_1_shape, chrono.ChFramed(chrono.ChVector3d(0,0,0), chrono.ChQuaterniond(1,0,0,0)))

exported_items.append(body_17)



# Rigid body part
body_18 = chrono.ChBodyAuxRef()
body_18.SetName('leg-8/foot-pad-1')
body_18.SetPos(chrono.ChVector3d(-3.23900761519599,1.33226762955019e-15,-0.767858085491453))
body_18.SetRot(chrono.ChQuaterniond(0.5,0.5,0.5,0.5))
body_18.SetMass(12.7623080096321)
body_18.SetInertiaXX(chrono.ChVector3d(0.42084860192368,0.222882689275692,0.220788812256542))
body_18.SetInertiaXY(chrono.ChVector3d(3.32062595589688e-10,-9.08274917748348e-10,3.27629571159402e-08))
body_18.SetFrameCOMToRef(chrono.ChFramed(chrono.ChVector3d(4.09467288537524e-09,0.0333119155790477,-2.12885734171292e-09),chrono.ChQuaterniond(1,0,0,0)))

# Visualization shape 
body_4_1_shape = chrono.ChVisualShapeModelFile() 
body_4_1_shape.SetFilename(shapes_dir +'body_4_1.obj') 
body_18.AddVisualShape(body_4_1_shape, chrono.ChFramed(chrono.ChVector3d(0,0,0), chrono.ChQuaterniond(1,0,0,0)))

exported_items.append(body_18)




# Mate constraint: Hinge1 [MateHinge] type:22 align:1 flip:False
#   Entity 0: C::E name: body_16 , SW name: leg-8/upper-leg-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_6 , SW name: chassis-v2-2 ,  SW ref.type:2 (2)
#   Entity 2: C::E name: body_16 , SW name: leg-8/upper-leg-1 ,  SW ref.type:2 (2)
#   Entity 3: C::E name: body_6 , SW name: chassis-v2-2 ,  SW ref.type:2 (2)
link_1 = chrono.ChLinkMateCylindrical()
cA = chrono.ChVector3d(-1.83000000000845,-0.100000000005201,1.29999999999561)
dA = chrono.ChVector3d(9.37431872258553e-16,1,-9.57151469755346e-17)
cB = chrono.ChVector3d(-1.83,-0.0749999999999999,1.3)
dB = chrono.ChVector3d(0,1,-1.38777878078145e-16)
link_1.SetName("Hinge1")
link_1.Initialize(body_16,body_6,False,cA,cB,dA,dB)
exported_items.append(link_1)
link_2 = chrono.ChLinkMateDistanceZ()
cA = chrono.ChVector3d(-1.76016946457771,-0.0250000000052011,1.31112188479345)
dA = chrono.ChVector3d(-9.37431872258553e-16,-1,9.57151469755346e-17)
cB = chrono.ChVector3d(-1.88,-0.0249999999999999,1.35)
dB = chrono.ChVector3d(0,1,0)
link_2.SetName("Hinge1")
link_2.Initialize(body_16,body_6,False,cA,cB,dB)
exported_items.append(link_2)

# Mate constraint: Hinge3 [MateHinge] type:22 align:1 flip:False
#   Entity 0: C::E name: body_15 , SW name: leg-8/support-leg-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_6 , SW name: chassis-v2-2 ,  SW ref.type:2 (2)
#   Entity 2: C::E name: body_15 , SW name: leg-8/support-leg-1 ,  SW ref.type:2 (2)
#   Entity 3: C::E name: body_6 , SW name: chassis-v2-2 ,  SW ref.type:2 (2)
link_3 = chrono.ChLinkMateCylindrical()
cA = chrono.ChVector3d(-1.82999999999979,0.0625000000035052,0.249999999995058)
dA = chrono.ChVector3d(7.37177616633535e-17,-1,1.4020274165223e-16)
cB = chrono.ChVector3d(-1.83,-0.0749999999999999,0.25)
dB = chrono.ChVector3d(0,-1,1.38777878078145e-16)
link_3.SetName("Hinge3")
link_3.Initialize(body_15,body_6,False,cA,cB,dA,dB)
exported_items.append(link_3)
link_4 = chrono.ChLinkMateDistanceZ()
cA = chrono.ChVector3d(-1.78328565202144,0.0250000000035052,0.19691733151506)
dA = chrono.ChVector3d(-7.37177616633535e-17,1,-1.4020274165223e-16)
cB = chrono.ChVector3d(-1.88,0.0250000000000001,0.3)
dB = chrono.ChVector3d(0,-1,0)
link_4.SetName("Hinge3")
link_4.Initialize(body_15,body_6,False,cA,cB,dB)
exported_items.append(link_4)

# Mate constraint: Hinge4 [MateHinge] type:22 align:1 flip:False
#   Entity 0: C::E name: body_3 , SW name: leg-7/upper-leg-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_6 , SW name: chassis-v2-2 ,  SW ref.type:2 (2)
#   Entity 2: C::E name: body_3 , SW name: leg-7/upper-leg-1 ,  SW ref.type:2 (2)
#   Entity 3: C::E name: body_6 , SW name: chassis-v2-2 ,  SW ref.type:2 (2)
link_5 = chrono.ChLinkMateCylindrical()
cA = chrono.ChVector3d(-0.0999999999999797,1.83000000000867,1.29999999999858)
dA = chrono.ChVector3d(1,-1.04744440165294e-14,-1.38777878077975e-16)
cB = chrono.ChVector3d(-0.0749999999999807,1.83,1.3)
dB = chrono.ChVector3d(1,-1.04744440165294e-14,-1.38777878078145e-16)
link_5.SetName("Hinge4")
link_5.Initialize(body_3,body_6,False,cA,cB,dA,dB)
exported_items.append(link_5)
link_6 = chrono.ChLinkMateDistanceZ()
cA = chrono.ChVector3d(0.0250000000000196,1.76016946457788,1.31112188479611)
dA = chrono.ChVector3d(1,-1.04744440165294e-14,-1.38777878077975e-16)
cB = chrono.ChVector3d(0.0250000000000198,1.88,1.35)
dB = chrono.ChVector3d(-1,1.04744440165294e-14,0)
link_6.SetName("Hinge4")
link_6.Initialize(body_3,body_6,False,cA,cB,dB)
exported_items.append(link_6)

# Mate constraint: Hinge5 [MateHinge] type:22 align:1 flip:False
#   Entity 0: C::E name: body_2 , SW name: leg-7/support-leg-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_6 , SW name: chassis-v2-2 ,  SW ref.type:2 (2)
#   Entity 2: C::E name: body_2 , SW name: leg-7/support-leg-1 ,  SW ref.type:2 (2)
#   Entity 3: C::E name: body_6 , SW name: chassis-v2-2 ,  SW ref.type:2 (2)
link_7 = chrono.ChLinkMateCylindrical()
cA = chrono.ChVector3d(0.0625000000000199,1.82999999998228,0.249999999931048)
dA = chrono.ChVector3d(-1,1.04426699136149e-14,4.36227745138058e-16)
cB = chrono.ChVector3d(-0.0749999999999807,1.83,0.25)
dB = chrono.ChVector3d(-1,1.04744440165294e-14,1.38777878078145e-16)
link_7.SetName("Hinge5")
link_7.Initialize(body_2,body_6,False,cA,cB,dA,dB)
exported_items.append(link_7)
link_8 = chrono.ChLinkMateDistanceZ()
cA = chrono.ChVector3d(0.0250000000000194,1.78328565200576,0.196917331449438)
dA = chrono.ChVector3d(1,-1.04426699136149e-14,-4.36227745138058e-16)
cB = chrono.ChVector3d(0.0250000000000198,1.88,0.3)
dB = chrono.ChVector3d(-1,1.04744440165294e-14,0)
link_8.SetName("Hinge5")
link_8.Initialize(body_2,body_6,False,cA,cB,dB)
exported_items.append(link_8)

# Mate constraint: Hinge6 [MateHinge] type:22 align:1 flip:False
#   Entity 0: C::E name: body_10 , SW name: leg-5/upper-leg-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_6 , SW name: chassis-v2-2 ,  SW ref.type:2 (2)
#   Entity 2: C::E name: body_10 , SW name: leg-5/upper-leg-1 ,  SW ref.type:2 (2)
#   Entity 3: C::E name: body_6 , SW name: chassis-v2-2 ,  SW ref.type:2 (2)
link_9 = chrono.ChLinkMateCylindrical()
cA = chrono.ChVector3d(1.82999999997846,0.100000000008333,1.29999999999339)
dA = chrono.ChVector3d(-2.02539885311095e-15,-1,-7.37121477458271e-16)
cB = chrono.ChVector3d(1.83,0.0749999999999999,1.3)
dB = chrono.ChVector3d(0,-1,-1.38777878078145e-16)
link_9.SetName("Hinge6")
link_9.Initialize(body_10,body_6,False,cA,cB,dA,dB)
exported_items.append(link_9)
link_10 = chrono.ChLinkMateDistanceZ()
cA = chrono.ChVector3d(1.76016946454753,-0.024999999991667,1.31112188479004)
dA = chrono.ChVector3d(-2.02539885311095e-15,-1,-7.37121477458271e-16)
cB = chrono.ChVector3d(1.88,-0.0250000000000001,1.35)
dB = chrono.ChVector3d(0,1,0)
link_10.SetName("Hinge6")
link_10.Initialize(body_10,body_6,False,cA,cB,dB)
exported_items.append(link_10)

# Mate constraint: Hinge7 [MateHinge] type:22 align:1 flip:False
#   Entity 0: C::E name: body_8 , SW name: leg-5/support-leg-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_6 , SW name: chassis-v2-2 ,  SW ref.type:2 (2)
#   Entity 2: C::E name: body_6 , SW name: chassis-v2-2 ,  SW ref.type:2 (2)
#   Entity 3: C::E name: body_8 , SW name: leg-5/support-leg-1 ,  SW ref.type:2 (2)
link_11 = chrono.ChLinkMateCylindrical()
cA = chrono.ChVector3d(1.82999999994511,-0.0625000000000016,0.250000000038482)
dA = chrono.ChVector3d(1.85284831631374e-15,1,7.15213792393899e-16)
cB = chrono.ChVector3d(1.83,0.0749999999999999,0.25)
dB = chrono.ChVector3d(0,1,1.38777878078145e-16)
link_11.SetName("Hinge7")
link_11.Initialize(body_8,body_6,False,cA,cB,dA,dB)
exported_items.append(link_11)
link_12 = chrono.ChLinkMateDistanceZ()
cA = chrono.ChVector3d(1.88,-0.0250000000000001,0.3)
dA = chrono.ChVector3d(0,1,0)
cB = chrono.ChVector3d(1.78328565196821,-0.0250000000000015,0.196917331557207)
dB = chrono.ChVector3d(-1.85284831631374e-15,-1,-7.15213792393899e-16)
link_12.SetName("Hinge7")
link_12.Initialize(body_6,body_8,False,cA,cB,dB)
exported_items.append(link_12)

# Mate constraint: Hinge8 [MateHinge] type:22 align:1 flip:False
#   Entity 0: C::E name: body_6 , SW name: chassis-v2-2 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_12 , SW name: leg-6/upper-leg-1 ,  SW ref.type:2 (2)
#   Entity 2: C::E name: body_12 , SW name: leg-6/upper-leg-1 ,  SW ref.type:2 (2)
#   Entity 3: C::E name: body_6 , SW name: chassis-v2-2 ,  SW ref.type:2 (2)
link_13 = chrono.ChLinkMateCylindrical()
cA = chrono.ChVector3d(0.0749999999999935,-1.83,1.3)
dA = chrono.ChVector3d(-1,3.49148133884313e-15,-1.38777878078145e-16)
cB = chrono.ChVector3d(0.0999999999999935,-1.82999999998726,1.30000000000852)
dB = chrono.ChVector3d(-1,3.49148133884309e-15,-1.38777878078151e-16)
link_13.SetName("Hinge8")
link_13.Initialize(body_6,body_12,False,cA,cB,dA,dB)
exported_items.append(link_13)
link_14 = chrono.ChLinkMateDistanceZ()
cA = chrono.ChVector3d(-0.0250000000000063,-1.76016946455662,1.311121884807)
dA = chrono.ChVector3d(-1,3.49148133884309e-15,-1.38777878078151e-16)
cB = chrono.ChVector3d(-0.0250000000000067,-1.88,1.35)
dB = chrono.ChVector3d(1,-3.49148133884313e-15,0)
link_14.SetName("Hinge8")
link_14.Initialize(body_12,body_6,False,cA,cB,dB)
exported_items.append(link_14)

# Mate constraint: Hinge9 [MateHinge] type:22 align:1 flip:False
#   Entity 0: C::E name: body_11 , SW name: leg-6/support-leg-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_6 , SW name: chassis-v2-2 ,  SW ref.type:2 (2)
#   Entity 2: C::E name: body_11 , SW name: leg-6/support-leg-1 ,  SW ref.type:2 (2)
#   Entity 3: C::E name: body_6 , SW name: chassis-v2-2 ,  SW ref.type:2 (2)
link_15 = chrono.ChLinkMateCylindrical()
cA = chrono.ChVector3d(-0.0625000000000073,-1.83000000000544,0.249999999998122)
dA = chrono.ChVector3d(1,-3.48071651781395e-15,1.17731757427394e-16)
cB = chrono.ChVector3d(0.0749999999999935,-1.83,0.25)
dB = chrono.ChVector3d(1,-3.49148133884313e-15,1.38777878078145e-16)
link_15.SetName("Hinge9")
link_15.Initialize(body_11,body_6,False,cA,cB,dA,dB)
exported_items.append(link_15)
link_16 = chrono.ChLinkMateDistanceZ()
cA = chrono.ChVector3d(-0.0250000000000071,-1.78328565202442,0.196917331520468)
dA = chrono.ChVector3d(-1,3.48071651781395e-15,-1.17731757427394e-16)
cB = chrono.ChVector3d(-0.0250000000000067,-1.88,0.3)
dB = chrono.ChVector3d(1,-3.49148133884313e-15,0)
link_16.SetName("Hinge9")
link_16.Initialize(body_11,body_6,False,cA,cB,dB)
exported_items.append(link_16)

# Mate constraint: Parallel4 [MateParallel] type:3 align:1 flip:False
#   Entity 0: C::E name: body_14 , SW name: leg-6/foot-pad-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_0 , SW name: lander ,  SW ref.type:4 (4)
link_17 = chrono.ChLinkMateParallel()
cA = chrono.ChVector3d(-1.13242748511766e-14,-3.23900761518929,-0.767858085526826)
dA = chrono.ChVector3d(1.45111035765907e-16,-6.93889390390723e-17,-1)
cB = chrono.ChVector3d(0,0,0)
dB = chrono.ChVector3d(0,0,1)
link_17.SetFlipped(True)
link_17.Initialize(body_14,body_0,False,cA,cB,dA,dB)
link_17.SetName("Parallel4")
exported_items.append(link_17)


# Mate constraint: Parallel5 [MateParallel] type:3 align:1 flip:False
#   Entity 0: C::E name: body_18 , SW name: leg-8/foot-pad-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_0 , SW name: lander ,  SW ref.type:4 (4)
link_18 = chrono.ChLinkMateParallel()
cA = chrono.ChVector3d(-3.23900761519599,1.33226762955019e-15,-0.767858085491453)
dA = chrono.ChVector3d(-1.38777878078145e-17,-1.44632154757323e-16,-1)
cB = chrono.ChVector3d(0,0,0)
dB = chrono.ChVector3d(0,0,1)
link_18.SetFlipped(True)
link_18.Initialize(body_18,body_0,False,cA,cB,dA,dB)
link_18.SetName("Parallel5")
exported_items.append(link_18)


# Mate constraint: Parallel6 [MateParallel] type:3 align:1 flip:False
#   Entity 0: C::E name: body_9 , SW name: leg-5/foot-pad-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_0 , SW name: lander ,  SW ref.type:4 (4)
link_19 = chrono.ChLinkMateParallel()
cA = chrono.ChVector3d(3.23900761521096,-1.55431223447522e-15,-0.767858085461681)
dA = chrono.ChVector3d(-1.52655665885959e-16,1.15312324677736e-16,-1)
cB = chrono.ChVector3d(0,0,0)
dB = chrono.ChVector3d(0,0,1)
link_19.SetFlipped(True)
link_19.Initialize(body_9,body_0,False,cA,cB,dA,dB)
link_19.SetName("Parallel6")
exported_items.append(link_19)


# Mate constraint: Parallel7 [MateParallel] type:3 align:1 flip:False
#   Entity 0: C::E name: body_4 , SW name: leg-7/foot-pad-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_0 , SW name: lander ,  SW ref.type:4 (4)
link_20 = chrono.ChLinkMateParallel()
cA = chrono.ChVector3d(3.41948691584548e-14,3.23900761519898,-0.767858085468066)
dA = chrono.ChVector3d(-1.48994631824758e-16,0,-1)
cB = chrono.ChVector3d(0,0,0)
dB = chrono.ChVector3d(0,0,1)
link_20.SetFlipped(True)
link_20.Initialize(body_4,body_0,False,cA,cB,dA,dB)
link_20.SetName("Parallel7")
exported_items.append(link_20)


# Mate constraint: Coincident8 [MateCoincident] type:0 align:1 flip:False
#   Entity 0: C::E name: body_6 , SW name: chassis-v2-2 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_1 , SW name: thruster-1 ,  SW ref.type:2 (2)
link_21 = chrono.ChLinkMateDistanceZ()
cA = chrono.ChVector3d(0,0,0)
cB = chrono.ChVector3d(0,0.139711392309965,0)
dA = chrono.ChVector3d(0,0,-1)
dB = chrono.ChVector3d(0,0,1)
link_21.Initialize(body_6,body_1,False,cA,cB,dB)
link_21.SetDistance(0)
link_21.SetName("Coincident8")
exported_items.append(link_21)

link_22 = chrono.ChLinkMateParallel()
cA = chrono.ChVector3d(0,0,0)
dA = chrono.ChVector3d(0,0,-1)
cB = chrono.ChVector3d(0,0.139711392309965,0)
dB = chrono.ChVector3d(0,0,1)
link_22.SetFlipped(True)
link_22.Initialize(body_6,body_1,False,cA,cB,dA,dB)
link_22.SetName("Coincident8")
exported_items.append(link_22)


# Mate constraint: Concentric2 [MateConcentric] type:1 align:0 flip:False
#   Entity 0: C::E name: body_6 , SW name: chassis-v2-2 ,  SW ref.type:1 (1)
#   Entity 1: C::E name: body_1 , SW name: thruster-1 ,  SW ref.type:1 (1)
link_23 = chrono.ChLinkMateParallel()
cA = chrono.ChVector3d(0,0,0)
dA = chrono.ChVector3d(0,0,1)
cB = chrono.ChVector3d(0,0,0)
dB = chrono.ChVector3d(0,0,1)
link_23.Initialize(body_6,body_1,False,cA,cB,dA,dB)
link_23.SetName("Concentric2")
exported_items.append(link_23)

link_24 = chrono.ChLinkMateGeneric()
link_24.SetConstrainedCoords(True, True, False, False, False, False)
cA = chrono.ChVector3d(0,0,0)
cB = chrono.ChVector3d(0,0,0)
dA = chrono.ChVector3d(0,0,1)
dB = chrono.ChVector3d(0,0,1)
link_24.Initialize(body_6,body_1,False,cA,cB,dA,dB)
link_24.SetName("Concentric2")
exported_items.append(link_24)


# Mate constraint: Concentric2 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_5 , SW name: leg-7/lower-leg-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_3 , SW name: leg-7/upper-leg-1 ,  SW ref.type:2 (2)
link_25 = chrono.ChLinkMateParallel()
cA = chrono.ChVector3d(3.40838468559923e-14,3.1509446392506,-0.521429455124817)
dA = chrono.ChVector3d(-6.05446549687042e-15,-0.587086506332658,0.809524202283116)
cB = chrono.ChVector3d(2.17603712826531e-14,1.97677162659183,1.09761894942781)
dB = chrono.ChVector3d(6.03706069239562e-15,0.587086506332657,-0.809524202283116)
link_25.SetFlipped(True)
link_25.Initialize(body_5,body_3,False,cA,cB,dA,dB)
link_25.SetName("Concentric2")
exported_items.append(link_25)

link_26 = chrono.ChLinkMateGeneric()
link_26.SetConstrainedCoords(True, True, False, False, False, False)
cA = chrono.ChVector3d(3.40838468559923e-14,3.1509446392506,-0.521429455124817)
cB = chrono.ChVector3d(2.17603712826531e-14,1.97677162659183,1.09761894942781)
dA = chrono.ChVector3d(-6.05446549687042e-15,-0.587086506332658,0.809524202283116)
dB = chrono.ChVector3d(6.03706069239562e-15,0.587086506332657,-0.809524202283116)
link_26.Initialize(body_5,body_3,False,cA,cB,dA,dB)
link_26.SetName("Concentric2")
exported_items.append(link_26)


# Mate constraint: Hinge3 [MateHinge] type:22 align:1 flip:False
#   Entity 0: C::E name: body_5 , SW name: leg-7/lower-leg-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_4 , SW name: leg-7/foot-pad-1 ,  SW ref.type:2 (2)
#   Entity 2: C::E name: body_5 , SW name: leg-7/lower-leg-1 ,  SW ref.type:2 (2)
#   Entity 3: C::E name: body_4 , SW name: leg-7/foot-pad-1 ,  SW ref.type:2 (2)
link_27 = chrono.ChLinkMateCylindrical()
cA = chrono.ChVector3d(0.0625000000000349,3.23900761520049,-0.642858085467284)
dA = chrono.ChVector3d(-1,1.04614456228241e-14,1.07851086818932e-16)
cB = chrono.ChVector3d(0.145710678118689,3.23900761519898,-0.642858085468066)
dB = chrono.ChVector3d(-1,1.04531289939303e-14,1.48994631824758e-16)
link_27.SetName("Hinge3")
link_27.Initialize(body_5,body_4,False,cA,cB,dA,dB)
exported_items.append(link_27)
link_28 = chrono.ChLinkMateDistanceZ()
cA = chrono.ChVector3d(-0.0249999999999652,3.22788573040297,-0.712688620898073)
dA = chrono.ChVector3d(-1,1.04614456228241e-14,1.07851086818932e-16)
cB = chrono.ChVector3d(-0.0249999999999664,3.18900761519898,-0.592858085468066)
dB = chrono.ChVector3d(1,-1.04531289939303e-14,-1.48994631824758e-16)
link_28.SetName("Hinge3")
link_28.Initialize(body_5,body_4,False,cA,cB,dB)
exported_items.append(link_28)

# Mate constraint: Hinge4 [MateHinge] type:22 align:1 flip:False
#   Entity 0: C::E name: body_2 , SW name: leg-7/support-leg-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_5 , SW name: leg-7/lower-leg-1 ,  SW ref.type:2 (2)
#   Entity 2: C::E name: body_2 , SW name: leg-7/support-leg-1 ,  SW ref.type:2 (2)
#   Entity 3: C::E name: body_5 , SW name: leg-7/lower-leg-1 ,  SW ref.type:2 (2)
link_29 = chrono.ChLinkMateCylindrical()
cA = chrono.ChVector3d(0.0628750000000257,2.378883590502,0.285025762709043)
dA = chrono.ChVector3d(-1,1.04426699136149e-14,4.36227745138058e-16)
cB = chrono.ChVector3d(0.0400000000000261,2.3788835905212,0.285025762779289)
dB = chrono.ChVector3d(-1,1.04614456228241e-14,1.07851086818932e-16)
link_29.SetName("Hinge4")
link_29.Initialize(body_2,body_5,False,cA,cB,dA,dB)
exported_items.append(link_29)
link_30 = chrono.ChLinkMateDistanceZ()
cA = chrono.ChVector3d(-0.0199999999999743,2.39141062034985,0.2457437881763)
dA = chrono.ChVector3d(-1,1.04426699136149e-14,4.36227745138058e-16)
cB = chrono.ChVector3d(-0.0199999999999744,2.33955265905791,0.244367702935885)
dB = chrono.ChVector3d(1,-1.04607261850877e-14,-1.51213205893754e-16)
link_30.SetName("Hinge4")
link_30.Initialize(body_2,body_5,False,cA,cB,dB)
exported_items.append(link_30)

# Mate constraint: Concentric2 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_7 , SW name: leg-5/lower-leg-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_10 , SW name: leg-5/upper-leg-1 ,  SW ref.type:2 (2)
link_31 = chrono.ChLinkMateParallel()
cA = chrono.ChVector3d(3.15094463924861,1.01252339845814e-13,-0.521429455130245)
dA = chrono.ChVector3d(-0.587086506342859,5.27195577257219e-16,0.809524202275717)
cB = chrono.ChVector3d(1.97677162656418,8.33266788902165e-12,1.09761894942446)
dB = chrono.ChVector3d(0.58708650634286,-5.9236666060404e-16,-0.809524202275717)
link_31.SetFlipped(True)
link_31.Initialize(body_7,body_10,False,cA,cB,dA,dB)
link_31.SetName("Concentric2")
exported_items.append(link_31)

link_32 = chrono.ChLinkMateGeneric()
link_32.SetConstrainedCoords(True, True, False, False, False, False)
cA = chrono.ChVector3d(3.15094463924861,1.01252339845814e-13,-0.521429455130245)
cB = chrono.ChVector3d(1.97677162656418,8.33266788902165e-12,1.09761894942446)
dA = chrono.ChVector3d(-0.587086506342859,5.27195577257219e-16,0.809524202275717)
dB = chrono.ChVector3d(0.58708650634286,-5.9236666060404e-16,-0.809524202275717)
link_32.Initialize(body_7,body_10,False,cA,cB,dA,dB)
link_32.SetName("Concentric2")
exported_items.append(link_32)


# Mate constraint: Hinge3 [MateHinge] type:22 align:1 flip:False
#   Entity 0: C::E name: body_7 , SW name: leg-5/lower-leg-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_9 , SW name: leg-5/foot-pad-1 ,  SW ref.type:2 (2)
#   Entity 2: C::E name: body_7 , SW name: leg-5/lower-leg-1 ,  SW ref.type:2 (2)
#   Entity 3: C::E name: body_9 , SW name: leg-5/foot-pad-1 ,  SW ref.type:2 (2)
link_33 = chrono.ChLinkMateCylindrical()
cA = chrono.ChVector3d(3.23900761520004,-0.0624999999998989,-0.642858085471603)
dA = chrono.ChVector3d(1.88578751669217e-15,1,7.16377380679035e-16)
cB = chrono.ChVector3d(3.23900761521096,-0.145710678118656,-0.642858085461681)
dB = chrono.ChVector3d(1.19622137191426e-16,1,1.15312324677736e-16)
link_33.SetName("Hinge3")
link_33.Initialize(body_7,body_9,False,cA,cB,dA,dB)
exported_items.append(link_33)
link_34 = chrono.ChLinkMateDistanceZ()
cA = chrono.ChVector3d(3.2278857304034,0.0250000000001013,-0.712688620902532)
dA = chrono.ChVector3d(1.88578751669217e-15,1,7.16377380679035e-16)
cB = chrono.ChVector3d(3.18900761521096,0.0249999999999985,-0.592858085461682)
dB = chrono.ChVector3d(-1.19622137191426e-16,-1,-1.15312324677736e-16)
link_34.SetName("Hinge3")
link_34.Initialize(body_7,body_9,False,cA,cB,dB)
exported_items.append(link_34)

# Mate constraint: Hinge4 [MateHinge] type:22 align:1 flip:False
#   Entity 0: C::E name: body_8 , SW name: leg-5/support-leg-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_7 , SW name: leg-5/lower-leg-1 ,  SW ref.type:2 (2)
#   Entity 2: C::E name: body_8 , SW name: leg-5/support-leg-1 ,  SW ref.type:2 (2)
#   Entity 3: C::E name: body_7 , SW name: leg-5/lower-leg-1 ,  SW ref.type:2 (2)
link_35 = chrono.ChLinkMateCylindrical()
cA = chrono.ChVector3d(2.37888359046509,-0.0628750000000026,0.285025762812534)
dA = chrono.ChVector3d(1.85284831631374e-15,1,7.15213792393899e-16)
cB = chrono.ChVector3d(2.37888359050906,-0.0399999999998978,0.285025762764131)
dB = chrono.ChVector3d(1.88578751669217e-15,1,7.16377380679035e-16)
link_35.SetName("Hinge4")
link_35.Initialize(body_8,body_7,False,cA,cB,dA,dB)
exported_items.append(link_35)
link_36 = chrono.ChLinkMateDistanceZ()
cA = chrono.ChVector3d(2.39141062031265,0.0199999999999974,0.2457437882797)
dA = chrono.ChVector3d(1.85284831631374e-15,1,7.15213792393899e-16)
cB = chrono.ChVector3d(2.33955265904628,0.0200000000001023,0.24436770292023)
dB = chrono.ChVector3d(-1.88506807895518e-15,-1,-7.59739499753848e-16)
link_36.SetName("Hinge4")
link_36.Initialize(body_8,body_7,False,cA,cB,dB)
exported_items.append(link_36)

# Mate constraint: Concentric2 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_13 , SW name: leg-6/lower-leg-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_12 , SW name: leg-6/upper-leg-1 ,  SW ref.type:2 (2)
link_37 = chrono.ChLinkMateParallel()
cA = chrono.ChVector3d(-1.14352971536391e-14,-3.15094463923166,-0.521429455176397)
dA = chrono.ChVector3d(1.94949713649005e-15,0.587086506321588,0.809524202291143)
cB = chrono.ChVector3d(-6.99440505513849e-15,-1.97677162656765,1.09761894943574)
dB = chrono.ChVector3d(-1.93745753006154e-15,-0.58708650632159,-0.809524202291142)
link_37.SetFlipped(True)
link_37.Initialize(body_13,body_12,False,cA,cB,dA,dB)
link_37.SetName("Concentric2")
exported_items.append(link_37)

link_38 = chrono.ChLinkMateGeneric()
link_38.SetConstrainedCoords(True, True, False, False, False, False)
cA = chrono.ChVector3d(-1.14352971536391e-14,-3.15094463923166,-0.521429455176397)
cB = chrono.ChVector3d(-6.99440505513849e-15,-1.97677162656765,1.09761894943574)
dA = chrono.ChVector3d(1.94949713649005e-15,0.587086506321588,0.809524202291143)
dB = chrono.ChVector3d(-1.93745753006154e-15,-0.58708650632159,-0.809524202291142)
link_38.Initialize(body_13,body_12,False,cA,cB,dA,dB)
link_38.SetName("Concentric2")
exported_items.append(link_38)


# Mate constraint: Hinge3 [MateHinge] type:22 align:1 flip:False
#   Entity 0: C::E name: body_13 , SW name: leg-6/lower-leg-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_14 , SW name: leg-6/foot-pad-1 ,  SW ref.type:2 (2)
#   Entity 2: C::E name: body_13 , SW name: leg-6/lower-leg-1 ,  SW ref.type:2 (2)
#   Entity 3: C::E name: body_14 , SW name: leg-6/foot-pad-1 ,  SW ref.type:2 (2)
link_39 = chrono.ChLinkMateCylindrical()
cA = chrono.ChVector3d(-0.0625000000000118,-3.2390076151799,-0.642858085520068)
dA = chrono.ChVector3d(1,-3.48568409899704e-15,1.19701131919937e-16)
cB = chrono.ChVector3d(-0.145710678118666,-3.23900761518929,-0.642858085526826)
dB = chrono.ChVector3d(1,-3.46870847430608e-15,1.45111035765907e-16)
link_39.SetName("Hinge3")
link_39.Initialize(body_13,body_14,False,cA,cB,dA,dB)
exported_items.append(link_39)
link_40 = chrono.ChLinkMateDistanceZ()
cA = chrono.ChVector3d(0.0249999999999883,-3.22788573038143,-0.712688620950705)
dA = chrono.ChVector3d(1,-3.48568409899704e-15,1.19701131919937e-16)
cB = chrono.ChVector3d(0.0249999999999889,-3.18900761518929,-0.592858085526825)
dB = chrono.ChVector3d(-1,3.46870847430608e-15,-1.45111035765907e-16)
link_40.SetName("Hinge3")
link_40.Initialize(body_13,body_14,False,cA,cB,dB)
exported_items.append(link_40)

# Mate constraint: Hinge4 [MateHinge] type:22 align:1 flip:False
#   Entity 0: C::E name: body_11 , SW name: leg-6/support-leg-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_13 , SW name: leg-6/lower-leg-1 ,  SW ref.type:2 (2)
#   Entity 2: C::E name: body_11 , SW name: leg-6/support-leg-1 ,  SW ref.type:2 (2)
#   Entity 3: C::E name: body_13 , SW name: leg-6/lower-leg-1 ,  SW ref.type:2 (2)
link_41 = chrono.ChLinkMateCylindrical()
cA = chrono.ChVector3d(-0.0628750000000091,-2.37888359052813,0.285025762729632)
dA = chrono.ChVector3d(1,-3.48071651781395e-15,1.17731757427394e-16)
cB = chrono.ChVector3d(-0.0400000000000088,-2.3788835905133,0.285025762738266)
dB = chrono.ChVector3d(1,-3.48568409899704e-15,1.19701131919937e-16)
link_41.SetName("Hinge4")
link_41.Initialize(body_11,body_13,False,cA,cB,dA,dB)
exported_items.append(link_41)
link_42 = chrono.ChLinkMateDistanceZ()
cA = chrono.ChVector3d(0.0199999999999909,-2.39141062037265,0.245743788195827)
dA = chrono.ChVector3d(1,-3.48071651781395e-15,1.17731757427394e-16)
cB = chrono.ChVector3d(0.0199999999999914,-2.33955265904945,0.2443677028954)
dB = chrono.ChVector3d(-1,3.48496466126119e-15,-1.63063250994769e-16)
link_42.SetName("Hinge4")
link_42.Initialize(body_11,body_13,False,cA,cB,dB)
exported_items.append(link_42)

# Mate constraint: Concentric2 [MateConcentric] type:1 align:1 flip:False
#   Entity 0: C::E name: body_17 , SW name: leg-8/lower-leg-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_16 , SW name: leg-8/upper-leg-1 ,  SW ref.type:2 (2)
link_43 = chrono.ChLinkMateParallel()
cA = chrono.ChVector3d(-3.15094463925514,-1.82820425465025e-12,-0.521429455153892)
dA = chrono.ChVector3d(0.587086506328965,-4.19313042483029e-16,0.809524202285794)
cB = chrono.ChVector3d(-1.97677162659069,-5.20095078115901e-12,1.09761894942416)
dB = chrono.ChVector3d(-0.587086506328965,4.72869874803657e-16,-0.809524202285794)
link_43.SetFlipped(True)
link_43.Initialize(body_17,body_16,False,cA,cB,dA,dB)
link_43.SetName("Concentric2")
exported_items.append(link_43)

link_44 = chrono.ChLinkMateGeneric()
link_44.SetConstrainedCoords(True, True, False, False, False, False)
cA = chrono.ChVector3d(-3.15094463925514,-1.82820425465025e-12,-0.521429455153892)
cB = chrono.ChVector3d(-1.97677162659069,-5.20095078115901e-12,1.09761894942416)
dA = chrono.ChVector3d(0.587086506328965,-4.19313042483029e-16,0.809524202285794)
dB = chrono.ChVector3d(-0.587086506328965,4.72869874803657e-16,-0.809524202285794)
link_44.Initialize(body_17,body_16,False,cA,cB,dA,dB)
link_44.SetName("Concentric2")
exported_items.append(link_44)


# Mate constraint: Hinge3 [MateHinge] type:22 align:1 flip:False
#   Entity 0: C::E name: body_17 , SW name: leg-8/lower-leg-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_18 , SW name: leg-8/foot-pad-1 ,  SW ref.type:2 (2)
#   Entity 2: C::E name: body_17 , SW name: leg-8/lower-leg-1 ,  SW ref.type:2 (2)
#   Entity 3: C::E name: body_18 , SW name: leg-8/foot-pad-1 ,  SW ref.type:2 (2)
link_45 = chrono.ChLinkMateCylindrical()
cA = chrono.ChVector3d(-3.23900761520448,0.0624999999981719,-0.642858085496761)
dA = chrono.ChVector3d(-1.06105273855995e-15,-1,2.51526393224531e-16)
cB = chrono.ChVector3d(-3.23900761519599,0.145710678118656,-0.642858085491452)
dB = chrono.ChVector3d(1.01355006689552e-17,-1,1.44632154757323e-16)
link_45.SetName("Hinge3")
link_45.Initialize(body_17,body_18,False,cA,cB,dA,dB)
exported_items.append(link_45)
link_46 = chrono.ChLinkMateDistanceZ()
cA = chrono.ChVector3d(-3.22788573040664,-0.0250000000018281,-0.712688620927499)
dA = chrono.ChVector3d(-1.06105273855995e-15,-1,2.51526393224531e-16)
cB = chrono.ChVector3d(-3.18900761519599,-0.0249999999999987,-0.592858085491452)
dB = chrono.ChVector3d(-1.01355006689552e-17,1,-1.44632154757323e-16)
link_46.SetName("Hinge3")
link_46.Initialize(body_17,body_18,False,cA,cB,dB)
exported_items.append(link_46)

# Mate constraint: Hinge4 [MateHinge] type:22 align:1 flip:False
#   Entity 0: C::E name: body_15 , SW name: leg-8/support-leg-1 ,  SW ref.type:2 (2)
#   Entity 1: C::E name: body_17 , SW name: leg-8/lower-leg-1 ,  SW ref.type:2 (2)
#   Entity 2: C::E name: body_15 , SW name: leg-8/support-leg-1 ,  SW ref.type:2 (2)
#   Entity 3: C::E name: body_17 , SW name: leg-8/lower-leg-1 ,  SW ref.type:2 (2)
link_47 = chrono.ChLinkMateCylindrical()
cA = chrono.ChVector3d(-2.37888359052072,0.0628750000035052,0.285025762754103)
dA = chrono.ChVector3d(7.37177616633535e-17,-1,1.4020274165223e-16)
cB = chrono.ChVector3d(-2.37888359052942,0.0399999999981712,0.285025762753736)
dB = chrono.ChVector3d(-1.06105273855995e-15,-1,2.51526393224531e-16)
link_47.SetName("Hinge4")
link_47.Initialize(body_15,body_17,False,cA,cB,dA,dB)
exported_items.append(link_47)
link_48 = chrono.ChLinkMateDistanceZ()
cA = chrono.ChVector3d(-2.39141062036721,-0.0199999999964948,0.245743788220927)
dA = chrono.ChVector3d(7.37177616633535e-17,-1,1.4020274165223e-16)
cB = chrono.ChVector3d(-2.33955265906595,-0.0200000000018289,0.244367702910512)
dB = chrono.ChVector3d(1.0603333008237e-15,1,-2.94888512299356e-16)
link_48.SetName("Hinge4")
link_48.Initialize(body_15,body_17,False,cA,cB,dB)
exported_items.append(link_48)
