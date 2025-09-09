# =============================================================================
# PROJECT CHRONO - http://projectchrono.org
#
# Copyright (c) 2019 projectchrono.org
# All rights reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http://projectchrono.org/license-chrono.txt.
#
# =============================================================================
#
# Authors: Bo-Hsun Chen, modified from demo_SEN_camera.py, demo_ROBOT_Viper_Rigid.py, and demo_VEH_DeformableSoil.py
#
# =============================================================================
#
# Chrono tutorial of a camera sensor.
# Generate several rock objects, rotate camera sensor around the scene, and 
# install a front-end camera in the front of a moving VIPER
#
# =============================================================================

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr  # for visualization
import pychrono.sensor as sens
import pychrono.robot as viper

import math
import time
from math import pi

def main():
	# -----------------
	# Create the system
	# -----------------
	m_phys_system = chrono.ChSystemNSC()

	#### ----------------------- ####
	#### PART 1: BUILD THE SCENE ####
	#### ----------------------- ####

	# -------------------------- #
	# Add objects into the scene #
	# -------------------------- #

	### Add 5 rows by 4 columns of rocks to the system ###
	rocks = []
	for rock_row_idx in range(num_rock_rows):
		for rock_col_idx in range(num_rock_cols):
			
			rock_idx = rock_row_idx * num_rock_cols + rock_col_idx

			## Create a contact mesh ##
			ct_mesh = chrono.ChTriangleMeshConnected()
			ct_mesh.LoadWavefrontMesh(
				chrono.GetChronoDataFile("sensor/offroad/boulder_rock/boulder_rock_contact.obj"),
				True,	# whether load normals
				True	# whether load UVs
			)
			
			# scale to a different size
			ct_mesh.Transform(chrono.ChVector3d(0, 0, 0), chrono.ChMatrix33d(rock_scale))

			rock_ct_shape = chrono.ChCollisionShapeTriangleMesh(
				chrono.ChContactMaterialNSC(),	# contact material
				ct_mesh,						# the mesh 
				True,							# is it static?
				False,							# is it convex?
				0.001,							# "thickness" for increased robustness
			)
			
			## Create a visual mesh, with loading normals and UVs ##
			visual_mesh = chrono.ChTriangleMeshConnected()
			visual_mesh.LoadWavefrontMesh(
				chrono.GetChronoDataFile("sensor/offroad/boulder_rock/boulder_rock.obj"),
				True,		# whether load normals
				True,		# whether load UVs
			)
			
			# scale to a different size
			visual_mesh.Transform(chrono.ChVector3d(0, 0, 0), chrono.ChMatrix33d(rock_scale))

			rock_visual_shape = chrono.ChVisualShapeTriangleMesh()
			rock_visual_shape.SetMesh(visual_mesh)
			rock_visual_shape.SetMutable(False)

			## Create visual material ##
			vis_mat = chrono.ChVisualMaterial()
			
			# Use uniform material
			if (rock_row_idx < 4):
				rock_diffuse = rock_diffuses[rock_idx]
				rock_diffuse = chrono.ChColor(rock_diffuse[0], rock_diffuse[1], rock_diffuse[2])
				vis_mat.SetDiffuseColor(rock_diffuse) # must be ChColor
				vis_mat.SetUseSpecularWorkflow(False)
				vis_mat.SetRoughness(rock_row_idx / 3.0)
				vis_mat.SetMetallic(rock_col_idx / 3.0)

			# Use material textures
			else:  
				vis_mat.SetKdTexture(chrono.GetChronoDataFile("sensor/offroad/boulder_rock/boulder_rock_diffuse.png"))
				vis_mat.SetRoughnessTexture(chrono.GetChronoDataFile("sensor/offroad/boulder_rock/boulder_rock_roughness.png"))
				vis_mat.SetMetallicTexture(chrono.GetChronoDataFile("sensor/offroad/boulder_rock/boulder_rock_metallic.png"))
				vis_mat.SetNormalMapTexture(chrono.GetChronoDataFile("sensor/offroad/boulder_rock/boulder_rock_normal.png"))
				vis_mat.SetUseSpecularWorkflow(False)

			# Set visual class and instance IDs of the rock for segmentation camera
			vis_mat.SetClassID(65535) 
			vis_mat.SetInstanceID((65535 // num_rocks) * (rock_idx + 1))

			## Create the body
			rock = chrono.ChBody()
			
			# Put to the desired pose
			posi_x = (rock_col_idx - (num_rock_cols - 1) / 2) * cols_inter
			posi_y = ((num_rock_rows - 1) / 2 - rock_row_idx) * rows_inter
			rock.SetPos(chrono.ChVector3d(posi_x, posi_y, 0))
			
			# Assign visual and contact shapes, and visual material to the rock
			rock.AddVisualShape(rock_visual_shape)
			rock.GetVisualShape(0).SetMaterial(0, vis_mat)
			rock.AddCollisionShape(rock_ct_shape)
			rock.SetFixed(True)
			rock.EnableCollision(True)

			## Add the rock to the system ##
			rocks.append(rock)
			m_phys_system.Add(rocks[-1])


	### Add a cuboid ground to the system ###
	ground_length	= (num_rock_rows - 1) * rows_inter + 2 * space_pad
	ground_width	= (num_rock_cols - 1) * cols_inter + 2 * space_pad
	print(f"ground length: {ground_length: .1f} m , ground width: {ground_width: .1f} m")
	
	## Define visual material
	vis_mat = chrono.ChVisualMaterial()
	vis_mat.SetDiffuseColor(chrono.ChColor(0.7, 0.7, 0.7)) # must be ChColor
	vis_mat.SetClassID(0)
	vis_mat.SetInstanceID(32768)

	## Create ground body
	ground = chrono.ChBodyEasyBox(
		ground_width,					# size along X-axis
		ground_length,					# size along Y-axis
		ground_depth,					# size along Z-axis
		1000,							# density, [kg/m^3]
		True,							# visible?
		True,							# contactable?
		chrono.ChContactMaterialNSC()	# contact material
	)
	ground.GetVisualShape(0).SetMaterial(0, vis_mat)

	ground.SetPos(chrono.ChVector3d(0, 0, ground_posi_z))
	ground.SetFixed(True)

	m_phys_system.Add(ground)

	### Add a mirror ball to the system ###
	## Define visual material
	vis_mat = chrono.ChVisualMaterial()
	vis_mat.SetDiffuseColor(chrono.ChColor(1.0, 1.0, 1.0)) # must be ChColor
	vis_mat.SetUseSpecularWorkflow(False)
	vis_mat.SetRoughness(0.)
	vis_mat.SetMetallic(1.)

	## Create mirror ball body
	mirror_ball = chrono.ChBody()

	# Assign visual shape, visual material, and pose to the mirror ball
	mirror_ball.AddVisualShape(chrono.ChVisualShapeSphere(2.0))
	mirror_ball.GetVisualShape(0).SetMaterial(0, vis_mat)
	mirror_ball.SetFixed(True)
	mirror_ball.SetPos(chrono.ChVector3d(0., 0., min(ground_length, ground_width) / 2))

	## Add the mirror ball to the system ##
	# m_phys_system.Add(mirror_ball)	# UNCOMMENT IT!

	# ------------------------------------------------------ #
	# Create a sensor manager, and add background and lights #
	# ------------------------------------------------------ #
	manager = sens.ChSensorManager(m_phys_system)

	## Set background
	bgd = sens.Background()
	
	# Set a uniform-colored background
	bgd.mode = sens.BackgroundMode_SOLID_COLOR
	bgd.color_zenith = chrono.ChVector3f(0.0, 0.0, 0.0)
	
	# Or set as environment map
	# bgd.mode = sens.BackgroundMode_ENVIRONMENT_MAP 									# UNCOMMENT IT!
	# bgd.env_tex = chrono.GetChronoDataFile("sensor/textures/starmap_2020_4k.hdr")	# UNCOMMENT IT!

	manager.scene.SetBackground(bgd)

	## Add a point light
	sun_intensity = 1.0
	sun_color = chrono.ChColor(sun_intensity, sun_intensity, sun_intensity)
	sun_radius = 100.0 # [m]
	sun_elevation = 45 * pi/180. # [rad], CHANGE THE VALUE!
	sun_azimuth = 45 * pi/180. # [rad], CHANGE THE VALUE!
	
	sun_x = sun_radius * math.cos(sun_elevation) * math.cos(sun_azimuth)
	sun_y = sun_radius * math.cos(sun_elevation) * math.sin(sun_azimuth)
	sun_z = sun_radius * math.sin(sun_elevation)

	manager.scene.AddPointLight(
		chrono.ChVector3f(sun_x, sun_y, sun_z),	# point light's position
		sun_color,								# point light's color
		10 * sun_radius,						# point light's max range
	)

	## Set ambient light
	ambient_intensity = 0.05
	ambient_color = chrono.ChVector3f(ambient_intensity, ambient_intensity, ambient_intensity)
	manager.scene.SetAmbientLight(ambient_color)

	# ------------------------------------------------- #
	# Create sensors and add them to the sensor manager #
	# ------------------------------------------------- #
	camera_hight = 4.0 # [m]
	camera_azimuth = 0 * pi/180.0 # [rad]
	offset_pose = chrono.ChFramed(
		chrono.ChVector3d(
			cam_radius * math.cos(camera_azimuth),
			cam_radius * math.sin(camera_azimuth),
			camera_hight
		),
			chrono.QuatFromAngleAxis(pi + camera_azimuth, chrono.ChVector3d(0, 0, 1))
		* chrono.QuatFromAngleAxis(math.atan2(camera_hight, cam_radius), chrono.ChVector3d(0, 1, 0))
	)
	
	### Create a color camera ###
	cam = sens.ChCameraSensor(
		ground,					# body camera is attached to
		update_rate,			# update rate in Hz
		offset_pose,			# offset pose
		image_width,			# image width, [pixel]
		image_height,			# image height, [pixel]
		fov						# camera's horizontal field of view (FOV)
	)
	cam.SetName("Camera Sensor")
	cam.SetLag(lag)
	cam.SetCollectionWindow(exposure_time)

	## Create a Filter Graph for post-processing the data from the camera ##
	if noise_model == "CONST_NORMAL":
		cam.PushFilter(sens.ChFilterCameraNoiseConstNormal(0.0, 0.2))
	
	elif noise_model == "PIXEL_DEPENDENT":
		cam.PushFilter(sens.ChFilterCameraNoisePixDep(0.02, 0.03))
	
	elif noise_model == "NONE":
		# Don't add any noise models
		pass

	# Renders the image at current point in the filter graph
	if vis:
		cam.PushFilter(sens.ChFilterVisualize(image_width, image_height, "Before Grayscale Filter"))

	# Provides the host access to this RGBA8 buffer
	cam.PushFilter(sens.ChFilterRGBA8Access())

	# Save the current image to a png file at the specified path
	if save:
		cam.PushFilter(sens.ChFilterSave(out_dir + "rgb/"))

	# Filter the sensor to grayscale
	cam.PushFilter(sens.ChFilterGrayscale())

	# Render the buffer again to see the new grayscaled image
	# if vis:
	# 	cam.PushFilter(sens.ChFilterVisualize(image_width // 2, image_height // 2, "Grayscale Image"))

	# Save the grayscaled image at the specified path
	if save:
		cam.PushFilter(sens.ChFilterSave(out_dir + "gray/"))

	# Resizes the image to the provided width and height
	cam.PushFilter(sens.ChFilterImageResize(int(image_width / 2), int(image_height / 2)))

	# Access the grayscaled buffer as R8 pixels
	cam.PushFilter(sens.ChFilterR8Access())

	# Add sensor to manager
	manager.AddSensor(cam)

	### Create a depth camera and add it to the sensor manager ###
	depth_cam = sens.ChDepthCamera(
		ground,					# body camera is attached to
		update_rate,			# update rate in Hz
		offset_pose,			# offset pose
		image_width,			# image width, [pixel]
		image_height,			# image height, [pixel]
		fov						# camera's horizontal field of view (FOV)
	)

	depth_cam.SetName("Depth Camera")
	depth_cam.SetLag(lag)
	depth_cam.SetCollectionWindow(exposure_time)
	depth_cam.SetMaxDepth(18.0) # [m], normalize the depth data from 0 to 255

	if vis:
		depth_cam.PushFilter(sens.ChFilterVisualize(image_width, image_height, "Depth Map"))

	if save:
		cam.PushFilter(sens.ChFilterSave(out_dir + "depth/"))

	# manager.AddSensor(depth_cam)
	
	### Create a segmentation camera and add it to the sensor manager ###
	segment_cam = sens.ChSegmentationCamera(
		ground,					# body camera is attached to
		update_rate,			# update rate in Hz
		offset_pose,			# offset pose
		image_width,			# image width, [pixel]
		image_height,			# image height, [pixel]
		fov						# camera's horizontal field of view (FOV)
	)

	segment_cam.SetName("Segmentation Camera")
	segment_cam.SetLag(lag)
	segment_cam.SetCollectionWindow(exposure_time)

	if vis:
		segment_cam.PushFilter(sens.ChFilterVisualize(image_width, image_height, "Segmentation Map"))

	if save:
		segment_cam.PushFilter(sens.ChFilterSave(out_dir + "segment/"))

	# manager.AddSensor(segment_cam)

	#### --------------------------- ####
	#### PART 2: ADD A RUNNING VIPER ####
	#### --------------------------- ####
	
	m_phys_system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
	m_phys_system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
	chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
	chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

	# ------------------ #
	# Create VIPER rover #
	# ------------------ #
	driver = viper.ViperDCMotorControl()
	rover = viper.Viper(m_phys_system)
	rover.SetDriver(driver)
	rover.Initialize(chrono.ChFramed(
		chrono.ChVector3d(0.03, - ground_length / 2 + 1.5, 0.6),
		chrono.QuatFromAngleAxis(pi / 2, chrono.ChVector3d(0, 0, 1))
	))

	### Create a VIPER observer ###
	viper_observer_radius = 4
	viper_observer_hight = 3.0
	viper_observer_angle = 270 * pi/180.0 # [rad]
	viper_observer_offset = chrono.ChFramed(
		chrono.ChVector3d(
			viper_observer_radius * math.cos(viper_observer_angle),
			viper_observer_radius * math.sin(viper_observer_angle),
			viper_observer_hight
		),
		chrono.QuatFromAngleAxis(pi + viper_observer_angle, chrono.ChVector3d(0, 0, 1))
		* chrono.QuatFromAngleAxis(
			math.atan2(viper_observer_hight, viper_observer_radius),
			chrono.ChVector3d(0, 1, 0)
		)
	)
	viper_observer = sens.ChCameraSensor(
		rover.GetChassis().GetBody(),	# body camera is attached to
		update_rate,					# update rate in Hz
		viper_observer_offset,			# offset pose
		image_width,					# image width, [pixel]
		image_height,					# image height, [pixel]
		fov								# camera's horizontal field of view (FOV)
	)
	
	viper_observer.SetName("VIPER Observer Camera")
	viper_observer.SetCollectionWindow(exposure_time)

	if vis:
		viper_observer.PushFilter(sens.ChFilterVisualize(image_width, image_height, "VIPER Observer"))

	if save:
		viper_observer.PushFilter(sens.ChFilterSave(out_dir + "viper_observer/"))

	manager.AddSensor(viper_observer)

	### Create a VIPER front-end camera ###
	viper_front_cam_offset = chrono.ChFramed(
		chrono.ChVector3d(0.8, 0., 1.0),
		chrono.QuatFromAngleAxis(30 * pi/180.0, chrono.ChVector3d(0, 1, 0))
	)
	viper_front_cam = sens.ChCameraSensor(
		rover.GetChassis().GetBody(),	# body camera is attached to
		update_rate,			# update rate in Hz
		viper_front_cam_offset,	# offset pose
		image_width,			# image width, [pixel]
		image_height,			# image height, [pixel]
		fov						# camera's horizontal field of view (FOV)
	)
	
	viper_front_cam.SetName("VIPER Front Camera")
	viper_front_cam.SetCollectionWindow(exposure_time)

	if vis:
		viper_front_cam.PushFilter(sens.ChFilterVisualize(image_width, image_height, "VIPER Front-end Camera"))

	if save:
		viper_front_cam.PushFilter(sens.ChFilterSave(out_dir + "viper_front_cam/"))

	manager.AddSensor(viper_front_cam)
	

	# ------------------- #
	# Simulate The System #
	# ------------------- #
	orbit_rate = 0.10
	fix_camera = True

	ch_time = 0.0 # [sec]

	t1 = time.time()

	# while (ch_time < end_time):
	while (True):
		
		if (fix_camera == False):
			camera_azimuth = ch_time * orbit_rate
			cam_pose = chrono.ChFramed(
				chrono.ChVector3d(
					cam_radius * math.cos(camera_azimuth),
					cam_radius * math.sin(camera_azimuth),
					camera_hight
				),
				chrono.QuatFromAngleAxis(pi + camera_azimuth, chrono.ChVector3d(0, 0, 1))
				* chrono.QuatFromAngleAxis(math.atan2(camera_hight, cam_radius), chrono.ChVector3d(0, 1, 0))
			)

			cam.SetOffsetPose(cam_pose)
			depth_cam.SetOffsetPose(cam_pose)
			segment_cam.SetOffsetPose(cam_pose)

		# Access the RGBA8 buffer from the camera
		rgba8_buffer = cam.GetMostRecentRGBA8Buffer()
		if (rgba8_buffer.HasData()):
			rgba8_data = rgba8_buffer.GetRGBA8Data()
			# print(f"RGBA8 buffer resolution: {rgba8_buffer.Width} x {rgba8_buffer.Height}")
			# print(f"middle Pixel: {rgba8_data[image_height // 2, image_width // 2, :]}")

		# Update sensor manager
		# Will render/save/filter automatically
		manager.Update()

		# Perform step of dynamics
		m_phys_system.DoStepDynamics(step_size)

		# Get the current time of the simulation
		ch_time = m_phys_system.GetChTime()

		#### PART 2 ####
		
		# P-control steering
		x_offset = rover.GetChassis().GetBody().GetPos().x
		steering = 1.0 * (pi / 3) * max(-1.0, min(x_offset - 0.03, 1.0))
		driver.SetSteering(steering)

		# Update rover's state
		rover.Update()
		

	print("Sim time:", end_time, "Wall time:", time.time() - t1)
    

if __name__ == '__main__':
	# -----------------
	# Camera parameters
	# -----------------

	# Noise model attached to the sensor
	# noise_model = "CONST_NORMAL"      # Gaussian noise with constant mean and standard deviation
	# noise_model = "PIXEL_DEPENDENT"   # Pixel dependent gaussian noise
	# noise_model = "RESPONSE_FUNCTION" # Noise model based on camera's response and parameters
	noise_model = "NONE"              # No noise model

	# Camera lens model
	# Either PINHOLE or FOV_LENS
	lens_model = sens.PINHOLE

	# Update rate in Hz
	update_rate = 30

	# Image width and height in pixels
	image_width = 1280
	image_height = 720

	# Camera's horizontal field of view
	fov = 80 * pi/180 # [rad]

	# Lag (in seconds) between sensing and when data becomes accessible
	lag = 0 # [sec]

	# Exposure (in seconds) of each image
	exposure_time = 0. # [sec]

	# ---------------------
	# Simulation parameters
	# ---------------------

	# Simulation step size
	step_size = 1e-3

	# Simulation end time in seconds
	end_time = 20.0

	# Save camera images
	save = False

	# Render camera images
	vis = True

	# Output directory
	out_dir = "SENSOR_OUTPUT/"

	## Scene setting parameters
	ground_depth = 0.2 # [m]

	### PART 1 ###
	# rows_inter = 1.5		# [m], interval between rows
	# cols_inter = 2.0		# [m], interval between columns
	# rock_scale = 1.0
	# num_rock_rows = 5		# number of rock rows
	# num_rock_cols = 4		# number of rock columns
	# space_pad = 1.0			# [m], space pad
	# cam_radius = 7			# [m], orbit radius of camera
	# ground_posi_z = - ground_depth / 2
	
	### PART 2 ###
	rows_inter = 2.00		# [m], interval between rows
	cols_inter = 1.25		# [m], interval between columns
	rock_scale = 0.3
	num_rock_rows = 8		# number of rock rows
	num_rock_cols = 2		# number of rock columns3
	space_pad = 3.0			# [m], space pad
	cam_radius = 15			# [m], orbit radius of camera
	ground_posi_z = - ground_depth / 5.0
	
	num_rocks = num_rock_rows * num_rock_cols

	rock_diffuses = [
		[0.0, 0.54, 0.3375],	[0.54, 0.0, 0.405],		[0.54, 0.2025, 0.0],	[1.0, 1.0, 1.0],		
		[0.4725, 0.0, 0.54],	[0.4725, 0.54, 0.0],	[0.0675, 0.54, 0.0],	[0.27, 0.0, 0.54],
		[0.27, 0.54, 0.0],		[0.54, 0.18, 0.18],		[0.81, 0.81, 0.18],		[0.54, 0.405, 0.0],
		[0.0, 0.54, 0.54],		[0.0675, 0.0, 0.54],	[0.54, 0.0, 0.0],		[0.0, 0.3375, 0.54],
	]

	# The path to the Chrono data directory containing various assets (meshes, textures, data files)
	# is automatically set, relative to the default location of this demo.
	# If running from a different directory, you must change the path to the data directory with:
	# chrono.SetChronoDataPath('path/to/data')

	main()
