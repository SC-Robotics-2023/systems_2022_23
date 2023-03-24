ROSMAKE = ros2 run
ROSBUILD = colcon build
SYMLINK = --symlink-install


DRIVER_PACKAGE = driver
DRIVER_PUB = controller_pub
DRIVER_SUB = drive_receiver_sub


SCIENCE_PACKAGE = science
# Available targets:
# 	funnel_cake_pub			uv_pub			pump_pub		lowering_platform_pub		vacuum_pub
# 	funnel_cake_sub			uv_sub			pump_sub		lowering_platform_sub		vacuum_sub


# AUTONOMY_PACKAGE =


TELECOM_PACKAGE = zed_cv
ZED_PUB = zed_pub
ZED_SUB = zed_sub


# ARM_PACKAGE =





# ==== BUILD ========================================
all: build

build: clean
	$(ROSBUILD) $(SYMLINK)
	@echo "\n\n Remember to do \". install/setup.bash \" to source the workspace \n\n"

clean:
	rm -rf build install log

select:
# 	make select TARGET={package_name}  // For example: make select TARGET=driver
	$(ROSBUILD) $(SYMLINK) --packages-select $(TARGET)

# ===================================================





# ==== DRIVE ========================================
drive_pub:
	$(ROSMAKE) $(DRIVER_PACKAGE) $(DRIVER_PUB)

drive_sub:
	$(ROSMAKE) $(DRIVER_PACKAGE) $(DRIVER_SUB)

# ===================================================





# ==== SCIENCE ======================================	
# Available targets:
# 	funnel_cake
# 	uv
#	pump
#	lowering_platform
# 	vacuum
# Run:
# 	make science_pub TARGET={target}
# 	make science_sub TARGET={target}
# For example:
# 	make science_pub TARGET=funnel_cake 
#	--> Will run "ros2 run science funnel_cake_pub" 
science_pub:
	$(ROSMAKE) $(SCIENCE_PACKAGE) $(TARGET)_pub

science_sub:
	$(ROSMAKE) $(SCIENCE_PACKAGE) $(TARGET)_sub

# ===================================================





# ==== AUTONOMY =====================================
# ===================================================





# ==== TELECOM ======================================
telecom_pub:
	$(ROSMAKE) $(TELECOM_PACKAGE) $(ZED_PUB)

telecom_sub:
	$(ROSMAKE) $(TELECOM_PACKAGE) $(ZED_SUB)

# ===================================================





# ==== ARM ==========================================
# ===================================================




# copy:
# 	scp -r /home/jasper/dev/drive_2022_23/ ubuntu@loki.local:/home/ubuntu/