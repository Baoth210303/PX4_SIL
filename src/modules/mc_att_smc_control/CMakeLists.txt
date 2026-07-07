############################################################################
#
#   Copyright (c) 2026 PX4 Development Team. All rights reserved.
#
############################################################################

px4_add_module(
	MODULE modules__mc_att_smc_control
	MAIN mc_att_smc_control
	COMPILE_FLAGS
		${MAX_CUSTOM_OPT_LEVEL}
	SRCS
		MulticopterAttSmcControl.cpp
		MulticopterAttSmcControl.hpp
	DEPENDS
		mathlib
		px4_work_queue
	)
