#
# Quadcopter attitude control app
#

MODULE_COMMAND	= q_control
SRCS		= q_control_main.cpp
MODULE_STACKSIZE = 3000
EXTRACFLAGS = -Wno-float-equal -Wframe-larger-than=5000
EXTRACXXFLAGS = -Wframe-larger-than=2400
