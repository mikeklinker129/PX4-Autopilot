px4_add_module(
	MODULE modules__rhoman_control
	MAIN rhoman_control
	STACK_MAX 1300
	SRCS
		rhoman_control.cpp
	DEPENDS
		modules__uORB
	)

