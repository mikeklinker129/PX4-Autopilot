px4_add_module(
	MODULE examples__new_control
	MAIN new_control
	STACK_MAX 1300
	SRCS
		new_control.cpp
	DEPENDS
	)