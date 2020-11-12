#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/actuator_outputs.h>

extern "C" __EXPORT int rhoman_control_main(int argc, char *argv[]);


class RhomanControl : public ModuleBase<RhomanControl>, public ModuleParams
{
public:
	RhomanControl(int example_param, bool example_flag);

	virtual ~RhomanControl() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static RhomanControl *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

private:

	void execute_parameter_update();

	/**
	 * Check for parameter changes and update them if needed.
	 * @param parameter_update_sub uorb subscription to parameter_update
	 * @param force for a parameter update
	 */
	void parameters_update(bool force = false);


	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SYS_AUTOSTART>) _param_sys_autostart,   /**< example parameter */
		(ParamInt<px4::params::SYS_AUTOCONFIG>) _param_sys_autoconfig,  /**< another parameter */
		(ParamFloat<px4::params::R_CTRL_ROLL_P>) _param_rc_roll_p,
		(ParamFloat<px4::params::R_CTRL_ROLL_P>) _param_rc_pitch_p,
		(ParamFloat<px4::params::R_CTRL_ROLL_P>) _param_rc_yaw_p
		)


	// Subscriptions
	uORB::Subscription	_parameter_update_sub{ORB_ID(parameter_update)};

	uORB::Subscription	test_actuator_output_sub{ORB_ID(actuator_outputs)};

};

