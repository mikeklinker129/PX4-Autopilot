# New Control Mode README




## Control Mode Transition

The first place to define the new control mode is in the commander state message. 
msg/commander_state.msg, add the following below line 19
```uint8 MAIN_STATE_RHOMAN_CONTROL = 16```


#### Add New Control Mode to Commander State Message

#### Commander Modifications



## Vehicle Control Mode Flags

#### Add Control Mode Flag for New Control Mode

Create a new control mode flag. These flags are activated via the Commander state machine based on the current flight mode.  
msg/vehicle_control_mode.msg, add the following below line 20
```bool flag_control_rhoman_ctrl_enabled```

#### Add Navigation State Flag to Vehicle Status Flag

Create a new navigation state for the vehicle status. 
msg/vehicle_status.msg, add the following below line 46
```uint8 NAVIGATION_STATE_RHOMAN_CTRL = 23```


## Controller Output -- Mixer Deactivation

#### Deactivate Mixer with Control Flags


## New Module Creation




#### Add New Module To CMake List
boards/px4/sitl/default.cmake




