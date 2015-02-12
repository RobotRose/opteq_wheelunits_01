/***********************************************************************************
* Copyright: Rose B.V. (2013)
*
* Revision History:
*	Author: Okke Hendriks
*	Date  : 2013/12/18
* 		- File created.
*
* Description:
*	This contains the WheelUnit class, maintaining the state of the indiviual 
* 	wheel units.
* 
***********************************************************************************/

#ifndef WHEEL_UNIT_HPP
#define WHEEL_UNIT_HPP

#include <ros/ros.h>

#include <iostream>
#include <stdio.h>
#include <math.h>

#include "rose_conversions/conversions.hpp"
#include "rose_common/common.hpp"
#include "rose_geometry/geometry.hpp"

#include "ros_name/ros_name.hpp"

#define ROS_NAME_WU 			(ROS_NAME + "|WU")

#define WHEEL_RADIUS            (0.0630)                    // [m]

#define STEER_PPR	            (4096.0)                    // [#] absolute encoder ticks per rotation
#define DRIVE_PPR            	(500.0*4.0*51.0)         	// [#] 500.0*4.0 = counts per rotation, 51.0 is gear ratio

#define WHEELUNIT_MAX_ANGLE     (1024.0)                    // [ticks]
#define WHEELUNIT_MIN_ANGLE     (-WHEELUNIT_MAX_ANGLE)      // [ticks]
#define WHEELUNIT_MAX_VEL       (DRIVE_PPR*1.25)            // [pulses/s] Limited by encoder measure rate, higher speeds return zero velocity
#define WHEELUNIT_MIN_VEL       (-WHEELUNIT_MAX_VEL)        // [pulses/s]
#define STEER_VELOCITY_LIMIT    (STEER_PPR/5.0)             // [pulses/s]

#define WHEELUNIT_START_MOVE_ANGLE_ERR_VAL 	((2.0*M_PI)/64.0)	// [rad] Start moving forwards again if the wheelunits are rotated to within this angle of the setpoint	
#define WHEELUNIT_STOP_MOVE_ANGLE_ERR_VAL   ((2.0*M_PI)/12.0) 	// [rad] Stop moving forwards if the wheelunits are rotated out of this angle from the setpoint
#define WHEELUNIT_START_MOVE_ANGLE_ERR_VAL_LOW_LEVEL ((int)(WHEELUNIT_START_MOVE_ANGLE_ERR_VAL*(STEER_PPR/(2.0*M_PI))))		// [abs encoder ticks]
#define WHEELUNIT_STOP_MOVE_ANGLE_ERR_VAL_LOW_LEVEL	 ((int)(WHEELUNIT_STOP_MOVE_ANGLE_ERR_VAL*(STEER_PPR/(2.0*M_PI))))		// [abs encoder ticks]
#define WHEELUNIT_STOP_TURN_SPEED_ERR_VAL   0.10 						// [m/s]

using namespace std; 

/**
 * The WheelUnit class contains the functions and variables for working with the rose 2.0 swifel wheel units.
 */
class WheelUnit
{
  public:
  	WheelUnit();
	WheelUnit(string name, int id);
	~WheelUnit();

	// Angele setters
	bool 	setAngleRad(float radians);
	bool 	setAngleDeg(float degrees);

	// Velocity setters
	bool 	setVelocityRadPerSec(float rad_per_sec);
	bool   	setVelocityMetersPerSec(float meters_per_sec);

	// Angles conversion functions
	float  	toAngleRad(int low_level_angle) const;
	float  	toAngleRad(float low_level_angle) const;
	float  	toAngleDeg(int low_level_angle) const;
	float  	toAngleDeg(float low_level_angle) const;
	int		toLowLevelSteer(float radians) const;
	int 	toLowLevelDrive(float radians) const;

	// Velocity conversion functions
	float	toVelocityRadPerSec(int low_level_velocity) const;
	float	toVelocityRadPerSec(float low_level_velocity) const;
	float	toVelocityMetersPerSec(int low_level_velocity) const;
	float	toVelocityMetersPerSec(float low_level_velocity) const;

	// Angle getters
	int		getSetAngleLowLevel() const;
	float  	getSetAngleRad() const;
	float 	getSetAngleDeg() const;
	int		getMeasuredAngleLowLevel() const;
	float 	getMeasuredAngleRad() const;
	float 	getMeasuredAngleDeg() const;
	int		getAngleErrorLowLevel() const;

	// Velocity getters
	int		getSetVelocityLowLevel() const;
	float	getSetVelocityRadPerSec() const;
	float	getSetVelocityMetersPerSec() const;
	int		getMeasuredVelocityLowLevel() const;
	float	getMeasuredVelocityRadPerSec() const;
	float	getMeasuredVelocityMetersPerSec() const;
	float	getMeasuredDiffMeters() const;
	int		getMeasuredDiffLowLevel() const;
	float	getDT() const;
	int		getVelocityErrorLowLevel() const;

	// IDs
	int    	getDriveMotorID() const;
	int    	getSteerMotorID() const;

	string 		name_;       //! @todo OH: Make private what can be private?
	int 		id_;
	int			set_velocity_;
	int			set_rotation_;
	int  		encoder_measured_velocity_;
	float 		measured_velocity_;
	int   		measured_rotation_;	
	int 		measured_drive_encoder_diff_;
	float 		dT_;
	int 		measured_drive_encoder_;			//! @todo OH: Cleanup
	ros::Time 	measured_drive_encoder_time_;

	int 		velocity_error_;		//! @todo OH: Initialize these variables 
	int 		position_error_;
	int 		drive_PID_out_;
	int 		steer_PID_out_;
	int 		drive_P_out_;
	int 		steer_P_out_;
	int 		drive_I_out_;
	int 		steer_I_out_;
	int 		drive_D_out_;
	int 		steer_D_out_;
	int 		MAE_;
	int 		FE_;
	int 		measured_drive_current_;				// [mA]
	int 		measured_max_drive_current_;			// [mA]
	int 		measured_steer_current_;				// [mA]
	int 		measured_max_steer_current_;			// [mA]

};

#endif // WHEEL_UNIT_HPP