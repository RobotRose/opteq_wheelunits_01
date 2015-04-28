/***********************************************************************************
* Copyright: Rose B.V. (2013)
*
* Revision History:
*	Author: Okke Hendriks
*	Date  : 2013/12/18
* 		- File created.
*
* Description:
*	WheelUnit class
* 
***********************************************************************************/

#include "opteq_wheelunits_01/wheel_unit.hpp"

//! @todo OH: Move into namespace

WheelUnit::WheelUnit()
	: name_("NO NAME")
	, id_(-1)
	, set_rotation_(0.0)
	, set_velocity_(0.0)
	, measured_rotation_(0)
    , measured_velocity_(0.0)
    , measured_drive_encoder_(0)
    , measured_drive_encoder_diff_(0)
    , dT_(0)
    , velocity_error_(0)
	, position_error_(0)
	, drive_PID_out_(0)
	, steer_PID_out_(0)
	, drive_P_out_(0)
	, steer_P_out_(0)
	, drive_I_out_(0)
	, steer_I_out_(0)
	, drive_D_out_(0)
	, steer_D_out_(0)
	, MAE_(0)
	, FE_(0)
	, measured_drive_current_(0)
	, measured_max_drive_current_(0)
	, measured_steer_current_(0)
	, measured_max_steer_current_(0)
{}

WheelUnit::WheelUnit(string name, int id)
	: name_(name)
	, id_(id)
	, set_rotation_(0.0)
	, set_velocity_(0.0)
	, measured_rotation_(0)
    , measured_velocity_(0.0)
    , measured_drive_encoder_(0)
    , measured_drive_encoder_diff_(0)
    , dT_(0)
    , velocity_error_(0)
	, position_error_(0)
	, drive_PID_out_(0)
	, steer_PID_out_(0)
	, drive_P_out_(0)
	, steer_P_out_(0)
	, drive_I_out_(0)
	, steer_I_out_(0)
	, drive_D_out_(0)
	, steer_D_out_(0)
	, MAE_(0)
	, FE_(0)
	, measured_drive_current_(0)
	, measured_max_drive_current_(0)
	, measured_steer_current_(0)
	, measured_max_steer_current_(0)
{}

WheelUnit::~WheelUnit()
{}

bool WheelUnit::setAngleRad(float radians)
{
	rose_geometry::wrapToHalfPi(&radians);
	int new_rotation = toLowLevelSteer(radians); 
	if(rose_conversions::limit(WHEELUNIT_MIN_ANGLE, WHEELUNIT_MAX_ANGLE, &new_rotation))
	{
		ROS_WARN_NAMED(ROS_NAME_WU, "Rotation limit reached, while trying to set to: %.4frad", radians);
		return false;
	}
	else
		ROS_DEBUG_NAMED(ROS_NAME_WU, "Setting Rotation: %.2frad, %d enc", radians, new_rotation);
	
	set_rotation_ = new_rotation;	
	return true;	
}

bool WheelUnit::setAngleDeg(float degrees)
{
	return setAngleRad(degrees*(M_PI/180.0));
}

bool WheelUnit::setVelocityRadPerSec(float rad_per_sec)
{
	int new_velocity = toLowLevelDrive(rad_per_sec);
	if(rose_conversions::limit(WHEELUNIT_MIN_VEL, WHEELUNIT_MAX_VEL, &new_velocity))
	{
		ROS_WARN_NAMED(ROS_NAME_WU, "Rad/s limit reached, while trying to set: %.4frad/s", rad_per_sec);
		return false;
	}
	else
		ROS_DEBUG_NAMED(ROS_NAME_WU, "Setting velocity: %.2frad/s, %d pulses/s", rad_per_sec, new_velocity);
	
	set_velocity_ = new_velocity;
	return true;
}

bool WheelUnit::setVelocityMetersPerSec(float meters_per_sec)
{
	return setVelocityRadPerSec(meters_per_sec/(WHEEL_RADIUS)); 
}
  
float WheelUnit::toAngleRad(int low_level_angle) const
{
    return toAngleRad((float)low_level_angle);
}

float WheelUnit::toAngleRad(float low_level_angle) const
{
    return (low_level_angle/STEER_PPR)*M_PI*2.0;
}

int WheelUnit::toLowLevelSteer(float radians) const
{
    return (int)((radians*STEER_PPR)/(2.0*M_PI));
}

int WheelUnit::toLowLevelDrive(float radians) const
{
    return (int)((radians*DRIVE_PPR)/(2.0*M_PI));						
}

float WheelUnit::toAngleDeg(int low_level_angle) const
{
    return toAngleRad(low_level_angle)*(180.0/M_PI);
}

float WheelUnit::toAngleDeg(float low_level_angle) const
{
    return toAngleRad(low_level_angle)*(180.0/M_PI);
}

float WheelUnit::toVelocityRadPerSec(int low_level_velocity) const
{
	return toVelocityRadPerSec((float)low_level_velocity);
}

float WheelUnit::toVelocityRadPerSec(float low_level_velocity) const
{
	return ((low_level_velocity)/DRIVE_PPR)*M_PI*2.0;
}

float WheelUnit::toVelocityMetersPerSec(int low_level_velocity) const
{
	return toVelocityRadPerSec(low_level_velocity)*WHEEL_RADIUS;
}

float WheelUnit::toVelocityMetersPerSec(float low_level_velocity) const
{
	return toVelocityRadPerSec(low_level_velocity)*WHEEL_RADIUS;
}

int WheelUnit::getSetAngleLowLevel() const
{
    return set_rotation_;
}

float WheelUnit::getSetAngleRad() const
{
	return toAngleRad(set_rotation_);
}

float WheelUnit::getSetAngleDeg() const
{
	return toAngleDeg(set_rotation_);
}

int WheelUnit::getMeasuredAngleLowLevel() const
{
	return measured_rotation_;
}

float WheelUnit::getMeasuredAngleRad() const
{
	return toAngleRad(measured_rotation_);
}

float WheelUnit::getMeasuredAngleDeg() const
{
	return toAngleDeg(measured_rotation_);
}

int WheelUnit::getAngleErrorLowLevel() const
{
	return getSetAngleLowLevel() - getMeasuredAngleLowLevel();
}

// Returns velocity in pulses/s
int WheelUnit::getSetVelocityLowLevel() const
{
	return set_velocity_;
}

float WheelUnit::getSetVelocityRadPerSec() const
{
	return toVelocityRadPerSec(set_velocity_);
}

float WheelUnit::getSetVelocityMetersPerSec() const
{
	return toVelocityMetersPerSec(set_velocity_);
}

int WheelUnit::getMeasuredVelocityLowLevel() const
{
	return measured_velocity_;
}

float WheelUnit::getMeasuredVelocityRadPerSec() const
{
	return toVelocityRadPerSec(measured_velocity_);
}

float WheelUnit::getMeasuredVelocityMetersPerSec() const
{
	return toVelocityMetersPerSec(measured_velocity_); 
}

float WheelUnit::getMeasuredDiffMeters() const
{
	return ((float)getMeasuredDiffLowLevel()/DRIVE_PPR)*M_PI*2.0*WHEEL_RADIUS;
}

int WheelUnit::getMeasuredDiffLowLevel() const
{
	return measured_drive_encoder_diff_;
}

float WheelUnit::getDT() const
{
	return dT_;
}

int WheelUnit::getVelocityErrorLowLevel() const
{
	return getSetVelocityLowLevel() - getMeasuredVelocityLowLevel();
}

int WheelUnit::getDriveMotorID() const
{
	return id_;
}

int WheelUnit::getSteerMotorID() const
{
	return id_ + 1;
}

float WheelUnit::getMinVel() const
{
	return toVelocityMetersPerSec((int)WHEELUNIT_MIN_VEL);
}

float WheelUnit::getMaxVel() const
{
	return toVelocityMetersPerSec((int)WHEELUNIT_MAX_VEL);
}

float WheelUnit::getMinAngle() const
{
	return toAngleRad((int)WHEELUNIT_MIN_ANGLE);
}

float WheelUnit::getMaxAngle() const
{
	return toAngleRad((int)WHEELUNIT_MAX_ANGLE) ;
}
