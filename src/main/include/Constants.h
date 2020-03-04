#pragma once

namespace ConfigFiles {
    const std::string ConfigFile     = "config.toml";
    const std::string AutoConfigFile = "auto.toml";
}

namespace Pins {
    constexpr int DIO_0 = 0;
    constexpr int DIO_1 = 1;
    constexpr int DIO_2 = 2;
    constexpr int DIO_3 = 3;
    constexpr int DIO_4 = 4;
    constexpr int DIO_5 = 5;
    constexpr int DIO_6 = 6;
    constexpr int DIO_7 = 7;
    constexpr int DIO_8 = 8;
    constexpr int DIO_9 = 9;

    constexpr int AIO_0 = 0;
    constexpr int AIO_1 = 1;
    constexpr int AIO_2 = 2;
    constexpr int AIO_3 = 3;

    constexpr int RELAY_0 = 0;
    constexpr int RELAY_1 = 1;
    constexpr int RELAY_2 = 2;
    constexpr int RELAY_3 = 3;

    constexpr int PWM_0 = 0;
    constexpr int PWM_1 = 1;
    constexpr int PWM_2 = 2;
    constexpr int PWM_3 = 3;
    constexpr int PWM_4 = 4;
    constexpr int PWM_5 = 5;
    constexpr int PWM_6 = 6;
    constexpr int PWM_7 = 7;
    constexpr int PWM_8 = 8;
    constexpr int PWM_9 = 9;

    constexpr int CAN_0 = 0;
    constexpr int CAN_1 = 1;
    constexpr int CAN_2 = 2;
    constexpr int CAN_3 = 3;
    constexpr int CAN_4 = 4;
    constexpr int CAN_5 = 5;
    constexpr int CAN_6 = 6;
    constexpr int CAN_7 = 7;
    constexpr int CAN_8 = 8;
    constexpr int CAN_9 = 9;
    constexpr int CAN_10 = 10;
    constexpr int CAN_11 = 11;
    constexpr int CAN_12 = 12;
    constexpr int CAN_13 = 13;
    constexpr int CAN_14 = 14;
    constexpr int CAN_15 = 15;
    constexpr int CAN_16 = 16;
    constexpr int CAN_17 = 17;
    constexpr int CAN_18 = 18;
    constexpr int CAN_19 = 19;
    constexpr int CAN_20 = 20;

    constexpr int AIR_0 = 0;
    constexpr int AIR_1 = 1;
    constexpr int AIR_2 = 2;
    constexpr int AIR_3 = 3;
    constexpr int AIR_4 = 4;
    constexpr int AIR_5 = 5;
    constexpr int AIR_6 = 6;
    constexpr int AIR_7 = 7;
}

// CAN IDs for drivetrain Spark MAXes
namespace DriveMotorPins {
    constexpr int Left1  = Pins::CAN_1;
    constexpr int Left2  = Pins::CAN_2;
    constexpr int Left3  = Pins::CAN_3;
    constexpr int Right1 = Pins::CAN_4;
    constexpr int Right2 = Pins::CAN_5;
    constexpr int Right3 = Pins::CAN_6;
}

// CAN ID for intake Talon SRX
constexpr int kIntakeMotor    = Pins::CAN_7;
constexpr int kConveyorMotor  = Pins::CAN_8;

// PCM pin for intake solenoid
constexpr int kIntakeExtendSolenoidPin  = Pins::AIR_2;
constexpr int kIntakeRetractSolenoidPin = Pins::AIR_6;

constexpr int kBeamPowerCellFeeder = Pins::DIO_2;

constexpr int kShooterMotor1 = Pins::CAN_9;
constexpr int kShooterMotor2 = Pins::CAN_10;

constexpr int kTurretFeederMotor = Pins::CAN_11;
constexpr int kTurretMotor       = Pins::CAN_12;

// PowerCellCounter
constexpr int kBeamPowerCellIn  = Pins::DIO_0;
constexpr int kBeamPowerCellOut = Pins::DIO_1;

// Control Panel
constexpr int kControlPanelRollerMotor = Pins::CAN_14;

constexpr int kControlPanelExtendSolenoidPin = Pins::AIR_0;     // Fixme:  Verify and assign.
constexpr int kControlPanelRetractSolenoidPin = Pins::AIR_1;    // Fixme:  Verify and assign.

// PCM 2
constexpr int kPCM2 = Pins::CAN_15;