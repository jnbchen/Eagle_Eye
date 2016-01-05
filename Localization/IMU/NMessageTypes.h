#pragma once
#ifndef NMESSAGETYPES_H
#define NMESSAGETYPES_H

namespace CHR6dm_SEND
{
    const static unsigned char SET_ACTIVE_CHANNELS      = 0x80;
    const static unsigned char SET_SILENT_MODE          = 0x81;
    const static unsigned char SET_BROADCAST_MODE       = 0x82;
    const static unsigned char SET_GYRO_BIAS            = 0x83;
    const static unsigned char SET_ACCEL_BIAS           = 0x84;
    const static unsigned char SET_ACCEL_REF_VECTOR     = 0x85;
    const static unsigned char AUTO_SET_ACCEL_REF       = 0x86;
    const static unsigned char ZERO_RATE_GYROS          = 0x87;
    const static unsigned char SELF_TEST                = 0x88;
    const static unsigned char SET_START_CAL            = 0x89;
    const static unsigned char SET_PROCESS_COVARIANCE   = 0x8A;
    const static unsigned char SET_MAG_COVARIANCE       = 0x8B;
    const static unsigned char SET_ACCEL_COVARIANCE     = 0x8C;
    const static unsigned char SET_EKF_CONFIG           = 0x8D;
    const static unsigned char SET_GYRO_ALIGNMENT       = 0x8E;
    const static unsigned char SET_ACCEL_ALIGNMENT      = 0x8F;
    const static unsigned char SET_MAG_REF_VECTOR       = 0x90;
    const static unsigned char AUTO_SET_MAG_REF         = 0x91;
    const static unsigned char SET_MAG_CAL              = 0x92;
    const static unsigned char SET_MAG_BIAS             = 0x93;
    const static unsigned char SET_GYRO_SCALE           = 0x94;
    const static unsigned char EKF_RESET                = 0x95;
    const static unsigned char RESET_TO_FACTORY         = 0x96;
    const static unsigned char WRITE_TO_FLASH           = 0xA0;
    const static unsigned char GET_DATA                 = 0x01;
    const static unsigned char GET_ACTIVE_CHANNELS      = 0x02;
    const static unsigned char GET_BROADCAST_MODE       = 0x03;
    const static unsigned char GET_ACCEL_BIAS           = 0x04;
    const static unsigned char GET_ACCEL_REF_VECTOR     = 0x05;
    const static unsigned char GET_GYRO_BIAS            = 0x06;
    const static unsigned char GET_GYRO_SCALE           = 0x07;
    const static unsigned char GET_START_CAL            = 0x08;
    const static unsigned char GET_EKF_CONFIG           = 0x09;
    const static unsigned char GET_ACCEL_COVARIANCE     = 0x0A;
    const static unsigned char GET_MAG_COVARIANCE       = 0x0B;
    const static unsigned char GET_PROCESS_COVARIANCE   = 0x0C;
    const static unsigned char GET_STATE_COVARIANCE     = 0x0D;
    const static unsigned char GET_GYRO_ALIGNMENT       = 0x0E;
    const static unsigned char GET_ACCEL_ALIGNMENT      = 0x0F;
    const static unsigned char GET_MAG_REF_VECTOR       = 0x10;
    const static unsigned char GET_MAG_CAL              = 0x11;
    const static unsigned char GET_MAG_BIAS             = 0x12;
}

namespace CHR6dm_GET
{
    const static unsigned char COMMAND_COMPLETE          = 0xB0;
    const static unsigned char COMMAND_FAILED            = 0xB1;
    const static unsigned char BAD_CHECKSUM              = 0xB2;
    const static unsigned char BAD_DATA_LENGTH           = 0xB3;
    const static unsigned char UNRECOGNIZED_PACKET       = 0xB4;
    const static unsigned char BUFFER_OVERFLOW           = 0xB5;
    const static unsigned char STATUS_REPORT             = 0xB6;
    const static unsigned char SENSOR_DATA               = 0xB7;
    const static unsigned char GYRO_BIAS_REPORT          = 0xB8;
    const static unsigned char GYRO_SCALE_REPORT         = 0xB9;
    const static unsigned char START_CAL_REPORT          = 0xBA;
    const static unsigned char ACCEL_BIAS_REPORT         = 0xBB;
    const static unsigned char ACCEL_REF_VECTOR_REPORT   = 0xBC;
    const static unsigned char ACTIVE_CHANNEL_REPORT     = 0xBD;
    const static unsigned char ACCEL_COVARIANCE_REPORT   = 0xBE;
    const static unsigned char MAG_COVARIANCE_REPORT     = 0xBF;
    const static unsigned char PROCESS_COVARIANCE_REPORT = 0xC0;
    const static unsigned char STATE_COVARIANCE_REPORT   = 0xC1;
    const static unsigned char EKF_CONFIG_REPORT         = 0xC2;
    const static unsigned char GYRO_ALIGNMENT_REPORT     = 0xC3;
    const static unsigned char ACCEL_ALIGNMENT_REPORT    = 0xC4;
    const static unsigned char MAG_REF_VECTOR_REPORT     = 0xC5;
    const static unsigned char MAG_CAL_REPORT            = 0xC6;
    const static unsigned char MAG_BIAS_REPORT           = 0xC7;
    const static unsigned char BROADCAST_MODE_REPORT     = 0xC8;
}

#endif
