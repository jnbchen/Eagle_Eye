#pragma once
#ifndef CSERIALWRAPPER_H
#define CSERIALWRAPPER_H

#define DBG_OUTPUTf

#include <iostream>
#include <iomanip>
#include <deque>
#include <vector>
#include "CSerial.h"
#include <boost/thread.hpp>
#include "boost/date_time/posix_time/posix_time.hpp"

namespace CHR6dm {

  struct SChannelData
  {
      bool yawEnabled;
      bool pitchEnabled;
      bool rollEnabled;
      bool yawRateEnabled;
      bool pitchRateEnabled;
      bool rollRateEnabled;
      bool mxEnabled;
      bool myEnabled;
      bool mzEnabled;
      bool gxEnabled;
      bool gyEnabled;
      bool gzEnabled;
      bool axEnabled;
      bool ayEnabled;
      bool azEnabled;

      SChannelData( bool yawEnabled, bool pitchEnabled, bool rollEnabled, bool yawRateEnabled, bool pitchRateEnabled, bool rollRateEnabled, bool mxEnabled, bool myEnabled, bool mzEnabled, bool gxEnabled, bool gyEnabled, bool gzEnabled, bool axEnabled, bool ayEnabled, bool azEnabled ) :
          yawEnabled( yawEnabled ), pitchEnabled( pitchEnabled ), rollEnabled( rollEnabled ),
          yawRateEnabled( yawRateEnabled ), pitchRateEnabled( pitchRateEnabled ), rollRateEnabled( rollRateEnabled ),
          mxEnabled( mxEnabled ), myEnabled( myEnabled ), mzEnabled( mzEnabled ),
          gxEnabled( gxEnabled ), gyEnabled( gyEnabled ), gzEnabled( gzEnabled ),
          axEnabled( axEnabled ), ayEnabled( ayEnabled ), azEnabled( azEnabled )
      {;}

      SChannelData( bool fMode = true ) :
          yawEnabled( fMode ), pitchEnabled( fMode ), rollEnabled( fMode ),
          yawRateEnabled( fMode ), pitchRateEnabled( fMode ), rollRateEnabled( fMode ),
          mxEnabled( fMode ), myEnabled( fMode ), mzEnabled( fMode ),
          gxEnabled( fMode ), gyEnabled( fMode ), gzEnabled( fMode ),
          axEnabled( fMode ), ayEnabled( fMode ), azEnabled( fMode )
      {;}
  };

  /** Struct, um die einen Satz der IMU-Sensorinformation zu repraesentieren */
  struct SSensorData
  {
      float yaw;        ///< gefilterter Gierwinkel in Grad
      float pitch;      ///< gefilterter Nickwinkel in Grad
      float roll;       ///< gefilterter Rollwinkel in Grad
      float yawRate;    ///< gefilterte Gierrate in Grad/sec
      float pitchRate;  ///< gefilterte Nickrate in Grad/sec
      float rollRate;   ///< gefilterte Rollrate in Grad/sec
      float mx;         ///< Messwert: magnetische Flussdichte x-Achse in MilliGauss
      float my;         ///< Messwert: magentische Flussdichte y-Achse in MilliGauss
      float mz;         ///< Messwert: magnetische Flussdichte z-Achse in MilliGauss
      float gx;         ///< Messwert: Rollrate in Grad/sec
      float gy;         ///< Messwert: Nickrate in Grad/sec
      float gz;         ///< Messwert: Gierrate in Grad/sec
      float ax;         ///< Messwert: Beschleunigung x-Achse in Tausendstel der Erdbeschleunigung
      float ay;         ///< Messwert: Beschleunigung y-Achse in Tausendstel der Erdbeschleunigung
      float az;         ///< Messwert: Beschleunigung z-Achse in Tausendstel der Erdbeschleunigung
      bool yawEnabled;
      bool pitchEnabled;
      bool rollEnabled;
      bool yawRateEnabled;
      bool pitchRateEnabled;
      bool rollRateEnabled;
      bool mxEnabled;
      bool myEnabled;
      bool mzEnabled;
      bool gxEnabled;
      bool gyEnabled;
      bool gzEnabled;
      bool axEnabled;
      bool ayEnabled;
      bool azEnabled;
      SSensorData () : yaw(0.0f), pitch(0.0f), roll(0.0f), yawRate(0.0f), pitchRate(0.0f), rollRate(0.0f), mx(0.0f), my(0.0f), mz(0.0f), gx(0.0f), gy(0.0f), gz(0.0f), ax(0.0f), ay(0.0f), az(0.0f) {;}
  };

  struct SMatrix3x3
  {
      float entry11;
      float entry12;
      float entry13;
      float entry21;
      float entry22;
      float entry23;
      float entry31;
      float entry32;
      float entry33;

      SMatrix3x3( float entry11, float entry12, float entry13, float entry21, float entry22, float entry23, float entry31, float entry32, float entry33 ) :
          entry11( entry11 ), entry12( entry12 ), entry13( entry13 ),
          entry21( entry21 ), entry22( entry22 ), entry23( entry23 ),
          entry31( entry31 ), entry32( entry32 ), entry33( entry33 )
      {;}

      SMatrix3x3( void ) :
          entry11( 0.0f ), entry12( 0.0f ), entry13( 0.0f ),
          entry21( 0.0f ), entry22( 0.0f ), entry23( 0.0f ),
          entry31( 0.0f ), entry32( 0.0f ), entry33( 0.0f )
      {;}
  };

  struct SMatrix3x1float
  {
      float entry11;
      float entry21;
      float entry31;

      SMatrix3x1float( float entry11, float entry21, float entry31 ) :
          entry11( entry11 ),
          entry21( entry21 ),
          entry31( entry31 )
      {;}

      SMatrix3x1float( void ) :
          entry11( 0.0f ),
          entry21( 0.0f ),
          entry31( 0.0f )
      {;}
  };

  struct SMatrix3x1short
  {
      short entry11;
      short entry21;
      short entry31;

      SMatrix3x1short( short entry11, short entry21, short entry31 ) :
          entry11( entry11 ),
          entry21( entry21 ),
          entry31( entry31 )
      {;}

      SMatrix3x1short( void ) :
          entry11( 0 ),
          entry21( 0 ),
          entry31( 0 )
      {;}
  };

  class CCHR6dm : public CSerial
  {
  public:
      /** ctor */
      CCHR6dm( void );

      /** dtor */
      ~CCHR6dm( void );

      /** Set callback for received data. Has to be setted before open().
       * Method accept functions with signatures like 'void FUNCNAME( const SSensorData )'
       *
       * OBJECTNAME->setCallback( boost::bind( &CLASSNAME::FUNCNAME, this, _1 ) ); */
      void setCallback( const boost::function<void ( const SSensorData )>& fnCallback );

      /** Clears the setted callback */
      void clearCallback( void );

      /** Specifies which channel data should be transmitted over the UART
       * in response to sendGetData(), or periodically in Broadcast Mode.
       * Any combination of sensor channels can be set as active or inactive. */
      void sendSetActiveChannels( const SChannelData );

      /** Enables "Silent Mode." In Silent Mode, the AHRS only reports data
       * when sendGetData() is called. */
      void sendSetSilentMode( void );

      /** Enables "Broadcast Mode." In Broadcast Mode, the AHRS automatically
       * transmits sensor data at regular time intervals. */
      void sendSetBroadcastMode( const int );

      /** Manually sets the rate gyro bias on the X, Y, and Z rate gyros.
       * The bias can be automatically set for all gyro axes by calling
       * sendZeroRateGyros().*/
      void sendSetGyroBias( const short, const short, const short );

      /** Manually sets the accelerometer biases on the X, Y, and Z accelerometers. */
      void sendSetAccelBias( const short, const short, const short );

      /** Manually sets the accelerometer reference vector. The data in the
       * sendSetAccelRefVector() command corresponds to the raw accelerometer
       * measurements expected when the pitch and roll angles are zero. */
      void sendSetAccelRefVector( const short, const short, const short );

      /** Sets the accel reference vector to the most recent data acquired
       * by the accelerometers. */
      void sendAutoSetAccelRef( void );

      /** Starts internal self-calibration of all three rate gyro axes.
       * By default, rate gyros are zeroed on AHRS startup, but gyro startup
       * calibration can be disabled (or re-enabled) by sending a
       * sendSetStartCal() command. */
      void sendZeroRateGyros( void );

      /** Instructs the AHRS to perform a self-test of the accelerometer and
       * gyro sensor channels. The self-test sequence takes approximately
       * 570 milliseconds to complete. During this time, the AHRS should be
       * kept stationary. */
      void sendSelfTest( void );

      /** Enables or disables automatic startup calibration of rate gyros. */
      void sendSetStartCal( const bool );

      /** Sets the process covariance to be used in the prediction step of
       * the EKF. The unit assumes that the process covariance will be a
       * diagonal matrix with equivalent diagonal entries. */
      void sendSetProcessCovariance( const float );

      /** Sets the covariance to be used in the magnetometer update step of
       * the EKF. The unit assumes that the magnetometer covariance will be
       * a diagonal matrix with equivalent diagonal entries. */
      void sendSetMagCovariance( const float );

      /** Sets the covariance to be used in the accelerometer update step of
       * the EKF. The unit assumes that the accelerometer covariance will be
       * a diagonal matrix with equivalent diagonal entries. */
      void sendSetAccelCovariance( const float );

      /** Sets the EKF configuration register. This packet is used to
       * enable/disable accelerometer and magnetometer updates to the angle estimates. */
      void sendSetEkfConfig( const bool, const bool );

      /** Sets the 3x3 calibration matrix used to correct cross-axis
       * misalignment of the rate gyro outputs. */
      void sendSetGyroAlignment( const SMatrix3x3 );

      /** Sets the 3x3 calibration matrix used to correct cross-axis
       * misalignment of the acclerometer outputs. */
      void sendSetAccelAlignment( const SMatrix3x3 );

      /** Manually sets the magnetometer reference vector. The data in the
       * sendSetMagRefVector() command corresponds to the raw magnetometer
       * measurements expected when the pitch, roll, and yaw angles are zero. */
      void sendSetMagRefVector( const short, const short, const short );

      /** Sets the magnetic field reference vector to the most recent
       * magnetic sensor measurement. */
      void sendAutoSetMagRef( void );

      /** Sets the 3x3 calibration matrix used for soft and hard iron
       * calibration, axis misalignment calibration, and scale calibration
       * of the magnetometer. */
      void sendSetMagCal( const SMatrix3x3 );

      /** Sets the magnetic field bias term to compensate for hard-iron
       * distortions of the magnetic field. */
      void sendSetMagBias( const short, const short, const short );

      /// Sets the scale factors used to compute rates from raw gyro data on all axes.
      void sendSetGyroScale( const float, const float, const float );

      /// Sets each term in the state covariance matrix to zero and re-initializes the EKF. This command can be used for recovery if the state is corrupted by passing too close to the singularity at pitch = 90 degrees.
      void sendEkfReset( void );

      /// Resets all AHRS configuration to the factory defaults. This includes calibration parameters, biases, communication settings, etc. The factory defaults are written to RAM. To make them persistent, sendWriteToFlash() must be sent following the sendResetToFactory() command.
      void sendResetToFactory( void );

      /// Writes AHRS configuration to on-board flash so that the configuration persists when the power is cycled.
      void sendWriteToFlash( void );

      /// In Silent Mode, the AHRS waits to receive a sendGetData() command before transmitting sensor data. The most recent data from all active sensor channels is transmitted in response. In Broadcast Mode, a sendGetData() packet is ignored.
      void sendGetData( void );

      /// Reports which channels are "active." Active channels are sensor channels that are measured and transmitted in response to a GET_DATA packet, or periodically in Broadcast Mode.
      void sendGetActiveChannels( void );

      /// Returns the broadcast frequency, if 0 is returned the AHRS is in silent mode.
      unsigned int sendGetBroadcastMode( const unsigned int nTimeoutMS = 2000 );

      /// Return the bias values for all three accel axes.
      SMatrix3x1short sendGetAccelBias( const unsigned int nTimeoutMS = 2000 );

      /// Return the accelerometer reference vector.
      SMatrix3x1short sendGetAccelRefVector( const unsigned int nTimeoutMS = 2000 );

      /// Returns the bias values for all three rate gyros.
      SMatrix3x1short sendGetGyroBias( const unsigned int nTimeoutMS = 2000 );

      /// Returns the scale factors used to convert raw gyro measurements to angular rates.
      SMatrix3x1float sendGetGyroScale( const unsigned int nTimeoutMS = 2000 );

      /// Returns the gyro bias startup calibriation status.
      bool sendGetStartCal( const unsigned int nTimeoutMS = 2000 );

      /// Returns the value of the EKF configuration register.
      void sendGetEkfConfig( const unsigned int nTimeoutMS = 2000 );

      /// Returns the covariance to be used in the accelerometer update step of the EKF.
      float sendGetAccelCovariance( const unsigned int nTimeoutMS = 2000 );

      /// Returns the covariance to be used in the magnetometer update step of the EKF.
      float sendGetMagCovariance( const unsigned int nTimeoutMS = 2000 );

      /// Returns the covariance to be used in the prediction step of the EKF.
      float sendGetProcessCovariance( const unsigned int nTimeoutMS = 2000 );

      /// Returns the 3x3 covariance matrix of the current state estimates in the EKF.
      SMatrix3x3 sendGetStateCovariance( const unsigned int nTimeoutMS = 2000 );

      /// Returns the 3x3 matrix used to correct gyro cross-axis misalignment.
      SMatrix3x3 sendGetGyroAlignment( const unsigned int nTimeoutMS = 2000 );

      /// Returns the 3x3 matrix used to correct accelerometer cross-axis misalignment.
      SMatrix3x3 sendGetAccelAlignment( const unsigned int nTimeoutMS = 2000 );

      /// Returns the magnetic field reference vector.
      SMatrix3x1short sendGetMagRefVector( const unsigned int nTimeoutMS = 2000 );

      /// Returns the 3x3 matrix used to correct magnetometer soft iron distortions, axis misalignment, and scale factors.
      SMatrix3x3 sendGetMagCal( const unsigned int nTimeoutMS = 2000 );

      /// Returns the magnetometer biases used to correct hard iron distortions.
      SMatrix3x1short sendGetMagBias( const unsigned int nTimeoutMS = 2000 );

  private:
      float charsToFloat( const unsigned char* ) const;
      short charsToShort( const unsigned char* ) const;
      bool charToBool( const unsigned char, const unsigned int ) const;
      unsigned short calculateChecksum( const unsigned char*, const unsigned int ) const;
      bool sendRawMessage( const unsigned char, const int, const unsigned char* );
      void received( const unsigned char*, const unsigned int );
      void handleData( void );
      void resetScale( void );

  private:
      std::deque<unsigned char> m_rgData;
      boost::shared_ptr<boost::thread> m_boostReadThread;
      boost::mutex m_boostReadBufferMutex;
      boost::condition_variable m_boostWaitCondition;
      boost::function<void ( const SSensorData )> m_boostDataCallback;

      float m_rScaleYaw;
      float m_rScalePitch;
      float m_rScaleRoll;
      float m_rScaleYawRate;
      float m_rScalePitchRate;
      float m_rScaleRollRate;
      float m_rScaleMagX;
      float m_rScaleMagY;
      float m_rScaleMagZ;
      float m_rScaleGyroX;
      float m_rScaleGyroY;
      float m_rScaleGyroZ;
      float m_rScaleAccelX;
      float m_rScaleAccelY;
      float m_rScaleAccelZ;

      boost::condition_variable m_boostWaitForResponse;
      boost::mutex m_boostResponseMutex;
      bool m_bWaitingForResponse;
      std::vector<float> m_rgExchangeFloat;
      std::vector<short> m_rgExchangeShort;
      std::vector<bool> m_rgExchangeBool;
      std::vector<int> m_rgExchangeInt;

  };

} // end namespace

#endif // CSERIALWRAPPER_H
