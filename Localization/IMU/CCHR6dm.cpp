#include "CCHR6dm.h"
#include "NMessageTypes.h"

using namespace CHR6dm;

CCHR6dm::CCHR6dm( void ) :
    CSerial()
{
    // init the scaling factors
    resetScale();

    // set read callback
    setReadCallback( boost::bind( &CCHR6dm::received, this, _1, _2 ) );

    // start new thread, which handles received data
    m_boostReadThread = boost::shared_ptr<boost::thread>( new boost::thread( boost::bind( &CCHR6dm::handleData, this ) ) );

    m_rgExchangeFloat.resize(9);
    m_rgExchangeShort.resize(3);
    m_rgExchangeBool.resize(2);
    m_rgExchangeInt.resize(1);
}

CCHR6dm::~CCHR6dm( void )
{
    // clear read callback
    clearReadCallback();

    // tell reading thread to stop and wait for it
    m_boostReadThread->interrupt();
    m_boostReadThread->join();
}

void CCHR6dm::setCallback( const boost::function<void ( const SSensorData )>& fnCallback )
{
    m_boostDataCallback = fnCallback;
}

void CCHR6dm::clearCallback( void )
{
    m_boostDataCallback.clear();
}

void CCHR6dm::resetScale( void )
{
    // reset scaling factors, standard factors, see documentation

    // [ °/LSB ]
    m_rScaleYaw = 0.0109863;
    m_rScalePitch = 0.0109863;
    m_rScaleRoll = 0.0109863;

    // [ °/s/LSB ]
    m_rScaleYawRate = 0.0137329;
    m_rScalePitchRate = 0.0137329;
    m_rScaleRollRate = 0.0137329;

    // [ mGauss/LSB ]
    m_rScaleMagX = 0.061035;
    m_rScaleMagY = 0.061035;
    m_rScaleMagZ = 0.061035;

    // [ °/s/LSB ]
    m_rScaleGyroX = 0.01812;
    m_rScaleGyroY = 0.01812;
    m_rScaleGyroZ = 0.01812;

    // [ mg/LSB ]
    m_rScaleAccelX = 0.15387;
    m_rScaleAccelY = 0.15387;
    m_rScaleAccelZ = 0.15387;
}

bool CCHR6dm::charToBool( const unsigned char ui8Byte, const unsigned int nPosition ) const
{
    return ( ( ui8Byte& ( 1 << nPosition ) ) != 0 );
}

float CCHR6dm::charsToFloat( const unsigned char* pui8Data ) const
{
    float tmp;

    ((unsigned char*) &(tmp))[3] = pui8Data[ 0 ];
    ((unsigned char*) &(tmp))[2] = pui8Data[ 1 ];
    ((unsigned char*) &(tmp))[1] = pui8Data[ 2 ];
    ((unsigned char*) &(tmp))[0] = pui8Data[ 3 ];

    return tmp;
}

short CCHR6dm::charsToShort( const unsigned char* pui8Data ) const
{
    short tmp;

    ((unsigned char*) &(tmp))[1] = pui8Data[ 0 ];
    ((unsigned char*) &(tmp))[0] = pui8Data[ 1 ];

    return tmp;
}

unsigned short CCHR6dm::calculateChecksum( const unsigned char* pui8Data, const unsigned int uiLength ) const
{
    unsigned short checksum( 0 );

    for ( unsigned int i = 0; i < uiLength; i++ )
        checksum += pui8Data[ i ];

    return checksum;
}

bool CCHR6dm::sendRawMessage( const unsigned char ui8Type, const int nSize, const unsigned char* pui8Data )
{
    // only send message if device is open
    if ( !isOpen() )
        return false;

    // alloc message
    unsigned char packet[ nSize+7 ];

    // write message header: 's'+'n'+'p'+'messageType'+'messageLength'+...
    packet[ 0 ] = 's';
    packet[ 1 ] = 'n';
    packet[ 2 ] = 'p';
    packet[ 3 ] = ui8Type;
    packet[ 4 ] = nSize;

    // write message data
    for ( int i = 0; i < nSize; i++ )
        packet[ 5+i ] = pui8Data[ i ];

    // calculate checksum of message
    const int nChecksum( calculateChecksum( packet, nSize+5 ) );

    // write message checksum
    packet[ nSize+5 ] = (unsigned char)( nChecksum >> 8 );
    packet[ nSize+6 ] = (unsigned char)( nChecksum );

    // write whole message
    write( packet, nSize+7 );

    return true;
}

void CCHR6dm::received( const unsigned char* pui8Data, const unsigned int uiLength )
{
    // lock dataarray for access
    boost::unique_lock<boost::mutex> boostUniqueLock ( m_boostReadBufferMutex );

    // copy data
    m_rgData.insert( m_rgData.end(), pui8Data, pui8Data+uiLength );

    // thread synchronization
    // 1) release lock (not really necessary, but worst case scenario)
    // 2) notify waiting thread to continue
    boostUniqueLock.unlock();
    m_boostWaitCondition.notify_all();
}

void CCHR6dm::handleData( void )
{
    unsigned char ui8PacketType( 0 );
    unsigned char ui8PacketLength( 0 );
    bool bSyncToHeader( false );

    try{
        while ( true )
        {
            // lock dataarray for access
            boost::unique_lock<boost::mutex> boostUniqueLock ( m_boostReadBufferMutex );

            // thread synchronization, part 1
            // wait until buffer is filled with maximal length (32byte+7byte) of one message plus one safty byte
            while ( m_rgData.size() < 40 )
            {
                m_boostWaitCondition.wait( boostUniqueLock );
            }

            // search for header-signature in data
            while ( !bSyncToHeader )
            {
                if ( m_rgData.at( 0 ) == 's' && m_rgData.at( 1 ) == 'n' && m_rgData.at( 2 ) == 'p' )
                {
                    // skip 's'+'n'+'p'
                    m_rgData.pop_front();
                    m_rgData.pop_front();
                    m_rgData.pop_front();

                    // read and skip packet type
                    ui8PacketType = m_rgData.front();
                    m_rgData.pop_front();

                    // read and skip packet length
                    ui8PacketLength = m_rgData.front();
                    m_rgData.pop_front();

                    bSyncToHeader = true;
                }
                else
                {
                    m_rgData.pop_front();
                }
            }

            // thread synchronization, part 2
            // wait until buffer is filled with excepted message length plus 2 bytes for checksum plus one safty byte
            while ( static_cast<int>(m_rgData.size()) < (ui8PacketLength+2+1) )
            {
                m_boostWaitCondition.wait( boostUniqueLock );
            }

            // header found?
            // whole message received?
            // now process data!

            // copy data...
            unsigned char rgData[ (ui8PacketLength+2) ];
            for ( int i = 0; i < (ui8PacketLength+2); i++ )
            {
                rgData[ i ] = m_rgData.front();
                m_rgData.pop_front();
            }

            // ... so that we can release the lock
            boostUniqueLock.unlock();

            // handle packet
            switch ( ui8PacketType )
            {
            case CHR6dm_GET::COMMAND_COMPLETE:
            {
#ifdef DBG_OUTPUT
                std::cout << "COMMAND_COMPLETE: " << (int)ui8PacketLength << std::endl;
                std::cout << "COMMAND_COMPLETE: " << (int)rgData[0] << std::endl;
#endif
            }
                break;

            case CHR6dm_GET::COMMAND_FAILED:
            {
#ifdef DBG_OUTPUT
                std::cout << "COMMAND_FAILED: " << (int)ui8PacketLength << std::endl;
                std::cout << "COMMAND_FAILED: " << (int)rgData[0] << std::endl;
#endif
            }
                break;

            case CHR6dm_GET::BAD_CHECKSUM:
            {
#ifdef DBG_OUTPUT
                std::cout << "BAD_CHECKSUM: " << (int)ui8PacketLength << std::endl;
#endif
            }
                break;

            case CHR6dm_GET::BAD_DATA_LENGTH:
            {
#ifdef DBG_OUTPUT
                std::cout << "BAD_DATA_LENGTH: " << (int)ui8PacketLength << std::endl;
#endif
            }
                break;

            case CHR6dm_GET::UNRECOGNIZED_PACKET:
            {
#ifdef DBG_OUTPUT
                std::cout << "UNRECOGNIZED_PACKET: " << (int)ui8PacketLength << std::endl;
#endif
            }
                break;

            case CHR6dm_GET::BUFFER_OVERFLOW:
            {
#ifdef DBG_OUTPUT
                std::cout << "BUFFER_OVERFLOW: " << (int)ui8PacketLength << std::endl;
#endif
            }
                break;

            case CHR6dm_GET::STATUS_REPORT:
            {
#ifdef DBG_OUTPUT
                std::cout << "STATUS_REPORT: " << (int)ui8PacketLength << std::endl;
#endif

                bool statusReport[6];
                statusReport[0] = charToBool( rgData[0], 5 );
                statusReport[1] = charToBool( rgData[0], 4 );
                statusReport[2] = charToBool( rgData[0], 3 );
                statusReport[3] = charToBool( rgData[0], 2 );
                statusReport[4] = charToBool( rgData[0], 1 );
                statusReport[5] = charToBool( rgData[0], 0 );

#ifdef DBG_OUTPUT
                std::cout << "gyro_z_error:" << statusReport[0] << " gyro_y_error:" << statusReport[1] << " gyro_x_error:" << statusReport[2] << std::endl;
                std::cout << "accel_z_error:" << statusReport[3] << " accel_y_error:" << statusReport[4] << " accel_x_error:" << statusReport[5] << std::endl;
#endif
            }
                break;

            case CHR6dm_GET::SENSOR_DATA:
            {
                //std::cout << "SENSOR_DATA: " << (int)ui8PacketLength << std::endl;

                SSensorData mySensorData;

                mySensorData.yawEnabled = charToBool( rgData[0], 7 );
                mySensorData.pitchEnabled = charToBool( rgData[0], 6 );
                mySensorData.rollEnabled = charToBool( rgData[0], 5 );

                mySensorData.yawRateEnabled = charToBool( rgData[0], 4 );
                mySensorData.pitchRateEnabled = charToBool( rgData[0], 3 );
                mySensorData.rollRateEnabled = charToBool( rgData[0], 2 );

                mySensorData.mxEnabled = charToBool( rgData[0], 1 );
                mySensorData.myEnabled = charToBool( rgData[0], 0 );
                mySensorData.mzEnabled = charToBool( rgData[1], 7 );

                mySensorData.gxEnabled = charToBool( rgData[1], 6 );
                mySensorData.gyEnabled = charToBool( rgData[1], 5 );
                mySensorData.gzEnabled = charToBool( rgData[1], 4 );

                mySensorData.axEnabled = charToBool( rgData[1], 3 );
                mySensorData.ayEnabled = charToBool( rgData[1], 2 );
                mySensorData.azEnabled = charToBool( rgData[1], 1 );

                unsigned int step( 2 );

                if ( mySensorData.yawEnabled )
                {
                    const short yaw = charsToShort( &(rgData[step]) );
                    mySensorData.yaw = yaw * m_rScaleYaw;
                    step += 2;
                }

                if ( mySensorData.pitchEnabled )
                {
                    const short pitch = charsToShort( &(rgData[step]) );
                    mySensorData.pitch = pitch * m_rScalePitch;
                    step += 2;
                }

                if ( mySensorData.rollEnabled )
                {
                    const short roll = charsToShort( &(rgData[step]) );
                    mySensorData.roll = roll * m_rScaleRoll;
                    step += 2;
                }

                if ( mySensorData.yawRateEnabled )
                {
                    const short yawRate = charsToShort( &(rgData[step]) );
                    mySensorData.yawRate = yawRate * m_rScaleYawRate;
                    step += 2;
                }

                if ( mySensorData.pitchRateEnabled )
                {
                    const short pitchRate = charsToShort( &(rgData[step]) );
                    mySensorData.pitchRate = pitchRate * m_rScalePitch;
                    step += 2;
                }

                if ( mySensorData.rollRateEnabled )
                {
                    const short rollRate = charsToShort( &(rgData[step]) );
                    mySensorData.rollRate = rollRate * m_rScaleRoll;
                    step += 2;
                }

                if ( mySensorData.mxEnabled )
                {
                    const short mx = charsToShort( &(rgData[step]) );
                    mySensorData.mx = mx * m_rScaleMagX;
                    step += 2;
                }

                if ( mySensorData.myEnabled )
                {
                    const short my = charsToShort( &(rgData[step]) );
                    mySensorData.my = my * m_rScaleMagY;
                    step += 2;
                }

                if ( mySensorData.mzEnabled )
                {
                    const short mz = charsToShort( &(rgData[step]) );
                    mySensorData.mz = mz * m_rScaleMagZ;
                    step += 2;
                }

                if ( mySensorData.gxEnabled )
                {
                    const short gx = charsToShort( &(rgData[step]) );
                    mySensorData.gx = gx * m_rScaleGyroX;
                    step += 2;
                }

                if ( mySensorData.gyEnabled )
                {
                    const short gy = charsToShort( &(rgData[step]) );
                    mySensorData.gy = gy * m_rScaleGyroY;
                    step += 2;
                }

                if ( mySensorData.gzEnabled )
                {
                    const short gz = charsToShort( &(rgData[step]) );
                    mySensorData.gz = gz * m_rScaleGyroZ;
                    step += 2;
                }

                if ( mySensorData.axEnabled )
                {
                    const short ax = charsToShort( &(rgData[step]) );
                    mySensorData.ax = ax * m_rScaleAccelX;
                    step += 2;
                }

                if ( mySensorData.ayEnabled )
                {
                    const short ay = charsToShort( &(rgData[step]) );
                    mySensorData.ay = ay * m_rScaleAccelY;
                    step += 2;
                }

                if ( mySensorData.azEnabled )
                {
                    const short az = charsToShort( &(rgData[step]) );
                    mySensorData.az = az * m_rScaleAccelZ;
                    step += 2;
                }

                // call data callback
                m_boostDataCallback( mySensorData );

                //std::cout << std::setprecision(4);
                //std::cout << std::fixed;
                //std::cout << mySensorData.yaw << " " << mySensorData.pitch << " " << mySensorData.roll << "\t" << m_rgData.size() <<  std::endl;
                //std::cout << mySensorData.ax << " " << mySensorData.ay << " " << mySensorData.az << "\t" << m_rgData.size() <<  std::endl;
            }
                break;

            case CHR6dm_GET::GYRO_BIAS_REPORT:
            {
                short gyroBiasReport[3];
                for ( unsigned int i = 0; i < 3; i++ )
                {
                    gyroBiasReport[i] = charsToShort( &(rgData[i*2]) );
                }

                if ( m_bWaitingForResponse )
                {
                    for ( unsigned int i = 0; i < 3; i++ )
                    {
                        m_rgExchangeShort[i] = gyroBiasReport[i];
                    }
                    m_bWaitingForResponse = false;
                }

#ifdef DBG_OUTPUT
                std::cout << "GYRO_BIAS_REPORT: " << (int)ui8PacketLength << std::endl;
                std::cout << gyroBiasReport[0] << " " << gyroBiasReport[1] << " " << gyroBiasReport[2] << " " << std::endl;
#endif
            }
                break;

            case CHR6dm_GET::GYRO_SCALE_REPORT:
            {
                float gyroScaleReport[3];
                for ( unsigned int i = 0; i < 3; i++ )
                {
                    gyroScaleReport[i] = charsToFloat( &(rgData[i*4]) );
                }

                if ( m_bWaitingForResponse )
                {
                    for ( unsigned int i = 0; i < 3; i++ )
                    {
                        m_rgExchangeFloat[i] = gyroScaleReport[i];
                    }
                    m_bWaitingForResponse = false;
                }

#ifdef DBG_OUTPUT
                std::cout << "GYRO_SCALE_REPORT: " << (int)ui8PacketLength << std::endl;
                std::cout << gyroScaleReport[0] << " " << gyroScaleReport[1] << " " << gyroScaleReport[2] << std::endl;
#endif
            }
                break;

            case CHR6dm_GET::START_CAL_REPORT:
            {
                bool startCalReport = charToBool( rgData[0], 0 );

                if ( m_bWaitingForResponse )
                {
                    m_rgExchangeBool[0] = startCalReport;
                    m_bWaitingForResponse = false;
                }

#ifdef DBG_OUTPUT
                std::cout << "START_CAL_REPORT: " << (int)ui8PacketLength << std::endl;
                std::cout << "Gyro bias startup calibriation status: " << startCalReport << std::endl;
#endif
            }
                break;

            case CHR6dm_GET::ACCEL_BIAS_REPORT:
            {
                short accelBiasReport[3];
                for ( unsigned int i = 0; i < 3; i++ )
                {
                    accelBiasReport[i] = charsToShort( &(rgData[i*2]) );
                }

                if ( m_bWaitingForResponse )
                {
                    for ( unsigned int i = 0; i < 3; i++ )
                    {
                        m_rgExchangeShort[i] = accelBiasReport[i];
                    }
                    m_bWaitingForResponse = false;
                }

#ifdef DBG_OUTPUT
                std::cout << "ACCEL_BIAS_REPORT: " << (int)ui8PacketLength << std::endl;
                std::cout << accelBiasReport[0] << " " << accelBiasReport[1] << " " << accelBiasReport[2] << " " << std::endl;
#endif
            }
                break;

            case CHR6dm_GET::ACCEL_REF_VECTOR_REPORT:
            {
                short accelRefVectorReport[3];
                for ( unsigned int i = 0; i < 3; i++ )
                {
                    accelRefVectorReport[i] = charsToShort( &(rgData[i*2]) );
                }

                if ( m_bWaitingForResponse )
                {
                    for ( unsigned int i = 0; i < 3; i++ )
                    {
                        m_rgExchangeShort[i] = accelRefVectorReport[i];
                    }
                    m_bWaitingForResponse = false;
                }

#ifdef DBG_OUTPUT
                std::cout << "ACCEL_REF_VECTOR_REPORT: " << (int)ui8PacketLength << std::endl;
                std::cout << accelRefVectorReport[0] << " " << accelRefVectorReport[1] << " " << accelRefVectorReport[2] << " " << std::endl;
#endif
            }
                break;

            case CHR6dm_GET::ACTIVE_CHANNEL_REPORT:
            {
                SChannelData myChannels( charToBool( rgData[0], 7 ), charToBool( rgData[0], 6 ), charToBool( rgData[0], 5 ), charToBool( rgData[0], 4 ), charToBool( rgData[0], 3 ), charToBool( rgData[0], 2 ), charToBool( rgData[0], 1 ), charToBool( rgData[0], 0 ), charToBool( rgData[1], 7 ), charToBool( rgData[1], 6 ), charToBool( rgData[1], 5 ), charToBool( rgData[1], 4 ), charToBool( rgData[1], 3 ), charToBool( rgData[1], 2 ), charToBool( rgData[1], 1 ) );

#ifdef DBG_OUTPUT
                std::cout << "ACTIVE_CHANNEL_REPORT: " << (int)ui8PacketLength << std::endl;
                std::cout << "yaw:" << myChannels.yawEnabled << " pitch:" << myChannels.pitchEnabled << " roll:" << myChannels.rollEnabled << std::endl;
                std::cout << "yaw_rate:" << myChannels.yawRateEnabled << " pitch_rate:" << myChannels.pitchRateEnabled << " roll_rate:" << myChannels.rollRateEnabled << std::endl;
                std::cout << "mx:" << myChannels.mxEnabled << " my:" << myChannels.myEnabled << " mz:" << myChannels.mzEnabled << std::endl;
                std::cout << "gx:" << myChannels.gxEnabled << " gy:" << myChannels.gyEnabled << " gz:" << myChannels.gzEnabled << std::endl;
                std::cout << "ax:" << myChannels.axEnabled << " ay:" << myChannels.ayEnabled << " az:" << myChannels.azEnabled << std::endl;
#endif
            }
                break;

            case CHR6dm_GET::ACCEL_COVARIANCE_REPORT:
            {
                float accelCovarianceReport = charsToFloat( &rgData[0] );

                if ( m_bWaitingForResponse )
                {
                    m_rgExchangeFloat[0] = accelCovarianceReport;
                    m_bWaitingForResponse = false;
                }

#ifdef DBG_OUTPUT
                std::cout << "ACCEL_COVARIANCE_REPORT: " << (int)ui8PacketLength << std::endl;
                std::cout << "ACCEL_COVARIANCE_REPORT: " << accelCovarianceReport << std::endl;
#endif
            }
                break;

            case CHR6dm_GET::MAG_COVARIANCE_REPORT:
            {
                float magCovarianceReport = charsToFloat( &rgData[0] );

                if ( m_bWaitingForResponse )
                {
                    m_rgExchangeFloat[0] = magCovarianceReport;
                    m_bWaitingForResponse = false;
                }

#ifdef DBG_OUTPUT
                std::cout << "MAG_COVARIANCE_REPORT: " << (int)ui8PacketLength << std::endl;
                std::cout << "MAG_COVARIANCE_REPORT: " << magCovarianceReport << std::endl;
#endif
            }
                break;

            case CHR6dm_GET::PROCESS_COVARIANCE_REPORT:
            {
                float processCovarianceReport = charsToFloat( &rgData[0] );

                if ( m_bWaitingForResponse )
                {
                    m_rgExchangeFloat[0] = processCovarianceReport;
                    m_bWaitingForResponse = false;
                }

#ifdef DBG_OUTPUT
                std::cout << "PROCESS_COVARIANCE_REPORT: " << (int)ui8PacketLength << std::endl;
                std::cout << "PROCESS_COVARIANCE_REPORT: " << processCovarianceReport << std::endl;
#endif
            }
                break;

            case CHR6dm_GET::STATE_COVARIANCE_REPORT:
            {
                float stateCovarianceReport[9];
                for ( unsigned int i = 0; i < 9; i++ )
                {
                    stateCovarianceReport[i] = charsToFloat( &(rgData[i*4]) );
                }

                if ( m_bWaitingForResponse )
                {
                    for ( unsigned int i = 0; i < 9; i++ )
                    {
                        m_rgExchangeFloat[i] = stateCovarianceReport[i];
                    }
                    m_bWaitingForResponse = false;
                }

#ifdef DBG_OUTPUT
                std::cout << "STATE_COVARIANCE_REPORT: " << (int)ui8PacketLength << std::endl;
                std::cout << stateCovarianceReport[0] << " " << stateCovarianceReport[1] << " " << stateCovarianceReport[2] << std::endl;
                std::cout << stateCovarianceReport[3] << " " << stateCovarianceReport[4] << " " << stateCovarianceReport[5] << std::endl;
                std::cout << stateCovarianceReport[6] << " " << stateCovarianceReport[7] << " " << stateCovarianceReport[8] << std::endl;
#endif
            }
                break;

            case CHR6dm_GET::EKF_CONFIG_REPORT:
            {
                bool ekfConfigReport[2];
                ekfConfigReport[0] = charToBool( rgData[0], 0 );
                ekfConfigReport[1] = charToBool( rgData[0], 1 );

                if ( m_bWaitingForResponse )
                {
                    m_rgExchangeBool[0] = ekfConfigReport[0];
                    m_rgExchangeBool[1] = ekfConfigReport[1];
                    m_bWaitingForResponse = false;
                }

#ifdef DBG_OUTPUT
                std::cout << "EKF_CONFIG_REPORT: " << (int)ui8PacketLength << std::endl;
                std::cout << "MagnetometerUpdatesUsed: " << ekfConfigReport[0] << std::endl;
                std::cout << "AccelerometerUpdatesUsed: " << ekfConfigReport[1] << std::endl;
#endif
            }
                break;

            case CHR6dm_GET::GYRO_ALIGNMENT_REPORT:
            {
                float gyroAlignmentReport[9];
                for ( unsigned int i = 0; i < 9; i++ )
                {
                    gyroAlignmentReport[i] = charsToFloat( &(rgData[i*4]) );
                }

                if ( m_bWaitingForResponse )
                {
                    for ( unsigned int i = 0; i < 9; i++ )
                    {
                        m_rgExchangeFloat[i] = gyroAlignmentReport[i];
                    }
                    m_bWaitingForResponse = false;
                }

#ifdef DBG_OUTPUT
                std::cout << "GYRO_ALIGNMENT_REPORT: " << (int)ui8PacketLength << std::endl;
                std::cout << gyroAlignmentReport[0] << " " << gyroAlignmentReport[1] << " " << gyroAlignmentReport[2] << std::endl;
                std::cout << gyroAlignmentReport[3] << " " << gyroAlignmentReport[4] << " " << gyroAlignmentReport[5] << std::endl;
                std::cout << gyroAlignmentReport[6] << " " << gyroAlignmentReport[7] << " " << gyroAlignmentReport[8] << std::endl;
#endif
            }
                break;

            case CHR6dm_GET::ACCEL_ALIGNMENT_REPORT:
            {
                float accelAlignmentReport[9];
                for ( unsigned int i = 0; i < 9; i++ )
                {
                    accelAlignmentReport[i] = charsToFloat( &(rgData[i*4]) );
                }

                if ( m_bWaitingForResponse )
                {
                    for ( unsigned int i = 0; i < 9; i++ )
                    {
                        m_rgExchangeFloat[i] = accelAlignmentReport[i];
                    }
                    m_bWaitingForResponse = false;
                }

#ifdef DBG_OUTPUT
                std::cout << "ACCEL_ALIGNMENT_REPORT: " << (int)ui8PacketLength << std::endl;
                std::cout << accelAlignmentReport[0] << " " << accelAlignmentReport[1] << " " << accelAlignmentReport[2] << std::endl;
                std::cout << accelAlignmentReport[3] << " " << accelAlignmentReport[4] << " " << accelAlignmentReport[5] << std::endl;
                std::cout << accelAlignmentReport[6] << " " << accelAlignmentReport[7] << " " << accelAlignmentReport[8] << std::endl;
#endif
            }
                break;

            case CHR6dm_GET::MAG_REF_VECTOR_REPORT:
            {
                short magRefVectorReport[3];
                for ( unsigned int i = 0; i < 3; i++ )
                {
                    magRefVectorReport[i] = charsToShort( &(rgData[i*2]) );
                }

                if ( m_bWaitingForResponse )
                {
                    for ( unsigned int i = 0; i < 3; i++ )
                    {
                        m_rgExchangeShort[i] = magRefVectorReport[i];
                    }
                    m_bWaitingForResponse = false;
                }

#ifdef DBG_OUTPUT
                std::cout << "MAG_REF_VECTOR_REPORT: " << (int)ui8PacketLength << std::endl;
                std::cout << magRefVectorReport[0] << " " << magRefVectorReport[1] << " " << magRefVectorReport[2] << " " << std::endl;
#endif
            }
                break;

            case CHR6dm_GET::MAG_CAL_REPORT:
            {
                float magCalReport[9];
                for ( unsigned int i = 0; i < 9; i++ )
                {
                    magCalReport[i] = charsToFloat( &(rgData[i*4]) );
                }

                if ( m_bWaitingForResponse )
                {
                    for ( unsigned int i = 0; i < 9; i++ )
                    {
                        m_rgExchangeFloat[i] = magCalReport[i];
                    }
                    m_bWaitingForResponse = false;
                }

#ifdef DBG_OUTPUT
                std::cout << "MAG_CAL_REPORT: " << (int)ui8PacketLength << std::endl;
                std::cout << magCalReport[0] << " " << magCalReport[1] << " " << magCalReport[2] << std::endl;
                std::cout << magCalReport[3] << " " << magCalReport[4] << " " << magCalReport[5] << std::endl;
                std::cout << magCalReport[6] << " " << magCalReport[7] << " " << magCalReport[8] << std::endl;
#endif
            }
                break;

            case CHR6dm_GET::MAG_BIAS_REPORT:
            {
                short magBiasReport[3];
                for ( unsigned int i = 0; i < 3; i++ )
                {
                    magBiasReport[i] = charsToShort( &(rgData[i*2]) );
                }

                if ( m_bWaitingForResponse )
                {
                    for ( unsigned int i = 0; i < 3; i++ )
                    {
                        m_rgExchangeShort[i] = magBiasReport[i];
                    }
                    m_bWaitingForResponse = false;
                }

#ifdef DBG_OUTPUT
                std::cout << "MAG_BIAS_REPORT: " << (int)ui8PacketLength << std::endl;
                std::cout << magBiasReport[0] << " " << magBiasReport[1] << " " << magBiasReport[2] << " " << std::endl;
#endif
            }
                break;

            case CHR6dm_GET::BROADCAST_MODE_REPORT:
            {
                int nFrequency = ( (rgData[0])*(280.0/255.0)+20 );
                bool fChannel = ( charToBool( rgData[1], 0 ) );

                if ( m_bWaitingForResponse )
                {
                    m_rgExchangeInt[0] = nFrequency;
                    m_rgExchangeBool[0] = fChannel;
                    m_bWaitingForResponse = false;
                }

#ifdef DBG_OUTPUT
                std::cout << "BROADCAST_MODE_REPORT: " << (int)ui8PacketLength << std::endl;
                std::cout << "Frequency: " << nFrequency << "Hz. Mode: " << fChannel << std::endl;
#endif
            }
                break;

            default:
                std::cerr << "[ERROR] " << (int)ui8PacketType << ": " << (int)ui8PacketLength << std::endl;
            }

            // reset
            bSyncToHeader = false;

            // setting interruption point for exiting thread
            boost::this_thread::interruption_point();
        }
    }
    catch( boost::thread_interrupted& ) {;}
}

void CCHR6dm::sendSetActiveChannels( const SChannelData activeChannels  )
{
    unsigned char data[2] = { 0, 0 };
    data[0] = (unsigned char)( (activeChannels.myEnabled) | (activeChannels.mxEnabled << 1) | (activeChannels.rollRateEnabled << 2) | (activeChannels.pitchRateEnabled << 3) | (activeChannels.yawRateEnabled << 4) | (activeChannels.rollEnabled << 5) | (activeChannels.pitchEnabled << 6) | (activeChannels.yawEnabled << 7) );
    data[1] = (unsigned char)( (activeChannels.azEnabled << 1) | (activeChannels.ayEnabled << 2) | (activeChannels.axEnabled << 3) | (activeChannels.gzEnabled << 4) | (activeChannels.gyEnabled << 5) | (activeChannels.gxEnabled << 6) | (activeChannels.mzEnabled << 7) );

    sendRawMessage( CHR6dm_SEND::SET_ACTIVE_CHANNELS, 2, &data[0] );
}

void CCHR6dm::sendSetSilentMode( void )
{
    sendRawMessage( CHR6dm_SEND::SET_SILENT_MODE, 0, 0 );
}

void CCHR6dm::sendSetBroadcastMode( const int nFrequency )
{
    if ( nFrequency > 300 || nFrequency < 20 )
    {
        std::cerr << "Choose Frequency between 20 and 300 Hz." << std::endl;
        return;
    }

    // calculate frequency, see documention for formular
    unsigned char data( (nFrequency-20) * (255.0/280.0) );

#ifdef DBG_OUTPUT
    std::cout << "Set frequency to " << nFrequency << "Hz (" << (int)data << ")" << std::endl;
#endif

    sendRawMessage( CHR6dm_SEND::SET_BROADCAST_MODE, 1, &data );
}

void CCHR6dm::sendSetGyroBias( const short nGyro_x_bias, const short nGyro_y_bias, const short nGyro_z_bias )
{
    unsigned char data[6];
    data[0] = ((unsigned char*) &nGyro_z_bias)[1];
    data[1] = ((unsigned char*) &nGyro_z_bias)[0];

    data[2] = ((unsigned char*) &nGyro_y_bias)[1];
    data[3] = ((unsigned char*) &nGyro_y_bias)[0];

    data[4] = ((unsigned char*) &nGyro_x_bias)[1];
    data[5] = ((unsigned char*) &nGyro_x_bias)[0];

    sendRawMessage( CHR6dm_SEND::SET_GYRO_BIAS, 6, &data[0] );
}

void CCHR6dm::sendSetAccelBias( const short nAccel_x_bias, const short nAccel_y_bias, const short nAccel_z_bias )
{
    unsigned char data[6];
    data[0] = ((unsigned char*) &nAccel_z_bias)[1];
    data[1] = ((unsigned char*) &nAccel_z_bias)[0];

    data[2] = ((unsigned char*) &nAccel_y_bias)[1];
    data[3] = ((unsigned char*) &nAccel_y_bias)[0];

    data[4] = ((unsigned char*) &nAccel_x_bias)[1];
    data[5] = ((unsigned char*) &nAccel_x_bias)[0];

    sendRawMessage( CHR6dm_SEND::SET_ACCEL_BIAS, 6, &data[0] );
}

void CCHR6dm::sendSetAccelRefVector( const short nAccelRef_x, const short nAccelRef_y, const short nAccelRef_z )
{
    unsigned char data[6];
    data[0] = ((unsigned char*) &nAccelRef_z)[1];
    data[1] = ((unsigned char*) &nAccelRef_z)[0];

    data[2] = ((unsigned char*) &nAccelRef_y)[1];
    data[3] = ((unsigned char*) &nAccelRef_y)[0];

    data[4] = ((unsigned char*) &nAccelRef_x)[1];
    data[5] = ((unsigned char*) &nAccelRef_x)[0];

    sendRawMessage( CHR6dm_SEND::SET_ACCEL_REF_VECTOR, 6, &data[0] );
}

void CCHR6dm::sendAutoSetAccelRef( void )
{
    sendRawMessage( CHR6dm_SEND::AUTO_SET_ACCEL_REF, 0, 0 );
}

void CCHR6dm::sendZeroRateGyros( void )
{
    sendRawMessage( CHR6dm_SEND::ZERO_RATE_GYROS, 0, 0 );
}

void CCHR6dm::sendSelfTest( void )
{
    sendRawMessage( CHR6dm_SEND::SELF_TEST, 0, 0 );
}

void CCHR6dm::sendSetStartCal( const bool bAutomaticStartupCalibriationEnabled )
{
    unsigned char data( 0 );
    data = (unsigned char)( bAutomaticStartupCalibriationEnabled );

    sendRawMessage( CHR6dm_SEND::SET_START_CAL, 1, &data );
}

void CCHR6dm::sendSetProcessCovariance( const float rProcessCovariance )
{
    unsigned char data[] = { ((unsigned char*) &rProcessCovariance)[3], ((unsigned char*) &rProcessCovariance)[2], ((unsigned char*) &rProcessCovariance)[1], ((unsigned char*) &rProcessCovariance)[0] };

    sendRawMessage( CHR6dm_SEND::SET_PROCESS_COVARIANCE, 4, &data[0] );
}

void CCHR6dm::sendSetMagCovariance( const float rMagnetometerCovariance )
{
    unsigned char data[] = { ((unsigned char*) &rMagnetometerCovariance)[3], ((unsigned char*) &rMagnetometerCovariance)[2], ((unsigned char*) &rMagnetometerCovariance)[1], ((unsigned char*) &rMagnetometerCovariance)[0] };

    sendRawMessage( CHR6dm_SEND::SET_MAG_COVARIANCE, 4, &data[0] );
}

void CCHR6dm::sendSetAccelCovariance( const float rAccelerometerCovariance )
{
    unsigned char data[] = { ((unsigned char*) &rAccelerometerCovariance)[3], ((unsigned char*) &rAccelerometerCovariance)[2], ((unsigned char*) &rAccelerometerCovariance)[1], ((unsigned char*) &rAccelerometerCovariance)[0] };

    sendRawMessage( CHR6dm_SEND::SET_ACCEL_COVARIANCE, 4, &data[0] );
}

void CCHR6dm::sendSetEkfConfig( const bool bMagEnabled, const bool bAccelEnabled )
{
    unsigned char data( 0 );
    data = (unsigned char)( bMagEnabled | (bAccelEnabled << 1) );

    sendRawMessage( CHR6dm_SEND::SET_EKF_CONFIG, 1, &data );
}

void CCHR6dm::sendSetGyroAlignment( const SMatrix3x3 matrix )
{
    unsigned char data[36];
    data[ 0] = ((unsigned char*) &(matrix.entry11))[3];
    data[ 1] = ((unsigned char*) &(matrix.entry11))[2];
    data[ 2] = ((unsigned char*) &(matrix.entry11))[1];
    data[ 3] = ((unsigned char*) &(matrix.entry11))[0];

    data[ 4] = ((unsigned char*) &(matrix.entry12))[3];
    data[ 5] = ((unsigned char*) &(matrix.entry12))[2];
    data[ 6] = ((unsigned char*) &(matrix.entry12))[1];
    data[ 7] = ((unsigned char*) &(matrix.entry12))[0];

    data[ 8] = ((unsigned char*) &(matrix.entry13))[3];
    data[ 9] = ((unsigned char*) &(matrix.entry13))[2];
    data[10] = ((unsigned char*) &(matrix.entry13))[1];
    data[11] = ((unsigned char*) &(matrix.entry13))[0];

    data[12] = ((unsigned char*) &(matrix.entry21))[3];
    data[13] = ((unsigned char*) &(matrix.entry21))[2];
    data[14] = ((unsigned char*) &(matrix.entry21))[1];
    data[15] = ((unsigned char*) &(matrix.entry21))[0];

    data[16] = ((unsigned char*) &(matrix.entry22))[3];
    data[17] = ((unsigned char*) &(matrix.entry22))[2];
    data[18] = ((unsigned char*) &(matrix.entry22))[1];
    data[19] = ((unsigned char*) &(matrix.entry22))[0];

    data[20] = ((unsigned char*) &(matrix.entry23))[3];
    data[21] = ((unsigned char*) &(matrix.entry23))[2];
    data[22] = ((unsigned char*) &(matrix.entry23))[1];
    data[23] = ((unsigned char*) &(matrix.entry23))[0];

    data[24] = ((unsigned char*) &(matrix.entry31))[3];
    data[25] = ((unsigned char*) &(matrix.entry31))[2];
    data[26] = ((unsigned char*) &(matrix.entry31))[1];
    data[27] = ((unsigned char*) &(matrix.entry31))[0];

    data[28] = ((unsigned char*) &(matrix.entry32))[3];
    data[29] = ((unsigned char*) &(matrix.entry32))[2];
    data[30] = ((unsigned char*) &(matrix.entry32))[1];
    data[31] = ((unsigned char*) &(matrix.entry32))[0];

    data[32] = ((unsigned char*) &(matrix.entry33))[3];
    data[33] = ((unsigned char*) &(matrix.entry33))[2];
    data[34] = ((unsigned char*) &(matrix.entry33))[1];
    data[35] = ((unsigned char*) &(matrix.entry33))[0];

    sendRawMessage( CHR6dm_SEND::SET_GYRO_ALIGNMENT, 36, &data[0] );
}

void CCHR6dm::sendSetAccelAlignment( const SMatrix3x3 matrix )
{
    unsigned char data[36];
    data[ 0] = ((unsigned char*) &(matrix.entry11))[3];
    data[ 1] = ((unsigned char*) &(matrix.entry11))[2];
    data[ 2] = ((unsigned char*) &(matrix.entry11))[1];
    data[ 3] = ((unsigned char*) &(matrix.entry11))[0];

    data[ 4] = ((unsigned char*) &(matrix.entry12))[3];
    data[ 5] = ((unsigned char*) &(matrix.entry12))[2];
    data[ 6] = ((unsigned char*) &(matrix.entry12))[1];
    data[ 7] = ((unsigned char*) &(matrix.entry12))[0];

    data[ 8] = ((unsigned char*) &(matrix.entry13))[3];
    data[ 9] = ((unsigned char*) &(matrix.entry13))[2];
    data[10] = ((unsigned char*) &(matrix.entry13))[1];
    data[11] = ((unsigned char*) &(matrix.entry13))[0];

    data[12] = ((unsigned char*) &(matrix.entry21))[3];
    data[13] = ((unsigned char*) &(matrix.entry21))[2];
    data[14] = ((unsigned char*) &(matrix.entry21))[1];
    data[15] = ((unsigned char*) &(matrix.entry21))[0];

    data[16] = ((unsigned char*) &(matrix.entry22))[3];
    data[17] = ((unsigned char*) &(matrix.entry22))[2];
    data[18] = ((unsigned char*) &(matrix.entry22))[1];
    data[19] = ((unsigned char*) &(matrix.entry22))[0];

    data[20] = ((unsigned char*) &(matrix.entry23))[3];
    data[21] = ((unsigned char*) &(matrix.entry23))[2];
    data[22] = ((unsigned char*) &(matrix.entry23))[1];
    data[23] = ((unsigned char*) &(matrix.entry23))[0];

    data[24] = ((unsigned char*) &(matrix.entry31))[3];
    data[25] = ((unsigned char*) &(matrix.entry31))[2];
    data[26] = ((unsigned char*) &(matrix.entry31))[1];
    data[27] = ((unsigned char*) &(matrix.entry31))[0];

    data[28] = ((unsigned char*) &(matrix.entry32))[3];
    data[29] = ((unsigned char*) &(matrix.entry32))[2];
    data[30] = ((unsigned char*) &(matrix.entry32))[1];
    data[31] = ((unsigned char*) &(matrix.entry32))[0];

    data[32] = ((unsigned char*) &(matrix.entry33))[3];
    data[33] = ((unsigned char*) &(matrix.entry33))[2];
    data[34] = ((unsigned char*) &(matrix.entry33))[1];
    data[35] = ((unsigned char*) &(matrix.entry33))[0];

    sendRawMessage( CHR6dm_SEND::SET_ACCEL_ALIGNMENT, 36, &data[0] );
}

void CCHR6dm::sendSetMagRefVector( const short nMagRef_x, const short nMagRef_y, const short nMagRef_z )
{
    unsigned char data[6];
    data[0] = ((unsigned char*) &nMagRef_z)[1];
    data[1] = ((unsigned char*) &nMagRef_z)[0];

    data[2] = ((unsigned char*) &nMagRef_y)[1];
    data[3] = ((unsigned char*) &nMagRef_y)[0];

    data[4] = ((unsigned char*) &nMagRef_x)[1];
    data[5] = ((unsigned char*) &nMagRef_x)[0];

    sendRawMessage( CHR6dm_SEND::SET_MAG_REF_VECTOR, 6, &data[0] );
}

void CCHR6dm::sendAutoSetMagRef( void )
{
    sendRawMessage( CHR6dm_SEND::AUTO_SET_MAG_REF, 0, 0 );
}

void CCHR6dm::sendSetMagCal( const SMatrix3x3 matrix )
{
    unsigned char data[36];
    data[ 0] = ((unsigned char*) &(matrix.entry11))[3];
    data[ 1] = ((unsigned char*) &(matrix.entry11))[2];
    data[ 2] = ((unsigned char*) &(matrix.entry11))[1];
    data[ 3] = ((unsigned char*) &(matrix.entry11))[0];

    data[ 4] = ((unsigned char*) &(matrix.entry12))[3];
    data[ 5] = ((unsigned char*) &(matrix.entry12))[2];
    data[ 6] = ((unsigned char*) &(matrix.entry12))[1];
    data[ 7] = ((unsigned char*) &(matrix.entry12))[0];

    data[ 8] = ((unsigned char*) &(matrix.entry13))[3];
    data[ 9] = ((unsigned char*) &(matrix.entry13))[2];
    data[10] = ((unsigned char*) &(matrix.entry13))[1];
    data[11] = ((unsigned char*) &(matrix.entry13))[0];

    data[12] = ((unsigned char*) &(matrix.entry21))[3];
    data[13] = ((unsigned char*) &(matrix.entry21))[2];
    data[14] = ((unsigned char*) &(matrix.entry21))[1];
    data[15] = ((unsigned char*) &(matrix.entry21))[0];

    data[16] = ((unsigned char*) &(matrix.entry22))[3];
    data[17] = ((unsigned char*) &(matrix.entry22))[2];
    data[18] = ((unsigned char*) &(matrix.entry22))[1];
    data[19] = ((unsigned char*) &(matrix.entry22))[0];

    data[20] = ((unsigned char*) &(matrix.entry23))[3];
    data[21] = ((unsigned char*) &(matrix.entry23))[2];
    data[22] = ((unsigned char*) &(matrix.entry23))[1];
    data[23] = ((unsigned char*) &(matrix.entry23))[0];

    data[24] = ((unsigned char*) &(matrix.entry31))[3];
    data[25] = ((unsigned char*) &(matrix.entry31))[2];
    data[26] = ((unsigned char*) &(matrix.entry31))[1];
    data[27] = ((unsigned char*) &(matrix.entry31))[0];

    data[28] = ((unsigned char*) &(matrix.entry32))[3];
    data[29] = ((unsigned char*) &(matrix.entry32))[2];
    data[30] = ((unsigned char*) &(matrix.entry32))[1];
    data[31] = ((unsigned char*) &(matrix.entry32))[0];

    data[32] = ((unsigned char*) &(matrix.entry33))[3];
    data[33] = ((unsigned char*) &(matrix.entry33))[2];
    data[34] = ((unsigned char*) &(matrix.entry33))[1];
    data[35] = ((unsigned char*) &(matrix.entry33))[0];

    sendRawMessage( CHR6dm_SEND::SET_MAG_CAL, 36, &data[0] );
}

void CCHR6dm::sendSetMagBias( const short nMagBias_x, const short nMagBias_y, const short nMagBias_z )
{
    unsigned char data[6];
    data[0] = ((unsigned char*) &nMagBias_z)[1];
    data[1] = ((unsigned char*) &nMagBias_z)[0];

    data[2] = ((unsigned char*) &nMagBias_y)[1];
    data[3] = ((unsigned char*) &nMagBias_y)[0];

    data[4] = ((unsigned char*) &nMagBias_x)[1];
    data[5] = ((unsigned char*) &nMagBias_x)[0];

    sendRawMessage( CHR6dm_SEND::SET_MAG_BIAS, 6, &data[0] );
}

void CCHR6dm::sendSetGyroScale( const float rXGyroScale, const float rYGyroScale, const float rZGyroScale )
{
    unsigned char data[12];
    data[0] = ((unsigned char*) &rZGyroScale)[3];
    data[1] = ((unsigned char*) &rZGyroScale)[2];
    data[2] = ((unsigned char*) &rZGyroScale)[1];
    data[3] = ((unsigned char*) &rZGyroScale)[0];

    data[4] = ((unsigned char*) &rYGyroScale)[3];
    data[5] = ((unsigned char*) &rYGyroScale)[2];
    data[6] = ((unsigned char*) &rYGyroScale)[1];
    data[7] = ((unsigned char*) &rYGyroScale)[0];

    data[8] = ((unsigned char*) &rXGyroScale)[3];
    data[9] = ((unsigned char*) &rXGyroScale)[2];
    data[10] = ((unsigned char*) &rXGyroScale)[1];
    data[11] = ((unsigned char*) &rXGyroScale)[0];

    sendRawMessage( CHR6dm_SEND::SET_GYRO_SCALE, 12, &data[0] );
}

void CCHR6dm::sendEkfReset( void )
{
    sendRawMessage( CHR6dm_SEND::EKF_RESET, 0, 0 );
}

void CCHR6dm::sendResetToFactory( void )
{
    sendRawMessage( CHR6dm_SEND::RESET_TO_FACTORY, 0, 0 );
}

void CCHR6dm::sendWriteToFlash( void )
{
    sendRawMessage( CHR6dm_SEND::WRITE_TO_FLASH, 0, 0 );
}

void CCHR6dm::sendGetData( void )
{
    sendRawMessage( CHR6dm_SEND::GET_DATA, 0, 0 );
}

void CCHR6dm::sendGetActiveChannels( void )
{
    sendRawMessage( CHR6dm_SEND::GET_ACTIVE_CHANNELS, 0, 0 );
}

unsigned int CCHR6dm::sendGetBroadcastMode( const unsigned int nTimeoutMS )
{
    sendRawMessage( CHR6dm_SEND::GET_BROADCAST_MODE, 0, 0 );

    unsigned int nFrequency( 0 );
    bool fMode( false );

    boost::posix_time::ptime startTime( boost::posix_time::microsec_clock::local_time() );

    boost::posix_time::time_duration timeout = boost::posix_time::milliseconds( nTimeoutMS );

    m_bWaitingForResponse = true;

    while ( (( (boost::posix_time::microsec_clock::local_time()) - startTime ) < timeout) && m_bWaitingForResponse )
    {
        boost::this_thread::sleep( boost::posix_time::milliseconds(100) );
    }

#ifdef DBG_OUTPUT
    std::cout << "Duration: " << ((boost::posix_time::microsec_clock::local_time()) - startTime) << std::endl;
#endif

    if ( fMode )
    {
        return nFrequency;
    }
    else
    {
        return 0;
    }

}

SMatrix3x1short CCHR6dm::sendGetAccelBias( const unsigned int nTimeoutMS )
{
    sendRawMessage( CHR6dm_SEND::GET_ACCEL_BIAS, 0, 0 );

    SMatrix3x1short i16AccelBias;

    boost::posix_time::ptime startTime( boost::posix_time::microsec_clock::local_time() );

    boost::posix_time::time_duration timeout = boost::posix_time::milliseconds( nTimeoutMS );

    m_bWaitingForResponse = true;

    while ( (( (boost::posix_time::microsec_clock::local_time()) - startTime ) < timeout) && m_bWaitingForResponse )
    {
        boost::this_thread::sleep( boost::posix_time::milliseconds(100) );
    }

    i16AccelBias.entry11 = m_rgExchangeShort.at(0);
    i16AccelBias.entry21 = m_rgExchangeShort.at(1);
    i16AccelBias.entry31 = m_rgExchangeShort.at(2);

#ifdef DBG_OUTPUT
    std::cout << "Duration: " << ((boost::posix_time::microsec_clock::local_time()) - startTime) << std::endl;
#endif

    return i16AccelBias;
}

SMatrix3x1short CCHR6dm::sendGetAccelRefVector( const unsigned int nTimeoutMS )
{
    sendRawMessage( CHR6dm_SEND::GET_ACCEL_REF_VECTOR, 0, 0 );

    SMatrix3x1short i16AccelRefVector;

    boost::posix_time::ptime startTime( boost::posix_time::microsec_clock::local_time() );

    boost::posix_time::time_duration timeout = boost::posix_time::milliseconds( nTimeoutMS );

    m_bWaitingForResponse = true;

    while ( (( (boost::posix_time::microsec_clock::local_time()) - startTime ) < timeout) && m_bWaitingForResponse )
    {
        boost::this_thread::sleep( boost::posix_time::milliseconds(100) );
    }

    i16AccelRefVector.entry11 = m_rgExchangeShort.at(0);
    i16AccelRefVector.entry21 = m_rgExchangeShort.at(1);
    i16AccelRefVector.entry31 = m_rgExchangeShort.at(2);

#ifdef DBG_OUTPUT
    std::cout << "Duration: " << ((boost::posix_time::microsec_clock::local_time()) - startTime) << std::endl;
#endif

    return i16AccelRefVector;
}

SMatrix3x1short CCHR6dm::sendGetGyroBias( const unsigned int nTimeoutMS )
{
    sendRawMessage( CHR6dm_SEND::GET_GYRO_BIAS, 0, 0 );

    SMatrix3x1short i16GyroBias;

    boost::posix_time::ptime startTime( boost::posix_time::microsec_clock::local_time() );

    boost::posix_time::time_duration timeout = boost::posix_time::milliseconds( nTimeoutMS );

    m_bWaitingForResponse = true;

    while ( (( (boost::posix_time::microsec_clock::local_time()) - startTime ) < timeout) && m_bWaitingForResponse )
    {
        boost::this_thread::sleep( boost::posix_time::milliseconds(100) );
    }

    i16GyroBias.entry11 = m_rgExchangeShort.at(0);
    i16GyroBias.entry21 = m_rgExchangeShort.at(1);
    i16GyroBias.entry31 = m_rgExchangeShort.at(2);

#ifdef DBG_OUTPUT
    std::cout << "Duration: " << ((boost::posix_time::microsec_clock::local_time()) - startTime) << std::endl;
#endif

    return i16GyroBias;
}

SMatrix3x1float CCHR6dm::sendGetGyroScale( const unsigned int nTimeoutMS )
{
    sendRawMessage( CHR6dm_SEND::GET_GYRO_SCALE, 0, 0 );

    SMatrix3x1float i16GyroScale;

    boost::posix_time::ptime startTime( boost::posix_time::microsec_clock::local_time() );

    boost::posix_time::time_duration timeout = boost::posix_time::milliseconds( nTimeoutMS );

    m_bWaitingForResponse = true;

    while ( (( (boost::posix_time::microsec_clock::local_time()) - startTime ) < timeout) && m_bWaitingForResponse )
    {
        boost::this_thread::sleep( boost::posix_time::milliseconds(100) );
    }

    i16GyroScale.entry11 = m_rgExchangeFloat.at(0);
    i16GyroScale.entry21 = m_rgExchangeFloat.at(1);
    i16GyroScale.entry31 = m_rgExchangeFloat.at(2);

#ifdef DBG_OUTPUT
    std::cout << "Duration: " << ((boost::posix_time::microsec_clock::local_time()) - startTime) << std::endl;
#endif

    return i16GyroScale;
}

bool CCHR6dm::sendGetStartCal( const unsigned int nTimeoutMS )
{
    sendRawMessage( CHR6dm_SEND::GET_START_CAL, 0, 0 );

    bool fStartCal( false );

    boost::posix_time::ptime startTime( boost::posix_time::microsec_clock::local_time() );

    boost::posix_time::time_duration timeout = boost::posix_time::milliseconds( nTimeoutMS );

    m_bWaitingForResponse = true;

    while ( (( (boost::posix_time::microsec_clock::local_time()) - startTime ) < timeout) && m_bWaitingForResponse )
    {
        boost::this_thread::sleep( boost::posix_time::milliseconds(100) );
    }

    fStartCal = m_rgExchangeBool.at(0);

#ifdef DBG_OUTPUT
    std::cout << "Duration: " << ((boost::posix_time::microsec_clock::local_time()) - startTime) << std::endl;
#endif

    return fStartCal;
}

void CCHR6dm::sendGetEkfConfig( const unsigned int nTimeoutMS )
{
    sendRawMessage( CHR6dm_SEND::GET_EKF_CONFIG, 0, 0 );

    bool fMagnetometerUpdatesUsed( false );
    bool fAccelerometerUpdatesUsed( false );

    boost::posix_time::ptime startTime( boost::posix_time::microsec_clock::local_time() );

    boost::posix_time::time_duration timeout = boost::posix_time::milliseconds( nTimeoutMS );

    m_bWaitingForResponse = true;

    while ( (( (boost::posix_time::microsec_clock::local_time()) - startTime ) < timeout) && m_bWaitingForResponse )
    {
        boost::this_thread::sleep( boost::posix_time::milliseconds(100) );
    }

    fMagnetometerUpdatesUsed = m_rgExchangeBool.at(0);
    fAccelerometerUpdatesUsed = m_rgExchangeBool.at(1);

#ifdef DBG_OUTPUT
    std::cout << "MagnetometerUpdatesUsed: " << fMagnetometerUpdatesUsed << std::endl;
    std::cout << "AccelerometerUpdatesUsed: " << fAccelerometerUpdatesUsed << std::endl;
#endif
}

float CCHR6dm::sendGetAccelCovariance( const unsigned int nTimeoutMS )
{
    sendRawMessage( CHR6dm_SEND::GET_ACCEL_COVARIANCE, 0, 0 );

    float rAccelCovariance( 0.0f );

    boost::posix_time::ptime startTime( boost::posix_time::microsec_clock::local_time() );

    boost::posix_time::time_duration timeout = boost::posix_time::milliseconds( nTimeoutMS );

    m_bWaitingForResponse = true;

    while ( (( (boost::posix_time::microsec_clock::local_time()) - startTime ) < timeout) && m_bWaitingForResponse )
    {
        boost::this_thread::sleep( boost::posix_time::milliseconds(100) );
    }

    rAccelCovariance = m_rgExchangeFloat.at(0);

#ifdef DBG_OUTPUT
    std::cout << "Duration: " << ((boost::posix_time::microsec_clock::local_time()) - startTime) << std::endl;
#endif

    return rAccelCovariance;
}

float CCHR6dm::sendGetMagCovariance( const unsigned int nTimeoutMS )
{
    sendRawMessage( CHR6dm_SEND::GET_MAG_COVARIANCE, 0, 0 );

    float rMagCovariance( 0.0f );

    boost::posix_time::ptime startTime( boost::posix_time::microsec_clock::local_time() );

    boost::posix_time::time_duration timeout = boost::posix_time::milliseconds( nTimeoutMS );

    m_bWaitingForResponse = true;

    while ( (( (boost::posix_time::microsec_clock::local_time()) - startTime ) < timeout) && m_bWaitingForResponse )
    {
        boost::this_thread::sleep( boost::posix_time::milliseconds(100) );
    }

    rMagCovariance = m_rgExchangeFloat.at(0);

#ifdef DBG_OUTPUT
    std::cout << "Duration: " << ((boost::posix_time::microsec_clock::local_time()) - startTime) << std::endl;
#endif

    return rMagCovariance;
}

float CCHR6dm::sendGetProcessCovariance( const unsigned int nTimeoutMS )
{
    sendRawMessage( CHR6dm_SEND::GET_PROCESS_COVARIANCE, 0, 0 );

    float rProcessCovariance( 0.0f );

    boost::posix_time::ptime startTime( boost::posix_time::microsec_clock::local_time() );

    boost::posix_time::time_duration timeout = boost::posix_time::milliseconds( nTimeoutMS );

    m_bWaitingForResponse = true;

    while ( (( (boost::posix_time::microsec_clock::local_time()) - startTime ) < timeout) && m_bWaitingForResponse )
    {
        boost::this_thread::sleep( boost::posix_time::milliseconds(100) );
    }

    rProcessCovariance = m_rgExchangeFloat.at(0);

#ifdef DBG_OUTPUT
    std::cout << "Duration: " << ((boost::posix_time::microsec_clock::local_time()) - startTime) << std::endl;
#endif

    return rProcessCovariance;
}

SMatrix3x3 CCHR6dm::sendGetStateCovariance( const unsigned int nTimeoutMS )
{
    sendRawMessage( CHR6dm_SEND::GET_STATE_COVARIANCE, 0, 0 );

    SMatrix3x3 d32StateCovariance;

    boost::posix_time::ptime startTime( boost::posix_time::microsec_clock::local_time() );

    boost::posix_time::time_duration timeout = boost::posix_time::milliseconds( nTimeoutMS );

    m_bWaitingForResponse = true;

    while ( (( (boost::posix_time::microsec_clock::local_time()) - startTime ) < timeout) && m_bWaitingForResponse )
    {
        boost::this_thread::sleep( boost::posix_time::milliseconds(100) );
    }

    d32StateCovariance.entry11 = m_rgExchangeFloat.at(0);
    d32StateCovariance.entry12 = m_rgExchangeFloat.at(1);
    d32StateCovariance.entry13 = m_rgExchangeFloat.at(2);

    d32StateCovariance.entry21 = m_rgExchangeFloat.at(3);
    d32StateCovariance.entry22 = m_rgExchangeFloat.at(4);
    d32StateCovariance.entry23 = m_rgExchangeFloat.at(5);

    d32StateCovariance.entry31 = m_rgExchangeFloat.at(6);
    d32StateCovariance.entry32 = m_rgExchangeFloat.at(7);
    d32StateCovariance.entry33 = m_rgExchangeFloat.at(8);

#ifdef DBG_OUTPUT
    std::cout << "Duration: " << ((boost::posix_time::microsec_clock::local_time()) - startTime) << std::endl;
#endif

    return d32StateCovariance;
}

SMatrix3x3 CCHR6dm::sendGetGyroAlignment( const unsigned int nTimeoutMS )
{
    sendRawMessage( CHR6dm_SEND::GET_GYRO_ALIGNMENT, 0, 0 );

    SMatrix3x3 d32GyroAlignment;

    boost::posix_time::ptime startTime( boost::posix_time::microsec_clock::local_time() );

    boost::posix_time::time_duration timeout = boost::posix_time::milliseconds( nTimeoutMS );

    m_bWaitingForResponse = true;

    while ( (( (boost::posix_time::microsec_clock::local_time()) - startTime ) < timeout) && m_bWaitingForResponse )
    {
        boost::this_thread::sleep( boost::posix_time::milliseconds(100) );
    }

    d32GyroAlignment.entry11 = m_rgExchangeFloat.at(0);
    d32GyroAlignment.entry12 = m_rgExchangeFloat.at(1);
    d32GyroAlignment.entry13 = m_rgExchangeFloat.at(2);

    d32GyroAlignment.entry21 = m_rgExchangeFloat.at(3);
    d32GyroAlignment.entry22 = m_rgExchangeFloat.at(4);
    d32GyroAlignment.entry23 = m_rgExchangeFloat.at(5);

    d32GyroAlignment.entry31 = m_rgExchangeFloat.at(6);
    d32GyroAlignment.entry32 = m_rgExchangeFloat.at(7);
    d32GyroAlignment.entry33 = m_rgExchangeFloat.at(8);

#ifdef DBG_OUTPUT
    std::cout << "Duration: " << ((boost::posix_time::microsec_clock::local_time()) - startTime) << std::endl;
#endif

    return d32GyroAlignment;
}

SMatrix3x3 CCHR6dm::sendGetAccelAlignment( const unsigned int nTimeoutMS )
{
    sendRawMessage( CHR6dm_SEND::GET_ACCEL_ALIGNMENT, 0, 0 );

    SMatrix3x3 d32AccelAlignment;

    boost::posix_time::ptime startTime( boost::posix_time::microsec_clock::local_time() );

    boost::posix_time::time_duration timeout = boost::posix_time::milliseconds( nTimeoutMS );

    m_bWaitingForResponse = true;

    while ( (( (boost::posix_time::microsec_clock::local_time()) - startTime ) < timeout) && m_bWaitingForResponse )
    {
        boost::this_thread::sleep( boost::posix_time::milliseconds(100) );
    }

    d32AccelAlignment.entry11 = m_rgExchangeFloat.at(0);
    d32AccelAlignment.entry12 = m_rgExchangeFloat.at(1);
    d32AccelAlignment.entry13 = m_rgExchangeFloat.at(2);

    d32AccelAlignment.entry21 = m_rgExchangeFloat.at(3);
    d32AccelAlignment.entry22 = m_rgExchangeFloat.at(4);
    d32AccelAlignment.entry23 = m_rgExchangeFloat.at(5);

    d32AccelAlignment.entry31 = m_rgExchangeFloat.at(6);
    d32AccelAlignment.entry32 = m_rgExchangeFloat.at(7);
    d32AccelAlignment.entry33 = m_rgExchangeFloat.at(8);

#ifdef DBG_OUTPUT
    std::cout << "Duration: " << ((boost::posix_time::microsec_clock::local_time()) - startTime) << std::endl;
#endif

    return d32AccelAlignment;
}

SMatrix3x1short CCHR6dm::sendGetMagRefVector( const unsigned int nTimeoutMS )
{
    sendRawMessage( CHR6dm_SEND::GET_MAG_REF_VECTOR, 0, 0 );

    SMatrix3x1short i16MagRefVector;

    boost::posix_time::ptime startTime( boost::posix_time::microsec_clock::local_time() );

    boost::posix_time::time_duration timeout = boost::posix_time::milliseconds( nTimeoutMS );

    m_bWaitingForResponse = true;

    while ( (( (boost::posix_time::microsec_clock::local_time()) - startTime ) < timeout) && m_bWaitingForResponse )
    {
        boost::this_thread::sleep( boost::posix_time::milliseconds(100) );
    }

    i16MagRefVector.entry11 = m_rgExchangeShort.at(0);
    i16MagRefVector.entry21 = m_rgExchangeShort.at(1);
    i16MagRefVector.entry31 = m_rgExchangeShort.at(2);

#ifdef DBG_OUTPUT
    std::cout << "Duration: " << ((boost::posix_time::microsec_clock::local_time()) - startTime) << std::endl;
#endif

    return i16MagRefVector;
}

SMatrix3x3 CCHR6dm::sendGetMagCal( const unsigned int nTimeoutMS )
{
    sendRawMessage( CHR6dm_SEND::GET_MAG_CAL, 0, 0 );

    SMatrix3x3 d32MagCal;

    boost::posix_time::ptime startTime( boost::posix_time::microsec_clock::local_time() );

    boost::posix_time::time_duration timeout = boost::posix_time::milliseconds( nTimeoutMS );

    m_bWaitingForResponse = true;

    while ( (( (boost::posix_time::microsec_clock::local_time()) - startTime ) < timeout) && m_bWaitingForResponse )
    {
        boost::this_thread::sleep( boost::posix_time::milliseconds(100) );
    }

    d32MagCal.entry11 = m_rgExchangeFloat.at(0);
    d32MagCal.entry12 = m_rgExchangeFloat.at(1);
    d32MagCal.entry13 = m_rgExchangeFloat.at(2);

    d32MagCal.entry21 = m_rgExchangeFloat.at(3);
    d32MagCal.entry22 = m_rgExchangeFloat.at(4);
    d32MagCal.entry23 = m_rgExchangeFloat.at(5);

    d32MagCal.entry31 = m_rgExchangeFloat.at(6);
    d32MagCal.entry32 = m_rgExchangeFloat.at(7);
    d32MagCal.entry33 = m_rgExchangeFloat.at(8);

#ifdef DBG_OUTPUT
    std::cout << "Duration: " << ((boost::posix_time::microsec_clock::local_time()) - startTime) << std::endl;
#endif

    return d32MagCal;
}

SMatrix3x1short CCHR6dm::sendGetMagBias( const unsigned int nTimeoutMS )
{
    sendRawMessage( CHR6dm_SEND::GET_MAG_BIAS, 0, 0 );

    SMatrix3x1short i16MagBias;

    boost::posix_time::ptime startTime( boost::posix_time::microsec_clock::local_time() );

    boost::posix_time::time_duration timeout = boost::posix_time::milliseconds( nTimeoutMS );

    m_bWaitingForResponse = true;

    while ( (( (boost::posix_time::microsec_clock::local_time()) - startTime ) < timeout) && m_bWaitingForResponse )
    {
        boost::this_thread::sleep( boost::posix_time::milliseconds(100) );
    }

    i16MagBias.entry11 = m_rgExchangeShort.at(0);
    i16MagBias.entry21 = m_rgExchangeShort.at(1);
    i16MagBias.entry31 = m_rgExchangeShort.at(2);

#ifdef DBG_OUTPUT
    std::cout << "Duration: " << ((boost::posix_time::microsec_clock::local_time()) - startTime) << std::endl;
#endif

    return i16MagBias;
}
