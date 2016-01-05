#include "CSerial.h"

using namespace CHR6dm;

CSerial::CSerial( void ) :
    m_asioIO(),
    m_asioSerialPort( m_asioIO ),
    m_boostBackgroundThread(),
    m_bOpen( false ),
    m_bError( false )
{
}

CSerial::~CSerial( void )
{
    if ( isOpen() )
    {
        try
        {
            close();
        }
        catch (...)
        {

        }
    }
}

void CSerial::open( const std::string& stDeviceName,
                    unsigned int uiBaudRate,
                    boost::asio::serial_port_base::parity optParity,
                    boost::asio::serial_port_base::character_size optCSize,
                    boost::asio::serial_port_base::flow_control optFlow,
                    boost::asio::serial_port_base::stop_bits optStopBits )
{
    if ( isOpen() )
    {
        close();
    }

    setErrorStatus( true );

    m_asioSerialPort.open( stDeviceName );
    m_asioSerialPort.set_option( boost::asio::serial_port_base::baud_rate( uiBaudRate ) );
    m_asioSerialPort.set_option( optParity );
    m_asioSerialPort.set_option( optCSize );
    m_asioSerialPort.set_option( optFlow );
    m_asioSerialPort.set_option( optStopBits );

    m_asioIO.post( boost::bind( &CSerial::doRead, this ) );

    boost::thread boostThread( boost::bind( &boost::asio::io_service::run, &m_asioIO ) );
    m_boostBackgroundThread.swap( boostThread );

    setErrorStatus( false );
    m_bOpen = true;
}

void CSerial::close( void )
{
    if ( !isOpen() )
    {
        return;
    }

    m_bOpen = false;
    m_asioIO.post( boost::bind( &CSerial::doClose, this ) );
    m_boostBackgroundThread.join();
    m_asioIO.reset();

    if ( getErrorStatus() )
        throw( boost::system::system_error( boost::system::error_code(), "Error while closing device" ) );
}

bool CSerial::isOpen( void ) const
{
    return m_bOpen;
}

bool CSerial::getErrorStatus( void ) const
{
    boost::lock_guard<boost::mutex> lg( m_boostErrorMutex );
    return m_bError;
}

void CSerial::write( const unsigned char *data, size_t size )
{
    boost::lock_guard<boost::mutex> lg( m_boostWriteMutex );

    m_rgChWriteQueueBuffer.insert( m_rgChWriteQueueBuffer.end(), data, data+size );
    m_asioIO.post( boost::bind( &CSerial::doWrite, this ) );
}

void CSerial::write( const std::vector<unsigned char>& data )
{
   boost::lock_guard<boost::mutex> lg( m_boostWriteMutex );

   m_rgChWriteQueueBuffer.insert( m_rgChWriteQueueBuffer.end(), data.begin(), data.end() );
   m_asioIO.post( boost::bind( &CSerial::doWrite, this ) );
}

void CSerial::doRead( void )
{
    m_asioSerialPort.async_read_some(
                boost::asio::buffer( m_rgChReadBuffer, s_nReadBufferSize ),
                boost::bind( &CSerial::readEnd, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred )
                );
}

void CSerial::readEnd( const boost::system::error_code& boostErrorCode, size_t bytes_transferred )
{
    if ( boostErrorCode )
    {
        if ( isOpen() )
        {
            setErrorStatus( true );
            doClose();
        }
    }
    else
    {
        if ( m_boostCallback )
            m_boostCallback( m_rgChReadBuffer, bytes_transferred );

        doRead();
    }
}

void CSerial::doWrite( void )
{
    // if a write operation is already in progress, do nothing
    if( m_rgChWriteBuffer == 0 )
    {
        boost::lock_guard<boost::mutex> lg( m_boostWriteMutex );

        m_cWriteBufferSize = m_rgChWriteQueueBuffer.size();
        m_rgChWriteBuffer.reset( new unsigned char[ m_cWriteBufferSize ] );
        std::copy( m_rgChWriteQueueBuffer.begin(), m_rgChWriteQueueBuffer.end(), m_rgChWriteBuffer.get() );
        m_rgChWriteQueueBuffer.clear();

        async_write(
                    m_asioSerialPort,
                    boost::asio::buffer( m_rgChWriteBuffer.get(), m_cWriteBufferSize ),
                    boost::bind( &CSerial::writeEnd, this, boost::asio::placeholders::error )
                    );
    }
}

void CSerial::writeEnd( const boost::system::error_code& boostErrorCode )
{
    if ( boostErrorCode )
    {
        setErrorStatus( true );
        doClose();
    }
    else
    {
        boost::lock_guard<boost::mutex> lg( m_boostWriteMutex );

        if( m_rgChWriteQueueBuffer.empty() )
        {
            m_rgChWriteBuffer.reset();
            m_cWriteBufferSize = 0;

            return;
        }

        m_cWriteBufferSize = m_rgChWriteQueueBuffer.size();
        m_rgChWriteBuffer.reset( new unsigned char[ m_cWriteBufferSize ] );
        std::copy( m_rgChWriteQueueBuffer.begin(), m_rgChWriteQueueBuffer.end(), m_rgChWriteBuffer.get() );
        m_rgChWriteQueueBuffer.clear();

        async_write(
                    m_asioSerialPort,
                    boost::asio::buffer( m_rgChWriteBuffer.get(), m_cWriteBufferSize ),
                    boost::bind( &CSerial::writeEnd, this, boost::asio::placeholders::error )
                    );
    }
}

void CSerial::doClose( void )
{
    boost::system::error_code boostErrorCode;

    m_asioSerialPort.cancel( boostErrorCode );

    if ( boostErrorCode )
    {
        setErrorStatus( true );
    }

    m_asioSerialPort.close( boostErrorCode );

    if ( boostErrorCode )
    {
        setErrorStatus( true );
    }
}

void CSerial::setErrorStatus( bool bError )
{
    boost::lock_guard<boost::mutex> lg( m_boostErrorMutex );
    m_bError = bError;
}

void CSerial::setReadCallback( const boost::function<void ( const unsigned char*, size_t )>& fnCallback )
{
    m_boostCallback = fnCallback;
}

void CSerial::clearReadCallback( void )
{
    m_boostCallback.clear();
}
