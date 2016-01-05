#pragma once
#ifndef CSERIAL_H
#define CSERIAL_H

#include <vector>

#include <boost/utility.hpp>
#include <boost/asio.hpp>
#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <boost/shared_array.hpp>

namespace CHR6dm {

  class CSerial : private boost::noncopyable
  {
  public:
      CSerial( void );
      ~CSerial( void );
      void open( const std::string& stDeviceName,
                unsigned int uiBaudRate = 115200,
                boost::asio::serial_port_base::parity optParity = boost::asio::serial_port_base::parity( boost::asio::serial_port_base::parity::none ),
                boost::asio::serial_port_base::character_size optCSize = boost::asio::serial_port_base::character_size( 8 ),
                boost::asio::serial_port_base::flow_control optFlow = boost::asio::serial_port_base::flow_control( boost::asio::serial_port_base::flow_control::none ),
                boost::asio::serial_port_base::stop_bits optStopBits = boost::asio::serial_port_base::stop_bits( boost::asio::serial_port_base::stop_bits::one )
                );
      void close ( void );
      bool isOpen( void ) const;
      bool getErrorStatus( void ) const;
      void write( const unsigned char *data, size_t size );
      void write( const std::vector<unsigned char>& data );

  private:
      void doRead( void );
      void readEnd( const boost::system::error_code& boostErrorCode, size_t bytes_transferred );
      void doWrite( void );
      void writeEnd( const boost::system::error_code& boostErrorCode );
      void doClose( void );

  public:
      static const unsigned int s_nReadBufferSize = 512;
      unsigned char m_rgChReadBuffer[ CSerial::s_nReadBufferSize ];

  private:
      boost::asio::io_service m_asioIO;
      boost::asio::serial_port m_asioSerialPort;
      boost::thread m_boostBackgroundThread;
      bool m_bOpen;
      bool m_bError;
      mutable boost::mutex m_boostErrorMutex;
      boost::mutex m_boostWriteMutex;
      std::vector<unsigned char> m_rgChWriteQueueBuffer;
      boost::shared_array<unsigned char> m_rgChWriteBuffer;
      size_t m_cWriteBufferSize;
      boost::function<void ( const unsigned char*, size_t )> m_boostCallback;

  protected:
      void setErrorStatus( bool bError );
      void setReadCallback( const boost::function<void ( const unsigned char*, size_t )>& fnCallback );
      void clearReadCallback( void );
  };

}

#endif // CSERIAL_H
