// File:           StarGazer.cpp
// Creation Date:  Tuesday, September 29 2009
// Author:         Julius Ziegler <ziegler@mrt.uka.de>

#include <unistd.h>
#include <cmath>

#include <stdexcept>
#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <utility>

#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>

#include "StarGazer.hpp"
#include "serial_port.hpp"
#include "../Elementary/ThreadSafeLogging.h"

// #define USE_LIBSERIAL

StarGazer::StarGazer( std::string device ):
  ser_fd( initport( device ) )
{
  if( ser_fd < 0 )
    {
      throw( std::runtime_error( "cannot connect to serial port @ " + device ) );
    }

 // EOUT("Connected to StarGazer using device " << device << ".\n");

  stop_calc();
  //if (!stop_calc())
  //  throw std::runtime_error ("StarGazer::StarGazer: could not connect to StarGazer");

 // EOUT("Firmware version: " << read_parameter( "Version" ) << "\n");

}

StarGazer::~StarGazer () {
  stop_calc();
}

std::string StarGazer::read_parameter( const std::string parameter )
{
  send_command_string( "~@" +parameter + "`" );

  std::string response;
  response = get_string();

  if( response.find( '|' ) == std::string::npos )
    return "XXXXXX";

  // response looks like this: ~$Parameter|Value'.
  // Strip everything except Value.

  response = response.substr( response.find( '|' ) + 1 );
  response = response.substr( 0, response.size() - 1 );

  return response;
}

//template<class T> T StarGazer::read_parameter_as( const std::string parameter );

template<class T> void StarGazer::write_parameter( const std::string parameter, const T value )
{
  send_command_string( "~#" +parameter + "|" + boost::lexical_cast<std::string>( value ) + "`" );
  send_command_string( "~#SetEnd`" );

  acknowledge_parameter_update();
}

std::string StarGazer::command_string( const std::string command )
{
  return std::string( "~#" ) + command + "`";
}

void StarGazer::flush_stargazer_buffer()
{
  // read everything and discard it, until read timeouts
  std::string garbage = get_string();

  while( garbage != "" )
    {
      // EOUT("discarded: " << garbage << "\n");
      // EOUT(".");
      EOUT("garbage: " << garbage << "\n");
      garbage = get_string();
    }
}

void StarGazer::build_map( int no_markers, int ref_marker_id )
{
  std::vector<int> ids;

  stop_calc();

  write_parameter( "IDNum", no_markers );
  write_parameter( "RefID", ref_marker_id );
  write_parameter( "MarkMode", "Map" );

  // Documentation says an extra ~#CalcStart` is necessary here,
  // but following command triggers calculation by it self:

  send_command_string( "~#MapMode|Start`" );


  // these datastructures records the minimum spanning tree for verification and debugging.
  std::vector< std::pair<int, int> >  edges;
  std::map<int, PositionData>         id_map;

  PositionData parent;
  parent.id = 0;

  while( true ) {
    std::string str = get_string();

    EOUT("\v");

    PositionData pd = parse_position_msg( str );

    if( !pd.dead ) {
      EOUT("FIX: " << pd.id << "\n");
      id_map[pd.id] = pd;
      parent = pd;
    }
    else
      EOUT("NO FIX, last was " << parent.id << "\n");

    EOUT("\nGot these ids:\n");

    BOOST_FOREACH( int id, ids )
      EOUT(id << "\n");

    EOUT("\b\b\n");

    if( str.size() >= 5 && str.substr( 2, 5 ) == "MAPID" )
      {
        LOUT("cast: " << str.substr( 8, str.size() - 9 ) << "\n");
        int id = boost::lexical_cast<int>( str.substr( 8, str.size() - 9 ) );
        EOUT("\n" << "got ID " << id << "\n");
        ids.push_back( id );
        edges.push_back( std::make_pair( id, parent.id ) );
      }

    if( str == "~!MapDataSave`" )
      {
        EOUT("\nmap saved.\n");
        break;
      }
  }

  // output the spanning tree (gnuplot format)
  std::ofstream os( "spanning_tree" );
  std::pair<int,int> e;
  BOOST_FOREACH( e, edges )
    {
      LOUT(e.second << "->" << e.first << "\n");

      PositionData child  = id_map[ e.first ];
      PositionData parent = id_map[ e.second ];
      os << parent.x << " " << parent.y << "\n";
      os << child.x << " " << child.y << "\n";
      os << " \n \n";
    }

  stop_calc();
}

StarGazer::PositionData StarGazer::get_position()
{
  return parse_position_msg( get_string() );
}

double StarGazer::calc_height()
{
  stop_calc();

  send_command_string( command_string( "HeightCalc" ) );

  EOUT("calculating height...\n");

  while( true )
    {
      std::string msg = get_string();
      EOUT(".");
      if( msg == "~!ParameterUpdate`" )
        {
          break;
        }
    }

  EOUT("\n");

  stop_calc();

  int height_mm = read_parameter_as<int>( "MarkHeight" );



  return height_mm/1000.;

}


void StarGazer::start_calc()
{
  send_command_string( command_string( "CalcStart" ) );
}

bool StarGazer::stop_calc()
{
  // must repeat this until success (see manual)
 // EOUT("stop calc...\n");

  bool success = false;
  unsigned int iter=0;
  while( !success && iter++<5)
    {
      try {
 //       EOUT("sending CalcStop...\n");
        send_command_string( command_string( "CalcStop" ) );
        success = true;
      } catch( std::runtime_error e )
      {
        success = false;
        // make sure the buffer is empty:
        flush_stargazer_buffer();
      }
    }
    return success;
}

void StarGazer::send_command_string( const std::string str )
{
  std::string cpy = str;
  send_string( str );
  cpy[1] = '!';
  std::string ack;
  do {
    ack = get_string();
  } while (ack!=cpy && ack!="");
  if( !( cpy == ack ) )
    {
      throw( std::runtime_error( "command " + str + " did not return correct acknowledge " + cpy + " ( returned " + ack + " )" ) );
    }
}

/*
void StarGazer::send_command_string( const std::string str )
{
  EOUT("sending command string: " << str << "\n");

  std::string cpy = str;;
  cpy[1] = '!';
  send_string( str );

  std::string ack;
  do {
    ack = get_string();
    EOUT("ack: " << ack << "\n");
  }
  while( cpy != ack );
}
*/

void StarGazer::send_string( const std::string str )
{
  BOOST_FOREACH( char c, str )
    {
      if( write( ser_fd, &c, 1 ) != 1 )
        throw( std::runtime_error( "write failed!" ) );

      usleep( 50000 );
    }
}

/** reads until the end of sentence character: "`"**/
std::string StarGazer::get_string()
{
  std::string result;

  char c;

  do {

    // EOUT("read...\n");
    int no_read = read( ser_fd, &c, 1 );
    // EOUT("read " << no_read << "\n");
    // EOUT("...done.\n");
    if( no_read == 0 )
      return "";

    // EOUT("[" << (unsigned int)c << "]" << "\n");

    if( (unsigned int)(c) <= 32 || (unsigned int)(c) > 127 ) 
      return "";

    result.push_back( c );
  } while( c != '`' && result.size() < 40 );

  // EOUT("got string: " << result <<"\n");

  return result;
}

void StarGazer::acknowledge_parameter_update()
{
  EOUT("wait for parameter update acknowledge (this may take some seconds)...\n");
  std::string str = get_string();

  while( str == "" ) 
    {
      EOUT(".");
      str = get_string();
    }

  EOUT("\n");

  if( str != "~!ParameterUpdate`" )
    throw( std::runtime_error( std::string( "did not receive expected message: ~!ParameterUpdate` after writing parameter; received " ) + str) );
}

std::string StarGazer::consume_token( std::string& str )
{
  size_t idx = str.find( '|' );
  if( idx == std::string::npos )
    return "XXXXXXX";

  //  EOUT("consume_token, idx: " << idx << "\n");
  std::string token = str.substr( 0, idx );
  str = str.substr( idx + 1 );
  return token;
}

StarGazer::PositionData StarGazer::parse_position_msg( std::string str )
{
  PositionData result;

  try {

    if( str == "~*DeadZone`" )
      {
        result.dead = true;
        return result;
      }

    if( str.size() < 5 ) // read timeout?
      {
        result.dead = true;
        return result;
      }

    str = str.substr( 3 );

    // primitive tokenizer expects | termination
    str[ str.size() - 1 ] = '|';

    result.id    =  boost::lexical_cast<int>( consume_token( str ) );

    // stargazer inverts angle!
    result.theta = -boost::lexical_cast<double>( consume_token( str ) );

    result.x = boost::lexical_cast<double>( consume_token( str ) );
    result.y = boost::lexical_cast<double>( consume_token( str ) );
    result.z = boost::lexical_cast<double>( consume_token( str ) );

    result.x *= 0.01;
    result.y *= 0.01;
    result.z *= 0.01;

    result.theta *= M_PI/180.;

    return result;
  }
  catch( std::exception e )
    {
      EOUT("StarGazer::parse_position_msg(): caught error: " << e.what() << "\n");
      result.dead = true;
      return result;
    }
}
