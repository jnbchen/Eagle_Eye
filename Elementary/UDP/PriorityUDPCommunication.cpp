
#include "PriorityUDPCommunication.h"
#include <cstring>

using namespace DerWeg;
using namespace std;


bool PriorityUDPCommunication::Message::complete () {
  for (unsigned int i=0; i<junks_available.size(); ++i)
    if (!junks_available[i]) {
      return false;
    }
  return true;
}


PriorityUDPCommunication::PriorityUDPCommunication (const char* mb, unsigned int ml) throw (std::bad_alloc) : max_buffer_size (ml+2), send_buffers (0), send_buffer_length (0), send_buffer_priorities (0), long_message_id(0), receive_buffers (0), receive_buffer_length (0), receive_messages (0), get_message (0) {
  magic_byte[0]=mb[0];
  magic_byte[1]=mb[1];
}


PriorityUDPCommunication::~PriorityUDPCommunication () throw () {
  clear_send_buffer ();
  clear_receive_buffer ();
}


void PriorityUDPCommunication::clear_receive_buffer () throw () {
  for (unsigned int i=0; i<receive_buffers.size(); i++)
    delete [] receive_buffers[i];
  receive_buffers.clear();
  receive_buffer_length.clear();
  receive_messages.clear();
  unsigned int i=0;
  while (i<receive_buffers_long.size()) {
    if (receive_buffers_long[i].complete())
      receive_buffers_long.erase (receive_buffers_long.begin()+i);
    else
      ++i;
  }
  get_message=0;
}


void PriorityUDPCommunication::clear_send_buffer () throw () {
  for (unsigned int i=0; i<send_buffers.size(); i++)
    delete [] send_buffers[i];
  for (unsigned int i=0; i<send_buffers_long.size(); i++)
    delete [] send_buffers_long[i];
  send_buffers.clear();
  send_buffer_length.clear();
  send_buffer_priorities.clear();
  send_buffers_long.clear();
  send_buffer_length_long.clear();
}


unsigned int PriorityUDPCommunication::max_message_length () const throw () {
  return max_buffer_size-5;
}


unsigned int PriorityUDPCommunication::send () throw () {
  unsigned int res1 = socket.send (send_buffers, send_buffer_length);
  unsigned int res2 = socket.send (send_buffers_long, send_buffer_length_long);
  clear_send_buffer ();
  return res1+res2;
}


unsigned int PriorityUDPCommunication::sendto (const struct sockaddr_in& add) throw () {
  unsigned int res1 = socket.sendto (add, send_buffers, send_buffer_length);
  unsigned int res2 = socket.sendto (add, send_buffers_long, send_buffer_length_long);
  clear_send_buffer ();
  return res1+res2;
}


unsigned int PriorityUDPCommunication::receive () throw (std::bad_alloc) {
  clear_receive_buffer ();
  unsigned int res = socket.receive (receive_buffers, receive_buffer_length, max_buffer_size);
  create_memessage_list();
  return res;
}


unsigned int PriorityUDPCommunication::receivefrom (const struct sockaddr_in& add) throw (std::bad_alloc) {
  clear_receive_buffer ();
  unsigned int res = socket.receivefrom (add, receive_buffers, receive_buffer_length, max_buffer_size);
  create_memessage_list();
  return res;
}


void PriorityUDPCommunication::create_memessage_list() {
  for (unsigned int i=0; i<receive_buffers.size(); ++i) {
    unsigned int buflen = receive_buffer_length[i];
    char* ptr = receive_buffers[i];  // an den Anfang des Pakets springen
    if (buflen<3 || ptr[0]!=magic_byte[0] || ptr[1]!=magic_byte[1])
      continue;  // Puffer ueberspringen, da keine oder falsche magic_bytes
    unsigned int packet_type=ptr[2];
    char* ptr_end=ptr+buflen;
    ptr+=3;  // hinter magic_bytes und Pakettyp springen
    if (packet_type==1) {
      // Paket mit zerlegter langer Nachricht
      if (ptr_end-ptr<8)
        continue;  // Header unvollstaendig
      unsigned int message_id = (static_cast<unsigned int>(ptr[0])<<8)+static_cast<unsigned int>(ptr[1]);
      ptr+=2;
      unsigned int num_junks = (static_cast<unsigned int>(ptr[0])<<8)+static_cast<unsigned int>(ptr[1]);
      ptr+=2;
      unsigned int junk_num = (static_cast<unsigned int>(ptr[0])<<8)+static_cast<unsigned int>(ptr[1]);
      ptr+=2;
      unsigned int junk_len = (static_cast<unsigned int>(ptr[0])<<8)+static_cast<unsigned int>(ptr[1]);
      ptr+=2;
      unsigned int this_junk_len=junk_len;
      if (junk_num+1==num_junks)
        this_junk_len=buflen-3-8;
      unsigned int bufindex = 0;
      while (bufindex<receive_buffers_long.size()) {
        if (receive_buffers_long[bufindex].message_id==message_id)
          break;
        ++bufindex;
      }
      if (bufindex==receive_buffers_long.size()) {
        Message new_message;
        new_message.length=num_junks*junk_len;
        new_message.buffer=new char[new_message.length];
        new_message.junks_available.resize (num_junks,false);
        new_message.junk_length=junk_len;
        new_message.message_id=message_id;
        receive_buffers_long.push_back (new_message);
      }
      memcpy (receive_buffers_long[bufindex].buffer+junk_num*junk_len, ptr, this_junk_len);
      receive_buffers_long[bufindex].junks_available[junk_num]=true;
      receive_buffers_long[bufindex].length-=junk_len-this_junk_len;
      if (receive_buffers_long[bufindex].complete()) {
        receive_messages.push_back (receive_buffers_long[bufindex]);
      }

      continue;
    }

    while (ptr+1<ptr_end) {
      // Nachricht fuer Nachricht lesen
      // Nachrichtenlaenge ermitteln
      unsigned char low = static_cast<unsigned char>(*ptr++);
      unsigned int message_length = low & 127;
      if (low & 128) {
        if (ptr>=ptr_end)
          break;  // 2. Byte nicht gefunden; ueberspringe Nachricht
        unsigned char high = static_cast<unsigned char>(*ptr++);
        message_length = (static_cast<unsigned int>(high)<<7)+static_cast<unsigned int>(low&127);
      }
      // Nachricht lesen
      if (ptr+message_length<=ptr_end) {
        // Nachricht vollstaendig im Puffer
        Message new_message;
        new_message.length=message_length;
        new_message.buffer=ptr;
        receive_messages.push_back (new_message);
      } else {
        // Nachricht nicht vollstaendig im Puffer, daher ignorieren
      }
      ptr+=message_length;
    }
  }
  get_message=0;
}


bool PriorityUDPCommunication::put (const char* msg, unsigned int msg_len, unsigned int prio) throw (std::bad_alloc) {
  // Nachrichtenformat:  LHN oder LN, je nach Lanege der Nachricht.
  // N=Nutzdaten, d.h. *msg, L und H codieren die Nachrichtenlaenge.
  // fuer Nachrichten <128 Byte wird Format LN verwendet, wobei L die Nachrichtenlaenge angibt (1Byte)
  // fuer Nachrichten >=128 Byte und < 32768 Byte wird Format LHN verwendet, wobei:
  //   L Bits 0-6 codieren die Nachrichtenlaenge modulo 2^7
  //   L Bit 7 ist gesetzt
  //   H Bits 0-7 codieren die Nachrichtenlaenge geteilt 2^7
  // d.h. H und L-Bits0-6 codieren die Nachrichtenlaenge binaer, L Bit7 gibt an, ob Laenge mit ein oder zwei Bytes codiert wird

  if (msg_len+5>max_buffer_size) {
    // lange Nachricht, diese in Teile zerlegen
    unsigned int junk_len=max_buffer_size-3-8;
    unsigned int num_junks=msg_len/junk_len+(msg_len%junk_len>0 ? 1 : 0);
    for (unsigned int j=0; j<num_junks; ++j) {
      unsigned int bufsz = (j+1==num_junks && msg_len%junk_len>0 ? msg_len%junk_len : junk_len);
      send_buffers_long.push_back (new char [bufsz+3+8]);
      send_buffer_length_long.push_back (bufsz+3+8);
      send_buffers_long [send_buffers_long.size()-1][0]=magic_byte[0];
      send_buffers_long [send_buffers_long.size()-1][1]=magic_byte[1];
      send_buffers_long [send_buffers_long.size()-1][2]=1;
      send_buffers_long [send_buffers_long.size()-1][3]=(long_message_id & 0xFF00)>>8;
      send_buffers_long [send_buffers_long.size()-1][4]=(long_message_id & 0xFF);
      send_buffers_long [send_buffers_long.size()-1][5]=(num_junks & 0xFF00)>>8;
      send_buffers_long [send_buffers_long.size()-1][6]=(num_junks & 0xFF);
      send_buffers_long [send_buffers_long.size()-1][7]=(j & 0xFF00)>>8;
      send_buffers_long [send_buffers_long.size()-1][8]=(j & 0xFF);
      send_buffers_long [send_buffers_long.size()-1][9]=(junk_len & 0xFF00)>>8;
      send_buffers_long [send_buffers_long.size()-1][10]=(junk_len & 0xFF);
      memcpy (send_buffers_long [send_buffers_long.size()-1]+3+8, msg+j*junk_len, bufsz);
    }
    long_message_id = (long_message_id + 1) & 0xFFFF;
    return true;
  }

  // kurze Nachricht
  // Codierung der Nachrichtengroesse
  unsigned int len_code_len = (msg_len>127 ? 2 : 1);
  unsigned char low = 0, high = 0;
  if (len_code_len==1)
    low = static_cast<unsigned char>(msg_len);
  else {
    high = static_cast<unsigned char>(msg_len>>7);
    low = 128 | static_cast<unsigned char>(msg_len%128);
  }

  // freien Puffer passender Prioritaet suchen
  unsigned target_buffer=static_cast<unsigned int>(send_buffers.size());
  for (unsigned int i=0; i<send_buffers.size(); i++)
    if (send_buffer_priorities[i]==prio && send_buffer_length[i]+len_code_len+msg_len<=max_buffer_size) {
      target_buffer=i;
      break;
    }
  if (target_buffer==send_buffers.size()) {
    send_buffers.push_back (new char [max_buffer_size]);
    send_buffers[send_buffers.size()-1][0]=magic_byte[0];  // Pakete markieren
    send_buffers[send_buffers.size()-1][1]=magic_byte[1];
    send_buffers[send_buffers.size()-1][2]=0;  // Puffer mit mhreren kleinen Nachrichten
    send_buffer_length.push_back (3);
    send_buffer_priorities.push_back (prio);
  }

  // in Target buffer schreiben
  (*(send_buffers[target_buffer]+send_buffer_length[target_buffer]))=static_cast<char>(low);
  if (len_code_len==2)
    (*(send_buffers[target_buffer]+send_buffer_length[target_buffer]+1))=static_cast<char>(high);
  memcpy ((send_buffers[target_buffer]+send_buffer_length[target_buffer]+len_code_len), msg, msg_len);
  send_buffer_length[target_buffer]+=len_code_len+msg_len;
  return true;
}


bool PriorityUDPCommunication::get (const char*& mb, unsigned int& ml) throw () {
  if (get_message>=receive_messages.size()) {
    // keine Nachricht mehr da
    mb = NULL;
    ml = 0;
    return false;
  }

  mb = receive_messages[get_message].buffer;
  ml = receive_messages[get_message].length;
  ++get_message;
  return true;
}

bool PriorityUDPCommunication::init_as_server (int port) throw () {
  return socket.init_as_server (port);
}

bool PriorityUDPCommunication::init_as_client (const char* host, int port) throw () {
  return socket.init_as_client (host, port);
}

void PriorityUDPCommunication::close () throw () {
  return socket.close();
}

const struct sockaddr_in& PriorityUDPCommunication::partner_address () const throw () {
  return socket.partner_address ();
}

bool PriorityUDPCommunication::started() const throw () {
  return socket.started();
}

bool PriorityUDPCommunication::as_server() const throw () {
  return socket.as_server();
}

UDPCommunicationStatistics PriorityUDPCommunication::send_load (int dt) const throw () {
  return socket.send_load(dt);
}

UDPCommunicationStatistics PriorityUDPCommunication::receive_load (int dt) const throw () {
  return socket.receive_load(dt);
}
