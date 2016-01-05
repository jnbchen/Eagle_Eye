/** Entwickelt im Rahmen des Brainstormers Tribots Projektes durch Martin
 Lauer (lauer@mrt.uka.de) und getestet unter Linux */

#ifndef _DerWeg_PriorityUDPCommunication_h_
#define _DerWeg_PriorityUDPCommunication_h_

#include "MultiPacketUDPCommunication.h"
#include <deque>

namespace DerWeg {

  /** UDP-Verbindung, die Nachrichten in internen Puffern nach Prioritaeten
      getrennt sammelt und verschickt, Behandelt auch ueberlange Nachrichten,
      die in mehreren Paketen verschickt werden */
  class PriorityUDPCommunication {
  public:
    /** Konstruktor; Arg1: zwei Markierungsbytes, Arg2: Maximalgroesse einer Nachricht in Byte */
    PriorityUDPCommunication (const char*, unsigned int =8189) throw (std::bad_alloc);
    virtual ~PriorityUDPCommunication () throw ();
    /** als Server an Port arg1 initialisieren; Rueckgabewert: erfolgreich? */
    virtual bool init_as_server (int) throw ();
    /** als Client initialisieren; Verbindung mit Rechner arg1 an Port arg2; Rueckgabewert: erfolgreich? */
    virtual bool init_as_client (const char*, int) throw ();
    /** Verbindung schliessen */
    virtual void close () throw ();

    /** Wie lange in Byte darf eine Nachricht maximal sein? */
    virtual unsigned int max_message_length () const throw ();

    /** Alle Pakete senden;
        Return: Anzahl gesendeter Pakete */
    virtual unsigned int send () throw ();
    /** Alle Pakete an bestimmten Empfaenger (Arg1) senden;
        Return: Anzahl gesendeter Pakete */
    virtual unsigned int sendto (const struct sockaddr_in&) throw ();
    /** Pakete des naechsten Senders empfangen;
        Return: Anzahl fehlerfrei empfangener Pakete */
    virtual unsigned int receive () throw (std::bad_alloc);
    /** Pakete eines Senders (Arg1) empfangen;
        Return: Anzahl fehlerfrei empfangener Pakete */
    virtual unsigned int receivefrom (const struct sockaddr_in&) throw (std::bad_alloc);

    /** eine Nachricht hinzufuegen;
        Arg1: die Nachricht,
        Arg2: Laenge der Nachricht in Byte,
        Arg3: Prioritaet der Nachricht,
        Return: konnte Nachricht hinzugefuegt werden? */
    virtual bool put (const char*, unsigned int, unsigned int =0) throw (std::bad_alloc);
    /** den Ausgabepuffer loeschen; wird nach "send" auch automatisch aufgerufen */
    virtual void clear_send_buffer () throw ();
    /** den Eingabepuffer loeschen; wird vor "receive" auch automatisch aufgerufen */
    virtual void clear_receive_buffer () throw ();
    /** die naechste empfange Nachricht holen 
        Arg1: Rueckgabewert: ein Zeiger auf den Puffer,
        Arg2: Rueckgabewert: die Laenge der Nachricht in Byte,
        Return: lag noch ein Puffer vor?
        Beachte: mit jedem "get"-Aufruf wird die nachfolgende Nachricht abgerufen, 
        "receive" setzt auf die erste empfange Nachricht zurueck */
    virtual bool get (const char*&, unsigned int&) throw ();

    /** Adresse des letzten Kommunikationspartners */
    virtual const struct sockaddr_in& partner_address () const throw ();
    /** true, wenn init_as_client oder init_as_server erfolgreich aufgerufen wurde */
    virtual bool started() const throw ();
    /** liefert true, wenn als Server gestartet */
    virtual bool as_server() const throw ();

    /** berechnet die durchschnittliche Kommunikationsbelastung durch ausgehende Pakete in den letzten maximal (arg1) Sekunden */
    virtual UDPCommunicationStatistics send_load (int =10) const throw ();
    /** berechnet die durchschnittliche Kommunikationsbelastung durch eingehende Pakete in den letzten maximal (arg1) Sekunden */
    virtual UDPCommunicationStatistics receive_load (int =10) const throw ();

  protected:
    struct Message {
      unsigned int length;
      char* buffer;
      unsigned int message_id;
      unsigned int junk_length;
      std::vector<bool> junks_available;
      
      bool complete ();
    };
    void create_memessage_list();
    MultiPacketUDPCommunication socket;
    char magic_byte [2];

    const unsigned int max_buffer_size;

    std::vector<char*> send_buffers;
    std::vector<unsigned int> send_buffer_length;
    std::vector<unsigned int> send_buffer_priorities;
    
    unsigned int long_message_id;
    std::vector<char*> send_buffers_long;
    std::vector<unsigned int> send_buffer_length_long;
    
    std::vector<char*> receive_buffers;
    std::vector<unsigned int> receive_buffer_length;
    std::deque<Message> receive_messages;  ///< Liste mit Verweisen auf die zuletzt empfangenen Nachrichten
    unsigned int get_message;  ///< kleinste Nachrichten-Nummer, die noch nicht mit get() ausgelesen wurde
    
    std::deque<Message> receive_buffers_long;
  };

} // namespace DerWeg

#endif
