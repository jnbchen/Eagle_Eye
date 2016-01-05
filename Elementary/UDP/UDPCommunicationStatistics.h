/** Entwickelt im Rahmen des Brainstormers Tribots Projektes durch Martin
 Lauer (lauer@mrt.uka.de) und getestet unter Linux */

#ifndef _DerWeg_UDPCommunicationStatistics_h_
#define _DerWeg_UDPCommunicationStatistics_h_

#ifdef WIN32
 #include <winsock2.h>
#else
 #include <netinet/in.h>
#endif
#include "../Timestamp.h"

namespace DerWeg {

  /** Struktur, um Informationen ueber ein ein- oder ausgehendes Paket zu speichern */
  struct UDPPacketStatistics {
    unsigned int size;  ///< Paketgroesse in Byte
    Timestamp timestamp;  ///< Zeitpunkt des Sendens/Empfangens (send/receive-Kommandos)
    struct sockaddr_in partner_address;  ///< Adresse des Empfaengers/Absenders
  };

  /** Struktur, um Informationen ueber den Kommunikations-Verkehr zu speichern */
  struct UDPCommunicationStatistics {
    double packet_rate;  ///< mittlere Anzahl gesendeter/empfangener Pakete pro Sekunde
    double packet_size;  ///< mittlere Packetgroesse der zuletzt gesendeten/empfangenen Pakete in Byte
  };

} // namespace DerWeg

#endif
