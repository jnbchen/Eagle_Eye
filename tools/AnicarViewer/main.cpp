
#include <QtGui/QApplication>
#include <QtGui/QMessageBox>
#include "AnicarViewer.h"

using namespace std;
using namespace DerWeg;

int main(int argc, char *argv[])
{
  DerWeg::ConfigReader cfg;
  cfg.add_command_line_shortcut ("h", "help", true);
  cfg.add_command_line_shortcut ("l", "logfile", true);
  cfg.add_command_line_shortcut ("n", "nologging", false);
  cfg.add_command_line_shortcut ("b", "background", true);
  cfg.add_command_line_shortcut ("d", "dimension", true);
  cfg.add_command_line_shortcut ("m", ",max_history_length", true);
  cfg.add_command_line_shortcut ("t", "trace", false);
  cfg.append_from_command_line (argc, argv);
  QApplication app(argc, argv);

  DerWeg::AnicarViewer window (cfg);

  window.show();
  return app.exec();
}
