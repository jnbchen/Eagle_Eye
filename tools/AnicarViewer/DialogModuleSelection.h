
#ifndef _DerWeg_DialogModuleSelection_h_
#define _DerWeg_DialogModuleSelection_h_

#include "Ui_DialogModuleSelection.h"
#include <QtGui/QDialog>
#include <vector>
#include <string>


namespace DerWeg {

  class DialogModuleSelection : public QDialog, private Ui::DialogModuleSelection {
    Q_OBJECT

  public:
    DialogModuleSelection(QWidget* =0, Qt::WindowFlags =0);
    ~DialogModuleSelection ();

  public slots:
    void setAllModules (const std::vector<std::string>&);
    void setPresentModules (const std::vector<std::string>&);

  signals:
    void addModule (std::string);
    void removeModule (std::string);

  private slots:
    void selectClicked ();
    void unselectClicked ();
    void updateLists ();

  private:
    std::vector<std::string> allModules;
    std::vector<std::string> selectedModules;
  };

}

#endif
