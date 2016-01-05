
#include "DialogModuleSelection.h"
#include <algorithm>

using namespace std;
using namespace DerWeg;


DerWeg::DialogModuleSelection::DialogModuleSelection(QWidget* p, Qt::WindowFlags f) : QDialog (p,f)
{
  setupUi(this);

  connect (pushButtonSelect,SIGNAL(clicked()),this,SLOT(selectClicked()));
  connect (pushButtonUnselect,SIGNAL(clicked()),this,SLOT(unselectClicked()));
  connect (pushButtonClose, SIGNAL(clicked()),this,SLOT(hide()));
}


DialogModuleSelection::~DialogModuleSelection()
{
}


void DerWeg::DialogModuleSelection::setAllModules (const std::vector<std::string>& m) {
  bool changed=(m.size()!=allModules.size());
  if (!changed) {
    for (unsigned int i=0; i<allModules.size(); ++i) {
      if (m[i]!=allModules[i]) {
        changed=true;
        break;
      }
    }
  }
  if (changed) {
    allModules=m;
    updateLists();
  }
}


void DerWeg::DialogModuleSelection::setPresentModules (const std::vector<std::string>& m) {
  bool changed=(m.size()!=selectedModules.size());
  if (!changed) {
    for (unsigned int i=0; i<selectedModules.size(); ++i) {
      if (m[i]!=selectedModules[i]) {
        changed=true;
        break;
      }
    }
  }
  if (changed) {
    selectedModules=m;
    updateLists();
  }
}


void DerWeg::DialogModuleSelection::updateLists() {
  vector<string> availableModules;
  for (unsigned int i=0; i<allModules.size(); i++) {
    if (std::find (selectedModules.begin(), selectedModules.end(), allModules[i])==selectedModules.end()) {
      availableModules.push_back (allModules[i]);
    }
  }

  listWidgetAvailable->clear();
  listWidgetSelected->clear();
  
  for (unsigned int i=0; i<availableModules.size(); i++)
    listWidgetAvailable->addItem (availableModules[i].c_str());
  for (unsigned int i=0; i<selectedModules.size(); i++)
    listWidgetSelected->addItem (selectedModules[i].c_str());
}


void DerWeg::DialogModuleSelection::selectClicked () {
  if (listWidgetAvailable->currentItem ()) {
    string sel = listWidgetAvailable->currentItem ()->text().toAscii().constData();
    emit (addModule (sel));
  }
}


void DerWeg::DialogModuleSelection::unselectClicked () {
  if (listWidgetSelected->currentItem ()) {
    string sel = listWidgetSelected->currentItem ()->text().toAscii().constData();
    emit (removeModule (sel));
  }
}
