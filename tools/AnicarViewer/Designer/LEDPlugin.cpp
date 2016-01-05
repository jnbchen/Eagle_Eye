
#include "../LED.h"
#include "LEDPlugin.h"

#include <QtCore/QtPlugin>

LEDPlugin::LEDPlugin(QObject *parent) : QObject(parent), initialized(false) {;}

QWidget *LEDPlugin::createWidget(QWidget *parent) {
  return new DerWeg::LED(parent);
}

QString LEDPlugin::name() const {
  return "DerWeg::LED";
}

QString LEDPlugin::group() const {
  return "DerWeg Widgets";
}

QIcon LEDPlugin::icon() const {
  return QIcon();
}

QString LEDPlugin::toolTip() const {
  return "LED";
}

QString LEDPlugin::whatsThis() const {
  return "ein Widget, um die Fahrzeugposition anzuzeigen";
}

bool LEDPlugin::isContainer() const {
  return false;
}

QString LEDPlugin::domXml() const {
  return "<widget class=\"DerWeg::LED\" name=\"LED\">\n"
            " <property name=\"geometry\">\n"
            "  <rect>\n"
            "   <x>0</x>\n"
            "   <y>0</y>\n"
            "   <width>40</width>\n"
            "   <height>40</height>\n"
            "  </rect>\n"
            " </property>\n"
            "</widget>\n";
}

QString LEDPlugin::includeFile() const {
  return "LED.h";
}

void LEDPlugin::initialize(QDesignerFormEditorInterface * /* core */) {
  if (initialized)
    return;
  initialized = true;
}

bool LEDPlugin::isInitialized() const {
  return initialized;
}

Q_EXPORT_PLUGIN2(customwidgetplugin, LEDPlugin)
