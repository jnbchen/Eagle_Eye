
#include "../MapWidget.h"
#include "MapWidgetPlugin.h"

#include <QtCore/QtPlugin>

MapWidgetPlugin::MapWidgetPlugin(QObject *parent) : QObject(parent), initialized(false) {;}

QWidget *MapWidgetPlugin::createWidget(QWidget *parent) {
  return new DerWeg::MapWidget(parent);
}

QString MapWidgetPlugin::name() const {
  return "DerWeg::MapWidget";
}

QString MapWidgetPlugin::group() const {
  return "DerWeg Widgets";
}

QIcon MapWidgetPlugin::icon() const {
  return QIcon();
}

QString MapWidgetPlugin::toolTip() const {
  return "MapWidget";
}

QString MapWidgetPlugin::whatsThis() const {
  return "ein Widget, um die Fahrzeugposition anzuzeigen";
}

bool MapWidgetPlugin::isContainer() const {
  return false;
}

QString MapWidgetPlugin::domXml() const {
  return "<widget class=\"DerWeg::MapWidget\" name=\"MapWidget\">\n"
            " <property name=\"geometry\">\n"
            "  <rect>\n"
            "   <x>0</x>\n"
            "   <y>0</y>\n"
            "   <width>320</width>\n"
            "   <height>240</height>\n"
            "  </rect>\n"
            " </property>\n"
            "</widget>\n";
}

QString MapWidgetPlugin::includeFile() const {
  return "MapWidget.h";
}

void MapWidgetPlugin::initialize(QDesignerFormEditorInterface * /* core */) {
  if (initialized)
    return;
  initialized = true;
}

bool MapWidgetPlugin::isInitialized() const {
  return initialized;
}

Q_EXPORT_PLUGIN2(customwidgetplugin, MapWidgetPlugin)
