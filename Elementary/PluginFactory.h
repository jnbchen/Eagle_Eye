/** Klasse zum Verwalten von Plugins in einer Factory. Alle Plugins muessen
 von einer gemeinsamen Elternklasse abgeleitet sein und mit Hilfe von
 PluginBuildern bei der Registry angemeldet werden. Anschliessend koennen
 Plugins anhand ihrer ID von der Registry angefordert werden. Entwickelt im
 Rahmen des Brainstormers Tribots Projektes durch Martin Lauer
 (lauer@mrt.uka.de) und getestet unter Linux */

#ifndef _DerWeg_PluginFactory_h_
#define _DerWeg_PluginFactory_h_

#include <string>
#include <vector>
#include <stdexcept>
#include <algorithm>

namespace DerWeg {

  /** Abstract class to generate plugins; every plugin class needs an own
      builder class derived from this class. The generator class registers
      the plugin at the plugin factory and generates an instance of the
      plugin, if necessary. To simplify this process, the class PluginBuilder
      has been derived, see below */
  template <class PLUGIN>
  class PluginParentBuilder {
  public:
    /** destructor */
    virtual ~PluginParentBuilder () throw () {;}
    /** generate a new instance of the plugin; throw an exception
        if that is not possible */
    virtual PLUGIN* getPlugin () throw (std::bad_alloc, std::invalid_argument) =0;
  };

  /** Class to simplify the generation of a plugin builder and to register at
      the factory. Every plugin class C one instance of the class PluginBuilder
      has to generated:
        DerWeg::PluginBuilder<OC,C> xx ("name")
      where OC is the parent class of C and "name" is a descriptor used to
      identify the plugin at the factory */
  template <class PLUGINPARENT, class PLUGINCHILD>
  class PluginBuilder : public PluginParentBuilder<PLUGINPARENT> {
  public:
    PluginBuilder (const char*);
    PLUGINPARENT* getPlugin () throw (std::bad_alloc, std::invalid_argument);
  };

  /** PluginFactory zur Verwaltung verschiedener Typen von Plugins
      Jedes Plugin muss sich zunaechst bei der Factory anmelden;

      Prinzip der Nutzung:
      Es soll eine PluginFactory fuer einen bestimmten Typ (Elternklasse)
      erzeugt werden und verschiedene Kinder dieser Elternklasse verfuegbar
      gemacht werden.

      Anmelden der Kindklassen durch Erzeugen eines statischen Builderobjektes:
      static DerWeg::PluginBuilder<Elternklasse,Kindklasse> variablenname ("plugin-id");
      wobei "variablenname" beliebig sein darf und "plugin-id" ein frei
      waehlbarer, aber eindeutiger Schluessel fuer diese Kindklasse sein muss.

      Ein Objekt einer Klasse mit dem Bezeichner "plugin-id" aus der Registry holen:
      DerWeg::PluginFactory<Elternklasse>::getPlugin ("plugin-id");
      Die erzeugten Objekte muessen ggf. von der aufrufenden Instanz vom
      Speicher entfernt werden, die Factory uebernimmt diese Aufgabe nicht.

      Voraussetzung zur Nutzung: Kindklasse muss einen Argument-freien
      Konstruktor besitzen, der vom PluginBuilder aufgerufen werden kann.
  */
  template <class PLUGIN>
  class PluginFactory {
  public:
    /** sign up a plugin of type PLUGIN with plugin builder 'builder'
      and key 'pluginId' */
    static void signUp (const std::string& pluginId, PluginParentBuilder<PLUGIN>* builder) throw (std::bad_alloc);
    /** get a plugin object of key 'pluginId'. Throws an std::invalid_argument
      if a plugin with the given name is not known to the registry */
    static PLUGIN* getPlugin (const std::string& pluginId);
    /** return the list of pluginIds known to the registry */
    static const std::vector<std::string>& pluginList () throw ();

  private:
    std::vector<PluginParentBuilder<PLUGIN>*> _builderList;  ///< list of available plugin builders
    std::vector<std::string> _idList;                        ///< list of available plugin identifiers. _builderList[i] refers to _idList[i]

    PluginFactory () throw (std::bad_alloc);              ///< private constructor (singleton)
    ~PluginFactory() throw ();                            ///< private destructor, factory should not be deleted

    static PluginFactory<PLUGIN>* the_only_factory;       ///< pointer to the only factory (singleton)

    /** static call instead of a constructor to get a pointer to the factory */
    static PluginFactory<PLUGIN>* getFactory () throw (std::bad_alloc);
    void _signUp (const std::string& pluginId, PluginParentBuilder<PLUGIN>* builder);
    PLUGIN* _getPlugin (const std::string&) const;
  };

} // namespace DerWeg




// implementation here, because of template declaration:
template <class PLUGIN> DerWeg::PluginFactory<PLUGIN>* DerWeg::PluginFactory<PLUGIN>::the_only_factory (NULL);  // Definition von static member

template <class PLUGINPARENT, class PLUGINCHILD>
DerWeg::PluginBuilder<PLUGINPARENT,PLUGINCHILD>::PluginBuilder (const char* name) {
  DerWeg::PluginFactory<PLUGINPARENT>::signUp (name, this);
 }

template <class PLUGINPARENT, class PLUGINCHILD>
PLUGINPARENT* DerWeg::PluginBuilder<PLUGINPARENT,PLUGINCHILD>::getPlugin () throw (std::bad_alloc, std::invalid_argument) {
  return new PLUGINCHILD;
}

template<class PLUGIN>
DerWeg::PluginFactory<PLUGIN>* DerWeg::PluginFactory<PLUGIN>::getFactory () throw (std::bad_alloc) {
  if (!the_only_factory)
    the_only_factory = new PluginFactory<PLUGIN>;
  return the_only_factory;
}

template<class PLUGIN>
void DerWeg::PluginFactory<PLUGIN>::_signUp (const std::string& pluginId, PluginParentBuilder<PLUGIN>* builder) {
  std::vector<std::string>::iterator it = std::lower_bound (_idList.begin(), _idList.end(), pluginId);
  int n=it-_idList.begin();
  if (it!=_idList.end() && (*it)==pluginId) {
    _builderList [n] = builder;
  } else {
    _idList.insert (it, pluginId);
    _builderList.insert (_builderList.begin()+n, builder);
  }
}

template<class PLUGIN>
void DerWeg::PluginFactory<PLUGIN>::signUp (const std::string& pluginId, PluginParentBuilder<PLUGIN>* builder) throw (std::bad_alloc) {
  getFactory ()->_signUp (pluginId, builder);
}

template<class PLUGIN>
PLUGIN* DerWeg::PluginFactory<PLUGIN>::_getPlugin (const std::string& pluginId) const {
  std::vector<std::string>::const_iterator it = std::lower_bound (_idList.begin(), _idList.end(), pluginId);
  int n=it-_idList.begin();
  if (it==_idList.end() || (*it)!=pluginId)
    throw std::invalid_argument (std::string("DerWeg::PluginFactory::getPlugin: plugin factory does not contain plugin with id: ")+pluginId);
  return _builderList[n]->getPlugin();
}

template<class PLUGIN>
PLUGIN* DerWeg::PluginFactory<PLUGIN>::getPlugin (const std::string& pluginId) {
  return getFactory ()->_getPlugin (pluginId);
}

template<class PLUGIN>
const std::vector<std::string>& DerWeg::PluginFactory<PLUGIN>::pluginList () throw () {
  return getFactory ()->_idList;
}

template<class PLUGIN>
DerWeg::PluginFactory<PLUGIN>::PluginFactory () throw (std::bad_alloc) {;}

template<class PLUGIN>
DerWeg::PluginFactory<PLUGIN>::~PluginFactory() throw () {;}


#endif
