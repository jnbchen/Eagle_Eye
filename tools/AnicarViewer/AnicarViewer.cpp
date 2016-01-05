
#include "AnicarViewer.h"
#include <QtGui/QFileDialog>
#include <QtGui/QInputDialog>
#include <QtGui/QMessageBox>
#include <algorithm>
#include <iostream>
#include <cstdlib>
#include <sstream>

using namespace std;
using namespace DerWeg;

namespace {
  double steer_sens (double x) {
    double alpha=0.5;
    return alpha*x+(1-alpha)*x*x*x;
  }

  double vel_sens (double x) {
    double alpha=0.5;
    return alpha*x+(1-alpha)*x*x*x;
  }
}

AnicarJoystick::AnicarJoystick () : joy(NULL) {;}

AnicarJoystick::AnicarJoystick (const char* device, unsigned int acc_axis, unsigned int steer_axis, unsigned int start_button, unsigned int stop_button, double vmax) : joy(NULL) {
  init (acc_axis, steer_axis, start_button, stop_button, vmax);
  init (device);
}

AnicarJoystick::~AnicarJoystick () { if (joy) delete joy; }

bool AnicarJoystick::init (const char* device) {
  if (joy) {
    delete joy;
    joy=NULL;
  }
  try{
    joy = new Joystick (device);
  }catch(std::exception&){ joy=NULL; return false; }
  return true;
}

void AnicarJoystick::init (unsigned int acc_axis, unsigned int steer_axis, unsigned int start_button, unsigned int stop_button, double vmax) {
  acc=acc_axis;
  steer=steer_axis;
  start=start_button;
  stop=stop_button;
  velmax=vmax;
}

bool AnicarJoystick::isInited () const { return joy; }

Velocity AnicarJoystick::getVelocity () {
  Velocity dv;
  if (joy) {
    const std::vector<double>& axis = joy->get_axis_state();
    dv.steer = Angle::deg_angle (45*steer_sens(-axis[steer]));
    dv.velocity = -velmax*vel_sens(axis[acc]);
  }
  return dv;
}

AnicarJoystick::Activity AnicarJoystick::getActivity () {
  const std::vector<bool>& buttons = joy->get_button_state();
  if (buttons[stop]) return Stop;
  if (buttons[start]) return Start;
  return Cont;
}





DerWeg::AnicarViewer::AnicarViewer(const ConfigReader& cfg, QWidget* p, Qt::WindowFlags f) : QMainWindow (p,f), logwriter (NULL), comm(NULL), autoStep(0)
{
  setupUi(this);
  QList<int> splits;
  splits.append(250);
  splits.append(100);
  splitter->setSizes(splits);
  moduleSelectionDialog = new DialogModuleSelection (this);
  setFocusPolicy (Qt::StrongFocus);
  setFocus ();

  connect (pushButtonStart,SIGNAL(clicked()),this,SLOT(startClicked()));
  connect (pushButtonStop,SIGNAL(clicked()),this,SLOT(stopClicked()));
  connect (actionEditModules,SIGNAL(triggered()),this,SLOT(editModulesClicked()));
  connect (actionSaveCameraImage,SIGNAL(triggered()),this,SLOT(saveCameraImageClicked()));
  connect (actionUseJoystick,SIGNAL(toggled(bool)),this,SLOT(useJoystickToggled(bool)));
  connect (actionQuit,SIGNAL(triggered()),this,SLOT(quit()));
  connect (moduleSelectionDialog,SIGNAL(addModule(std::string)),this,SLOT(addModuleClicked(std::string)));
  connect (moduleSelectionDialog,SIGNAL(removeModule(std::string)),this,SLOT(removeModuleClicked(std::string)));
  connect (actionZoomIn,SIGNAL(triggered()),this,SLOT(zoomIn()));
  connect (actionZoomOut,SIGNAL(triggered()),this,SLOT(zoomOut()));
  connect (actionZoomAll,SIGNAL(triggered()),mapWidget,SLOT(setFullCourse()));
  connect (actionShowTrace,SIGNAL(toggled(bool)),this,SLOT(setTrace(bool)));
  connect (actionChangeOrientation,SIGNAL(triggered()),this,SLOT(changeOrientation()));
  connect (pushButtonHistoryStart,SIGNAL(clicked()),this,SLOT(historyStartClicked()));
  connect (pushButtonHistoryEnd,SIGNAL(clicked()),this,SLOT(historyEndClicked()));
  connect (horizontalSliderHistory,SIGNAL(sliderMoved(int)),this,SLOT(historySliderMoved(int)));
  connect (horizontalSliderHistory,SIGNAL(actionTriggered(int)),this,SLOT(historySliderAction(int)));
  connect (actionOpenLogfile,SIGNAL(triggered()),this,SLOT(loadLogfile()));
  connect (actionConnectToHost,SIGNAL(triggered()),this,SLOT(connectToHost()));
  connect (actionForward,SIGNAL(triggered()),this,SLOT(setAutoStepPlusOne()));
  connect (actionBackward,SIGNAL(triggered()),this,SLOT(setAutoStepMinusOne()));
  connect (actionFastForward,SIGNAL(triggered()),this,SLOT(setAutoStepPlusTen()));
  connect (actionFastBackward,SIGNAL(triggered()),this,SLOT(setAutoStepMinusTen()));
  connect (actionStopHistory,SIGNAL(triggered()),this,SLOT(setAutoStepZero()));
  connect (actionNextStep,SIGNAL(triggered()),this,SLOT(nextStep()));
  connect (actionPreviousStep,SIGNAL(triggered()),this,SLOT(previousStep()));
  connect (actionBeginOfHistory,SIGNAL(triggered()),this,SLOT(historyStartClicked()));
  connect (actionEndOfHistory,SIGNAL(triggered()),this,SLOT(historyEndClicked()));
  connect (actionClearHistory,SIGNAL(triggered()),this,SLOT(clearHistory()));

  useJoystick=false;
  remoteState.active=false;
  commState.connected=false;
  commState.interrupted=false;
  commState.activeRequest=false;
  commState.desiredActive=false;
  commState.velocityRequest=false;
  commState.saveImageRequest=false;

  ledActive->setColor (0,QColor (127,0,0));
  ledActive->setColor (1,QColor (0,255,0));
  ledConnection->setColor (0,QColor (127,0,0));
  ledConnection->setColor (1,QColor (0,255,0));

  string dummy;
  if (cfg.get("help",dummy)) {
    string pn;
    cfg.get("ConfigReader::program_name",pn);
    cerr << "call: " << pn << " [Options] [Hostname]\n";
    cerr << "  with: \n";
    cerr << "   -h | --help: print this help and quit\n";
    cerr << "   --joydev=Device: specifiy the joystick device (default=/dev/input/js0)\n";
    cerr << "   --joyspeed=v: specify the maximal joystick speed\n";
    cerr << "   -b IMAGE | --background=Image: specify the background image\n";
    cerr << "   -d x1,y1,x2,y2 | --dimension=x1,y1,x2,y2: specify the position of the\n";
    cerr << "     left lower and right upper corner of the background image\n";
    cerr << "   -l File | --logfile=File: use AnicarViewer as viewer to visualize a logfile\n";
    cerr << "   -n | --nologging: suppresses the logging mechanism\n";
    cerr << "   --activelogging: suppresses logging when vehicle is inactive\n";
    cerr << "   -m UINT | --max_history_length=UINT: maximal history length\n";
    cerr << "   --size=UINTxUINT: widget size\n";
    cerr << "   -t | --trace: show trace\n";
    cerr << "   Hostname being the hostname (or IP address) of Anicar\n";
    exit (-1);
  }

  maxHistoryLength=36000;
  cfg.get ("max_history_length", maxHistoryLength);
  bool nologging=false;
  cfg.get ("nologging", nologging);
  logActiveOnly=false;
  cfg.get ("activelogging", logActiveOnly);
  string logfile = "";
  cfg.get ("logfile", logfile);
  string hostname = "localhost";
  cfg.get ("ConfigReader::unknown_argument_1", hostname);
  string joydev = "/dev/input/js0";
  cfg.get ("joydev", joydev);
  string bgimgname = ":/AnicarViewer/Images/rasterimage.png";
  cfg.get ("background", bgimgname);
  double joymaxvel = 1.0;
  cfg.get ("joyspeed", joymaxvel);
  string dimension="-1000,-1000,17000,7000";
  cfg.get ("dimension", dimension);
  stringstream inout;
  for (unsigned int i=0; i<dimension.size(); ++i) {
    if (dimension[i]!=',') {
      inout.put (dimension[i]);
    } else {
      inout.put (' ');
    }
  }
  Vec ll, ru;
  inout >> ll.x >> ll.y >> ru.x >> ru.y;

  QImage bgimg;
  bgimg.load (bgimgname.c_str());
  QPixmap pxm = QPixmap::fromImage (bgimg);
  mapWidget->setBackgroundPixmap (&pxm, ll, ru);
  mapWidget->setFullCourse();

  if (logfile.length()>0) {
    LogReader lr (logfile.c_str());
    lr.read (history);
    lineEditIP->setText (logfile.c_str());
    commState.connected=false;
    presentHistoryIndex=0;
  } else {
    commState.hostname = hostname;
    lineEditIP->setText (commState.hostname.c_str());
    initCommunication (commState.hostname.c_str(), 34396);
    presentHistoryIndex=-1;
    if (!nologging)
      logwriter = new LogWriter ("anicarviewer.log");
  }
  string sizestring;
  if (cfg.get("size",sizestring)) {
    stringstream inout;
    for (unsigned int i=0; i<sizestring.length(); ++i)
      inout.put (sizestring[i]=='x' ? ' ' : sizestring[i]);
    int w=200, h=200;
    inout >> w >> h;
    resize (w,h);
  }
  bool show_trace=false;
  cfg.get("trace", show_trace);
  if (show_trace)
    actionShowTrace->setChecked(true);

  joystick.init (4, 0, 0, 1, joymaxvel);
  joystickDevice = joydev;

  cycleTimer = new QTimer;
  connect (cycleTimer, SIGNAL(timeout()), this, SLOT(cycleCallback()));
  cycleTimer->start (100);
}


AnicarViewer::~AnicarViewer()
{
  deinitCommunication ();
}


void AnicarViewer::loadLogfile () {
  QString filename = QFileDialog::getOpenFileName (this, "Open Logfile", "", "Logfiles (*.log);;All files (*)");
  if (filename.length()>0) {
    try{
      LogReader lr (filename.toAscii());
      deinitCommunication ();
      history.clear();
      presentHistoryIndex=0;
      lr.read (history);
      lineEditIP->setText (filename);
      commState.connected=false;
      autoStep=0;
      updateHistory();
    }catch(std::exception& e) {
      QMessageBox::warning (this, "Warning", e.what());
    }
  }
}

void AnicarViewer::connectToHost () {
  QString hostname = QInputDialog::getText (this, "Connect to Host", "enter hostname:");
  if (hostname.length()>0) {
    KogmolaborCommunication* old_comm=comm;
    comm=NULL;
    try{
      history.clear();
      string new_hostname = string(hostname.toAscii());
      initCommunication (new_hostname.c_str(), 34396);
      KogmolaborCommunication* new_comm=comm;
      comm=old_comm;
      deinitCommunication ();
      comm=new_comm;
      commState.hostname = new_hostname;
      lineEditIP->setText (commState.hostname.c_str());
      presentHistoryIndex=-1;
      autoStep=0;
      updateHistory();
    }catch(std::exception& e) {
      comm=old_comm;
      QMessageBox::warning (this, "Warning", e.what());
    }
  }
}


void AnicarViewer::startClicked () {
  if (commState.connected) {
    commState.activeRequest=true;
    commState.desiredActive=true;
  }
}


void AnicarViewer::stopClicked () {
  commState.activeRequest=true;
  commState.desiredActive=false;
}


void AnicarViewer::editModulesClicked () {
  if (commState.connected) {
    moduleSelectionDialog->show();
  } else {
    statusbar->showMessage ("cannot edit modules: no connection to Anicar", 5000);
  }
}


void AnicarViewer::saveCameraImageClicked () {
  if (commState.connected) {
    commState.saveImageRequest=true;
  } else {
    statusbar->showMessage ("cannot make image: no connection to Anicar", 5000);
  }
}


void AnicarViewer::useJoystickToggled (bool use) {
  if (use) {
    if (joystick.init (joystickDevice.c_str())) {
      useJoystick=true;
    } else {
      useJoystick=false;
      statusbar->showMessage ((std::string("connecting to joystick failed on device ")+joystickDevice).c_str(), 5000);
      actionUseJoystick->setChecked(false);
    }
  } else {
    useJoystick=use;
  }
}


void AnicarViewer::quit () {
  qApp->quit();
}


void AnicarViewer::addModuleClicked (std::string m) {
  if (commState.connected) {
    commState.desiredAddModule=m;
  }
}


void AnicarViewer::removeModuleClicked (std::string m) {
  if (commState.connected) {
    commState.desiredRemoveModule=m;
  }
}

void AnicarViewer::updateHistory () {
  if (history.size()==0)
    return;
  const AnicarRemoteHistoryState& hstate (history [presentHistoryIndex<static_cast<long int>(history.size()) && presentHistoryIndex>=0 ? presentHistoryIndex : history.size()-1]);

  mapWidget->setVehiclePose (hstate.pose.position, hstate.pose.orientation, hstate.odometry.velocity, hstate.odometry.steer);
  mapWidget->setPlotcmd (hstate.plotcmd);

  lineEditX->setText (QString("%1").arg (hstate.pose.position.x, 8, 'f', 0));
  lineEditY->setText (QString("%1").arg (hstate.pose.position.y, 8, 'f', 0));
  lineEditPhi->setText (QString("%1").arg (hstate.pose.orientation.get_deg(), 8, 'f', 0));
  lineEditV->setText (QString("%1").arg (hstate.desiredVelocity.velocity, 8, 'f', 1));
  lineEditDelta->setText (QString("%1").arg (hstate.desiredVelocity.steer.get_deg_180(), 8, 'f', 0));
  lineEditTime->setText (QString("%1").arg (static_cast<double>(hstate.pose.timestamp.get_msec())/1000, 8, 'f', 2));
  if (presentMessageText!=hstate.message) {
    presentMessageText=hstate.message.c_str();
    textEditDebug->setText(presentMessageText.c_str());
  }
  if (static_cast<long int>(history.size())>horizontalSliderHistory->maximum()) {
    horizontalSliderHistory->setMaximum (history.size()+20);
  }
  horizontalSliderHistory->setValue (presentHistoryIndex<0 ? history.size()-1 : presentHistoryIndex);
  mapWidget->repaint();
}

void AnicarViewer::cycleCallback () {
  if (comm) {
    if (comm->receive()) {  // receive messages
      commState.latestMessageReceived.update();
      commState.connected=true;
      commState.interrupted=false;
      vector<string> stringlist1;
      if (comm->getAllModules(stringlist1)) {  // receive all modules
        remoteState.allModules=stringlist1;
        commState.modulesRequest=false;
      }
      vector<string> stringlist2;
      if (comm->getPresentModules(stringlist2)) {  // receive present modules
        remoteState.presentModules=stringlist2;
      }
      bool b;
      if (comm->getActive(b)) {
        remoteState.active=b;
        if (commState.activeRequest && commState.desiredActive==b) {
          commState.activeRequest=false;
        }
      }
      Pose p;
      if (comm->getPose(p)) {
        remoteState.pose=p;
      }
      Odometry o;
      if (comm->getOdometry(o)) {
        remoteState.odometry=o;
      }
      Velocity v;
      if (comm->getVelocity(v)) {
        remoteState.desiredVelocity=v;
      }
      std::string m;
      if (comm->getMessages(m)) {
        remoteState.message=m;
      } else {
        remoteState.message="";
      }
      if (comm->getPlotCommand(m)) {
        remoteState.plotcmd=m;
      } else {
        remoteState.plotcmd="";
      }
      if (comm->goodbye()) {
        commState.connected=false;
        commState.interrupted=false;
        commState.desiredActive=false;
        commState.activeRequest=false;
        commState.velocityRequest=false;
        commState.saveImageRequest=false;
        commState.modulesRequest=true;
        commState.desiredAddModule="";
        commState.desiredRemoveModule="";
        remoteState.presentModules.clear();
        remoteState.allModules.clear();
        statusbar->showMessage ("received goodbye from DerWeg", 5000);
      }
    }

    if (commState.latestMessageReceived.elapsed_msec()>500) {
      commState.interrupted=true;
    }
    if (commState.latestMessageReceived.elapsed_msec()>5000) {
      commState.connected=false;
      commState.interrupted=false;
      commState.desiredActive=false;
      commState.activeRequest=false;
      commState.velocityRequest=false;
      commState.saveImageRequest=false;
      commState.modulesRequest=true;
      commState.desiredAddModule="";
      commState.desiredRemoveModule="";
      remoteState.presentModules.clear();
      remoteState.allModules.clear();
    }

    comm->putPing ();
    if (useJoystick) {
      comm->putVelocity (joystick.getVelocity());
      AnicarJoystick::Activity act = joystick.getActivity();
      if (act==AnicarJoystick::Start) {
        commState.desiredActive=true;
        commState.activeRequest=true;
      } else if (act==AnicarJoystick::Stop) {
        commState.desiredActive=false;
        commState.activeRequest=true;
      }
    }
    if (commState.activeRequest)
      comm->putActive (commState.desiredActive);
    if (commState.velocityRequest) {
      comm->putVelocity (commState.desiredVelocity);
      commState.velocityRequest=false;
    }
    if (commState.saveImageRequest) {
      comm->putSaveImage ();
      commState.saveImageRequest=false;
    }
    if (commState.modulesRequest) {
      comm->askForAllModules ();
    }
    if (commState.desiredAddModule.length()>0) {
      comm->putAddModule (commState.desiredAddModule);
      commState.desiredAddModule="";
    }
    if (commState.desiredRemoveModule.length()>0) {
      comm->putRemoveModule (commState.desiredRemoveModule);
      commState.desiredRemoveModule="";
    }

    comm->send();
  };

  ledActive->setOn (remoteState.active);
  if (commState.connected) {
    if (commState.interrupted) {
      ledConnection->setBlinkState (1);
    } else {
      ledConnection->setState (1);
    }
  } else {
    ledConnection->setState (0);
  }
  moduleSelectionDialog->setAllModules (remoteState.allModules);
  moduleSelectionDialog->setPresentModules (remoteState.presentModules);

  bool stepMade=false;
  if (autoStep!=0) {
    if (presentHistoryIndex<0) {
      if (autoStep<0) {
        presentHistoryIndex=static_cast<long int>(history.size())-1+autoStep;
        stepMade=true;
        if (presentHistoryIndex<0)
          presentHistoryIndex=0;
      }
    } else {
      presentHistoryIndex+=autoStep;
      stepMade=true;
      if (presentHistoryIndex>=static_cast<long int>(history.size()))
        presentHistoryIndex=-1;
    }
  }

  if (commState.connected && (remoteState.active || !logActiveOnly || history.size()==0)) {
    history.push_back (remoteState);
    if (history.size()>maxHistoryLength) {
      // Speicherueberlauf verhindern, nach einer Stunde wird der Anfang entfernt
      const int taillength=maxHistoryLength/5;
      history.erase (history.begin(), history.begin()+taillength);
      if (presentHistoryIndex>=taillength)
        presentHistoryIndex-=taillength;
      else if (presentHistoryIndex>=0)
        presentHistoryIndex=0;
    }
    if (logwriter)
      logwriter->write (remoteState);
    updateHistory();
  } else if (stepMade) {
    updateHistory();
  }
}


void AnicarViewer::initCommunication (const char* hostname, unsigned int port) {
  if (comm)
    delete comm;
  comm = new KogmolaborCommunication;
  bool okay = comm->init_as_client (hostname, port);
  if (!okay)
    throw invalid_argument (std::string("Cannot establish connection."));
  commState.hostname=hostname;
  commState.modulesRequest=true;
}


void AnicarViewer::deinitCommunication () {
  if (comm) {
    comm->clear_send_buffer();
    comm->sayGoodbye();
    comm->send();
    comm->close();
    delete comm;
    comm=NULL;
  }
}


void AnicarViewer::zoomIn () {
  mapWidget->setZoom (1.5*mapWidget->getZoom());
}


void AnicarViewer::zoomOut () {
  mapWidget->setZoom (mapWidget->getZoom()/1.5);
}


void AnicarViewer::changeOrientation () {
  mapWidget->setOrientation (mapWidget->getOrientation()+1);
}


void AnicarViewer::keyPressEvent (QKeyEvent* ev) {
  switch (ev->key()) {
  case Qt::Key_Space:
    stopClicked();
    break;
  case Qt::Key_G:
    startClicked();
    break;
  case Qt::Key_Y:
  case Qt::Key_Left:
    commState.desiredVelocity.velocity=remoteState.desiredVelocity.velocity;
    commState.desiredVelocity.steer=remoteState.desiredVelocity.steer+Angle::deg_angle(5);
    commState.velocityRequest=true;
    break;
  case Qt::Key_X:
  case Qt::Key_Right:
    commState.desiredVelocity.velocity=remoteState.desiredVelocity.velocity;
    commState.desiredVelocity.steer=remoteState.desiredVelocity.steer+Angle::deg_angle(-5);
    commState.velocityRequest=true;
    break;
  case Qt::Key_Plus:
  case Qt::Key_Up:
    commState.desiredVelocity.velocity=remoteState.desiredVelocity.velocity+0.1;
    commState.desiredVelocity.steer=remoteState.desiredVelocity.steer;
    commState.velocityRequest=true;
    break;
  case Qt::Key_Minus:
  case Qt::Key_Down:
    commState.desiredVelocity.velocity=remoteState.desiredVelocity.velocity-0.1;
    commState.desiredVelocity.steer=remoteState.desiredVelocity.steer;
    commState.velocityRequest=true;
    break;
  case Qt::Key_C:
    presentHistoryIndex=-1;
    updateHistory();
    showHistoryPosition();
    break;
  default: break;
  }
}

void AnicarViewer::setTrace (bool b) {
  mapWidget->setTrace (b);
  updateHistory();
}

void AnicarViewer::historyStartClicked () {
  presentHistoryIndex=0;
  autoStep=0;
  updateHistory();
  showHistoryPosition();
}

void AnicarViewer::historyEndClicked () {
  presentHistoryIndex=-1;
  autoStep=0;
  updateHistory();
  showHistoryPosition();
}

void AnicarViewer::historySliderMoved (int v) {
  presentHistoryIndex=v;
  if (presentHistoryIndex>=static_cast<long int>(history.size()))
    presentHistoryIndex=-1;
  autoStep=0;
  updateHistory();
  showHistoryPosition();
}

void AnicarViewer::historySliderAction (int a) {
  if (a==QAbstractSlider::SliderPageStepAdd) {
    if (presentHistoryIndex>=0)
      presentHistoryIndex+=horizontalSliderHistory->pageStep();
    if (presentHistoryIndex>=static_cast<long int>(history.size()))
      presentHistoryIndex=-1;
    autoStep=0;
    updateHistory();
    showHistoryPosition();
  } else if (a==QAbstractSlider::SliderPageStepSub) {
    if (presentHistoryIndex<0)
      presentHistoryIndex=history.size()-1;
    presentHistoryIndex-=horizontalSliderHistory->pageStep();
    if (presentHistoryIndex<0)
      presentHistoryIndex=0;
    autoStep=0;
    updateHistory();
    showHistoryPosition();
  }
}

void AnicarViewer::showHistoryPosition () {
  QString s;
  if (presentHistoryIndex<0)
    s="position in history: present";
  else {
    QString s1;
    s1.setNum (presentHistoryIndex);
    s=QString("position in history: ")+s1;
  }
  statusbar->showMessage (s, 1000);
}

void AnicarViewer::setAutoStepZero () { autoStep=0; if (presentHistoryIndex<0 && history.size()>0) presentHistoryIndex=history.size()-1; }

void AnicarViewer::setAutoStepPlusOne() { autoStep=+1; }

void AnicarViewer::setAutoStepMinusOne() { autoStep=-1; }

void AnicarViewer::setAutoStepPlusTen() { autoStep=+10; }

void AnicarViewer::setAutoStepMinusTen() { autoStep=-10; }

void AnicarViewer::nextStep() {
  if (presentHistoryIndex<0) presentHistoryIndex=-1;
  else if (presentHistoryIndex+1>=static_cast<long int>(history.size())) presentHistoryIndex=-1;
  else if (presentHistoryIndex+1<static_cast<long int>(history.size())) presentHistoryIndex++;
  autoStep=0;
  updateHistory();
  showHistoryPosition();
}

void AnicarViewer::previousStep() {
  if (presentHistoryIndex<0) {
    if (history.size()>=2) presentHistoryIndex=history.size()-2;
  } else if (presentHistoryIndex>=1) presentHistoryIndex--;
  autoStep=0;
  updateHistory();
  showHistoryPosition();
}

void AnicarViewer::clearHistory() {
  history.clear();
  presentHistoryIndex=-1;
  autoStep=0;
  updateHistory();
  showHistoryPosition();
}


LogWriter::LogWriter (const char* filename) : file (filename) {
  file << "A2";
}

void LogWriter::write (const AnicarRemoteHistoryState& s) {
  file.write (reinterpret_cast<const char*>(&s.active), sizeof(bool));
  file.write (reinterpret_cast<const char*>(&s.pose.position), sizeof(Vec));
  file.write (reinterpret_cast<const char*>(&s.pose.orientation), sizeof(Angle));
  file.write (reinterpret_cast<const char*>(&s.pose.velocity), sizeof(double));
  file.write (reinterpret_cast<const char*>(&s.pose.yawrate), sizeof(double));
  long int t=s.pose.timestamp.get_msec();
  file.write (reinterpret_cast<const char*>(&t), sizeof(long int));
  file.write (reinterpret_cast<const char*>(&s.odometry), sizeof(Odometry));
  file.write (reinterpret_cast<const char*>(&s.desiredVelocity), sizeof(Velocity));
  file.write (s.message.c_str(), s.message.length()+1);
  file.write (s.plotcmd.c_str(), s.plotcmd.length()+1);
  file << flush;
}

LogReader::LogReader (const char* filename) throw (std::invalid_argument) : file (filename), version(1) {
  if (!file)
    throw invalid_argument (string("log file named ")+filename+string(" does not exist"));
  char codingid [2];
  file.read (codingid, 2);
  if (codingid[0]!='A' || (codingid[1]!='1' && codingid[1]!='2'))
    throw invalid_argument (string("log file named ")+filename+string(" doe not start with correct magic number"));
  version = codingid[1]-'0';
}

unsigned int LogReader::read (std::deque<AnicarRemoteHistoryState>& dest) {
  dest.clear();
  while (!file.eof()) {
    AnicarRemoteHistoryState s;
    file.read (reinterpret_cast<char*>(&s.active), sizeof(bool));
    if (file.eof()) break;
    file.read (reinterpret_cast<char*>(&s.pose.position), sizeof(Vec));
    if (file.eof()) break;
    file.read (reinterpret_cast<char*>(&s.pose.orientation), sizeof(Angle));
    if (file.eof()) break;
    file.read (reinterpret_cast<char*>(&s.pose.velocity), sizeof(double));
    if (file.eof()) break;
    file.read (reinterpret_cast<char*>(&s.pose.yawrate), sizeof(double));
    if (file.eof()) break;
    long int t=0;
    file.read (reinterpret_cast<char*>(&t), sizeof(long int));
    s.pose.timestamp.set_msec (t);
    if (file.eof()) break;
    file.read (reinterpret_cast<char*>(&s.odometry), sizeof(Odometry));
    if (file.eof()) break;
    file.read (reinterpret_cast<char*>(&s.desiredVelocity), sizeof(Velocity));
    if (file.eof()) break;
    char c;
    while (!file.eof()) {
      file.get (c);
      if (c=='\0') break;
      s.message+=c;
    }
    if (version>=2) {
      while (!file.eof()) {
        file.get (c);
        if (c=='\0') break;
        s.plotcmd+=c;
      }
    }
    dest.push_back(s);
  }
  return dest.size();
}
