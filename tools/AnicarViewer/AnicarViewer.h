
#ifndef _DerWeg_AnicarViewer_h_
#define _DerWeg_AnicarViewer_h_

#include "Ui_AnicarViewer.h"
#include "DialogModuleSelection.h"
#include "../../Elementary/UDP/KogmolaborCommunication.h"
#include "../../Elementary/ConfigReader.h"
#include "Joystick.h"
#include <QtGui/QMainWindow>
#include <QtCore/QTimer>
#include <deque>
#include <fstream>

namespace DerWeg {

  struct AnicarRemoteHistoryState {
    bool active;
    Pose pose;
    Odometry odometry;
    Velocity desiredVelocity;
    std::string message;
    std::string plotcmd;
  };

  class LogWriter {
    std::ofstream file;
  public:
    LogWriter (const char* filename);
    void write (const AnicarRemoteHistoryState&);
  };

  class LogReader {
    std::ifstream file;
    unsigned int version;
  public:
    LogReader (const char* filename) throw (std::invalid_argument);
    unsigned int read (std::deque<AnicarRemoteHistoryState>&);
  };

  struct AnicarRemoteState : public AnicarRemoteHistoryState {
    std::vector<std::string> presentModules;
    std::vector<std::string> allModules;
  };

  struct CommunicationState {
    std::string hostname;
    bool connected;
    bool interrupted;
    Timestamp latestMessageReceived;

    bool activeRequest;
    bool desiredActive;

    bool velocityRequest;
    Velocity desiredVelocity;

    bool saveImageRequest;

    bool modulesRequest;

    std::string desiredAddModule;
    std::string desiredRemoveModule;
  };

  class AnicarJoystick {
    Joystick* joy;
    unsigned int acc;
    unsigned int steer;
    unsigned int start;
    unsigned int stop;
    double velmax;
  public:
    enum Activity { Stop=0, Start=1, Cont=2 };

    AnicarJoystick ();
    AnicarJoystick (const char* device, unsigned int acc_axis, unsigned int steer_axis, unsigned int start_button, unsigned int stop_button, double vmax);
    ~AnicarJoystick ();

    bool init (const char* device);
    void init (unsigned int acc_axis, unsigned int steer_axis, unsigned int start_button, unsigned int stop_button, double vmax);

    bool isInited () const;
    Velocity getVelocity ();
    Activity getActivity ();
  };

  class AnicarViewer : public QMainWindow, private Ui::AnicarViewer {
    Q_OBJECT

  public:
    AnicarViewer(const ConfigReader& cfg, QWidget* =0, Qt::WindowFlags =0);
    ~AnicarViewer ();

  public slots:

  signals:

  private slots:
    void startClicked ();
    void stopClicked ();
    void editModulesClicked ();
    void saveCameraImageClicked ();
    void useJoystickToggled (bool);
    void quit ();
    void addModuleClicked (std::string);
    void removeModuleClicked (std::string);
    void zoomIn ();
    void zoomOut ();
    void changeOrientation ();
    void historyStartClicked ();
    void historyEndClicked ();
    void historySliderMoved (int);
    void historySliderAction (int);
    void setTrace (bool);
    void loadLogfile ();
    void connectToHost ();
    void setAutoStepZero ();
    void setAutoStepPlusOne();
    void setAutoStepMinusOne();
    void setAutoStepPlusTen();
    void setAutoStepMinusTen();
    void nextStep();
    void previousStep();
    void clearHistory();

    void cycleCallback ();

    void keyPressEvent (QKeyEvent*);

  private:
    LogWriter* logwriter;
    AnicarRemoteState remoteState;
    CommunicationState commState;
    KogmolaborCommunication* comm;
    AnicarJoystick joystick;
    std::string joystickDevice;
    std::deque<AnicarRemoteHistoryState> history;
    long int presentHistoryIndex;  // -1 means, the latest information
    std::string presentMessageText;

    DialogModuleSelection* moduleSelectionDialog;
    QTimer* cycleTimer;
    int autoStep;

    bool useJoystick;
    bool logActiveOnly;
    unsigned int maxHistoryLength;

    void initCommunication (const char* hostname, unsigned int port);
    void deinitCommunication ();
    void updateHistory ();
    void showHistoryPosition ();

    // the following attributes belong to the UI:
    // ledActive
    // ledConnection
    // lineEditIP
    // pushButtonStart
    // pushButtonStop
    // mapWidget
    // textEditDebug
    // actionEditModules
    // actionSaveCameraImage
    // actionUseJoystick
    // actionQuit
    // statusbar
    // actionZoomIn
    // actionZoomOut
    // actionZoomAll
    // actionChangeOrientation
    // horizontalSliderHistory
    // pushButtonHistoryStart
    // pushButtonHistoryEnd
    // lineEditX
    // lineEditY
    // lineEditPhi
    // lineEditV
    // lineEditDelta
    // lineEditTime
  };

}

#endif
