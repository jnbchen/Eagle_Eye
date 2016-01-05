
#ifndef _DerWeg_MapWidget_h_
#define _DerWeg_MapWidget_h_

#include <vector>
#include <QtGui/QWidget>
#include "../../Elementary/Vec.h"

namespace DerWeg {

  class MapWidget : public QWidget {
    Q_OBJECT

  public:
    MapWidget(QWidget* =0, Qt::WindowFlags =0);
    ~MapWidget ();

  public slots:
    void paintEvent (QPaintEvent*);
    void setVehiclePose (Vec pos, Angle ori, double vel, Angle steer);
    void setBackgroundPixmap (const QPixmap* pm, Vec leftlower, Vec rightupper);
    void setFullCourse ();
    void setZoom (double);
    double getZoom ();
    void setTrace (bool);
    void setOrientation (unsigned int);
    unsigned int getOrientation ();
    void setPlotcmd (const std::string&);

  private:
    const QPixmap* background;
    Vec pixmap_left_lower;
    Vec pixmap_right_upper;
    bool vehicle_centered_display;
    double zoom_factor;
    bool do_trace;
    std::vector<Vec> trace_pos;
    unsigned int orientation;  // turn by orientation*90 degree

    Vec vehicle_position;
    Angle vehicle_orientation;
    double vehicle_velocity;
    Angle vehicle_steering_angle;
    std::string plotcmd;
  };

}

#endif
