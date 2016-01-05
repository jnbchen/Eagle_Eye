
#include "../../Localization/StarGazerCalibration.h"
#include "../../Elementary/Gnuplot.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <cstdlib>
#include <algorithm>

using namespace std;
using namespace DerWeg;

namespace {
  std::string int2str(int i) {
    std::stringstream s;
    s << i;
    return s.str();
  }
}

Gnuplot plot;

void plotAll (const LandmarkList& landmarks, const RobotPositionList& positions, const MeasurementList& measurements, bool ignore_isolated, StarGazerCalibration& cal) {
  std::vector<unsigned int> num_landmarks_per_position = cal.connectedLandmarks ();
  ofstream tmpfile1 ("/tmp/osgc1.dat");
  ofstream tmpfile2 ("/tmp/osgc2.dat");
  for (unsigned int il=0; il<landmarks.size(); ++il) {
    Vec v = 400*Vec::unit_vector (landmarks[il].orientation);
    tmpfile1 << landmarks[il].position.x << ' ' << landmarks[il].position.y << ' ' << v.x << ' ' << v.y << '\n';
  }
  tmpfile1 << '\n';
  for (unsigned int ip=0; ip<positions.size(); ++ip) {
    if (!ignore_isolated || num_landmarks_per_position[ip]>1 || positions[ip].fixed) {
      Vec v = 400*Vec::unit_vector (positions[ip].orientation);
      tmpfile1 << positions[ip].position.x << ' ' << positions[ip].position.y << ' ' << v.x << ' ' << v.y << '\n';
    }
  }
  tmpfile1 << flush;
  vector<TensionTriple> relations = cal.getTensions();
  std::sort (relations.begin(), relations.end());
  cout << "Most tensioning: \n";
  unsigned int i=0;
  unsigned int n=0;
  for (vector<TensionTriple>::reverse_iterator it=relations.rbegin(); it!=relations.rend(); ++it) {
    if (!ignore_isolated || num_landmarks_per_position[it->position_id]>1 || positions[it->position_id].fixed) {
      Vec lp = landmarks[landmarks.findLandmark (it->landmark_id)].position;
      Vec pp = positions[it->position_id].position;
      tmpfile2 << lp.x << ' ' << lp.y << '\n';
      tmpfile2 << pp.x << ' ' << pp.y << '\n';
      tmpfile2 << '\n';
      if (i<10) ++n;
      if (!it->fixed || !ignore_isolated) {
        if (i++<10) {
          std::cout << "L" << it->landmark_id << " - P" << it->position_id << " with " << it->num << " measurements, length " << it->length << ", deviation " << it->std << ", and error " << it->error << std::endl;
        }
      }
    }
  }
  tmpfile2 << flush;
  string command = "set size ratio -1; unset label;";
  command += "set style lines 1 linecolor rgb \"#a0a0a0\"; set style lines 2 linecolor rgb \"#FF0000\"; set style lines 3 linecolor rgb \"#0000FF\"; ";
  command += "set style lines 4 linecolor rgb \"#00c000\"; set style lines 5 linecolor rgb \"#00ff00\"; ";
  for (unsigned int il=0; il<landmarks.size(); ++il) {
    command += std::string("set label \" L")+int2str(landmarks[il].id)+std::string("\" at ")+int2str(landmarks[il].position.x)+std::string(",")+int2str(landmarks[il].position.y)+std::string(" textcolor ls 2; ");
  }
  for (unsigned int ip=0; ip<positions.size(); ++ip) {
    if (!ignore_isolated || num_landmarks_per_position[ip]>1 || positions[ip].fixed) {
      command += std::string("set label \" P")+int2str(positions[ip].id)+std::string("\" at ")+int2str(positions[ip].position.x)+std::string(",")+int2str(positions[ip].position.y)+std::string(" textcolor ls 3; ");
    }
  }
  command += "plot \"/tmp/osgc2.dat\" using 1:2 every :::::0 notitle with lines ls 5,";  // measurements most tensioning
  command += "\"/tmp/osgc2.dat\" using 1:2 every :::1::" + int2str(n-1) + " notitle with lines ls 4,";  // measurements tensioning
  command += "\"/tmp/osgc2.dat\" using 1:2 every :::" + int2str(n) + " notitle with lines ls 1,";  // other measurements
  command += "\"/tmp/osgc1.dat\" using 1:2:3:4 every :::0::0 notitle with vectors ls 2,";  // landmarks
  command += "\"/tmp/osgc1.dat\" using 1:2:3:4 every :::1::1 notitle with vectors ls 3,";  // positions
  command += "\"/tmp/osgc1.dat\" using 1:2 every :::0::0 notitle with points ls 2,";  // landmarks
  command += "\"/tmp/osgc1.dat\" using 1:2 every :::1::1 notitle with points ls 3";  // positions
  plot (command.c_str());
}


int main (int argc, char** argv) {
  if (argc<2) {
    cerr << "Aufruf: " << argv[0] << " Cfg-File(input) [Cfg-File2(input) ...]\n";
    cerr << "Ergebnis wird nach \"result.cfg\" geschrieben\n";
    return -1;
  }
  try{
    srand (26623); rand(); rand();
    StarGazerCalibration cal;
    for (int i=1; i<argc; ++i) {
      ConfigReader cfg;
      cfg.append_from_file (argv[i]);
      LandmarkList landmarks;
      landmarks.readLandmarks (cfg);
      RobotPositionList positions;
      positions.readPositions (cfg);
      MeasurementList measurements;
      measurements.readMeasurements (cfg);
      cal.addAndMerge (landmarks, positions, measurements);
    }
    bool first=true;
    while (true) {
      char c='h';
      if (first) {
        first=false;
      } else {
        cout << "> ";
        cin >> c;
      }
      switch (c) {
        case 'x' : return 0;
        case 'Q' : {
          string filename;
          cin >> filename;
          ofstream rf (filename.c_str());
          cal.getLandmarks().writeLandmarks (rf);
          cal.getPositions().writePositions (rf);
          cal.getMeasurements().writeMeasurements (rf);
          rf << flush;
          break;
        }
        case 'q' : {
          ofstream rf ("result.cfg");
          cal.getLandmarks().writeLandmarks (rf);
          cal.getPositions().writePositions (rf);
          cal.getMeasurements().writeMeasurements (rf);
          rf << flush;
          return 0;
        }
        case 'c' : cal.calibrateLandmarks ();
        case 'e' : cout << "average error(angle,pos)=" << cal.totalAngleError (1.0) << ", " << cal.totalPositionError (1.0) << endl; break;
        case 'd' : {
          double maxstd;
          cin >> maxstd;
          cal.removeLargeDeviation (maxstd);
          break;
        }
        case 'f' : {
          vector<vector<string> > comps = cal.connectedComponents();
          for (unsigned int i=0; i<comps.size(); ++i) {
            cout << "   {";
            for (unsigned int j=0; j<comps[i].size(); ++j) {
              cout << ' ' << comps[i][j];
            }
            cout << " }\n";
          }
          break;
        }
        case 'i' : cal.removeIrrelevantMeasurements (std::cout); break;
        case 'l' : {
          double maxdist;
          cin >> maxdist;
          cal.removeLargeDistances (maxdist);
          break;
        }
        case 'n' : cal.removeMostTensioningMeasurements (false, std::cout); break;
        case 'N' : cal.removeMostTensioningMeasurements (true, std::cout); break;
        case 'p' : plotAll (cal.getLandmarks(), cal.getPositions(), cal.getMeasurements(), false, cal); break;
        case 'P' : plotAll (cal.getLandmarks(), cal.getPositions(), cal.getMeasurements(), true, cal); break;
        case 'r' : {
          unsigned int lid, pid;
          cin >> lid >> pid;
          cal.removeRelationship (lid,pid);
          break;
        }
        case 's' : cal.printStatisticsLandmarks (cout); break;
        case 'S' : cal.printStatisticsPositions (cout); break;
        case 't' : {
          double accrate;
          cin >> accrate;
          cal.trimMeasurements (accrate);
          break;
        }
        case 'v' : {
          string filename;
          cin >> filename;
          try{
            StarGazerCalibration valid;
            ConfigReader cfg;
            cfg.append_from_file (filename.c_str());
            RobotPositionList positions1;
            positions1.readPositions (cfg);
            MeasurementList measurements1;
            measurements1.readMeasurements (cfg);
            valid.addAndMerge (cal.getLandmarks(), positions1, measurements1);
            valid.determinePositions ();
            cout << "average error(angle,pos)=" << valid.totalAngleError (1.0) << ", " << valid.totalPositionError (1.0) << endl;
          }catch(std::exception& e) {
            cerr << e.what() << endl;
          }
          break;
        }
        case 'w' : {
          unsigned int nmin;
          cin >> nmin;
          cal.removeWeakRelationships (nmin);
          break;
        }
        default : {
          cout << "c: recalibrate\n";
          cout << "d <maxstd>: remove relationships with deviation larger than maxstd\n";
          cout << "e: average calibration error\n";
          cout << "f: show connected components\n";
          cout << "i: remove irrelevant relationships (connecting only one non-fixed position)\n";
          cout << "l <maxdist>: remove relationships with distance larger than maxdist\n";
          cout << "n: remove measurements that create largest error in network\n";
          cout << "N: remove measurements that create largest error in network, ignore fixed positions\n";
          cout << "p: plot landmarks and positions\n";
          cout << "P: plot landmarks and non-isolated positions\n";
          cout << "q: save (in file result.cfg) and quit\n";
          cout << "Q <filename>: save in file filename\n";
          cout << "r <L> <P>: remove relationship between landmark L and position P\n";
          cout << "s/S: print statistics\n";
          cout << "t <accept-rate>: trim measurements with acceptance rate accept-rate\n";
          cout << "v <filename>: calculate validation error from data in file filename\n";
          cout << "w <nmin>: remove relationships with less than nmin measurements\n";
          cout << "x: exit without saving\n";
        }
      }
    }
  }catch(std::exception& e){
    cerr << "Exception: " << e.what() << endl;
    return -1;
  }
}
