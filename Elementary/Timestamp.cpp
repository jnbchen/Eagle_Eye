
#include "Timestamp.h"
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/date_time/gregorian/gregorian.hpp>

#ifdef WIN32

// der Code zur Anpassung an Win32 ist aus dem Internet kopiert (26.5.2009): http://social.msdn.microsoft.com/forums/en-US/vcgeneral/thread/430449b3-f6dd-4e18-84de-eebd26a8d668
struct timezone 
{
  int  tz_minuteswest; /* minutes W of Greenwich */
  int  tz_dsttime;     /* type of dst correction */
};
#if defined(_MSC_VER) || defined(_MSC_EXTENSIONS)
  #define DELTA_EPOCH_IN_MICROSECS  11644473600000000Ui64
#else
  #define DELTA_EPOCH_IN_MICROSECS  11644473600000000ULL
#endif
namespace {
  int gettimeofday(struct timeval *tv, struct timezone *tz)
  {
    FILETIME ft;
    unsigned __int64 tmpres = 0;
    static int tzflag;

    if (NULL != tv)
    {
      GetSystemTimeAsFileTime(&ft);

      tmpres |= ft.dwHighDateTime;
      tmpres <<= 32;
      tmpres |= ft.dwLowDateTime;

      /*converting file time to unix epoch*/
      tmpres -= DELTA_EPOCH_IN_MICROSECS; 
      tmpres /= 10;  /*convert into microseconds*/
      tv->tv_sec = (long)(tmpres / 1000000UL);
      tv->tv_usec = (long)(tmpres % 1000000UL);
    }

    if (NULL != tz)
    {
      if (!tzflag)
      {
        _tzset();
        tzflag++;
      }
      tz->tz_minuteswest = _timezone / 60;
      tz->tz_dsttime = _daylight;
    }

    return 0;
  }
}

#endif

const DerWeg::Timestamp DerWeg::Timestamp::starting_time;
const boost::posix_time::ptime DerWeg::Timestamp::time_origin (boost::gregorian::date(1970,1,1));

DerWeg::Timestamp::Timestamp () throw () {
  update ();
}

DerWeg::Timestamp::Timestamp (const DerWeg::Timestamp& src) throw () : sec (src.sec), usec (src.usec) {;}

DerWeg::Timestamp::Timestamp (const timeval& tv) throw () {
  set (tv);
}

DerWeg::Timestamp::Timestamp (const boost::posix_time::ptime& pt) throw () {
  boost::posix_time::time_duration diff = pt-time_origin;
  sec = diff.total_seconds();
  diff -= boost::posix_time::seconds(sec);
  usec = diff.total_microseconds();
}

DerWeg::Timestamp::operator boost::posix_time::ptime () const throw () {
  boost::posix_time::ptime res (time_origin);
  res+=boost::posix_time::seconds (sec);
  res+=boost::posix_time::microseconds (usec);
  return res;
}

const DerWeg::Timestamp& DerWeg::Timestamp::operator= (const DerWeg::Timestamp& src) throw () {
  sec=src.sec;
  usec=src.usec;
  return *this;
}

const DerWeg::Timestamp& DerWeg::Timestamp::operator= (const boost::posix_time::ptime& pt) throw () {
  return operator= (DerWeg::Timestamp(pt));
}

void DerWeg::Timestamp::set_usec (const long int& d) throw () {
  sec=starting_time.sec;
  usec=starting_time.usec;
  add_usec (d);
}

void DerWeg::Timestamp::set_msec (const long int& d) throw () {
  sec=starting_time.sec;
  usec=starting_time.usec;
  add_msec (d);
}

void DerWeg::Timestamp::set_sec (const long int& d) throw () {
  sec=starting_time.sec;
  usec=starting_time.usec;
  add_sec (d);
}

void DerWeg::Timestamp::update () throw () {
  timeval systime;
  struct timezone no_good;
  gettimeofday (&systime, &no_good);
  sec=systime.tv_sec;
  usec=systime.tv_usec;
}

long int DerWeg::Timestamp::elapsed_usec () const throw () {
  Timestamp now;
  return now.diff_usec (*this);
}

long int DerWeg::Timestamp::elapsed_msec () const throw () {
  Timestamp now;
  return now.diff_msec (*this);
}

long int DerWeg::Timestamp::elapsed_sec () const throw () {
  Timestamp now;
  return now.diff_sec (*this);
}

void DerWeg::Timestamp::add_usec (long int n) throw () {
  usec+=n;
  if (usec<0) {
    long int k=-usec/1000000+1;
    sec-=k;
    usec+=1000000*k;
  } else if (usec>=1000000) {
    long int k=usec/1000000;
    sec+=k;
    usec-=1000000*k;
  }
}

void DerWeg::Timestamp::add_msec (long int n) throw () {
  add_usec (1000*n);
}

void DerWeg::Timestamp::add_sec (long int n) throw () {
  sec+=n;
}

long int DerWeg::Timestamp::diff_usec (const DerWeg::Timestamp& src) const throw () {
  return 1000000*(sec-src.sec)+(usec-src.usec);
}

long int DerWeg::Timestamp::diff_msec (const DerWeg::Timestamp& src) const throw () {
  return 1000*(sec-src.sec)+(usec-src.usec)/1000;
}

long int DerWeg::Timestamp::diff_sec (const DerWeg::Timestamp& src) const throw () {
  return sec-src.sec;
}

bool DerWeg::Timestamp::operator== (const DerWeg::Timestamp& src) const throw () {
  return ((sec==src.sec) && (usec==src.usec));
}

bool DerWeg::Timestamp::operator!= (const DerWeg::Timestamp& src) const throw () {
  return !operator==(src);
}

bool DerWeg::Timestamp::operator<= (const DerWeg::Timestamp& src) const throw () {
  return (diff_usec (src)<=0);
}

bool DerWeg::Timestamp::operator< (const DerWeg::Timestamp& src) const throw () {
  return (diff_usec (src)<0);
}

bool DerWeg::Timestamp::operator>= (const DerWeg::Timestamp& src) const throw () {
  return (diff_usec (src)>=0);
}

bool DerWeg::Timestamp::operator> (const DerWeg::Timestamp& src) const throw () {
  return (diff_usec (src)>0);
}

long int DerWeg::Timestamp::get_usec () const throw () {
  return (sec-Timestamp::starting_time.sec)*1000000+usec-Timestamp::starting_time.usec;
}

long int DerWeg::Timestamp::get_msec () const throw () {
  return (sec-Timestamp::starting_time.sec)*1000+(usec-Timestamp::starting_time.usec)/1000;
}

long int DerWeg::Timestamp::get_sec () const throw () {
  return (sec-Timestamp::starting_time.sec);
}

void DerWeg::Timestamp::set (const timeval& tv) throw () {
  sec=tv.tv_sec;
  usec=tv.tv_usec;
}

void DerWeg::Timestamp::get (timeval& tv) const throw () {
  tv.tv_sec=sec;
  tv.tv_usec=usec;
}

std::ostream& operator<< (std::ostream& os, const DerWeg::Timestamp& tt) throw() {
  os << tt.get_msec();
  return os;
}
