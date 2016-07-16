#include <stdio.h>
#include <ao/ao.h>
#include <sndfile.h>
#include <math.h>

int main(int argc, char **argv)
{
    SNDFILE*    infile = 0;
    SF_INFO        sfinfo;
    int            readcount;

    int rate = 44100;
    int channels = 2;

    if(!(infile = sf_open("rock.wav",SFM_READ,&sfinfo)))
    {
        return  1 ;
    };

    rate = sfinfo.samplerate;
    printf("rate %d\n",rate);
    channels = sfinfo.channels;
    printf("channels %d\n",channels);
    printf("format %d\n",sfinfo.format);

    short data[4096];
    ao_device *device;
    ao_sample_format format;
    int default_driver;
    int i;

    ao_initialize();

    default_driver = ao_default_driver_id();
    memset(&format, 0, sizeof(format));
    format.bits = 16;
    format.channels = channels;
    format.rate = rate;
    format.byte_format = AO_FMT_LITTLE;

    device = ao_open_live(default_driver,&format, NULL);
    if(device == NULL)
    {
        fprintf(stderr,"error opening device.\n");
        return 1;
    }

    while((readcount = sf_read_short(infile,data,4096)))
    {
        ao_play(device,data,readcount*2);
    };

    sf_close (infile) ;
    ao_close(device);
    ao_shutdown();
    return (0);
}
