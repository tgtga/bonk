/* Bonk - lossy/lossless audio compressor
   Copyright (C) 2001  Paul Francis Harrison

  This program is free software; you can redistribute it and/or modify it
  under the terms of the GNU General Public License as published by the
  Free Software Foundation; either version 2 of the License, or (at your
  option) any later version.

  This program is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  General Public License for more details.

  You should have received a copy of the GNU General Public License along
  with this program; if not, write to the Free Software Foundation, Inc.,
  675 Mass Ave, Cambridge, MA 02139, USA.

  The author may be contacted at:
    pfh@csse.monash.edu.au
  or
    3 Currajong St., Oakleigh East, 3166, Melbourne, Australia

  See also
    http://yoyo.cc.monash.edu.au/~pfh/
*/

const char *version = "0.6";

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <vector>
#include <string>
#include <algorithm>
#include <cstring>

using namespace std;

// Support from Linux and *BSD sound output

#if defined(__linux__)
#  define dsp_device "/dev/dsp"
#  include <linux/soundcard.h>
#elif defined(__FreeBSD__)
#  define dsp_device "/dev/dsp"
#  include <sys/soundcard.h>
#  if !defined(AFMT_S16_NE)
#  include <machine/endian.h>
#  if BYTE_ORDER == LITTLE_ENDIAN
#  define  AFMT_S16_NE AFMT_S16_LE
#  else
#  define  AFMT_S16_NE AFMT_S16_BE
#  endif
#  endif
#elif defined(__NetBSD__) || defined(__OpenBSD__)
#  define dsp_device "/dev/sound"
#  include <soundcard.h>
#endif
	      
#include "utility.h"
#include "wav.h"

//Accuracy of fixed point calculations
const int    lattice_shift  = 10,
             sample_shift   = 4,
             lattice_factor = 1<<lattice_shift,
             sample_factor  = 1<<sample_shift,

//Maximum allowable taps
             max_tap        = 2048;
      
//Default quantization level
const double base_quant     = 0.6,

//Amount of bit rate variation
             rate_variation = 3.0;


const int tap_quant[max_tap] = { //int(sqrt(i+1))
   1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4,
   4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6,
   6, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
   7, 7, 7, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
   9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9,10,
  10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,
  11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,
  11,11,11,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,
  12,12,12,12,12,12,12,12,13,13,13,13,13,13,13,13,13,13,13,13,
  13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,14,14,14,14,14,
  14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,
  14,14,14,14,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,
  15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,
  16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,
  16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,17,17,17,17,17,
  17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,
  17,17,17,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,
  18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,
  19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,
  19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,20,
  20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,
  20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,
  21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,
  21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,
  21,21,21,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,
  22,22,22,22,22,22,22,22,22,22,22,22
};

inline int divide(int a,int b) {
  if (a < 0)
    return -( (-a + b/2)/b );
  else
    return (a + b/2)/b;
}

inline int shift(int a,int b) {
  return a+(1<<b-1) >> b;
}

inline int shift_down(int a,int b) {
  return (a>>b)+(a<0?1:0);
}

struct lattice {
  int order;
  vector<int> k, state;

  void init(int _order) {
    order = _order;
    k.resize(order);
    state.resize(order);
    for(int i=0;i<order;i++) {
      state[i] = 0;
      k[i] = 0;
    }
  }

  void dequantize() { 
    for(int i=0;i<order;i++)
      k[i] *= tap_quant[i];
  }

  void init_state() {
    for(int i=order-2;i>=0;i--) {
      int x = state[i], new_x;
      for(int j=0,p=i+1;p<order;j++,p++) {
        new_x = x + shift_down(k[j]*state[p],lattice_shift);
        state[p] += shift_down(k[j]*x,lattice_shift);
        x = new_x;
      }
    }
  }

  /* This is now done in parallel with calculating the
   * reflection coefficients
   *
   * int advance_by_value(int value) { // returns error
   *   int  x = value,
   *       *k_ptr = &(k[0]),
   * 	   *state_ptr = &(state[0]);
   *   for(int i=0;i<order;i++,k_ptr++,state_ptr++) {
   *     int k_value =     *k_ptr,
   *         state_value = *state_ptr;
   *     state[i] += shift_down(k_value*x,lattice_shift);
   *     x += shift_down(k_value*state_value,lattice_shift);
   *   }
   *
   *   for(int i=order-1;i>0;i--)
   *     state[i] = state[i-1];
   *   state[0] = value;
   * 
   *   return x;
   * } 
   */

  int advance_by_error(int error) { // returns value
    int x = error;
    x -= shift_down(k[order-1]*state[order-1],lattice_shift);

    int *k_ptr     = &(k[order-2]),
        *state_ptr = &(state[order-2]);
    for(int i=order-2;i>=0;i--,k_ptr--,state_ptr--) {
      int k_value     = *k_ptr,
          state_value = *state_ptr;
      x -= shift_down(k_value*state_value,lattice_shift);
      state_ptr[1] = state_value+shift_down(k_value*x,lattice_shift);
    }

    //Don't drift too far, to avoid overflows 
    if (x >  (sample_factor<<16)) x =  (sample_factor<<16);
    if (x < -(sample_factor<<16)) x = -(sample_factor<<16);
    
    state[0] = x;

    return x;
  }
};


// Heavily modified Levinson-Durbin algorithm which
// copes better with quantization, and calculates the
// actual whitened result as it goes.

void modified_levinson_durbin(vector<int> &x, 
                              int channels, bool lossless,
                              vector<int> &k_out) {
  vector<int> state = x;

  for(int i=0;i<k_out.size();i++) {
    int step = (i+1)*channels;

    double xx=0.0, xy=0.0;
    int n         = x.size()-step,
       *x_ptr     = &(x[step]),
       *state_ptr = &(state[0]);
    for(;n>=0;n--,x_ptr++,state_ptr++) {
      double x_value     = *x_ptr,
             state_value = *state_ptr;
	     
      xx += state_value*state_value;
      xy += x_value*state_value;
    }

    int k;
    if (xx == 0.0)
      k = 0;
    else
      k = int(floor( -xy/xx *double(lattice_factor)/double(tap_quant[i]) +0.5 ));

    if (k  > lattice_factor/tap_quant[i]) k =   lattice_factor/tap_quant[i];
    if (-k > lattice_factor/tap_quant[i]) k = -(lattice_factor/tap_quant[i]);

    k_out[i] = k;
    k *= tap_quant[i];

    n         = x.size()-step;
    x_ptr     = &(x[step]);
    state_ptr = &(state[0]);
    for(;n>=0;n--,x_ptr++,state_ptr++) {
      int x_value     = *x_ptr,
          state_value = *state_ptr;
      *x_ptr     = x_value     + shift_down(k*state_value,lattice_shift);
      *state_ptr = state_value + shift_down(k*x_value,lattice_shift);
    }
  }
}

struct encoder {
  FILE *f_out;
  bitstream_out bit_out;
  int channels, rate;
  int samples_size;
  bool lossless;
  bool mid_side;
  int n_taps;
  int down_sampling, samples_per_packet;
  double quant_level;

  int sample_count; 

  vector< int > tail;
  vector< vector<int> > output_samples;

  void begin(FILE *_f_out,
             const char *text,
             uint32 length,
	     uint32 _rate,
	     int _channels,
	     bool _lossless,
	     bool _mid_side,
	     int _n_taps,
	     int _down_sampling,
	     int _samples_per_packet,
	     double _quant_level) {
    f_out = _f_out;
    channels = _channels;
    rate = _rate;
    lossless = _lossless;
    mid_side = _mid_side;
    n_taps = _n_taps;
    down_sampling = _down_sampling;
    samples_per_packet = _samples_per_packet;
    quant_level = _quant_level;

    if (n_taps > max_tap)
      throw error("Number of taps is very silly");

    if (mid_side && channels <= 1)
      mid_side = false;

    if (samples_per_packet <= 0)
      throw error("Bad packet size specified");

    fwrite(text,1,strlen(text),f_out);

    fputc(0,f_out);
    fputc('B',f_out);
    fputc('O',f_out);
    fputc('N',f_out);
    fputc('K',f_out);

    write_uint8 (f_out, 0); // version
    write_uint32(f_out, length);
    write_uint32(f_out, rate);
    write_uint8 (f_out, channels);
    write_uint8 (f_out, lossless?1:0);
    write_uint8 (f_out, mid_side?1:0);
    write_uint16(f_out, n_taps);
    write_uint8 (f_out, down_sampling);
    write_uint16(f_out, samples_per_packet);

    tail.resize(n_taps*channels);
    for(int i=0;i<tail.size();i++) 
      tail[i] = 0;

    output_samples.resize(channels);
    for(int i=0;i<channels;i++)
      output_samples[i].resize(samples_per_packet);

    samples_size = channels*samples_per_packet*down_sampling;
  
    sample_count = 0;

    bit_out.setup(f_out);
  }

  void finish() {
    bit_out.flush();
  }

  void store_packet(vector<int> &samples) {
    //samples must be correct size (samples_size)

    if (!lossless)
      for(int i=0;i<samples.size();i++)
        samples[i] <<= sample_shift;

    if (mid_side)
      for(int i=0;i<samples.size();i+=channels) {
        samples[i]   += samples[i+1];
	samples[i+1] -= shift(samples[i],1);
      }  
 
    vector<int> window(tail.size()*2+samples_size);
    int *ptr = &(window[0]);

    for(int i=0;i<tail.size();i++)
      *(ptr++) = tail[i];

    for(int i=0;i<samples_size;i++)
      *(ptr++) = samples[i];
   
    for(int i=0;i<tail.size();i++)
      *(ptr++) = 0;
    
    for(int i=0;i<tail.size();i++)
      tail[i] = samples[samples_size-tail.size()+i];
    
    vector<int> k(n_taps);
    modified_levinson_durbin(window,channels,lossless,k);

    write_list(k,false, bit_out); 

    for(int channel=0;channel<channels;channel++) {
      ptr = &(window[tail.size()+channel]);
      for(int i=0;i<samples_per_packet;i++) {
        int sum = 0;
	for(int j=0;j<down_sampling;j++,ptr += channels)
	  sum += *ptr;
	output_samples[channel][i] = sum;
      }
    }

    int quant;
    if (!lossless) {
      double energy2 = 0.0;
      double energy1 = 0.0;
      for(int channel=0;channel<channels;channel++) 
        for(int i=0;i<samples_per_packet;i++) {
	  double sample = output_samples[channel][i];
	  energy2 += sample*sample;
	  energy1 += fabs(sample);
	}

      energy2 = sqrt(energy2/(channels*samples_per_packet));
      energy1 = sqrt(2.0)*energy1/(channels*samples_per_packet);

      //Increase bitrate when samples are like a gaussian distribution 
      //Reduce bitrate when samples are like a two-tailed exponentional distribution 

      if (energy2 > energy1)
        energy2 += (energy2-energy1)*rate_variation;
      
      quant = int(base_quant*quant_level*energy2/sample_factor);

      if (quant < 1) 
        quant = 1;
      if (quant > 65535)
        quant = 65535;

      bit_out.write_uint(quant,16);

      quant *= sample_factor;
    }

    for(int channel=0;channel<channels;channel++) {
      if (!lossless)
        for(int i=0;i<samples_per_packet;i++)
          output_samples[channel][i] = divide(output_samples[channel][i],quant);

      write_list(output_samples[channel],true, bit_out);
    }

    sample_count += samples_size;
  }
};

struct decoder {
  FILE *f_in;
  bitstream_in bit_in;
  int length, length_remaining, rate;
  int channels;
  bool lossless;
  bool mid_side;
  int n_taps;
  int down_sampling, samples_per_packet;
  double quant_level;

  lattice predictor;
  vector< vector<int> > predictor_initer;

  void begin(FILE *_f_in) {
    f_in = _f_in;

    vector<char> buffer;
    for(int i=0;i<5;i++)
      buffer.push_back(fgetc(f_in));

    for(;;) {
      if (buffer[buffer.size()-5] == 0   &&
          buffer[buffer.size()-4] == 'B' &&
          buffer[buffer.size()-3] == 'O' &&
          buffer[buffer.size()-2] == 'N' &&
          buffer[buffer.size()-1] == 'K')
	break;

      buffer.push_back(fgetc(f_in));

      if (feof(f_in) || buffer.size() > 1000000)
        throw error("Input file is not in BONK format");
    }

    fprintf(stderr,"%s",&(buffer[0]));

    if (read_uint8(f_in) != 0) // version
      throw error("This version of Bonk can not read this BONK file format");

    length = read_uint32(f_in);
    rate = read_uint32(f_in); 
    channels = read_uint8 (f_in);
    lossless = read_uint8 (f_in);
    mid_side = read_uint8 (f_in);
    n_taps = read_uint16(f_in);
    down_sampling = read_uint8(f_in);
    samples_per_packet = read_uint16(f_in);

    if (channels == 0 ||
        (channels<2 && mid_side) ||
	n_taps > max_tap ||
	n_taps == 0 ||
	down_sampling == 0 ||
	samples_per_packet == 0)
      throw error("Bonk file has strange settings");

    predictor.init(n_taps);
    predictor_initer.resize(channels);
    for(int i=0;i<channels;i++) {
      predictor_initer[i].resize(n_taps);
      for(int j=0;j<n_taps;j++)
        predictor_initer[i][j] = 0;
    }

    length_remaining = length;

    bit_in.setup(f_in);
  }

  void read_packet(vector<int> &samples) {
    samples.resize(samples_per_packet*down_sampling*channels);

    vector<int> input_samples(samples_per_packet);

    read_list(predictor.k,false, bit_in);

    predictor.dequantize();

    int quant = (lossless?1:bit_in.read_uint(16)*sample_factor);
 
    for(int channel=0;channel<channels;channel++) {
      int *sample = &(samples[channel]);

      predictor.state = predictor_initer[channel];
      predictor.init_state();

      read_list(input_samples,true, bit_in);
      
      for(int i=0;i<samples_per_packet;i++) {
	for(int j=0;j<down_sampling-1;j++) {
	  *sample = predictor.advance_by_error(0);
	  sample += channels;
	}

	*sample = predictor.advance_by_error(input_samples[i]*quant);
	sample += channels;
      }

      for(int i=0;i<n_taps;i++)
        predictor_initer[channel][i] = 
	  samples[samples.size()-channels+channel-i*channels];
    }

    if (mid_side)
      for(int i=0;i<samples.size();i+=channels) {
	samples[i+1] += shift(samples[i],1);
        samples[i]   -= samples[i+1];
      }  
    
    if (!lossless)
      for(int i=0;i<samples.size();i++)
        samples[i] = shift(samples[i],sample_shift);

    if (length_remaining < samples.size()) {
      samples.resize(length_remaining);
      length_remaining = 0;
    } else
      length_remaining -= samples.size();
  }
};

FILE *open_dsp(int rate,bool stereo) {
  int device = open(dsp_device,O_WRONLY);
  if (device < 0)
    throw error("Couldn't open sound device");
    
  int format = AFMT_S16_NE;
  if (ioctl(device,SNDCTL_DSP_SETFMT,&format) < 0)
    throw error("Couldn't set up sound device");

  int is_stereo = stereo ? 1 : 0;
  if (ioctl(device,SNDCTL_DSP_STEREO,&is_stereo) < 0)
    throw error("Couldn't set up sound device");
  
  if (ioctl(device,SNDCTL_DSP_SPEED,&rate) < 0)
    throw error("Couldn't set up sound device");

  return fdopen(device,"wb"); 
}

bool has_parameter(int &argc,char **&argv,char *name,char *&value) {
  for(int i=1;i<argc-1;i++) {
    if (strcasecmp(argv[i],name) == 0) {
      value = argv[i+1];
      for(int j=i;j<argc-2;j++)
        argv[j] = argv[j+2];
      argc -= 2;
      return true;
    }
  }

  return false;
}

bool has_flag(int &argc,char **&argv,char *name) {
  for(int i=1;i<argc;i++) {
    if (strcasecmp(argv[i],name) == 0) {
      for(int j=i;j<argc-1;j++)
        argv[j] = argv[j+1];
      argc -= 1;
      return true;
    }
  }

  return false;
}

void play_file(char *name) {
  fprintf(stderr,"%s\n",name);

  decoder deco;

  FILE *f_in  = fopen(name,"rb");
  if (!f_in)
    throw error("Couldn't open file");

  deco.begin(f_in);

  if (deco.channels > 2)
    throw error("Don't know how to play more than 2 channels");

  FILE *f_out = open_dsp(deco.rate,deco.channels>1); 

  while(deco.length_remaining) {
    vector<int> samples;
    deco.read_packet(samples);

    vector<int16> little_samples(samples.size());
    for(int i=0;i<samples.size();i++) {
      if (samples[i] >  32767) 
        little_samples[i] =  32767;
      else if (samples[i] < -32768) 
        little_samples[i] = -32768;
      else                          
        little_samples[i] =  samples[i];
    }
    fwrite(&(little_samples[0]),2,little_samples.size(),f_out);
    fflush(f_out);
  }

  fclose(f_in);
  fclose(f_out);
}

void do_play(int argc,char **argv) {
  bool loop = false, random = false;

  for(;;)
    if (has_flag(argc,argv,"-r"))
      random = true;
    else if (has_flag(argc,argv,"-l"))
      loop = true;
    else break;

  if (argc <= 2)
    return;

  do {
    if (random)
      for(int i=2;i<argc;i++) {
        int j = rand()%(argc-2)+2;
	char *temp = argv[i];
	argv[i] = argv[j];
	argv[j] = temp;
      }

    for(int i=2;i<argc;i++) {
      play_file(argv[i]);
    }
  } while(loop);
}

void do_decode(int argc,char **argv) {
  char *out_name;
  bool out_name_specified = false;

  for(;;)
    if (has_parameter(argc,argv,"-o",out_name))
      out_name_specified = true;
    else
      break;

  if (out_name_specified && argc > 3)
    throw error("Can only decode one file at a time while using -o option");

  for(int i=2;i<argc;i++) {
    if (!out_name_specified) {
      out_name = new char[strlen(argv[i])+5];
      strcpy(out_name,argv[i]);
      if (strcasecmp(out_name+strlen(out_name)-5,".bonk") == 0)
        out_name[strlen(out_name)-5] = 0;
      strcat(out_name,".wav");
    }

    fprintf(stderr, "Decoding %s\n"
                    "      to %s\n\n",argv[i],out_name);

    decoder deco;

    FILE *f_in  = fopen(argv[i],"rb");
    if (!f_in)
      throw error("Couldn't open input file");

    deco.begin(f_in);

    FILE *f_out = fopen(out_name,"wb");
    if (!f_out)
      throw error("Couldn't open output file");

    if (!out_name_specified)
      delete out_name;
    
    write_wav_header(f_out,deco.channels,deco.rate,deco.length);

    while(deco.length_remaining) {
      vector<int> samples;
      deco.read_packet(samples);

      for(int i=0;i<samples.size();i++) {
	if (samples[i] >  32767) samples[i] =  32767;
	if (samples[i] < -32768) samples[i] = -32768;
	write_uint16(f_out,int16(samples[i]));
      }
      
      fprintf(stderr,
          "\r%5.1f%% complete ",
          100.0-deco.length_remaining*100.0/deco.length);
      fflush(stderr);
    }

    fclose(f_in);
    fclose(f_out);

    fprintf(stderr,"\r                        \r");
    fflush(stderr);
  }
}

void do_encode(int argc,char **argv) {
  int down_sampling = 2;
  double quantization = 1.0;
  int tap_count = 128;
  bool lossless = false;
  bool mid_side = true;
  char *comment = 0, *artist = 0, *title = 0;

  char *out_name;
  bool out_name_specified = false;

  char *param;
  for(;;)
    if (has_parameter(argc,argv,"-o",param)) {
      out_name = param;
      out_name_specified = true;
    } else if (has_parameter(argc,argv,"-c",param)) {
      comment = param;
    } else if (has_parameter(argc,argv,"-a",param)) {
      artist = param;
    } else if (has_parameter(argc,argv,"-t",param)) {
      title = param;
    } else if (has_parameter(argc,argv,"-q",param)) {
      quantization = atof(param);
      if (quantization < 0.0)
        throw error("Bad quantization level specified");
    } else if (has_parameter(argc,argv,"-d",param)) {
      down_sampling = atoi(param);
      if (down_sampling < 1)
        throw error("Bad down-sampling level specified");
    } else if (has_parameter(argc,argv,"-s",param)) {
      tap_count = atoi(param);
      if (tap_count < 1 || tap_count > max_tap)
        throw error("Bad predictor size specified");
    } else if (has_parameter(argc,argv,"-m",param)) {
      if (strcasecmp(param,"on") == 0)
        mid_side = true;
      else if (strcasecmp(param,"off") == 0)
        mid_side = false;
      else
        throw error("Middle/side option can only be either \"on\" or \"off\"");
    } else if (has_flag(argc,argv,"-l")) {
      lossless = true;
      down_sampling = 1;
      quantization = 0.0;
      tap_count = 32;
    } else
      break;

  if (out_name_specified && argc > 3)
    throw error("Can only encode one file at a time while using -o option");

  string description = "";
  if (artist) {
    description += "Artist: ";
    description += artist;
    description += "\n";
  }

  if (title) {
    description += "Title: ";
    description += title;
    description += "\n";
  }
  
  if (comment) {
    description += comment;
    description += "\n";
  }

  fprintf(stderr,"Quantization options:\n");

  if (lossless)
    fprintf(stderr,"  -l\n\n");
  else
    fprintf(stderr,
         "  -d %d -q %.2f -m %s\n\n",
	 down_sampling,quantization,mid_side?"on":"off");
  
  for(int i=2;i<argc;i++) {
    if (!out_name_specified) {
      out_name = new char[strlen(argv[i])+6];
      strcpy(out_name,argv[i]);
      if (strcasecmp(out_name+strlen(out_name)-4,".wav") == 0)
        out_name[strlen(out_name)-4] = 0;
      strcat(out_name,".bonk");
    }

    fprintf(stderr, "Encoding %s\n"
                    "      to %s\n\n",argv[i],out_name);

    FILE *f_in = fopen(argv[i],"rb");
    if (!f_in)
      throw error("Couldn't open input file");

    int channels,rate,length;
    read_wav_header(f_in,channels,rate,length);

    FILE *f_out = fopen(out_name,"wb");
    if (!f_out)
      throw error("Couldn't open output file");

    if (!out_name_specified)
      delete out_name;

    int packet_size = int(2048.0 *rate/44100);

    encoder enco;
    enco.begin(f_out,
                 description.c_str(),
                 length, rate, channels, lossless, mid_side,
                 tap_count, down_sampling,
	         packet_size/down_sampling,
                 quantization);
    
    vector<int> samples(enco.samples_size); 

    bool first = true;
    int position = 0;
    int n_loops = (length+enco.samples_size-1)/enco.samples_size;
    for(int loop=0;loop<n_loops;loop++) {
      int step = samples.size();
      if (position+step > length)
        step = length-position;

      for(int i=0;i<step;i++)
        samples[i] = int16(read_uint16(f_in));

      if (feof(f_in))
        throw error("Unexpected end of input file");
	
      for(int i=step;i<samples.size();i++)
        samples[i] = 0;

      position += step;

      enco.store_packet(samples);
      
      fprintf(stderr,
          "\r%5.1f%% complete, averaging %3.0f kbps ",
          position*100.0/length,
          double(enco.bit_out.bytes_written)
	  * 8
	  / enco.sample_count
	  * enco.rate * enco.channels
	  / 1024);

      fflush(stderr);
    }

    enco.finish();

    fclose(f_in);
    fclose(f_out);

    fprintf(stderr,"\r                                               \r");
    fprintf(stderr,"Encoded at %.1f:1 compression (%.0f kbps)\n\n",
          length*2.0/enco.bit_out.bytes_written,
	  enco.bit_out.bytes_written*8.0/1024.0/length 
	  *enco.channels*enco.rate);

    fflush(stderr);
  } 
}

int main(int argc,char **argv) {
  try {
    srand(time(0));

    if (argc >= 2 && strcmp(argv[1],"encode") == 0) {
      do_encode(argc,argv);
    } else if (argc >= 2 && strcmp(argv[1],"decode") == 0) {
      do_decode(argc,argv);
    } else if (argc >= 2 && strcmp(argv[1],"play") == 0) {
      do_play(argc,argv);
    } else {
      fprintf(stderr,
             "[ Bonk lossy/lossless audio compressor (version %s) ]\n\n"
	     "Copyright (C) 2001 Paul Harrison\n"
	     "This is free software distributed under the GNU General Public License.\n"
	     "See the source for further information.\n\n"
             "[ Encoding into BONK format ]\n\n"
	     "  bonk encode [options] file1.wav file2.wav ...\n\n"
	     "  Options:\n"
	     "    -q n.nn      Set sample quantization level to n.nn\n"
	     "                   (default 1.00)\n"
	     "    -d n         Set downsampling ratio to n:1\n"
	     "                   (default 2, use 1 to disable)\n"
	     "    -s n         Set predictor size to n\n"
	     "                   (default 128, use 10 for speech)\n"
	     "    -m on        Enable middle/side rather than left/right stereo encoding\n"
	     "                   (default)\n"
	     "    -m off       Disable middle/side encoding\n"
	     "    -l           Lossless encoding mode\n"
	     "    -a \"Blah\"    Specify who the file is by\n"
	     "    -t \"Blah\"    Specify the title of the file\n"
	     "    -c \"Blah\"    Include a comment in the file\n"
	     "    -o file.bonk Write output to file.bonk\n\n"
	     "[ Decoding back to WAV format ]\n\n"
	     "  bonk decode [options] file1.bonk file2.bonk ...\n\n"
	     "  Options:\n"
	     "    -o file.wav  Write output to file.wav\n\n"
	     "[ Playing BONK format files ]\n\n"
	     "  bonk play [options] file1.bonk file2.bonk ...\n\n"
	     "  Options:\n"
	     "    -r           Random order\n"
	     "    -l           Loop\n",
	     version
	     );
    }
    
  } catch(error e) {
    fprintf(stderr,"\nError: %s\n",e.message);
    return 1;
  }
  return 0;
}
