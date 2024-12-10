#ifndef WAV_H
#define WAV_H

#include <stdio.h>
#include "utility.h"

bool read_pattern(FILE *file,char *pattern) {
  while(*pattern) {
    if (fgetc(file) != *pattern)
      return false;
    pattern++;
  }
  return true;
}

void read_wav_header(FILE *file, int &n_channel, int &sampling_rate, int &length) {
  //Read RIFF chunk
  if (!read_pattern(file,"RIFF"))
    throw error("File is not in WAV format");
  
  for(int i=0;i<4;i++)
    fgetc(file);
  
  if (!read_pattern(file,"WAVE"))
    throw error("File is not in WAV format");
    
  //Read FMT chunk
  for(int i=0;i<10;i++)
    fgetc(file);

  n_channel = read_uint16(file);
  sampling_rate = read_uint32(file);

  for(int i=0;i<8;i++)
    fgetc(file);
    
  //Read DATA chunk
  if (!read_pattern(file,"data"))
    throw error("File is not in WAV format");

  length = read_uint32(file) / 2;
}

void write_wav_header(FILE *file, int channels, int rate, int samples) {
  int data_size = samples*2;

  fputc('R',file); fputc('I',file); fputc('F',file); fputc('F',file);
  write_uint32(file, data_size+8+16+12);

  fputc('W',file); fputc('A',file); fputc('V',file); fputc('E',file);
  fputc('f',file); fputc('m',file); fputc('t',file); fputc(' ',file);

  write_uint32(file, 16);
  write_uint16(file, 1) ; //format = PCM
  write_uint16(file, channels);
  write_uint32(file, rate);
  write_uint32(file, rate*channels*2);
  write_uint16(file, channels*2); //block align
  write_uint16(file, 16); //bits per sample

  fputc('d',file); fputc('a',file); fputc('t',file); fputc('a',file);
  write_uint32(file, data_size);
}

#endif /* WAV_H */
