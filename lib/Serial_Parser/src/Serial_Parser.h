#ifndef SERIAL_PARSER_H
#define SERIAL_PARSER_H

#include <Arduino.h>


class Serial_Parser
{
  public:
    Serial_Parser(char delimiter, int range_M, int range_m);
    // int GetNumParams();
    void GetParams(int* nums, int* output);


  private:
    // long _baud_rate;
    char _delimiter;
    int _range_M;
    int _range_m;

};

#endif
