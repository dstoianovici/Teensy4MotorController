#include <Serial_Parser.h>


///////////Constructor////////////
Serial_Parser::Serial_Parser(char delimiter, int range_M, int range_m){
  _delimiter = delimiter;
  _range_M = range_M;
  _range_m = range_m;
  Serial.println("parser created");
}

//////////Methods//////////////////
void Serial_Parser::GetParams(int* nums, int* output){
  String py_data = "/n";
  String sub_string = "/n";
  int index = 0; //Index of the comma
  int i = 0; // itterator for nums array
  int endstring_flag = 1; //Flag for end of string object processing
  int range_flag = 0;
  //int* output; //Slot 0 for number of args parsed, 1 for if in range

  if (Serial.available() > 0){
    py_data = Serial.readString();

    while(endstring_flag == 1){
      index = py_data.indexOf(_delimiter);

      if(index == -1){//No more delimiter
        nums[i] = py_data.toInt();
        if(_range_M < nums[i] || nums[i] < _range_m) range_flag = 1;
        i++;
        endstring_flag = 0;
      }

      else{
        sub_string = py_data.substring(0,index);
        py_data.remove(0,index+1);
        nums[i] = sub_string.toInt();
        if(_range_M < nums[i] ||nums[i] < _range_m) range_flag = 1;
        i++;
        endstring_flag = 1;
      }
    }
  }

  else{
    i = 0;
    range_flag = 0;
    //output[0] = 0;
    //output[1] = 0;
    //return output; //return 0 if
  }

  output[0] = i;
  output[1] = range_flag;

  //return output;
  }
