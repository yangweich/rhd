# ifndef SIMULINKIF_DOT_H
# define SIMULINKIF_DOT_H

extern "C"
{
# include "rhd.h"
}
# define NAMELENGTH 256

typedef struct
{
  const char *name;
  int size;
} rhd_signal;

class rhd_data
{
  public:
    double ts_now;
    double ts_pre;
    
    unsigned int input_num, output_num;
    rhd_signal input_sig[1], output_sig[4];
        
    rhd_data()
    {
      input_num = 1;
      input_sig[0].name = "speedyaw";
      input_sig[0].size = 1;
      
      output_num = 4;
      output_sig[0].name = "tick";
      output_sig[0].size = 1;
      output_sig[1].name = "encyaw";
      output_sig[1].size = 1;
      output_sig[2].name = "rudder";
      output_sig[2].size = 2;
      output_sig[3].name = "rudderN";
      output_sig[3].size = 1;
    };
    
    ~rhd_data();
};

# endif  // SIMULINKIF_DOT_H
