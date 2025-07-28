//
// Run func

float vac_sen = InputSignal(0, 0);
float deta_theta = acos(InputSignal(0, 1));
float temp_theta= 0.0f;
//
//run sogi spll func
//
spll_sogi_func(&spll_data, vac_sen);

temp_theta = spll_data.theta-deta_theta;  //感性加 容性减



  if(temp_theta > value_2pi)
    {
        temp_theta -= value_2pi;
    }
    else if (temp_theta < 0)
    {
        temp_theta += value_2pi;
    }

//
//debug
//
    OutputSignal(0, 0) = temp_theta;
    OutputSignal(0, 1) = spll_data.pll_freq_out;
    OutputSignal(0, 2) =spll_data.spll_diff;