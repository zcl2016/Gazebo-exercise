#include "pseudo_controller_example.h"
//modified by kobe
namespace hector
{

pseudo_controller_example::pseudo_controller_example() : pseudo_controller_base()
{
    current_zone = alt_zones::PREPARE;
    //current_zone = alt_zones::TAKE_OFF;
    c_error_ = 0;
    c_integrator_ = 0;
    r_error_ = 0;
    r_integrator_ = 0;
    a_error_ = 0;
    a_integrator_ = 0;
    at_error_ = 0;
    at_integrator_ = 0;
    ap_error_ = 0;
    ap_integrator_ = 0;
}

void pseudo_controller_example::control(const params_s &params, const input_s &input, output_s &output)
{
    output.up_down_trans = 0;
    output.forward_trans = 0;
    output.rotate_trans = 0;
    output.left_right_trans = 0;

    output.rotate_value=pseudo_course(input.chi, input.chi_c);
    output.rotate_trans = output.rotate_value;//pseudo_course_hold(input.chi, input.chi_c, params, input.Ts);
    
    
    if (input.landing==true) // protect-landing first
    {
        current_zone = alt_zones::LANDING;
    }
    if (input.takeoff==true && current_zone==alt_zones::LANDING)
    {
        current_zone = alt_zones::PREPARE;
    }

    switch (current_zone)
    {
    case alt_zones::PREPARE:
        output.up_down_trans = 0.5;
        output.forward_trans = 0;
        output.left_right_trans = 0;
        output.rotate_trans=0;
        if (input.h >= 1) //
        {
            ROS_DEBUG("TAKEOFF");
            current_zone = alt_zones::TAKE_OFF;
        }
        break;
    case alt_zones::TAKE_OFF:
        //output.rotate_trans = 1.5*pseudo_course_hold(input.chi, input.chi_c, params, input.Ts);//recover
        //output.forward_trans = 0.7;//0.1 * pseudo_throttle_hold(input.Va_c, input.va, params, input.Ts);
        output.forward_trans = input.Va_c;//pseudo_throttle_hold(input.Va_c, input.va, params, input.Ts);
        output.up_down_trans = pow(1.1,input.h)*pseudo_pitch_hold(input.h_c, input.h, params, input.Ts);
        //output.up_down_trans = 0.01 * pseudo_pitch_hold(input.h_c, input.h, params, input.Ts);
        //if (input.h >= params.alt_toz) //alt_toz=1.5//recovery
        //output.left_right_trans = 1.5*pseudo_left_right(input.chi, input.chi_c, output.forward_trans);
        if (input.h >= 1.5) //
        {
            ROS_DEBUG("climb");
            current_zone = alt_zones::CLIMB;
            ap_error_ = 0;
            ap_integrator_ = 0;
            ap_differentiator_ = 0;
        }
        break;
    case alt_zones::CLIMB:
        output.forward_trans = input.Va_c;//sat(input.Va_c*cos(output.rotate_trans),5,0.5);//pseudo_throttle_hold(input.Va_c, input.va, params, input.Ts);
        //output.forward_trans = 0.8;//0.1 * pseudo_throttle_hold(input.Va_c, input.va, params, input.Ts);
        output.left_right_trans = 0;//sat(input.Va_c*sin(output.rotate_trans),5,-5);//pseudo_left_right(input.chi, input.chi_c, output.forward_trans);
        output.up_down_trans =  pseudo_pitch_hold(input.h_c, input.h, params, input.Ts);
        output.rotate_trans = output.rotate_value;//pseudo_course_hold(input.chi, input.chi_c, params, input.Ts);
        
        //output.left_right_trans = 0.1 * pseudo_left_right(input.chi, input.chi_c, output.forward_trans);
        //output.up_down_trans = 0.01 * pseudo_pitch_hold(input.h_c, input.h, params, input.Ts);
         //output.rotate_trans = 0.3*pseudo_course_hold(input.chi, input.chi_c, params, input.Ts);
        ////if (input.h >= input.h_c - params.alt_hz) //alt_hz=0.2//recovery
        if (input.h >= input.h_c - 0.3) //alt_hz=0.2
        {
            ROS_DEBUG("hold");
            current_zone = alt_zones::ALTITUDE_HOLD;
            at_error_ = 0;
            at_integrator_ = 0;
            at_differentiator_ = 0;
            a_error_ = 0;
            a_integrator_ = 0;
            a_differentiator_ = 0;
        }
        //else if (input.h <= params.alt_toz) //1.5//recovery
        // else if (input.h <= 1) //1.5
        // {
        //     ROS_DEBUG("takeoff");
        //     current_zone = alt_zones::TAKE_OFF; //  Is that necessary?
        // }
        break;
    case alt_zones::DESCEND:
        output.forward_trans = input.Va_c;//sat(input.Va_c*cos(output.rotate_trans),5,0.5);//pseudo_throttle_hold(input.Va_c, input.va, params, input.Ts);
        output.left_right_trans = 0;//sat(input.Va_c*sin(output.rotate_trans),5,-5);//pseudo_left_right(input.chi, input.chi_c, output.forward_trans);
        output.rotate_trans = output.rotate_value;//pseudo_course_hold(input.chi, input.chi_c, params, input.Ts);
        output.up_down_trans = pseudo_pitch_hold(input.h_c, input.h, params, input.Ts);

        //output.forward_trans = 0.1 * pseudo_throttle_hold(input.Va_c, input.va, params, input.Ts);
        //output.left_right_trans = 0.1 * pseudo_left_right(input.chi, input.chi_c, output.forward_trans);
        //output.rotate_trans = pseudo_course_hold(input.chi, input.chi_c, params, input.Ts);
        //output.up_down_trans = 0.1 * pseudo_pitch_hold(input.h_c, input.h, params, input.Ts);
        //if (input.h <= input.h_c + params.alt_hz)//recovery
        if (input.h <= input.h_c + 0.3)
        {
            if (input.h < 0) //modify later
            {
                ROS_DEBUG("landing");
                current_zone = alt_zones::LANDING;
                // output.forward_trans = sat(input.Va_c*cos(output.rotate_trans),1,0);//pseudo_throttle_hold(input.Va_c, input.va, params, input.Ts) * 0.5;
                // output.up_down_trans = pseudo_pitch_hold(input.h_c, input.h, params, input.Ts) / 2;
                // output.rotate_trans = output.rotate_value;//pseudo_course_hold(input.chi, input.chi_c, params, input.Ts);
            }
            else
            {
                ROS_DEBUG("hold");
                current_zone = alt_zones::ALTITUDE_HOLD;
                at_error_ = 0;
                at_integrator_ = 0;
                at_differentiator_ = 0;
                a_error_ = 0;
                a_integrator_ = 0;
                a_differentiator_ = 0;
            }
        }
        break;
    case alt_zones::ALTITUDE_HOLD:
        output.forward_trans = input.Va_c;//sat(input.Va_c*cos(output.rotate_trans),5,0.5);//pseudo_throttle_hold(input.Va_c, input.va, params, input.Ts);
        output.left_right_trans = 0;//sat(input.Va_c*sin(output.rotate_trans),5,-5);//0.5*pseudo_left_right(input.chi, input.chi_c, output.forward_trans);
        output.up_down_trans = pseudo_altitiude_hold(input.h_c, input.h, params, input.Ts);
        output.rotate_trans = output.rotate_value;//pseudo_course_hold(input.chi, input.chi_c, params, input.Ts);
        //output.forward_trans = 1;//0.1 * pseudo_throttle_hold(input.Va_c, input.va, params, input.Ts);
        //output.left_right_trans = pseudo_left_right(input.chi, input.chi_c, output.forward_trans);
        //output.up_down_trans = 0.1 * pseudo_altitiude_hold(input.h_c, input.h, params, input.Ts);
        //output.rotate_trans = 0.7*pseudo_course_hold(input.chi, input.chi_c, params, input.Ts);
        //if (input.h >= input.h_c + params.alt_hz)//recovery
        if (input.h >= input.h_c + 0.3)
        {
            ROS_DEBUG("desend");
            current_zone = alt_zones::DESCEND;
            ap_error_ = 0;
            ap_integrator_ = 0;
            ap_differentiator_ = 0;
        }
        //else if (input.h <= input.h_c - params.alt_hz)//recovery
        else if (input.h <= input.h_c - 0.3)
        {
            ROS_DEBUG("climb");
            current_zone = alt_zones::CLIMB;
            ap_error_ = 0;
            ap_integrator_ = 0;
            ap_differentiator_ = 0;
        }
        break;
    case alt_zones::LANDING:
            ROS_DEBUG("landing");
            current_zone = alt_zones::LANDING;
            ap_error_ = 0;
            ap_integrator_ = 0;
            ap_differentiator_ = 0;
            output.forward_trans = 0;
            if (input.h < 0.6)
            {
                output.up_down_trans = 0;
            }
            else
            {
                output.up_down_trans = -input.h/3;
            }

            output.rotate_trans =0;
            output.left_right_trans = 0;
        break;
    default:
        break;
    }

    output.current_zone = current_zone;
    //output.delta_e = pitch_hold(output.theta_c, input.theta, input.q, params, input.Ts);
    //output.up_down_trans=pseudo_pitch_hold(output.theta_c, input.theta, input.q, params, input.Ts);
}

float pseudo_controller_example::pseudo_course(float chi, float chi_c)
{
    chi_c=sat(chi_c,3.14159,-3.14159);
    chi=sat(chi,3.14159,-3.14159);
    float rotate = sat(chi_c-chi,2*3.14159,-2*3.14159);//-pi to pi
    // float lr;
    // if (rotate>3.14159)
    // {
    //     lr=-1;
    // }
    // else if(rotate<-3.14159)
    // {
    //     lr=1;
    // }
    //rotate=rotate+lr*2*3.14159;//more easier to rotate

    // rotate=sat(rotate,3.14159/4,-3.14159/4);//limit
    return rotate;
}

float pseudo_controller_example::pseudo_left_right(float chi, float chi_c, float forward_trans)
{
    float  error= chi_c-chi;
    float a = fabs(forward_trans);
    float x = (3.14 / 4 > fabs(error)) ? fabs(error) : 3.14 / 4;
    float b = tan(x);
    float c = error / (fabs(error + 0.001));
    float left_right_trans = sat(a * b * c, forward_trans, -forward_trans);

    return left_right_trans;
}

float pseudo_controller_example::pseudo_course_hold(float chi, float chi_c, const params_s &params, float Ts)
{
    float error = chi_c-chi;

    //c_integrator_ = c_integrator_ + (Ts / 2.0) * (error + c_error_);

    //float up = params.c_kp * error;
    //float ui = params.c_ki * c_integrator_;

    float rotate_trans = sat((error + c_error_) / 2, 45.0 * 3.14 / 180.0, -45.0 * 3.14 / 180.0);
    //if (fabs(params.c_ki) >= 0.00001)
    //{
    //float rotate_trans_unsat = up + ui;
    //c_integrator_ = c_integrator_ + (Ts / params.c_ki) * (rotate_trans - rotate_trans_unsat);
    //}

    c_error_ = error;
    return rotate_trans;
}

float pseudo_controller_example::pseudo_pitch_hold(float h_c, float h, const struct params_s &params, float Ts)
{
    float error = h_c - h;
    //ap_integrator_ = ap_integrator_ + (Ts / 2.0) * (error + ap_error_);
    // float up = 0.5 * error;
    //float ui = params.a_p_ki * ap_integrator_;
    //float up_down_trans = sat(up + ui, 0.8, -0.8);//recovery
    float up_down_trans = sat(0.5 * error, 2, -2);
    //if (fabs(params.a_p_ki) >= 0.00001)
    //{
        //float up_down_trans_unsat = up + ui;
        //ap_integrator_ = ap_integrator_ + (Ts / params.a_p_ki) * (up_down_trans - up_down_trans_unsat);
    //}
    // ap_error_ = error;
    return up_down_trans;
}

float pseudo_controller_example::pseudo_throttle_hold(float Va_c, float Va, const params_s &params, float Ts)
{
    float error = Va_c - Va;

    //at_integrator_ = at_integrator_ + (Ts / 2.0) * (error + at_error_);
    //at_differentiator_ = (2.0 * params.tau - Ts) / (2.0 * params.tau + Ts) * at_differentiator_ + (2.0 /(2.0 * params.tau + Ts)) *(error - at_error_);

    float up = params.a_t_kp * error;
    //float ui = params.a_t_ki * at_integrator_;
    //float ud = params.a_t_kd * at_differentiator_;

    //float forward_trans = sat(Va + up + ui + ud, params.max_t, 0.1);
    float forward_trans = sat(Va+error, params.max_t, 0.5);
    //if (fabs(params.a_t_ki) >= 0.00001)
    //{
    //float forward_trans_unsat = params.trim_t + up + ui + ud;
    //at_integrator_ = at_integrator_ + (Ts / params.a_t_ki) * (forward_trans - forward_trans_unsat);
    //}

    //at_error_ = error;
    return forward_trans;
}

float pseudo_controller_example::pseudo_altitiude_hold(float h_c, float h, const params_s &params, float Ts)
{
    float error = h_c - h;

    //a_integrator_ = a_integrator_ + (Ts / 2.0) * (error + a_error_);
    //a_differentiator_ = (2.0 * params.tau - Ts) / (2.0 * params.tau + Ts) * a_differentiator_ + (2.0 /(2.0 * params.tau + Ts)) * (error - a_error_);
    float up = 0.7 * error;
    //float up = params.a_kp * error;
    //ROS_INFO("hello %f",params.a_kp);
    //float ui = params.a_ki * a_integrator_;
    //float ud = params.a_kd * a_differentiator_;

    //float up_down_trans = sat(up + ui + ud, 0.8, -0.8);//recovery
    float up_down_trans = sat(up, 2, -2);
    //if (fabs(params.a_ki) >= 0.00001)
    //{
        //float up_down_trans_unsat = up + ui + ud;
        //a_integrator_ = a_integrator_ + (Ts / params.a_ki) * (up_down_trans - up_down_trans_unsat);
    //}

    // at_error_ = error;
    return up_down_trans;
}

//float pseudo_controller_example::cooridinated_turn_hold(float v, const params_s &params, float Ts)
//{
//    //todo finish this if you want...
//    return 0;
//}

float pseudo_controller_example::sat(float value, float up_limit, float low_limit)
{
    float rVal;
    if (value > up_limit)
        rVal = up_limit;
    else if (value < low_limit)
        rVal = low_limit;
    else
        rVal = value;

    return rVal;
}

} // namespace hector
