#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include<chrono>
using namespace std::chrono_literals;
#include <functional>
#include <memory>

#include <iostream>
using namespace std;
#include <fstream>
#include <cstdlib>
#include <ctime>
#include <cmath>

double limit(double value,double max,double min){
    if(value>max){
        return max;
    }
    if(value<min){
        return min;
    }
         return value;
    };

double noise(double noise_amplitude){
    double randomnum=(double)rand()/RAND_MAX;
    double randomnum_f1_1=randomnum*2-1;
    double noise=randomnum_f1_1*noise_amplitude;
    return noise;
}

class pid{
    private:
    double kp;
    double ki;
    double kd;
    double p_term;
    double i_term;
    double d_term;
    double integral=0;
    double previous_error=0;
    int first=1;
    double lvbo_derivate=0;
    double a=0.85;
    double previous_output=0;
    double output_rate=3;
    double deadband=0.02;
    double integral_max=20;
    double integral_min=-20;
    double output_max=5;
    double output_min=-5;
    public:
    pid(double p,double i ,double d){
        kp=p;
        ki=i;
        kd=d;
    };
    void setIntegralLimit(double max,double min){
        integral_max=max;
        integral_min=min;
    }
    void setOutputLimit(double max,double min){
        output_max=max;
        output_min=min;
    }
    void setDerivativeFilter(double value){
        a=value;
    }
    void setOutputRate(double value){
        output_rate=value;
    }
    void setDeadband(double value){
        deadband=value;
    };
    void reset(){
    integral = 0;
    previous_error = 0;
    first = 1;
    lvbo_derivate = 0;
    previous_output = 0;

    p_term = 0;
    i_term = 0;
    d_term = 0;}
    double output(double target,double current,double dt){
        if(dt<=0){
            cout<<"pid"<<endl;
            return 0;
        }
        double error=target-current;
        integral=integral+error*dt;
        integral=limit(integral,integral_max,integral_min);
        if (fabs(error) < deadband)
        {
            error = 0.0;
        }
        if (first==1){
            previous_error=error;
        };
        double derivate=(error-previous_error)/dt;
        lvbo_derivate=lvbo_derivate*a+(1-a)*derivate;
        double output1=kp*error+ki*integral+kd*lvbo_derivate;
        p_term=kp*error;
        i_term=ki*integral;
        d_term=kd*lvbo_derivate;
        previous_error=error;
        output1=limit(output1,output_max,output_min);
        double maxoutput_change=output_rate*dt;
        double output_change=output1-previous_output;
        output_change=limit(output_change,maxoutput_change,-maxoutput_change);
        output1=previous_output+output_change;
        previous_output=output1;
        first++;
        return output1;
    };
    double getP()
    {
        return p_term;
    }
    double getI()
    {
        return i_term;
    };
    double getD()
    {
        return d_term;
    };
};
   
class auvmodel{
    private:
    double  current;
    double velocity;
    double damp;
    double disturbance;
    public:
    auvmodel(double c,double v,double d){
        current=c;
        velocity=v;
        damp=d;
        disturbance=0.2;
    };
    double update(double output,double dt){
        double accleration=output-damp*velocity+disturbance;
        velocity=velocity+accleration*dt;
        current=current+velocity*dt;
        return current;
    };
    double getdepth(){
        return current;
    };    
    double getvelocity(){
        return velocity;
    };
    void setDamp(double d){
        damp = d;
    }
    void setDisturbance(double dis){
        disturbance = dis;
    }
};

class PidNode :  public rclcpp::Node{
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr current_trueposition_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr measure_position_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr lvbo_posiition_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr velocity_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr newposition_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr output_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr error_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr p_term_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr i_term_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr d_term_pub_;
    double target=10.0;
    double dt=0.1;
    double total_time = 100;
    int steps = total_time / dt;
    pid p= pid(0.25,0.015,0.8);
    auvmodel auv=auvmodel(0.0,0,0.8);
    double noise_amplitude = 0.03;
    double b=0.9;
    int i=0;
    double lvbo_posiition=0;
    ofstream file; 
    public:
    
    PidNode(): Node("auv_pid_node"){
    srand(time(0));
    p.setIntegralLimit(20,-20);
    p.setOutputLimit(5,-5);
    p.setDerivativeFilter(0.85);
    p.setOutputRate(3);
    p.setDeadband(0.02);
    auv.setDamp(0.8);
    auv.setDisturbance(0.2);
    lvbo_posiition=auv.getdepth();
        
    current_trueposition_pub_ = this->create_publisher<std_msgs::msg::Float64>("/auv/current_trueposition",10);
    measure_position_pub_ = this->create_publisher<std_msgs::msg::Float64>("/auv/measure_position",10);
    lvbo_posiition_pub_ = this->create_publisher<std_msgs::msg::Float64>("/auv/lvbo_posiition",10);
    velocity_pub_ = this->create_publisher<std_msgs::msg::Float64>("/auv/velocity",10);
    newposition_pub_ = this->create_publisher<std_msgs::msg::Float64>("/auv/newposition",10);
    output_pub_ = this->create_publisher<std_msgs::msg::Float64>("/auv/output",10);
    error_pub_ = this->create_publisher<std_msgs::msg::Float64>("/auv/error",10);
    p_term_pub_ = this->create_publisher<std_msgs::msg::Float64>("/auv/p_term",10);
    i_term_pub_ = this->create_publisher<std_msgs::msg::Float64>("/auv/i_term",10);
    d_term_pub_ = this->create_publisher<std_msgs::msg::Float64>("/auv/d_term",10);
    std::string csv_filename =
    "auv_pid_" +
    std::to_string(static_cast<int>(target)) +
    "m.csv";

file.open(csv_filename, std::ios::out);
    if(!file.is_open()){
        cout<<"csv is fail"<<endl;
    }
    file << "time,current_trueposition,measure_position,lvbo_posiition,velocity,newposition,output,error,p_term,i_term,d_term"<<endl;
    timer_ = this->create_wall_timer(100ms,std::bind(&PidNode::timer_callback, this));
    }
    private:
    
    void publish_double(rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub,double value)
{
    std_msgs::msg::Float64 msg;
    msg.data = value;
    pub->publish(msg);
};

    void timer_callback()
{
      if(i >= steps)
    {
        RCLCPP_INFO(this->get_logger(), "simulation finished");
        file.close();
        timer_->cancel();
        return;
    }
        double time=i*dt;
        double current_trueposition=auv.getdepth();
        double velocity=auv.getvelocity();
        double measure_position=current_trueposition+noise(noise_amplitude);
        lvbo_posiition=lvbo_posiition*b+(1-b)*measure_position;
        double output=p.output(target, lvbo_posiition,dt);
        double p_term = p.getP();
        double i_term = p.getI();
        double d_term = p.getD();
        double newposition=auv.update(output,dt);
        double error=target-newposition;
        
        publish_double(current_trueposition_pub_,current_trueposition);
        publish_double(measure_position_pub_,measure_position);
        publish_double(lvbo_posiition_pub_,lvbo_posiition);
        publish_double(velocity_pub_,velocity);
        publish_double(newposition_pub_,newposition);
        publish_double(output_pub_,output);
        publish_double(error_pub_,error);
        publish_double(p_term_pub_,p_term);
        publish_double(i_term_pub_,i_term);
        publish_double(d_term_pub_,d_term);


        cout<<time<<" "
            <<current_trueposition<<" "
            <<measure_position<<" "
            <<lvbo_posiition<<" "
            <<output<<" "
            <<velocity<<" "
            <<newposition<<" "
            <<error<<" "
            <<p_term<<" "
            <<i_term<<" "
            <<d_term<<endl;

        if(file.is_open()){
            file << time << ","
                 << current_trueposition << ","
                 << measure_position << ","
                 << lvbo_posiition << ","
                 << velocity << ","
                 << newposition << ","
                 << output << ","
                 << error << ","
                 << p_term << ","
                 << i_term << ","
                 << d_term << endl;
        };
        i++;
    };
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PidNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
