#include <chrono>
#include <memory>
#include <string>
#include <iostream>

#include <fftw3.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "audio_common_msgs/msg/audio_stamped.hpp"
#include "audio_common_msgs/msg/audio.hpp"
#include "sound_signal_msgs/msg/sound_signal_with_freq.hpp"

using namespace std::chrono_literals;

/* 
* This class processes sound from the microphone using a fft to identify two patterns
* It breaks a 1.5 second window of sound into six 0.25 second blocks.
* Each block is assigned a 1 if the alert frequency is sufficiently loud in that block 
* and a zero otherwise. 
* If the pattern 111100 occurs this is pattern A (one second blast) and we publish that
* If the pattern 110011 occurs this is pattern B (two 0.5 second blasts) and we publish that
* After we detect a sound signal there is a 1.5 sceond cool down before we publish again to 
* avoid detecting the same signal multiple times. This is okay beucase there is supposed to 
* be two seconds between signals. 
* */

class SoundSignal : public rclcpp::Node
{
public:
  SoundSignal() : Node("sound_signal_node")
  {
  // declare parameters with default
//  this->declare_parameter<int>("format", paInt16); not implemented
  this->declare_parameter<int>("frequency", 800);
  this->declare_parameter<int>("rate", 16000);
  this->declare_parameter<int>("chunk", 500); // this is not standard, you have to adjust the paraeter in audio_capturer to match
  this->declare_parameter<int>("sensitivity", 10);

  // get paramter values
//  this->format_ = this->get_parameter("format").as_int(); not implemented
  this->frequency_ = this->get_parameter("frequency").as_int();
  this->rate_ = this->get_parameter("rate").as_int();
  this->chunk_ = this->get_parameter("chunk").as_int();
  this->sensitivity_ = this->get_parameter("sensitivity").as_int();

    count_ = 3 * rate_ /2;
    delay_flag_ = 0;
    in_buffer_ = fftw_alloc_complex(rate_ * 3); // we want to be buffering for 3 seconds
    out_buffer_ = fftw_alloc_complex(rate_ * 3 / 2);
    int rank = 1; /* not 2: we are computing 1d transforms */
    int n[] = {rate_/4}; /* 1d transforms of length 0.25 seconds */
    int howmany = 6;
    int idist = n[0];    
    int odist = n[0]; /* distance between arrays to transform */
    int istride = 1 ;
    int ostride = 1; /* distance between two elements in the same column */
    int *inembed = n;
    int *onembed = n;
    fftw_plan_ = fftw_plan_many_dft(rank, n, howmany, in_buffer_, inembed, istride, idist, out_buffer_, onembed, ostride, odist, FFTW_FORWARD, FFTW_ESTIMATE);
    interupt_publisher_ = this->create_publisher<std_msgs::msg::Int32>("sound_signal_interupt", 10);
    interupt_publisher_freq_ = this->create_publisher<sound_signal_msgs::msg::SoundSignalWithFreq>("sound_signal_interupt_freq", 10);
    audio_subscription_ = this->create_subscription<audio_common_msgs::msg::AudioStamped>("audio", rclcpp::SensorDataQoS(), std::bind(&SoundSignal::topic_callback, this, std::placeholders::_1));

    delay_timer_ = this->create_wall_timer(1.5s, [this]() {
				        delay_flag_ = 0;
					this->delay_timer_->cancel();
			  		});
        // cancel immediately to prevent it running the first time.
     delay_timer_->cancel();
  }

private:
  inline double abs(fftw_complex z){
	return sqrt(z[0]*z[0] + z[1]*z[1]);
  } 

  int hamming(int a, int b){
	int diff = a ^ b;
	if (diff == 0)
		return 0;
	//if ((diff & (diff - 1)) == 0)
	//	return 1;
	return 2;
  }

  void topic_callback(const audio_common_msgs::msg::AudioStamped::SharedPtr msg) {
  	  // buffer the lastest message
	  for (int i = count_; i < count_ + chunk_; i++){
	  	short val = msg->audio.audio_data.int16_data[i - count_];
		in_buffer_ [i][0] = val;
		in_buffer_ [i][1] = 0;
	  }
	  count_ += chunk_;
	  
	  // compute the fft on the right part of the array
	  fftw_execute_dft(fftw_plan_, (fftw_complex*)in_buffer_[count_ - 3*rate_/2], out_buffer_);

	  // process the results of the transform
	  int bin_width = rate_/4;
	  int centre_bin = frequency_ * bin_width / rate_;
	  int output = 0;
	  for (int i = 0; i < 6; i++) {
		int target_magnitude = 0;
		for (int j = -10; j<=10; j++) {
			int a = abs(out_buffer_[i * bin_width + centre_bin + j]);
			target_magnitude = (a > target_magnitude) ? a : target_magnitude;
		}
//          	RCLCPP_INFO_STREAM(this->get_logger(), "centre_bin: " << centre_bin);
//          	RCLCPP_INFO_STREAM(this->get_logger(), "target: " << target_magnitude);
	  	int neighbors = 0;
//	 	std::string str = "";
		  for(int j = 12; j < 20; j++){
			neighbors += abs(out_buffer_[i * bin_width + centre_bin - j]);
//			str = str + ", " + std::to_string(abs(buffer_[centre_bin - 5*i]));
			neighbors += abs(out_buffer_[i * bin_width + centre_bin + j]);
//		str = str + ", " + std::to_string(abs(buffer_[centre_bin + 5*i]));
		  }
//		  for (int j = centre_bin-40; j < centre_bin+40; j++){			 
//			str = str + ", " + std::to_string(abs(out_buffer_[i * bin_width + j]));
//	 	 }
		  neighbors /= 16;
//        	  RCLCPP_INFO_STREAM(this->get_logger(), "neighbors: " << str);
		  if (target_magnitude > sensitivity_*neighbors){
			  output += (1 << i);
		  }
	  }
        	  //RCLCPP_INFO_STREAM(this->get_logger(), "output: " << output);
	  // check for a match
	  //RCLCPP_INFO_STREAM(this->get_logger(), hamming(output, 0b110011));
	  if (hamming(output, 0b110011) <= 1 && delay_flag_ != 1) {
		// Detector disabled for single-blast pattern: do not publish any messages
		delay_flag_ = 1;
		delay_timer_->reset();
	  } else if (hamming(output, 0b001111) <=1 && delay_flag_ != 1) {
		// Detector disabled for double-blast pattern: do not publish any messages
		delay_flag_ = 1;
		delay_timer_->reset();
	  }
	  	  // shift everything down if you're at the end of the buffer
	  if (count_ == rate_*3) {
		// shift eveyrthing down by hald the array
	  	memcpy(in_buffer_, in_buffer_[3 * rate_/2], sizeof(fftw_complex) * 3 * rate_/2);
	  	count_ = 3 * rate_/2;
	  }
  }
  rclcpp::Subscription<audio_common_msgs::msg::AudioStamped>::SharedPtr audio_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr delay_timer_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr interupt_publisher_;
  rclcpp::Publisher<sound_signal_msgs::msg::SoundSignalWithFreq>::SharedPtr interupt_publisher_freq_;
  fftw_complex* in_buffer_; 
  fftw_complex* out_buffer_; 
  fftw_plan fftw_plan_;
  int frequency_;
  int chunk_;
  int rate_;
  int sensitivity_;
  int count_;
  int delay_flag_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SoundSignal>());
  rclcpp::shutdown();
  return 0;
}
