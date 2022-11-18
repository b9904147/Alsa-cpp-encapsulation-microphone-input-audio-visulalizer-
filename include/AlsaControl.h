#ifndef ALSA_ENCAPSULATION_ALSA_CONTROL_H_
#define ALSA_ENCAPSULATION_ALSA_CONTROL_H_

#include <iostream>
#include <future>
#include <functional>
#include <alsa/asoundlib.h>
#include <WavFunctions.h>
#include <fftw3.h>

#define STEREO 2
#define MONO 1

enum {re, im}; //real and imaginary

struct FFTW_Results
{
	double* peakfreq;
	double* peakpower;
	double** peakmagMatrix;
	char*** outputMatrix;
	double phase;
};

struct FFTWop
{
	fftw_complex *in;
	fftw_complex *out;
	fftw_plan p;
	int index;
};

class AlsaControl {
public:
  void ShowALSAParameters();
  void Listen();
  void Listen(std::string filename);
  void ListenWithCallback(std::function<void(void *, int)> func);
  void ListenWithCallback(std::function<void(void *, int)> func,
      std::string filename);
  void RecordToFile(std::string filename, int const &duration_in_us);

  void ForcePeriodSize(int const &value);

  void Stop();

  AlsaControl(unsigned int const &rate, unsigned long const &frames,
      int const &bits, unsigned int const &stereo_mode);
  virtual ~AlsaControl();

private:
  unsigned int rate_;
  snd_pcm_uframes_t frames_;
  int bits_;
  unsigned int stereo_mode_;
  int packet_pos = 0;

  unsigned int time_period_;
  snd_pcm_hw_params_t *params_;
  snd_pcm_t *handle_;
  snd_pcm_uframes_t period_size_;
  const int BUCKETS = 5;

  std::atomic<bool> continue_listening_;
  std::future<void> thread_;

  unsigned int totalpackets;
  struct FFTWop* fftw_ptr;
  struct FFTW_Results *FFTW_Results_ptr;

  void OpenPcmDevice();
  void SetParametersALSA();
  void allocate_FFTWop();
  void allocate_FFTW_Results();
  void SetupDFTForSound(unsigned char *buffer, int bytesRead);
  void analyze_FFTW_Results(struct FFTWop fftwop , int packet_index, int ch,size_t bytesRead);
  void outputpowerspectrum();

  double Get16bitAudioSample(unsigned char* bytebuffer);

  void ThreadListen(std::string filename);
  void ThreadListenWithCallback(std::function<void(void *, int)> func,
      std::string filename);
  void ThreadRecordToFile(std::string filename, int const &duration_in_us);
  
  AlsaControl() = delete;
  AlsaControl(const AlsaControl &) = delete;
};

#endif //ALSA_ENCAPSULATION_ALSA_CONTROL_H_
