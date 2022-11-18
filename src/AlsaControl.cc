#include <math.h>
#include "AlsaControl.h"

AlsaControl::AlsaControl(unsigned int const& rate, unsigned long const& frames,
    int const& bits, unsigned int const& stereo_mode)
    :
    rate_(rate),
    frames_(frames),
    bits_(bits),
    stereo_mode_(stereo_mode) {
    
  continue_listening_.store(false, std::memory_order_relaxed);
  totalpackets = (8 * (rate_ * bits_ / 8)) / ((bits_ / 8) * stereo_mode_ * frames_);
  OpenPcmDevice();
  snd_pcm_hw_params_alloca(&params_);
  SetParametersALSA();
  allocate_FFTWop();
  allocate_FFTW_Results();
}

AlsaControl::~AlsaControl() {
  snd_pcm_drain(handle_);
  snd_pcm_close(handle_);

  if (continue_listening_.load(std::memory_order_relaxed)) {
    std::cout << std::endl << "ERROR - All process seems not finished" <<
              std::endl;
    exit(1);
  }
}

void AlsaControl::ShowALSAParameters() {
  int val;

  std::cout << "ALSA library version: " << SND_LIB_VERSION_STR << std::endl;

  std::cout << std::endl << "PCM stream types:" << std::endl;
  for (val = 0; val <= SND_PCM_STREAM_LAST; val++)
    std::cout << " " << snd_pcm_stream_name((snd_pcm_stream_t) val) <<
              std::endl;

  std::cout << std::endl << "PCM access types:" << std::endl;
  for (val = 0; val <= SND_PCM_ACCESS_LAST; val++)
    std::cout << " " << snd_pcm_access_name((snd_pcm_access_t) val) <<
              std::endl;

  std::cout << std::endl << "PCM formats:" << std::endl;
  for (val = 0; val <= SND_PCM_FORMAT_LAST; val++) {
    if (snd_pcm_format_name((snd_pcm_format_t) val) != NULL) {
      std::cout << "  " << snd_pcm_format_name((snd_pcm_format_t) val) <<
                " (" <<
                snd_pcm_format_description((snd_pcm_format_t) val) << ")" <<
                std::endl;
    }
  }

  std::cout << std::endl << "PCM subformats:" << std::endl;
  for (val = 0; val <= SND_PCM_SUBFORMAT_LAST; val++) {
    std::cout << "  " << snd_pcm_subformat_name((snd_pcm_subformat_t) val) <<
              " (" <<
              snd_pcm_subformat_description((snd_pcm_subformat_t) val) << ")" <<
              std::endl;
  }

  std::cout << std::endl << "PCM states:" << std::endl;
  for (val = 0; val <= SND_PCM_STATE_LAST; val++) {
    std::cout << " " << snd_pcm_state_name((snd_pcm_state_t) val) << std::endl;
  }
}

void AlsaControl::allocate_FFTWop(){
    fftw_ptr = (struct FFTWop *)malloc(stereo_mode_ * sizeof(struct FFTWop));
    if(!fftw_ptr) exit(EXIT_FAILURE);
    //allocating space for dft operations
    for(int i=0; i< (int)stereo_mode_; ++i){
        fftw_ptr[i].in = (fftw_complex *)fftw_malloc(sizeof(fftw_complex) * frames_);
        if(!fftw_ptr[i].in) exit(EXIT_FAILURE);
        fftw_ptr[i].out = (fftw_complex *)fftw_malloc(sizeof(fftw_complex) * frames_);
        if(!fftw_ptr[i].out) exit(EXIT_FAILURE);
        fftw_ptr[i].index = i;
    }
} 

void AlsaControl::allocate_FFTW_Results(){
    int i, ch;

    FFTW_Results_ptr = (struct FFTW_Results *)malloc(totalpackets * sizeof(struct FFTW_Results));
    if(!(FFTW_Results_ptr)) exit(EXIT_FAILURE);;

    for (i = 0; i < (int)totalpackets; ++i){
        //for peak results
        FFTW_Results_ptr[i].peakfreq = (double *)malloc(stereo_mode_*sizeof(double));

        if(!FFTW_Results_ptr[i].peakfreq) exit(EXIT_FAILURE);

        FFTW_Results_ptr[i].peakpower = (double *)malloc(stereo_mode_*sizeof(double));

        if(!FFTW_Results_ptr[i].peakpower) exit(EXIT_FAILURE);
        //for power spectrum (i.e. a double matrix) of
        //N BUCKETS that represent a frequency range
        FFTW_Results_ptr[i].peakmagMatrix = (double **)malloc(stereo_mode_*sizeof(double));
        if(!FFTW_Results_ptr[i].peakmagMatrix) exit(EXIT_FAILURE);
        for(ch = 0; ch < (int)stereo_mode_ ; ++ch){
            FFTW_Results_ptr[i].peakmagMatrix[ch] = (double *)malloc(BUCKETS * sizeof(double));
            if(!FFTW_Results_ptr[i].peakmagMatrix[ch]) exit(EXIT_FAILURE);
        }
    }  
}

void AlsaControl::SetupDFTForSound(unsigned char *buffer, int bytesRead){
    int bytewidth = bits_ / 8;
    int channels = stereo_mode_;
    int frames = bytesRead / (bytewidth * channels);
    int count = 0, c;

    for(c = 0; c < channels; ++c){
        fftw_ptr[c].p = fftw_plan_dft_1d(frames, fftw_ptr[c].in,
                fftw_ptr[c].out, FFTW_FORWARD, FFTW_MEASURE);
    }

    while(count < frames){
        for(c = 0; c< channels; ++c){
            fftw_ptr[c].in[count][re] = Get16bitAudioSample(buffer);
            fftw_ptr[c].in[count][im] = 0.0;

            buffer+=bytewidth;
        }
        count++;
    }
}

void AlsaControl::analyze_FFTW_Results(struct FFTWop fftwop , int packet_index, int ch,size_t bytesRead)
{
    double real, imag;
    double peakmax = 1.7E-308 ;
    int max_index = -1, i, j;
    double magnitude;
    double* peakmaxArray = (double*)malloc(BUCKETS * sizeof(double));
    double nyquist = rate_ / 2;
    double freq_bin[] = {19.0, 140.0, 400.0, 2600.0, 5200.0, nyquist };

    int frames = bytesRead / (stereo_mode_ * bits_ / 8);


    for(i = 0; i<BUCKETS; ++i) peakmaxArray[i] = 1.7E-308;

    for(j = 0; j < frames/2; ++j){

        real =  fftwop.out[j][0];
        imag =  fftwop.out[j][1];
        magnitude = sqrt(real*real+imag*imag);
        double freq = j * rate_ / frames;

        for (i = 0; i < BUCKETS; ++i){
            if((freq>freq_bin[i]) && (freq <=freq_bin[i+1])){
                if (magnitude > peakmaxArray[i]){
                    peakmaxArray[i] = magnitude;
                }
            }
        }

        if(magnitude > peakmax){
                peakmax = magnitude;
                max_index = j;
        }
    }

    FFTW_Results_ptr[packet_index].peakpower[ch] =  10*(log10(peakmax));
    FFTW_Results_ptr[packet_index].peakfreq[ch] = max_index*(double)(rate_)/frames;
    for(i = 0; i< BUCKETS; ++i){
        FFTW_Results_ptr[packet_index].peakmagMatrix[ch][i]=10*(log10(peakmaxArray[i]));
    }

    free(peakmaxArray);
}

void AlsaControl::outputpowerspectrum(){
    int totchannels = stereo_mode_;
    int thickness = 2, b, c, t, p, energy;

    /*PRINT GRID*/
    if(system("clear") < 0){}

    for(c = 0; c < totchannels; ++c){
        for(b = 0; b < BUCKETS; ++b){
            if((c+1)%2 == 0)
                energy = FFTW_Results_ptr[packet_pos].peakmagMatrix[c][b];
            else
                energy = FFTW_Results_ptr[packet_pos].peakmagMatrix[c][BUCKETS-b-1];
            for(t = 0; t < thickness; ++t){
                for (p = 0; p < energy; ++p)
                    putchar('|');

                putchar('>');
                putchar('\n');
            }
        }
        putchar('\n');
        fflush(stdout);
    }
    fflush(stdout);
}

double AlsaControl::Get16bitAudioSample(unsigned char* bytebuffer){
    uint16_t val =  0x0;
    
    val = ((uint16_t)bytebuffer[1] << 8) | (uint16_t)bytebuffer[0];

    return ((int16_t)val)/32768.0;
}

void AlsaControl::OpenPcmDevice() {
  int rc = snd_pcm_open(&handle_, "default", SND_PCM_STREAM_CAPTURE, 0);

  if (rc < 0) {
    std::cout << "ERROR :  unable to open pcm device: " << snd_strerror(rc) <<
              std::endl;
    exit(1);
  }
}

void AlsaControl::SetParametersALSA() {
  snd_pcm_hw_params_any(handle_, params_); // def values
  snd_pcm_hw_params_set_access(handle_, params_,
      SND_PCM_ACCESS_RW_INTERLEAVED
  ); //non interleaved
  snd_pcm_hw_params_set_format(handle_, params_,
      SND_PCM_FORMAT_S16_LE
  ); //16bits little-endian
  snd_pcm_hw_params_set_channels(handle_, params_,
      stereo_mode_
  ); // stereo ou mono


  snd_pcm_hw_params_set_rate_near(handle_, params_, &rate_,
      NULL
  ); // sample rate (freq echantillonage)
  snd_pcm_hw_params_set_period_size_near(handle_, params_,
      &frames_,
      NULL
  ); //frames pour une période

  int rc = snd_pcm_hw_params(handle_, params_);
  if (rc < 0) {
    std::cout << "ERROR - unable to set hw parameters: " << snd_strerror(rc) <<
              std::endl;
    exit(1);
  }

  snd_pcm_hw_params_get_period_size(params_, &period_size_, NULL);
  snd_pcm_hw_params_get_period_time(params_, &time_period_, NULL);
}

void AlsaControl::Listen() {
  if (!continue_listening_.load(std::memory_order_relaxed)) {
    continue_listening_.store(true, std::memory_order_relaxed);
    thread_ = std::async(std::launch::async, &AlsaControl::ThreadListen,
        this, ""
    );
  } else {
    std::cout << "ERROR - System is already listening/recording use stop()" <<
              std::endl;
  }
}

void AlsaControl::Listen(std::string filename) {
  if (!continue_listening_.load(std::memory_order_relaxed)) {
    continue_listening_.store(true, std::memory_order_relaxed);
    thread_ = std::async(std::launch::async, &AlsaControl::ThreadListen,
        this, filename
    );
  } else {
    std::cout << "ERROR - System is already listening/recording use stop()" <<
              std::endl;
  }
}

void AlsaControl::ListenWithCallback(std::function<void(void*, int)> func) {
  if (!continue_listening_.load(std::memory_order_relaxed)) {
    continue_listening_.store(true, std::memory_order_relaxed);
    thread_ = std::async(std::launch::async,
        &AlsaControl::ThreadListenWithCallback, this, func, ""
    );
  } else {
    std::cout << "ERROR - System is already listening/recording use stop()" <<
              std::endl;
  }
}

void AlsaControl::ListenWithCallback(std::function<void(void*, int)> func,
    std::string filename) {
  if (!continue_listening_.load(std::memory_order_relaxed)) {
    continue_listening_.store(true, std::memory_order_relaxed);
    thread_ = std::async(std::launch::async,
        &AlsaControl::ThreadListenWithCallback, this, func,
        filename
    );
  } else {
    std::cout << "ERROR - System is already listening/recording use stop()" <<
              std::endl;
  }
}

void AlsaControl::RecordToFile(std::string filename,
    int const& duration_in_us) {
  if (!continue_listening_.load(std::memory_order_relaxed)) {
    continue_listening_.store(true, std::memory_order_relaxed);
    thread_ = std::async(std::launch::async,
        &AlsaControl::ThreadRecordToFile, this, filename,
        duration_in_us
    );
    thread_.get();
    continue_listening_.store(false, std::memory_order_relaxed);
  } else {
    std::cout << std::endl <<
              "ERROR - System is already listening/recording use stop()";
  }
}

void AlsaControl::Stop() {
  continue_listening_.store(false, std::memory_order_relaxed);
  thread_.get();
}

void AlsaControl::ThreadListen(std::string filename) {
  std::ofstream f;
  int rc;
  int nb_ech = 0;
  int channels = (int)stereo_mode_;
  int packet_index = 0, i;

  if (filename != "") {
    filename += ".wav";
    f.open(filename, std::ios::binary);
    WriteHeaderWav(f, rate_, static_cast<short>(bits_),
        static_cast<short>(stereo_mode_), 10000);
    // 10000 is an arbitrary constant because we don't know yet
    // the size of the recording
  }

  snd_pcm_uframes_t size = period_size_ * 2 *
      stereo_mode_; /* 2 bytes/sample, 1 channels */
  void* buffer = malloc(size);

    std::cout<<"henry read"<<size<<std::endl;
  while (continue_listening_.load(std::memory_order_relaxed)) {
    rc = (int) snd_pcm_readi(handle_, buffer, period_size_);
    if (rc == -EPIPE) {
      std::cout << std::endl << "ERROR - overrun occurred";
      snd_pcm_prepare(handle_);
    } else if (rc < 0) {
      std::cout << std::endl << "ERROR - error from read: " << snd_strerror(rc);
    } else if (rc != (int) period_size_) {
      std::cout << std::endl << "ERROR - short read, read " << rc << " frames";
    }

    if (rc > 0 && filename != "") {
      SetupDFTForSound((unsigned char *)buffer, size);
      for(i = 0; i < channels; ++i){
          fftw_execute(fftw_ptr[i].p);
          analyze_FFTW_Results(fftw_ptr[i], packet_index, i ,size);
          fftw_destroy_plan(fftw_ptr[i].p);
      }
      outputpowerspectrum();
      packet_pos++;
      packet_index++;
      f.write(static_cast<char*>(buffer), rc * 2 * stereo_mode_);
      nb_ech += rc;
    }
  }

  free(buffer);
  for(i = 0; i<channels; ++i){

      free(fftw_ptr[i].in);
      free(fftw_ptr[i].out);
  }
  free(fftw_ptr);

  if (filename != "") {
    WriteHeaderWav(f, rate_, static_cast<short>(bits_),
        static_cast<short>(stereo_mode_),
        nb_ech
    );
    f.close();
  }
}

void AlsaControl::ThreadListenWithCallback(
    std::function<void(void*, int)> func, std::string filename) {
  std::ofstream f;
  int rc;
  int nb_ech = 0;

  if (filename != "") {
    filename += ".wav";
    f.open(filename, std::ios::binary);
    WriteHeaderWav(f, rate_, static_cast<short>(bits_),
        static_cast<short>(stereo_mode_), 10000
    );
  }

  snd_pcm_uframes_t size = period_size_ * 2 *
      stereo_mode_; /* 2 bytes/sample, 1 channels */
  void* buffer = malloc(size);


  while (continue_listening_.load(std::memory_order_relaxed)) {
    rc = (int) snd_pcm_readi(handle_, buffer, period_size_);
    if (rc == -EPIPE) {
      std::cout << std::endl << "ERROR - overrun occurred";
      snd_pcm_prepare(handle_);
    } else if (rc < 0) {
      std::cout << std::endl << "ERROR - error from read: " << snd_strerror(rc);
    } else if (rc != (int) period_size_) {
      std::cout << std::endl << "ERROR - short read, read " << rc << " frames";
    }

    if (rc > 0 && filename != "") {
      f.write(static_cast<char*>(buffer), rc * 2 * stereo_mode_);
      nb_ech += rc;
    }

    func(buffer, rc);
  }

  free(buffer);

  if (filename != "") {
    WriteHeaderWav(f, rate_, static_cast<short>(bits_),
        static_cast<short>(stereo_mode_), nb_ech
    );
    f.close();
  }
}

void AlsaControl::ThreadRecordToFile(std::string filename,
    int const& duration_in_us) {
  std::ofstream f;
  int rc;
  int nb_ech = 0;

  filename += ".wav";
  f.open(filename, std::ios::binary);
  WriteHeaderWav(f, rate_, static_cast<short>(bits_),
      static_cast<short>(stereo_mode_), 10000);

  // 2 bytes/sample, 1 channels
  snd_pcm_uframes_t size = period_size_ * 2 * stereo_mode_;

  void* buffer = malloc(size);
  long loops = duration_in_us / time_period_;

  while (loops-- > 0) {
    rc = (int) snd_pcm_readi(handle_, buffer, period_size_);
    if (rc == -EPIPE) {
      std::cout << std::endl << "ERROR - overrun occurred";
      snd_pcm_prepare(handle_);
    } else if (rc < 0) {
      std::cout << std::endl << "ERROR - error from read: " << snd_strerror(rc);
    } else if (rc != (int) period_size_) {
      std::cout << std::endl << "ERROR - short read, read " << rc << " frames";
    }

    if (rc > 0) {
      f.write(static_cast<char*> (buffer), rc * 2 * stereo_mode_);
    }

    nb_ech += rc;
  }

  WriteHeaderWav(f, rate_, static_cast<short>(bits_),
      static_cast<short>(stereo_mode_), nb_ech
  );
  f.close();
  free(buffer);
}

void AlsaControl::ForcePeriodSize(int const& value) {
  period_size_ = static_cast<snd_pcm_uframes_t>(value);
}
