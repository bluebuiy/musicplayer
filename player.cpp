
#include <atomic>
#include <thread>

#include "portaudio.h"

extern "C"
{

#include "ffmpeg/include/libavcodec/avcodec.h"
#include "ffmpeg/include/libavformat/avformat.h"

}
#include <iostream>
#include <fstream>

#define BUFFER_COUNT (5)

struct audio_data
{
  AVFrame * frames[BUFFER_COUNT];
  std::atomic<int> current_read_frame;
  std::atomic<int> current_write_frame;
  std::atomic<bool> flushed;
  std::atomic<bool> eof;
  int sampleindex;
  AVSampleFormat format;
};


bool init_audio(PaStream **, audio_data *, int samplerate);
void finalize_audio(PaStream *);

int decode_frame(audio_data *, AVFormatContext *, AVCodecContext *, AVPacket * packet, int stream, int frameindex);

template <typename T, bool packed>
int pacallback(const void * input, void * output, unsigned long frameCount, const PaStreamCallbackTimeInfo * timeInfo, PaStreamCallbackFlags statusFlags, void * userData);

int pacallback1(const void * input, void * output, unsigned long frameCount, const PaStreamCallbackTimeInfo * timeInfo, PaStreamCallbackFlags statusFlags, void * userData);

int main(int argc, char ** argv)
{
  if (argc != 2)
  {
    printf("no input\n");
    return 1;
  }
  char const * filename = argv[1];// "C:\\Users\\Max\\Desktop\\musics\\music\\explore\\mus_explore_day_01.xwm";
  ////char const * filename = "click.wav";
  int err;
  AVFormatContext *format_context = NULL;

  format_context = avformat_alloc_context();
  if (format_context == nullptr)
  {
    return 1;
  }

  if ((err = avformat_open_input(&format_context, filename, nullptr, nullptr)) < 0)
  {
    char errstr[128];
    av_make_error_string(errstr, 128, err);
    std::ofstream test("testout.txt", std::ios_base::binary);
    test << filename;
    printf("error opening file: [%s]\n%s\n", filename, errstr);
    return 1;
  }
  printf("file: %s\n", argv[1]);
  printf("format: %s\n", format_context->iformat->name);
  
  AVCodec * codec = nullptr;
  int stream_id = -1;
  AVCodecParameters *codec_parms = nullptr;
  for (int i = 0; i < format_context->nb_streams; ++i)
  {
    codec_parms = format_context->streams[i]->codecpar;
    if (codec_parms->codec_type == AVMEDIA_TYPE_AUDIO)
    {
      codec = avcodec_find_decoder(codec_parms->codec_id);
      stream_id = i;
      break;
    }
  }

  if (stream_id == -1)
  {
    printf("No audio stream found.\n");
    return 1;
  }

  printf("Sample rate: %d\n", codec_parms->sample_rate);
  printf("uisng codec %s in stream %d\n", codec->name, stream_id);
  
  if (codec_parms->channels != 2)
  {
    printf("Unsuported channel count %d\n", codec_parms->channels);
    return 1;
  }


  
  AVCodecContext * codec_context = avcodec_alloc_context3(codec);
  if (!codec_context)
  {
    printf("Failed to allocate codec context\n");
    return 1;
  }

  if (avcodec_parameters_to_context(codec_context, codec_parms) < 0)
  {
    printf("Failed to copy parameters to context\n");
    return 1;
  }

  if (avcodec_open2(codec_context, codec, nullptr) < 0)
  {
    printf("Failed to open codec\n");
  }

  audio_data audiostuff;
  audiostuff.current_read_frame = 0;
  audiostuff.format = codec_context->sample_fmt;
  for (int i = 0; i < BUFFER_COUNT; ++i)
  {
    AVFrame * frame = av_frame_alloc();
    if (!frame)
    {
      printf("Failed to allocate frame\n");
      return 1;
    }
    audiostuff.frames[i] = frame;
  }

  AVPacket * packet = av_packet_alloc();
  if (!packet)
  {
    printf("Failed to allcate packet\n");
    return 1;
  }

  // decode first BUFFER_COUNT frames before initializing audio
  for (int i = 0; i < BUFFER_COUNT - 1; ++i)
  {
    decode_frame(&audiostuff, format_context, codec_context, packet, stream_id, i);
  }
  audiostuff.current_write_frame = BUFFER_COUNT - 1;

  PaStream * audiostream;

  if (init_audio(&audiostream, &audiostuff, codec_parms->sample_rate))
  {
    return 1;
  }

  while (true)
  {
    while (audiostuff.current_write_frame == (audiostuff.current_read_frame + BUFFER_COUNT - 1) % BUFFER_COUNT)
    {
      std::this_thread::yield();
    }
    int ret = decode_frame(&audiostuff, format_context, codec_context, packet, stream_id, audiostuff.current_write_frame);
    audiostuff.current_write_frame = (audiostuff.current_write_frame + 1) % BUFFER_COUNT;
    if (ret == AVERROR_EOF)
    {
      break;
    }
  }
  audiostuff.eof = true;
  while (!audiostuff.flushed);

  finalize_audio(audiostream);

  av_packet_free(&packet);
  av_frame_free(&audiostuff.frames[0]);
  av_frame_free(&audiostuff.frames[1]);
  avcodec_close(codec_context);
  avformat_close_input(&format_context);

  return 0;
}

// return 0 on success
// eof on end
int decode_frame(audio_data * audio, AVFormatContext * format, AVCodecContext * codec, AVPacket * packet, int streamid, int frameindex)
{
  int frame_state = -1;
  bool eof = false;
  do
  {
    // try to decode a frame
    do
    {
      frame_state = avcodec_receive_frame(codec, audio->frames[frameindex]);
      if (frame_state == 0 || frame_state == AVERROR_EOF)
      {
        return frame_state;
      }
      else if (frame_state != AVERROR(EAGAIN))
      {
        char errstr[128];
        av_make_error_string(errstr, 128, frame_state);
        printf("Error decoding frame: %s\n", errstr);
        // what to do here?
        // I will try again.
        // at worst it will consume the whole file and do nothing...
      }
    } while (!(frame_state == AVERROR_EOF || frame_state == AVERROR(EAGAIN)));



    // frame_state is EAGAIN so insert data

    int stream_state = 0;
    int read_ret = 0;
    // read file
    while ((read_ret = av_read_frame(format, packet)) == 0)
    {
      if (packet->stream_index == streamid)
      {
        // send data to decoder
        stream_state = avcodec_send_packet(codec, packet);
        if (stream_state == AVERROR(EAGAIN) || stream_state == AVERROR_EOF || stream_state == 0)
        {
          // need to stop inserting data
          break;
        }
        else if (stream_state != 0)
        {
          char errstr[128];
          av_make_error_string(errstr, 128, stream_state);
          printf("Error decoding frame: %s\n", errstr);
          // what to do here?
          // try again...
        }
      }
    }
    if (read_ret == AVERROR_EOF)
    {
      avcodec_send_packet(codec, nullptr);
    }
    else if (read_ret != 0)
    {
      char errstr[128];
      av_make_error_string(errstr, 128, read_ret);
      printf("Error reading file: %s\n", errstr);
      return read_ret;
    }

  } while (true);

  // what
  printf("Something strange happened\n");
  return -1;
}

bool init_audio(PaStream ** stream, audio_data * data, int samplerate)
{
  data->sampleindex = 0;
  data->flushed = false;
  data->eof = false;
  PaStreamCallback * func = nullptr;
  switch (data->format)
  {
  case AV_SAMPLE_FMT_U8:
    func = pacallback<uint8_t, true>;
    break;
  case AV_SAMPLE_FMT_S16:
    func = pacallback<int16_t, true>;
    break;
  case AV_SAMPLE_FMT_S32:
    func = pacallback<int32_t, true>;
    break;
  case AV_SAMPLE_FMT_FLT:
    func = pacallback<float, true>;
    break;
  case AV_SAMPLE_FMT_DBL:
    func = pacallback<double, true>;
    break;
  case AV_SAMPLE_FMT_U8P:
    func = pacallback<uint8_t, false>;
    break;
  case AV_SAMPLE_FMT_S16P:
    func = pacallback<int16_t, false>;
    break;
  case AV_SAMPLE_FMT_S32P:
    func = pacallback<int32_t, false>;
    break;
  case AV_SAMPLE_FMT_FLTP:
    func = pacallback<float, false>;
    break;
  case AV_SAMPLE_FMT_DBLP:
    func = pacallback<double, false>;
    break;
  default:
    printf("Unsuported sample format\n");
    return true;
  }
  Pa_Initialize();
  //PaDeviceIndex device = Pa_GetDefaultOutputDevice();
  //PaStreamParameters args;
  //args.channelCount = 2;
  //args.device = device;
  //args.sampleFormat = paFloat32;
  //args.suggestedLatency = 0.1f;
  //PaError err = Pa_OpenStream(stream, nullptr, &args, samplerate, paFramesPerBufferUnspecified, paNoFlag, func, data);
  PaError err = Pa_OpenDefaultStream(stream, 0, 2, paFloat32, samplerate, paFramesPerBufferUnspecified, func, data);
  if (err == PaErrorCode::paInvalidSampleRate)
  {
    printf("Failed to open output stream with samplerate %d\n", samplerate);
    return true;
  }
  else if (err != PaErrorCode::paNoError)
  {
    printf("Faild to open output stream: %s\n", Pa_GetErrorText(err));
    return true;
  }
  Pa_StartStream(*stream);
  return false;
}

void finalize_audio(PaStream * stream)
{
  Pa_StopStream(stream);
  Pa_CloseStream(stream);
  Pa_Terminate();
}

template <typename T>
typename std::enable_if<!std::is_floating_point<T>::value && std::is_signed<T>::value, void>::type get_sample(AVFrame * frame, int index, int channel, float * out)
{
  T * buffer = (T *)frame->data[channel];
  *out = ((float)buffer[index]) / std::numeric_limits<T>::max();
}

template <typename T>
typename std::enable_if<!std::is_floating_point<T>::value && std::is_unsigned<T>::value, void>::type get_sample(AVFrame * frame, int index, int channel, float * out)
{
  T * buffer = (T *)frame->data[channel];
  *out = (((float)buffer[index]) - std::numeric_limits<T>::max() / 2.f) / (std::numeric_limits<T>::max() / 2.f);
}

template <typename T>
typename std::enable_if<std::is_floating_point<T>::value, void>::type get_sample(AVFrame * frame, int index, int channel, float * out)
{
  T * buffer = (T *)frame->data[channel];
  *out = (float)buffer[index];
}

template <typename T>
void get_sample_packed(AVFrame * frame, int index, int channel, float * out)
{
  get_sample<T>(frame, index * frame->channels + channel, 0, out);
}

int pacallback1(const void * input, void * output, unsigned long frameCount, const PaStreamCallbackTimeInfo * timeInfo, PaStreamCallbackFlags statusFlags, void * userData)
{
  float * b = (float *)output;
  for (int i = 0; i < frameCount; ++i)
  {
    if (i & 64)
    {
      b[i * 2] = 1;
      b[i * 2 + 1] = 1;
    }
    else
    {
      b[i * 2] = -1;
      b[i * 2 + 1] = -1;
    }

  }

  return paContinue;
}

template <typename T, bool packed>
int pacallback(const void * input, void * output, unsigned long frameCount, const PaStreamCallbackTimeInfo * timeInfo, PaStreamCallbackFlags statusFlags, void * userData)
{
  audio_data * data = (audio_data *)userData;
  float * outbuffer = (float *)output;

  bool eof = data->eof;

  int i = 0;
  while (i < frameCount)
  {
    if (data->current_read_frame != data->current_write_frame)
    {
      AVFrame * frame = data->frames[data->current_read_frame];
      while (data->sampleindex < frame->nb_samples && i < frameCount)
      {
        float L;
        float R;
        if (packed)
        {
          get_sample_packed<T>(frame, data->sampleindex, 0, &L);
          get_sample_packed<T>(frame, data->sampleindex, 1, &R);
        }
        else
        {
          get_sample<T>(frame, data->sampleindex, 0, &L);
          get_sample<T>(frame, data->sampleindex, 1, &R);
        }
        outbuffer[i * 2] = L;
        outbuffer[i * 2 + 1] = R;
        ++data->sampleindex;
        ++i;
      }
      if (data->sampleindex == frame->nb_samples)
      {
        data->sampleindex = 0;
        data->current_read_frame = (data->current_read_frame + 1) % BUFFER_COUNT;
      }
    }
    else
    {
      for (; i < frameCount; ++i)
      {
        outbuffer[i * 2] = 0;
        outbuffer[i * 2 + 1] = 0;
      }
      if (eof)
      {
        data->flushed = true;
        return paComplete;
      }
    }
  }

  return paContinue;
}


