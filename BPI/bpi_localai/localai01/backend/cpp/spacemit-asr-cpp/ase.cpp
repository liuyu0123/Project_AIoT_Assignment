#include <iostream>
#include <string>
#include <memory>
#include <chrono>
#include <vector>
#include <cstdlib>
#include <iomanip>
#include <sndfile.h>

extern "C" {
    #include <libavcodec/avcodec.h>
    #include <libavformat/avformat.h>
    #include <libswresample/swresample.h>
    #include <libavutil/opt.h>
}

#include "asr_model.hpp"
#include "model_downloader.hpp"

class ASEDemo {
public:
    ASEDemo() = default;
    ~ASEDemo() = default;

    bool initialize() {
        std::cout << "Initializing ASE (Audio Speech Engine) Demo..." << std::endl;
        
        // Download models if needed
        ModelDownloader downloader;
        if (!downloader.ensureModelsExist()) {
            std::cerr << "Failed to ensure models exist" << std::endl;
            return false;
        }
        
        // Initialize ASR model
        ASRModel::Config asr_config;
        asr_config.model_path = downloader.getModelPath(ModelDownloader::ASR_MODEL_QUANT_NAME);
        asr_config.config_path = downloader.getModelPath(ModelDownloader::CONFIG_NAME);
        asr_config.vocab_path = downloader.getModelPath(ModelDownloader::VOCAB_NAME);
        asr_config.decoder_path = downloader.getModelPath(ModelDownloader::DECODER_NAME);
        asr_config.sample_rate = 16000;
        asr_config.language = "zh";
        asr_config.use_itn = true;
        asr_config.quantized = true;
        
        asr_model_ = std::make_unique<ASRModel>(asr_config);
        if (!asr_model_->initialize()) {
            std::cerr << "Failed to initialize ASR model" << std::endl;
            return false;
        }
        
        std::cout << "ASE Demo initialized successfully!" << std::endl;
        return true;
    }
    
    std::vector<float> loadAudioFile(const std::string& filename) {
        SF_INFO sf_info;
        memset(&sf_info, 0, sizeof(sf_info));
        
        SNDFILE* sf = sf_open(filename.c_str(), SFM_READ, &sf_info);
        if (!sf) {
            std::cerr << "Error opening audio file: " << filename << std::endl;
            std::cerr << "Error: " << sf_strerror(sf) << std::endl;
            return {};
        }
        
        std::cout << "Audio file info:" << std::endl;
        std::cout << "  Sample rate: " << sf_info.samplerate << " Hz" << std::endl;
        std::cout << "  Channels: " << sf_info.channels << std::endl;
        std::cout << "  Frames: " << sf_info.frames << std::endl;
        std::cout << "  Duration: " << std::fixed << std::setprecision(2) 
                  << (double)sf_info.frames / sf_info.samplerate << " seconds" << std::endl;

        // Read audio data
        std::vector<float> audio_data(sf_info.frames * sf_info.channels);
        sf_count_t frames_read = sf_readf_float(sf, audio_data.data(), sf_info.frames);
        if (frames_read != sf_info.frames) {
            std::cerr << "Warning: Only read " << frames_read << " frames out of " << sf_info.frames << std::endl;
        }
        
        sf_close(sf);
        
        // Convert to mono if stereo
        if (sf_info.channels > 1) {
            std::vector<float> mono_audio(sf_info.frames);
            for (int i = 0; i < sf_info.frames; i++) {
                float sum = 0.0f;
                for (int ch = 0; ch < sf_info.channels; ch++) {
                    sum += audio_data[i * sf_info.channels + ch];
                }
                mono_audio[i] = sum / sf_info.channels;
            }
            audio_data = std::move(mono_audio);
        }
        
        // Resample to 16kHz if needed
        if (sf_info.samplerate != 16000) {
            std::cout << "Resampling from " << sf_info.samplerate << " Hz to 16000 Hz..." << std::endl;
            audio_data = resampleAudio(audio_data, sf_info.samplerate, 16000);
        }
        
        return audio_data;
    }

    std::vector<float> loadAudioFileV2(const std::string& filename) {
        // 初始化 FFmpeg
        avformat_network_init();
        AVFormatContext* format_ctx = avformat_alloc_context();
        
        // 打开音频文件
        if (avformat_open_input(&format_ctx, filename.c_str(), nullptr, nullptr) != 0) {
            throw std::runtime_error("无法打开音频文件: " + filename);
        }
        
        // 获取流信息
        if (avformat_find_stream_info(format_ctx, nullptr) < 0) {
            avformat_close_input(&format_ctx);
            throw std::runtime_error("无法获取流信息");
        }
        
        // 查找音频流
        int audio_stream_idx = -1;
        AVCodecParameters* codec_params = nullptr;
        const AVCodec* codec = nullptr;
        
        for (int i = 0; i < format_ctx->nb_streams; i++) {
            if (format_ctx->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_AUDIO) {
                audio_stream_idx = i;
                codec_params = format_ctx->streams[i]->codecpar;
                codec = avcodec_find_decoder(codec_params->codec_id);
                break;
            }
        }
        
        if (!codec || audio_stream_idx == -1) {
            avformat_close_input(&format_ctx);
            throw std::runtime_error("未找到音频流");
        }
        
        // 打印音频信息
        std::cout << "Audio file info:" << std::endl;
        std::cout << "  Format: " << format_ctx->iformat->name << std::endl;
        std::cout << "  Sample rate: " << codec_params->sample_rate << " Hz" << std::endl;
        std::cout << "  Channels: " << codec_params->ch_layout.nb_channels << std::endl;
        
        // 创建解码器上下文
        AVCodecContext* codec_ctx = avcodec_alloc_context3(codec);
        avcodec_parameters_to_context(codec_ctx, codec_params);
        
        if (avcodec_open2(codec_ctx, codec, nullptr) < 0) {
            avcodec_free_context(&codec_ctx);
            avformat_close_input(&format_ctx);
            throw std::runtime_error("无法打开解码器");
        }
        
        // 准备重采样器 (目标: 单声道 16kHz 浮点数)
        SwrContext* swr_ctx = swr_alloc();
        av_opt_set_int(swr_ctx, "in_channel_layout",    codec_params->ch_layout.u.mask, 0);
        av_opt_set_int(swr_ctx, "out_channel_layout",   AV_CH_LAYOUT_MONO, 0);
        av_opt_set_int(swr_ctx, "in_sample_rate",       codec_params->sample_rate, 0);
        av_opt_set_int(swr_ctx, "out_sample_rate",      16000, 0);
        av_opt_set_sample_fmt(swr_ctx, "in_sample_fmt", codec_ctx->sample_fmt, 0);
        av_opt_set_sample_fmt(swr_ctx, "out_sample_fmt", AV_SAMPLE_FMT_FLT, 0);
        swr_init(swr_ctx);
        
        // 准备数据包和帧
        AVPacket* packet = av_packet_alloc();
        AVFrame* frame = av_frame_alloc();
        std::vector<float> audio_data;
        
        // 读取并解码帧
        while (av_read_frame(format_ctx, packet) >= 0) {
            if (packet->stream_index != audio_stream_idx) {
                av_packet_unref(packet);
                continue;
            }
            
            int ret = avcodec_send_packet(codec_ctx, packet);
            if (ret < 0) {
                av_packet_unref(packet);
                continue;
            }
            
            while (ret >= 0) {
                ret = avcodec_receive_frame(codec_ctx, frame);
                if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) break;
                if (ret < 0) throw std::runtime_error("解码错误");
                
                // 配置重采样输出
                uint8_t* out_data[1] = { nullptr };
                int out_samples = swr_get_out_samples(swr_ctx, frame->nb_samples);
                std::vector<float> resampled(out_samples);
                out_data[0] = reinterpret_cast<uint8_t*>(resampled.data());
                
                // 执行重采样 (直接输出为单声道 16kHz 浮点数)
                int converted = swr_convert(
                    swr_ctx, 
                    out_data, 
                    out_samples,
                    const_cast<const uint8_t**>(frame->data), 
                    frame->nb_samples
                );
                
                if (converted > 0) {
                    audio_data.insert(
                        audio_data.end(),
                        resampled.begin(),
                        resampled.begin() + converted
                    );
                }
                av_frame_unref(frame);
            }
            av_packet_unref(packet);
        }
        
        // 清理资源
        av_packet_free(&packet);
        av_frame_free(&frame);
        swr_free(&swr_ctx);
        avcodec_free_context(&codec_ctx);
        avformat_close_input(&format_ctx);
        
        std::cout << "Decoded samples: " << audio_data.size() << std::endl;
        std::cout << "Duration: " << std::fixed << std::setprecision(2) 
                << (double)audio_data.size() / 16000 << " seconds" << std::endl;
        
        return audio_data;
    }
    
    std::vector<float> resampleAudio(const std::vector<float>& input, int input_rate, int output_rate) {
        // Simple linear interpolation resampling
        if (input_rate == output_rate) {
            return input;
        }
        
        double ratio = (double)input_rate / output_rate;
        size_t output_length = (size_t)(input.size() / ratio);
        std::vector<float> output(output_length);
        
        for (size_t i = 0; i < output_length; i++) {
            double src_index = i * ratio;
            size_t index = (size_t)src_index;
            double frac = src_index - index;
            
            if (index + 1 < input.size()) {
                output[i] = input[index] * (1.0 - frac) + input[index + 1] * frac;
            } else {
                output[i] = input[index];
            }
        }
        
        return output;
    }

    // 检查文件扩展名是否为.wav
    bool isWavFile(const std::string& filename) {
        size_t dot_pos = filename.find_last_of(".");
        if(dot_pos == std::string::npos) return false;
        
        std::string ext = filename.substr(dot_pos + 1);
        std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
        return ext == "wav";
    }
    
    std::string processAudioFile(const std::string& filename) {
        std::cout << "\n=== Processing Audio File: " << filename << " ===" << std::endl;
        
        // 根据扩展名选择加载方式
        std::vector<float> audio_data;
        if(isWavFile(filename)) {
            audio_data = loadAudioFile(filename);  // 原始WAV处理函数
        } else {
            audio_data = loadAudioFileV2(filename); // FFmpeg通用处理函数
        }
        if (audio_data.empty()) {
            std::cerr << "Failed to load audio file: " << filename << std::endl;
            return "";
        }
        
        // Perform ASR
        std::cout << "\nPerforming speech recognition..." << std::endl;
        auto start_time = std::chrono::high_resolution_clock::now();
        
        std::string result = asr_model_->recognize(audio_data);
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        
        // Display results
        std::cout << "\n=== ASR Results ===" << std::endl;
        std::cout << "Recognition Result: " << result << std::endl;
        std::cout << "Processing Time: " << duration.count() << " ms" << std::endl;
        
        // Calculate real-time factor
        double audio_duration = (double)audio_data.size() / 16000.0;
        double processing_time = duration.count() / 1000.0;
        double rtf = processing_time / audio_duration;
        std::cout << "Real-time Factor: " << std::fixed << std::setprecision(3) << rtf << std::endl;
        std::cout << "===================" << std::endl;

        return result;
    }

private:
    std::unique_ptr<ASRModel> asr_model_;
};

