#include <grpcpp/grpcpp.h>
#include <grpcpp/server_builder.h>
#include <grpcpp/server_context.h>
#include <grpcpp/resource_quota.h>
#include <signal.h>
#include <unistd.h>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <atomic>
#include <mutex>

#include "backend.pb.h"
#include "backend.grpc.pb.h"
#include "ase.cpp"

using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::Status;
using backend::Backend;
using backend::HealthMessage;
using backend::Reply;
using backend::Result;
using backend::ModelOptions;
using backend::TranscriptRequest;
using backend::TranscriptResult;
using backend::TranscriptSegment;

class ASRService {
private:
    std::unique_ptr<ASEDemo> ase_demo_;
    std::mutex model_mutex_;
    std::atomic<bool> model_loaded;

public:
    ASRService() : model_loaded(false) {}

    bool LoadModel(const ModelOptions* request) {
        std::lock_guard<std::mutex> lock(model_mutex_);
        try {
            std::cerr << "Preparing ASR models, please wait..." << std::endl;
            
            ase_demo_ = std::make_unique<ASEDemo>();
            
            // 初始化模型 - 这里简化处理，实际应用中需要根据请求参数配置
            if (!ase_demo_->initialize()) {
                std::cerr << "Failed to initialize ASR model" << std::endl;
                return false;
            }
            
            model_loaded = true;
            return true;
        } catch (const std::exception& e) {
            std::cerr << "Error loading model: " << e.what() << std::endl;
            return false;
        }
    }

    TranscriptResult ProcessAudio(const TranscriptRequest* request) {
        TranscriptResult result;
        
        if (!model_loaded) {
            result.set_text("Model not loaded");
            return result;
        }

        try {
            // 处理音频文件
            std::string audio_path = request->dst();
            std::cout << "Processing audio file: " << audio_path << std::endl;
            
            // 调用ASEDemo处理音频
            std::lock_guard<std::mutex> lock(model_mutex_);
            
            std::string transcription = ase_demo_->processAudioFile(audio_path);
            
            result.set_text(transcription);
            
            // 添加分段结果
            TranscriptSegment* segment = result.add_segments();
            segment->set_id(0);
            segment->set_start(0);
            segment->set_end(1000);  // 假设1秒时长
            segment->set_text(transcription);
            
            // 添加占位符token
            for (size_t i = 0; i < transcription.length(); i++) {
                segment->add_tokens(0);
            }
        } catch (const std::exception& e) {
            std::cerr << "Error in transcription: " << e.what() << std::endl;
            result.set_text("Transcription failed: " + std::string(e.what()));
        }
        return result;
    }
};

class BackendServiceImpl final : public Backend::Service {
private:
    ASRService asr_service_;

public:
    grpc::Status Health(ServerContext* context, const HealthMessage* request, Reply* response) override {
        response->set_message("OK");
        return Status::OK;
    }

    grpc::Status LoadModel(ServerContext* context, const ModelOptions* request, Result* response) override {
        if (asr_service_.LoadModel(request)) {
            response->set_message("Model loaded successfully");
            response->set_success(true);
        } else {
            response->set_message("Failed to load model");
            response->set_success(false);
        }
        return Status::OK;
    }

    grpc::Status AudioTranscription(ServerContext* context, const TranscriptRequest* request, TranscriptResult* response) override {
        *response = asr_service_.ProcessAudio(request);
        return Status::OK;
    }

    
};

std::unique_ptr<Server> server;

void signalHandler(int signal) {
    std::cout << "Received termination signal. Shutting down..." << std::endl;
    if (server) {
        server->Shutdown();
    }
}

void RunServer(const std::string& address) {
    BackendServiceImpl service;
    
    ServerBuilder builder;
    // 设置消息大小限制 (50MB)
    builder.SetMaxReceiveMessageSize(50 * 1024 * 1024);
    builder.SetMaxSendMessageSize(50 * 1024 * 1024);
    
    // 获取工作线程数 (默认为1)
    const char* max_workers_env = std::getenv("PYTHON_GRPC_MAX_WORKERS");
    int max_workers = max_workers_env ? std::stoi(max_workers_env) : 4;
    
    builder.AddListeningPort(address, grpc::InsecureServerCredentials());
    builder.RegisterService(&service);
    
    // 设置资源配额和工作线程数
    grpc::ResourceQuota quota;
    quota.SetMaxThreads(max_workers);
    builder.SetResourceQuota(quota);

    server = builder.BuildAndStart();
    std::cout << "Server started. Listening on: " << address << std::endl;
    
    // 设置信号处理
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    server->Wait();
}

int main(int argc, char** argv) {
    std::string address = "0.0.0.0:50051";
    
    // 解析命令行参数
    if (argc > 1) {
        for (int i = 1; i < argc; ++i) {
            std::string arg = argv[i];
            if (arg == "--addr" && i + 1 < argc) {
                address = argv[++i];
            }
        }
    }
    
    RunServer(address);
    return 0;
}