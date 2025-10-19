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
#include <fstream>

#include "backend.pb.h"
#include "backend.grpc.pb.h"
#include "tts_demo.hpp"

using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::Status;
using backend::Backend;
using backend::HealthMessage;
using backend::Reply;
using backend::Result;
using backend::ModelOptions;
using backend::TTSRequest;

class TTSModelHandler {
private:
    TTSDemo::Params params_;
    std::unique_ptr<TTSDemo> tts_demo_;
    std::mutex model_mutex_;
    std::atomic<bool> model_loaded;

public:
    TTSModelHandler() : model_loaded(false) {}

    bool LoadModel() {
        std::lock_guard<std::mutex> lock(model_mutex_);
        try {
            std::cerr << "Preparing TTS models, please wait..." << std::endl;
            
            tts_demo_ = std::make_unique<TTSDemo>(params_);
            
            if (!tts_demo_->initialize()) {
                std::cerr << "Failed to initialize TTS model" << std::endl;
                return false;
            }
            
            model_loaded = true;
            std::cerr << "TTS model initialized successfully" << std::endl;
            return true;
        } catch (const std::exception& e) {
            std::cerr << "Error loading TTS model: " << e.what() << std::endl;
            return false;
        }
    }

    Result ProcessText(const TTSRequest* request) {
        Result result;
        
        if (!model_loaded) {
            result.set_success(false);
            result.set_message("TTS model not loaded");
            return result;
        }

        try {
            const std::string& text = request->text();
            const std::string& dst = request->dst();
            
            std::lock_guard<std::mutex> lock(model_mutex_);
            tts_demo_->run(text, dst);
            
            // 验证文件是否生成
            std::ifstream file(dst, std::ios::binary | std::ios::ate);
            if (!file) {
                result.set_success(false);
                result.set_message("File creation failed");
                return result;
            }
            
            result.set_success(true);
            result.set_message("Audio generated successfully");
        } catch (const std::exception& e) {
            result.set_success(false);
            result.set_message("TTS processing error: " + std::string(e.what()));
        }
        return result;
    }
};

class BackendServiceImpl final : public Backend::Service {
private:
    TTSModelHandler tts_handler_;

public:
    grpc::Status Health(ServerContext* context, const HealthMessage* request, Reply* response) override {
        response->set_message("OK");
        return Status::OK;
    }

    grpc::Status LoadModel(ServerContext* context, const ModelOptions* request, Result* response) override {
        if (tts_handler_.LoadModel()) {
            response->set_message("TTS model loaded successfully");
            response->set_success(true);
        } else {
            response->set_message("Failed to load TTS model");
            response->set_success(false);
        }
        return Status::OK;
    }

    grpc::Status TTS(ServerContext* context, 
                    const TTSRequest* request,
                    Result* response) override 
    {
        *response = tts_handler_.ProcessText(request);
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
    
    // 获取工作线程数
    const char* max_workers_env = std::getenv("GRPC_MAX_WORKERS");
    int max_workers = max_workers_env ? std::stoi(max_workers_env) : 4;
    
    builder.AddListeningPort(address, grpc::InsecureServerCredentials());
    builder.RegisterService(&service);
    
    // 设置资源配额和工作线程数
    grpc::ResourceQuota quota;
    quota.SetMaxThreads(max_workers);
    builder.SetResourceQuota(quota);

    server = builder.BuildAndStart();
    std::cout << "TTS Server started. Listening on: " << address << std::endl;
    
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