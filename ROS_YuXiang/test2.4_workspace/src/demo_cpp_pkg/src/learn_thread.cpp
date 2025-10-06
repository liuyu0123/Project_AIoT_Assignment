#include <iostream>
#include <thread>
#include <chrono>
#include <functional>
#include <cpp-httplib/httplib.h>

class Downlaod
{
public:
    void download(const std::string &host,
                  const std::string &path,
                  const std::function<void(const std::string &,
                                           const std::string &)> &callback)
    {
        std::cout << "Thread ID  " << std::this_thread::get_id() << std::endl;
        httplib::Client client(host);
        auto response = client.Get(path);
        if (response && response->status == 200)
        {
            callback(path, response->body);
        }
    }

    void start_download(const std::string &host,
                        const std::string &path,
                        const std::function<void(const std::string &,
                                                 const std::string &)> &callback)
    {
        auto download_fun = std::bind(&Downlaod::download, this,
                                      std::placeholders::_1,
                                      std::placeholders::_2,
                                      std::placeholders::_3);
        std::thread download_thread(download_fun, host, path, callback);
        download_thread.detach();
    }
};

int main()
{
    Downlaod downlaod;
    auto download_finish_callback = [](const std::string &path,
                                       const std::string &result) -> void
    {
        std::cout << "Download " << path << " finish" << std::endl;
        std::cout << "Totally " << result.length() << " words" << std::endl;
        std::cout << "Content " << result.substr(0, 16) << std::endl;
    };

    downlaod.start_download("http://localhost:8000",
                            "/novel1.txt",
                            download_finish_callback);
    downlaod.start_download("http://localhost:8000",
                            "/novel2.txt",
                            download_finish_callback);
    downlaod.start_download("http://localhost:8000",
                            "/novel3.txt",
                            download_finish_callback);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000*10));
    return 0;
}