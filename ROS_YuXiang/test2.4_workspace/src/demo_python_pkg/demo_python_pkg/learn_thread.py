import threading
import requests

class Download:
    def downlaod(self, url, callback):
        print(f"Thread {threading.get_ident()} is downloading: {url}")
        response = requests.get(url)
        response.encoding = 'utf-8'
        callback(url, response.text)

    def start_download(self, url, callback):
        thread = threading.Thread(target=self.downlaod, args=(url, callback))
        thread.start()

def download_finish_callback(url, result):
    print(f"{url} was downloaded, total length: {len(result)}, content: {result[:5]}")


def main():
    d = Download()
    url1 = 'http://localhost:8000/novel1.txt'
    url2 = 'http://localhost:8000/novel2.txt'
    url3 = 'http://localhost:8000/novel3.txt'
    d.start_download(url1, download_finish_callback)
    d.start_download(url2, download_finish_callback)
    d.start_download(url3, download_finish_callback)

