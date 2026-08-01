#include "WebSocketServer.h"


static const char* TAG = "WebSocketServer";

WebSocketServer::WebSocketServer(httpd_handle_t sharedServer)
    : server(sharedServer) {}

void WebSocketServer::setupRoutes() {
    static httpd_uri_t ws_uri = {
        .uri = "/ws",
        .method = HTTP_GET,
        .handler = WebSocketServer::wsHandler,
        .user_ctx = this,
        .is_websocket = true
    };

    httpd_register_uri_handler(server, &ws_uri);
}

esp_err_t WebSocketServer::wsHandler(httpd_req_t *req) {
    WebSocketServer* self = static_cast<WebSocketServer*>(req->user_ctx);
    
    if (req->method == HTTP_GET) {
        int newClientFd = httpd_req_to_sockfd(req);
        if (clientFd >= 0 && clientFd != newClientFd) {
            closeClient(self->server, clientFd);
        }
        clientFd = newClientFd;
        return ESP_OK;
    }
    
    
    httpd_ws_frame_t frame = {};
    frame.type = HTTPD_WS_TYPE_TEXT;
    frame.payload = nullptr;

    esp_err_t ret = httpd_ws_recv_frame(req, &frame, 0);
    if (ret != ESP_OK) {
        return ret;
    }

    frame.payload = (uint8_t*)malloc(frame.len + 1);
    if (!frame.payload) return ESP_ERR_NO_MEM;

    ret = httpd_ws_recv_frame(req, &frame, frame.len);
    if (ret != ESP_OK) {
        free(frame.payload);
        if (clientFd == httpd_req_to_sockfd(req)) {
            clientFd = -1;
        }
        return ret;
    }
    frame.payload[frame.len] = '\0';

    // Bind the reply socket here, from the data frame — NOT only at the GET
    // handshake. On this board/IDF the handshake's clientFd assignment never
    // sticks (clientFd stays -1), so every response was dropped at the
    // `if (clientFd < 0) return;` guard in sendText. A data frame carries the
    // real socket fd, and it arrives immediately before the command's output is
    // generated, so refreshing clientFd here guarantees a valid reply target.
    clientFd = httpd_req_to_sockfd(req);

    // Push chars one by one into buffer
    for (size_t i = 0; i < frame.len; ++i) {
        self->buffer.push_back(((char*)frame.payload)[i]);
    }

    free(frame.payload);
    return ESP_OK;
}

char WebSocketServer::readCharBlocking() {
    // The CLI is about to wait for the next keystroke — push out whatever the last
    // command produced, as a single frame.
    flushOutput();
    while (buffer.empty()) {
        delay(10);
    }
    char c = buffer.front();
    buffer.pop_front();
    return c;
}

char WebSocketServer::readCharNonBlocking() {
    flushOutput();
    if (buffer.empty()) return KEY_NONE;

    char c = buffer.front();
    buffer.pop_front();

    return c;
}

namespace {
struct WsAsyncArg {
    httpd_handle_t hd;
    int fd;
    std::string* payload;
};

// Runs in the httpd task context (via httpd_queue_work). WS frames must be sent
// from there — calling httpd_ws_send_frame_async directly from another task (the
// dispatcher) silently drops the output and errors out.
void wsAsyncSendCb(void* arg) {
    WsAsyncArg* a = static_cast<WsAsyncArg*>(arg);
    if (!a) return;
    httpd_ws_frame_t pkt = {};
    pkt.final = true;   // complete (non-fragmented) frame, or the client waits forever
    pkt.type = HTTPD_WS_TYPE_TEXT;
    pkt.payload = reinterpret_cast<uint8_t*>(const_cast<char*>(a->payload->data()));
    pkt.len = a->payload->size();
    httpd_ws_send_frame_async(a->hd, a->fd, &pkt);
    delete a->payload;
    delete a;
}
}  // namespace

void WebSocketServer::sendText(const std::string& msg) {
    if (clientFd < 0) return;
    // Accumulate. A single command emits many print()/println() calls; sending each
    // as its own WebSocket frame (whether directly or queued) is what dropped the
    // terminal output. Batch them and flush once as the CLI goes idle to read the
    // next keystroke (see flushOutput + readChar*).
    outBuffer += msg;
    if (outBuffer.size() >= 4096) flushOutput();  // bound growth on huge dumps
}

void WebSocketServer::flushOutput() {
    if (outBuffer.empty()) return;
    if (clientFd < 0) { outBuffer.clear(); return; }

    std::string safe = sanitizeUtf8(outBuffer);
    outBuffer.clear();

    // One frame for the whole batch, sent from the httpd task (httpd_queue_work);
    // fall back to a direct send if the work queue is unavailable. Never close the
    // socket on a send hiccup — that made the browser reconnect-loop.
    WsAsyncArg* a = new WsAsyncArg{ server, clientFd, new std::string(std::move(safe)) };
    if (httpd_queue_work(server, wsAsyncSendCb, a) != ESP_OK) {
        httpd_ws_frame_t pkt = {};
        pkt.final = true;
        pkt.type = HTTPD_WS_TYPE_TEXT;
        pkt.payload = reinterpret_cast<uint8_t*>(const_cast<char*>(a->payload->data()));
        pkt.len = a->payload->size();
        httpd_ws_send_frame_async(server, clientFd, &pkt);
        delete a->payload;
        delete a;
    }
}

std::string WebSocketServer::sanitizeUtf8(const std::string& input) {
    std::string output;
    size_t i = 0;

    while (i < input.size()) {
        unsigned char c = input[i];

        if (c <= 0x7F) {  // ASCII
            output += c;
            i++;
        } else if ((c & 0xE0) == 0xC0 && i + 1 < input.size() &&
                   (input[i+1] & 0xC0) == 0x80) {
            output += input.substr(i, 2);
            i += 2;
        } else if ((c & 0xF0) == 0xE0 && i + 2 < input.size() &&
                   (input[i+1] & 0xC0) == 0x80 &&
                   (input[i+2] & 0xC0) == 0x80) {
            output += input.substr(i, 3);
            i += 3;
        } else if ((c & 0xF8) == 0xF0 && i + 3 < input.size() &&
                   (input[i+1] & 0xC0) == 0x80 &&
                   (input[i+2] & 0xC0) == 0x80 &&
                   (input[i+3] & 0xC0) == 0x80) {
            output += input.substr(i, 4);
            i += 4;
        } else {
            // Invalid byte or sequence, skip it
            i++;
        }
    }

    return output;
}

void WebSocketServer::closeClient(httpd_handle_t server, int fd) {
    if (fd < 0) return;
    httpd_sess_trigger_close(server, fd);
    if (clientFd == fd) {
        clientFd = -1;
        buffer.clear();
    }
}
