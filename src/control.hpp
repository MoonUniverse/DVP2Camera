#pragma once
#include <json-c/json.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace control {
using Json = std::unique_ptr<json_object, decltype(&json_object_put)>;
inline Json object() { return Json(json_object_new_object(), json_object_put); }
inline void put(json_object* obj, const char* key, const std::string& value) {
    json_object_object_add(obj, key, json_object_new_string_len(value.data(), value.size()));
}
inline void put(json_object* obj, const char* key, int64_t value) {
    json_object_object_add(obj, key, json_object_new_int64(value));
}
inline std::string get(json_object* obj, const char* key, bool required = true) {
    json_object* value = nullptr;
    if (!json_object_object_get_ex(obj, key, &value)) {
        if (!required) return {};
        throw std::runtime_error(std::string("Missing field: ") + key);
    }
    if (!json_object_is_type(value, json_type_string)) throw std::runtime_error(std::string("Expected string: ") + key);
    std::string result(json_object_get_string(value), json_object_get_string_len(value));
    if (result.find('\0') != std::string::npos) throw std::runtime_error("NUL in string");
    return result;
}
inline Json parse(const std::string& raw) {
    json_tokener* tok = json_tokener_new();
    json_tokener_set_flags(tok, JSON_TOKENER_STRICT);
    Json value(json_tokener_parse_ex(tok, raw.data(), raw.size()), json_object_put);
    const bool valid = json_tokener_get_error(tok) == json_tokener_success &&
                       json_tokener_get_parse_end(tok) == raw.size();
    json_tokener_free(tok);
    if (!valid || !value || !json_object_is_type(value.get(), json_type_object))
        throw std::runtime_error("Expected one JSON object");
    return value;
}
inline std::string dump(json_object* value) { return json_object_to_json_string_ext(value, JSON_C_TO_STRING_PLAIN); }
inline Json error(const std::string& code, const std::string& message) {
    auto result = object();
    json_object_object_add(result.get(), "ok", json_object_new_boolean(false));
    put(result.get(), "error", code); put(result.get(), "message", message);
    return result;
}
inline int64_t unix_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}
inline std::string default_socket() {
    const char* runtime = std::getenv("XDG_RUNTIME_DIR");
    return runtime && *runtime ? std::string(runtime) + "/dvp-monitor.sock" :
        "/tmp/dvp-monitor-" + std::to_string(getuid()) + "/control.sock";
}
// One bounded JSON packet/reply per connection. Same UID, no network listener.
class Server {
    struct Client { int fd; std::chrono::steady_clock::time_point expires; };
    int fd_ = -1;
    std::string path_;
    std::vector<Client> clients_;
public:
    explicit Server(std::string path) : path_(path == "auto" ? default_socket() : path) {
        auto parent = std::filesystem::path(path_).parent_path();
        if (parent.empty()) throw std::runtime_error("Control socket needs a private parent directory");
        if (!std::filesystem::exists(parent) && mkdir(parent.c_str(), 0700) != 0)
            throw std::runtime_error("Cannot create socket directory");
        struct stat st{};
        if (lstat(parent.c_str(), &st) || !S_ISDIR(st.st_mode) || st.st_uid != getuid() || (st.st_mode & 0077))
            throw std::runtime_error("Socket parent must be an owned private directory (0700)");
        sockaddr_un addr{}; addr.sun_family = AF_UNIX;
        if (path_.size() >= sizeof(addr.sun_path)) throw std::runtime_error("Socket path too long");
        std::memcpy(addr.sun_path, path_.c_str(), path_.size()+1);
        fd_ = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
        if (fd_ < 0) throw std::runtime_error("Cannot create control socket");
        if (bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
            close(fd_); fd_ = -1;
            throw std::runtime_error("Cannot bind socket (existing socket is never replaced): " + path_);
        }
        if (chmod(path_.c_str(), 0600) || listen(fd_, 8)) {
            close(fd_); fd_ = -1; unlink(path_.c_str());
            throw std::runtime_error("Cannot secure/listen on control socket");
        }
    }
    ~Server() {
        for (auto& client : clients_) close(client.fd);
        if (fd_ >= 0) { close(fd_); unlink(path_.c_str()); }
    }
    Server(const Server&) = delete;
    Server& operator=(const Server&) = delete;
    void poll(const std::function<Json(json_object*)>& handler) {
        for (int n = 0; n < 8 && clients_.size() < 8; ++n) {
            int client = accept4(fd_, nullptr, nullptr, SOCK_NONBLOCK | SOCK_CLOEXEC);
            if (client < 0) break;
            ucred peer{}; socklen_t size = sizeof(peer);
            if (getsockopt(client, SOL_SOCKET, SO_PEERCRED, &peer, &size) || peer.uid != getuid()) { close(client); continue; }
            clients_.push_back({client, std::chrono::steady_clock::now() + std::chrono::seconds(1)});
        }
        for (auto it = clients_.begin(); it != clients_.end();) {
            char data[4096];
            ssize_t size = recv(it->fd, data, sizeof(data), MSG_DONTWAIT | MSG_TRUNC);
            if (size < 0 && (errno == EAGAIN || errno == EWOULDBLOCK) && std::chrono::steady_clock::now() < it->expires) { ++it; continue; }
            if (size > 0) {
                Json response(nullptr, json_object_put);
                try {
                    if (size > static_cast<ssize_t>(sizeof(data))) throw std::runtime_error("Request exceeds 4096 bytes");
                    auto request = parse(std::string(data, size));
                    response = handler(request.get());
                } catch (const std::exception& e) { response = error("request_failed", e.what()); }
                std::string raw = dump(response.get());
                // A lost reply does not roll back an action. Client queries same run ID.
                send(it->fd, raw.data(), raw.size(), MSG_NOSIGNAL | MSG_DONTWAIT);
            }
            close(it->fd); it = clients_.erase(it);
        }
    }
};
} // namespace control
