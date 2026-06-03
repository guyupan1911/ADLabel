#pragma once

#include <cstdint>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

#include <glog/logging.h>

namespace adlabel {
namespace mapping {

template <typename T>
std::vector<T> ReadMetaFile(const std::string& meta_path) {
    std::vector<T> messages;
    std::ifstream ifs(meta_path, std::ios::binary);
    if (!ifs) {
        LOG(ERROR) << "failed to open meta file: " << meta_path;
        return messages;
    }

    size_t record_no = 0;
    while (true) {
        uint8_t header[4];
        ifs.read(reinterpret_cast<char*>(header), sizeof(header));
        if (ifs.eof() && ifs.gcount() == 0) {
            break;  // clean EOF on record boundary
        }
        if (!ifs || ifs.gcount() != static_cast<std::streamsize>(sizeof(header))) {
            LOG(ERROR) << "truncated length header at record " << record_no << " of " << meta_path;
            break;
        }
        const uint32_t size =
                static_cast<uint32_t>(header[0]) | (static_cast<uint32_t>(header[1]) << 8) |
                (static_cast<uint32_t>(header[2]) << 16) | (static_cast<uint32_t>(header[3]) << 24);
        std::string buf(size, '\0');
        if (size > 0) {
            ifs.read(&buf[0], static_cast<std::streamsize>(size));
            if (!ifs || ifs.gcount() != static_cast<std::streamsize>(size)) {
                LOG(ERROR) << "truncated payload at record " << record_no << " (expected " << size
                           << " bytes) of " << meta_path;
                break;
            }
        }
        T msg;
        if (!msg.ParseFromString(buf)) {
            LOG(WARNING) << "failed to parse record " << record_no << " of " << meta_path;
        } else {
            messages.push_back(std::move(msg));
        }
        ++record_no;
    }
    LOG(INFO) << "read " << messages.size() << " records from " << meta_path;
    return messages;
}


template <typename T>
bool WriteMetaFile(const std::string& meta_path, const std::vector<T>& messages) {
    std::ofstream ofs(meta_path, std::ios::binary | std::ios::trunc);
    if (!ofs) {
        LOG(ERROR) << "failed to open meta file for writing: " << meta_path;
        return false;
    }
    for (const auto& msg : messages) {
        const std::string buf = msg.SerializeAsString();
        if (buf.size() > std::numeric_limits<uint32_t>::max()) {
            LOG(ERROR) << "message too large to serialize (" << buf.size() << " bytes)";
            return false;
        }
        const uint32_t size = static_cast<uint32_t>(buf.size());
        const uint8_t header[4] = {
                static_cast<uint8_t>(size & 0xFF),
                static_cast<uint8_t>((size >> 8) & 0xFF),
                static_cast<uint8_t>((size >> 16) & 0xFF),
                static_cast<uint8_t>((size >> 24) & 0xFF),
        };
        ofs.write(reinterpret_cast<const char*>(header), sizeof(header));
        ofs.write(buf.data(), static_cast<std::streamsize>(buf.size()));
    }
    return ofs.good();
}

}  // namespace mapping
}  // namespace adlabel
