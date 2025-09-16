#pragma once

#include <vector>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <type_traits> 

namespace serial_device {

template <typename T>
concept TriviallyCopyable = std::is_trivially_copyable_v<T>;


template <TriviallyCopyable T>
inline T fromVector(const std::vector<uint8_t>& data)
{
    if (data.size() < sizeof(T)) {
        throw std::runtime_error("Data size insufficient for type conversion");
    }

    T obj;
    std::memcpy(&obj, data.data(), sizeof(T));
    return obj;
}

template <TriviallyCopyable T>
inline std::vector<uint8_t> toVector(const T& obj)
{
    //static_assert(std::is_pod_v<T>, "toVector only supports POD types!");
    std::vector<uint8_t> data(sizeof(T));
    std::memcpy(data.data(), &obj, sizeof(T));
    return data;
}

} // namespace serial_device