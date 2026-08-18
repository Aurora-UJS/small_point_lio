/**
 * 最小 CDR（XCDR1，小端）编解码器。
 *
 * 只覆盖 sensor_msgs/Imu 和 sensor_msgs/PointCloud2 用到的那几种类型。
 * 布局是照着 `ros2 bag record -s mcap` 真录出来的字节反推并逐字节核对过的，
 * 所以写出来的包 `ros2 bag play` 能直接放。
 *
 * 规则（对齐都相对于「封装头之后」的正文起点，不是缓冲区起点）：
 *   - 封装头 4 字节：00 01 00 00
 *   - 基本类型按自身大小对齐（int32→4，float64→8，uint8/bool→1）
 *   - string：uint32 长度（含结尾 '\0'）+ 字符 + '\0'
 *   - 变长序列：uint32 元素个数 + 元素
 */

#pragma once

#include <pch.h>

#include <cstring>
#include <stdexcept>

namespace small_point_lio::transport::cdr {

    inline constexpr size_t ENCAPSULATION_SIZE = 4;

    class Writer {
    public:
        Writer() {
            buffer = {0x00, 0x01, 0x00, 0x00};// CDR_LE，options=0
        }

        void u8(uint8_t value) { raw(&value, 1); }
        void boolean(bool value) { u8(value ? 1 : 0); }
        void u32(uint32_t value) { align(4), raw(&value, 4); }
        void i32(int32_t value) { align(4), raw(&value, 4); }
        void f32(float value) { align(4), raw(&value, 4); }
        void f64(double value) { align(8), raw(&value, 8); }

        void string(const std::string &value) {
            u32(static_cast<uint32_t>(value.size() + 1));
            raw(value.data(), value.size());
            u8(0);
        }

        void bytes(const void *data, size_t size) {
            u32(static_cast<uint32_t>(size));
            raw(data, size);
        }

        /// 预留 size 字节的序列空间并返回可写指针（避免大点云多一次拷贝）。
        uint8_t *bytes_uninitialized(size_t size) {
            u32(static_cast<uint32_t>(size));
            size_t offset = buffer.size();
            buffer.resize(offset + size);
            return buffer.data() + offset;
        }

        const std::vector<uint8_t> &data() const { return buffer; }

    private:
        void align(size_t boundary) {
            while ((buffer.size() - ENCAPSULATION_SIZE) % boundary != 0) {
                buffer.push_back(0);
            }
        }
        void raw(const void *source, size_t size) {
            size_t offset = buffer.size();
            buffer.resize(offset + size);
            std::memcpy(buffer.data() + offset, source, size);
        }

        std::vector<uint8_t> buffer;
    };

    class Reader {
    public:
        Reader(const uint8_t *data, size_t size) : begin(data), end(data + size), cursor(data) {
            if (size < ENCAPSULATION_SIZE) {
                throw std::runtime_error("CDR 数据太短，连封装头都不够");
            }
            if (data[1] != 0x01) {
                throw std::runtime_error("只支持小端 CDR（封装头 00 01 00 00）");
            }
            cursor += ENCAPSULATION_SIZE;
        }

        uint8_t u8() { return read<uint8_t>(1); }
        bool boolean() { return u8() != 0; }
        uint32_t u32() { return align(4), read<uint32_t>(4); }
        int32_t i32() { return align(4), read<int32_t>(4); }
        float f32() { return align(4), read<float>(4); }
        double f64() { return align(8), read<double>(8); }

        std::string string() {
            uint32_t length = u32();
            require(length);
            std::string value(reinterpret_cast<const char *>(cursor), length > 0 ? length - 1 : 0);
            cursor += length;
            return value;
        }

        /// 返回序列起点并跳过它，不做拷贝。
        const uint8_t *bytes(size_t &size) {
            uint32_t count = u32();
            require(count);
            const uint8_t *start = cursor;
            cursor += count;
            size = count;
            return start;
        }

        void skip(size_t size) { require(size), cursor += size; }

    private:
        void align(size_t boundary) {
            size_t offset = static_cast<size_t>(cursor - begin) - ENCAPSULATION_SIZE;
            size_t padding = (boundary - offset % boundary) % boundary;
            require(padding);
            cursor += padding;
        }
        void require(size_t size) {
            if (static_cast<size_t>(end - cursor) < size) {
                throw std::runtime_error("CDR 数据被截断");
            }
        }
        template<typename T>
        T read(size_t size) {
            require(size);
            T value;
            std::memcpy(&value, cursor, size);
            cursor += size;
            return value;
        }

        const uint8_t *begin;
        const uint8_t *end;
        const uint8_t *cursor;
    };

}// namespace small_point_lio::transport::cdr
