#pragma once

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdexcept>
#include <string>

// POSIX 공유메모리 래퍼.
// owner=true  : shm 생성 + 소유 (쓰는 쪽, 소멸 시 shm_unlink)
// owner=false : 기존 shm 연결  (읽는 쪽)
template <typename T>
class SharedMemory {
public:
    SharedMemory(const std::string& name, bool owner)
        : name_(name), owner_(owner)
    {
        int flags = owner_ ? (O_CREAT | O_RDWR) : O_RDWR;
        fd_ = shm_open(name_.c_str(), flags, 0666);
        if (fd_ < 0)
            throw std::runtime_error("shm_open failed: " + name_);

        if (owner_) {
            if (ftruncate(fd_, sizeof(T)) < 0)
                throw std::runtime_error("ftruncate failed: " + name_);
        }

        ptr_ = static_cast<T*>(
            mmap(nullptr, sizeof(T), PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0));
        if (ptr_ == MAP_FAILED) {
            ptr_ = nullptr;
            throw std::runtime_error("mmap failed: " + name_);
        }
    }

    ~SharedMemory() {
        if (ptr_) munmap(ptr_, sizeof(T));
        if (fd_ >= 0) close(fd_);
        if (owner_) shm_unlink(name_.c_str());
    }

    SharedMemory(const SharedMemory&)            = delete;
    SharedMemory& operator=(const SharedMemory&) = delete;

    T*       get()       { return ptr_; }
    const T* get() const { return ptr_; }

    T&       data()       { return *ptr_; }
    const T& data() const { return *ptr_; }

    void write(const T& val) { *ptr_ = val; }
    T    read()  const       { return *ptr_; }

private:
    std::string name_;
    bool        owner_;
    int         fd_  = -1;
    T*          ptr_ = nullptr;
};
