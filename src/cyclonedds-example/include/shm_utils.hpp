#pragma once

#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

class SharedMemory {
    protected:
        const char* name_;
        size_t size_;
        void* ptr_ {nullptr};

        SharedMemory(const char* name, size_t size)
            : name_ (name), size_ (size) 
        {}

    public:
        SharedMemory(const SharedMemory&) = delete;
        SharedMemory& operator=(const SharedMemory&) = delete;

        virtual ~SharedMemory() = default;

        template <typename T>
        T* get() const { return reinterpret_cast<T*>(ptr_); }

        void* raw_ptr() const { return ptr_; }
        bool is_valid() const { return ptr_ != nullptr; }
};


class SharedMemoryOwner : public SharedMemory {
    public:
        SharedMemoryOwner(const char* name, size_t size) 
            : SharedMemory(name, size) 
        {
            // Clear potential residue from previous crashes
            shm_unlink(name_);

            // Create shared memory object in /dev/shm
            int fd = shm_open(name_, O_CREAT | O_EXCL | O_RDWR, 0666);
            if (fd < 0)
                return;

            // Set its size to the size of our structure
            if (ftruncate(fd, size_) == -1) {
                close(fd);
                return;
            }

            // Map the object into the caller's address space
            ptr_ = mmap(nullptr, size_, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
            close(fd);
            if (ptr_ == MAP_FAILED) 
                return;
        }

        ~SharedMemoryOwner() override {
            if (ptr_ && ptr_ != MAP_FAILED)
                munmap(ptr_, size_);

            // Unlink the shared memory object from /dev/shm
            // Even if the peer process is still using the object, this is okay. 
            // The object will be removed only after all open references are closed
            shm_unlink(name_);
        }
};

class SharedMemoryClient : public SharedMemory {
    public:
        SharedMemoryClient(const char* name, size_t size)
            : SharedMemory(name, size)
        {
            // Open existing shared memory object and map it into the caller's address space
            int fd = shm_open(name_, O_RDWR, 0666);
            if (fd < 0) 
                return;

            ptr_ = mmap(nullptr, size_, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
            close(fd);
            if (ptr_ == MAP_FAILED) 
                return;
        }

        ~SharedMemoryClient() override {
            if (ptr_ && ptr_ != MAP_FAILED)
                munmap(ptr_, size_);
        }
};  