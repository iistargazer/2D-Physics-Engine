#include <cstddef>
#include <cstdlib>
#include <new>

std::size_t testAllocations = 0;
bool countTestAllocations = false;

void* operator new(std::size_t size) {
    if (void* p = std::malloc(size ? size : 1)) {
        if (countTestAllocations) ++testAllocations;
        return p;
    }
    throw std::bad_alloc();
}
void* operator new[](std::size_t size) { return ::operator new(size); }
void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { ::operator delete(p); }
void operator delete(void* p, std::size_t) noexcept { ::operator delete(p); }
void operator delete[](void* p, std::size_t) noexcept { ::operator delete(p); }
