/* This file is part of the dynarmic project.
 * Copyright (c) 2016 MerryMage
 * SPDX-License-Identifier: 0BSD
 */

#include "dynarmic/common/memory_pool.h"

#include <cstdlib>

namespace Dynarmic::Common {

Pool::Pool(size_t object_size, size_t initial_pool_size)
        : Pool(object_size, initial_pool_size, initial_pool_size) {}

Pool::Pool(size_t object_size, size_t initial_pool_size, size_t max_pool_size)
        : object_size(object_size), slab_size(initial_pool_size), max_slab_size(max_pool_size) {
    AllocateNewSlab();
}

Pool::~Pool() {
    std::free(current_slab);

    for (char* slab : slabs) {
        std::free(slab);
    }
}

void* Pool::Alloc() {
    if (remaining == 0) {
        slabs.push_back(current_slab);
        if (slab_size < max_slab_size) {
            slab_size = slab_size * 2 < max_slab_size ? slab_size * 2 : max_slab_size;
        }
        AllocateNewSlab();
    }

    void* ret = static_cast<void*>(current_ptr);
    current_ptr += object_size;
    remaining--;

    return ret;
}

void Pool::AllocateNewSlab() {
    current_slab = static_cast<char*>(std::malloc(object_size * slab_size));
    current_ptr = current_slab;
    remaining = slab_size;
}

}  // namespace Dynarmic::Common
