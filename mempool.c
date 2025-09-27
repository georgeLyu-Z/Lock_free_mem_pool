#include "mempool.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/mman.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <stdatomic.h>

// 系统配置 - 全局原子变量独占缓存行
static size_t PAGE_SIZE = 0;
static mem_pool_t *pool = NULL;

// 全局原子变量独占缓存行
static atomic_int init_flag __attribute__((aligned(CACHE_LINE_SIZE))) = 0;

// 常量定义
#define DEFAULT_HASH_SIZE 128
#define ALIGNMENT 16
#define MIN_BLOCK_SIZE (sizeof(header_t) + sizeof(footer_t))
#define MAGIC 0xDEADBEEF
#define MAX_RETRY_ATTEMPTS 10
#define CLEANUP_INTERVAL 256  // 每256次操作清理一次空页面

// 缓存行大小定义（通常为64字节）
#define CACHE_LINE_SIZE 64

// 计算各种原子类型的缓存行填充大小
#define CACHE_LINE_PADDING(type) (CACHE_LINE_SIZE - sizeof(type))

// 确保结构体对齐到缓存行边界
#define CACHE_ALIGNED __attribute__((aligned(CACHE_LINE_SIZE)))

// 优化的数据结构 - 每个原子变量独占缓存行
typedef struct header CACHE_ALIGNED {
    atomic_size_t size;
    char padding1[CACHE_LINE_PADDING(atomic_size_t)];
    
    atomic_char is_free;
    char padding2[CACHE_LINE_PADDING(atomic_char)];
    
    atomic_uintptr_t next_free;
    char padding3[CACHE_LINE_PADDING(atomic_uintptr_t)];
    
    atomic_uintptr_t prev_free;
    char padding4[CACHE_LINE_PADDING(atomic_uintptr_t)];
    
    atomic_uint32_t magic;
    char padding5[CACHE_LINE_PADDING(atomic_uint32_t)];
    
    atomic_uint32_t version;
    char padding6[CACHE_LINE_PADDING(atomic_uint32_t)];
} header_t;

typedef struct footer CACHE_ALIGNED {
    atomic_size_t size;
    char padding1[CACHE_LINE_PADDING(atomic_size_t)];
    
    atomic_uint32_t magic;
    char padding2[CACHE_LINE_PADDING(atomic_uint32_t)];
} footer_t;

typedef struct page_header CACHE_ALIGNED {
    atomic_uintptr_t next;
    char padding1[CACHE_LINE_PADDING(atomic_uintptr_t)];
    
    struct mem_pool *pool;
    char padding2[CACHE_LINE_PADDING(struct mem_pool*)];
    
    atomic_size_t block_count;
    char padding3[CACHE_LINE_PADDING(atomic_size_t)];
    
    atomic_uint32_t version;
    char padding4[CACHE_LINE_PADDING(atomic_uint32_t)];
} page_header_t;

typedef struct mem_pool CACHE_ALIGNED {
    atomic_uintptr_t *hash_table;
    size_t hash_size;
    char padding1[CACHE_LINE_PADDING(atomic_uintptr_t*) + CACHE_LINE_PADDING(size_t)];
    
    atomic_uintptr_t page_list;
    char padding2[CACHE_LINE_PADDING(atomic_uintptr_t)];
    
    atomic_int is_initialized;
    char padding3[CACHE_LINE_PADDING(atomic_int)];
    
    // 统计信息 - 每个统计变量独占缓存行
    atomic_size_t total_allocated;
    char padding4[CACHE_LINE_PADDING(atomic_size_t)];
    
    atomic_size_t total_free;
    char padding5[CACHE_LINE_PADDING(atomic_size_t)];
    
    atomic_size_t fragmentation_count;
    char padding6[CACHE_LINE_PADDING(atomic_size_t)];
    
    atomic_size_t total_pages;
    char padding7[CACHE_LINE_PADDING(atomic_size_t)];
    
    atomic_size_t active_pages;
    char padding8[CACHE_LINE_PADDING(atomic_size_t)];
} mem_pool_t;

// 原子操作宏 - 直接使用标准内存序
#define CAS_PTR(ptr, expected, desired) \
    atomic_compare_exchange_weak_explicit(ptr, expected, desired, \
        memory_order_acq_rel, memory_order_acquire)

#define LOAD_PTR(ptr) atomic_load_explicit(ptr, memory_order_acquire)
#define STORE_PTR(ptr, value) atomic_store_explicit(ptr, value, memory_order_release)

// 统计更新宏（使用宽松内存序）
#define UPDATE_STATS(p, alloc_change, free_change, frag_change) do { \
    if (alloc_change) atomic_fetch_add_explicit(&(p)->total_allocated, alloc_change, memory_order_relaxed); \
    if (free_change) atomic_fetch_add_explicit(&(p)->total_free, free_change, memory_order_relaxed); \
    if (frag_change) atomic_fetch_add_explicit(&(p)->fragmentation_count, frag_change, memory_order_relaxed); \
} while(0)

// 内联辅助函数
static inline footer_t *get_footer(header_t *h) {
    return (footer_t *)((char *)h + atomic_load_explicit(&h->size, memory_order_acquire) - sizeof(footer_t));
}

static inline header_t *get_header(void *ptr) {
    return (header_t *)((char *)ptr - sizeof(header_t));
}

static inline void *get_payload(header_t *h) {
    return (void *)((char *)h + sizeof(header_t));
}

static inline uintptr_t align_up(uintptr_t v, size_t a) {
    assert((a & (a - 1)) == 0);
    return (v + a - 1) & ~(uintptr_t)(a - 1);
}

// 优化的哈希函数 - 使用位运算优化
static inline size_t hash_function(size_t s, size_t hash_size) {
    // 对于2的幂次方大小，使用位运算
    if ((hash_size & (hash_size - 1)) == 0) {
        return s & (hash_size - 1);
    }
    return s % hash_size;
}

// 初始化页面大小
static int init_page_size() {
    if (PAGE_SIZE == 0) {
        long page_size = sysconf(_SC_PAGESIZE);
        PAGE_SIZE = (page_size > 0) ? (size_t)page_size : 4096;
    }
    return 0;
}

// 优化的块初始化
static inline void init_block(header_t *block, size_t size, int is_free) {
    atomic_store_explicit(&block->size, size, memory_order_release);
    atomic_store_explicit(&block->magic, MAGIC, memory_order_release);
    atomic_fetch_add_explicit(&block->version, 1, memory_order_release);
    STORE_PTR(&block->next_free, 0);
    STORE_PTR(&block->prev_free, 0);
    atomic_store_explicit(&block->is_free, is_free, memory_order_release);
    
    footer_t *footer = get_footer(block);
    atomic_store_explicit(&footer->size, size, memory_order_release);
    atomic_store_explicit(&footer->magic, MAGIC, memory_order_release);
}

// 优化的块验证
static inline int is_block_valid(header_t *b) {
    if (!b) return 0;
    
    size_t size = atomic_load_explicit(&b->size, memory_order_acquire);
    uint32_t magic = atomic_load_explicit(&b->magic, memory_order_acquire);
    
    if (magic != MAGIC || size % ALIGNMENT != 0 || size < MIN_BLOCK_SIZE) {
        return 0;
    }
    
    footer_t *f = get_footer(b);
    if (!f) return 0;
    
    size_t footer_size = atomic_load_explicit(&f->size, memory_order_acquire);
    uint32_t footer_magic = atomic_load_explicit(&f->magic, memory_order_acquire);
    
    return (footer_size == size && footer_magic == MAGIC);
}

// 优化的空闲链表操作
static int remove_from_free_list(header_t *b) {
    if (!b || !atomic_load_explicit(&b->is_free, memory_order_acquire)) return 0;
    
    size_t i = hash_function(atomic_load_explicit(&b->size, memory_order_acquire), pool->hash_size);
    atomic_uintptr_t *bucket = &pool->hash_table[i];
    
    uint32_t version = atomic_load_explicit(&b->version, memory_order_acquire);
    
    for (int retry = 0; retry < MAX_RETRY_ATTEMPTS; retry++) {
        uintptr_t expected = (uintptr_t)b;
        uintptr_t desired = LOAD_PTR(&b->next_free);
        
        if (CAS_PTR(bucket, &expected, desired)) {
            goto success;
        }
        
        // 遍历链表查找并移除
        header_t *prev = NULL;
        uintptr_t current = LOAD_PTR(bucket);
        
        while (current != 0) {
            header_t *cur_block = (header_t *)current;
            
            if (cur_block == b) {
                uint32_t current_version = atomic_load_explicit(&cur_block->version, memory_order_acquire);
                if (current_version != version) {
                    version = atomic_load_explicit(&b->version, memory_order_acquire);
                    break;
                }
                
                if (prev) {
                    STORE_PTR(&prev->next_free, LOAD_PTR(&b->next_free));
                }
                goto success;
            }
            prev = cur_block;
            current = LOAD_PTR(&cur_block->next_free);
        }
    }
    
    return 0;

success:
    uintptr_t next_ptr = LOAD_PTR(&b->next_free);
    if (next_ptr != 0) {
        header_t *next_block = (header_t *)next_ptr;
        STORE_PTR(&next_block->prev_free, LOAD_PTR(&b->prev_free));
    }
    
    STORE_PTR(&b->next_free, 0);
    STORE_PTR(&b->prev_free, 0);
    atomic_store_explicit(&b->is_free, 0, memory_order_release);
    atomic_fetch_add_explicit(&b->version, 1, memory_order_release);
    
    return 1;
}

static int add_to_free_list(header_t *b) {
    if (!b || atomic_load_explicit(&b->is_free, memory_order_acquire)) return 0;
    
    size_t i = hash_function(atomic_load_explicit(&b->size, memory_order_acquire), pool->hash_size);
    atomic_uintptr_t *bucket = &pool->hash_table[i];
    
    uintptr_t expected = LOAD_PTR(bucket);
    uintptr_t desired = (uintptr_t)b;
    
    int retry_count = 0;
    do {
        STORE_PTR(&b->next_free, expected);
        STORE_PTR(&b->prev_free, 0);
        if (++retry_count > MAX_RETRY_ATTEMPTS) return 0;
    } while (!CAS_PTR(bucket, &expected, desired));
    
    if (expected != 0) {
        header_t *next_block = (header_t *)expected;
        STORE_PTR(&next_block->prev_free, (uintptr_t)b);
    }
    
    atomic_store_explicit(&b->is_free, 1, memory_order_release);
    atomic_fetch_add_explicit(&b->version, 1, memory_order_release);
    
    return 1;
}

// 页管理函数
static int is_page_empty(page_header_t *page) {
    uintptr_t page_start = (uintptr_t)page;
    uintptr_t aligned_start = align_up(page_start + sizeof(page_header_t), ALIGNMENT);
    header_t *first_block = (header_t *)aligned_start;
    
    size_t expected_size = PAGE_SIZE - (aligned_start - page_start);
    expected_size = align_up(expected_size, ALIGNMENT);
    
    return (atomic_load_explicit(&first_block->is_free, memory_order_acquire) && 
            atomic_load_explicit(&first_block->size, memory_order_acquire) == expected_size &&
            is_block_valid(first_block));
}

static void free_empty_pages() {
    uintptr_t current = LOAD_PTR(&pool->page_list);
    uintptr_t prev = 0;
    
    while (current != 0) {
        page_header_t *page = (page_header_t *)current;
        uintptr_t next = LOAD_PTR(&page->next);
        
        if (is_page_empty(page)) {
            uint32_t page_version = atomic_load_explicit(&page->version, memory_order_acquire);
            
            if (is_page_empty(page) && 
                atomic_load_explicit(&page->version, memory_order_acquire) == page_version) {
                
                uintptr_t page_start = (uintptr_t)page;
                uintptr_t aligned_start = align_up(page_start + sizeof(page_header_t), ALIGNMENT);
                header_t *first_block = (header_t *)aligned_start;
                
                if (remove_from_free_list(first_block)) {
                    if (prev == 0) {
                        CAS_PTR(&pool->page_list, &current, next);
                    } else {
                        page_header_t *prev_page = (page_header_t *)prev;
                        CAS_PTR(&prev_page->next, &current, next);
                    }
                    
                    munmap(page, PAGE_SIZE);
                    atomic_fetch_sub_explicit(&pool->active_pages, 1, memory_order_relaxed);
                    current = next;
                    continue;
                }
            }
        }
        
        prev = current;
        current = next;
    }
}

static header_t *allocate_page() {
    void *page = mmap(NULL, PAGE_SIZE, PROT_READ | PROT_WRITE, 
                     MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (page == MAP_FAILED) {
        return NULL;
    }

    page_header_t *ph = page;
    STORE_PTR(&ph->next, LOAD_PTR(&pool->page_list));
    ph->pool = pool;
    atomic_store_explicit(&ph->block_count, 1, memory_order_release);
    atomic_store_explicit(&ph->version, 0, memory_order_release);
    
    // 原子地将页面添加到页面列表
    uintptr_t expected = LOAD_PTR(&pool->page_list);
    uintptr_t desired = (uintptr_t)ph;
    if (!CAS_PTR(&pool->page_list, &expected, desired)) {
        munmap(page, PAGE_SIZE);
        return NULL;
    }

    uintptr_t page_start = (uintptr_t)ph;
    uintptr_t aligned_start = align_up(page_start + sizeof(page_header_t), ALIGNMENT);
    
    header_t *b = (header_t *)aligned_start;
    size_t block_size = align_up(PAGE_SIZE - (aligned_start - page_start), ALIGNMENT);
    
    init_block(b, block_size, 1);
    
    // 尝试将块添加到空闲列表
    if (!add_to_free_list(b)) {
        // 清理：从页面列表中移除页面
        uintptr_t current = LOAD_PTR(&pool->page_list);
        if (current == (uintptr_t)ph) {
            uintptr_t next = LOAD_PTR(&ph->next);
            CAS_PTR(&pool->page_list, &current, next);
        }
        munmap(page, PAGE_SIZE);
        return NULL;
    }
    
    // 更新统计信息
    UPDATE_STATS(pool, 0, block_size, 0);
    atomic_fetch_add_explicit(&pool->total_pages, 1, memory_order_relaxed);
    atomic_fetch_add_explicit(&pool->active_pages, 1, memory_order_relaxed);
    
    return b;
}

// 优化的查找算法
static header_t *find_best_fit_block(size_t s) {
    header_t *best = NULL;
    size_t min_diff = SIZE_MAX;
    
    size_t start_idx = hash_function(s, pool->hash_size);
    
    // 首先检查目标桶
    uintptr_t current = LOAD_PTR(&pool->hash_table[start_idx]);
    while (current != 0) {
        header_t *cur = (header_t *)current;
        
        if (atomic_load_explicit(&cur->is_free, memory_order_acquire)) {
            size_t cur_size = atomic_load_explicit(&cur->size, memory_order_acquire);
            if (cur_size >= s) {
                size_t diff = cur_size - s;
                if (diff == 0) return cur; // 精确匹配，立即返回
                if (diff < min_diff) {
                    min_diff = diff;
                    best = cur;
                }
            }
        }
        
        current = LOAD_PTR(&cur->next_free);
    }
    
    // 如果没有找到合适的块，搜索相邻桶
    for (size_t offset = 1; offset < pool->hash_size / 4 && min_diff > s / 4; offset++) {
        size_t left_idx = (start_idx - offset + pool->hash_size) % pool->hash_size;
        size_t right_idx = (start_idx + offset) % pool->hash_size;
        
        // 搜索左侧桶
        current = LOAD_PTR(&pool->hash_table[left_idx]);
        while (current != 0) {
            header_t *cur = (header_t *)current;
            
            if (atomic_load_explicit(&cur->is_free, memory_order_acquire)) {
                size_t cur_size = atomic_load_explicit(&cur->size, memory_order_acquire);
                if (cur_size >= s) {
                    size_t diff = cur_size - s;
                    if (diff == 0) return cur; // 精确匹配，立即返回
                    if (diff < min_diff) {
                        min_diff = diff;
                        best = cur;
                    }
                }
            }
            
            current = LOAD_PTR(&cur->next_free);
        }
        
        // 搜索右侧桶
        current = LOAD_PTR(&pool->hash_table[right_idx]);
        while (current != 0) {
            header_t *cur = (header_t *)current;
            
            if (atomic_load_explicit(&cur->is_free, memory_order_acquire)) {
                size_t cur_size = atomic_load_explicit(&cur->size, memory_order_acquire);
                if (cur_size >= s) {
                    size_t diff = cur_size - s;
                    if (diff == 0) return cur; // 精确匹配，立即返回
                    if (diff < min_diff) {
                        min_diff = diff;
                        best = cur;
                    }
                }
            }
            
            current = LOAD_PTR(&cur->next_free);
        }
    }
    
    return best;
}

// 公共API实现
int mempool_init(size_t hash_size) {
    int expected = 0;
    if (!atomic_compare_exchange_strong_explicit(&init_flag, &expected, 1, 
        memory_order_acq_rel, memory_order_acquire)) {
        return 0; // 已经初始化或正在初始化
    }
    
    if (pool && atomic_load_explicit(&pool->is_initialized, memory_order_acquire)) {
        atomic_store_explicit(&init_flag, 0, memory_order_release);
        return 0;
    }

    if (pool) mempool_destroy();

    if (init_page_size() != 0) {
        atomic_store_explicit(&init_flag, 0, memory_order_release);
        return -1;
    }

    size_t hsize = hash_size ?: DEFAULT_HASH_SIZE;
    pool = mmap(NULL, sizeof(mem_pool_t), PROT_READ | PROT_WRITE, 
               MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (pool == MAP_FAILED) {
        atomic_store_explicit(&init_flag, 0, memory_order_release);
        return -1;
    }

    pool->hash_size = hsize;
    pool->hash_table = mmap(NULL, sizeof(atomic_uintptr_t) * hsize,
                           PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (pool->hash_table == MAP_FAILED) {
        munmap(pool, sizeof(mem_pool_t));
        pool = NULL;
        atomic_store_explicit(&init_flag, 0, memory_order_release);
        return -1;
    }

    // 初始化哈希表
    for (size_t i = 0; i < hsize; i++) {
        STORE_PTR(&pool->hash_table[i], 0);
    }
    
    STORE_PTR(&pool->page_list, 0);
    atomic_store_explicit(&pool->is_initialized, 0, memory_order_release);
    
    // 初始化统计信息
    atomic_store_explicit(&pool->total_allocated, 0, memory_order_relaxed);
    atomic_store_explicit(&pool->total_free, 0, memory_order_relaxed);
    atomic_store_explicit(&pool->fragmentation_count, 0, memory_order_relaxed);
    atomic_store_explicit(&pool->total_pages, 0, memory_order_relaxed);
    atomic_store_explicit(&pool->active_pages, 0, memory_order_relaxed);

    atomic_store_explicit(&pool->is_initialized, 1, memory_order_release);
    
    if (!allocate_page()) {
        atomic_store_explicit(&pool->is_initialized, 0, memory_order_release);
        munmap(pool->hash_table, sizeof(atomic_uintptr_t) * hsize);
        munmap(pool, sizeof(mem_pool_t));
        pool = NULL;
        atomic_store_explicit(&init_flag, 0, memory_order_release);
        return -1;
    }

    atomic_store_explicit(&init_flag, 0, memory_order_release);
    return 0;
}

void mempool_destroy() {
    int expected = 0;
    if (!atomic_compare_exchange_strong_explicit(&init_flag, &expected, 1, 
        memory_order_acq_rel, memory_order_acquire)) {
        return;
    }
    
    if (!pool) {
        atomic_store_explicit(&init_flag, 0, memory_order_release);
        return;
    }

    if (atomic_load_explicit(&pool->is_initialized, memory_order_acquire)) {
        uintptr_t current = LOAD_PTR(&pool->page_list);
        while (current != 0) {
            page_header_t *page = (page_header_t *)current;
            uintptr_t next = LOAD_PTR(&page->next);
            munmap(page, PAGE_SIZE);
            current = next;
        }
        
        STORE_PTR(&pool->page_list, 0);
        atomic_store_explicit(&pool->is_initialized, 0, memory_order_release);
    }

    if (pool->hash_table) {
        munmap(pool->hash_table, sizeof(atomic_uintptr_t) * pool->hash_size);
    }
    
    munmap(pool, sizeof(mem_pool_t));
    pool = NULL;
    
    atomic_store_explicit(&init_flag, 0, memory_order_release);
}

void *mempool_alloc(size_t size) {
    // 参数验证
    if (!size || !pool || !atomic_load_explicit(&pool->is_initialized, memory_order_acquire)) {
        return NULL;
    }

    // 计算对齐后的请求大小
    size_t req = align_up(size + sizeof(header_t) + sizeof(footer_t), ALIGNMENT);
    req = req < MIN_BLOCK_SIZE ? MIN_BLOCK_SIZE : req;
    
    // 查找最佳匹配块
    header_t *b = find_best_fit_block(req);
    
    // 如果没有找到合适的块，尝试分配新页面
    if (!b) {
        if (!allocate_page()) {
            return NULL;
        }
        b = find_best_fit_block(req);
        if (!b) {
            return NULL;
        }
    }

    // 尝试从空闲列表中移除块
    for (int retry = 0; retry < MAX_RETRY_ATTEMPTS; retry++) {
        if (remove_from_free_list(b)) {
            break;
        }
        // 如果移除失败，重新查找块
        b = find_best_fit_block(req);
        if (!b) {
            return NULL;
        }
    }
    
    size_t original_size = atomic_load_explicit(&b->size, memory_order_acquire);
    
    // 块拆分优化：只在剩余空间足够大时拆分
    if (original_size - req >= MIN_BLOCK_SIZE) {
        header_t *nb = (header_t *)((char *)b + req);
        init_block(nb, original_size - req, 1);
        init_block(b, req, 0);
        
        // 将新块添加到空闲列表
        if (!add_to_free_list(nb)) {
            // 如果添加失败，合并回原块
            init_block(b, original_size, 0);
            UPDATE_STATS(pool, original_size, -original_size, 0);
        } else {
            UPDATE_STATS(pool, req, -original_size, 1);
        }
    } else {
        init_block(b, original_size, 0);
        UPDATE_STATS(pool, original_size, -original_size, 0);
    }

    return get_payload(b);
}

void mempool_free(void *ptr) {
    if (!ptr || !pool || !atomic_load_explicit(&pool->is_initialized, memory_order_acquire)) {
        return;
    }

    header_t *b = get_header(ptr);
    
    if (!is_block_valid(b) || atomic_load_explicit(&b->is_free, memory_order_acquire)) {
        return;
    }

    size_t original_size = atomic_load_explicit(&b->size, memory_order_acquire);
    uintptr_t page_base = (uintptr_t)b & ~(PAGE_SIZE - 1);
    char *page_end = (char *)(page_base + PAGE_SIZE);
    char *b_end = (char *)b + original_size;
    
    header_t *merged_block = b;
    size_t merged_size = original_size;
    
    // 向后合并
    if (b_end + sizeof(header_t) <= page_end) {
        header_t *nb = (header_t *)b_end;
        
        if (atomic_load_explicit(&nb->is_free, memory_order_acquire) && 
            is_block_valid(nb) && remove_from_free_list(nb)) {
            merged_size += atomic_load_explicit(&nb->size, memory_order_acquire);
        }
    }
    
    // 向前合并
    char *block_start = (char *)align_up(page_base + sizeof(page_header_t), ALIGNMENT);
    if ((char *)merged_block > block_start) {
        footer_t *pf = (footer_t *)((char *)merged_block - sizeof(footer_t));
        
        if (atomic_load_explicit(&pf->magic, memory_order_acquire) == MAGIC) {
            size_t prev_size = atomic_load_explicit(&pf->size, memory_order_acquire);
            header_t *pb = (header_t *)((char *)merged_block - prev_size);
            
            if (atomic_load_explicit(&pb->is_free, memory_order_acquire) && 
                is_block_valid(pb) && remove_from_free_list(pb)) {
                merged_size += prev_size;
                merged_block = pb;
            }
        }
    }
    
    // 初始化合并后的块
    init_block(merged_block, merged_size, 1);
    add_to_free_list(merged_block);
    
    UPDATE_STATS(pool, -original_size, merged_size, -1);

    // 定期清理空页面 - 静态原子变量独占缓存行
    static atomic_int cleanup_counter __attribute__((aligned(CACHE_LINE_SIZE))) = 0;
    if (atomic_fetch_add_explicit(&cleanup_counter, 1, memory_order_relaxed) % CLEANUP_INTERVAL == 0) {
        free_empty_pages();
    }
}

// 统计接口
size_t mempool_get_page_size() {
    if (PAGE_SIZE == 0) init_page_size();
    return PAGE_SIZE;
}

void mempool_get_stats(mempool_stats_t *stats) {
    if (!stats || !pool || !atomic_load_explicit(&pool->is_initialized, memory_order_acquire)) {
        return;
    }
    
    stats->total_allocated = atomic_load_explicit(&pool->total_allocated, memory_order_relaxed);
    stats->total_free = atomic_load_explicit(&pool->total_free, memory_order_relaxed);
    stats->fragmentation_count = atomic_load_explicit(&pool->fragmentation_count, memory_order_relaxed);
    stats->total_pages = atomic_load_explicit(&pool->total_pages, memory_order_relaxed);
    stats->active_pages = atomic_load_explicit(&pool->active_pages, memory_order_relaxed);
    stats->hash_size = pool->hash_size;
    
    if (stats->total_allocated > 0) {
        stats->fragmentation_ratio = (double)stats->fragmentation_count / stats->total_allocated;
    } else {
        stats->fragmentation_ratio = 0.0;
    }
    
    size_t total_memory = stats->total_pages * PAGE_SIZE;
    if (total_memory > 0) {
        stats->memory_utilization = (double)(stats->total_allocated - stats->total_free) / total_memory;
    } else {
        stats->memory_utilization = 0.0;
    }
}

void mempool_print_stats() {
    if (!pool || !atomic_load_explicit(&pool->is_initialized, memory_order_acquire)) {
        printf("Memory pool not initialized\n");
        return;
    }
    
    mempool_stats_t stats;
    mempool_get_stats(&stats);
    
    printf("\n=== Memory Pool Statistics ===\n");
    printf("Total allocated: %zu bytes\n", stats.total_allocated);
    printf("Total free: %zu bytes\n", stats.total_free);
    printf("Fragmentation count: %zu\n", stats.fragmentation_count);
    printf("Fragmentation ratio: %.4f\n", stats.fragmentation_ratio);
    printf("Total pages: %zu\n", stats.total_pages);
    printf("Active pages: %zu\n", stats.active_pages);
    printf("Memory utilization: %.4f\n", stats.memory_utilization);
    printf("Hash table size: %zu\n", stats.hash_size);
    printf("================================\n");
}
