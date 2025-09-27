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

// 跨平台兼容的快速访问宏
#define FAST_LOAD_SIZE(h) ((size_t)atomic_load_explicit(&(h)->size, memory_order_relaxed))
#define FAST_LOAD_MAGIC(h) atomic_load_explicit(&(h)->magic, memory_order_relaxed)
#define FAST_IS_FREE(h) atomic_load_explicit(&(h)->is_free, memory_order_relaxed)
#define FAST_LOAD_NEXT(h) CAST_TO_PTR(atomic_load_explicit(&(h)->next_free, memory_order_relaxed))
#define FAST_STORE_NEXT(h, ptr) atomic_store_explicit(&(h)->next_free, CAST_FROM_PTR(ptr), memory_order_relaxed)

// 批量原子操作宏 - 减少内存屏障开销
#define ATOMIC_BATCH_START() atomic_thread_fence(memory_order_acquire)
#define ATOMIC_BATCH_END() atomic_thread_fence(memory_order_release)
#define MAGIC 0xDEADBEEF
#define MAX_RETRY_ATTEMPTS 8      // 减少重试次数，提高响应速度
#define CLEANUP_INTERVAL 128      // 更频繁的清理，减少内存碎片
#define SEARCH_EARLY_EXIT_THRESHOLD 2  // 早期退出阈值

// 编译时优化提示
#ifdef __GNUC__
#define LIKELY(x)   __builtin_expect(!!(x), 1)
#define UNLIKELY(x) __builtin_expect(!!(x), 0)
#define PREFETCH(addr, rw, locality) __builtin_prefetch(addr, rw, locality)
#define FORCE_INLINE __attribute__((always_inline)) inline
#else
#define LIKELY(x)   (x)
#define UNLIKELY(x) (x)
#define PREFETCH(addr, rw, locality)
#define FORCE_INLINE inline
#endif

// 缓存行大小定义（通常为64字节）
#define CACHE_LINE_SIZE 64

// 计算各种原子类型的缓存行填充大小
#define CACHE_LINE_PADDING(type) (CACHE_LINE_SIZE - sizeof(type))

// 确保结构体对齐到缓存行边界
#define CACHE_ALIGNED __attribute__((aligned(CACHE_LINE_SIZE)))

// 跨平台兼容的数据结构设计

// 定义固定大小的类型，确保32/64位兼容
#if UINTPTR_MAX == 0xFFFFFFFF
    // 32位系统
    #define PTR_SIZE 4
    #define SIZE_T_SIZE 4
    typedef atomic_uint32_t atomic_ptr_t;
    typedef atomic_uint32_t atomic_size_type_t;
    #define HEADER_PADDING (CACHE_LINE_SIZE - 4 - 4 - 4 - 4 - 1)
    #define FOOTER_PADDING (16 - 4 - 4)
#elif UINTPTR_MAX == 0xFFFFFFFFFFFFFFFF
    // 64位系统
    #define PTR_SIZE 8
    #define SIZE_T_SIZE 8
    typedef atomic_uint64_t atomic_ptr_t;
    typedef atomic_uint64_t atomic_size_type_t;
    #define HEADER_PADDING (CACHE_LINE_SIZE - 8 - 8 - 4 - 4 - 1)
    #define FOOTER_PADDING (16 - 8 - 4)
#else
    #error "Unsupported pointer size"
#endif

// 跨平台兼容的紧凑header结构
typedef struct header {
    // 核心数据（保证在一个缓存行内）
    atomic_size_type_t size;      // 32位：4字节，64位：8字节
    atomic_ptr_t next_free;       // 32位：4字节，64位：8字节
    atomic_uint32_t magic;        // 固定4字节
    atomic_uint32_t version;      // 固定4字节
    atomic_char is_free;          // 固定1字节
    char reserved[HEADER_PADDING]; // 动态计算填充
} __attribute__((aligned(CACHE_LINE_SIZE))) header_t;

// 跨平台兼容的紧凑footer结构
typedef struct footer {
    atomic_size_type_t size;      // 32位：4字节，64位：8字节
    atomic_uint32_t magic;        // 固定4字节
    char reserved[FOOTER_PADDING]; // 动态计算填充
} __attribute__((aligned(16))) footer_t;

// 跨平台兼容的辅助宏
#define CAST_TO_PTR(x) ((void*)(uintptr_t)(x))
#define CAST_FROM_PTR(x) ((uintptr_t)(x))

// 编译时的跨平台兼容性检查
_Static_assert(sizeof(header_t) == CACHE_LINE_SIZE, "Header size must be exactly one cache line");
_Static_assert(sizeof(footer_t) == 16, "Footer size must be 16 bytes for optimal alignment");
_Static_assert(HEADER_PADDING > 0, "Header padding must be positive to ensure proper alignment");
_Static_assert(FOOTER_PADDING >= 0, "Footer padding must be non-negative");

// 验证原子类型大小
#if UINTPTR_MAX == 0xFFFFFFFF
    _Static_assert(sizeof(atomic_ptr_t) == 4, "32-bit atomic pointer must be 4 bytes");
    _Static_assert(sizeof(atomic_size_type_t) == 4, "32-bit atomic size must be 4 bytes");
#elif UINTPTR_MAX == 0xFFFFFFFFFFFFFFFF
    _Static_assert(sizeof(atomic_ptr_t) == 8, "64-bit atomic pointer must be 8 bytes");
    _Static_assert(sizeof(atomic_size_type_t) == 8, "64-bit atomic size must be 8 bytes");
#endif

// 内存效率宏
#define MEMORY_EFFICIENT_THRESHOLD (PAGE_SIZE >> 3)  // 1/8页面大小
#define LARGE_BLOCK_THRESHOLD (PAGE_SIZE >> 1)       // 1/2页面大小

// 跨平台兼容的页面头结构
typedef struct page_header CACHE_ALIGNED {
    atomic_ptr_t next;            // 跨平台兼容的指针
    char padding1[CACHE_LINE_SIZE - PTR_SIZE];
    
    struct mem_pool *pool;        // 原生指针（不参与原子操作）
    char padding2[CACHE_LINE_SIZE - sizeof(struct mem_pool*)];
    
    atomic_size_type_t block_count;  // 跨平台兼容的大小类型
    char padding3[CACHE_LINE_SIZE - SIZE_T_SIZE];
    
    atomic_uint32_t version;      // 固定32位
    char padding4[CACHE_LINE_SIZE - 4];
} page_header_t;

// 跨平台兼容的内存池结构
typedef struct mem_pool CACHE_ALIGNED {
    atomic_uintptr_t *hash_table; // 原生指针数组
    size_t hash_size;             // 原生大小类型
    char padding1[CACHE_LINE_SIZE - sizeof(atomic_uintptr_t*) - sizeof(size_t)];
    
    atomic_ptr_t page_list;       // 跨平台兼容的页面链表
    char padding2[CACHE_LINE_SIZE - PTR_SIZE];
    
    atomic_int is_initialized;    // 固定大小
    char padding3[CACHE_LINE_SIZE - sizeof(atomic_int)];
    
    // 统计信息 - 使用跨平台兼容的类型
    atomic_size_type_t total_allocated;
    char padding4[CACHE_LINE_SIZE - SIZE_T_SIZE];
    
    atomic_size_type_t total_free;
    char padding5[CACHE_LINE_SIZE - SIZE_T_SIZE];
    
    atomic_size_type_t fragmentation_count;
    char padding6[CACHE_LINE_SIZE - SIZE_T_SIZE];
    
    atomic_size_type_t total_pages;
    char padding7[CACHE_LINE_SIZE - SIZE_T_SIZE];
    
    atomic_size_type_t active_pages;
    char padding8[CACHE_LINE_SIZE - SIZE_T_SIZE];
} mem_pool_t;

// 原子操作宏 - 直接使用标准内存序
// 跨平台兼容的指针操作宏
#define CAS_PTR(ptr, expected, desired) \
    atomic_compare_exchange_weak_explicit(ptr, expected, desired, \
        memory_order_acq_rel, memory_order_acquire)

#define LOAD_PTR(ptr) atomic_load_explicit(ptr, memory_order_acquire)
#define STORE_PTR(ptr, value) atomic_store_explicit(ptr, value, memory_order_release)

// 兼容header中next_free字段的操作
#define LOAD_NEXT_FREE(h) CAST_TO_PTR(atomic_load_explicit(&(h)->next_free, memory_order_acquire))
#define STORE_NEXT_FREE(h, ptr) atomic_store_explicit(&(h)->next_free, CAST_FROM_PTR(ptr), memory_order_release)
#define CAS_NEXT_FREE(h, expected_ptr, desired_ptr) \
    ({ uintptr_t exp = CAST_FROM_PTR(expected_ptr); \
       atomic_compare_exchange_weak_explicit(&(h)->next_free, &exp, CAST_FROM_PTR(desired_ptr), \
                                           memory_order_acq_rel, memory_order_acquire); })

// 兼容页面操作的宏
#define LOAD_PAGE_NEXT(ph) CAST_TO_PTR(atomic_load_explicit(&(ph)->next, memory_order_acquire))
#define STORE_PAGE_NEXT(ph, ptr) atomic_store_explicit(&(ph)->next, CAST_FROM_PTR(ptr), memory_order_release)
#define CAS_PAGE_NEXT(ph, expected_ptr, desired_ptr) \
    ({ uintptr_t exp = CAST_FROM_PTR(expected_ptr); \
       atomic_compare_exchange_weak_explicit(&(ph)->next, &exp, CAST_FROM_PTR(desired_ptr), \
                                           memory_order_acq_rel, memory_order_acquire); })

// 兼容池页面列表操作的宏
#define LOAD_POOL_PAGE_LIST(pool) CAST_TO_PTR(atomic_load_explicit(&(pool)->page_list, memory_order_acquire))
#define STORE_POOL_PAGE_LIST(pool, ptr) atomic_store_explicit(&(pool)->page_list, CAST_FROM_PTR(ptr), memory_order_release)
#define CAS_POOL_PAGE_LIST(pool, expected_ptr, desired_ptr) \
    ({ uintptr_t exp = CAST_FROM_PTR(expected_ptr); \
       atomic_compare_exchange_weak_explicit(&(pool)->page_list, &exp, CAST_FROM_PTR(desired_ptr), \
                                           memory_order_acq_rel, memory_order_acquire); })

// 跨平台兼容的统计更新宏
#define UPDATE_STATS(p, alloc_change, free_change, frag_change) do { \
    if ((alloc_change) != 0) atomic_fetch_add_explicit(&(p)->total_allocated, (atomic_size_type_t)(alloc_change), memory_order_relaxed); \
    if ((free_change) != 0) atomic_fetch_add_explicit(&(p)->total_free, (atomic_size_type_t)(free_change), memory_order_relaxed); \
    if ((frag_change) != 0) atomic_fetch_add_explicit(&(p)->fragmentation_count, (atomic_size_type_t)(frag_change), memory_order_relaxed); \
} while(0)

// 跨平台兼容的统计读取宏
#define GET_STAT(p, field) ((size_t)atomic_load_explicit(&(p)->field, memory_order_relaxed))

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
        return (s >> 4) & (hash_size - 1); // 右移4位忽略对齐位
    }
    return (s >> 4) % hash_size;
}

// 初始化页面大小
static int init_page_size() {
    if (PAGE_SIZE == 0) {
        long page_size = sysconf(_SC_PAGESIZE);
        PAGE_SIZE = (page_size > 0) ? (size_t)page_size : 4096;
    }
    return 0;
}

// 跨平台兼容的高效块初始化
static inline void init_block(header_t *block, size_t size, int is_free) {
    ATOMIC_BATCH_START();
    
    // 批量初始化header中的所有字段
    atomic_store_explicit(&block->size, (atomic_size_type_t)size, memory_order_relaxed);
    atomic_store_explicit(&block->magic, MAGIC, memory_order_relaxed);
    atomic_fetch_add_explicit(&block->version, 1, memory_order_relaxed);
    atomic_store_explicit(&block->next_free, 0, memory_order_relaxed);
    atomic_store_explicit(&block->is_free, is_free, memory_order_relaxed);
    
    // 初始化footer
    footer_t *footer = get_footer(block);
    atomic_store_explicit(&footer->size, (atomic_size_type_t)size, memory_order_relaxed);
    atomic_store_explicit(&footer->magic, MAGIC, memory_order_relaxed);
    
    ATOMIC_BATCH_END();
}

// 高效的快速块验证
static inline int is_block_valid(header_t *b) {
    if (!b) return 0;
    
    // 快速验证 - 使用relaxed内存序减少开销
    size_t size = FAST_LOAD_SIZE(b);
    uint32_t magic = FAST_LOAD_MAGIC(b);
    
    // 快速检查关键条件
    if (magic != MAGIC) return 0;
    if (size < MIN_BLOCK_SIZE) return 0;
    if (size & (ALIGNMENT - 1)) return 0; // 使用位运算检查对齐
    
    // footer验证（只在必要时执行）
    footer_t *f = get_footer(b);
    if (!f) return 0;
    
    return (atomic_load_explicit(&f->size, memory_order_relaxed) == size && 
            atomic_load_explicit(&f->magic, memory_order_relaxed) == MAGIC);
}

// 新增：轻量级块验证（仅检查header）
static inline int is_block_valid_fast(header_t *b) {
    return b && FAST_LOAD_MAGIC(b) == MAGIC && FAST_LOAD_SIZE(b) >= MIN_BLOCK_SIZE;
}

// 优化的空闲链表操作
static int remove_from_free_list(header_t *b) {
    if (!b || !atomic_load_explicit(&b->is_free, memory_order_acquire)) return 0;
    
    size_t i = hash_function(atomic_load_explicit(&b->size, memory_order_acquire), pool->hash_size);
    atomic_uintptr_t *bucket = &pool->hash_table[i];
    
    uint32_t version = atomic_load_explicit(&b->version, memory_order_acquire);
    
    for (int retry = 0; retry < MAX_RETRY_ATTEMPTS; retry++) {
        // 检查是否为头节点
        uintptr_t current_head = LOAD_PTR(bucket);
        if (current_head == (uintptr_t)b) {
            uintptr_t expected = current_head;
            uintptr_t desired = CAST_FROM_PTR(LOAD_NEXT_FREE(b));
            
            if (CAS_PTR(bucket, &expected, desired)) {
                goto success;
            }
            continue; // 重试
        }
        
        // 遍历链表查找并移除
        header_t *prev = NULL;
        uintptr_t current = current_head;
        
        while (current != 0) {
            header_t *cur_block = (header_t *)current;
            
            if (cur_block == b) {
                uint32_t current_version = atomic_load_explicit(&cur_block->version, memory_order_acquire);
                if (current_version != version) {
                    version = atomic_load_explicit(&b->version, memory_order_acquire);
                    break;
                }
                
                if (prev) {
                    // 使用CAS操作更新前一个节点的next指针
                    header_t *prev_next = LOAD_NEXT_FREE(prev);
                    header_t *desired_next = LOAD_NEXT_FREE(b);
                    if (prev_next == b && CAS_NEXT_FREE(prev, b, desired_next)) {
                        goto success;
                    }
                } else {
                    // 这种情况不应该发生，因为我们已经检查了头节点
                    break;
                }
                break;
            }
            prev = cur_block;
            current = CAST_FROM_PTR(LOAD_NEXT_FREE(cur_block));
        }
    }
    
    return 0;

success:
    header_t *next_block = LOAD_NEXT_FREE(b);
    if (next_block != NULL) {
        // 注意：在新的设计中，我们只使用next_free，不使用prev_free
        // 这简化了链表操作并减少了内存占用
    }
    
    STORE_NEXT_FREE(b, NULL);
    atomic_store_explicit(&b->is_free, 0, memory_order_release);
    atomic_fetch_add_explicit(&b->version, 1, memory_order_release);
    
    return 1;
}

static int add_to_free_list(header_t *b) {
    if (!b || FAST_IS_FREE(b)) return 0;
    
    size_t i = hash_function(FAST_LOAD_SIZE(b), pool->hash_size);
    atomic_uintptr_t *bucket = &pool->hash_table[i];
    
    // 预先设置状态，减少后续操作
    atomic_store_explicit(&b->is_free, 1, memory_order_relaxed);
    atomic_fetch_add_explicit(&b->version, 1, memory_order_relaxed);
    
    // 优化的CAS循环 - 减少内存屏障
    for (int retry = 0; retry < MAX_RETRY_ATTEMPTS; retry++) {
        uintptr_t expected = atomic_load_explicit(bucket, memory_order_relaxed);
        STORE_NEXT_FREE(b, CAST_TO_PTR(expected));
        
        if (atomic_compare_exchange_weak_explicit(bucket, &expected, (uintptr_t)b,
                                                   memory_order_release, memory_order_relaxed)) {
            // 成功添加到链表头部
            return 1;
        }
    }
    
    // CAS失败，恢复状态
    atomic_store_explicit(&b->is_free, 0, memory_order_relaxed);
    return 0;
}

// 页管理函数
// 安全地从页面列表中移除页面
static int remove_page_from_list(page_header_t *page) {
    uintptr_t current = LOAD_PTR(&pool->page_list);
    uintptr_t prev = 0;
    
    // 遍历页面列表找到目标页面
    while (current != 0) {
        if (current == (uintptr_t)page) {
            uintptr_t next = LOAD_PTR(&page->next);
            
            // 原子地移除页面
            if (prev == 0) {
                // 移除头节点
                return CAS_PTR(&pool->page_list, &current, next);
            } else {
                // 移除中间节点
                page_header_t *prev_page = (page_header_t *)prev;
                return CAS_PTR(&prev_page->next, &current, next);
            }
        }
        prev = current;
        current = LOAD_PTR(&((page_header_t *)current)->next);
    }
    
    return 0; // 页面未找到
}

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
        
        // 双重检查页面是否为空，防止竞态条件
        if (is_page_empty(page)) {
            uint32_t page_version = atomic_load_explicit(&page->version, memory_order_acquire);
            
            // 再次检查页面状态，确保没有其他线程修改
            if (is_page_empty(page) && 
                atomic_load_explicit(&page->version, memory_order_acquire) == page_version) {
                
                uintptr_t page_start = (uintptr_t)page;
                uintptr_t aligned_start = align_up(page_start + sizeof(page_header_t), ALIGNMENT);
                header_t *first_block = (header_t *)aligned_start;
                
                // 尝试移除页面中的唯一块
                if (remove_from_free_list(first_block)) {
                    // 原子地从页面列表中移除页面
                    uintptr_t expected_current = current;
                    uintptr_t desired_next = next;
                    
                    int removed = 0;
                    if (prev == 0) {
                        // 移除头节点
                        if (CAS_PTR(&pool->page_list, &expected_current, desired_next)) {
                            removed = 1;
                        }
                    } else {
                        // 移除中间节点
                        page_header_t *prev_page = (page_header_t *)prev;
                        if (CAS_PTR(&prev_page->next, &expected_current, desired_next)) {
                            removed = 1;
                        }
                    }
                    
                    if (removed) {
                        munmap(page, PAGE_SIZE);
                        atomic_fetch_sub_explicit(&pool->active_pages, (atomic_size_type_t)1, memory_order_relaxed);
                        current = next;
                        continue;
                    } else {
                        // CAS 失败，将块重新添加到空闲列表
                        add_to_free_list(first_block);
                    }
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
    ph->pool = pool;
    atomic_store_explicit(&ph->block_count, 1, memory_order_release);
    atomic_store_explicit(&ph->version, 0, memory_order_release);
    // next指针将在CAS操作中设置
    
    // 原子地将页面添加到页面列表
    uintptr_t expected = LOAD_PTR(&pool->page_list);
    uintptr_t desired = (uintptr_t)ph;
    
    // 修复竞态条件：在CAS操作前设置正确的next指针
    STORE_PTR(&ph->next, expected);
    
    if (!CAS_PTR(&pool->page_list, &expected, desired)) {
        // CAS失败，需要重新设置next指针并重试
        for (int retry = 0; retry < MAX_RETRY_ATTEMPTS; retry++) {
            expected = LOAD_PTR(&pool->page_list);
            STORE_PTR(&ph->next, expected);
            if (CAS_PTR(&pool->page_list, &expected, desired)) {
                goto page_added_successfully;
            }
        }
        // 重试失败，清理并返回
        munmap(page, PAGE_SIZE);
        return NULL;
    }
    
page_added_successfully:

    uintptr_t page_start = (uintptr_t)ph;
    uintptr_t aligned_start = align_up(page_start + sizeof(page_header_t), ALIGNMENT);
    
    header_t *b = (header_t *)aligned_start;
    size_t block_size = align_up(PAGE_SIZE - (aligned_start - page_start), ALIGNMENT);
    
    init_block(b, block_size, 1);
    
    // 尝试将块添加到空闲列表
    if (!add_to_free_list(b)) {
        // 使用辅助函数安全地移除页面
        remove_page_from_list(ph);
        munmap(page, PAGE_SIZE);
        return NULL;
    }
    
    // 更新统计信息
    UPDATE_STATS(pool, 0, block_size, 0);
    atomic_fetch_add_explicit(&pool->total_pages, (atomic_size_type_t)1, memory_order_relaxed);
    atomic_fetch_add_explicit(&pool->active_pages, (atomic_size_type_t)1, memory_order_relaxed);
    
    return b;
}

// 高度优化的智能查找算法
static header_t *find_best_fit_block(size_t s) {
    header_t *best = NULL;
    size_t min_diff = SIZE_MAX;
    const size_t search_limit = pool->hash_size >> 2; // hash_size / 4
    
    size_t start_idx = hash_function(s, pool->hash_size);
    
    // 阶段一：精确匹配搜索 - 优先搜索目标大小及相近大小
    for (size_t offset = 0; offset < 3 && offset < pool->hash_size; offset++) {
        size_t idx = (start_idx + offset) % pool->hash_size;
        uintptr_t current = LOAD_PTR(&pool->hash_table[idx]);
        
        while (current != 0) {
            header_t *cur = (header_t *)current;
            
            // 预取下一个节点，提高缓存命中率
            header_t *next = LOAD_NEXT_FREE(cur);
            if (next != NULL) {
                PREFETCH((void*)next, 0, 3); // 预取到L3缓存
            }
            
            if (FAST_IS_FREE(cur)) {
                size_t cur_size = FAST_LOAD_SIZE(cur);
                if (cur_size >= s) {
                    size_t diff = cur_size - s;
                    if (diff == 0) return cur; // 精确匹配，立即返回
                    if (diff < min_diff) {
                        min_diff = diff;
                        best = cur;
                        // 如果差异很小，直接返回
                        if (diff <= ALIGNMENT) return cur;
                    }
                }
            }
            current = CAST_FROM_PTR(next);
        }
        
        // 如果已经找到了相当好的匹配，停止搜索
        if (best && min_diff <= s >> 3) break; // 小于s/8
    }
    
    // 阶段二：如果没有找到好的匹配，扩大搜索范围
    if (!best || min_diff > s >> 2) { // 大于s/4
        for (size_t offset = 1; offset < search_limit; offset++) {
            size_t left_idx = (start_idx - offset + pool->hash_size) % pool->hash_size;
            size_t right_idx = (start_idx + offset) % pool->hash_size;
            
            // 交替搜索左右桶，提高命中率
            for (int side = 0; side < 2; side++) {
                size_t idx = (side == 0) ? right_idx : left_idx;
                uintptr_t current = LOAD_PTR(&pool->hash_table[idx]);
                
                while (current != 0) {
                    header_t *cur = (header_t *)current;
                    
                    if (FAST_IS_FREE(cur)) {
                        size_t cur_size = FAST_LOAD_SIZE(cur);
                        if (cur_size >= s) {
                            size_t diff = cur_size - s;
                            if (diff == 0) return cur;
                            if (diff < min_diff) {
                                min_diff = diff;
                                best = cur;
                            }
                        }
                    }
                    current = CAST_FROM_PTR(LOAD_NEXT_FREE(cur));
                }
                
                // 如果已经找到了很好的匹配，提前退出
                if (best && min_diff <= ALIGNMENT) return best;
            }
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

    // 保存必要的信息，因为pool指针即将失效
    atomic_uintptr_t *hash_table = pool->hash_table;
    size_t hash_size = pool->hash_size;
    
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

    // 先释放哈希表
    if (hash_table) {
        munmap(hash_table, sizeof(atomic_uintptr_t) * hash_size);
    }
    
    // 最后释放pool结构体并设置为NULL
    munmap(pool, sizeof(mem_pool_t));
    pool = NULL;
    
    atomic_store_explicit(&init_flag, 0, memory_order_release);
}

void *mempool_alloc(size_t size) {
    // 快速参数验证
    if (UNLIKELY(!size || !pool)) return NULL;
    if (UNLIKELY(!atomic_load_explicit(&pool->is_initialized, memory_order_acquire))) return NULL;

    // 优化的大小计算
    size_t req = (size + sizeof(header_t) + sizeof(footer_t) + ALIGNMENT - 1) & ~(ALIGNMENT - 1);
    req = req < MIN_BLOCK_SIZE ? MIN_BLOCK_SIZE : req;
    
    header_t *b = NULL;
    
    // 快速路径：直接查找一次
    b = find_best_fit_block(req);
    
    // 慢速路径：需要分配新页面
    if (UNLIKELY(!b)) {
        if (!allocate_page()) return NULL;
        b = find_best_fit_block(req);
        if (!b) return NULL;
    }

    // 优化的移除操作 - 使用指数退避
    int retry_delay = 1;
    for (int retry = 0; retry < MAX_RETRY_ATTEMPTS; retry++) {
        if (remove_from_free_list(b)) goto block_acquired;
        
        // 指数退避策略
        for (int i = 0; i < retry_delay; i++) {
            __builtin_ia32_pause(); // CPU暂停指令
        }
        retry_delay <<= 1; // 双倍递增
        
        // 重新查找
        b = find_best_fit_block(req);
        if (!b) return NULL;
    }
    return NULL; // 所有重试失败
    
block_acquired:
    size_t original_size = FAST_LOAD_SIZE(b);
    
    // 高级智能块拆分策略
    size_t remainder = original_size - req;
    
    // 对于大块，使用更激进的拆分策略
    size_t split_threshold = (req > LARGE_BLOCK_THRESHOLD) ? MIN_BLOCK_SIZE : MIN_BLOCK_SIZE + ALIGNMENT;
    
    if (UNLIKELY(remainder >= split_threshold)) {
        // 需要拆分
        header_t *nb = (header_t *)((char *)b + req);
        
        ATOMIC_BATCH_START();
        init_block(nb, remainder, 1);
        init_block(b, req, 0);
        ATOMIC_BATCH_END();
        
        // 尝试添加新块到空闲列表
        if (add_to_free_list(nb)) {
            UPDATE_STATS(pool, req, -req, 1);
        } else {
            // 添加失败，合并回原块
            init_block(b, original_size, 0);
            UPDATE_STATS(pool, original_size, -original_size, 0);
        }
    } else {
        // 不需要拆分 - 减少碎片化
        init_block(b, original_size, 0);
        UPDATE_STATS(pool, original_size, -original_size, 0);
    }

    return get_payload(b);
}

void mempool_free(void *ptr) {
    // 快速参数验证
    if (UNLIKELY(!ptr || !pool)) return;
    if (UNLIKELY(!atomic_load_explicit(&pool->is_initialized, memory_order_acquire))) return;

    header_t *b = get_header(ptr);
    
    // 快速验证
    if (UNLIKELY(!is_block_valid_fast(b) || FAST_IS_FREE(b))) return;

    size_t original_size = FAST_LOAD_SIZE(b);
    uintptr_t page_base = (uintptr_t)b & ~(PAGE_SIZE - 1);
    char *page_end = (char *)(page_base + PAGE_SIZE);
    char *b_end = (char *)b + original_size;
    
    header_t *merged_block = b;
    size_t merged_size = original_size;
    size_t freed_blocks = 0;
    
    // 优化的向后合并
    if (LIKELY(b_end + sizeof(header_t) <= page_end)) {
        header_t *nb = (header_t *)b_end;
        
        if (FAST_IS_FREE(nb) && is_block_valid_fast(nb)) {
            size_t nb_size = FAST_LOAD_SIZE(nb);
            if (remove_from_free_list(nb)) {
                merged_size += nb_size;
                freed_blocks++;
            }
        }
    }
    
    // 优化的向前合并
    char *block_start = (char *)align_up(page_base + sizeof(page_header_t), ALIGNMENT);
    if (LIKELY((char *)merged_block > block_start)) {
        footer_t *pf = (footer_t *)((char *)merged_block - sizeof(footer_t));
        
        if (atomic_load_explicit(&pf->magic, memory_order_relaxed) == MAGIC) {
            size_t prev_size = atomic_load_explicit(&pf->size, memory_order_relaxed);
            header_t *pb = (header_t *)((char *)merged_block - prev_size);
            
            if (FAST_IS_FREE(pb) && is_block_valid_fast(pb)) {
                if (remove_from_free_list(pb)) {
                    merged_size += prev_size;
                    merged_block = pb;
                    freed_blocks++;
                }
            }
        }
    }
    
    // 批量初始化和添加到空闲列表
    ATOMIC_BATCH_START();
    init_block(merged_block, merged_size, 1);
    ATOMIC_BATCH_END();
    
    add_to_free_list(merged_block);
    
    // 批量更新统计信息
    UPDATE_STATS(pool, -original_size, merged_size, -freed_blocks);

    // 优化的清理计数器 - 使用更快的局部静态变量
    static _Thread_local int cleanup_counter = 0;
    if (UNLIKELY(++cleanup_counter >= CLEANUP_INTERVAL)) {
        cleanup_counter = 0;
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
    
    stats->total_allocated = GET_STAT(pool, total_allocated);
    stats->total_free = GET_STAT(pool, total_free);
    stats->fragmentation_count = GET_STAT(pool, fragmentation_count);
    stats->total_pages = GET_STAT(pool, total_pages);
    stats->active_pages = GET_STAT(pool, active_pages);
    stats->hash_size = pool->hash_size;
    
    if (stats->total_allocated > 0) {
        stats->fragmentation_ratio = (double)stats->fragmentation_count / stats->total_allocated;
    } else {
        stats->fragmentation_ratio = 0.0;
    }
    
    size_t total_memory = stats->total_pages * PAGE_SIZE;
    if (total_memory > 0) {
        // 修复内存利用率计算：已分配的内存占总内存的比例
        stats->memory_utilization = (double)stats->total_allocated / total_memory;
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
    printf("\n=== Cross-Platform Info ===\n");
    printf("Pointer size: %zu bytes\n", sizeof(void*));
    printf("Header size: %zu bytes\n", sizeof(header_t));
    printf("Footer size: %zu bytes\n", sizeof(footer_t));
    printf("Cache line size: %d bytes\n", CACHE_LINE_SIZE);
    printf("Atomic ptr size: %zu bytes\n", sizeof(atomic_ptr_t));
    printf("Atomic size type: %zu bytes\n", sizeof(atomic_size_type_t));
    printf("================================\n");
}
