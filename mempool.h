#ifndef MEMPOOL_H
#define MEMPOOL_H

#include <stddef.h>

// 内存池统计信息结构
typedef struct {
    size_t total_allocated;      // 总分配内存
    size_t total_free;           // 总空闲内存
    size_t fragmentation_count;  // 碎片计数
    size_t total_pages;          // 总页数
    size_t active_pages;        // 活跃页数
    size_t hash_size;           // 哈希表大小
    double fragmentation_ratio; // 碎片化率
    double memory_utilization;  // 内存利用率
} mempool_stats_t;

/**
 * @brief 初始化内存池。此函数不是线程安全的。
 *
 * @param hash_table_size 哈希表的大小。如果为0，将使用默认值（128）。
 * @return 0表示成功, -1表示失败。
 */
int mempool_init(size_t hash_table_size);

/**
 * @brief 销毁内存池，释放所有关联的内存。此函数不是线程安全的。
 */
void mempool_destroy();

/**
 * @brief 从池中分配内存。此函数是线程安全的。
 *        但是，它不是可重入的。不要从信号处理程序或
 *        可能中断另一个内存池操作的上下文中调用mempool_alloc或mempool_free。
 *
 * @param size 请求的内存大小（以字节为单位）。
 * @return 成功时返回指向已分配内存的指针，失败时返回NULL。
 *         调用者必须检查返回值。
 */
void *mempool_alloc(size_t size);

/**
 * @brief 将内存释放回池中。此函数是线程安全的。
 *        与mempool_alloc一样，它不是可重入的。
 *
 * @param ptr 指向先前由mempool_alloc分配的内存块的指针。
 *            传递NULL或无效指针会被安全处理（无操作）。
 */
void mempool_free(void *ptr);

/**
 * @brief 获取系统页面大小。此函数是线程安全的。
 *
 * @return 系统页面大小（以字节为单位）。
 */
size_t mempool_get_page_size();

/**
 * @brief 获取内存池统计信息。此函数是线程安全的。
 *
 * @param stats 指向mempool_stats_t结构的指针，用于存储统计信息。
 *              如果为NULL，函数将不执行任何操作。
 */
void mempool_get_stats(mempool_stats_t *stats);

/**
 * @brief 打印内存池统计信息到标准输出。此函数是线程安全的。
 */
void mempool_print_stats();

#endif // MEMPOOL_H