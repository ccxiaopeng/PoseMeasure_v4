#include "memorypool.h"
#include <QDebug>

MemoryPool::MemoryPool(int blockSize, int poolSize)
    : m_blockSize(blockSize), m_poolSize(poolSize)
{
    // 预分配内存块
    for (int i = 0; i < poolSize; ++i) {
        MemoryBlock* block = createBlock();
        if (block) {
            m_allBlocks.append(block);
            m_availableBlocks.enqueue(block);
        }
    }
    
    qDebug() << "MemoryPool initialized with" << m_availableBlocks.size() 
             << "blocks of" << blockSize << "bytes each";
}

MemoryPool::~MemoryPool()
{
    clear();
}

MemoryBlock* MemoryPool::createBlock()
{
    try {
        return new MemoryBlock(m_blockSize);
    } catch (const std::bad_alloc& e) {
        qWarning() << "Failed to allocate memory block:" << e.what();
        return nullptr;
    }
}

QByteArray* MemoryPool::acquireBlock()
{
    QMutexLocker locker(&m_mutex);
    
    if (m_availableBlocks.isEmpty()) {
        // 尝试创建新的内存块（动态扩展）
        if (m_allBlocks.size() < m_poolSize * 2) { // 最大扩展到2倍初始大小
            MemoryBlock* newBlock = createBlock();
            if (newBlock) {
                m_allBlocks.append(newBlock);
                m_availableBlocks.enqueue(newBlock);
                qDebug() << "Memory pool expanded, total blocks:" << m_allBlocks.size();
            }
        }
        
        if (m_availableBlocks.isEmpty()) {
            qWarning() << "Memory pool exhausted!";
            return nullptr;
        }
    }
    
    MemoryBlock* block = m_availableBlocks.dequeue();
    block->inUse = true;
    return &(block->data);
}

void MemoryPool::releaseBlock(QByteArray* block)
{
    if (!block) return;
    
    QMutexLocker locker(&m_mutex);
    
    // 找到对应的内存块
    for (MemoryBlock* memBlock : m_allBlocks) {
        if (&(memBlock->data) == block) {
            if (memBlock->inUse) {
                memBlock->inUse = false;
                m_availableBlocks.enqueue(memBlock);
            }
            return;
        }
    }
    
    qWarning() << "Attempted to release unknown memory block";
}

int MemoryPool::getAvailableCount() const
{
    QMutexLocker locker(&m_mutex);
    return m_availableBlocks.size();
}

int MemoryPool::getTotalCount() const
{
    QMutexLocker locker(&m_mutex);
    return m_allBlocks.size();
}

void MemoryPool::clear()
{
    QMutexLocker locker(&m_mutex);
    
    qDeleteAll(m_allBlocks);
    m_allBlocks.clear();
    m_availableBlocks.clear();
}
