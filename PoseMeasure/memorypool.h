#pragma once

#include <QMutex>
#include <QQueue>
#include <QByteArray>

// 内存块结构
struct MemoryBlock {
    QByteArray data;
    bool inUse;
    
    MemoryBlock() : inUse(false) {}
    MemoryBlock(int size) : data(size, 0), inUse(false) {}
};

class MemoryPool
{
public:
    explicit MemoryPool(int blockSize, int poolSize = 50);
    ~MemoryPool();
    
    // 获取内存块
    QByteArray* acquireBlock();
    
    // 释放内存块
    void releaseBlock(QByteArray* block);
    
    // 获取池状态
    int getAvailableCount() const;
    int getTotalCount() const;
    
    // 清理池
    void clear();

private:
    mutable QMutex m_mutex;
    QQueue<MemoryBlock*> m_availableBlocks;
    QList<MemoryBlock*> m_allBlocks;
    int m_blockSize;
    int m_poolSize;
    
    // 创建新的内存块
    MemoryBlock* createBlock();
};
