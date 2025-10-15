#include "CircularBuffer.h"

void CircularBuffer::write(const uint8_t* data, int count)
{
  if (count <= 0 || !canWrite(count)) return;
  
  for (int i = 0; i < count; i++)
  {
    m_buffer[m_head] = data[i];
    m_head = (m_head + 1) & (CIRCULAR_BUFFER_SIZE - 1);  // Use bitwise AND for power-of-2 modulo
  }
  m_size += count;
}

int CircularBuffer::read(uint8_t* data, int count) const
{
  if (count <= 0 || m_size == 0) return 0;
  
  int bytesToRead = (count < m_size) ? count : m_size;
  int tail = m_tail;
  
  for (int i = 0; i < bytesToRead; i++)
  {
    data[i] = m_buffer[tail];
    tail = (tail + 1) & (CIRCULAR_BUFFER_SIZE - 1);
  }
  
  return bytesToRead;
}

void CircularBuffer::advance(int count)
{
  if (count <= 0 || count > m_size) return;
  
  m_tail = (m_tail + count) & (CIRCULAR_BUFFER_SIZE - 1);
  m_size -= count;
}

int CircularBuffer::getContiguousData(uint8_t* data, int maxCount) const
{
  if (m_size == 0 || maxCount <= 0) return 0;
  
  int bytesToRead = (maxCount < m_size) ? maxCount : m_size;
  int tail = m_tail;
  
  // Check if we can read contiguously without wraparound
  int contiguousBytes = CIRCULAR_BUFFER_SIZE - tail;
  if (contiguousBytes > m_size) contiguousBytes = m_size;
  
  int actualBytes = (bytesToRead < contiguousBytes) ? bytesToRead : contiguousBytes;
  
  for (int i = 0; i < actualBytes; i++)
  {
    data[i] = m_buffer[tail + i];
  }
  
  return actualBytes;
}

int CircularBuffer::peek(uint8_t* data, int count) const
{
  if (count <= 0 || m_size == 0) return 0;
  
  int bytesToRead = (count < m_size) ? count : m_size;
  int tail = m_tail;
  
  for (int i = 0; i < bytesToRead; i++)
  {
    data[i] = m_buffer[tail];
    tail = (tail + 1) & (CIRCULAR_BUFFER_SIZE - 1);
  }
  
  return bytesToRead;
}
