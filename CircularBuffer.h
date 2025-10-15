#pragma once
#include <cstdint>

constexpr int CIRCULAR_BUFFER_SIZE = 1024;  // Power of 2 for efficient modulo operations

/**
 * Circular buffer for receiving serial data with efficient wraparound handling
 */
class CircularBuffer
{
public:
  CircularBuffer() : m_head(0), m_tail(0), m_size(0) {}
  
  // Add data to the buffer
  void write(const uint8_t* data, int count);
  
  // Read data from the buffer (non-destructive)
  int read(uint8_t* data, int count) const;
  
  // Advance the tail pointer (mark data as consumed)
  void advance(int count);
  
  // Get the number of bytes available for reading
  int available() const { return m_size; }
  
  // Check if buffer has enough space for writing
  bool canWrite(int count) const { return (CIRCULAR_BUFFER_SIZE - m_size) >= count; }
  
  // Clear the buffer
  void clear() { m_head = m_tail = m_size = 0; }
  
  // Get a contiguous block of data starting from tail
  int getContiguousData(uint8_t* data, int maxCount) const;
  
  // Peek at data without advancing the tail pointer
  int peek(uint8_t* data, int count) const;
  
private:
  uint8_t m_buffer[CIRCULAR_BUFFER_SIZE];
  int m_head;    // Write position
  int m_tail;    // Read position
  int m_size;    // Number of bytes currently in buffer
};
