#pragma once
#include <cstdint>
#include <vector>
#include "TesterType.h"

class Packet
{
public:
  Packet() {};
  explicit Packet(TesterType testerType);

  // Parse from raw buffer into fields. Returns false if incomplete/invalid
  bool parse(const uint8_t* buf, int bufLen);

  // Serialize fields into raw buffer for transmission
  std::vector<uint8_t> serialize() const;
  uint16_t totalLength();

  // Accessors
  const std::vector<uint8_t>& header() const { return m_header; }
  uint8_t command() const { return m_command; }
  const std::vector<uint8_t>& data() const { return m_data; }
  uint8_t module() const { return m_module; }

  // Mutators
  void setCommand(uint8_t cmd) { m_command = cmd; }
  void setData(const std::vector<uint8_t>& data) { m_data = data; }
  void setReplyData(const std::vector<uint8_t>& data);
  void setModule(uint8_t module) { m_module = module; }
  void setReply(bool success, const std::vector<uint8_t>&data);

private:
  // Protocol helpers
  void configureHeader();
  void configureReplyHeader();
  uint8_t computeChecksum(const std::vector<uint8_t>& bytes) const;
  bool isXBoard() const;
  bool isSD2() const;
  int commandIndex() const;
  int moduleIndex() const;

private:
  TesterType m_testerType;
  std::vector<uint8_t> m_header; // "P" or "WAY" (and reply variants as needed externally)
  uint8_t m_command = 0;
  uint8_t m_module = 0;
  std::vector<uint8_t> m_data;    // payload between command and checksum
};


