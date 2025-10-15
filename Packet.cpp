#include "Packet.h"

Packet::Packet(TesterType testerType)
  : m_testerType(testerType)
{
  configureHeader();
}

void Packet::configureHeader()
{
  m_header.clear();
  if (m_testerType == TesterType::xBOARD)
  {
    m_header = {'W','A','Y'};
  }
  else
  {
    m_header = {'P'};
  }
}

void Packet::configureReplyHeader()
{
  m_header.clear();
  if (m_testerType == TesterType::xBOARD)
  {
    m_header = {'D','E','S'};
  }
  else
  {
    m_header = {'T'};
  }
}

void Packet::setReply(bool success, const std::vector<uint8_t> &data)
{
  configureReplyHeader();
  uint8_t successVal = isSD2() ? 0x01 : 0x00;
  if (!success) {
    successVal ^= 1;
  }
  std::vector<uint8_t> replyData = { successVal };
  replyData.insert(replyData.end(), data.begin(), data.end());
  m_data = replyData;
}

void Packet::setReplyData(const std::vector<uint8_t> &data)
{
  configureReplyHeader();
  m_data = data;
}

bool Packet::isXBoard() const { return m_testerType == TesterType::xBOARD; }
bool Packet::isSD2() const { return m_testerType == TesterType::SD2; }

bool Packet::parse(const uint8_t* buf, int bufLen)
{
  if (buf == nullptr || bufLen <= 0) return false;

  // Validate header
  const int headerLen = static_cast<int>(m_header.size());
  if (bufLen < headerLen + 2) return false;
  for (int i = 0; i < headerLen; ++i)
  {
    if (buf[i] != m_header[i]) return false;
  }

  // Extract total length per protocol
  int totalLength = static_cast<uint16_t>((buf[headerLen] << 8) | buf[headerLen + 1]);
  if (isSD2()) totalLength = static_cast<uint16_t>(totalLength + 1); // SD2 length excludes prefix

  if (bufLen < static_cast<int>(totalLength)) return false;

  // Parse command position and payload window
  if (commandIndex() >= totalLength) return false;
  if (moduleIndex() >= totalLength) return false;
  m_command = buf[commandIndex()];
  m_module = buf[moduleIndex()];

  // Data runs from byte after command up to final checksum (excluded for xBOARD, included for SD2)
  const int dataStart = commandIndex() + 1;
  int dataLen = 0;
  if (isXBoard()) {
    // Exclude last byte (checksum)
    const int checksumIndex = static_cast<int>(totalLength) - 1;
    dataLen = checksumIndex - dataStart;
  } else {
    // SD2: all remaining bytes after command are data (no checksum at end)
    dataLen = static_cast<int>(totalLength) - dataStart;
  }
  if (dataLen < 0) return false;
  m_data.assign(&buf[dataStart], &buf[dataStart + dataLen]);

  return true;
}

uint8_t Packet::computeChecksum(const std::vector<uint8_t>& bytes) const
{
  if (bytes.empty()) return 0;
  uint8_t acc = bytes[0];
  for (size_t i = 1; i < bytes.size(); ++i)
  {
    acc ^= bytes[i];
  }
  return acc;
}

int Packet::commandIndex() const { return isXBoard() ? 7 : 6; }
int Packet::moduleIndex() const { return isXBoard() ? 6 : 5; }
uint16_t Packet::totalLength()  { return m_header.size() + m_data.size() + 5 + 1; }

std::vector<uint8_t> Packet::serialize() const
{
  std::vector<uint8_t> bytes;
  const int headerLen = m_header.size();
  const int totalLen = headerLen + 5 + 1 + m_data.size();
  bytes.reserve(totalLen);

  // Header
  bytes.insert(bytes.end(), m_header.begin(), m_header.end());

  // Length value: SD2 uses total-1, xBOARD uses total
  const uint16_t field = isSD2() ? totalLen - 1 : totalLen;
  bytes.push_back(static_cast<uint8_t>((field >> 8) & 0xFF));
  bytes.push_back(static_cast<uint8_t>(field & 0xFF));

  bytes.push_back(0); // sync

  if (isSD2())
  {
    bytes.push_back(1); // cPC
  }

  bytes.push_back(m_module);
  bytes.push_back(m_command);

  bytes.insert(bytes.end(), m_data.begin(), m_data.end());

  if (isXBoard())
  {
    // Append checksum
    const uint8_t checksum = computeChecksum(bytes);
    bytes.push_back(checksum);
  }

  return bytes;
}
