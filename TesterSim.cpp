#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <chrono>
#include <thread>
#include "TesterSim.h"
#include "utilities.h"
#include <QFile>
#include <QDataStream>
#include <QFileInfo>

std::map<uint8_t,std::function<void(const Packet&,Packet&,TesterSim*)>> TesterSim::s_commandProcs =
{
  { 0x00, TesterSim::process63TesterStatus },
  { 0x01, TesterSim::process01TabletInfo },
  { 0x02, TesterSim::process02SerialNo },
  { 0x09, TesterSim::process09 },
  { 0x0A, TesterSim::process0AWorkshopData },
  { 0x0B, TesterSim::process0BStartApplModGest },
  { 0x11, TesterSim::process11DoSlowInit },
  { 0x12, TesterSim::process12GetISOKeyword },
  { 0x13, TesterSim::process13CommandToECU },
  { 0x15, TesterSim::process15DisplayString },
  { 0x1C, TesterSim::process1C },
  { 0x1E, TesterSim::process1ECloseFile },
    // 0x1F - 31, setBoard  // SDX?
  { 0x20, TesterSim::process20OpenFileForWriting },
  { 0x21, TesterSim::process21WriteToFile },
  { 0x23, TesterSim::process23OpenFileForReading },
  { 0x24, TesterSim::process24ReadFromFile },
  { 0x25, TesterSim::process25ChecksumFile },
  { 0x2A, TesterSim::process2AChdir },
  { 0x2B, TesterSim::process2BGetNextDirEntry },
  { 0x3A, TesterSim::process3AGetDateTime },
    // 0x3B - 59, SetDateTimeBoard
  { 0x3D, TesterSim::process3DEraseFlash },
  { 0x60, TesterSim::process60SiliconNumber },
  { 0x61, TesterSim::process61SetBoard }, // SD2?
  { 0x62, TesterSim::process62SendReset },
  { 0x63, TesterSim::process63TesterStatus },
};


TesterSim::TesterSim(QObject* parent) : QObject(parent)
{
  memset(m_inbuf, 0, 128);
  memset(m_outbuf, 0, 128);
  memset(m_checksumBuf, 0, CHKSUM_BUF_SIZE);
  memset(m_lastInbuf, 0, 128);
  m_receiveBuffer.clear();
  for (int i = 0; i < 16; i++)
  {
    m_applRun[i] = false;
  }
}

void TesterSim::setRAMLoc(uint16_t addr, uint8_t val)
{
  m_ramData[addr] = val;
}

void TesterSim::setValue(uint16_t id, uint32_t val)
{
  m_valueData[id] = val;
}

int TesterSim::readBytes(uint8_t* buf, int count)
{
  while(m_port.bytesAvailable() < count)
  {
      // wait
      if (m_shutdown)
      {
          return 0;
      }
  }

  return m_port.read(reinterpret_cast<char*>(buf), count);
}

bool TesterSim::fillReceiveBuffer()
{
  if (m_shutdown) return false;
  
  // Read available data from serial port
  int bytesAvailable = m_port.bytesAvailable();
  if (bytesAvailable <= 0) return true;  // No data available, but not an error
  
  // Limit read size to prevent buffer overflow
  int maxReadSize = CIRCULAR_BUFFER_SIZE - m_receiveBuffer.available();
  if (maxReadSize <= 0) return true;  // Buffer is full
  
  int readSize = (bytesAvailable < maxReadSize) ? bytesAvailable : maxReadSize;
  
  uint8_t tempBuffer[512];  // Temporary buffer for reading from serial port
  int bytesRead = m_port.read(reinterpret_cast<char*>(tempBuffer), readSize);
  
  if (bytesRead > 0)
  {
    m_receiveBuffer.write(tempBuffer, bytesRead);
  }
  
  return true;
}

bool TesterSim::findCompletePacket(uint8_t* packetBuf, int& packetSize)
{
  // If there is nothing to inspect, bail early
  int available = m_receiveBuffer.available();
  if (available <= 0) return false;

  // Determine header by tester type; treat as string to obtain length generically
  const bool isXBoard = (m_testerType == TesterType::xBOARD);
  const char* headerStr = isXBoard ? "WAY" : "P";
  const int headerLen = static_cast<int>(strlen(headerStr));

  // Peek as much as available (bounded by the circular buffer capacity)
  uint8_t temp[CIRCULAR_BUFFER_SIZE];
  const int toPeek = (available < CIRCULAR_BUFFER_SIZE) ? available : CIRCULAR_BUFFER_SIZE;
  const int peeked = m_receiveBuffer.peek(temp, toPeek);
  if (peeked <= 0) return false;

  // Scan for a valid start byte and header
  int maxAdvance = 0; // how many bytes we can safely discard (shift) if no full packet is found
  for (int i = 0; i < peeked; ++i)
  {
    // Ensure we have enough to check the full header
    if (i + headerLen > peeked)
    {
      // keep possible partial header; drop bytes before it
      if (i > 0) m_receiveBuffer.advance(i);
      return false;
    }

    // Check header match
    if (memcmp(&temp[i], headerStr, headerLen) != 0)
    {
      maxAdvance = i + 1;
      continue;
    }

    // Found a potential start. Ensure we have at least the header and the 2-byte length
    const int lengthIndex = i + headerLen;
    if (lengthIndex + 2 > peeked)
    {
      // Keep the potential prefix for next time; drop bytes before it
      if (i > 0) m_receiveBuffer.advance(i);
      return false;
    }

    const uint8_t lengthHi = temp[lengthIndex + 0];
    const uint8_t lengthLo = temp[lengthIndex + 1];

    // Interpret length according to protocol
    const uint16_t length = (static_cast<uint16_t>(lengthHi) << 8) | lengthLo;
    if (!isXBoard)
    {
      // SD2: packetSize = prefix(1) + length
      packetSize = static_cast<int>(length) + 1;
    }
    else
    {
      // xBOARD: length includes the entire message (header and checksum)
      packetSize = static_cast<int>(length);
    }

    // Sanity check minimum size
    const int minPacketSize = isXBoard ? 9 : 7; // xBOARD includes 3-byte header and checksum
    if (packetSize < minPacketSize)
    {
      // Invalid size at this prefix; skip this prefix and continue searching
      emit logMsg(QString("Invalid packet size %1 at prefix; shifting to next candidate").arg(packetSize));
      maxAdvance = i + 1;
      continue;
    }

    // If we don't yet have the full packet in the buffer, keep the prefix and wait for more
    if (i + packetSize > peeked)
    {
      if (i > 0) m_receiveBuffer.advance(i);
      return false;
    }

    // We have a complete packet starting at offset i. Shift buffer up to i, then extract.
    if (i > 0) m_receiveBuffer.advance(i);
    return extractPacketFromBuffer(packetBuf, packetSize);
  }

  // No valid prefix found in the peeked data. Discard scanned bytes to reframe.
  if (maxAdvance > 0)
  {
    emit logMsg(QString("Invalid data encountered - shifting buffer by %1 bytes to reframe").arg(maxAdvance));
    m_receiveBuffer.advance(maxAdvance);
  }
  return false;
}

bool TesterSim::extractPacketFromBuffer(uint8_t* packetBuf, int packetSize)
{
  if (m_receiveBuffer.available() < packetSize) return false;
  
  // Read the complete packet
  int bytesRead = m_receiveBuffer.read(packetBuf, packetSize);
  if (bytesRead != packetSize) return false;
  
  // Advance the buffer to mark this packet as consumed
  m_receiveBuffer.advance(packetSize);
  
  return true;
}

bool TesterSim::sendReply(Packet packet, bool print)
{
  bool status = true;
  if (packet.totalLength() != 0)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    uint16_t len = packet.totalLength();

    if (print)
    {
      printPacket(packet);
    }

    const int wroteBytes = m_port.write(reinterpret_cast<char*>(packet.serialize().data()), len);
    m_port.waitForBytesWritten();
    status = (wroteBytes == len);
  }
  return status;
}

bool TesterSim::processBuf(Packet packet, bool print)
{
  bool status = false;
  uint16_t size = packet.totalLength();

  if (size >= 7) // TODO
  {
    memcpy(m_outbuf, m_inbuf, size); // tablet SW usually starts by copying the message
                                     // from the PC into the reply buffer
    Packet outPacket(packet);

    // For xBOARD, preserve the copied header/length; response shaping handled per-command

    // To keep the log output cleaner, we keep track of whether we
    // received multiple consecutive Write-to-File commands.
    if (packet.command() != 0x21)
    {
      m_lastCmdWasWriteToFile = false;
    }

    if (s_commandProcs.count(packet.command()))
    {
      s_commandProcs.at(packet.command())(packet, outPacket, this);
    }
    else
    {
      emit logMsg(QString("Sending generic 'success' response to command msg type 0x%1").arg(packet.command(), 2, 16, QChar('0')));
      outPacket.setReply(true, {});
    }
    status = sendReply(outPacket, print);

    // TODO: Of the ECUs that send unsolicited info immediately after the ISO
    // keyword sequence, we need to determine which of them have their ID info
    // concatenated to the cmd 0x11 (or 0x12) response payload, and which have
    // their info sent in separate messages after the 0x11/0x12 response.
    // In the case of separate messages, the message type is probably supposed
    // to be set to 0x13.
  }
  else
  {
    emit logMsg("Warning: received message smaller than minimum size for active protocol.\n");
  }
  return status;
}

bool TesterSim::connectToSocket(const QString &sockPath)
{
  m_port.setPortName(sockPath);
  //m_port.setBaudRate(38400);  //SD2
  m_port.setBaudRate(115200);   //SDX
  return m_port.open(QIODevice::ReadWrite);
}

void TesterSim::stopListening()
{
  m_shutdown = true;
  m_port.close();
}

/**
 * Determines whether the packet containing the supplied buffer should be
 * printed in its entirety.
 */
bool TesterSim::shouldDisplayPacket(Packet packet)
{
  bool status = true;
  const uint16_t totalLen = packet.totalLength();
  auto buf = packet.serialize();
  if (totalLen > 0 && memcmp(buf.data(), m_lastInbuf, totalLen) == 0)
  {
    emit lastLogMsgRepeated();
    status = false;
  }
  if (totalLen > 0) memcpy(m_lastInbuf, buf.data(), totalLen);
  return status;
}

void TesterSim::printPacket(Packet packet)
{
  QString packetStr;
  auto bytes = packet.serialize();
  for (int i = 0; i < packet.totalLength(); i++)
  {
    packetStr += QString("%1 ").arg(bytes[i], 2, 16, QChar('0'));
  }
  log(packetStr);
}

void TesterSim::emitConsecutiveWriteToFileSignal()
{
  emit consecutiveWriteToFileCmd();
}

bool TesterSim::listen()
{
  bool status = true;
  int packetSize = 0;

  while (status && !m_shutdown)
  {
    // Fill the receive buffer with available serial data
    if (!fillReceiveBuffer())
    {
      status = false;
      break;
    }
    
    // Try to find complete packets in the buffer
    while (findCompletePacket(m_inbuf, packetSize))
    {
      Packet packet(m_testerType);
      packet.parse(m_inbuf, packetSize);
      // Validate packet size
      if (packetSize >= 7)
      {
        if (shouldDisplayPacket(packet))
        {
          printPacket(packet);
          status = processBuf(packet, true);
        }
        else
        {
          status = processBuf(packet, false);
        }
        
        if (!status) break;  // Exit if processing failed
      }
      else
      {
        emit logMsg(QString("Error: reported packet size of %1 too small").arg(packetSize));
        status = false;
        break;
      }
    }
    
    // If no complete packet found, wait a bit before checking again
    if (m_receiveBuffer.available() < 3)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  return status;
}

void TesterSim::process01TabletInfo(const Packet& /*inbuf*/, Packet &out, TesterSim* sim)
{
  sim->log("Request for Tester info");
  std::vector<uint8_t> outbuf = {
    0x03, // sys loader maj version
    0x01, // sys loader min version
    0x25, // sys loader date (day)
    0x09, // sys loader date (month)
    0x19, // sys loader date (decade)
    0x98, // sys loader date (year)
    0x05, // OS maj version
    0x05, // OS min version
    0x04, // OS date (day)
    0x03, // OS date (month)
    0x14, // OS date (decade)
    0x19, // OS date (year)
    0x00, // free space on flash storage (32 bit val)
    0x23,
    0x33,
    0x33,
    0,   // serial num hi
    212, // serial num lo
  };
  out.setReplyData(outbuf);
}

void TesterSim::process02SerialNo(const Packet& /*in*/, Packet& out, TesterSim* sim)
{
  sim->log("Request for Tester serial no.");
  std::vector<uint8_t> outbuf = {
    0,   // hi byte
    212, // lo byte
  };
  out.setReplyData(outbuf);
}

void TesterSim::process09(const Packet& /*in*/, Packet& out, TesterSim* /*sim*/)
{
  out.setReplyData({0x10});
}

void TesterSim::process0AWorkshopData(const Packet& in, Packet& out, TesterSim* sim)
{
  sim->log("Request for workshop data");
  std::vector<uint8_t> outbuf(111);
  //outbuf[2] = 0x76;
  //outbuf[7] = 1;
  outbuf[0] = in.data()[1];
  char c = 'a';
  int pos = 0;
  while (pos < 11)
  {
    outbuf[pos++] = c++;
  }
  outbuf[pos++] = '\0';
  while (pos < 21)
  {
    outbuf[pos++] = c++;
  }
  outbuf[pos++] = '\0';
  while (pos < 111)
  {
    outbuf[pos++] = '\0';
  }
  out.setReply(true, outbuf);
}

void TesterSim::process0BStartApplModGest(const Packet& in, Packet& out, TesterSim* sim)
{
  const uint16_t ecuId = (in.data()[0] * 0x100) + in.data()[1];
  const uint8_t pipeNum = in.data()[2];
  sim->log(QString("Starting _applModGest%1 thread on pipe %2").arg(ecuId, 4, 10, QChar('0')).arg(pipeNum));
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  sim->m_applRun[pipeNum] = true;
  sim->m_currentECUID = ecuId;
  out.setReply(true, {});
}

void TesterSim::process11DoSlowInit(const Packet& in, Packet& out, TesterSim* sim)
{
  std::this_thread::sleep_for(std::chrono::seconds(1));

  const uint8_t ecuAddr = in.data()[0];
  if (in.data().size() >= 2)
  {
    sim->log(QString("Do 5-baud slow init for ECU address 0x%1 with %2 bytes expected in response sequence").arg(ecuAddr, 2, 16, QChar('0')).arg(in.data()[1]));
  }
  else
  {
    sim->log(QString("Do 5-baud slow init for ECU address 0x%1").arg(ecuAddr, 2, 16, QChar('0')));
  }

  process12GetISOKeyword(in, out, sim);
}

void TesterSim::process12GetISOKeyword(const Packet& /*in*/, Packet& out, TesterSim* sim)
{
  if (sim->s_isoBytes.count(sim->m_currentECUID))
  {
    const std::vector<uint8_t>& isoBytes = sim->s_isoBytes.at(sim->m_currentECUID);
    const int isoByteCount = isoBytes.size();
    QString replyLogMsg = QString("Replying with keyword sequence of %1 bytes:").arg(isoByteCount);
    std::vector<uint8_t> outbuf(isoByteCount);

    for (int i = 0; i < isoByteCount; i++)
    {
      outbuf[i] = isoBytes[i];
      replyLogMsg += QString(" %1").arg(isoBytes[i], 2, 16, QChar('0'));
    }

    // There are some modules whose cmd 11/12 reply message contains
    // more than just the ISO keyword sequence -- it contains one or more
    // frames of ID data from the ECU, which are concatenated into the same
    // serial message payload from the Tester back to WSDC32.
    if (sim->s_moduleExtraInitInfo.count(sim->m_currentECUID))
    {
      const int extraDataLen = sim->s_moduleExtraInitInfo.at(sim->m_currentECUID).size();
      outbuf.resize(isoByteCount + extraDataLen);
      std::copy_n(sim->s_moduleExtraInitInfo.at(sim->m_currentECUID).data(), extraDataLen, &outbuf[isoByteCount]);

      printf("slow init reply msg:");
      for (size_t i = 0; i <= outbuf.size(); i++)
      {
        printf(" %02X", outbuf[i]);
      }
      printf("\n");
    }
    out.setReply(true, outbuf);

    sim->log(replyLogMsg);
  }
  else
  {
    sim->log(QString("Warning: no ISO byte record for ECU ID %1").arg(sim->m_currentECUID, 4, 10, QChar('0')));
  }
}

void TesterSim::process13CommandToECU(const Packet& in, Packet& out, TesterSim* sim)
{
  const int currentECU = sim->m_currentECUID;
  if (s_protocols.count(currentECU))
  {
    const ProtocolType proto = s_protocols.at(currentECU);

    // Sometimes (maybe just for certain ECUs like BMOT0145?), WSDC32 sends requests with 0x13 in position 06,
    // and the request payload starting immediately after (at position 07). In this format, the request payload
    // does not contain the block's prefix/size byte, the block sequence number, or the terminator.
    // The other format that cmd 0x13 messages can take (e.g. for BABS0096) has 0x13 at position 06, an unused
    // byte at position 07 (as a placeholder for the 0x01 status in the response), and then a *complete* ECU
    // protocol block starting at 08, for example:
    //   pos:  06 07 08 09 0A 0B
    //         -----------------
    //   val:  13 00 03 04 00 03 <-- note 03 byte count, 04 sequence num, and 03 terminator
    // instead of:
    //   val:  13 00 <-- this 00 is actually just the ID request block title, not a placeholder
    
    // The data is this command packet is assumed to verbosely contain an ECU protocol block
    // if the packet has 9 or more bytes AND there is 0x00 in position 07.
    const bool hasVerbosePayload = in.data().size() > 2 && in.data()[0] == 0x00;

    if (proto == ProtocolType::KWP71)
    {
      processKWP71CommandToECU(in, out, sim, hasVerbosePayload);
    }
    else if (proto == ProtocolType::FIAT9141)
    {
      processFIAT9141CommandToECU(in, out, sim, hasVerbosePayload);
    }
    else if (proto == ProtocolType::Marelli1AF)
    {
      processMarelli1AFCommandToECU(in, out, sim, hasVerbosePayload);
    }
    else if (proto == ProtocolType::BoschAlarm)
    {
      processBoschAlarmCommandToECU(in, out, sim, hasVerbosePayload);
    }
    else if (proto == ProtocolType::BilsteinSuspension)
    {
      processBilsteinSuspensionCommandToECU(in, out, sim, hasVerbosePayload);
    }
  }
  else
  {
    sim->log(QString("Warning: protocol for ECU ID %1 is not known").arg(sim->m_currentECUID, 4, 10, QChar('0')));
  }
}

/**
 * Parses a KWP-71 protocol block in the input buffer, and produces an
 * appropriate KWP-71 response in the output buffer. Certain bytes may be
 * omitted from the input block due to SD2 framing; this is indicated by the
 * state of the hasVerbosePayload flag.
 */
void TesterSim::processKWP71CommandToECU(const Packet& in, Packet& out, TesterSim* sim, bool hasVerbosePayload)
{
  const uint8_t blockTitle = hasVerbosePayload ? in.data()[3] : in.data()[0];

  if (blockTitle == 0x00) // Req ID code
  {
    std::vector<uint8_t> outbuf = {
      8,     // number of bytes following
      0xF6,  // KWP71 response title with ASCII/ID data
      0x31,
      0x31,
      0x32,
      0x33,
      0x35,
      0x38,
      0x03,
    };
    out.setReply(true, outbuf);
  }
  else if (blockTitle == 0x01) // Read RAM
  {
    const uint8_t count = hasVerbosePayload ? in.data()[4] : in.data()[1];
    const uint16_t addr = hasVerbosePayload ? (((uint16_t)in.data()[5] * 0x100) + in.data()[6]) : (((uint16_t)in.data()[2] * 0x100) + in.data()[3]);
    if (sim->m_ramData.count(addr) == 0)
    {
      sim->m_ramData[addr] = 0;
    }

    std::vector<uint8_t> outbuf = {
      static_cast<uint8_t>(count + 2),  // number of bytes that follow (response from ECU)
      0xFD,       // KWP71 response type to request 01
      sim->m_ramData[addr],
      // TODO: should there be more here?
      0x03,      // end-of-packet marker
    };
    out.setReply(true, outbuf);
  }
  else if (blockTitle == 0x07) // read trouble codes
  {
    // TODO: Determine the proper format and number of bytes in which the fault code
    // data is returned. We're starting with an array fixed to five bytes because that
    // appears to be what the F355 Motronic 5.2 is expecting (?)
    if (sim->m_errorMemory.size() < 5)
    {
      sim->m_errorMemory.resize(5);
    }
    const uint8_t numFaultCodeBytes = sim->m_errorMemory.size();

    std::vector<uint8_t> outbuf(numFaultCodeBytes + 1);

    outbuf[0] = numFaultCodeBytes;
    for (uint8_t errorBytePos = 0; errorBytePos < numFaultCodeBytes; errorBytePos++)
    {
      outbuf[1 + errorBytePos] = sim->m_errorMemory[errorBytePos];
    }
    out.setReply(true, outbuf);
  }
  else
  {
    sim->log("Warning: unhandled KWP71 command");
    out.setReply(true, {});
  }
}

/**
 * Parses a FIAT-9141 protocol block in the input buffer, and produces an
 * appropriate FIAT-9141 response in the output buffer. Certain bytes may be
 * omitted from the input block due to SD2 framing; this is indicated by the
 * state of the hasVerbosePayload flag.
 */
void TesterSim::processFIAT9141CommandToECU(const Packet& in, Packet& out, TesterSim* sim, bool hasVerbosePayload)
{
  const uint8_t blockTitle = hasVerbosePayload ? in.data()[2] : in.data()[0];

  if (blockTitle == 0x00) // Req ID code
  {
    std::vector<uint8_t> outbuf = {
      8,
      0xF6,
      0x31,
      0x31,
      0x32,
      0x33,
      0x35,
      0x38,
      0x03,
    };
    out.setReply(true, outbuf);
  }
  else if (blockTitle == 0x01) // Read RAM
  {
    const uint8_t count = hasVerbosePayload ? in.data()[3] : in.data()[1];
    const uint16_t addr = hasVerbosePayload ? (((uint16_t)in.data()[4] * 0x100) + in.data()[5]) : (((uint16_t)in.data()[2] * 0x100) + in.data()[3]);
    if (sim->m_ramData.count(addr) == 0)
    {
      sim->m_ramData[addr] = 0;
    }

    std::vector<uint8_t> outbuf = {

      static_cast<uint8_t>(count + 2),  // number of bytes that follow (response from ECU)
      0xFD,       // KWP71 response type to request 01
      sim->m_ramData[addr],
      // TODO: should there be more here?
      0x03,      // end-of-packet marker
    };
    out.setReply(true, outbuf);
  }
  else
  {
    sim->log("Warning: unhandled FIAT9141 command");
    out.setReply(true, {});
  }
}

/**
 * Parses a Marelli 1AF protocol block in the input buffer, and produces an
 * appropriate 1AF response in the output buffer. Certain bytes may be omitted
 * from the input block due to SD2 framing; this is indicated by the state of
 * the hasVerbosePayload flag.
 */
void TesterSim::processMarelli1AFCommandToECU(const Packet& in, Packet& out, TesterSim* sim, bool hasVerbosePayload)
{
  const uint8_t blockTitle = hasVerbosePayload ? in.data()[2] : in.data()[0];

  if (blockTitle == 0x51) // request for ID info
  {
    std::vector<uint8_t> outbuf = {
      16,   // count
      0xAE, // ID of reply to request for info
      0xAA, // normally sync bytes for 1AF protocol, but SD2 seems to expect that
            // the Marelli controller for the Ferrari 355 F1 gearbox put the
            // "Marelli ECU code" value here
      0x55,
      0xCC,
      0x33,
      0x31, // start of Marelli SW version
      0x32,
      0x33,
      0x34,
      0x35,
      0x36,
      0x97, // SW release year in BCD
      0x01, // SW release month in BCD
      0x02, // SW release day in BCD
      0xAA, // ID info block terminator
      0x00, // checkusm placeholder
    };
    add8BitChecksum(&outbuf[0]);
    out.setReply(true, outbuf);
  }
  else if ((blockTitle == 0x20) || (blockTitle == 0x21)) // activate actuator / stop actuation
  {
    if (blockTitle == 0x20)
    {
      const uint8_t actuatorID = hasVerbosePayload ? in.data()[3] : in.data()[2];
      const uint8_t actuatorParam = hasVerbosePayload ? in.data()[4] : in.data()[3];
      sim->log(QString("ACTUATOR: ID 0x%1, parameter 0x%2").arg(actuatorID, 2, 16, QChar('0')).arg(actuatorParam, 2, 16, QChar('0')));
    }
    std::vector<uint8_t> outbuf = {
      3,
      0x09,
      0x00,
      0x00,
    };

    add16BitChecksum(&outbuf[0]);
    out.setReply(true, outbuf);
  }
  else if (blockTitle == 0x01) // set diagnostic mode
  {
    const uint8_t diagnosticMode = hasVerbosePayload ? in.data()[3] : in.data()[1];

    std::vector<uint8_t> outbuf = {
      5,
      0x0D,
      diagnosticMode,
      0x00, // fixed at 00 according to page 40 of FIAT 3.00601 PDF
      0x00,
      0x00,
    };
    add16BitChecksum(&outbuf[0]);
    out.setReply(true, outbuf);
  }
  else if (blockTitle == 0x30) // read RAM/ROM/EEPROM
  {
    const uint16_t startAddr = hasVerbosePayload ?
      (static_cast<uint16_t>(in.data()[3] << 8) | (in.data()[4] & 0xff)) :
      (static_cast<uint16_t>(in.data()[1] << 8) | (in.data()[2] & 0xff));
    const uint8_t numBytes = hasVerbosePayload ? in.data()[5] : in.data()[3];

    std::vector<uint8_t> outbuf(3 + numBytes);
    outbuf[0] = 3 + numBytes;
    outbuf[1] = 0xCF;
    for (uint16_t addr = startAddr; addr < (startAddr + numBytes); addr++)
    {
      outbuf[2 + addr] = sim->m_ramData[addr];
    }
    add16BitChecksum(&outbuf[0]);
    out.setReply(true, outbuf);
  }
  else if (blockTitle == 0x31) // read value
  {
    const uint8_t valueCode = hasVerbosePayload ? in.data()[3] : in.data()[1];

    std::vector<uint8_t> outbuf = {
      7,    // bytecount
      0xCE, // reply title
      static_cast<uint8_t>(sim->m_valueData[valueCode] >> 24),
      static_cast<uint8_t>(sim->m_valueData[valueCode] >> 16),
      static_cast<uint8_t>(sim->m_valueData[valueCode] >> 8),
      static_cast<uint8_t>(sim->m_valueData[valueCode] & 0xff),
      0x00,
      0x00,
    };
    add16BitChecksum(&outbuf[0]);
    out.setReply(true, outbuf);
  }
  else if (blockTitle == 0x32) // request for snapshot
  {
    const uint8_t snapshotIndex = hasVerbosePayload ? in.data()[3] : in.data()[1];

    // If we get a request for snapshot data on a page that hasn't yet been
    // explicitly populated by the GUI, resize it to the minimum page size
    // that we don't sent a short page that would cause WSDC32 to read
    // uninitialized memory.
    if (sim->m_snapshotData[snapshotIndex].size() < DEFAULT_SNAPSHOT_SIZE)
    {
      sim->m_snapshotData[snapshotIndex].resize(DEFAULT_SNAPSHOT_SIZE, 0);
    }

    const uint8_t numBytesInSnapshot = sim->m_snapshotData[snapshotIndex].size();

    std::vector<uint8_t> outbuf(4 + numBytesInSnapshot);

    outbuf[0] = 3 + numBytesInSnapshot; // bytecount in the 1AF frame; pg. 28 of FIAT 3.00601 Marelli 1AF document seems to have an error here
    outbuf[1] = 0xCD; // reply title

    for (unsigned int i = 0; i < numBytesInSnapshot; i++)
    {
      outbuf[2 + i] = sim->m_snapshotData[snapshotIndex][i];
    }
    add16BitChecksum(&outbuf[0]);
    out.setReply(true, outbuf);
  }
  else if (blockTitle == 0x50)
  {
    if (sim->m_errorMemory.size() < DEFAULT_ERROR_MEMORY_SIZE)
    {
      sim->m_errorMemory.resize(DEFAULT_ERROR_MEMORY_SIZE, 0);
    }
    const uint8_t numBytesInErrorMem = sim->m_errorMemory.size();

    std::vector<uint8_t> outbuf(4 + numBytesInErrorMem);

    outbuf[0] = 3 + numBytesInErrorMem; // bytecount in the 1AF frame
    outbuf[1] = 0xAF; // reply title

    for (unsigned int i = 0; i < numBytesInErrorMem; i++)
    {
      outbuf[2 + i] = sim->m_errorMemory[i];
    }
    add16BitChecksum(&outbuf[0]);
    out.setReply(true, outbuf);
  }
  else
  {
    sim->log("Warning: unhandled FIAT/Marelli 1AF command");
    out.setReply(true, {});
  }
}

/**
 * When commands 52 FE 01 and 52 FF 01 are sent to the ECU, the bytes in the
 * reply are taken together as a BCD representation of the Bosch VIM security
 * system version, e.g. 01 71 is VIM171.
 * WSDC32's behavior (i.e. the commands it then sends for diagnostics) will
 * change depending on the VIM version.
 */
void TesterSim::processBoschAlarmCommandToECU(const Packet& in, Packet& out, TesterSim* sim, bool /*hasVerbosePayload*/)
{
  // A typical command looks like: (... 13 01) 52 FE 01
  if (in.data()[1] == 0x52)
  {
    const uint8_t commNumberHi = in.data()[2];
    const uint8_t commNumberLo = in.data()[3];
    const uint16_t commNumber = ((uint16_t)commNumberHi << 8) | commNumberLo;

    if (sim->m_ramData.count(commNumber) == 0)
    {
      sim->m_ramData[commNumber] = 0;
    }

    // This command apparently reads a single byte from the ECU, and that is the only
    // thing echoed back to WSDC32 in the payload (i.e. after byte index 07)
    out.setReply(true, {sim->m_ramData[commNumber]});

  }
  else if (in.data()[1] == 0x44)
  {
    // This is another type of Read command -- possibly from a different address space or device?
    // Unlike cmd 52h, it is followed by only a single byte (which must be an 8-bit address.)
    const uint8_t commNumber = in.data()[2];
    if (sim->m_ramData.count(commNumber) == 0)
    {
      sim->m_ramData[commNumber] = 0;
    }
    out.setReply(true, {sim->m_ramData[commNumber]});
  }
  else
  {
    sim->log("Warning: unhandled Bosch Alarm command");
  }
}

/**
 */
void TesterSim::processBilsteinSuspensionCommandToECU(const Packet& in, Packet& out, TesterSim* sim, bool /*hasVerbosePayload*/)
{
  // A typical command looks like: (... 13 01) 01 00 23 00 22

  // Is this attempting to read a location in ECU memory?
  if (in.data()[1] == 0x01)
  {
    const uint8_t addrHi = in.data()[2];
    const uint8_t addrLo = in.data()[3];
    const uint16_t addr = ((uint16_t)addrHi << 8) | addrLo;

    if (sim->m_ramData.count(addr) == 0)
    {
      sim->m_ramData[addr] = 0;
    }

    // Total byte count for the SD2 Tester msg (should match the index of the last byte)
    // Indication of success. Note that this byte overwrites a byte *count* that we
    // received from WSDC32 (where it would have been 05 for the 5-byte message that follows)

    std::vector<uint8_t> outbuf(5);
    outbuf[0] = 0x01;
    outbuf[1] = addrHi;
    outbuf[2] = addrLo;
    outbuf[3] = sim->m_ramData[addr];
    outbuf[4] = (outbuf[0] ^ outbuf[1] ^ outbuf[2] ^ outbuf[3]);
    out.setReply(true, outbuf);
  }
  else if (in.data()[1] == 0x06) // unknown
  {
    const uint8_t addrHi = in.data()[2];
    const uint8_t addrLo = in.data()[3];

    // total byte count for the SD2 Tester msg (should match the index of the last byte)
    // Indication of success. Note that this byte overwrites a byte *count* that we
    // received from WSDC32 (where it would have been 05 for the 5-byte message that follows)

    std::vector<uint8_t> outbuf(5);
    outbuf[0] = 0x06;
    outbuf[1] = addrHi;
    outbuf[2] = addrLo;
    outbuf[3] = 7;
    outbuf[4] = (outbuf[0] ^ outbuf[1] ^ outbuf[2] ^ outbuf[3]);
    out.setReply(true, outbuf);
  }
  else if (in.data()[1] == 0x0B) // something to do with actuator activation
  {
    sim->log("Warning: Bilstein suspension ECU command for actuators not yet implemented");
  }
  else if (in.data()[1] == 0x11)
  {
    // This command seems to be requesting fault codes from a redundant memory location
    // In addition to the faults being stored in normally addressable RAM locations
    // (at least for BSOS0088), they seem to be stored -- with the same relative bit
    // positions -- in data locations that are read with the command 0x11.

    const uint8_t byteA = in.data()[2];
    const uint8_t byteB = in.data()[3];

    // total byte count for the SD2 Tester msg (should match the index of the last byte)
    // Indication of success. Note that this byte overwrites a byte *count* that we
    // received from WSDC32 (where it would have been 05 for the 5-byte message that follows)

    std::vector<uint8_t> outbuf(5);
    outbuf[0] = 0x11;
    outbuf[1] = byteA;
    outbuf[2] = byteB;
    outbuf[3] = sim->m_ramData[0x55 + byteB]; // NOTE: this works for BSOS0088, others may vary
    outbuf[4] = (outbuf[0] ^ outbuf[1] ^ outbuf[2] ^ outbuf[3]);
  }
  else
  {
    sim->log("Warning: unhandled Bilstein suspension ECU command");
  }
}

void TesterSim::process15DisplayString(const Packet& in, Packet& out, TesterSim* sim)
{
  std::string dstring((char*)(in.data().data() + 6), in.data().size() - 6);
  sim->log(QString("Display string on Tester screen: '%1'").arg(QString::fromStdString(dstring)));
  out.setReply(true, {});
}

void TesterSim::process1C(const Packet& in, Packet& out, TesterSim* sim)
{
  const uint8_t pipeNum = in.module();
  sim->log(QString("Shut down ECU appl thread monitoring pipe %1").arg(pipeNum));

  if (sim->m_applRun[pipeNum])
  {
    sim->m_applRun[pipeNum] = false;
    out.setReply(true, {});
  }
  else
  {
    sim->log("Thread not yet running; replying with negative status from applModGen...");
    //outbuf[5] = 0;
    out.setReplyData({0xfe});
  }
}

void TesterSim::process1ECloseFile(const Packet& /*in*/, Packet& out, TesterSim* sim)
{
  sim->m_curFileContents = nullptr;
  sim->log(QString("Close file (which is currently '%1')").arg(sim->m_curFile));
  out.setReply(true, {});
}

void TesterSim::process20OpenFileForWriting(const Packet& in, Packet& out, TesterSim* sim)
{
  const QString filenameWithPath = QString::fromStdString(std::string((char*)in.data().data(), in.data().size()));
  const QFileInfo fileinfo(filenameWithPath);
  const QString filenameOnly = fileinfo.fileName();
  const QString dirOnly = fileinfo.absolutePath();

  sim->m_curDir = dirOnly;
  sim->m_curFile = filenameOnly;
  sim->m_curFileContents = &(sim->m_fileContents[dirOnly][filenameOnly]);
  sim->m_curFileContents->clear(); // only truncate is supported (no append)
  sim->log(QString("Open file for writing: %1 (in dir %2)").arg(sim->m_curFile).arg(sim->m_curDir));
  out.setReply(true, {});
}

void TesterSim::process21WriteToFile(const Packet& in, Packet& out, TesterSim* sim)
{
  const int byteCount = in.data().size() - 4;

  // If cmd 0x21 (Write-to-File) was the last packet we received,
  // just signal that we're processing another write. This is done
  // to decrease log clutter, since it is common for there to be
  // many dozens (or hundreds) of Write-to-File commands received
  // consecutively.
  if (sim->m_lastCmdWasWriteToFile)
  {
    sim->emitConsecutiveWriteToFileSignal();
  }
  else
  {
    sim->log(QString("Write bytes to file"));
    sim->m_lastCmdWasWriteToFile = true;
  }
  for (int i = 0; i < byteCount; i++)
  {
    sim->m_curFileContents->append(in.data()[4 + i]);
  }
  out.setReply(true, {});
}

void TesterSim::process23OpenFileForReading(const Packet& in, Packet& out, TesterSim* sim)
{
  const QString filenameWithPath = QString::fromStdString(std::string((char*)in.data().data(), in.data().size()));
  const QFileInfo fileinfo(filenameWithPath);
  const QString filenameOnly = fileinfo.fileName();
  const QString dirOnly = fileinfo.absolutePath();

  sim->m_curDir = dirOnly;
  sim->m_curFile = filenameOnly;
  sim->m_fileReadPos = 0;
  sim->m_curFileContents = &(sim->m_fileContents[dirOnly][filenameOnly]);
  memset(sim->m_checksumBuf, 0, CHKSUM_BUF_SIZE);

  sim->log(QString("Open file for reading: %1 (in dir %2)").arg(sim->m_curFile).arg(sim->m_curDir));
  out.setReply(true, {});
}

void TesterSim::process24ReadFromFile(const Packet& in, Packet& out, TesterSim* sim)
{
  if (!sim->m_curFileContents)
  {
    sim->log("Error: m_curFileContents is null. File read/write operation without an open file?");
    return;
  }

  const int bytesLeftInFile = (sim->m_curFileContents->size() - sim->m_fileReadPos);
  const int numBytesToSend = (bytesLeftInFile >= CHKSUM_BUF_SIZE) ? CHKSUM_BUF_SIZE : bytesLeftInFile;
  sim->log(QString("Read from file (%1 bytes left, %2 bytes in this chunk, file pos 0x%3)").
    arg(bytesLeftInFile).arg(numBytesToSend).arg(sim->m_fileReadPos, 8, 16, QChar('0')));
  std::vector<uint8_t> outbuf;
  outbuf.reserve(numBytesToSend + 4);
  std::copy_n(in.data().data(), 4, outbuf.end());

  if (numBytesToSend > 0)
  {
    for (int i = 0; i < numBytesToSend; i++)
    {
      outbuf[4 + i] = sim->m_curFileContents->at(sim->m_fileReadPos + i);
      sim->m_checksumBuf[i] += outbuf[4 + i];
    }
    const int checksumBufPos = 4 + numBytesToSend;
    outbuf[checksumBufPos] = 0;
    for (int i = 4; i < 4 + numBytesToSend; i++)
    {
      outbuf[checksumBufPos] += outbuf[i];
    }
    outbuf[checksumBufPos] = ~outbuf[checksumBufPos];
    sim->log(QString("Computed checksum of %1 for this chunk").arg(outbuf[checksumBufPos], 2, 16));
    sim->m_fileReadPos += numBytesToSend;
  }
  else
  {
    // No more bytes to send from this file.
    // Indicate this with a 00 in position 0xC of the reply message.
    outbuf[4] = 0;
  }
  out.setReply(true, outbuf);
}

void TesterSim::process25ChecksumFile(const Packet& /*inbuf*/, Packet& out, TesterSim* sim)
{
  sim->log("Request for checksum verification of file");
  std::vector<uint8_t> outbuf(CHKSUM_BUF_SIZE);
  for (int i = 0; i < CHKSUM_BUF_SIZE; i++)
  {
    outbuf[i] = ~(sim->m_checksumBuf[i]);
  }
  out.setReply(true, outbuf);
}

void TesterSim::process2AChdir(const Packet& in, Packet& out, TesterSim* sim)
{
  const QString curDir = QString::fromStdString(std::string((char*)in.data().data(), in.data().size()));
  sim->m_curDir = curDir;
  sim->m_curDirIterator = sim->m_fileContents[curDir].begin();
  sim->log(QString("Change directory: %1").arg(curDir));
  out.setReply(true, {});
}

void TesterSim::process2BGetNextDirEntry(const Packet& in, Packet& out, TesterSim* sim)
{
  sim->log("Request for next directory entry");

  if (sim->m_curDirIterator !=
      sim->m_fileContents[sim->m_curDir].end())
  {
    const QString filename = sim->m_curDirIterator.key();
    const uint32_t filesize = sim->m_curDirIterator.value().size();
    sim->log(QString(" File: %1, size %2").arg(filename).arg(filesize));
    const uint8_t truncLen = (filename.length() < 90) ? filename.length() : 90;
    sim->log(QString(" Truncated length of filename: %1").arg(truncLen));

    std::vector<uint8_t> outbuf = {
      in.data()[0], // 32-bit sequence num
      in.data()[1],
      in.data()[2],
      in.data()[3],
      2, // 1 == dir, 2 == other (e.g. regular file)
      static_cast<uint8_t>((filesize >> 24) & 0xff),
      static_cast<uint8_t>((filesize >> 16) & 0xff),
      static_cast<uint8_t>((filesize >> 8) & 0xff),
      static_cast<uint8_t>(filesize & 0xff),
    };
    //memcpy(outbuf + 10, "AUG-06-1998  13:24:55", 21);
    //strncpy((char*)(outbuf + 30), filename.toStdString().c_str(), 89);

    sim->m_curDirIterator++;
    out.setReply(true, outbuf);
  }
  else
  {
    // indicate end of directory
    out.setReplyData({4});
  }
}

void TesterSim::process3AGetDateTime(const Packet& /*in*/, Packet& out, TesterSim* sim)
{
  sim->log("Request for Tester date/time");
  std::vector<uint8_t> outbuf = {
    0x06,
    0x31,
    0x50,
    0x02,
    0x12,
    0x23,
    0x06,
  };
  out.setReplyData(outbuf);
}

void TesterSim::process3DEraseFlash(const Packet& in, Packet& out, TesterSim* sim)
{
  sim->log("Command to erase flash on Tester");
  out.setReply(true, {in.data()[0]});
}

void TesterSim::process60SiliconNumber(const Packet& /*in*/, Packet& out, TesterSim* sim)
{
  sim->log("Request silicon number");
  std::vector<uint8_t> outbuf(8, 0);
  out.setReplyData(outbuf);
}

void TesterSim::process61SetBoard(const Packet& in, Packet& out, TesterSim* sim)
{
  sim->log("Set board");
  uint8_t boardType = in.data()[0];
  if (boardType == 2)
  {
    sim->setTesterType(TesterType::xBOARD);
  }
  else if (boardType == 1)
  {
    sim->setTesterType(TesterType::SD2);
  }
  out.setReply(true, {});
}

void TesterSim::process62SendReset(const Packet& /*in*/, Packet& out, TesterSim* sim)
{
  sim->log("Request reset");
  out.setReply(true, {});
}

void TesterSim::process63TesterStatus(const Packet& /*in*/, Packet& out, TesterSim* sim)
{
  sim->log("Request for Tester status");
  std::vector<uint8_t> outbuf = {
    0,
    7,
    0x14,
    0x5d,
  };
  out.setReply(true, outbuf);
}

bool TesterSim::loadState(const QString& filename)
{
  bool status = false;
  QFile infile(filename);

  if (infile.open(QIODevice::ReadOnly))
  {
    QDataStream in(&infile);
    in >> m_fileContents;
    infile.close();
    status = true;

    emit logMsg("Loaded filesystem entries:");
    foreach (QString dirname, m_fileContents.keys())
    {
      emit logMsg(QString(" dir: %1").arg(dirname));
      foreach (QString filename, m_fileContents[dirname].keys())
      {
        emit logMsg(QString("  file: %1 (%2 bytes)").arg(filename).arg(m_fileContents[dirname][filename].size()));
      }
    }
    emit logMsg("(End of filesystem entry list)");
  }

  return status;
}

bool TesterSim::saveState(const QString& filename)
{
  bool status = false;
  QFile outfile(filename);

  if (outfile.open(QIODevice::WriteOnly))
  {
    QDataStream out(&outfile);
    out << m_fileContents;
    outfile.close();
    status = true;
  }

  return status;
}

void TesterSim::log(const QString& line)
{
  emit logMsg(line);
}

const std::vector<uint8_t>& TesterSim::getSnapshotContent(int snapshotIndex)
{
  return m_snapshotData[snapshotIndex];
}

void TesterSim::setSnapshotContent(int snapshotIndex, const std::vector<uint8_t>& content)
{
  m_snapshotData[snapshotIndex] = content;
  emit logMsg(QString("Set snapshot data with %1 bytes").arg(content.size()));
}

void TesterSim::setErrorMemoryContent(const std::vector<uint8_t>& content)
{
  m_errorMemory = content;
  emit logMsg(QString("Set error memory with %1 bytes").arg(content.size()));
}

void TesterSim::setTesterType(TesterType testerType)
{
  m_testerType = testerType;
  if (m_testerType == TesterType::SD2)
  {
    m_port.setBaudRate(38400);
  }
  else if (m_testerType == TesterType::xBOARD)
  {
    m_port.setBaudRate(115200);
  }
}
