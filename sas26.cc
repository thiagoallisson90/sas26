/*
 * Copyright (c) 2017 University of Padova
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Davide Magrin <magrinda@dei.unipd.it>
 */

/*
 * This script simulates a complex scenario with multiple gateways and end
 * devices. The metric of interest for this script is the throughput of the
 * network.
 */

/* IEEE SAS 2026 <https://sensorapps.org/2026>
 * Apps: [IMR, PCC]
 * IMR: 5 pkts/h
 * PCC: 1 pkt/h
 * 
 * IMR Min PDR: 99%
 * PCC Min PDR: 99%
 * 
 * IMR Max Delay: 1 minute = 60 s = 60k ms
 * PCC Max Delay: 1 s = 1000 ms
 */

 #include "ns3/building-allocator.h"
 #include "ns3/building-penetration-loss.h"
 #include "ns3/buildings-helper.h"
 #include "ns3/class-a-end-device-lorawan-mac.h"
 #include "ns3/command-line.h"
 #include "ns3/constant-position-mobility-model.h"
 #include "ns3/correlated-shadowing-propagation-loss-model.h"
 #include "ns3/double.h"
 #include "ns3/end-device-lora-phy.h"
 #include "ns3/forwarder-helper.h"
 #include "ns3/gateway-lora-phy.h"
 #include "ns3/gateway-lorawan-mac.h"
 #include "ns3/log.h"
 #include "ns3/lora-helper.h"
 #include "ns3/mobility-helper.h"
 #include "ns3/network-server-helper.h"
 #include "ns3/node-container.h"
 #include "ns3/periodic-sender-helper.h"
 #include "ns3/pointer.h"
 #include "ns3/position-allocator.h"
 #include "ns3/random-variable-stream.h"
 #include "ns3/simulator.h"
 #include "ns3/poisson-sender.h"
 #include "ns3/lora-radio-energy-model-helper.h"
 #include "ns3/basic-energy-source-helper.h"
 #include "ns3/app-tag.h"
 #include "ns3/adr-component.h"
 #include "ns3/csv-reader.h"
 
 #include <algorithm>
 #include <ctime>
 
 using namespace ns3;
 using namespace lorawan;
 
 NS_LOG_COMPONENT_DEFINE("ComplexLorawanNetworkExample");
 
 // Network settings
 int nDevices = 200;                 //!< Number of end device nodes to create
 int nGateways = 1;                  //!< Number of gateway nodes to create
 double radiusMeters = 7500;         //!< Radius (m) of the deployment
 double simulationTimeSeconds = 24 * 60 * 60; //!< Scenario duration (s) in simulated time
 
 // Channel model
 bool realisticChannelModel = true; //!< Whether to use a more realistic channel model with
                                     //!< Buildings and correlated shadowing
 
 int appPeriodSeconds = 12 * 60; //!< Duration (s) of the inter-transmission time of end devices [IMR]
 int appPeriodicSecondsPcc = 60 * 60; //!< Duration (s) of the inter-transmission time of end devices [PCC]
 
 double imrDelay = 60 * 1000; //!< 60000ms (60s)
 double pccDelay = 1 * 1000; //!< 1000ms (1s) => PCC
 
 // Output control
 bool printBuildingInfo = true; //!< Whether to print building information
 bool adrEnabled = false;
 
 std::string txMode = "nack"; //!< [nack, ack]
 std::string adrType = "ns3::AdrComponent";
 std::string adrName = "adr"; //!< [adr, adrPlus]
 std::string smFile = "";
 std::string gwFile = "";
 
 // Data Strucutures
 enum PktStatus
 {
   SENT,
   OK,
   LOST,
   EXPIRED
 };
 
 struct PacketInfo
 {
   int m_pktId;
   int m_edId;
 
   double m_txTime;
   double m_delay;
   double m_cpsrDelay;
 
   PktStatus m_status;
 
   PacketInfo(int pktId, 
              int edId, 
              double txTime) : 
              m_pktId(pktId), 
              m_edId(edId), 
              m_txTime(txTime)
   {
     m_delay = -1;
     m_cpsrDelay = -1;
     m_status = SENT;
   }
 };
 std::map<int, PacketInfo> pktInfoMap;
 
 std::vector<uint64_t> expiredPkts;
 std::vector<uint64_t> interfPkts;
 std::vector<uint64_t> underPkts;
 std::vector<uint64_t> busyPkts;
 std::vector<uint64_t> noMorePkts;
 std::vector<uint64_t> okPkts;
 
 int nSent = 0;
 int nRec = 0;
 int nRetx = 0;
 int nReqTx = 0;
 int nRecAck = 0;
 
 double sumDelay = 0;
 double sumRssi = 0;
 double sumPktsRssi = 0;
 double sumSnr = 0;
 double sumPktsSnr = 0;
 double consumption = 0;
 
 int nTotalPkts = 0;
 int nLost = 0;
 int nInterf = 0;
 int nUnder = 0;
 int nBusy = 0;
 int nNoMore = 0;
 int nExpired = 0;
 int nRun = 1;
 
 std::vector<int> sfDist(6, 0);
 std::string path = "./";
 std::string sfa = ""; //!< ["isfa", "rsfa", "drsfa", "drsfa+"]
 
 int nSentPerHour = 0;
 int nRecPerHour = 0;
 int payloadSize = 51;
 
 std::vector<double> pdrsPerHourVec;
 NodeContainer endDevices;
 
 std::vector<int> interfPerSf(6, 0);
 std::vector<int> underPerSf(6, 0);
 std::vector<int> expPerSf(6, 0);
 std::vector<int> busyPerSf(6, 0);
 std::vector<int> noMorePerSf(6, 0);
 
 std::vector<double> delayPerApp(2, 0.0); //!< { 0: 'IMR', 1: 'AN' }
 
 int nImrSent = 0;
 int nPccSent = 0;
 int nImrRec = 0;
 int nPccRec = 0;
 
 double RxPowerToSNR(double transmissionPower, double bandwidth = 125e3, double NF = 6)
 {
   return transmissionPower + 174 - 10 * log10(bandwidth) - NF;
 }
 
 void Sent(Ptr<const Packet> pkt, uint32_t edId)
 {
   int pktId = (int) pkt->GetUid();
   auto pktInfoIt = pktInfoMap.find(pktId);
   if(pktInfoIt != pktInfoMap.end())
   {
     nRetx++;
     return;
   }
 
   double txTime = Simulator::Now().GetNanoSeconds() * 1e-6;
   PacketInfo pktInfo(pktId, (int) edId, txTime);
   pktInfo.m_status = SENT;
   pktInfoMap.insert(std::make_pair(pktId, pktInfo));
 
   AppTag appTag;
   pkt->PeekPacketTag(appTag);
 
   if (appTag.GetMsgType() == IMR)
   {
     nImrSent++;
   }
   else if (appTag.GetMsgType() == PCC)
   {
     nPccSent++;
   }
 
   nSent++;
   nSentPerHour++;
 }
 
 int GetEdId(Ptr<const Packet> pkt)
 {
   int pktId = (int) pkt->GetUid();
   auto pktInfoIt = pktInfoMap.find(pktId);
 
   if(pktInfoIt == pktInfoMap.end())
   {
     return -1;
   }
 
   return pktInfoIt->second.m_edId;
 }
 
 void ComputeLqi(Ptr<const Packet> pkt)
 {
   LoraTag tag;
   pkt->PeekPacketTag(tag);
 
   sumPktsRssi += tag.GetReceivePower();
   sumPktsSnr += RxPowerToSNR(tag.GetReceivePower());
   nTotalPkts++;
 }
 
 int GetIndexBasedOnSf(Ptr<const Packet> pkt)
 {
   LoraTag tag;
   pkt->PeekPacketTag(tag);
   int index = (int) tag.GetSpreadingFactor() - 7;
   return index;
 }
 
 void Ok(Ptr<const Packet> pkt, uint32_t gwId)
 {
   int pktId = (int) pkt->GetUid();
   auto pktInfoIt = pktInfoMap.find(pktId);
   if(pktInfoIt == pktInfoMap.end())
   {
     return;
   }
 
   ComputeLqi(pkt);
 
   if (pktInfoIt->second.m_delay != -1)
   {
     return;
   }
 
   LoraTag tag;
   pkt->PeekPacketTag(tag);
 
   double rssi = tag.GetReceivePower();
   double snr = RxPowerToSNR(rssi);  
 
   sumRssi += rssi;
   sumSnr += snr;
   
   double delay = Simulator::Now().GetNanoSeconds() * 1e-6 - pktInfoIt->second.m_txTime;
   pktInfoIt->second.m_delay = delay;
   sumDelay += delay;
 
   AppTag appTag;
   pkt->PeekPacketTag(appTag);
   
   if (appTag.GetMsgType() == IMR && delay <= imrDelay)
   {
     pktInfoIt->second.m_status = OK;
     
     nImrRec++;
     nRec++;
     nRecPerHour++;
 
     okPkts.push_back(pkt->GetUid());
 
     delayPerApp[0] += delay;
   }
   else if (appTag.GetMsgType() == PCC && delay <= pccDelay)
   {
     pktInfoIt->second.m_status = OK;
     
     nPccRec++;
     nRec++;
     nRecPerHour++;
 
     okPkts.push_back(pkt->GetUid());
 
     delayPerApp[1] += delay;
   }
   else 
   {
     pktInfoIt->second.m_status = EXPIRED;
     nExpired++;
 
     expiredPkts.push_back(pkt->GetUid());
 
     int index = GetIndexBasedOnSf(pkt);
     expPerSf[index]++;
   }
 }
 
 void Interf(Ptr<const Packet> pkt, uint32_t gwId)
 {
   nInterf++;
   nLost++;
 
   LoraTag tag;
   pkt->PeekPacketTag(tag);
 
   ComputeLqi(pkt);
 
   uint8_t sf = tag.GetSpreadingFactor();
   interfPerSf[sf - 7]++;
 
   interfPkts.push_back(pkt->GetUid());
   
   if (sfa == "asfa")
   {
     int edId = GetEdId(pkt);
 
     Ptr<Node> node = endDevices.Get(edId);
     Ptr<LoraNetDevice> dev = node->GetDevice(0)->GetObject<LoraNetDevice>();
     Ptr<EndDeviceLorawanMac> mac = dev->GetMac()->GetObject<EndDeviceLorawanMac>();
 
     uint8_t dr = 12 - sf;
     mac->SetDataRate(dr > 0 ? dr - 1 : 0);
   }
 }
 
 void Under(Ptr<const Packet> pkt, uint32_t gwId)
 {
   nUnder++;
   nLost++;
 
   ComputeLqi(pkt);
 
   underPkts.push_back(pkt->GetUid());
 
   int index = GetIndexBasedOnSf(pkt);
   underPerSf[index]++;
 }
 
 void NoMore(Ptr<const Packet> pkt, uint32_t gwId)
 {
   nNoMore++;
   nLost++;
 
   ComputeLqi(pkt);
 
   noMorePkts.push_back(pkt->GetUid());
 
   int index = GetIndexBasedOnSf(pkt);
   noMorePerSf[index]++;
 }
 
 void Busy(Ptr<const Packet> pkt, uint32_t gwId)
 {
   nBusy++;
   nLost++;
 
   ComputeLqi(pkt);
 
   busyPkts.push_back(pkt->GetUid());
 
   int index = GetIndexBasedOnSf(pkt);
   busyPerSf[index]++;
 }
 
 std::string MakeFileName(std::string name, std::string extension = "csv") 
 {
   std::string fileName = path + "/" + std::to_string(nGateways) + "gw_" 
                               + name + "." + extension;
   return fileName;
 }
 
 void WriteFile(std::string fileName, std::string content) 
 {
   std::ofstream file(fileName, std::ios::out | std::ios::app);
 
   if (file.is_open())
   {
     file << content;
     file.close();
   }
 }
 
 void CalcEnergyConsumption(NodeContainer endDevices) 
 {
   //std::ostringstream oss;
 
   consumption = 0;
   for (uint32_t i = 0; i < endDevices.GetN(); i++)
   {
     Ptr<Node> node = endDevices.Get(i);
     if (auto esc = node->GetObject<EnergySourceContainer>())
     {
       auto demc = esc->Get(0)->FindDeviceEnergyModels("ns3::LoraRadioEnergyModel");
       if (demc.GetN())
       {
         //oss << demc.Get(0)->GetTotalEnergyConsumption() << ",";
         consumption += demc.Get(0)->GetTotalEnergyConsumption();
       }
     }
   }
 
   //oss << nRun << std::endl;
   //WriteFile(MakeFileName("energy"), oss.str());
   //oss.clear();
 }
 
 void PrintSFAndTP()
 {
   // SF7, SF8, SF9, SF10, SF11, SF12, TP2, TP4, TP6, TP8, TP10, TP12, TP14, nRun
   std::ostringstream oss;
 
   //std::vector<int> tpDist(7, 0);
   std::vector<int> tpDist(14, 0);
 
   for (int i = 0; i < nDevices; i++)
   {
     Ptr<Node> node = endDevices.Get(i);
     Ptr<NetDevice> netDevice = node->GetDevice(0);
     Ptr<LoraNetDevice> loraNetDevice = DynamicCast<LoraNetDevice>(netDevice);
     Ptr<ClassAEndDeviceLorawanMac> mac =
         DynamicCast<ClassAEndDeviceLorawanMac>(loraNetDevice->GetMac());
 
     sfDist[5 - (int) mac->GetDataRate()]++;
 
     int txPower = (int) mac->GetTransmissionPower();
     if (txPower > 0)
     {
       //int index = txPower / 2 - 1; // index of TP2 => index => 2 / 2 - 1 = 0
       int index = txPower - 1; // index of TP1 => index => TP1 - 1 = 0
       tpDist[index]++;
     }
   }
 
   for (size_t i = 0; i < sfDist.size(); i++)
   {
     oss << (1.0 * sfDist[i] / nDevices * 100) << ",";
     //std::cout << sfDist[i] << " | ";
   }
   //std::cout << std::endl;
   for (size_t i = 0; i < tpDist.size(); i++)
   {
     oss << (1.0 * tpDist[i] / nDevices * 100) << ",";
   }
   oss << nRun << std::endl;
 
   WriteFile(MakeFileName("sf_tp"), oss.str());
   oss.clear();
   
   tpDist.clear();
   sfDist.clear();
 }
 
 void PrintMainData()
 {
   // sent,rec,pdr,imr_sent,imr_rec,imr_pdr,billing_sent,billing_rec
   // billing_pdr,delay,rssi,snr,energy,tput,ee1,ee2,ee3,ee4,nRun
   std::ostringstream oss;
 
   double pdr = ((nSent > 0 ? 1.0 * nRec / nSent : 0.0) * 100);
   double imrPdr = ((nImrSent > 0 ? 1.0 * nImrRec / nImrSent : 0.0) * 100);
   double billingPdr = ((nPccSent > 0 ? 1.0 * nPccRec / nPccSent : 0.0) * 100);
   
   double avgDelay = (nRec > 0 ? sumDelay / nRec : 0.0);
   double avgRssi = (nRec > 0 ? sumRssi / nRec : 0.0);
   double avgSnr = (nRec > 0 ? sumSnr / nRec : 0.0);
   
   double energyCons = consumption / nDevices;
   double tput = (nRec * payloadSize * 8) / (simulationTimeSeconds);
   double ee1 = (nRec * payloadSize * 8) / consumption;
   double ee2 = (nRec * payloadSize * 8) / energyCons;
   double ee3 = tput / energyCons;
   double ee4 = tput / consumption;
 
   double avgPktsRssi = (nTotalPkts > 0 ? sumPktsRssi / nTotalPkts : 0.0);
   double avgPktsSnr = (nTotalPkts > 0 ? sumPktsSnr / nTotalPkts : 0.0);
   
   double cpsr = 0;
   if (txMode == "ack")
   {
     cpsr = (nRec > 0 ? (1.0 * nRecAck / nRec) * 100 : 0.0);
     oss << nSent << "," << nRec << "," << pdr << "," << nImrSent << "," << nImrRec << "," << imrPdr 
       << "," << nPccSent << "," << nPccRec << "," << billingPdr << "," << avgDelay << "," 
       << avgRssi << "," << avgSnr << "," << energyCons << "," << tput << "," << ee1 << "," << ee2 
       << "," << ee3 << "," << ee4 << "," << nReqTx << "," << nRecAck << "," << cpsr << "," 
       << avgPktsRssi << "," << avgPktsSnr << "," << nRun << std::endl;
   }
   else if (txMode == "nack")
   {
     oss << nSent << "," << nRec << "," << pdr << "," << nImrSent << "," << nImrRec << "," << imrPdr 
         << "," << nPccSent << "," << nPccRec << "," << billingPdr << "," << avgDelay << "," 
         << (delayPerApp[0] / nImrRec) << "," << (delayPerApp[1] / nPccRec) << "," << avgRssi << "," 
         << avgSnr << "," << energyCons << "," << tput << "," << ee1 << "," << ee2 << "," 
         << ee3 << "," << ee4 << "," << avgPktsRssi << "," << avgPktsSnr << "," << nRun << std::endl;
   }
 
   WriteFile(MakeFileName("data"), oss.str());
   oss.clear();
 
   /*std::cout << "nSent = " << nSent << std::endl;
   std::cout << "nRec = " << nRec << std::endl;
   std::cout << "PDR = " << pdr << "%" << std::endl;
   std::cout << "Avg. Delay = " << avgDelay << " ms" << std::endl;
   
   std::cout << "Avg. RSSI = " << avgRssi << " dBm" << std::endl;
   std::cout << "Avg. SNR = " << avgSnr << " dB" << std::endl;
   
   std::cout << "Avg. Energy Consumption = " << energyCons << " J" << std::endl;
   std::cout << "Tput = " << tput << " b/s" << std::endl;
   std::cout << "EE1 = " << ee1 << " b/J" << std::endl;
   std::cout << "EE2 = " << ee2 << " b/J" << std::endl;
   std::cout << "EE3 = " << ee3 << " b/s/J" << std::endl;
   std::cout << "EE4 = " << ee4 << " b/s/J" << std::endl;
 
   std::cout << "Number of Retx = " << nReqTx << std::endl;
   std::cout << "Number of Received ACK = " << nRecAck << std::endl;
   std::cout << "CPSR = " << cpsr << "%" << std::endl;*/
 }
 
 void PrintSep()
 {
   for (int i = 0; i < 100; i++)
   {
     std::cout << "#";
   }
   std::cout << std::endl;
 }
 
 void PrintLoss()
 {
   // nInterf, nUnder, nNoMore, nBusy, nExp, nLost
   // interf_rate, under_rate, nomore_rate, busy_rate, exp_rate
   // interf_sf7, interf_sf8, interf_sf9, interf_sf10, interf_sf11, interf_sf12
 
   std::ostringstream oss;
   
   oss << nInterf << ",";
   oss << nUnder << ",";
   oss << nNoMore << ",";
   oss << nBusy << ",";
   oss << nExpired << ",";
   oss << nLost << ",";
 
   oss << (nLost > 0 ? (1.0 * nInterf / nLost) * 100 : 0.0) << ",";
   oss << (nLost > 0 ? (1.0 * nUnder / nLost) * 100  : 0.0) << ",";
   oss << (nLost > 0 ? (1.0 * nNoMore / nLost) * 100 : 0.0) << ",";
   oss << (nLost > 0 ? (1.0 * nBusy / nLost) * 100 : 0.0) << ",";  
   oss << (nLost > 0 ? (1.0 * nExpired / nLost) * 100 : 0.0) << ",";  
 
   if (nInterf > 0)
   {
     for (auto interf: interfPerSf)
     {
       oss << ((1.0 * interf / nInterf) * 100) << ",";
     }
   }
 
   oss << nRun << std::endl;
 
   WriteFile(MakeFileName("losses"), oss.str());
   oss.clear();
 
   /*for (auto exp: expPerSf)
   {
     std::cout << exp << " ";
   }
   std::cout << std::endl;*/
 }
 
 void PrintPdrsPerHour()
 {
   std::ostringstream oss;
   for (size_t i = 0; i < pdrsPerHourVec.size(); i++)
   {
     oss << i+1 << "," << pdrsPerHourVec[i] << std::endl;
   }
 
   WriteFile(MakeFileName("pdrs_" + std::to_string(nRun)), oss.str());
   oss.clear();
 }
 
 void PrintData()
 {
   PrintSep();
   
   std::cout << "** nRun = " << nRun << " **" << std::endl;
   PrintMainData();
   PrintSFAndTP();
   //PrintPdrsPerHour();
   PrintLoss();
 
   /*std::cout << std::endl;
   std::cout << "Ok: " << unsigned(okPkts.size()) << std::endl;
   std::cout << "Expired: " << unsigned(expiredPkts.size()) << std::endl;
   std::cout << "Under: " << unsigned(underPkts.size()) << std::endl;
   std::cout << "Interf: " << unsigned(interfPkts.size()) << std::endl;
   std::cout << "No More: " << unsigned(noMorePkts.size()) << std::endl;
   std::cout << "Busy: " << unsigned(busyPkts.size()) << std::endl;
   PrintSep();
 
   for (auto ok: okPkts)
   {
     for (size_t i = 0; i < expiredPkts.size(); i++)
     {
       if (ok == expiredPkts[i])
       {
         expiredPkts.erase(expiredPkts.begin() + i);
       }
     }
 
     for (size_t i = 0; i < underPkts.size(); i++)
     {
       if (ok == underPkts[i])
       {
         underPkts.erase(underPkts.begin() + i);
         
       }
     }
 
     for (size_t i = 0; i < interfPkts.size(); i++)
     {
       if (ok == interfPkts[i])
       {
         interfPkts.erase(interfPkts.begin() + i);
         
       }
     }
 
     for (size_t i = 0; i < noMorePkts.size(); i++)
     {
       if (ok == noMorePkts[i])
       {
         noMorePkts.erase(noMorePkts.begin() + i);
         
       }
     }
 
     for (size_t i = 0; i < busyPkts.size(); i++)
     {
       if (ok == busyPkts[i])
       {
         busyPkts.erase(busyPkts.begin() + i);
         
       }
     }
   }
 
   std::cout << "Sent: " << nSent << std::endl;
   std::cout << "Ok: " << unsigned(okPkts.size()) << std::endl;
   std::cout << "Ok: " << nRec << std::endl;
   std::cout << "Expired: " << unsigned(expiredPkts.size()) << std::endl;
   std::cout << "Under: " << unsigned(underPkts.size()) << std::endl;
   std::cout << "Interf: " << unsigned(interfPkts.size()) << std::endl;
   std::cout << "NoMore: " << unsigned(noMorePkts.size()) << std::endl;
   std::cout << "Busy: " << unsigned(busyPkts.size()) << std::endl;*/
 
   PrintSep();
 }
 
 void ClearData()
 {
   pktInfoMap.clear();
   sfDist.clear();
   pdrsPerHourVec.clear();
   interfPerSf.clear();
 
   expiredPkts.clear();
   interfPkts.clear();
   underPkts.clear();
   busyPkts.clear();
   noMorePkts.clear();
   okPkts.clear();
 
   underPerSf.clear();
   expPerSf.clear();
   noMorePerSf.clear();
   busyPerSf.clear();
   delayPerApp.clear();
 }
 
 void RequiredTransmissionsCallback(uint8_t reqTx,
                                   bool success,
                                   Time firstAttempt,
                                   Ptr<Packet> packet)
 {
   if (!packet)
   {
     return;
   }
   
   int pktId = (int) packet->GetUid();
   auto pktInfoIt = pktInfoMap.find(pktId);
   if(pktInfoIt == pktInfoMap.end())
   {
     return;
   }
 
   if (success && pktInfoIt->second.m_cpsrDelay == -1)
   {
     pktInfoIt->second.m_cpsrDelay = Simulator::Now().GetNanoSeconds() * 1e-6
                                       - firstAttempt.GetNanoSeconds() * 1e-6;              
     
     nRecAck++;
     nReqTx += reqTx;
   }
 }
 
 void CalcPdrsPerHour()
 {
   double pdr = (nSentPerHour > 0 ? (1.0 * nRecPerHour / nSentPerHour) * 100 : 0.0);
   pdrsPerHourVec.push_back(pdr <= 100 ? pdr : 100);
 
   nSentPerHour = 0;
   nRecPerHour = 0;
 
   Simulator::Schedule(Hours(1.0), &CalcPdrsPerHour);
 }
 
 /**
  * Record a change in the data rate setting on an end device.
  *
  * \param oldDr The previous data rate value.
  * \param newDr The updated data rate value.
  */
 void
 OnDataRateChange(uint8_t oldDr, uint8_t newDr)
 {
     std::cout << "DR" << unsigned(oldDr) << " -> DR" << unsigned(newDr) << std::endl;
 }
 
 /**
  * Record a change in the transmission power setting on an end device.
  *
  * \param oldTxPower The previous transmission power value.
  * \param newTxPower The updated transmission power value.
  */
 void
 OnTxPowerChange(double oldTxPower, double newTxPower)
 {
     std::cout << oldTxPower << " dBm -> " << newTxPower << " dBm" << std::endl;
 }
 
 void PositionNodes(NodeContainer nodes, std::string filePath, double z)
 {
   CsvReader csv(filePath);
 
   MobilityHelper mob;
   mob.SetPositionAllocator("ns3::ConstantPositionMobilityModel");
   Ptr<ListPositionAllocator> alloc = CreateObject<ListPositionAllocator>();
 
   while (csv.FetchNextRow ())
   {
     if (csv.IsBlankRow ())
     {
       continue;
     }
 
     double x, y;
     csv.GetValue(0, x);
     csv.GetValue(1, y);
     alloc->Add(Vector3D(x, y, z));
   }
 
   mob.SetPositionAllocator(alloc);
   mob.Install(nodes);
 }
 
 void CalcDataPerHour()
 {
 }

 int
 main(int argc, char* argv[])
 {
     CommandLine cmd(__FILE__);
     cmd.AddValue("nDevices", "Number of end devices to include in the simulation", nDevices);
     cmd.AddValue("nGateways", "Number of gateways to include in the simulation", nGateways);
 
     cmd.AddValue("radius", "The radius (m) of the area to simulate", radiusMeters);
 
     cmd.AddValue("realisticChannel",
                  "Whether to use a more realistic channel model",
                  realisticChannelModel);
 
     cmd.AddValue("simulationTime", "The time (s) for which to simulate", simulationTimeSeconds);
     cmd.AddValue("appPeriod",
                  "The period in seconds to be used by periodically transmitting applications",
                  appPeriodSeconds);
     
     cmd.AddValue("nRun", "Number of Running", nRun);
     cmd.AddValue("path", "Path to Save Results", path);
 
     cmd.AddValue("sfa", "Spreading Factor Allocation Scheme", sfa);
     cmd.AddValue("payload", "Payload Size", payloadSize);
     cmd.AddValue("txMode", "Transmissiom Mode: NACK or ACK", txMode);
     
     cmd.AddValue("adrEnabled", "Whether to enable Adaptive Data Rate (ADR)", adrEnabled);
     cmd.AddValue("adrType", "ADR Type", adrType);
     cmd.AddValue("adrName", "ADR Name", adrName);
 
     cmd.AddValue("smFile", "File with the SM coordinates", smFile);
     cmd.AddValue("gwFile", "File with the GW coordinates", gwFile);
 
     cmd.Parse(argc, argv);
 
     RngSeedManager::SetSeed(2);
     RngSeedManager::SetRun(nRun);
 
     // Set ToAs
     std::vector<double> toas = {0.112896, 0.205312, 0.369664, 0.698368, 1.47866, 2.62963};
 
     if (adrEnabled)
     {
       if (adrName == "adr")
       {
          Config::SetDefault("ns3::EndDeviceLorawanMac::DRControl", BooleanValue(true));
          Config::SetDefault("ns3::AdrComponent::HistoryRange", IntegerValue(20));
          Config::SetDefault("ns3::AdrComponent::MultiplePacketsCombiningMethod", 
                             EnumValue(AdrComponent::MAXIMUM));
       }
       if (adrName == "caadr")
       {
          Config::SetDefault("ns3::EndDeviceLorawanMac::DRControl", BooleanValue(true));
          Config::SetDefault("ns3::AdrComponent::HistoryRange", IntegerValue(20));
          Config::SetDefault("ns3::AdrComponent::MultiplePacketsCombiningMethod", 
                             EnumValue(AdrComponent::AVERAGE));
          Config::SetDefault("ns3::CAADR::Interval", DoubleValue(600));
          Config::SetDefault("ns3::CAADR::ToAs", 
                             StringValue("0.112896,0.205312,0.369664,0.698368,1.47866,2.62963"));
       }
       if (adrName == "mbadr")
       {
          Config::SetDefault("ns3::EndDeviceLorawanMac::DRControl", BooleanValue(true));
          Config::SetDefault("ns3::AdrComponent::HistoryRange", IntegerValue(5));
       }  
     }
 
     //Config::SetDefault("ns3::LinearLoraTxCurrentModel::Voltage", DoubleValue(3.7));
 
     /***********
      *  Setup  *
      ***********/
 
     // Create the time value from the period
     Time appPeriod = Seconds(appPeriodSeconds);
 
     /************************
      *  Create End Devices  *
      ************************/
 
     // Create a set of nodes
     endDevices.Create(nDevices);
 
     // Mobility
     MobilityHelper mobility;
 
     if (smFile == "")
     {
       mobility.SetPositionAllocator("ns3::UniformDiscPositionAllocator",
                                     "rho",
                                     DoubleValue(radiusMeters),
                                     "X",
                                     DoubleValue(0.0),
                                     "Y",
                                     DoubleValue(0.0));
       mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
 
       // Assign a mobility model to each node
       mobility.Install(endDevices);
 
       // Make it so that nodes are at a certain height > 0
       for (auto j = endDevices.Begin(); j != endDevices.End(); ++j)
       {
           Ptr<MobilityModel> mobility = (*j)->GetObject<MobilityModel>();
           Vector position = mobility->GetPosition();
           position.z = 1.5;
           mobility->SetPosition(position);
       }
     }
     else
     {
       PositionNodes(endDevices, smFile, 1.5);
     }
 
     /************************
      *  Create the channel  *
      ************************/
 
     // Create the lora channel object
     Ptr<LogDistancePropagationLossModel> loss = CreateObject<LogDistancePropagationLossModel>();
     loss->SetPathLossExponent(3.76); // Anterior: 3.52 (Suburban)
     loss->SetReference(1, 7.7);
 
     // Create the correlated shadowing component
     Ptr<CorrelatedShadowingPropagationLossModel> shadowing =
         CreateObject<CorrelatedShadowingPropagationLossModel>();
     // Aggregate shadowing to the logdistance loss
     loss->SetNext(shadowing);

     // Here we can add variance to the propagation model with multipath Rayleigh fading
     /*Ptr<NakagamiPropagationLossModel> rayleigh = CreateObject<NakagamiPropagationLossModel>();
     rayleigh->SetAttribute("m0", DoubleValue(1.0));
     rayleigh->SetAttribute("m1", DoubleValue(1.0));
     rayleigh->SetAttribute("m2", DoubleValue(1.0));
     // Aggregate fading to the Correlated Shadowing loss
     shadowing->SetNext(rayleigh);*/
 
     // Add the effect to the channel propagation loss
     /*Ptr<BuildingPenetrationLoss> buildingLoss = CreateObject<BuildingPenetrationLoss>();
     shadowing->SetNext(buildingLoss);*/
 
     Ptr<PropagationDelayModel> delay = CreateObject<ConstantSpeedPropagationDelayModel>();
     Ptr<LoraChannel> channel = CreateObject<LoraChannel>(loss, delay);
 
     /************************
      *  Create the helpers  *
      ************************/
 
     // Create the LoraPhyHelper
     LoraPhyHelper phyHelper = LoraPhyHelper();
     phyHelper.SetChannel(channel);
 
     // Create the LorawanMacHelper
     LorawanMacHelper macHelper = LorawanMacHelper();
 
     // Create the LoraHelper
     LoraHelper helper = LoraHelper();
     // helper.EnablePacketTracking(); // Output filename
     // helper.EnableSimulationTimePrinting ();
 
     // Create the NetworkServerHelper
     NetworkServerHelper nsHelper = NetworkServerHelper();
 
     // Create the ForwarderHelper
     ForwarderHelper forHelper = ForwarderHelper();
 
     // Create the LoraNetDevices of the end devices
     uint8_t nwkId = 54;
     uint32_t nwkAddr = 1864;
     Ptr<LoraDeviceAddressGenerator> addrGen =
         CreateObject<LoraDeviceAddressGenerator>(nwkId, nwkAddr);
 
     // Create the LoraNetDevices of the end devices
     macHelper.SetAddressGenerator(addrGen);
     phyHelper.SetDeviceType(LoraPhyHelper::ED);
     macHelper.SetDeviceType(LorawanMacHelper::ED_A);
     NetDeviceContainer endDevicesNetDevices = helper.Install(phyHelper, macHelper, endDevices);
 
     // Now end devices are connected to the channel
 
     /*********************
      *  Create Gateways  *
      *********************/
 
     // Create the gateway nodes (allocate them uniformly on the disc)
     NodeContainer gateways;
 
     if (gwFile == "")
     {
       gateways.Create(1);
       Ptr<ListPositionAllocator> allocator = CreateObject<ListPositionAllocator>();
       // Make it so that nodes are at a certain height > 0
       allocator->Add(Vector(radiusMeters / 2, radiusMeters / 2, 30.0));
       mobility.SetPositionAllocator(allocator);
       mobility.Install(gateways);
     }
     else 
     {
       gateways.Create(nGateways);
       PositionNodes(gateways, gwFile, 30.0);
     }
 
     // Create a netdevice for each gateway
     phyHelper.SetDeviceType(LoraPhyHelper::GW);
     macHelper.SetDeviceType(LorawanMacHelper::GW);
     helper.Install(phyHelper, macHelper, gateways);
 
 
     /**********************************************
      *  Set up the end device's spreading factor  *
      **********************************************/
     if (sfa == "isfa")
     {
       LorawanMacHelper::SetSpreadingFactorsUpBasedOnGWSens(endDevices, gateways, channel);
     } 
     else if (sfa == "rsfa")
     {
       // LorawanMacHelper::RSFA(endDevices, gateways, channel, true);
       LorawanMacHelper::RSFA1(endDevices, gateways, channel, 
                               toas, true, 600, 0.99, 3);
     }
     else if (sfa == "sftpa")
     {
        //std::cout << "Accessando SFTPA1\n";
        LorawanMacHelper::SFTPA1(endDevices, gateways, channel, 
                                 toas, 3, true, 600, 0.99);
     }
     else if (sfa == "drsfa")
     {
        //std::cout << "DR-SFA\n";
        std::vector<double> maxDelays((int) endDevices.GetN(), 1);
        LorawanMacHelper::DRSFA1(endDevices, gateways, channel, toas,
                                 maxDelays, nRun, true, 600, 0.99);
     }
     else if (sfa == "drsftpa")
     {
        //std::cout << "DR-SFA\n";
        std::vector<double> maxDelays((int) endDevices.GetN(), 1);
        LorawanMacHelper::DRSFTPA(endDevices, gateways, channel, toas,
                                  maxDelays, nRun, true, 600, 0.99);
     }
 
     for (uint32_t i = 0; i < gateways.GetN(); i++)
     {
       Ptr<Node> node = gateways.Get(i);
       Ptr<LoraNetDevice> dev = node->GetDevice(0)->GetObject<LoraNetDevice>();
       Ptr<GatewayLoraPhy> phy = dev->GetPhy()->GetObject<GatewayLoraPhy>();
       phy->TraceConnectWithoutContext("ReceivedPacket", 
                                       MakeCallback(&Ok));
       phy->TraceConnectWithoutContext("LostPacketBecauseInterference", 
                                       MakeCallback(&Interf));
       phy->TraceConnectWithoutContext("LostPacketBecauseUnderSensitivity", 
                                       MakeCallback(&Under));
       phy->TraceConnectWithoutContext("LostPacketBecauseNoMoreReceivers",
                                       MakeCallback(&NoMore));
       phy->TraceConnectWithoutContext("NoReceptionBecauseTransmitting",
                                       MakeCallback(&Busy));
     }
     NS_LOG_DEBUG("Completed configuration");
 
     /*********************************************
      *  Install applications on the end devices  *
      *********************************************/
 
     Time appStopTime = Seconds(simulationTimeSeconds);
     /*PeriodicSenderHelper appHelper = PeriodicSenderHelper();
     appHelper.SetPeriod(Seconds(appPeriodSeconds));
     appHelper.SetPacketSize(payloadSize);
     Ptr<RandomVariableStream> rv =
         CreateObjectWithAttributes<UniformRandomVariable>("Min",
                                                           DoubleValue(0),
                                                           "Max",
                                                           DoubleValue(10));
     ApplicationContainer appContainer = appHelper.Install(endDevices);
 
     appContainer.Start(Seconds(0));
     appContainer.Stop(appStopTime);*/
 
     Ptr<UniformRandomVariable> m_intervalProb = CreateObject<UniformRandomVariable>();
     for (uint32_t i = 0; i < endDevices.GetN(); i++)
     {
       Ptr<Node> node = endDevices.Get(i);
       Ptr<LoraNetDevice> dev = node->GetDevice(0)->GetObject<LoraNetDevice>();
       Ptr<EndDeviceLorawanMac> mac = dev->GetMac()->GetObject<EndDeviceLorawanMac>();
       
       if (txMode == "ack")
       {
         mac->SetMType(LorawanMacHeader::CONFIRMED_DATA_UP);
       }
 
       mac->TraceConnectWithoutContext("RequiredTransmissions",
                                       MakeCallback(&RequiredTransmissionsCallback));
 
       // IMR
       Ptr<PoissonSender> app = CreateObject<PoissonSender>();
       app->SetPacketSize(payloadSize);
       app->SetInterval(Seconds(appPeriodSeconds));
       app->SetStartTime(Seconds(m_intervalProb->GetValue(0, appPeriodSeconds)));
       app->SetStopTime(appStopTime);
       app->SetMsgType(IMR);
       node->AddApplication(app);
 
       // PCC
       Ptr<PoissonSender> appPcc = CreateObject<PoissonSender>();
       appPcc->SetPacketSize(payloadSize);
       appPcc->SetInterval(Seconds(appPeriodicSecondsPcc));
       appPcc->SetStartTime(Seconds(m_intervalProb->GetValue(0, appPeriodicSecondsPcc)));
       appPcc->SetStopTime(appStopTime);
       appPcc->SetMsgType(PCC);
       node->AddApplication(appPcc);
 
       Ptr<LoraPhy> phy = dev->GetPhy();
       phy->TraceConnectWithoutContext(
           "StartSending", MakeCallback (&Sent)
       );
     }
 
     //Config::ConnectWithoutContext(
     //    "/NodeList/*/DeviceList/0/$ns3::LoraNetDevice/Mac/$ns3::EndDeviceLorawanMac/TxPower",
     //    MakeCallback(&OnTxPowerChange));
     //Config::ConnectWithoutContext(
     //    "/NodeList/*/DeviceList/0/$ns3::LoraNetDevice/Mac/$ns3::EndDeviceLorawanMac/DataRate",
     //    MakeCallback(&OnDataRateChange));
 
     /**************************
      *  Create network server  *
      ***************************/
 
     // Create the network server node
     Ptr<Node> networkServer = CreateObject<Node>();
 
     // PointToPoint links between gateways and server
     PointToPointHelper p2p;
     p2p.SetDeviceAttribute("DataRate", StringValue("5Mbps"));
     p2p.SetChannelAttribute("Delay", StringValue("2ms"));
     // Store network server app registration details for later
     P2PGwRegistration_t gwRegistration;
     for (auto gw = gateways.Begin(); gw != gateways.End(); ++gw)
     {
         auto container = p2p.Install(networkServer, *gw);
         auto serverP2PNetDev = DynamicCast<PointToPointNetDevice>(container.Get(0));
         gwRegistration.emplace_back(serverP2PNetDev, *gw);
     }
 
     // Create a network server for the network
     nsHelper.SetGatewaysP2P(gwRegistration);
     nsHelper.SetEndDevices(endDevices);
     if (adrEnabled)
     {
       std::cout << "Enabling ADR " << adrType << " in Network Server\n";
       nsHelper.EnableAdr(adrEnabled);
       nsHelper.SetAdr(adrType);
     }
     nsHelper.Install(networkServer);
 
     // Create a forwarder for each gateway
     forHelper.Install(gateways);
 
     /************************
      * Install Energy Model *
      ************************/
 
     NS_LOG_INFO("Installing energy model on end devices...");
     BasicEnergySourceHelper basicSourceHelper;
     LoraRadioEnergyModelHelper radioEnergyHelper;
 
     // configure energy source
     basicSourceHelper.Set("BasicEnergySourceInitialEnergyJ", DoubleValue(10000)); // Energy in J
     basicSourceHelper.Set("BasicEnergySupplyVoltageV", DoubleValue(3.3));
 
     radioEnergyHelper.Set("StandbyCurrentA", DoubleValue(0.0014));
     radioEnergyHelper.Set("TxCurrentA", DoubleValue(0.028));
     radioEnergyHelper.Set("SleepCurrentA", DoubleValue(0.0000015));
     radioEnergyHelper.Set("RxCurrentA", DoubleValue(0.0112));
 
     radioEnergyHelper.SetTxCurrentModel("ns3::LinearLoraTxCurrentModel");
     /*radioEnergyHelper.SetTxCurrentModel("ns3::ConstantLoraTxCurrentModel",
                                         "TxCurrent",
                                         DoubleValue(0.028));*/
 
 
     // install source on EDs' nodes
     EnergySourceContainer sources = basicSourceHelper.Install(endDevices);
     
     // install device model
     DeviceEnergyModelContainer deviceModels =
         radioEnergyHelper.Install(endDevicesNetDevices, sources);
 
     ////////////////
     // Simulation //
     ////////////////
 
     Simulator::Stop(appStopTime + Hours(1));
     //Simulator::Schedule(Hours(1.0), &CalcDataPerHour);
 
     NS_LOG_INFO("Running simulation...");
     Simulator::Run();
 
     CalcEnergyConsumption(endDevices);
     PrintData();
 
     Simulator::Destroy();
 
     ClearData();
     toas.clear();
 
     ///////////////////////////
     // Print results to file //
     ///////////////////////////
     NS_LOG_INFO("Computing performance metrics...");
 
     /*LoraPacketTracker& tracker = helper.GetPacketTracker();
     std::cout << tracker.CountMacPacketsGlobally(Seconds(0), appStopTime + Hours(1)) << std::endl;*/
 
     return 0;
 }
 