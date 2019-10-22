
#include <eigen_conversions/eigen_msg.h>
#include <mavros/frame_tf.h>
#include <mavros/utils.h>
//#include <tf_conversions/tf_eigen.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>

#include <fstream>
#include <iostream>

#include <math.h>
#include <condition_variable>
#include <mutex>

#include "transparent_transmission_api/plugin_base.hpp"

using mavlink::dji_icg::DJI_FTP_REQUEST_RESULT;
using mavros::utils::enum_value;

// Similar to plugins/command.cpp

namespace icg_plugins {

/**
 * @brief Dji FTP plugin.
 *
 * Receives files from the Android device.
 */
class DjiFtpPlugin : public icg_plugins::PluginBase {
 public:
  DjiFtpPlugin() : PluginBase(), nh("~") {}

  void initialize(const std::shared_ptr<SendCommand> &sendComm) {
    PluginBase::initialize(sendComm);
  }

  Subscriptions get_subscriptions() {
    return {make_handler(&DjiFtpPlugin::handleDjiFtpRequest),
            make_handler(&DjiFtpPlugin::handleDjiFtp)};
  }

 private:
  size_t TIMEOUT_MILISECS = 3000;

  ros::NodeHandle nh;

  //std::ofstream androidFile;
  uint8_t currentFileId;
  uint16_t currentNumOfSequences;
  std::vector<bool> receivedSequences;
  uint32_t currentFileSize;
  std::string currentFileName;
  std::vector<uint8_t> currentData;


  std::condition_variable ftpConditionVar;

  std::unique_ptr<std::thread> ftpTimeoutThread;

  void sendAck(uint8_t fileId, DJI_FTP_REQUEST_RESULT &result,
               uint8_t progress = 0) {
    mavlink::dji_icg::msg::DJI_FTP_REQUEST_ACK msg;
    msg.target_system = GROUND_STATION_ID;
    msg.file_id = fileId;
    msg.result = enum_value(result);
    msg.progress = progress;

    sendCommand->sendMavlinkMsgToRemoteDevice(msg);
  }

  uint16_t getNumOfSequences(uint32_t fileSize) {
    mavlink::dji_icg::msg::DJI_FTP msg;
    uint16_t payloadLength = msg.payload.size();
    return static_cast<uint16_t>(
        ceil((double)fileSize / (double)payloadLength));
  }

  /**
   * @brief handleDjiFtpRequest Handles the request and returns an ACK
   * @param mavMsg
   * @param msg
   */
  void handleDjiFtpRequest(const mavlink::mavlink_message_t *mavMsg,
                           mavlink::dji_icg::msg::DJI_FTP_REQUEST &msg) {
    DJI_FTP_REQUEST_RESULT result;

    ROS_INFO("Received DJI_FTP_REQUEST.");

    currentFileId = msg.file_id;
    currentFileSize = msg.file_size;
    currentNumOfSequences = getNumOfSequences(msg.file_size);
    ROS_INFO("Num of sequences to receive: %d", currentNumOfSequences);

    receivedSequences.clear();
    receivedSequences.resize(currentNumOfSequences, false);

    currentData.clear();
    currentData.resize(currentFileSize, 0);

    char *fileNameChar = reinterpret_cast<char *>(msg.file_name.data());
    std::string fileName(fileNameChar);
    currentFileName = fileName;

    result = DJI_FTP_REQUEST_RESULT::ACCEPTED;
    sendAck(msg.file_id, result);
  }

  void sendFtpRequestResend(uint8_t fileId, uint16_t sequenceStart,
                            uint16_t sequenceEnd) {
    mavlink::dji_icg::msg::DJI_FTP_REQUEST_RESEND msg;
    msg.target_system = GROUND_STATION_ID;
    msg.file_id = fileId;
    msg.sequence_start = sequenceStart;
    msg.sequence_end = sequenceEnd;

    sendCommand->sendMavlinkMsgToRemoteDevice(msg);
  }

  void prepareFtpRequestResend() {
    uint16_t sequenceStart = 0;
    bool foundSequenceStart = false;
    // Initialize to this value, in case the sequences are at the end
    uint16_t sequenceEnd = receivedSequences.size() - 1;
    for (uint16_t i = 0; i < receivedSequences.size(); i++) {
      if (foundSequenceStart == false && receivedSequences.at(i) == false) {
        sequenceStart = i;
        foundSequenceStart = true;
      }
      if (foundSequenceStart && receivedSequences.at(i) == true) {
        sequenceEnd = i - 1;
        break;
      }
    }

    sendFtpRequestResend(currentFileId, sequenceStart, sequenceEnd);

    // Start the first timeout for files
    restartDjiFtpTimeout();
  }

  void waitForNewFtp() {
    std::chrono::milliseconds span(TIMEOUT_MILISECS);

    // This lock creation has to be always done
    std::mutex mtx;
    std::unique_lock<std::mutex> lock(mtx);

    if (ftpConditionVar.wait_for(lock, span) == std::cv_status::timeout) {
      // A timeout happened, start requesting individual parts of the file
      prepareFtpRequestResend();
    }
  }

  void restartDjiFtpTimeout() {
    // Start the timeout event
    ftpTimeoutThread = std::unique_ptr<std::thread>(
        new std::thread(&DjiFtpPlugin::waitForNewFtp, this));
  }

  void endDjiFtpTimeout() {
    // release the previous lock in the timeout event
    ftpConditionVar.notify_all();

    // This join() seems to be buggy.
    if (ftpTimeoutThread != nullptr && ftpTimeoutThread->joinable()) {
      // Wait for the thread to end
      ftpTimeoutThread->join();
    }
  }

  void handleDjiFtp(const mavlink::mavlink_message_t *mavMsg,
                    mavlink::dji_icg::msg::DJI_FTP &msg) {
    if (currentFileId != msg.file_id) {
      ROS_WARN("Received FTP with different file_id!");
      return;
    }

    ROS_INFO("Received DJI_FTP, sequence=%d", msg.sequence);
    // Stop the previous timeout event
    endDjiFtpTimeout();

    // Register the receiving of the sequence
    receivedSequences.at(msg.sequence) = true;

    // Write to the correct location in the buffer
    uint32_t bytesPosition = msg.sequence * msg.payload.size();

    //ROS_INFO("Saving the current chunk of data.");
    std::copy(std::begin(msg.payload), std::end(msg.payload),
              std::begin(currentData) + bytesPosition);

    if (!std::all_of(receivedSequences.begin(), receivedSequences.end(),
                     [](bool x) { return x; })) {
      // We haven't yet received all packets
      restartDjiFtpTimeout();
    } else {
      // We have all the packets and we may now end
      ROS_INFO("File transfer complete.");
      ROS_INFO("Creating new file: %s", currentFileName.c_str());
      std::ofstream androidFile;
      androidFile.open(currentFileName, std::ios::out | std::ios::binary);

      char *dataChar = reinterpret_cast<char *>(currentData.data());
      ROS_INFO("Writing to the file.");
      androidFile.write(dataChar, currentData.size());

      androidFile.close();
      // This sends the confirmation message
      sendFtpRequestResend(msg.file_id, 0xffff, 0xffff);
    }
  }
};
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(icg_plugins::DjiFtpPlugin, icg_plugins::PluginBase)
