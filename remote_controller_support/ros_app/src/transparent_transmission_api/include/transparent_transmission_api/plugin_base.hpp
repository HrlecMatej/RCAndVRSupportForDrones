/**
 * @brief MAVROS Plugin base
 * @file mavros_plugin.h
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 *  @brief MAVROS Plugin system
 */
/*
 * Copyright 2013,2014,2015 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#pragma once

#include <mavconn/interface.h>
#include <functional>
#include <tuple>
#include <vector>

#include <ros/ros.h>

#include "send_command.hpp"

namespace icg_plugins {

/**
 * @brief MAVROS Plugin base class
 */
class PluginBase {
 private:
  PluginBase(const PluginBase &) = delete;

 public:
  static const uint8_t GROUND_STATION_ID = 255;

  //! generic message handler callback
  using HandlerCb = mavconn::MAVConnInterface::ReceivedCb;
  //! Tuple: MSG ID, MSG NAME, message type into hash_code, message handler
  //! callback
  using HandlerInfo =
      std::tuple<mavlink::msgid_t, const char *, size_t, HandlerCb>;
  //! Subscriptions vector
  using Subscriptions = std::vector<HandlerInfo>;

  // pluginlib return boost::shared_ptr
  using Ptr = boost::shared_ptr<PluginBase>;
  using ConstPtr = boost::shared_ptr<PluginBase const>;

  virtual ~PluginBase(){};

  /**
   * @brief Plugin initializer
   *
   * @param[in] uas  @p UAS instance
   */

  std::shared_ptr<SendCommand> sendCommand;

  virtual void initialize(const std::shared_ptr<SendCommand> &sendComm) {
    sendCommand = sendComm;
    // transparentTranmissionApi = &ttApi;
  }

  /**
   * @brief Return vector of MAVLink message subscriptions (handlers)
   */
  virtual Subscriptions get_subscriptions() = 0;

 protected:
  /**
   * @brief Plugin constructor
   * Should not do anything before initialize()
   */
  PluginBase(){};

  // TransparentTransmissionAPI transparentTransmissionAPI;
  // TODO: filtered handlers

  /**
   * Make subscription to raw message.
   *
   * @param[in] id  message id
   * @param[in] fn  pointer to member function (handler)
   */
  template <class _C>
  HandlerInfo make_handler(const mavlink::msgid_t id,
                           void (_C::*fn)(const mavlink::mavlink_message_t *msg,
                                          const mavconn::Framing framing)) {
    auto bfn = std::bind(fn, static_cast<_C *>(this), std::placeholders::_1,
                         std::placeholders::_2);
    const auto type_hash_ = typeid(mavlink::mavlink_message_t).hash_code();

    return HandlerInfo{id, nullptr, type_hash_, bfn};
  }

  /**
   * Make subscription to message with automatic decoding.
   *
   * @param[in] fn  pointer to member function (handler)
   */
  template <class _C, class _T>
  HandlerInfo make_handler(void (_C::*fn)(const mavlink::mavlink_message_t *,
                                          _T &)) {
    auto bfn = std::bind(fn, static_cast<_C *>(this), std::placeholders::_1,
                         std::placeholders::_2);
    const auto id = _T::MSG_ID;
    const auto name = _T::NAME;
    const auto type_hash_ = typeid(_T).hash_code();

    return HandlerInfo{id, name, type_hash_,
                       [bfn](const mavlink::mavlink_message_t *msg,
                             const mavconn::Framing framing) {
                         if (framing != mavconn::Framing::ok) return;

                         mavlink::MsgMap map(msg);
                         _T obj;
                         obj.deserialize(map);

                         bfn(msg, obj);
                       }};
  }
};
}  // namespace icg_plugins
