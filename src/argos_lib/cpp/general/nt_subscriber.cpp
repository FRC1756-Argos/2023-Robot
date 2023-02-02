/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "argos_lib/general/nt_subscriber.h"

#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

#include <span>

using argos_lib::NTSubscriber;

NTSubscriber::NTSubscriber(const std::string& tableName)
    : m_pntTable{nt::NetworkTableInstance::GetDefault().GetTable(tableName)} {}

void NTSubscriber::AddMonitor(const std::string& keyName,
                              std::function<void(double)> onUpdateCallback,
                              const double defaultValue,
                              const bool forceUpdate) {
  if (forceUpdate) {
    m_pntTable->PutNumber(keyName, defaultValue);
  } else {
    m_pntTable->SetDefaultNumber(keyName, defaultValue);
  }

  auto subscriber = m_pntTable->GetDoubleTopic(keyName).Subscribe(defaultValue);
  m_pntTable->GetInstance().AddListener(
      {{keyName}}, NT_EventFlags::NT_EVENT_VALUE_ALL, [onUpdateCallback](const nt::Event& e) {
        onUpdateCallback(e.GetValueEventData()->value.GetDouble());
      });
}
