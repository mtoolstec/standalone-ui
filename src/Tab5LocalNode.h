#pragma once

#include "comms/IClientBase.h"

class Tab5LocalNode
{
  public:
    static Tab5LocalNode &instance();

    bool begin();
    void taskHandler();
    IClientBase &client();

  private:
    Tab5LocalNode();
    Tab5LocalNode(const Tab5LocalNode &) = delete;
    Tab5LocalNode &operator=(const Tab5LocalNode &) = delete;

    class Impl;
    Impl *impl;
};