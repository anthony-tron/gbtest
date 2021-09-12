#ifndef GBTEST_INTERRUPTCONTROLLER_H
#define GBTEST_INTERRUPTCONTROLLER_H

#include "../utils/Tickable.h"

namespace gbtest {

class InterruptController
        : public Tickable {

public:
    InterruptController();
    ~InterruptController() override = default;

    void setInterruptMasterEnable(bool interruptMasterEnable);
    [[nodiscard]] bool isInterruptMasterEnabled() const;

    void setDelayedInterruptEnableCountdown(int delayedInterruptEnableCountdown);
    void handleDelayedInterrupt();

    void tick() override;

private:
    bool m_interruptMasterEnable;
    int m_delayedInterruptEnableCountdown;

}; // class InterruptController

} // namespace InterruptController

#endif //GBTEST_INTERRUPTCONTROLLER_H