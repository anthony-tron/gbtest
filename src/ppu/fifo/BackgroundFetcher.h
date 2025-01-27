#ifndef GBTEST_BACKGROUNDFETCHER_H
#define GBTEST_BACKGROUNDFETCHER_H

#include "Fetcher.h"
#include "PixelFIFO.h"

#include "../PPURegisters.h"
#include "../vram/VRAM.h"

namespace gbtest {

class BackgroundFetcher
        : public Fetcher {

public:
    BackgroundFetcher(const PPURegisters& ppuRegisters, const VRAM& vram, PixelFIFO& pixelFifo);
    ~BackgroundFetcher() override = default;

    void beginScanline() override;

    void executeState() override;

private:
    uint8_t m_currentTileNumber;
    uint16_t m_currentTileData;

    uint8_t m_fetcherX;
    bool m_scanlineBeginSkip;

}; // class BackgroundFetcher

} // namespace gbtest

#endif //GBTEST_BACKGROUNDFETCHER_H
