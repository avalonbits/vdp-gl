/*
  Created by Fabrizio Di Vittorio (fdivitto2013@gmail.com) - <http://www.fabgl.com>
  Copyright (c) 2019-2022 Fabrizio Di Vittorio.
  All rights reserved.


* Please contact fdivitto2013@gmail.com if you need a commercial license.


* This library and related software is available under GPL v3.

  FabGL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  FabGL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with FabGL.  If not, see <http://www.gnu.org/licenses/>.
 */



#include <alloca.h>
#include <stdarg.h>
#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "soc/i2s_struct.h"
#include "soc/i2s_reg.h"
#include "driver/periph_ctrl.h"
#include "soc/rtc.h"
#include "esp_spi_flash.h"

#include "fabutils.h"
#include "vgacontroller.h"
#include "devdrivers/swgenerator.h"




#pragma GCC optimize ("O2")


namespace fabgl {





/*************************************************************************************/
/* VGAController definitions */


VGAController * VGAController::s_instance = nullptr;


VGAController::VGAController()
{
  s_instance = this;
}


void VGAController::init()
{
  VGABaseController::init();

  m_doubleBufferOverDMA = true;
}


void VGAController::suspendBackgroundPrimitiveExecution()
{
  VGABaseController::suspendBackgroundPrimitiveExecution();
  if (m_primitiveProcessingSuspended == 1) {
    I2S1.int_clr.val     = 0xFFFFFFFF;
    I2S1.int_ena.out_eof = 0;
  }
}


void VGAController::resumeBackgroundPrimitiveExecution()
{
  VGABaseController::resumeBackgroundPrimitiveExecution();
  if (m_primitiveProcessingSuspended == 0) {
    if (m_isr_handle == nullptr)
      esp_intr_alloc(ETS_I2S1_INTR_SOURCE, ESP_INTR_FLAG_LEVEL1, VSyncInterrupt, this, &m_isr_handle);
    I2S1.int_clr.val     = 0xFFFFFFFF;
    I2S1.int_ena.out_eof = 1;
  }
}


void VGAController::allocateViewPort()
{
  VGABaseController::allocateViewPort(MALLOC_CAP_DMA, m_viewPortWidth);
}


void VGAController::setResolution(VGATimings const& timings, int viewPortWidth, int viewPortHeight, bool doubleBuffered)
{
  VGABaseController::setResolution(timings, viewPortWidth, viewPortHeight, doubleBuffered);

  // fill view port
  for (int i = 0; i < m_viewPortHeight; ++i)
    fill(m_viewPort[i], 0, m_viewPortWidth, 0, 0, 0, false, false);

  // number of microseconds usable in VSynch ISR
  m_maxVSyncISRTime = ceil(1000000.0 / m_timings.frequency * m_timings.scanCount * m_HLineSize * (m_timings.VSyncPulse + m_timings.VBackPorch + m_timings.VFrontPorch + m_viewPortRow));

  startGPIOStream();
  resumeBackgroundPrimitiveExecution();
}


void VGAController::onSetupDMABuffer(lldesc_t volatile * buffer, bool isStartOfVertFrontPorch, int scan, bool isVisible, int visibleRow)
{
  // generate interrupt at the beginning of vertical front porch
  if (isStartOfVertFrontPorch)
    buffer->eof = 1;
}


void IRAM_ATTR VGAController::VSyncInterrupt(void * arg)
{
  if (I2S1.int_st.out_eof) {
    auto VGACtrl = (VGAController*)arg;
    int64_t startTime = VGACtrl->backgroundPrimitiveTimeoutEnabled() ? esp_timer_get_time() : 0;
    Rect updateRect = Rect(SHRT_MAX, SHRT_MAX, SHRT_MIN, SHRT_MIN);
    do {
      Primitive prim;
      if (VGACtrl->getPrimitiveISR(&prim) == false)
        break;

      VGACtrl->execPrimitive(prim, updateRect, true);

      if (VGACtrl->m_primitiveProcessingSuspended)
        break;

    } while (!VGACtrl->backgroundPrimitiveTimeoutEnabled() || (startTime + VGACtrl->m_maxVSyncISRTime > esp_timer_get_time()));
    VGACtrl->showSprites(updateRect);
  }
  I2S1.int_clr.val = I2S1.int_st.val;
}


std::function<uint8_t(RGB888 const &)> IRAM_ATTR VGAController::getPixelLambda(PaintMode mode)
{
  switch (mode) {
    case PaintMode::XOR:
      return [&] (RGB888 const & color) { return preparePixel(color) & 63; };
    default: // PaintMode::Set, et al
      return [&] (RGB888 const & color) { return preparePixel(color); };
  }
}


std::function<void(int X, int Y, uint8_t pattern)> IRAM_ATTR VGAController::setPixelLambda(PaintMode mode)
{
  switch (mode) {
    case PaintMode::Set:
      return [&] (int X, int Y, uint8_t pattern) { VGA_PIXEL(X, Y) = pattern; };
    case PaintMode::OR:
      return [&] (int X, int Y, uint8_t pattern) { VGA_PIXEL(X, Y) |= pattern; };
    case PaintMode::ORNOT:
      return [&] (int X, int Y, uint8_t pattern) { VGA_PIXEL(X, Y) |= (~pattern) & 63; };
    case PaintMode::AND:
      return [&] (int X, int Y, uint8_t pattern) { VGA_PIXEL(X, Y) &= pattern; };
    case PaintMode::ANDNOT:
      return [&] (int X, int Y, uint8_t pattern) { VGA_PIXEL(X, Y) &= ((~pattern) & 63) | m_HVSync; };
    case PaintMode::XOR:
      return [&] (int X, int Y, uint8_t pattern) { VGA_PIXEL(X, Y) ^= pattern; };
    case PaintMode::Invert:
      return [&] (int X, int Y, uint8_t pattern) { VGA_INVERT_PIXEL(X, Y); };
    default:  // PaintMode::NoOp
      return [&] (int X, int Y, uint8_t pattern) { return; };
  }
}


std::function<void(uint8_t * row, int x, uint8_t pattern)> IRAM_ATTR VGAController::setRowPixelLambda(PaintMode mode)
{
  switch (mode) {
    case PaintMode::Set:
      return [&] (uint8_t * row, int x, uint8_t pattern) { VGA_PIXELINROW(row, x) = pattern; };
    case PaintMode::OR:
      return [&] (uint8_t * row, int x, uint8_t pattern) { VGA_PIXELINROW(row, x) |= pattern; };
    case PaintMode::ORNOT:
      return [&] (uint8_t * row, int x, uint8_t pattern) { VGA_PIXELINROW(row, x) |= (~pattern & 63); };
    case PaintMode::AND:
      return [&] (uint8_t * row, int x, uint8_t pattern) { VGA_PIXELINROW(row, x) &= pattern; };
    case PaintMode::ANDNOT:
      return [&] (uint8_t * row, int x, uint8_t pattern) { VGA_PIXELINROW(row, x) &= (~pattern & 63) | m_HVSync; };
    case PaintMode::XOR:
      return [&] (uint8_t * row, int x, uint8_t pattern) { VGA_PIXELINROW(row, x) ^= pattern & 63; };
    case PaintMode::Invert:
      return [&] (uint8_t * row, int x, uint8_t pattern) { auto px = &VGA_PIXELINROW(row, x); *px = ~(*px ^ VGA_SYNC_MASK); };
    default:  // PaintMode::NoOp
      return [&] (uint8_t * row, int x, uint8_t pattern) { return; };
  
  }
}


std::function<void(int Y, int X1, int X2, uint8_t pattern)> IRAM_ATTR VGAController::fillRowLambda(PaintMode mode)
{
  switch (mode) {
    case PaintMode::Set:
      return [&] (int Y, int X1, int X2, uint8_t pattern) { rawFillRow(Y, X1, X2, pattern); };
    case PaintMode::OR:
      return [&] (int Y, int X1, int X2, uint8_t pattern) { rawORRow(Y, X1, X2, pattern); };
    case PaintMode::ORNOT:
      return [&] (int Y, int X1, int X2, uint8_t pattern) { rawORRow(Y, X1, X2, (~pattern & 63)); };
    case PaintMode::AND:
      return [&] (int Y, int X1, int X2, uint8_t pattern) { rawANDRow(Y, X1, X2, pattern); };
    case PaintMode::ANDNOT:
      return [&] (int Y, int X1, int X2, uint8_t pattern) { rawANDRow(Y, X1, X2, (~pattern & 63) | m_HVSync); };
    case PaintMode::XOR:
      return [&] (int Y, int X1, int X2, uint8_t pattern) { rawXORRow(Y, X1, X2, pattern); };
    case PaintMode::Invert:
      return [&] (int Y, int X1, int X2, uint8_t pattern) { rawInvertRow(Y, X1, X2); };
    default:  // PaintMode::NoOp
      return [&] (int Y, int X1, int X2, uint8_t pattern) { return; };
  }
}



void IRAM_ATTR VGAController::setPixelAt(PixelDesc const & pixelDesc, Rect & updateRect)
{
  auto paintMode = paintState().paintOptions.mode;
  genericSetPixelAt(pixelDesc, updateRect, getPixelLambda(paintMode), setPixelLambda(paintMode));
}



// coordinates are absolute values (not relative to origin)
// line clipped on current absolute clipping rectangle
void IRAM_ATTR VGAController::absDrawLine(int X1, int Y1, int X2, int Y2, RGB888 color)
{
  auto paintMode = paintState().paintOptions.NOT ? PaintMode::NOT : paintState().paintOptions.mode;
  genericAbsDrawLine(X1, Y1, X2, Y2, color,
                     getPixelLambda(paintMode),
                     fillRowLambda(paintMode),
                     setPixelLambda(paintMode)
                     );
}


// parameters not checked
void IRAM_ATTR VGAController::fillRow(int y, int x1, int x2, RGB888 color)
{
  // This version, passing an RGB888 color, is only used by shape drawing methods,
  // so we will pick fill method based on paint mode
  auto paintMode = paintState().paintOptions.mode;
  auto getPixel = getPixelLambda(paintMode);
  auto pixel = getPixel(color);
  auto fill = fillRowLambda(paintMode);
  fill(y, x1, x2, pixel);
}


// parameters not checked
void IRAM_ATTR VGAController::rawFillRow(int y, int x1, int x2, uint8_t pattern)
{
  auto row = m_viewPort[y];
  // fill first bytes before full 32 bits word
  int x = x1;
  for (; x <= x2 && (x & 3) != 0; ++x) {
    VGA_PIXELINROW(row, x) = pattern;
  }
  // fill whole 32 bits words (don't care about VGA_PIXELINROW adjusted alignment)
  if (x <= x2) {
    int sz = (x2 & ~3) - x;
    memset((void*)(row + x), pattern, sz);
    x += sz;
  }
  // fill last unaligned bytes
  for (; x <= x2; ++x) {
    VGA_PIXELINROW(row, x) = pattern;
  }
}


// parameters not checked
void IRAM_ATTR VGAController::rawORRow(int y, int x1, int x2, uint8_t pattern)
{
  auto row = m_viewPort[y];
  // naive implementation - just do whole row iteratively
  for (int x = x1; x <= x2; ++x) {
    VGA_PIXELINROW(row, x) |= pattern;
  }
}


// parameters not checked
void IRAM_ATTR VGAController::rawANDRow(int y, int x1, int x2, uint8_t pattern)
{
  auto row = m_viewPort[y];
  // naive implementation - just do whole row iteratively
  for (int x = x1; x <= x2; ++x) {
    VGA_PIXELINROW(row, x) &= pattern;
  }
}


// parameters not checked
void IRAM_ATTR VGAController::rawXORRow(int y, int x1, int x2, uint8_t pattern)
{
  auto row = m_viewPort[y];
  // naive implementation - just do whole row iteratively
  for (int x = x1; x <= x2; ++x) {
    VGA_PIXELINROW(row, x) ^= pattern;
  }
}


// parameters not checked
void IRAM_ATTR VGAController::rawInvertRow(int y, int x1, int x2)
{
  auto row = m_viewPort[y];
  for (int x = x1; x <= x2; ++x) {
    uint8_t * px = (uint8_t*) &VGA_PIXELINROW(row, x);
    *px = m_HVSync | ~(*px);
  }
}


// swaps all pixels inside the range x1...x2 of yA and yB
// parameters not checked
void IRAM_ATTR VGAController::swapRows(int yA, int yB, int x1, int x2)
{
  uint8_t * rowA = (uint8_t*) m_viewPort[yA];
  uint8_t * rowB = (uint8_t*) m_viewPort[yB];
  // swap first bytes before full 32 bits word
  int x = x1;
  for (; x <= x2 && (x & 3) != 0; ++x)
    tswap(VGA_PIXELINROW(rowA, x), VGA_PIXELINROW(rowB, x));
  // swap whole 32 bits words (don't care about VGA_PIXELINROW adjusted alignment)
  uint32_t * a = (uint32_t*)(rowA + x);
  uint32_t * b = (uint32_t*)(rowB + x);
  for (int right = (x2 & ~3); x < right; x += 4)
    tswap(*a++, *b++);
  // swap last unaligned bytes
  for (x = (x2 & ~3); x <= x2; ++x)
    tswap(VGA_PIXELINROW(rowA, x), VGA_PIXELINROW(rowB, x));
}


void IRAM_ATTR VGAController::drawEllipse(Size const & size, Rect & updateRect)
{
  auto mode = paintState().paintOptions.mode;
  genericDrawEllipse(size, updateRect, getPixelLambda(mode), setPixelLambda(mode));
}


void IRAM_ATTR VGAController::drawArc(Rect const & rect, Rect & updateRect)
{
  auto mode = paintState().paintOptions.mode;
  genericDrawArc(rect, updateRect, getPixelLambda(mode), setPixelLambda(mode));
}


void IRAM_ATTR VGAController::fillSegment(Rect const & rect, Rect & updateRect)
{
  auto mode = paintState().paintOptions.mode;
  genericFillSegment(rect, updateRect, getPixelLambda(mode), fillRowLambda(mode));
}


void IRAM_ATTR VGAController::fillSector(Rect const & rect, Rect & updateRect)
{
  auto mode = paintState().paintOptions.mode;
  genericFillSector(rect, updateRect, getPixelLambda(mode), fillRowLambda(mode));
}


void IRAM_ATTR VGAController::clear(Rect & updateRect)
{
  hideSprites(updateRect);
  uint8_t pattern = preparePixel(getActualBrushColor());
  for (int y = 0; y < m_viewPortHeight; ++y)
    memset((uint8_t*) m_viewPort[y], pattern, m_viewPortWidth);
}


// scroll < 0 -> scroll UP
// scroll > 0 -> scroll DOWN
// Speciying horizontal scrolling region slow-down scrolling!
void IRAM_ATTR VGAController::VScroll(int scroll, Rect & updateRect)
{
  genericVScroll(scroll, updateRect,
                 [&] (int yA, int yB, int x1, int x2)      { swapRows(yA, yB, x1, x2); },              // swapRowsCopying
                 [&] (int yA, int yB)                      { tswap(m_viewPort[yA], m_viewPort[yB]); }, // swapRowsPointers
                 [&] (int y, int x1, int x2, RGB888 color) { rawFillRow(y, x1, x2, preparePixel(color)); }         // rawFillRow
                );

  if (scroll != 0) {
    // reassign DMA pointers
    int viewPortBuffersPerLine = 0;
    int linePos = 1;
    switch (m_timings.HStartingBlock) {
      case VGAScanStart::FrontPorch:
        // FRONTPORCH -> SYNC -> BACKPORCH -> VISIBLEAREA
        viewPortBuffersPerLine = (m_viewPortCol + m_viewPortWidth) < m_timings.HVisibleArea ? 3 : 2;
        break;
      case VGAScanStart::Sync:
        // SYNC -> BACKPORCH -> VISIBLEAREA -> FRONTPORCH
        viewPortBuffersPerLine = 3;
        break;
      case VGAScanStart::BackPorch:
        // BACKPORCH -> VISIBLEAREA -> FRONTPORCH -> SYNC
        viewPortBuffersPerLine = 3;
        break;
      case VGAScanStart::VisibleArea:
        // VISIBLEAREA -> FRONTPORCH -> SYNC -> BACKPORCH
        viewPortBuffersPerLine = m_viewPortCol > 0 ? 3 : 2;
        linePos = m_viewPortCol > 0 ? 1 : 0;
        break;
    }
    const int Y1 = paintState().scrollingRegion.Y1;
    const int Y2 = paintState().scrollingRegion.Y2;
    for (int i = Y1, idx = Y1 * m_timings.scanCount; i <= Y2; ++i)
      for (int scan = 0; scan < m_timings.scanCount; ++scan, ++idx)
        setDMABufferView(m_viewPortRow * m_timings.scanCount + idx * viewPortBuffersPerLine + linePos, i, scan, m_viewPort, false);
  }
}


// Scrolling by 1, 2, 3 and 4 pixels is optimized. Also scrolling multiples of 4 (8, 16, 24...) is optimized.
// Scrolling by other values requires up to three steps (scopose scrolling by 1, 2, 3 or 4): for example scrolling by 5 is scomposed to 4 and 1, scrolling
// by 6 is 4 + 2, etc.
// Horizontal scrolling region start and size (X2-X1+1) must be aligned to 32 bits, otherwise the unoptimized (very slow) version is used.
void IRAM_ATTR VGAController::HScroll(int scroll, Rect & updateRect)
{
  hideSprites(updateRect);
  uint8_t pattern8   = preparePixel(getActualBrushColor());
  uint16_t pattern16 = pattern8 << 8 | pattern8;
  uint32_t pattern32 = pattern16 << 16 | pattern16;

  int Y1 = paintState().scrollingRegion.Y1;
  int Y2 = paintState().scrollingRegion.Y2;
  int X1 = paintState().scrollingRegion.X1;
  int X2 = paintState().scrollingRegion.X2;

  int width   = X2 - X1 + 1;
  int width32 = width >> 2;
  bool HScrolllingRegionAligned = ((X1 & 3) == 0 && (width & 3) == 0);

  if (scroll < 0) {
    // scroll left
    for (int y = Y1; y <= Y2; ++y) {
      if (HScrolllingRegionAligned) {
        // aligned horizontal scrolling region, fast version
        uint8_t * row = (uint8_t*) (m_viewPort[y] + X1);
        for (int s = -scroll; s > 0;) {
          if (s >= 4) {
            // scroll left 4, 8 12, etc.. pixels moving 32 bit words
            uint8_t * w = row;
            int sz = (s & ~3) >> 2;
            for (int i = 0; i < width32 - sz; ++i, w += 4)
              *((uint32_t*)w) = *((uint32_t*)w + sz);
            for (int i = tmax(0, width32 - sz); i < width32; ++i, w += 4)
              *((uint32_t*)w) = pattern32;
            s -= (s & ~3);
          } else if ((s & 3) == 3) {
            // scroll left 3 pixels swapping 8 bit words
            uint8_t * b = row;
            for (int i = 1; i < width32; ++i, b += 4) {
              b[2] = b[1];
              b[1] = b[4];
              b[0] = b[7];
              b[3] = b[6];
            }
            b[2] = b[1];
            b[1] = b[0] = b[3] = pattern8;
            s -= 3;
          } else if (s & 2) {
            // scroll left 2 pixels swapping 16 bit words
            uint16_t * w = (uint16_t*) row;
            for (int i = 1; i < width32; ++i, w += 2) {
              w[1] = w[0];
              w[0] = w[3];
            }
            w[1] = w[0];
            w[0] = pattern16;
            s -= 2;
          } else if (s & 1) {
            // scroll left 1 pixel by rotating 32 bit words
            uint8_t * w = row;
            for (int i = 1; i < width32; ++i, w += 4) {
              *((uint32_t*)w) = *((uint32_t*)w) >> 8 | *((uint32_t*)w) << 24;
              w[1] = w[6];
            }
            *((uint32_t*)w) = *((uint32_t*)w) >> 8 | *((uint32_t*)w) << 24;
            w[1] = pattern8;
            --s;
          }
        }
      } else {
        // unaligned horizontal scrolling region, fallback to slow version
        uint8_t * row = (uint8_t*) m_viewPort[y];
        for (int x = X1; x <= X2 + scroll; ++x)
          VGA_PIXELINROW(row, x) = VGA_PIXELINROW(row, x - scroll);
        // fill right area with brush color
        for (int x = X2 + 1 + scroll; x <= X2; ++x)
          VGA_PIXELINROW(row, x) = pattern8;
      }
    }
  } else if (scroll > 0) {
    // scroll right
    for (int y = Y1; y <= Y2; ++y) {
      if (HScrolllingRegionAligned) {
        // aligned horizontal scrolling region, fast version
        uint8_t * row = (uint8_t*) (m_viewPort[y] + X1);
        for (int s = scroll; s > 0;) {
          if (s >= 4) {
            // scroll right 4, 8 12, etc.. pixels moving 32 bit words
            int sz = (s & ~3) >> 2;
            uint8_t * w = row + width - 4;
            for (int i = 0; i < width32 - sz; ++i, w -= 4)
              *((uint32_t*)w) = *((uint32_t*)w - sz);
            for (int i = tmax(0, width32 - sz); i < width32; ++i, w -= 4)
              *((uint32_t*)w) = pattern32;
            s -= (s & ~3);
          } else if ((s & 3) == 3) {
            // scroll right 3 pixels swapping 8 bit words
            uint8_t * b = row + width - 4;
            for (int i = 1; i < width32; ++i, b -= 4) {
              b[0] = b[-3];
              b[1] = b[2];
              b[2] = b[-1];
              b[3] = b[-4];
            }
            b[1] = b[2];
            b[0] = b[2] = b[3] = pattern8;
            s -= 3;
          } else if (s & 2) {
            // scroll right 2 pixels swapping 16 bit words
            uint16_t * w = (uint16_t*) (row + width - 4);
            for (int i = 1; i < width32; ++i, w -= 2) {
              w[0] = w[1];
              w[1] = w[-2];
            }
            w[0] = w[1];
            w[1] = pattern16;
            s -= 2;
          } else if (s & 1) {
            // scroll right 1 pixel by rotating 32 bit words
            uint8_t * w = row + width - 4;
            for (int i = 1; i < width32; ++i, w -= 4) {
              *((uint32_t*)w) = *((uint32_t*)w) << 8 | *((uint32_t*)w) >> 24;
              w[2] = w[-3];
            }
            *((uint32_t*)w) = *((uint32_t*)w) << 8 | *((uint32_t*)w) >> 24;
            w[2] = pattern8;
            --s;
          }
        }
      } else {
        // unaligned horizontal scrolling region, fallback to slow version
        uint8_t * row = (uint8_t*) m_viewPort[y];
        for (int x = X2 - scroll; x >= X1; --x)
          VGA_PIXELINROW(row, x + scroll) = VGA_PIXELINROW(row, x);
        // fill left area with brush color
        for (int x = X1; x < X1 + scroll; ++x)
          VGA_PIXELINROW(row, x) = pattern8;
      }
    }

  }
}


void IRAM_ATTR VGAController::drawGlyph(Glyph const & glyph, GlyphOptions glyphOptions, RGB888 penColor, RGB888 brushColor, Rect & updateRect)
{
  auto mode = paintState().paintOptions.mode;
  auto getPixel = getPixelLambda(mode);
  auto setRowPixel = setRowPixelLambda(mode);
  genericDrawGlyph(glyph, glyphOptions, penColor, brushColor, updateRect,
                   getPixel,
                   [&] (int y) { return (uint8_t*) m_viewPort[y]; },
                   setRowPixel
                  );
}


void IRAM_ATTR VGAController::invertRect(Rect const & rect, Rect & updateRect)
{
  genericInvertRect(rect, updateRect,
                    [&] (int Y, int X1, int X2) { rawInvertRow(Y, X1, X2); }
                   );
}


void IRAM_ATTR VGAController::swapFGBG(Rect const & rect, Rect & updateRect)
{
  genericSwapFGBG(rect, updateRect,
                  [&] (RGB888 const & color)                  { return preparePixel(color); },
                  [&] (int y)                                 { return (uint8_t*) m_viewPort[y]; },
                  [&] (uint8_t * row, int x)                  { return VGA_PIXELINROW(row, x); },
                  [&] (uint8_t * row, int x, uint8_t pattern) { VGA_PIXELINROW(row, x) = pattern; }
                 );
}


// Slow operation!
// supports overlapping of source and dest rectangles
void IRAM_ATTR VGAController::copyRect(Rect const & source, Rect & updateRect)
{
  genericCopyRect(source, updateRect,
                  [&] (int y)                                 { return (uint8_t*) m_viewPort[y]; },
                  [&] (uint8_t * row, int x)                  { return VGA_PIXELINROW(row, x); },
                  [&] (uint8_t * row, int x, uint8_t pattern) { VGA_PIXELINROW(row, x) = pattern; }
                 );
}


// no bounds check is done!
void VGAController::readScreen(Rect const & rect, RGB888 * destBuf)
{
  for (int y = rect.Y1; y <= rect.Y2; ++y) {
    uint8_t * row = (uint8_t*) m_viewPort[y];
    for (int x = rect.X1; x <= rect.X2; ++x, ++destBuf) {
      uint8_t rawpix = VGA_PIXELINROW(row, x);
      *destBuf = RGB888((rawpix & 3) * 85, ((rawpix >> 2) & 3) * 85, ((rawpix >> 4) & 3) * 85);
    }
  }
}


// no bounds check is done!
void VGAController::readScreen(Rect const & rect, RGB222 * destBuf)
{
  uint8_t * dbuf = (uint8_t*) destBuf;
  for (int y = rect.Y1; y <= rect.Y2; ++y) {
    uint8_t * row = (uint8_t*) m_viewPort[y];
    for (int x = rect.X1; x <= rect.X2; ++x, ++dbuf)
      *dbuf = VGA_PIXELINROW(row, x) & ~VGA_SYNC_MASK;
  }
}


// no bounds check is done!
void VGAController::writeScreen(Rect const & rect, RGB222 * srcBuf)
{
  uint8_t * sbuf = (uint8_t*) srcBuf;
  for (int y = rect.Y1; y <= rect.Y2; ++y) {
    uint8_t * row = (uint8_t*) m_viewPort[y];
    for (int x = rect.X1; x <= rect.X2; ++x, ++sbuf)
      VGA_PIXELINROW(row, x) = *sbuf | m_HVSync;
  }
}


void IRAM_ATTR VGAController::rawDrawBitmap_Native(int destX, int destY, Bitmap const * bitmap, int X1, int Y1, int XCount, int YCount)
{
  genericRawDrawBitmap_Native(destX, destY, (uint8_t*) bitmap->data, bitmap->width, X1, Y1, XCount, YCount,
                              [&] (int y) { return (uint8_t*) m_viewPort[y]; },   // rawGetRow
                              [&] (uint8_t * row, int x, uint8_t src) { VGA_PIXELINROW(row, x) = src; }  // rawSetPixelInRow
                             );
}


void IRAM_ATTR VGAController::rawDrawBitmap_Mask(int destX, int destY, Bitmap const * bitmap, void * saveBackground, int X1, int Y1, int XCount, int YCount)
{
  auto paintMode = paintState().paintOptions.mode;
  auto setRowPixel = setRowPixelLambda(paintMode);
  auto getPixel = getPixelLambda(paintMode);
  auto pattern = getPixel(paintState().paintOptions.swapFGBG ? paintState().penColor : bitmap->foregroundColor);
  genericRawDrawBitmap_Mask(destX, destY, bitmap, (uint8_t*)saveBackground, X1, Y1, XCount, YCount,
                            [&] (int y)                { return (uint8_t*) m_viewPort[y]; },  // rawGetRow
                            [&] (uint8_t * row, int x) { return VGA_PIXELINROW(row, x); },    // rawGetPixelInRow
                            [&] (uint8_t * row, int x) { setRowPixel(row, x, pattern); }      // rawSetPixelInRow
                           );
}


void IRAM_ATTR VGAController::rawDrawBitmap_RGBA2222(int destX, int destY, Bitmap const * bitmap, void * saveBackground, int X1, int Y1, int XCount, int YCount)
{
  auto paintMode = paintState().paintOptions.mode;
  auto setRowPixel = setRowPixelLambda(paintMode);

  if (paintState().paintOptions.swapFGBG) {
    // used for bitmap plots to indicate drawing with BG color instead of bitmap color
    auto bg = preparePixel(paintState().penColor);
    genericRawDrawBitmap_RGBA2222(destX, destY, bitmap, (uint8_t*)saveBackground, X1, Y1, XCount, YCount,
                                  [&] (int y)                             { return (uint8_t*) m_viewPort[y]; },  // rawGetRow
                                  [&] (uint8_t * row, int x)              { return VGA_PIXELINROW(row, x); },    // rawGetPixelInRow
                                  [&] (uint8_t * row, int x, uint8_t src) { setRowPixel(row, x, bg); }           // rawSetPixelInRow
                                );
    return;
  }

  genericRawDrawBitmap_RGBA2222(destX, destY, bitmap, (uint8_t*)saveBackground, X1, Y1, XCount, YCount,
                                [&] (int y)                             { return (uint8_t*) m_viewPort[y]; },              // rawGetRow
                                [&] (uint8_t * row, int x)              { return VGA_PIXELINROW(row, x); },                // rawGetPixelInRow
                                [&] (uint8_t * row, int x, uint8_t src) { setRowPixel(row, x, m_HVSync | (src & 0x3f)); }  // rawSetPixelInRow
                               );
}


void IRAM_ATTR VGAController::rawDrawBitmap_RGBA8888(int destX, int destY, Bitmap const * bitmap, void * saveBackground, int X1, int Y1, int XCount, int YCount)
{
  auto paintMode = paintState().paintOptions.mode;
  auto setRowPixel = setRowPixelLambda(paintMode);

  if (paintState().paintOptions.swapFGBG) {
    // used for bitmap plots to indicate drawing with BG color instead of bitmap color
    auto bg = preparePixel(paintState().penColor);
    genericRawDrawBitmap_RGBA8888(destX, destY, bitmap, (uint8_t*)saveBackground, X1, Y1, XCount, YCount,
                                  [&] (int y)                                      { return (uint8_t*) m_viewPort[y]; },  // rawGetRow
                                  [&] (uint8_t * row, int x)                       { return VGA_PIXELINROW(row, x); },    // rawGetPixelInRow
                                  [&] (uint8_t * row, int x, RGBA8888 const & src) { setRowPixel(row, x, bg); }           // rawSetPixelInRow
                                  );
    return;
  }

  genericRawDrawBitmap_RGBA8888(destX, destY, bitmap, (uint8_t*)saveBackground, X1, Y1, XCount, YCount,
                                 [&] (int y)                                      { return (uint8_t*) m_viewPort[y]; },   // rawGetRow
                                 [&] (uint8_t * row, int x)                       { return VGA_PIXELINROW(row, x); },     // rawGetPixelInRow
                                 [&] (uint8_t * row, int x, RGBA8888 const & src) { setRowPixel(row, x, m_HVSync | (src.R >> 6) | (src.G >> 6 << 2) | (src.B >> 6 << 4)); }   // rawSetPixelInRow
                                );
}


void IRAM_ATTR VGAController::rawCopyToBitmap(int srcX, int srcY, int width, void * saveBuffer, int X1, int Y1, int XCount, int YCount)
{
  genericRawCopyToBitmap(srcX, srcY, width, (uint8_t*)saveBuffer, X1, Y1, XCount, YCount,
                        [&] (int y)                { return (uint8_t*) m_viewPort[y]; },  // rawGetRow
                        [&] (uint8_t * row, int x) { return 0xC0 | VGA_PIXELINROW(row, x); }     // rawGetPixelInRow
                      );
}


} // end of namespace
