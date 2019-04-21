/*
//
// Copyright (c) 1998-2019 Joe Bertolami. All Right Reserved.
//
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions are met:
//
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistributions in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//   AND ANY EXPRESS OR IMPLIED WARRANTIES, CLUDG, BUT NOT LIMITED TO, THE
//   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//   ARE DISCLAIMED.  NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
//   LIABLE FOR ANY DIRECT, DIRECT, CIDENTAL, SPECIAL, EXEMPLARY, OR
//   CONSEQUENTIAL DAMAGES (CLUDG, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
//   GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSESS TERRUPTION)
//   HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER  CONTRACT, STRICT
//   LIABILITY, OR TORT (CLUDG NEGLIGENCE OR OTHERWISE) ARISG  ANY WAY  OF THE
//   USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Additional Information:
//
//   For more information, visit http://www.bertolami.com.
//
*/

#ifndef __BASE_WINDOW_H__
#define __BASE_WINDOW_H__

#include <string>
#include <vector>
#include "base_types.h"

#define BASE_WINDOW_STYLE_SIZABLE (0x00000001)
#define BASE_WINDOW_STYLE_FULLSCREEN (0x00000002)
#define BASE_WINDOW_STYLE_CURSOR_HIDDEN (0x00000004)
#define BASE_WINDOW_STYLE_WINDOW_HIDDEN (0x00000008)

namespace base {

// We encapsulate input device formats under two metaphores: switches and
// targets. Switches represent simple push-button style input. A switch may be
// in a state of on (enabled) or off (disabled). The most common uses for
// switches are keyboard buttons and gamepad buttons.
//
// Targets extend the switch by adding the notion of location. Thus a target is
// simply a location on screen plus state information for a single switch. The
// most common uses for targets are mouses, touches, and analog sticks on a
// gamepad.

const uint32 kInputMouseMoveIndex = 0x100000;
const uint32 kInputMouseWheelIndex = 0x100001;
const uint32 kInputMouseLeftButtonIndex = 0x100002;
const uint32 kInputMouseRightButtonIndex = 0x100003;
const uint32 kInputKeyControlIndex = 0x101000;
const uint32 kInputKeyCommandIndex = 0x101001; // Command on MacOS, Windows key on Windows.
const uint32 kInputKeyAltIndex = 0x101002;
const uint32 kInputKeyShiftIndex = 0x101003;

enum InputType : uint8 {
  InputTypeUnknown,
  InputTypeSwitch,
  InputTypeTarget,
};

typedef struct InputEvent {
  // The type of event.
  InputType input_type;
  // The index of the button, key, finger, or device that was pressed. This
  // value will be set for every input event as it indicates the source of 
  // the event.
  uint64 switch_index;
  // Extended information about the switch. For example, this will indicate
  // pressure for analog input devices, and whether a keyboard event is shifted.
  uint64 switch_extension;
  // The x coordinate of the event. Only valid if input_type is InputTypeTarget.
  float32 target_x;
  // The y coordinate of the event. Only valid if input_type is InputTypeTarget.
  float32 target_y;
  // Indicates whether the button/key at switch_index is currently held down
  // or is otherwise activated.
  bool is_on;
} InputEvent;

class BaseWindow {
 public:
  BaseWindow(const ::std::string& title, uint32 x, uint32 y, uint32 width,
             uint32 height, uint32 style_flags = 0);
  BaseWindow(const BaseWindow& rhs) = delete;
  virtual ~BaseWindow();

  // Returns true if the window was constructed successfully and is currently
  // active. Returns false if there was an issue with construction, the window
  // was destroyed, or the user tapped a close button.
  bool IsValid() const;
  // Updates the window and deposits all recent input events since the last call
  // to Update, into the supplied input queue. Returns zero on success, non-zero
  // otherwise.
  uint32 Update(::std::vector<InputEvent>* queue = nullptr);
  // Resizes the window to width by height.
  void Resize(uint32 width, uint32 height);
  // Moves the window such that it's upper left coordinate will be at (x,y).
  void Move(uint32 x, uint32 y);
  // Toggles fullscreen vs. windowed presentation style.
  void SetFullscreen(bool fullscreen);
  // Toggles whether the window is visible.
  void SetVisible(bool visible);
  // Toggles whether the cursor should be visible.
  void SetCursorVisible(bool visible);
  // Returns the x pixel coordinate of the upper left corner of the window.
  uint32 GetOriginX() const;
  // Returns the y pixel coordinate of the upper left corner of the window.
  uint32 GetOriginY() const;
  // Returns the width of the window.
  uint32 GetWidth() const;
  // Returns the height of the window.
  uint32 GetHeight() const;
  // Returns the title of the window.
  const ::std::string& GetTitle() const;

 protected:
  // Protected constructor added for derived classes.
  BaseWindow();
  // Creates and initializes the window object and connects it to the
  // appropriate operating system resources. This method will assert if window
  // creation fails.
  void Create(const ::std::string& title, uint32 x, uint32 y, uint32 width,
              uint32 height, uint32 style_flags);
  // Tears down the window and releases any connected operating system
  // resources.
  void Destroy();

  // Set to true if the window was successfully constructed. False otherwise.
  bool is_valid_;
  // The title displayed at the top of the window (when a title bar is present).
  ::std::string title_;
  // The x coordinate of the upper left corner of the window.
  uint32 origin_x_;
  // The y coordinate of the upper left corner of the window.
  uint32 origin_y_;
  // The current width of the window.
  uint32 width_;
  // The current height of the window.
  uint32 height_;
  // The input cache asynchronously retrieves input commands from the OS
  // and preserves them for users.
  ::std::vector<InputEvent> input_cache_;

#if defined(BASE_PLATFORM_WINDOWS)
  HWND window_handle_;
  HINSTANCE instance_;
  friend LRESULT CALLBACK DefWndProc(HWND hWnd, uint32 message, WPARAM wParam,
                                     LPARAM lParam);
#elif defined(BASE_PLATFORM_MACOS)
  class NSBaseWindow* window_handle_;
#endif
};

}  // namespace base

#endif  // __BASE_WINDOW_H__
