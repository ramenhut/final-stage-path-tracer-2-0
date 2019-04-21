
#include "base_window.h"

#if defined(BASE_PLATFORM_WINDOWS)
#define BASE_WINDOW_STYLE_WINDOWED_STYLE (WS_SYSMENU | WS_VISIBLE)
#define BASE_WINDOW_STYLE_FULLSCREEN_STYLE (WS_POPUP | WS_VISIBLE)

#define GET_UNIT_X_VALUE(value, span) \
  (2.0f * (((float32)value + 0.5f) / span) - 1.0f)
#define GET_UNIT_Y_VALUE(value, span) \
  (-2.0f * (((float32)value + 0.5f) / span) + 1.0f)

namespace base {

const uint16 kDefaultInputEventQueueCapacity = 32;

uint32 ConvertScan(uint32 scancode, uint32 shift);

LRESULT CALLBACK DefWndProc(HWND hWnd, uint32 message, WPARAM wParam,
                            LPARAM lParam);

void BaseWindow::Create(const ::std::string& title, uint32 x, uint32 y,
                        uint32 width, uint32 height, uint32 style_flags) {
  if (width == 0 || height == 0 || width > 32768 || height > 16384) {
    return;
  }

  input_cache_.reserve(kDefaultInputEventQueueCapacity);

  instance_ = GetModuleHandle(NULL);

  if (!instance_) {
    return;
  }

  // Our unicode story is still in the works, and we'd still like to build
  // using the unicode character set. Thus for now we convert our ascii window
  // title to unicode right before registering our winclass.

  WCHAR wszTitle[MAX_PATH];
  MultiByteToWideChar(CP_ACP, 0, title.c_str(), -1, wszTitle, MAX_PATH);

  WNDCLASSEX window_class;
  window_class.cbSize = sizeof(WNDCLASSEX);
  window_class.style = CS_HREDRAW | CS_VREDRAW | CS_DBLCLKS | CS_OWNDC;
  window_class.lpfnWndProc = DefWndProc;
  window_class.cbClsExtra = 0;
  window_class.cbWndExtra = 0;
  window_class.hInstance = instance_;
  window_class.hIcon = NULL;
  window_class.hCursor = LoadCursor(NULL, IDC_ARROW);
  window_class.hbrBackground = (HBRUSH)GetStockObject(GRAY_BRUSH);
  window_class.lpszMenuName = NULL;
  window_class.lpszClassName = wszTitle;
  window_class.hIconSm = LoadIcon(NULL, IDI_APPLICATION);

  if (!RegisterClassEx(&window_class)) {
    return;
  }

  bool is_fullscreen = style_flags & BASE_WINDOW_STYLE_FULLSCREEN;
  bool is_hidden = style_flags & BASE_WINDOW_STYLE_WINDOW_HIDDEN;
  bool hide_cursor = style_flags & BASE_WINDOW_STYLE_CURSOR_HIDDEN;

  window_handle_ =
      CreateWindowEx(0L, wszTitle, wszTitle,
                     (is_fullscreen ? BASE_WINDOW_STYLE_FULLSCREEN_STYLE
                                    : BASE_WINDOW_STYLE_WINDOWED_STYLE),
                     (is_fullscreen ? 0 : x), (is_fullscreen ? 0 : y), width,
                     height, NULL, NULL, instance_, NULL);

  if (!window_handle_) {
    UnregisterClass(wszTitle, instance_);
    return;
  }

  SetWindowLongPtr(window_handle_, GWLP_USERDATA, (LONG_PTR)this);
  ShowWindow(window_handle_, is_hidden ? SW_HIDE : SW_SHOW);
  UpdateWindow(window_handle_);
  ShowCursor(!hide_cursor);

  title_ = title;
  origin_x_ = x;
  origin_y_ = y;
  width_ = width;
  height_ = height;
  is_valid_ = true;

  // Windows doesn't always create windows the size that we want. Here we ensure
  // that our window will be fit to the proper size (required for pixel
  // alignment).
  Resize(width, height);
}

uint32 BaseWindow::Update(::std::vector<InputEvent>* queue) {
  if (!is_valid_) {
    return -1;
  }

  MSG msg;
  // Windows provides us with a single message pump for each thread. This
  // queue will receive messages for all windows on the thread. We peek the
  // message to see if it is a quit (for which we behave universally
  // regardless of the instigating window).
  while (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE)) {
    if (WM_QUIT == msg.message) {
      // We do not burden individual message handlers with the responsibility
      // of handling quit messages. Note also that we do not halt the message
      // pump here, as we wish to flush out any remaining messages before
      // handing control back to the higher level modules (which will likely
      // terminate the application).
      is_valid_ = false;
    }

    // If the message is not a quit, then we translate and dispatch it, so
    // that it will arrive at the appropriate window's message handler.
    TranslateMessage(&msg);
    DispatchMessage(&msg);
  }

  if (queue) {
    // Move the input events that are cached in the window to the output queue.
    // To be extra safe we clear the input_cache_ after the move.
    *queue = ::std::move(input_cache_);
    input_cache_.clear();
  }

  return 0;
}

void BaseWindow::Destroy() {
  if (!is_valid_) {
    return;
  }

  WCHAR wszTitle[MAX_PATH] = {0};
  MultiByteToWideChar(CP_ACP, 0, title_.c_str(), -1, wszTitle, MAX_PATH);

  // We call DestroyWindow here but do not check its return type. This is
  // due to the fact that this can legitimately fail in certain circumstances.
  // For example, if the window was closed via a WM_CLOSE / WM_QUIT message,
  // then the dispatch handler for this message will have already destroyed
  // the window, thus rendering our handle invalid.
  DestroyWindow(window_handle_);
  UnregisterClass(wszTitle, instance_);
}

void BaseWindow::SetVisible(bool visible) {
  ShowWindow(window_handle_, visible ? SW_SHOW : SW_HIDE);
}

void BaseWindow::SetCursorVisible(bool visible) { ShowCursor(visible); }

void BaseWindow::SetFullscreen(bool fullscreen) {
  if (!is_valid_) {
    return;
  }

  uint32 old_style = GetWindowLong(window_handle_, GWL_STYLE);

  if (fullscreen) {
    old_style &= ~WS_CAPTION;
    old_style &= ~BASE_WINDOW_STYLE_WINDOWED_STYLE;
    old_style |= BASE_WINDOW_STYLE_FULLSCREEN_STYLE;

    SetWindowLongPtr(window_handle_, GWL_STYLE, old_style);
    SetWindowPos(window_handle_, HWND_TOP, 0, 0, GetSystemMetrics(SM_CXSCREEN),
                 GetSystemMetrics(SM_CYSCREEN),
                 SWP_NOZORDER | SWP_FRAMECHANGED);
  } else {
    old_style &= ~BASE_WINDOW_STYLE_FULLSCREEN_STYLE;
    old_style |= BASE_WINDOW_STYLE_WINDOWED_STYLE;
    old_style |= WS_CAPTION;

    SetWindowLongPtr(window_handle_, GWL_STYLE, old_style);
    SetWindowPos(window_handle_, HWND_TOP, origin_x_, origin_y_, 0, 0,
                 SWP_NOSIZE | SWP_FRAMECHANGED);
    Resize(width_, height_);
  }
}

void BaseWindow::Move(uint32 x, uint32 y) {
  if (!is_valid_) {
    return;
  }

  origin_x_ = x;
  origin_y_ = y;

  SetWindowPos(window_handle_, HWND_TOP, origin_x_, origin_y_, 0, 0,
               SWP_NOSIZE | SWP_FRAMECHANGED);
}

void BaseWindow::Resize(uint32 width, uint32 height) {
  if (!is_valid_ || 0 == width || 0 == height || width > 32768 ||
      height > 16384) {
    return;
  }

  RECT window_rect = {0};

  // We wish to create a window with a client area equal to that of
  // width x height. Unfortunately, CreateWindowEx will consume some of the
  // requested space with the non-client area. Thus, we calculate how much it
  // consumed, and then resize up to guarantee a client area that matches our
  // requested dimensions.

  if (GetClientRect(window_handle_, &window_rect)) {
    uint32 client_width = (window_rect.right - window_rect.left);
    uint32 client_height = (window_rect.bottom - window_rect.top);
    uint32 nc_width = (client_width < width_) ? width_ - client_width : 0;
    uint32 nc_height = (client_height < height_) ? height_ - client_height : 0;

    width_ = width;
    height_ = height;
    SetWindowPos(window_handle_, HWND_TOP, 0, 0, width_ + nc_width,
                 height_ + nc_height, SWP_NOMOVE | SWP_FRAMECHANGED);
  }
}

LRESULT CALLBACK DefWndProc(HWND hWnd, UINT message, WPARAM wParam,
                            LPARAM lParam) {
  InputEvent event;
  uint32 shifted_hold = 0;

  // We pull out the CVWindow object, as well as the input queue. Note that
  // upon error, we overload the return value to be an engine error value.
  BaseWindow* window = (BaseWindow*)GetWindowLongPtr(hWnd, GWLP_USERDATA);

  // The window or queue may not always be valid. For example, during WM_CREATE
  // messages which are issued before the window pointer can be bound. In these
  // cases, we simply avoid using these resources.
  if (!window) {
    return DefWindowProc(hWnd, message, wParam, lParam);
  }

  switch (message) {
    case WM_SYSCOMMAND: {
      // Prevent screensaving or sleeping while this window is active.
      switch (wParam) {
        case SC_SCREENSAVE:
        case SC_MONITORPOWER:
          return 0;
      }

    } break;

    case WM_CLOSE: {
      PostQuitMessage(0);
    } break;

    case WM_MOUSEMOVE: {
      event.input_type = InputTypeTarget;
      event.switch_index = kInputMouseMoveIndex;
      // Convert our mouse coordinates into normalized device coordinates,
      // which range from -1...1 on each axis. Note that due to aspect ratio,
      // one axis may span a greater number of pixels than the other, though
      // their normalized device span is the same (2.0).
      event.target_x = GET_UNIT_X_VALUE(LOWORD(lParam), window->GetWidth());
      event.target_y = GET_UNIT_Y_VALUE(HIWORD(lParam), window->GetHeight());
      window->input_cache_.push_back(event);
    } break;

    case WM_MOUSEWHEEL: {
      // Windows insists on providing a delta value for the mouse wheel. This
      // does not align with our input model, so we instead track the absolute
      // mouse wheel coordinates.
      static float32 abs_wheel_y = 0;
      abs_wheel_y += GET_WHEEL_DELTA_WPARAM(wParam);

      event.input_type = InputTypeTarget;
      event.target_x = 0;
      event.target_y = abs_wheel_y;
      event.switch_index = kInputMouseWheelIndex;

      window->input_cache_.push_back(event);
    } break;

    case WM_LBUTTONDOWN: {
      event.input_type = InputTypeSwitch;
      event.is_on = true;
      event.switch_index = kInputMouseLeftButtonIndex;
      event.target_x = GET_UNIT_X_VALUE(LOWORD(lParam), window->GetWidth());
      event.target_y = GET_UNIT_Y_VALUE(HIWORD(lParam), window->GetHeight());
      window->input_cache_.push_back(event);
    } break;

    case WM_LBUTTONUP: {
      event.input_type = InputTypeSwitch;
      event.is_on = false;
      event.switch_index = kInputMouseLeftButtonIndex;
      event.target_x = GET_UNIT_X_VALUE(LOWORD(lParam), window->GetWidth());
      event.target_y = GET_UNIT_Y_VALUE(HIWORD(lParam), window->GetHeight());
      window->input_cache_.push_back(event);
    } break;

    case WM_RBUTTONDOWN: {
      event.input_type = InputTypeSwitch;
      event.is_on = true;
      event.switch_index = kInputMouseRightButtonIndex;
      event.target_x = GET_UNIT_X_VALUE(LOWORD(lParam), window->GetWidth());
      event.target_y = GET_UNIT_Y_VALUE(HIWORD(lParam), window->GetHeight());
      window->input_cache_.push_back(event);
    } break;

    case WM_RBUTTONUP: {
      event.input_type = InputTypeSwitch;
      event.is_on = false;
      event.switch_index = kInputMouseRightButtonIndex;
      event.target_x = GET_UNIT_X_VALUE(LOWORD(lParam), window->GetWidth());
      event.target_y = GET_UNIT_Y_VALUE(HIWORD(lParam), window->GetHeight());
      window->input_cache_.push_back(event);
    } break;

    case WM_KEYDOWN: {
      // We do not send repeating strokes -- if the key was previously
      // down, then we've already handled this and can ignore it.
      if (lParam & 0x40000000) {
        break;
      }

      event.input_type = InputTypeSwitch;
      event.is_on = true;
      event.switch_index = wParam;
      event.switch_extension = ConvertScan((uint32)wParam, shifted_hold);

      if (16 == wParam) {
        shifted_hold = 1;
      }

      window->input_cache_.push_back(event);
    } break;

    case WM_KEYUP: {
      if (16 == wParam) {
        shifted_hold = 0;
      }

      event.input_type = InputTypeSwitch;
      event.is_on = false;
      event.switch_index = wParam;
      event.switch_extension = ConvertScan((uint32)wParam, shifted_hold);

      window->input_cache_.push_back(event);
    } break;

    case WM_SYSKEYDOWN: {
      if ((lParam & 0xFFFF) > 1) {
        break;
      }

      event.input_type = InputTypeSwitch;
      event.is_on = true;
      event.switch_extension = 0;

      switch (wParam) {
        case VK_CONTROL:
          event.switch_index = kInputKeyControlIndex;
          break;
        case VK_MENU:
          event.switch_index = kInputKeyAltIndex;
          break;
        case VK_SHIFT:
          event.switch_index = kInputKeyShiftIndex;
          break;
      };

      window->input_cache_.push_back(event);
    } break;

    case WM_SYSKEYUP: {
      event.input_type = InputTypeSwitch;
      event.is_on = false;
      event.switch_extension = 0;

      switch (wParam) {
        case VK_CONTROL:
          event.switch_index = kInputKeyControlIndex;
          break;
        case VK_MENU:
          event.switch_index = kInputKeyAltIndex;
          break;
        case VK_SHIFT:
          event.switch_index = kInputKeyShiftIndex;
          break;
      };

      window->input_cache_.push_back(event);
    } break;
  }

  return DefWindowProc(hWnd, message, wParam, lParam);
}

uint32 ConvertScan(uint32 scancode, uint32 shift) {
  if (scancode <= 32) {
    return scancode;
  } else if (scancode >= 65 && scancode < 91) {
    if (shift) return scancode;
    return scancode + 32;
  }

  switch (scancode) {
    case '1':
      return (!shift) ? scancode : '!';
    case '2':
      return (!shift) ? scancode : '@';
    case '3':
      return (!shift) ? scancode : '#';
    case '4':
      return (!shift) ? scancode : '$';
    case '5':
      return (!shift) ? scancode : '%';
    case '6':
      return (!shift) ? scancode : '^';
    case '7':
      return (!shift) ? scancode : '&';
    case '8':
      return (!shift) ? scancode : '*';
    case '9':
      return (!shift) ? scancode : '(';
    case '0':
      return (!shift) ? scancode : ')';
    case '`':
      return (!shift) ? scancode : '~';
    case '.':
      return (!shift) ? scancode : '>';
    case 186:
      return (!shift) ? ';' : ':';
    case 187:
      return (!shift) ? '=' : '+';
    case 188:
      return (!shift) ? ',' : '<';
    case 189:
      return (!shift) ? '-' : '_';
    case 190:
      return (!shift) ? '.' : '>';
    case 191:
      return (!shift) ? '/' : '?';
    case 219:
      return (!shift) ? '[' : '{';
    case 220:
      return (!shift) ? '\\' : '|';
    case 221:
      return (!shift) ? ']' : '}';
    case 222:
      return (!shift) ? '\'' : '\"';
  };

  return scancode;
}

}  // namespace base

#endif