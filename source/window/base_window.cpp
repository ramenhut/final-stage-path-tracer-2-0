
#include "base_window.h"

namespace base {

BaseWindow::BaseWindow()
    : is_valid_(false), origin_x_(0), origin_y_(0), width_(0), height_(0) {}

BaseWindow::BaseWindow(const ::std::string& title, uint32 x, uint32 y,
                       uint32 width, uint32 height, uint32 style_flags) {
  Create(title, x, y, width, height, style_flags);
}

BaseWindow::~BaseWindow() { Destroy(); }

bool BaseWindow::IsValid() const { return is_valid_; }

uint32 BaseWindow::GetOriginX() const { return origin_x_; }

uint32 BaseWindow::GetOriginY() const { return origin_y_; }

uint32 BaseWindow::GetWidth() const { return width_; }

uint32 BaseWindow::GetHeight() const { return height_; }

const ::std::string& BaseWindow::GetTitle() const { return title_; }

}  // namespace base