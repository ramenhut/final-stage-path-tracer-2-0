
#include <fstream>
#include "bitmap.h"
#include "math/scalar.h"

namespace base {

#ifndef BI_RGB
#define BI_RGB (0)
#endif

#pragma pack(push)
#pragma pack(2)

typedef struct PTCX_BITMAP_FILE_HEADER {
  uint16 type;
  uint32 size;
  uint16 reserved[2];
  uint32 off_bits;

} PTCX_BITMAP_FILE_HEADER;

typedef struct PTCX_BITMAP_INFO_HEADER {
  uint32 size;
  int32 width;
  int32 height;
  uint16 planes;
  uint16 bit_count;
  uint32 compression;
  uint32 size_image;
  int32 x_pels_per_meter;
  int32 y_pels_per_meter;
  uint32 clr_used;
  uint32 clr_important;

} PTCX_BITMAP_INFO_HEADER;

#pragma pack(pop)

bool LoadBitmap(const ::std::string& filename, ::std::vector<float32>* output, uint32 *width, uint32 *height) {
  if (BASE_PARAM_CHECK) {
    if (filename.empty() || !output) {
      return false;
    }
  }

  printf("Loading bitmap file: %s.\n", filename.c_str());

  uint32 bytes_read = 0;
  PTCX_BITMAP_INFO_HEADER bih;
  PTCX_BITMAP_FILE_HEADER bmf_header;

  ::std::ifstream input_file(filename, ::std::ios::in | ::std::ios::binary);

  if (!input_file.read((char*)&bmf_header, sizeof(PTCX_BITMAP_FILE_HEADER))) {
    printf("Failed to read bitmap file header for file %s.\n", filename.c_str());
    return false;
  }

  if (!input_file.read((char*)&bih, sizeof(PTCX_BITMAP_INFO_HEADER))) {
      printf("Failed to read bitmap info header for file %s.\n", filename.c_str());
    return false;
  }

  if (bih.bit_count != 24) {
    printf("Unsupported bitmap file type for file %s.\n", filename.c_str());
    return false;
  }

  output->resize(bih.width * bih.height * 3);
  *width = bih.width;
  *height = bih.height;

  // The BMP format requires each scanline to be 32 bit aligned, so we insert
  // padding if necessary.
  uint32 scanline_padding =
      greater_multiple(bih.width * 3, 4) - (bih.width * 3);

  ::std::vector<uint8> one_texel_row;
  one_texel_row.resize(bih.width * 3);
  uint8* row_ptr = &one_texel_row.at(0);
  uint32 row_stride = bih.width * 3;

  for (uint32 i = 0; i < bih.height; i++) {
    if (!input_file.read((char*)row_ptr, row_stride)) {
        return false;
    }

    uint32 dummy = 0;  // Padding will always be < 4 bytes.
    if (!input_file.read((char*)&dummy, scanline_padding)) {
        return false;
    }

    // Convert our integer texel data into float values, and
    // store the result in our output buffer. Also swap the 
    // R and B channels (as BMP stores its data in BGR).
    for (uint32 j = 0; j < bih.width; j++) {
      output->at(i * row_stride + j * 3 + 0) = row_ptr[j * 3 + 2] / 255.0f;
      output->at(i * row_stride + j * 3 + 1) = row_ptr[j * 3 + 1] / 255.0f;
      output->at(i * row_stride + j * 3 + 2) = row_ptr[j * 3 + 0] / 255.0f;
    }
  }

  return true;
}

}  // namespace base