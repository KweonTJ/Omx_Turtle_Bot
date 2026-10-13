// Copyright 2026 KweonTJ. SPDX-License-Identifier: Apache-2.0
#pragma once
#include <QBuffer>
#include <QImage>
#include <QImageReader>
#include <stdexcept>
#include <string>
#include <vector>
namespace leader_monitor {
inline QImage rawImage(unsigned width,unsigned height,unsigned step,const std::string & encoding,
                       const std::vector<uint8_t> & data) {
  int channels=0;
  QImage::Format format=QImage::Format_Invalid;
  if (encoding=="rgb8" || encoding=="bgr8") {channels=3; format=QImage::Format_RGB888;}
  else if (encoding=="rgba8" || encoding=="bgra8") {channels=4; format=QImage::Format_RGBA8888;}
  else if (encoding=="mono8") {channels=1; format=QImage::Format_Grayscale8;}
  else throw std::invalid_argument("지원하지 않는 영상 인코딩: "+encoding);
  if (!width || !height || width>4096 || height>2160 || step<width*channels ||
      uint64_t(step)*height>32*1024*1024 || data.size()<uint64_t(step)*height)
    throw std::invalid_argument("영상 크기 / stride / 데이터 길이 오류");
  QImage image(data.data(),static_cast<int>(width),static_cast<int>(height),static_cast<int>(step),format);
  return encoding=="bgr8" || encoding=="bgra8" ? image.rgbSwapped() : image.copy();
}
inline QImage compressedImage(const std::vector<uint8_t> & data) {
  if (data.empty() || data.size()>8*1024*1024) throw std::invalid_argument("압축 영상 크기 오류");
  QByteArray bytes(reinterpret_cast<const char *>(data.data()),static_cast<int>(data.size()));
  QBuffer buffer(&bytes); buffer.open(QIODevice::ReadOnly);
  QImageReader reader(&buffer);
  auto size=reader.size();
  if (!size.isValid() || size.width()>4096 || size.height()>2160)
    throw std::invalid_argument("압축 영상 해상도 오류");
  auto image=reader.read();
  if (image.isNull()) throw std::invalid_argument("압축 영상 디코딩 오류");
  return image;
}
}  // namespace leader_monitor
