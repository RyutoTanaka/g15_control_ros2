#include "spi.hpp"

extern "C" {
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
}

Spi::Spi(std::string_view dev_name, std::uint32_t hz, const Option & option)
: fd_(open(dev_name.data(), O_RDWR)), ioc_transfer_([&]() {
    spi_ioc_transfer ioc_transfer = {};
    ioc_transfer.speed_hz = hz;
    ioc_transfer.bits_per_word = (option.bits_per_word == 0) ? 8 : option.bits_per_word;
    return ioc_transfer;
  }())
{
  if (fd_ < 0) {
    throw Error(Error::Kind::kOpenDevFile, "spi : open device file");
  }

  if (ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &hz) < 0) {
    throw Error(Error::Kind::kSetSpeedHz, "spi : set speed hz");
  }

  if (ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &ioc_transfer_.bits_per_word) < 0) {
    throw Error(Error::Kind::kSetBitsPerWord, "spi : set bits per word");
  }

  if (ioctl(fd_, SPI_IOC_WR_MODE32, &option.mode) < 0) {
    throw Error(Error::Kind::kSetMode, "spi : set mode");
  }
}

Spi::Spi(Spi && spi)
{
  fd_ = spi.fd_;
  ioc_transfer_ = spi.ioc_transfer_;
  spi.fd_ = -1;
}

Spi::~Spi()
{
  if (fd_ >= 0) {
    close(fd_);
  }
}

std::vector<std::uint8_t> Spi::write(
  const std::vector<std::uint8_t> & tx_buffer, std::vector<std::uint8_t> && rx_buffer)
{
  auto length = tx_buffer.size();
  rx_buffer.resize(length);
  ioc_transfer_.tx_buf = reinterpret_cast<std::uint64_t>(tx_buffer.data());
  ioc_transfer_.rx_buf = reinterpret_cast<std::uint64_t>(rx_buffer.data());
  ioc_transfer_.len = length;
  if (ioctl(fd_, SPI_IOC_MESSAGE(1), ioc_transfer_) < 0) {
    throw Error(Error::Kind::kWrite, "spi : write");
  }
  return rx_buffer;
}

Spi::Error::Error(Kind kind, std::string_view what) noexcept
: std::runtime_error(what.data()), kind_(kind)
{
}

constexpr const Spi::Error::Kind & Spi::Error::kind() const noexcept { return kind_; }