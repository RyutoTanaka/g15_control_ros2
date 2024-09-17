#ifndef SPI_HPP_
#define SPI_HPP_

extern "C" {
#include <linux/spi/spidev.h>
}

#include <cstdint>
#include <stdexcept>
#include <string_view>
#include <array>

class Spi
{
public:
  struct Option
  {
    std::uint8_t bits_per_word;
    std::uint32_t mode;
  };

  class Error : public std::runtime_error
  {
  public:
    enum class Kind { kUnknown, kOpenDevFile, kSetSpeedHz, kSetBitsPerWord, kSetMode, kWrite };

    Error(Kind kind, std::string_view what) noexcept;

    constexpr const Kind & kind() const noexcept;

  private:
    Kind kind_;
  };

  Spi(std::string_view dev_name, std::uint32_t hz, const Option & option = {});

  Spi(Spi && spi);

  Spi(const Spi & spi) = delete;

  ~Spi();

  void write(std::uint8_t* tx_buffer,std::uint8_t* rx_buffer,size_t length);

private:
  int fd_;

  spi_ioc_transfer ioc_transfer_;
};

#endif  // SPI_HPP_